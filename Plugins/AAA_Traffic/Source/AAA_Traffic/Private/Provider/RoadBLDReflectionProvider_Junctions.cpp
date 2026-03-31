// Copyright AAA_Traffic Contributors. All Rights Reserved.

#include "RoadBLDReflectionProvider.h"
#include "TrafficLog.h"

extern bool GEnableDiagnosticDumps;
extern int32 GTrafficDiagnosticsLevel;
extern int32 GTrafficDiagnosticsSampleLimit;

// Defined in RoadBLDReflectionProvider.cpp
extern bool ShouldLogDiagnostics(int32 Level);
extern int32 GetDiagnosticsSampleLimit();


// ---------------------------------------------------------------------------
// BuildJunctionGrouping — register mask-based junctions, then assign
// proximity connections
// ---------------------------------------------------------------------------

void URoadBLDReflectionProvider::BuildJunctionGrouping()
{
	LaneToJunctionMap.Empty();
	JunctionCentroids.Empty();
	TArray<FString> GroupingSamples;
	const bool bSampleDiagnostics = ShouldLogDiagnostics(3);
	const int32 MaxSamples = GetDiagnosticsSampleLimit();

	// ── Mask-based junctions (primary path) ─────────────────────
	// When IntersectionGroupCentroids is populated, mask groups define the
	// authoritative set of junctions. Junctions are registered FIRST from
	// mask data, then ProximityConnections are assigned to them.
	if (IntersectionGroupCentroids.Num() > 0)
	{
		// Step 1: Register all mask group centroids as junctions.
		for (const auto& Pair : IntersectionGroupCentroids)
		{
			JunctionCentroids.Add(Pair.Key, Pair.Value);
		}

		UE_LOG(LogAAATraffic, Log,
			TEXT("  Registered %d mask-based junctions with centroids."),
			JunctionCentroids.Num());

		// Step 2: Build GroupId → set of road handles from mask data.
		// Also build reverse map: road handle → list of GroupIds (for
		// through-roads that participate in multiple intersections).
		TMap<int32, TSet<int32>> GroupToRoadHandles;
		TMap<int32, TArray<int32>> RoadHandleToGroups;
		TMap<int32, TSet<UObject*>> GroupToRoads; // For proximity connection matching.

		// Pre-compute reverse map: RoadActor* → road handle.
		// Avoids O(N) scan of RoadHandleMap for every mask.
		TMap<UObject*, int32> RoadActorToHandleCache;
		for (const auto& RHPair : RoadHandleMap)
		{
			if (UObject* R = RHPair.Value.Get())
			{
				RoadActorToHandleCache.Add(R, RHPair.Key);
			}
		}

		for (const FIntersectionMaskInfo& Mask : CachedIntersectionMasks)
		{
			UObject* RoadActor = Mask.ParentRoad.Get();
			if (!RoadActor || Mask.GroupId <= 0) { continue; }

			GroupToRoads.FindOrAdd(Mask.GroupId).Add(RoadActor);

			// Resolve road handle for this road actor.
			const int32* CachedHandle = RoadActorToHandleCache.Find(RoadActor);
			if (CachedHandle)
			{
				GroupToRoadHandles.FindOrAdd(Mask.GroupId).Add(*CachedHandle);
				RoadHandleToGroups.FindOrAdd(*CachedHandle).AddUnique(Mask.GroupId);
			}
		}

		if (ShouldLogDiagnostics(2))
		{
			TArray<int32> SortedGroupIds;
			GroupToRoadHandles.GetKeys(SortedGroupIds);
			SortedGroupIds.Sort();
			for (const int32 GroupId : SortedGroupIds)
			{
				TArray<int32> RoadHandles = GroupToRoadHandles.FindRef(GroupId).Array();
				RoadHandles.Sort();
				UE_LOG(LogAAATraffic, Log,
					TEXT("  JunctionGroup %d: Roads=[%s] Centroid=(%.0f,%.0f,%.0f)"),
					GroupId,
					*FString::JoinBy(RoadHandles, TEXT(","), [](const int32 Handle) { return FString::FromInt(Handle); }),
					JunctionCentroids.FindRef(GroupId).X,
					JunctionCentroids.FindRef(GroupId).Y,
					JunctionCentroids.FindRef(GroupId).Z);
			}
		}

		// Step 3: Map lanes to junctions using road participation.
		// For through-roads that were split by DetectAndSplitThroughRoads,
		// each virtual segment already has its group ID in VirtualLaneToGroupId.
		// For non-split lanes, map directly by road participation.
		int32 LanesMapped = 0;
		for (const auto& GroupPair : GroupToRoadHandles)
		{
			const int32 JunctionId = GroupPair.Key;

			for (const int32 RoadHandle : GroupPair.Value)
			{
				const TArray<int32>* Lanes = RoadToLaneHandles.Find(RoadHandle);
				if (!Lanes) { continue; }

				for (const int32 LaneHandle : *Lanes)
				{
					// Check if this lane was split into virtual segments.
					if (ReplacedLaneHandles.Contains(LaneHandle))
					{
						// Through-road lane was split. Map each virtual segment
						// using its pre-computed group ID from splitting.
						const TArray<int32>* Virtuals = OriginalToVirtualMap.Find(LaneHandle);
						if (Virtuals)
						{
							for (const int32 VH : *Virtuals)
							{
								const int32* VGroupId = VirtualLaneToGroupId.Find(VH);
								if (VGroupId && *VGroupId > 0)
								{
									LaneToJunctionMap.FindOrAdd(VH) = *VGroupId;
									++LanesMapped;
									if (bSampleDiagnostics && GroupingSamples.Num() < MaxSamples)
									{
										GroupingSamples.Add(FString::Printf(
											TEXT("MapVirtual Lane=%d Orig=%d Road=%d Group=%d Reason=split-mask-segment"),
											VH, LaneHandle, RoadHandle, *VGroupId));
									}
								}
								// VGroupId == 0 → free-flow segment, no junction.
							}
						}
						continue; // Don't map the original lane.
					}

					LaneToJunctionMap.Add(LaneHandle, JunctionId);
					++LanesMapped;
					if (bSampleDiagnostics && GroupingSamples.Num() < MaxSamples)
					{
						GroupingSamples.Add(FString::Printf(
							TEXT("MapDirect Lane=%d Road=%d Group=%d Reason=road-participates-in-mask-group"),
							LaneHandle, RoadHandle, JunctionId));
					}
				}
			}
		}

		UE_LOG(LogAAATraffic, Log,
			TEXT("  Mapped %d lanes to mask-based junctions."), LanesMapped);

		// Step 4: Process ProximityConnections.
		// These represent end-to-end road joins found by endpoint matching.
		// Assign each to an existing mask junction or create a new one.

		// Pre-compute lane→road UObject map to avoid repeated lookups
		// inside the proximity connection loop.
		TMap<int32, UObject*> LaneToRoadActorCache;
		for (const FProximityConnection& PC : ProximityConnectionList)
		{
			for (const int32 LH : { PC.FromLane, PC.ToLane })
			{
				if (LaneToRoadActorCache.Contains(LH)) { continue; }
				int32 EffectiveLane = LH;
				if (const FVirtualLaneInfo* V = VirtualLaneMap.Find(LH))
				{
					EffectiveLane = V->OriginalLaneHandle;
				}
				const int32* RoadHandlePtr = LaneToRoadHandleMap.Find(EffectiveLane);
				UObject* RoadObj = RoadHandlePtr ? RoadHandleMap.FindRef(*RoadHandlePtr).Get() : nullptr;
				LaneToRoadActorCache.Add(LH, RoadObj);
			}
		}

		// Build reverse lookup: (RoadA, RoadB) pair → list of GroupIds.
		// Reduces proximity connection assignment from O(P×G) to O(P).
		// Key: sorted pair of road actor pointers encoded as int64.
		auto MakeRoadPairKey = [](UObject* A, UObject* B) -> int64
		{
			const uintptr_t PA = reinterpret_cast<uintptr_t>(A);
			const uintptr_t PB = reinterpret_cast<uintptr_t>(B);
			const uintptr_t Lo = FMath::Min(PA, PB);
			const uintptr_t Hi = FMath::Max(PA, PB);
			// Combine into a single 64-bit key via hash.
			return static_cast<int64>(HashCombine(GetTypeHash(Lo), GetTypeHash(Hi)));
		};

		TMap<int64, TArray<int32>> RoadPairToGroups;
		for (const auto& GPair : GroupToRoads)
		{
			TArray<UObject*> Roads = GPair.Value.Array();
			for (int32 RIdx = 0; RIdx < Roads.Num(); ++RIdx)
			{
				for (int32 RJdx = RIdx + 1; RJdx < Roads.Num(); ++RJdx)
				{
					const int64 PKey = MakeRoadPairKey(Roads[RIdx], Roads[RJdx]);
					RoadPairToGroups.FindOrAdd(PKey).AddUnique(GPair.Key);
				}
			}
		}

		int32 PCMatched = 0;
		int32 PCSkipped = 0;

		for (const FProximityConnection& PC : ProximityConnectionList)
		{
			auto ResolveRoadHandleForLane = [this](const int32 LaneHandle) -> int32
			{
				int32 EffectiveLane = LaneHandle;
				if (const FVirtualLaneInfo* V = VirtualLaneMap.Find(LaneHandle))
				{
					EffectiveLane = V->OriginalLaneHandle;
				}
				return LaneToRoadHandleMap.FindRef(EffectiveLane);
			};

			UObject* RoadA = LaneToRoadActorCache.FindRef(PC.FromLane);
			UObject* RoadB = LaneToRoadActorCache.FindRef(PC.ToLane);
			const int32 RoadHandleA = ResolveRoadHandleForLane(PC.FromLane);
			const int32 RoadHandleB = ResolveRoadHandleForLane(PC.ToLane);

			// Try to find an intersection group that contains both roads.
			int32 GroupId = 0;
			const TArray<int32>* Candidates = nullptr;
			if (RoadA && RoadB)
			{
				const int64 PKey = MakeRoadPairKey(RoadA, RoadB);
				Candidates = RoadPairToGroups.Find(PKey);
			}

			if (Candidates && Candidates->Num() == 1)
			{
				GroupId = (*Candidates)[0];
				if (bSampleDiagnostics && GroupingSamples.Num() < MaxSamples)
				{
					GroupingSamples.Add(FString::Printf(
						TEXT("MatchProximity From=%d To=%d RoadA=%d RoadB=%d Candidates=[%d] Chosen=%d"),
						PC.FromLane, PC.ToLane, RoadHandleA, RoadHandleB, GroupId, GroupId));
				}
			}
			else if (Candidates && Candidates->Num() > 1)
			{
				// Tiebreak: nearest centroid.
				float BestDistSq = TNumericLimits<float>::Max();
				TArray<FString> TieBreakDistances;
				for (const int32 CandId : *Candidates)
				{
					const FVector* C = JunctionCentroids.Find(CandId);
					if (C)
					{
						const float DSq = FVector::DistSquared(PC.Midpoint, *C);
						if (ShouldLogDiagnostics(2))
						{
							TieBreakDistances.Add(FString::Printf(TEXT("%d:%.1f"), CandId, FMath::Sqrt(DSq)));
						}
						if (DSq < BestDistSq) { BestDistSq = DSq; GroupId = CandId; }
					}
				}
				if (ShouldLogDiagnostics(2))
				{
					UE_LOG(LogAAATraffic, Log,
						TEXT("  Proximity tie-break FromLane=%d ToLane=%d Candidates=[%s] Mid=(%.0f,%.0f,%.0f) Chosen=%d"),
						PC.FromLane,
						PC.ToLane,
						*FString::Join(TieBreakDistances, TEXT(", ")),
						PC.Midpoint.X,
						PC.Midpoint.Y,
						PC.Midpoint.Z,
						GroupId);
				}
			}
			else
			{
				// Neither road shares a mask group with the other.
				// Treat as a simple road continuation — the lane connectivity
				// (next-lane) already handles the hand-off.  Adding a junction
				// here would cause vehicles to stop at a non-intersection
				// join point.
				UE_LOG(LogAAATraffic, Log,
					TEXT("  Connection FromLane=%d -> ToLane=%d (Road=%s -> Road=%s): road continuation (no shared mask group) — skipping junction assignment."),
					PC.FromLane, PC.ToLane,
					RoadA ? *RoadA->GetName() : TEXT("NULL"),
					RoadB ? *RoadB->GetName() : TEXT("NULL"));
				TArray<int32> GroupsA = RoadHandleToGroups.FindRef(RoadHandleA);
				TArray<int32> GroupsB = RoadHandleToGroups.FindRef(RoadHandleB);
				GroupsA.Sort();
				GroupsB.Sort();
				UE_LOG(LogAAATraffic, Log,
					TEXT("    ContinuationReason FromLane=%d ToLane=%d RoadHandleA=%d GroupsA=[%s] RoadHandleB=%d GroupsB=[%s] Mid=(%.0f,%.0f,%.0f)"),
					PC.FromLane,
					PC.ToLane,
					RoadHandleA,
					*FString::JoinBy(GroupsA, TEXT(","), [](const int32 Value) { return FString::FromInt(Value); }),
					RoadHandleB,
					*FString::JoinBy(GroupsB, TEXT(","), [](const int32 Value) { return FString::FromInt(Value); }),
					PC.Midpoint.X,
					PC.Midpoint.Y,
					PC.Midpoint.Z);
				if (bSampleDiagnostics && GroupingSamples.Num() < MaxSamples)
				{
					GroupingSamples.Add(FString::Printf(
						TEXT("SkipContinuation From=%d To=%d RoadA=%d RoadB=%d Reason=no-shared-mask-group"),
						PC.FromLane, PC.ToLane, RoadHandleA, RoadHandleB));
				}
				++PCSkipped;
				continue;
			}

			if (GroupId > 0)
			{
				++PCMatched;
				UE_LOG(LogAAATraffic, Log,
					TEXT("  Connection FromLane=%d -> ToLane=%d -> matched junction %d"),
					PC.FromLane, PC.ToLane, GroupId);
				if (ShouldLogDiagnostics(2))
				{
					UE_LOG(LogAAATraffic, Log,
						TEXT("    MatchReason FromLane=%d ToLane=%d RoadHandleA=%d RoadHandleB=%d Mid=(%.0f,%.0f,%.0f)"),
						PC.FromLane,
						PC.ToLane,
						RoadHandleA,
						RoadHandleB,
						PC.Midpoint.X,
						PC.Midpoint.Y,
						PC.Midpoint.Z);
				}
			}

			// DO NOT tag FromLane/ToLane as junction here.
			// Mask-based splitting (Step 3) already set the correct IDs:
			// intersection (mask) segments have GroupId > 0, free-flow
			// (pre/post-mask) segments are deliberately JunctionId=0.
			// Tagging proximity-connection participants would pollute
			// pre-mask approach lanes and post-mask departure lanes with
			// junction status, causing vehicles to:
			//   - be filtered from spawn pool (too few spawn candidates)
			//   - re-detect the junction after exiting (stuck mid-intersection)
			//   - get assigned to wrong signal groups
			// The connectivity (which lane leads where) is already stored
			// in LaneConnectionMap by BuildProximityConnections.
		}

		UE_LOG(LogAAATraffic, Log,
			TEXT("RoadBLDReflectionProvider: Junction grouping — %d mask junctions, %d proximity connections (%d matched to junctions, %d skipped as road continuations), %d total junctions, %d lanes mapped."),
			IntersectionGroupCentroids.Num(), ProximityConnectionList.Num(),
			PCMatched, PCSkipped, JunctionCentroids.Num(), LaneToJunctionMap.Num());

		// Build reverse map: junction ID → list of lane handles.
		JunctionToLanesMap.Empty();
		for (const auto& Pair : LaneToJunctionMap)
		{
			JunctionToLanesMap.FindOrAdd(Pair.Value).AddUnique(Pair.Key);
		}
		// Sort each list for deterministic iteration.
		for (auto& Pair : JunctionToLanesMap)
		{
			Pair.Value.Sort();
		}
		UE_LOG(LogAAATraffic, Log,
			TEXT("RoadBLDReflectionProvider: Built JunctionToLanesMap — %d junctions."),
			JunctionToLanesMap.Num());

		if (bSampleDiagnostics)
		{
			for (const FString& Line : GroupingSamples)
			{
				UE_LOG(LogAAATraffic, Log, TEXT("RoadBLDReflectionProvider: JunctionGroupingSample %s"), *Line);
			}
		}

		return;
	}

	// ── No mask data + no connections ──────────────────────────
	if (ProximityConnectionList.Num() == 0)
	{
		UE_LOG(LogAAATraffic, Log,
			TEXT("RoadBLDReflectionProvider: No mask data and no proximity connections — junction grouping skipped."));
		return;
	}

	// ── No mask data: all proximity connections are road continuations ──
	// RoadBLD is the authoritative source for intersection data.
	// Zero masks means RoadBLD determined there are no intersections in
	// this road network (e.g. an overpass, or isolated road segments).
	// Proximity connections still provide lane-to-lane handoffs, but no
	// junctions should be created — creating them would spawn false
	// junction markers at simple road joins.
	UE_LOG(LogAAATraffic, Log,
		TEXT("RoadBLDReflectionProvider: No mask data — %d proximity connections treated as road continuations, 0 junctions created."),
		ProximityConnectionList.Num());
}
int32 URoadBLDReflectionProvider::GetJunctionForLane(const FTrafficLaneHandle& Lane)
{
	// Check the handle directly first (works for both virtual and original).
	if (const int32* JId = LaneToJunctionMap.Find(Lane.HandleId))
	{
		UE_LOG(LogAAATraffic, Verbose,
			TEXT("JNCT GetJunctionForLane: Lane=%d → JunctionId=%d (direct map hit)"),
			Lane.HandleId, *JId);
		return *JId;
	}
	// Virtual lane — also check via original handle.
	if (const FVirtualLaneInfo* VInfo = VirtualLaneMap.Find(Lane.HandleId))
	{
		if (const int32* JId = LaneToJunctionMap.Find(VInfo->OriginalLaneHandle))
		{
			UE_LOG(LogAAATraffic, Verbose,
				TEXT("JNCT GetJunctionForLane: Lane=%d (virtual, orig=%d) → JunctionId=%d (fallback hit)"),
				Lane.HandleId, VInfo->OriginalLaneHandle, *JId);
			return *JId;
		}
	}
	// No junction for this lane — this fires very frequently for free-flow lanes,
	// so we use Verbose severity (not Log/Warning).
	UE_LOG(LogAAATraffic, Verbose,
		TEXT("JNCT GetJunctionForLane: Lane=%d → 0 (no junction found)"),
		Lane.HandleId);
	return 0;
}

bool URoadBLDReflectionProvider::GetJunctionCentroid(int32 JunctionId, FVector& OutCentroid)
{
	const FVector* C = JunctionCentroids.Find(JunctionId);
	if (C)
	{
		OutCentroid = *C;
		return true;
	}
	return false;
}

TArray<FTrafficLaneHandle> URoadBLDReflectionProvider::GetLanesForJunction(int32 JunctionId)
{
	TArray<FTrafficLaneHandle> Result;
	if (const TArray<int32>* LaneIds = JunctionToLanesMap.Find(JunctionId))
	{
		Result.Reserve(LaneIds->Num());
		for (const int32 Id : *LaneIds)
		{
			Result.Emplace(FTrafficLaneHandle(Id));
		}
	}
	return Result;
}

bool URoadBLDReflectionProvider::GetJunctionPath(
	const FTrafficLaneHandle& FromLane,
	const FTrafficLaneHandle& ToLane,
	TArray<FVector>& OutPath)
{
	// Generate a cubic Hermite curve from the end of FromLane to the start of
	// ToLane using cached endpoint tangents.  This replaces the old BUG-4 fix
	// that returned false (forcing the controller to synthesize its own curve).
	// Generating it here ensures DoJunctionPathsConflict can also use curved
	// paths for accurate conflict detection.
	const FLaneEndpointCache* FromCache = LaneEndpointMap.Find(FromLane.HandleId);
	const FLaneEndpointCache* ToCache = LaneEndpointMap.Find(ToLane.HandleId);
	if (!FromCache || !ToCache)
	{
		return false;
	}

	const FVector P0 = FromCache->EndPos;
	const FVector P1 = ToCache->StartPos;
	const float SpanDist = FVector::Dist(P0, P1);

	if (SpanDist < 50.0f)
	{
		// Negligible gap — no junction curve needed.
		return false;
	}

	// ── Smoothed tangent directions for curved-road stability ──────
	// On curved roads (loops, arcs), the instantaneous tangent at the
	// junction boundary diverges from the overall approach direction,
	// causing the Hermite curve to overshoot along the road curvature
	// and produce a tight apex that triggers the infeasibility check.
	// Sample over a wider polyline window (up to 10m) for a more
	// representative approach arc.
	FVector ApproachDir = FromCache->EndDir;
	if (FromCache->Polyline.Num() >= 4)
	{
		const int32 Window = FMath::Min(10, FromCache->Polyline.Num() / 2);
		const int32 Idx = FromCache->Polyline.Num() - 1 - Window;
		const FVector Dir = (FromCache->Polyline.Last() - FromCache->Polyline[Idx]).GetSafeNormal();
		if (!Dir.IsNearlyZero()) { ApproachDir = Dir; }
	}
	FVector ExitDir = ToCache->StartDir;
	if (ToCache->Polyline.Num() >= 4)
	{
		const int32 Window = FMath::Min(10, ToCache->Polyline.Num() / 2);
		const FVector Dir = (ToCache->Polyline[Window] - ToCache->Polyline[0]).GetSafeNormal();
		if (!Dir.IsNearlyZero()) { ExitDir = Dir; }
	}

	// ── Tangent scaling: circular-arc Hermite approximation ────────
	// For a Hermite spline approximating a circular arc of turn angle θ,
	// the optimal tangent/chord ratio is α = 2·tan(θ/4) / (3·sin(θ/2)).
	// This produces smooth arcs without the overshoot and fishhook shapes
	// that proportional scaling (TangentScale ∝ SpanDist) causes on wide
	// junctions.  No lateral bend is needed — properly-scaled tangents
	// produce arcs that naturally clear the inner corner.
	const float DotFactor = FVector::DotProduct(ApproachDir, ExitDir);
	const float CrossZ = FVector::CrossProduct(ApproachDir, ExitDir).Z;
	const float TurnAngleRad = FMath::Acos(FMath::Clamp(DotFactor, -1.0f, 1.0f));
	const bool bLogPathDiagnostics = ShouldLogDiagnostics(2);

	float Alpha;
	if (TurnAngleRad < 0.05f) // Nearly straight-through
	{
		Alpha = 0.5f;
	}
	else
	{
		const float HalfAngle = TurnAngleRad * 0.5f;
		const float QuarterAngle = TurnAngleRad * 0.25f;
		Alpha = (2.0f * FMath::Tan(QuarterAngle)) / (3.0f * FMath::Sin(HalfAngle));
		Alpha = FMath::Clamp(Alpha, 0.25f, 0.55f);
	}
	const float TangentScale = SpanDist * Alpha;

	// Segment count scales with span distance for consistent resolution.
	// 200 cm per segment, clamped to [6, 48].
	// Sharp turns (>90°) get higher resolution so the min-radius measurement
	// doesn't miss the tightest apex between coarse sample points.
	const int32 BaseSegments = FMath::CeilToInt32(SpanDist / 200.0f);
	const int32 SharpBonus = (TurnAngleRad > PI * 0.5f) ? 8 : 0;
	const int32 NumSegments = FMath::Clamp(BaseSegments + SharpBonus, 6, 48);

	// ── Generate Hermite curve ─────────────────────────────────────
	// Pure forward tangents only — no lateral bend.  The circular-arc
	// ratio produces clean arcs that stay within the junction surface.
	const FVector M0 = ApproachDir * TangentScale;
	const FVector M1 = ExitDir * TangentScale;

	OutPath.Reset();
	OutPath.Reserve(NumSegments + 1);
	for (int32 i = 0; i <= NumSegments; ++i)
	{
		const float T = static_cast<float>(i) / static_cast<float>(NumSegments);
		const float T2 = T * T;
		const float T3 = T2 * T;
		const float H00 = 2.0f * T3 - 3.0f * T2 + 1.0f;
		const float H10 = T3 - 2.0f * T2 + T;
		const float H01 = -2.0f * T3 + 3.0f * T2;
		const float H11 = T3 - T2;
		OutPath.Add(H00 * P0 + H10 * M0 + H01 * P1 + H11 * M1);
	}

	// ── Turn radius diagnostic ─────────────────────────────────────
	// Log if the curve's minimum radius is below the fleet minimum.
	// The arc shape is determined by the junction geometry; if it's too
	// tight there's no parameter that can fix it without leaving the road.
	constexpr float AbsoluteMinTurnRadiusCm = 270.0f;
	const float EffectiveMinTurnRadius = FMath::Max(FleetMinTurnRadiusCm, AbsoluteMinTurnRadiusCm);
	if (TurnAngleRad > 0.26f && OutPath.Num() >= 3)
	{
		float CurveMinR = TNumericLimits<float>::Max();
		for (int32 Idx = 1; Idx + 1 < OutPath.Num(); ++Idx)
		{
			const FVector SA = OutPath[Idx] - OutPath[Idx - 1];
			const FVector SB = OutPath[Idx + 1] - OutPath[Idx];
			const float CMag = FMath::Abs(FVector::CrossProduct(SA, SB).Z);
			if (CMag <= KINDA_SMALL_NUMBER) { continue; }
			const float Chord = FVector::Dist(OutPath[Idx - 1], OutPath[Idx + 1]);
			const float Num = SA.Size() * SB.Size() * Chord;
			if (Num <= KINDA_SMALL_NUMBER) { continue; }
			const float R = Num / (2.0f * CMag);
			if (R > 0.0f) { CurveMinR = FMath::Min(CurveMinR, R); }
		}
		if (CurveMinR == TNumericLimits<float>::Max()) { CurveMinR = 0.0f; }

		if (CurveMinR > 0.0f && CurveMinR < EffectiveMinTurnRadius)
		{
			UE_LOG(LogAAATraffic, Log,
				TEXT("JNCT: Turn radius below fleet min: From=%d To=%d MinR=%.0f Effective=%.0f Fleet=%.0f — geometry-limited"),
				FromLane.HandleId, ToLane.HandleId, CurveMinR, EffectiveMinTurnRadius, FleetMinTurnRadiusCm);
		}
	}

	if (bLogPathDiagnostics && OutPath.Num() >= 2)
	{
		float ArcLength = 0.0f;
		for (int32 PointIdx = 0; PointIdx < OutPath.Num() - 1; ++PointIdx)
		{
			ArcLength += FVector::Dist(OutPath[PointIdx], OutPath[PointIdx + 1]);
		}

		const FVector ExitDir2D = FVector(ToCache->StartDir.X, ToCache->StartDir.Y, 0.0f).GetSafeNormal();
		const FVector ExitPerp2D = FVector(-ExitDir2D.Y, ExitDir2D.X, 0.0f);
		const FVector PreEndDelta = OutPath[OutPath.Num() - 2] - P1;
		const FVector PreEndDelta2D = FVector(PreEndDelta.X, PreEndDelta.Y, 0.0f);
		const float PreEndLongitudinal = FVector::DotProduct(PreEndDelta2D, ExitDir2D);
		const float PreEndLateral = FVector::DotProduct(PreEndDelta2D, ExitPerp2D);
		const FVector FinalSeg2D = FVector(
			OutPath.Last().X - OutPath[OutPath.Num() - 2].X,
			OutPath.Last().Y - OutPath[OutPath.Num() - 2].Y,
			0.0f).GetSafeNormal();
		const float ExitAlignDot = FVector::DotProduct(FinalSeg2D, ExitDir2D);
		const float ExitAlignCross = FVector::CrossProduct(FinalSeg2D, ExitDir2D).Z;

		UE_LOG(LogAAATraffic, Log,
			TEXT("JNCT PROVIDER-PATH: From=%d To=%d Span=%.1f Arc=%.1f Pts=%d Dot=%.3f Cross=%.3f Alpha=%.3f Tangent=%.1f PreEndLat=%.1f PreEndLong=%.1f ExitAlignDot=%.3f ExitAlignCross=%.3f"),
			FromLane.HandleId,
			ToLane.HandleId,
			SpanDist,
			ArcLength,
			OutPath.Num(),
			DotFactor,
			CrossZ,
			Alpha,
			TangentScale,
			PreEndLateral,
			PreEndLongitudinal,
			ExitAlignDot,
			ExitAlignCross);
	}
	return true;
}

// ---------------------------------------------------------------------------
// ITrafficRoadProvider — junction path conflict detection
// ---------------------------------------------------------------------------

static bool Segments2DIntersect(const FVector& A, const FVector& B,
	const FVector& C, const FVector& D)
{
	// Parametric intersection of AB and CD in XY plane.
	const float Ax = B.X - A.X, Ay = B.Y - A.Y;
	const float Bx = D.X - C.X, By = D.Y - C.Y;
	const float Denom = Ax * By - Ay * Bx;

	if (FMath::Abs(Denom) < KINDA_SMALL_NUMBER)
	{
		return false; // Parallel or degenerate.
	}

	const float Cx = C.X - A.X, Cy = C.Y - A.Y;
	const float T = (Cx * By - Cy * Bx) / Denom;
	const float U = (Cx * Ay - Cy * Ax) / Denom;

	// Strict interior intersection (excluding shared endpoints).
	constexpr float Eps = 0.01f;
	return (T > Eps && T < (1.0f - Eps)) && (U > Eps && U < (1.0f - Eps));
}

bool URoadBLDReflectionProvider::DoJunctionPathsConflict(
	const FTrafficLaneHandle& FromA, const FTrafficLaneHandle& ToA,
	const FTrafficLaneHandle& FromB, const FTrafficLaneHandle& ToB)
{
	// Same approach or same exit → conflict (vehicles would merge).
	if (FromA == FromB || ToA == ToB) { return true; }

	// Get actual curved junction paths (Hermite curves from GetJunctionPath).
	TArray<FVector> PathA, PathB;
	const bool bHasPathA = GetJunctionPath(FromA, ToA, PathA);
	const bool bHasPathB = GetJunctionPath(FromB, ToB, PathB);

	// Fallback to straight-line if curves unavailable.
	if (!bHasPathA || PathA.Num() < 2)
	{
		PathA.Reset();
		const FLaneEndpointCache* FromACache = LaneEndpointMap.Find(FromA.HandleId);
		const FLaneEndpointCache* ToACache = LaneEndpointMap.Find(ToA.HandleId);
		if (!FromACache || !ToACache) { return true; }
		PathA.Add(FromACache->EndPos);
		PathA.Add(ToACache->StartPos);
	}
	if (!bHasPathB || PathB.Num() < 2)
	{
		PathB.Reset();
		const FLaneEndpointCache* FromBCache = LaneEndpointMap.Find(FromB.HandleId);
		const FLaneEndpointCache* ToBCache = LaneEndpointMap.Find(ToB.HandleId);
		if (!FromBCache || !ToBCache) { return true; }
		PathB.Add(FromBCache->EndPos);
		PathB.Add(ToBCache->StartPos);
	}

	// Test all pairs of segments between PathA and PathB for 2D intersection.
	for (int32 i = 0; i < PathA.Num() - 1; ++i)
	{
		for (int32 j = 0; j < PathB.Num() - 1; ++j)
		{
			if (Segments2DIntersect(PathA[i], PathA[i + 1], PathB[j], PathB[j + 1]))
			{
				return true;
			}
		}
	}
	return false;
}
// ---------------------------------------------------------------------------
// PrecomputeJunctionMap — ONE-TIME pass over the entire lane graph at world
// startup. For every lane in the network, computes:
//   1. Total polyline length (stored in PrecomputedLaneLengths)
//   2. Distance to the nearest downstream junction via graph walk
//      (stored in PrecomputedJunctionMap)
//
// After this runs, GetLaneLength and GetDistanceToNextJunction are O(1)
// lookups. Every vehicle has instant access to the ENTIRE road/junction
// layout — no per-tick scanning needed.
// ---------------------------------------------------------------------------
void URoadBLDReflectionProvider::PrecomputeJunctionMap()
{
	PrecomputedLaneLengths.Empty();
	PrecomputedJunctionMap.Empty();

	// ── Phase 1: Compute and cache every lane's polyline length ──
	for (const auto& Pair : LaneEndpointMap)
	{
		const int32 LaneId = Pair.Key;
		const FLaneEndpointCache& Cache = Pair.Value;

		float Length = 0.0f;
		for (int32 i = 0; i < Cache.Polyline.Num() - 1; ++i)
		{
			Length += FVector::Dist(Cache.Polyline[i], Cache.Polyline[i + 1]);
		}
		PrecomputedLaneLengths.Add(LaneId, Length);
	}

	// ── Phase 2: For every lane, walk forward to find the next junction ──
	// We iterate ALL lanes (including virtual segments) and for each one,
	// walk the LaneConnectionMap graph until we hit a junction lane or
	// exhaust the search (no junction downstream).
	//
	// The result is keyed by each lane's handle ID and stores the cumulative
	// distance from that lane's END to the junction's START.

	constexpr int32 MaxWalkHops = 50; // generous — real networks are unlikely to exceed this

	// Collect all lane IDs to iterate deterministically.
	TArray<int32> AllLaneIds;
	LaneEndpointMap.GetKeys(AllLaneIds);
	AllLaneIds.Sort(); // deterministic order

	int32 JunctionLaneCount = 0;
	int32 FreeFlowWithJunctionAhead = 0;
	int32 DeadEndLanes = 0;

	for (const int32 StartLaneId : AllLaneIds)
	{
		const bool bTraceThisLane = ShouldLogDiagnostics(2);
		TArray<FString> WalkTrace;
		auto AppendTrace = [&WalkTrace](const FString& Line)
		{
			if (WalkTrace.Num() < 12)
			{
				WalkTrace.Add(Line);
			}
		};

		// If this lane IS a junction lane, distance = 0.
		const int32* StartJunction = LaneToJunctionMap.Find(StartLaneId);
		if (StartJunction && *StartJunction != 0)
		{
			FJunctionScanResult Result;
			Result.JunctionId = *StartJunction;
			Result.DistanceCm = 0.0f;
			Result.JunctionLane = FTrafficLaneHandle(StartLaneId);
			Result.ApproachLane = FTrafficLaneHandle(StartLaneId);
			PrecomputedJunctionMap.Add(StartLaneId, Result);
			++JunctionLaneCount;
			if (bTraceThisLane)
			{
				UE_LOG(LogAAATraffic, Log,
					TEXT("PrecomputeTrace StartLane=%d RESULT=JUNCTION-SELF JunctionId=%d"),
					StartLaneId,
					*StartJunction);
			}
			continue;
		}

		// BFS walk forward from this lane through ALL successor branches
		// to find the nearest downstream junction. Previous code followed
		// only the first unvisited path at each fork, missing junctions
		// reachable via other branches.
		struct FBFSEntry
		{
			int32 LaneId;
			int32 PreviousLaneId;
			float AccumulatedDist;
		};

		TArray<FBFSEntry> Queue;
		TSet<int32> Visited;
		Visited.Add(StartLaneId);
		bool bFound = false;

		// Seed the queue with all direct successors of the start lane.
		{
			const TArray<FTrafficLaneHandle>* NextLanes = LaneConnectionMap.Find(StartLaneId);
			if (NextLanes)
			{
				for (const FTrafficLaneHandle& Next : *NextLanes)
				{
					if (!Visited.Contains(Next.HandleId))
					{
						Visited.Add(Next.HandleId);
						FBFSEntry Entry;
						Entry.LaneId = Next.HandleId;
						Entry.PreviousLaneId = StartLaneId;
						Entry.AccumulatedDist = 0.0f;
						Queue.Add(Entry);
						if (bTraceThisLane)
						{
							AppendTrace(FString::Printf(TEXT("seed next=%d prev=%d dist=0"), Next.HandleId, StartLaneId));
						}
					}
				}
			}
		}

		int32 QueueIdx = 0;
		while (QueueIdx < Queue.Num() && QueueIdx < MaxWalkHops)
		{
			const FBFSEntry Current = Queue[QueueIdx++];
			const float CurrentLaneLen = PrecomputedLaneLengths.FindRef(Current.LaneId);
			const TArray<FTrafficLaneHandle>* NextLanes = LaneConnectionMap.Find(Current.LaneId);
			const int32* CurJunction = LaneToJunctionMap.Find(Current.LaneId);
			if (bTraceThisLane)
			{
				AppendTrace(FString::Printf(
					TEXT("visit lane=%d prev=%d dist=%.0f len=%.0f j=%d next=%d"),
					Current.LaneId,
					Current.PreviousLaneId,
					Current.AccumulatedDist,
					CurrentLaneLen,
					CurJunction ? *CurJunction : 0,
					NextLanes ? NextLanes->Num() : 0));
			}

			// Check if this lane is a junction lane.
			if (CurJunction && *CurJunction != 0)
			{
				FJunctionScanResult Result;
				Result.JunctionId = *CurJunction;
				Result.DistanceCm = Current.AccumulatedDist;
				Result.JunctionLane = FTrafficLaneHandle(Current.LaneId);
				Result.ApproachLane = FTrafficLaneHandle(Current.PreviousLaneId);
				PrecomputedJunctionMap.Add(StartLaneId, Result);
				bFound = true;
				++FreeFlowWithJunctionAhead;
				if (bTraceThisLane)
				{
					UE_LOG(LogAAATraffic, Log,
						TEXT("PrecomputeTrace StartLane=%d RESULT=FOUND JunctionId=%d Dist=%.0f Approach=%d JunctionLane=%d Trace=%s"),
						StartLaneId,
						Result.JunctionId,
						Result.DistanceCm,
						Result.ApproachLane.HandleId,
						Result.JunctionLane.HandleId,
						*FString::Join(WalkTrace, TEXT(" | ")));
				}
				break; // found nearest junction — done for this start lane
			}

			// Accumulate this lane's length and enqueue its successors.
			float DistAfterThis = Current.AccumulatedDist;
			if (CurrentLaneLen > 0.0f)
			{
				DistAfterThis += CurrentLaneLen;
			}

			if (NextLanes)
			{
				for (const FTrafficLaneHandle& Next : *NextLanes)
				{
					if (!Visited.Contains(Next.HandleId))
					{
						Visited.Add(Next.HandleId);
						FBFSEntry Entry;
						Entry.LaneId = Next.HandleId;
						Entry.PreviousLaneId = Current.LaneId;
						Entry.AccumulatedDist = DistAfterThis;
						Queue.Add(Entry);
						if (bTraceThisLane)
						{
							AppendTrace(FString::Printf(
								TEXT("enqueue next=%d via=%d dist=%.0f"),
								Next.HandleId,
								Current.LaneId,
								DistAfterThis));
						}
					}
				}
			}
		}

		if (!bFound)
		{
			++DeadEndLanes;
			// No junction downstream — store an invalid result so lookup
			// still returns instantly (no junction = JunctionId 0).
			FJunctionScanResult NoJunction;
			PrecomputedJunctionMap.Add(StartLaneId, NoJunction);
			UE_LOG(LogAAATraffic, Log,
				TEXT("PrecomputeTrace StartLane=%d RESULT=NO-JUNCTION Visited=%d QueueProcessed=%d Trace=%s"),
				StartLaneId,
				Visited.Num(),
				QueueIdx,
				*FString::Join(WalkTrace, TEXT(" | ")));
		}
	}

	UE_LOG(LogAAATraffic, Log,
		TEXT("PrecomputeJunctionMap: %d total lanes mapped. "
			 "%d junction lanes, %d free-flow with junction ahead, %d dead-end/no-junction."),
		AllLaneIds.Num(), JunctionLaneCount, FreeFlowWithJunctionAhead, DeadEndLanes);

	// ── DETAILED PER-LANE DUMP ──
	// Log every lane's junction map result so we can see exactly which lanes
	// "see" junctions and which are blind. This is the DEFINITIVE record
	// of the precomputed map. If a lane shows "NO-JUNCTION" here, vehicles
	// on that lane will NEVER detect an intersection via the approach scan.
	UE_LOG(LogAAATraffic, Log, TEXT("PrecomputeJunctionMap — PER-LANE DUMP START (%d lanes):"), AllLaneIds.Num());
	for (const int32 LaneId : AllLaneIds)
	{
		const FJunctionScanResult* R = PrecomputedJunctionMap.Find(LaneId);
		const float* LLen = PrecomputedLaneLengths.Find(LaneId);
		const float Len = LLen ? *LLen : 0.0f;
		const int32* JID = LaneToJunctionMap.Find(LaneId);
		const bool bIsJunctionLane = JID && *JID != 0;
		const TArray<FTrafficLaneHandle>* Connections = LaneConnectionMap.Find(LaneId);
		const int32 NumConnections = Connections ? Connections->Num() : 0;

		if (R && R->IsValid())
		{
			UE_LOG(LogAAATraffic, Verbose,
				TEXT("  Lane=%d Length=%.0f IsJunction=%s Connections=%d => JunctionId=%d Dist=%.0f JnctLane=%d ApproachLane=%d"),
				LaneId, Len,
				bIsJunctionLane ? TEXT("YES") : TEXT("no"),
				NumConnections,
				R->JunctionId, R->DistanceCm, R->JunctionLane.HandleId, R->ApproachLane.HandleId);
		}
		else
		{
			UE_LOG(LogAAATraffic, Verbose,
				TEXT("  Lane=%d Length=%.0f IsJunction=%s Connections=%d => NO-JUNCTION-DOWNSTREAM (vehicles here are blind)"),
				LaneId, Len,
				bIsJunctionLane ? TEXT("YES") : TEXT("no"),
				NumConnections);
		}
	}
	UE_LOG(LogAAATraffic, Verbose, TEXT("PrecomputeJunctionMap — PER-LANE DUMP END"));
}