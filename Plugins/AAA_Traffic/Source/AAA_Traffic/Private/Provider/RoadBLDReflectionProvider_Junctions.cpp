// Copyright AAA_Traffic Contributors. All Rights Reserved.

#include "RoadBLDReflectionProvider.h"
#include "TrafficLog.h"


// ---------------------------------------------------------------------------
// BuildJunctionGrouping — register mask-based junctions, then assign
// proximity connections
// ---------------------------------------------------------------------------

void URoadBLDReflectionProvider::BuildJunctionGrouping()
{
	LaneToJunctionMap.Empty();
	JunctionCentroids.Empty();

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
								}
								// VGroupId == 0 → free-flow segment, no junction.
							}
						}
						continue; // Don't map the original lane.
					}

					LaneToJunctionMap.Add(LaneHandle, JunctionId);
					++LanesMapped;
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
			UObject* RoadA = LaneToRoadActorCache.FindRef(PC.FromLane);
			UObject* RoadB = LaneToRoadActorCache.FindRef(PC.ToLane);

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
			}
			else if (Candidates && Candidates->Num() > 1)
			{
				// Tiebreak: nearest centroid.
				float BestDistSq = TNumericLimits<float>::Max();
				for (const int32 CandId : *Candidates)
				{
					const FVector* C = JunctionCentroids.Find(CandId);
					if (C)
					{
						const float DSq = FVector::DistSquared(PC.Midpoint, *C);
						if (DSq < BestDistSq) { BestDistSq = DSq; GroupId = CandId; }
					}
				}
			}
			else
			{
				// Neither road shares a mask group with the other.
				// This is a simple end-to-end road join (e.g. user extended a
				// road by drawing a new piece), NOT an intersection.
				// Do NOT assign a junction — the lane connectivity (next-lane)
				// already handles the hand-off; adding a junction here would
				// cause vehicles to stop at a non-intersection join point.
				UE_LOG(LogAAATraffic, Log,
					TEXT("  Connection FromLane=%d -> ToLane=%d (Road=%s -> Road=%s): road continuation (no shared mask group) — skipping junction assignment."),
					PC.FromLane, PC.ToLane,
					RoadA ? *RoadA->GetName() : TEXT("NULL"),
					RoadB ? *RoadB->GetName() : TEXT("NULL"));
				++PCSkipped;
				continue;
			}

			if (GroupId > 0)
			{
				++PCMatched;
				UE_LOG(LogAAATraffic, Log,
					TEXT("  Connection FromLane=%d -> ToLane=%d -> matched junction %d"),
					PC.FromLane, PC.ToLane, GroupId);
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

	// Angle-aware tangent scaling: sharp turns need shorter tangents to
	// avoid overshooting into the oncoming lane.  Straight-through keeps
	// the original 0.5 × SpanDist; a 90° turn uses ~0.35; a U-turn ~0.25.
	const float DotFactor = FVector::DotProduct(FromCache->EndDir, ToCache->StartDir);
	// For actual turns (dot < 0.3) raise the floor from 0.25 to 0.35 to
	// pull the curve outward through the mid-arc — wider clearance at apex.
	const float AlphaFloor = (DotFactor < 0.3f) ? 0.35f : 0.25f;
	const float Alpha = FMath::Lerp(AlphaFloor, 0.5f,
		FMath::Clamp((DotFactor + 1.0f) * 0.5f, 0.0f, 1.0f));
	const float TangentScale = SpanDist * Alpha;

	// ── Inner-corner offset for turns ──────────────────────────────
	// Shift the control points outward (away from the inner curb) so the
	// Hermite curve clears the corner.  Cross-product Z of approach×exit
	// gives turn direction: positive = left turn, negative = right turn.
	const float CrossZ = FVector::CrossProduct(FromCache->EndDir, ToCache->StartDir).Z;
	FVector AdjP0 = P0;
	FVector AdjP1 = P1;
	constexpr float InnerCornerOffset = 150.0f; // cm — roughly half a lane width
	if (FMath::Abs(CrossZ) > 0.26f) // Only for actual turns (>15°)
	{
		// Perpendicular in XY: rotate the tangent 90° about Z.
		// For a right turn (CrossZ < 0): shift LEFT (positive perp).
		// For a left turn (CrossZ > 0): shift RIGHT (negative perp).
		const float Sign = (CrossZ < 0.0f) ? 1.0f : -1.0f;
		const FVector PerpFrom = FVector(-FromCache->EndDir.Y, FromCache->EndDir.X, 0.0f) * Sign;
		const FVector PerpTo = FVector(-ToCache->StartDir.Y, ToCache->StartDir.X, 0.0f) * Sign;
		// Scale offset by turn sharpness: 90° turns get full offset, gentle turns less.
		const float SharpnessFactor = FMath::Clamp(FMath::Abs(CrossZ), 0.0f, 1.0f);
		AdjP0 += PerpFrom * (InnerCornerOffset * SharpnessFactor);
		AdjP1 += PerpTo * (InnerCornerOffset * SharpnessFactor);
	}

	const FVector M0 = FromCache->EndDir * TangentScale;
	const FVector M1 = ToCache->StartDir * TangentScale;

	// Segment count scales with span distance for consistent resolution.
	// 200 cm per segment, clamped to [6, 32].
	const int32 NumSegments = FMath::Clamp(FMath::CeilToInt32(SpanDist / 200.0f), 6, 32);
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
		OutPath.Add(H00 * AdjP0 + H10 * M0 + H01 * AdjP1 + H11 * M1);
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
					}
				}
			}
		}

		int32 QueueIdx = 0;
		while (QueueIdx < Queue.Num() && QueueIdx < MaxWalkHops)
		{
			const FBFSEntry Current = Queue[QueueIdx++];

			// Check if this lane is a junction lane.
			const int32* CurJunction = LaneToJunctionMap.Find(Current.LaneId);
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
				break; // found nearest junction — done for this start lane
			}

			// Accumulate this lane's length and enqueue its successors.
			float DistAfterThis = Current.AccumulatedDist;
			const float* CurLen = PrecomputedLaneLengths.Find(Current.LaneId);
			if (CurLen)
			{
				DistAfterThis += *CurLen;
			}

			const TArray<FTrafficLaneHandle>* NextLanes = LaneConnectionMap.Find(Current.LaneId);
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