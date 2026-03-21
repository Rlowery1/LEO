// Copyright AAA_Traffic Contributors. All Rights Reserved.

#include "RoadBLDReflectionProvider.h"
#include "TrafficLog.h"
#include "Engine/World.h"
#include "EngineUtils.h"

// split so each intersection segment gets its own junction mapping.
// Requires: CachedIntersectionMasks populated (BuildMaskBasedIntersections).
// ---------------------------------------------------------------------------

void URoadBLDReflectionProvider::DetectAndSplitThroughRoads()
{
	VirtualLaneMap.Empty();
	OriginalToVirtualMap.Empty();
	ReplacedLaneHandles.Empty();
	VirtualLaneToGroupId.Empty();
	IntersectionEntryPointMap.Empty();

	if (CachedIntersectionMasks.Num() == 0)
	{
		UE_LOG(LogAAATraffic, Log,
			TEXT("RoadBLDReflectionProvider: No intersection masks — through-road splitting skipped."));
		return;
	}

	// ── Step 1: Group masks by ParentRoad UObject ───────────────
	// Build: RoadActor → list of mask indices (sorted by StartDistance).
	TMap<UObject*, TArray<int32>> RoadToMaskIndices;
	for (int32 i = 0; i < CachedIntersectionMasks.Num(); ++i)
	{
		UObject* Road = CachedIntersectionMasks[i].ParentRoad.Get();
		if (Road)
		{
			RoadToMaskIndices.FindOrAdd(Road).Add(i);
		}
	}

	// Sort each road's mask list by StartDistance for deterministic ordering.
	for (auto& Pair : RoadToMaskIndices)
	{
		Pair.Value.Sort([this](const int32 A, const int32 B)
		{
			return CachedIntersectionMasks[A].StartDistance < CachedIntersectionMasks[B].StartDistance;
		});
	}

	// ── Step 2: Identify roads that need splitting (1+ masks) ────
	// Any road with at least one intersection mask must be split into
	// virtual segments so that only the mask-covered portion is tagged
	// as a junction lane. Without splitting, BuildJunctionGrouping maps
	// ALL lanes on the road to the junction, preventing vehicle spawning
	// on the free-flow portions.
	// Also need: road handle for each road actor (reverse lookup).
	TMap<UObject*, int32> RoadActorToHandle;
	for (const auto& Pair : RoadHandleMap)
	{
		UObject* Actor = Pair.Value.Get();
		if (Actor)
		{
			RoadActorToHandle.Add(Actor, Pair.Key);
		}
	}

	int32 TotalVirtuals = 0;
	int32 ThroughRoadCount = 0;

	for (const auto& RMPair : RoadToMaskIndices)
	{
		UObject* RoadActor = RMPair.Key;
		const TArray<int32>& MaskIndices = RMPair.Value;

		// Split any road that has at least one intersection mask.
		if (MaskIndices.IsEmpty()) { continue; }

		const int32* RoadHandlePtr = RoadActorToHandle.Find(RoadActor);
		if (!RoadHandlePtr) { continue; }
		const int32 RoadHandle = *RoadHandlePtr;

		const TArray<int32>* LaneHandles = RoadToLaneHandles.Find(RoadHandle);
		if (!LaneHandles || LaneHandles->Num() == 0) { continue; }

		const double RoadLength = GetRoadLength(RoadActor);
		if (RoadLength <= 0.0) { continue; }

		++ThroughRoadCount;

		UE_LOG(LogAAATraffic, Log,
			TEXT("  Splitting road %s (handle=%d, length=%.1f): %d mask(s)"),
			*RoadActor->GetName(), RoadHandle, RoadLength, MaskIndices.Num());

		// ── Step 3: For each lane on this road, compute split indices ──
		for (const int32 LaneHandle : *LaneHandles)
		{
			const FLaneEndpointCache* OrigCache = LaneEndpointMap.Find(LaneHandle);
			if (!OrigCache || OrigCache->Polyline.Num() < 3) { continue; }

			const TArray<FVector>& Poly = OrigCache->Polyline;
			const int32 TotalPoints = Poly.Num();

			// Build split-point list from mask StartDistance/EndDistance.
			// Each mask produces TWO split points: one at its start, one at its end.
			// This creates segments: [road-start..mask1-start], [mask1-start..mask1-end],
			// [mask1-end..mask2-start], [mask2-start..mask2-end], ... [maskN-end..road-end]
			struct FSplitPoint
			{
				int32 PolyIndex;
				int32 GroupId; // >0 = start of intersection segment, 0 = start of free segment
				double MaskDistance; // exact reference-line distance for this boundary (cm)
			};
			TArray<FSplitPoint> SplitPoints;

			// FIX: Detect if this lane's polyline is reversed relative to
			// the reference line. GetLanePath() reverses polylines for
			// opposite-direction lanes so vehicles drive correctly, but
			// mask distances are always in reference-line order. Without
			// mirroring, split indices land at the wrong polyline position
			// — placing the intersection segment on the wrong portion of
			// the road for reversed lanes.
			const bool bIsReversedLane = ReversedLaneSet.Contains(LaneHandle);

			for (const int32 MaskIdx : MaskIndices)
			{
				const FIntersectionMaskInfo& Mask = CachedIntersectionMasks[MaskIdx];

				// Convert reference-line distance to polyline index.
				// Polyline is sampled uniformly: point[i] = distance (RoadLength * i) / (NumSamples - 1)
				// So: index = round(Distance / RoadLength * (TotalPoints - 1))
				const float StartNorm = static_cast<float>(Mask.StartDistance / RoadLength);
				const float EndNorm = static_cast<float>(Mask.EndDistance / RoadLength);

				int32 StartIdx, EndIdx;
				double SplitMaskDistStart, SplitMaskDistEnd;

				if (bIsReversedLane)
				{
					// Reversed lane: Poly[0] = ref-END, Poly[N-1] = ref-START.
					// Mirror the mapping: refDist D → idx = (N-1) - round(D/L*(N-1)).
					// StartDistance (lower ref-line value) maps to a HIGHER poly index,
					// EndDistance (higher ref-line value) maps to a LOWER poly index.
					const int32 MaxIdx = TotalPoints - 1;
					const int32 RawStart = FMath::RoundToInt32(StartNorm * static_cast<float>(MaxIdx));
					const int32 RawEnd = FMath::RoundToInt32(EndNorm * static_cast<float>(MaxIdx));
					StartIdx = MaxIdx - RawEnd;     // EndDist → lower poly index (earlier in travel)
					EndIdx   = MaxIdx - RawStart;   // StartDist → higher poly index (later in travel)

					// Boundary distances: for the reversed vehicle approaching from
					// ref-END, the first intersection boundary it encounters is at
					// Mask.EndDistance on the reference line (StartIdx position).
					// The far boundary (where intersection ends) is at Mask.StartDistance.
					SplitMaskDistStart = Mask.EndDistance;
					SplitMaskDistEnd = Mask.StartDistance;
				}
				else
				{
					// Non-reversed: polyline matches reference-line order.
					StartIdx = FMath::RoundToInt32(StartNorm * static_cast<float>(TotalPoints - 1));
					EndIdx = FMath::RoundToInt32(EndNorm * static_cast<float>(TotalPoints - 1));
					SplitMaskDistStart = Mask.StartDistance;
					SplitMaskDistEnd = Mask.EndDistance;
				}

				// Clamp to valid range (never at very first or last point).
				StartIdx = FMath::Clamp(StartIdx, 1, TotalPoints - 2);
				EndIdx = FMath::Clamp(EndIdx, StartIdx + 1, TotalPoints - 1);

				// Mark start of intersection segment and start of free segment after it.
				SplitPoints.Add({ StartIdx, Mask.GroupId, SplitMaskDistStart });
				if (EndIdx < TotalPoints - 1)
				{
					SplitPoints.Add({ EndIdx, 0, SplitMaskDistEnd }); // Free segment starts after intersection
				}
				// If EndIdx == TotalPoints - 1, the mask extends to the very end
				// of the road. No free segment split is added because there is no
				// road beyond this point. The final virtual segment (from StartIdx
				// to TotalPoints-1) will carry the mask's GroupId, which is the
				// correct behavior — the road ends inside the intersection.

				UE_LOG(LogAAATraffic, Log,
					TEXT("    Lane %d: Mask group=%d StartDist=%.1f EndDist=%.1f → idx=[%d..%d] reversed=%s"),
					LaneHandle, Mask.GroupId, Mask.StartDistance, Mask.EndDistance,
					StartIdx, EndIdx, bIsReversedLane ? TEXT("YES") : TEXT("NO"));
			}

			// Sort split points by poly index, then by GroupId descending
			// so intersection starts (GroupId > 0) are preferred over free
			// starts (GroupId == 0) when two split points collide at the
			// same quantized poly index.
			SplitPoints.Sort([](const FSplitPoint& A, const FSplitPoint& B)
			{
				if (A.PolyIndex != B.PolyIndex) { return A.PolyIndex < B.PolyIndex; }
				// Prefer intersection (GroupId > 0) over free (GroupId == 0).
				return A.GroupId > B.GroupId;
			});

			// Remove duplicates at the same poly index, keeping the first
			// entry (which, after sorting, is the intersection-start if any).
			for (int32 i = SplitPoints.Num() - 1; i > 0; --i)
			{
				if (SplitPoints[i].PolyIndex == SplitPoints[i - 1].PolyIndex)
				{
					SplitPoints.RemoveAt(i);
				}
			}

			if (SplitPoints.Num() == 0) { continue; }

			// ── Step 4: Create virtual segments ──────────────
			// Resolve lane edge curves + refline for exact entry-point computation.
			const FReflectionLaneData* LData = LaneHandleMap.Find(LaneHandle);
			UObject* LaneRoadActor = LData ? LData->RoadActor.Get() : nullptr;
			UObject* LaneRefLine = LaneRoadActor ? GetReferenceLine(LaneRoadActor) : nullptr;
			UObject* LaneLeftEdge = LData ? LData->LeftEdge.Get() : nullptr;
			UObject* LaneRightEdge = LData ? LData->RightEdge.Get() : nullptr;
			const bool bCanComputeEntryPoint = LaneLeftEdge && LaneRightEdge && LaneRefLine
				&& LaneRoadActor && Get3DPosFunc && ConvertDistFunc;

			TArray<int32> VirtualHandles;
			TArray<int32> VirtualGroupIds; // parallel array: group ID per virtual segment
			TArray<double> VirtualBoundaryDistances; // parallel: mask distance at end of each segment (0 if N/A)
			int32 PrevStart = 0;
			int32 PrevGroupId = 0; // First segment before any intersection = free

			for (int32 s = 0; s <= SplitPoints.Num(); ++s)
			{
				const int32 SegEnd = (s < SplitPoints.Num())
					? SplitPoints[s].PolyIndex
					: (TotalPoints - 1);

				if (SegEnd <= PrevStart) { PrevStart = SegEnd; continue; }

				const int32 VirtualId = NextHandleId++;
				FVirtualLaneInfo VInfo;
				VInfo.OriginalLaneHandle = LaneHandle;
				VInfo.StartPointIndex = PrevStart;
				VInfo.EndPointIndex = SegEnd;
				VirtualLaneMap.Add(VirtualId, VInfo);
				VirtualHandles.Add(VirtualId);
				VirtualGroupIds.Add(PrevGroupId);

				// Store the exact mask distance at this segment's end boundary.
				// For segments ending at a split point, this is the split point's MaskDistance.
				// For the final segment, there's no boundary.
				const double BoundaryDist = (s < SplitPoints.Num())
					? SplitPoints[s].MaskDistance : 0.0;
				VirtualBoundaryDistances.Add(BoundaryDist);

				// Tag this virtual lane with its group (0 = free, >0 = intersection).
				VirtualLaneToGroupId.Add(VirtualId, PrevGroupId);

				// Cache endpoint data for the virtual segment.
				FLaneEndpointCache VCache;
				VCache.Width = OrigCache->Width;
				for (int32 p = PrevStart; p <= SegEnd; ++p)
				{
					VCache.Polyline.Add(Poly[p]);
				}
				VCache.StartPos = VCache.Polyline[0];
				VCache.EndPos = VCache.Polyline.Last();
				if (VCache.Polyline.Num() >= 2)
				{
					VCache.StartDir = (VCache.Polyline[1] - VCache.Polyline[0]).GetSafeNormal();
					VCache.EndDir = (VCache.Polyline.Last() - VCache.Polyline[VCache.Polyline.Num() - 2]).GetSafeNormal();
				}
				LaneEndpointMap.Add(VirtualId, MoveTemp(VCache));

				UE_LOG(LogAAATraffic, Log,
					TEXT("      Virtual %d: points [%d..%d], group=%d (%s)"),
					VirtualId, PrevStart, SegEnd, PrevGroupId,
					PrevGroupId > 0 ? TEXT("intersection") : TEXT("free-flow"));

				PrevStart = SegEnd;
				// Next segment's group: if this was the split point, use its GroupId.
				PrevGroupId = (s < SplitPoints.Num()) ? SplitPoints[s].GroupId : 0;
			}

			// ── Step 5: Compute exact intersection entry points ──
			// For each free-flow virtual segment whose next segment is an
			// intersection, derive the precise 3D boundary position from
			// MaskDistance via continuous edge-curve evaluation (no polyline
			// quantization error).
			for (int32 v = 0; v < VirtualHandles.Num() - 1; ++v)
			{
				const int32 ThisGroup = VirtualGroupIds[v];
				const int32 NextGroup = VirtualGroupIds[v + 1];

				if (ThisGroup == 0 && NextGroup > 0 && bCanComputeEntryPoint)
				{
					// This free-flow segment precedes an intersection.
					// The boundary distance is stored in VirtualBoundaryDistances[v].
					const double BoundaryDist = VirtualBoundaryDistances[v];
					if (BoundaryDist > 0.0)
					{
						// Convert reference-line distance to edge-curve distances.
						const double LeftDist = ConvertDistanceBetweenCurves(
							LaneRoadActor, LaneRefLine, LaneLeftEdge, BoundaryDist);
						const double RightDist = ConvertDistanceBetweenCurves(
							LaneRoadActor, LaneRefLine, LaneRightEdge, BoundaryDist);

						// Sample exact 3D positions on both lane edges.
						const FVector LeftPos = Get3DPositionAtDistance(LaneLeftEdge, LaneRefLine, LeftDist);
						const FVector RightPos = Get3DPositionAtDistance(LaneRightEdge, LaneRefLine, RightDist);

						// Lane centerline at the intersection boundary.
						const FVector EntryPoint = (LeftPos + RightPos) * 0.5f;

						// Store on the VirtualLaneInfo.
						FVirtualLaneInfo* VInfo = VirtualLaneMap.Find(VirtualHandles[v]);
						if (VInfo)
						{
							VInfo->IntersectionEntryPoint = EntryPoint;
							VInfo->bHasIntersectionEntryPoint = true;
						}

						// Store in the lookup map for the provider interface.
						IntersectionEntryPointMap.Add(VirtualHandles[v], EntryPoint);

						UE_LOG(LogAAATraffic, Log,
							TEXT("      Virtual %d: intersection entry point computed at MaskDist=%.1f → (%.1f, %.1f, %.1f)"),
							VirtualHandles[v], BoundaryDist, EntryPoint.X, EntryPoint.Y, EntryPoint.Z);
					}
				}
			}

			if (VirtualHandles.Num() > 1)
			{
				ReplacedLaneHandles.Add(LaneHandle);
				OriginalToVirtualMap.Add(LaneHandle, MoveTemp(VirtualHandles));
				TotalVirtuals += OriginalToVirtualMap[LaneHandle].Num();

				// Connect consecutive virtual segments internally.
				const TArray<int32>& Virtuals = OriginalToVirtualMap[LaneHandle];
				for (int32 v = 0; v < Virtuals.Num() - 1; ++v)
				{
					TArray<FTrafficLaneHandle>& Conns = LaneConnectionMap.FindOrAdd(Virtuals[v]);
					Conns.AddUnique(FTrafficLaneHandle(Virtuals[v + 1]));
				}
			}
			else
			{
				// Only one segment — splitting didn't help. Clean up.
				// NOTE: NextHandleId is NOT rolled back. Handle IDs are opaque
				// monotonic identifiers; gaps in the sequence are acceptable and
				// have no functional impact. Rolling back would risk collisions
				// if other code had already observed the incremented value.
				for (const int32 VH : VirtualHandles)
				{
					VirtualLaneMap.Remove(VH);
					LaneEndpointMap.Remove(VH);
					VirtualLaneToGroupId.Remove(VH);
				}
			}
		}
	}

	// ── Remap pre-existing corner-based connections to virtual handles ──
	// BuildLaneConnectivity ran BEFORE splitting and wrote turn connections
	// keyed by original lane handles. Those originals are now in
	// ReplacedLaneHandles — no vehicle will ever drive on them. We must
	// FIX: Use positional proximity instead of blind first/last assignment.
	// Corner connections are at specific physical locations — a corner at
	// the ref-line END should route to the virtual segment nearest that
	// position, not always to the last/first virtual. This matters for
	// reversed lanes where polyline order is opposite to ref-line order,
	// and for roads with junctions at both ends.
	int32 RemappedOutgoing = 0;
	int32 RemappedIncoming = 0;

	for (const int32 OrigHandle : ReplacedLaneHandles)
	{
		const TArray<int32>* Virtuals = OriginalToVirtualMap.Find(OrigHandle);
		if (!Virtuals || Virtuals->Num() == 0) { continue; }

		const int32 FirstVirtual = (*Virtuals)[0];
		const int32 LastVirtual  = (*Virtuals).Last();

		// --- Outgoing: assign each connection to the virtual whose EndPos
		//     is physically closest to the destination lane's StartPos. ---
		if (const TArray<FTrafficLaneHandle>* OldConns = LaneConnectionMap.Find(OrigHandle))
		{
			// Copy because we remove the key below.
			TArray<FTrafficLaneHandle> OldConnsCopy = *OldConns;
			LaneConnectionMap.Remove(OrigHandle);

			for (const FTrafficLaneHandle& Dst : OldConnsCopy)
			{
				// Skip connections to other virtuals of the same original
				// (internal chain is already set up above).
				if (Virtuals->Contains(Dst.HandleId)) { continue; }

				// Find which virtual's EndPos is closest to the destination's StartPos.
				const FLaneEndpointCache* DstCache = LaneEndpointMap.Find(Dst.HandleId);
				int32 BestVirtual = LastVirtual; // fallback
				if (DstCache)
				{
					float BestDistSq = MAX_FLT;
					for (const int32 VH : *Virtuals)
					{
						const FLaneEndpointCache* VCache = LaneEndpointMap.Find(VH);
						if (!VCache) { continue; }
						const float DSq = FVector::DistSquared(VCache->EndPos, DstCache->StartPos);
						if (DSq < BestDistSq)
						{
							BestDistSq = DSq;
							BestVirtual = VH;
						}
					}
				}

				LaneConnectionMap.FindOrAdd(BestVirtual).AddUnique(Dst);
				++RemappedOutgoing;
			}
		}

		// --- Incoming: rewrite any connection that targets the original ---
		// Find which virtual's StartPos is closest to the source lane's EndPos.
		for (auto& ConnPair : LaneConnectionMap)
		{
			TArray<FTrafficLaneHandle>& DstList = ConnPair.Value;
			for (FTrafficLaneHandle& Dst : DstList)
			{
				if (Dst.HandleId == OrigHandle)
				{
					const FLaneEndpointCache* SrcCache = LaneEndpointMap.Find(ConnPair.Key);
					int32 BestVirtual = FirstVirtual; // fallback
					if (SrcCache)
					{
						float BestDistSq = MAX_FLT;
						for (const int32 VH : *Virtuals)
						{
							const FLaneEndpointCache* VCache = LaneEndpointMap.Find(VH);
							if (!VCache) { continue; }
							const float DSq = FVector::DistSquared(VCache->StartPos, SrcCache->EndPos);
							if (DSq < BestDistSq)
							{
								BestDistSq = DSq;
								BestVirtual = VH;
							}
						}
					}
					Dst = FTrafficLaneHandle(BestVirtual);
					++RemappedIncoming;
				}
			}
		}
	}

	// Sort all connection lists for determinism after remapping.
	for (auto& Pair : LaneConnectionMap)
	{
		Pair.Value.Sort([](const FTrafficLaneHandle& A, const FTrafficLaneHandle& B)
		{
			return A.HandleId < B.HandleId;
		});
	}

	UE_LOG(LogAAATraffic, Log,
		TEXT("RoadBLDReflectionProvider: Road splitting — %d roads split, %d virtual segments from %d source lanes. "
			 "Remapped %d outgoing + %d incoming corner connections to virtual handles."),
		ThroughRoadCount, TotalVirtuals, ReplacedLaneHandles.Num(),
		RemappedOutgoing, RemappedIncoming);
}

// ---------------------------------------------------------------------------
// BuildMaskBasedIntersections — read IntersectionMasks from RoadBLD and
// group them into physical intersection clusters for junction assignment
// ---------------------------------------------------------------------------

void URoadBLDReflectionProvider::BuildMaskBasedIntersections(UWorld* World)
{
	CachedIntersectionMasks.Empty();
	IntersectionGroupCentroids.Empty();

	if (!DynNetworkClass || !World)
	{
		return;
	}

	// Find the DynamicRoadNetwork actor deterministically.
	AActor* NetworkActor = nullptr;
	{
		TArray<AActor*> Candidates;
		for (FActorIterator It(World); It; ++It)
		{
			if (It->IsA(DynNetworkClass))
			{
				Candidates.Add(*It);
			}
		}
		if (Candidates.Num() > 0)
		{
			Candidates.Sort([](const AActor& A, const AActor& B) { return A.GetName() < B.GetName(); });
			NetworkActor = Candidates[0];
		}
	}
	if (!NetworkActor)
	{
		UE_LOG(LogAAATraffic, Log,
			TEXT("RoadBLDReflectionProvider: No DynamicRoadNetwork actor — mask-based intersections skipped."));
		return;
	}

	// ── Step 1: Read IntersectionMasks ──────────────────────────
	FArrayProperty* MasksArr = CastField<FArrayProperty>(
		DynNetworkClass->FindPropertyByName(TEXT("IntersectionMasks")));
	if (!MasksArr)
	{
		UE_LOG(LogAAATraffic, Log,
			TEXT("RoadBLDReflectionProvider: IntersectionMasks property not found — mask-based intersections skipped."));
		return;
	}

	FScriptArrayHelper MaskHelper(MasksArr, MasksArr->ContainerPtrToValuePtr<void>(NetworkActor));
	const int32 NumMasks = MaskHelper.Num();
	if (NumMasks == 0)
	{
		UE_LOG(LogAAATraffic, Log,
			TEXT("RoadBLDReflectionProvider: IntersectionMasks is empty — no intersections to group."));
		return;
	}

	FStructProperty* MaskStructProp = CastField<FStructProperty>(MasksArr->Inner);
	if (!MaskStructProp) { return; }

	UScriptStruct* MaskStruct = MaskStructProp->Struct;
	FDoubleProperty* MaskSDProp = CastField<FDoubleProperty>(MaskStruct->FindPropertyByName(TEXT("StartDistance")));
	FDoubleProperty* MaskEDProp = CastField<FDoubleProperty>(MaskStruct->FindPropertyByName(TEXT("EndDistance")));
	FObjectPropertyBase* MaskPRObj = CastField<FObjectPropertyBase>(MaskStruct->FindPropertyByName(TEXT("ParentRoad")));
	FIntProperty* MaskSCIProp = CastField<FIntProperty>(MaskStruct->FindPropertyByName(TEXT("StartCutIndex")));
	FIntProperty* MaskECIProp = CastField<FIntProperty>(MaskStruct->FindPropertyByName(TEXT("EndCutIndex")));

	if (!MaskSDProp || !MaskEDProp || !MaskPRObj || !MaskSCIProp || !MaskECIProp)
	{
		UE_LOG(LogAAATraffic, Warning,
			TEXT("RoadBLDReflectionProvider: IntersectionMask struct missing expected properties (SD=%d ED=%d PR=%d SCI=%d ECI=%d)."),
			!!MaskSDProp, !!MaskEDProp, !!MaskPRObj, !!MaskSCIProp, !!MaskECIProp);
		return;
	}

	// ── Step 2: Read RoadNetworkPerimeterCuts (optional) ────────
	// PerimeterCuts are used for Signal A (corner-index grouping) but are
	// NOT required for mask-based intersection detection to work. If cuts
	// are unavailable, we skip Signal A and group by world-position only.
	FArrayProperty* CutsArr = CastField<FArrayProperty>(
		DynNetworkClass->FindPropertyByName(TEXT("RoadNetworkPerimeterCuts")));

	int32 NumCuts = 0;
	FScriptArrayHelper* CutHelperPtr = nullptr;
	FScriptArrayHelper CutHelperStorage(CutsArr, CutsArr ? CutsArr->ContainerPtrToValuePtr<void>(NetworkActor) : nullptr);
	FIntProperty* CutLCIProp = nullptr;
	FIntProperty* CutRCIProp = nullptr;

	if (CutsArr)
	{
		CutHelperPtr = &CutHelperStorage;
		NumCuts = CutHelperStorage.Num();

		FStructProperty* CutStructProp = CastField<FStructProperty>(CutsArr->Inner);
		if (CutStructProp)
		{
			UScriptStruct* CutStruct = CutStructProp->Struct;
			CutLCIProp = CastField<FIntProperty>(CutStruct->FindPropertyByName(TEXT("LeftCornerIndex")));
			CutRCIProp = CastField<FIntProperty>(CutStruct->FindPropertyByName(TEXT("RightCornerIndex")));
		}
	}

	if (NumCuts == 0)
	{
		UE_LOG(LogAAATraffic, Log,
			TEXT("RoadBLDReflectionProvider: RoadNetworkPerimeterCuts unavailable or empty — Signal A (corner grouping) will be skipped; grouping by world-position only."));
	}

	// ── Step 3: Parse masks, collect corner indices, compute world positions ─
	// We compute the world-space centroid for each mask here (needed for
	// grouping in Step 4). Corner indices are also collected as a supplementary
	// signal but may be all -1 in practice (confirmed from FastPreview rebuilds).
	TArray<FVector> MaskWorldPositions;
	MaskWorldPositions.SetNum(NumMasks);

	for (int32 i = 0; i < NumMasks; ++i)
	{
		const uint8* MaskPtr = MaskHelper.GetRawPtr(i);

		FIntersectionMaskInfo MaskInfo;
		MaskInfo.StartDistance = MaskSDProp->GetPropertyValue_InContainer(MaskPtr);
		MaskInfo.EndDistance   = MaskEDProp->GetPropertyValue_InContainer(MaskPtr);
		MaskInfo.ParentRoad    = MaskPRObj->GetObjectPropertyValue_InContainer(MaskPtr);
		MaskInfo.StartCutIndex = MaskSCIProp->GetPropertyValue_InContainer(MaskPtr);
		MaskInfo.EndCutIndex   = MaskECIProp->GetPropertyValue_InContainer(MaskPtr);

		// Collect all unique corner indices from this mask's cuts.
		// Handle inverted ranges (StartCutIndex > EndCutIndex).
		// Only available when perimeter cuts were successfully read.
		if (NumCuts > 0 && CutLCIProp && CutRCIProp)
		{
			const int32 CutLo = FMath::Min(MaskInfo.StartCutIndex, MaskInfo.EndCutIndex);
			const int32 CutHi = FMath::Max(MaskInfo.StartCutIndex, MaskInfo.EndCutIndex);
			for (int32 CutIdx = CutLo; CutIdx <= CutHi && CutIdx < NumCuts; ++CutIdx)
			{
				if (CutIdx < 0) { continue; }
				const uint8* CutPtr = CutHelperPtr->GetRawPtr(CutIdx);
				const int32 LCI = CutLCIProp->GetPropertyValue_InContainer(CutPtr);
				const int32 RCI = CutRCIProp->GetPropertyValue_InContainer(CutPtr);
				if (LCI >= 0) { MaskInfo.CornerIndices.AddUnique(LCI); }
				if (RCI >= 0) { MaskInfo.CornerIndices.AddUnique(RCI); }
			}
		}

		// Compute world-space position at mask midpoint for grouping.
		FVector WorldPos = FVector::ZeroVector;
		UObject* RoadActor = MaskInfo.ParentRoad.Get();
		if (RoadActor)
		{
			const double MidDist = (MaskInfo.StartDistance + MaskInfo.EndDistance) * 0.5;
			UObject* RefLine = GetReferenceLine(RoadActor);
			if (RefLine && Get3DPosFunc)
			{
				WorldPos = Get3DPositionAtDistance(RefLine, RefLine, MidDist);
			}
		}
		MaskWorldPositions[i] = WorldPos;

		UE_LOG(LogAAATraffic, Log,
			TEXT("  Mask[%d]: Road=%s Dist=[%.1f, %.1f] Cuts=[%d, %d] Corners=%d WorldPos=(%.1f, %.1f, %.1f)"),
			i,
			MaskInfo.ParentRoad.IsValid() ? *MaskInfo.ParentRoad->GetName() : TEXT("NULL"),
			MaskInfo.StartDistance, MaskInfo.EndDistance,
			MaskInfo.StartCutIndex, MaskInfo.EndCutIndex,
			MaskInfo.CornerIndices.Num(),
			WorldPos.X, WorldPos.Y, WorldPos.Z);

		CachedIntersectionMasks.Add(MoveTemp(MaskInfo));
	}

	// ── Step 4: Group masks using two complementary signals ─────
	//
	// Signal A — Corner indices (structural):
	//   Two masks sharing a corner index belong to the same intersection.
	//   Currently all PerimeterCut corner indices return -1 at runtime,
	//   so this signal may produce 0 links. Kept for forward compatibility.
	//
	// Signal B — World-position proximity:
	//   Masks at the same physical intersection have nearby X,Y world
	//   positions. For straight crossroads the midpoints are nearly identical,
	//   but for angled or roundabout junctions, road reference-line offsets
	//   can separate them by up to ~500 cm. Uses 800 cm proximity threshold.
	//
	// Both signals feed into the same union-find.

	// Filter: exclude masks with no valid ParentRoad (phantom/orphan masks).
	TArray<bool> MaskValid;
	MaskValid.SetNumUninitialized(NumMasks);
	int32 ValidCount = 0;
	for (int32 k = 0; k < NumMasks; ++k)
	{
		MaskValid[k] = CachedIntersectionMasks[k].ParentRoad.IsValid();
		if (!MaskValid[k])
		{
			UE_LOG(LogAAATraffic, Warning,
				TEXT("  Mask[%d] has NULL ParentRoad — excluded from grouping"), k);
		}
		else
		{
			++ValidCount;
		}
	}
	UE_LOG(LogAAATraffic, Log, TEXT("  %d/%d masks valid for grouping"), ValidCount, NumMasks);

	TArray<int32> Parent;
	Parent.SetNumUninitialized(NumMasks);
	for (int32 k = 0; k < NumMasks; ++k) { Parent[k] = k; }

	// Iterative find with path compression — avoids unbounded recursion
	// if the parent graph were to contain cycles due to a bug.
	auto Find = [&Parent](int32 X) -> int32
	{
		int32 Root = X;
		while (Parent[Root] != Root) { Root = Parent[Root]; }

		// Path compression pass.
		int32 Current = X;
		while (Parent[Current] != Current)
		{
			const int32 Next = Parent[Current];
			Parent[Current] = Root;
			Current = Next;
		}
		return Root;
	};

	auto Union = [&Parent, &Find](int32 A, int32 B)
	{
		const int32 RA = Find(A);
		const int32 RB = Find(B);
		if (RA != RB) { Parent[RA] = RB; }
	};

	// Signal A: corner-index grouping.
	int32 CornerLinkCount = 0;
	{
		TMap<int32, TArray<int32>> CornerToMasks;
		for (int32 MaskIdx = 0; MaskIdx < NumMasks; ++MaskIdx)
		{
			if (!MaskValid[MaskIdx]) { continue; }
			for (const int32 CornerIdx : CachedIntersectionMasks[MaskIdx].CornerIndices)
			{
				CornerToMasks.FindOrAdd(CornerIdx).Add(MaskIdx);
			}
		}
		for (const auto& Pair : CornerToMasks)
		{
			const TArray<int32>& MaskIndices = Pair.Value;
			for (int32 m = 1; m < MaskIndices.Num(); ++m)
			{
				Union(MaskIndices[0], MaskIndices[m]);
				++CornerLinkCount;
			}
		}
	}

	// Signal B: world-position proximity.
	// Masks whose XY world positions are within a spatial tolerance belong
	// to the same physical intersection. Uses grid-based spatial bucketing
	// to avoid O(N²) pairwise comparisons for large mask counts.
	constexpr float PositionEpsilon = 800.0f; // cm radius
	constexpr float PositionEpsilonSq = PositionEpsilon * PositionEpsilon;
	int32 PositionLinkCount = 0;

	// Grid bucketing: cell size = PositionEpsilon so each mask only needs
	// to check its own cell and 8 neighbours.
	TMap<int64, TArray<int32>> SpatialGrid;
	auto CellKey = [](float X, float Y, float CellSize) -> int64
	{
		const int32 CX = FMath::FloorToInt32(X / CellSize);
		const int32 CY = FMath::FloorToInt32(Y / CellSize);
		return (static_cast<int64>(CX) << 32) | static_cast<int64>(static_cast<uint32>(CY));
	};

	for (int32 i = 0; i < NumMasks; ++i)
	{
		if (!MaskValid[i]) { continue; }
		const int64 Key = CellKey(MaskWorldPositions[i].X, MaskWorldPositions[i].Y, PositionEpsilon);
		SpatialGrid.FindOrAdd(Key).Add(i);
	}

	for (int32 i = 0; i < NumMasks; ++i)
	{
		if (!MaskValid[i]) { continue; }
		const int32 CX = FMath::FloorToInt32(MaskWorldPositions[i].X / PositionEpsilon);
		const int32 CY = FMath::FloorToInt32(MaskWorldPositions[i].Y / PositionEpsilon);

		// Check own cell and 8 neighbours.
		for (int32 DX = -1; DX <= 1; ++DX)
		{
			for (int32 DY = -1; DY <= 1; ++DY)
			{
				const int64 NKey = (static_cast<int64>(CX + DX) << 32) | static_cast<int64>(static_cast<uint32>(CY + DY));
				const TArray<int32>* Cell = SpatialGrid.Find(NKey);
				if (!Cell) { continue; }

				for (const int32 j : *Cell)
				{
					if (j <= i) { continue; } // Only process pairs once (j > i).
					if (!MaskValid[j]) { continue; }
					const float DistX = MaskWorldPositions[i].X - MaskWorldPositions[j].X;
					const float DistY = MaskWorldPositions[i].Y - MaskWorldPositions[j].Y;
					const float XYDistSq = DistX * DistX + DistY * DistY;
					if (XYDistSq <= PositionEpsilonSq)
					{
						Union(i, j);
						++PositionLinkCount;

						UE_LOG(LogAAATraffic, Log,
							TEXT("    Masks [%d] and [%d] grouped by world-position proximity (XY dist=%.1f cm)"),
							i, j, FMath::Sqrt(XYDistSq));
					}
				}
			}
		}
	}

	UE_LOG(LogAAATraffic, Log,
		TEXT("  Grouping signals: %d corner-index links, %d world-position links"),
		CornerLinkCount, PositionLinkCount);

	// ── Step 5: Collect groups ─────────────────────────────────
	TMap<int32, TArray<int32>> Groups;
	for (int32 k = 0; k < NumMasks; ++k)
	{
		if (!MaskValid[k]) { continue; }
		Groups.FindOrAdd(Find(k)).Add(k);
	}

	// Sort groups deterministically.
	// Use the minimum (StartDistance, ParentRoad name) in each group as sort key
	// so junction IDs are stable across rebuilds.
	struct FGroupEntry
	{
		FString SortKey;
		TArray<int32> MaskIndices;
	};
	TArray<FGroupEntry> SortedGroups;

	for (auto& GroupPair : Groups)
	{
		FGroupEntry Entry;
		Entry.MaskIndices = MoveTemp(GroupPair.Value);

		// Build sort key from first mask (by road name + start distance).
		Entry.MaskIndices.Sort();
		const FIntersectionMaskInfo& FirstMask = CachedIntersectionMasks[Entry.MaskIndices[0]];
		const FString RoadName = FirstMask.ParentRoad.IsValid() ? FirstMask.ParentRoad->GetName() : TEXT("_");
		Entry.SortKey = FString::Printf(TEXT("%s_%.1f"), *RoadName, FirstMask.StartDistance);

		SortedGroups.Add(MoveTemp(Entry));
	}

	SortedGroups.Sort([](const FGroupEntry& A, const FGroupEntry& B)
	{
		return A.SortKey < B.SortKey;
	});

	// ── Step 6: Assign group IDs and compute centroids ─────────
	// Centroids are averaged from mask world positions computed in Step 3.
	for (int32 GIdx = 0; GIdx < SortedGroups.Num(); ++GIdx)
	{
		const int32 GroupId = GIdx + 1;

		FVector CentroidSum = FVector::ZeroVector;
		int32 CentroidCount = 0;

		for (const int32 MaskIdx : SortedGroups[GIdx].MaskIndices)
		{
			CachedIntersectionMasks[MaskIdx].GroupId = GroupId;

			const FIntersectionMaskInfo& Mask = CachedIntersectionMasks[MaskIdx];
			const FVector& WorldPos = MaskWorldPositions[MaskIdx];

			CentroidSum += WorldPos;
			++CentroidCount;

			UE_LOG(LogAAATraffic, Log,
				TEXT("    Group %d Mask[%d]: Road=%s Dist=[%.1f, %.1f] WorldPos=(%.1f, %.1f, %.1f)"),
				GroupId, MaskIdx,
				Mask.ParentRoad.IsValid() ? *Mask.ParentRoad->GetName() : TEXT("NULL"),
				Mask.StartDistance, Mask.EndDistance,
				WorldPos.X, WorldPos.Y, WorldPos.Z);
		}

		if (CentroidCount > 0)
		{
			const FVector Centroid = CentroidSum / static_cast<float>(CentroidCount);
			IntersectionGroupCentroids.Add(GroupId, Centroid);

			UE_LOG(LogAAATraffic, Log,
				TEXT("    Group %d centroid: (%.1f, %.1f, %.1f) from %d masks"),
				GroupId, Centroid.X, Centroid.Y, Centroid.Z, CentroidCount);
		}
	}

	UE_LOG(LogAAATraffic, Log,
		TEXT("RoadBLDReflectionProvider: Mask-based intersections — %d masks, %d corner links, %d position links, %d intersection groups."),
		NumMasks, CornerLinkCount, PositionLinkCount, SortedGroups.Num());
}