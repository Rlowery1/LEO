// Copyright AAA_Traffic Contributors. All Rights Reserved.

#include "RoadBLDReflectionProvider.h"
#include "RoadBLDAPIContract.h"
#include "TrafficLog.h"
#include "Engine/World.h"
#include "EngineUtils.h"

// CVars defined in RoadBLDReflectionProvider.cpp
extern bool GEnableDiagnosticDumps;

// ---------------------------------------------------------------------------
// CacheRoadData — discover roads and lanes via reflection
// ---------------------------------------------------------------------------

void URoadBLDReflectionProvider::CacheRoadData(UWorld* World)
{
	if (bCached) { return; }

	// ── Resolve class pointers ───────────────────────────────────
	DynRoadClass = FindObject<UClass>(nullptr, TEXT("/Script/RoadBLDRuntime.DynamicRoad"));
	DynNetworkClass = FindObject<UClass>(nullptr, TEXT("/Script/RoadBLDRuntime.DynamicRoadNetwork"));

	if (!DynRoadClass)
	{
		UE_LOG(LogAAATraffic, Warning,
			TEXT("RoadBLDReflectionProvider: DynamicRoad class not found."));
		return;
	}

	// ── Resolve function/property pointers ───────────────────────
	GetLengthFunc   = DynRoadClass->FindFunctionByName(TEXT("GetLength"));
	GetAllLanesFunc = DynRoadClass->FindFunctionByName(TEXT("GetAllLanes"));
	ConvertDistFunc = DynRoadClass->FindFunctionByName(TEXT("ConvertDistanceBetweenCurves"));
	GetWorldPosFunc = DynRoadClass->FindFunctionByName(TEXT("GetWorldPositionAtDistance"));
	RefLineProp     = DynRoadClass->FindPropertyByName(TEXT("ReferenceLine"));

	// Phase 1A: road-class property/function resolution
	RoadTypeProp          = DynRoadClass->FindPropertyByName(TEXT("RoadType"));
	StartSnappedRoadProp  = DynRoadClass->FindPropertyByName(TEXT("StartSnappedRoad"));
	EndSnappedRoadProp    = DynRoadClass->FindPropertyByName(TEXT("EndSnappedRoad"));
	LeftLanesProp         = DynRoadClass->FindPropertyByName(TEXT("LeftLanes"));
	RightLanesProp        = DynRoadClass->FindPropertyByName(TEXT("RightLanes"));
	GetPointTurnRadiusFunc = DynRoadClass->FindFunctionByName(TEXT("GetPointTurnRadius"));

	if (!GetLengthFunc || !GetAllLanesFunc)
	{
		UE_LOG(LogAAATraffic, Warning,
			TEXT("RoadBLDReflectionProvider: Missing GetLength or GetAllLanes on DynamicRoad."));
		return;
	}

	// ── Iterate all DynamicRoad actors ───────────────────────────
	TArray<AActor*> RoadActors;
	for (FActorIterator It(World); It; ++It)
	{
		if (It->IsA(DynRoadClass))
		{
			RoadActors.Add(*It);
		}
	}
	// Deterministic ordering (System.md §4.4).
	RoadActors.Sort([](const AActor& A, const AActor& B) { return A.GetName() < B.GetName(); });

	UE_LOG(LogAAATraffic, Log,
		TEXT("RoadBLDReflectionProvider: Found %d DynamicRoad actors in world."),
		RoadActors.Num());

	for (AActor* RoadActor : RoadActors)
	{
		const double RoadLength = GetRoadLength(RoadActor);
		if (RoadLength <= 0.0)
		{
			UE_LOG(LogAAATraffic, Log,
				TEXT("RoadBLDReflectionProvider: Skipping road '%s' — Length=%.1f <= 0."),
				*RoadActor->GetName(), RoadLength);
			continue;
		}

		const int32 RoadId = NextHandleId++;
		RoadHandleMap.Add(RoadId, RoadActor);

		// ── Phase 1A: Read road type ────────────────────────────
		if (RoadTypeProp)
		{
			int64 RoadTypeVal = 0;
			if (FEnumProperty* EP = CastField<FEnumProperty>(RoadTypeProp))
			{
				FNumericProperty* Under = EP->GetUnderlyingProperty();
				RoadTypeVal = Under->GetSignedIntPropertyValue(
					RoadTypeProp->ContainerPtrToValuePtr<void>(RoadActor));
			}
			else if (FByteProperty* BP = CastField<FByteProperty>(RoadTypeProp))
			{
				RoadTypeVal = BP->GetPropertyValue(
					RoadTypeProp->ContainerPtrToValuePtr<void>(RoadActor));
			}
			RoadTypeMap.Add(RoadId, static_cast<uint8>(RoadTypeVal));
		}

		// ── Phase 1A: Collect left-lane UObjects for side detection ─
		TSet<UObject*> LeftLaneSet;
		if (LeftLanesProp)
		{
			FArrayProperty* ArrProp = CastField<FArrayProperty>(LeftLanesProp);
			if (ArrProp)
			{
				FScriptArrayHelper Arr(ArrProp,
					LeftLanesProp->ContainerPtrToValuePtr<void>(RoadActor));
				FObjectPropertyBase* InnerObj = CastField<FObjectPropertyBase>(ArrProp->Inner);
				if (InnerObj)
				{
					for (int32 i = 0; i < Arr.Num(); ++i)
					{
						UObject* LaneObj = InnerObj->GetObjectPropertyValue(Arr.GetRawPtr(i));
						if (LaneObj) { LeftLaneSet.Add(LaneObj); }
					}
				}
			}
		}

		TArray<UObject*> Lanes = GetAllLanesForRoad(RoadActor);
		// Filter null entries before sorting — GetAllLanes() can return nulls.
		Lanes.RemoveAll([](const UObject* Obj) { return Obj == nullptr; });
		// Sort lanes by name for deterministic handle ordering.
		Lanes.Sort([](const UObject& A, const UObject& B)
		{
			return A.GetName() < B.GetName();
		});

		// ── Filter lanes by ELaneType — only keep drivable types ─
		{
			// Resolve the ELaneType UEnum once for name-based comparison.
			UEnum* LaneTypeEnum = FindObject<UEnum>(nullptr, TEXT("/Script/RoadBLDRuntime.ELaneType"));

			const int32 PreFilterCount = Lanes.Num();
			Lanes.RemoveAll([LaneTypeEnum](const UObject* LaneObj) -> bool
			{
				if (!LaneObj) { return true; }

				// Read LaneSections array from the lane object.
				FArrayProperty* SectionsProp = CastField<FArrayProperty>(
					LaneObj->GetClass()->FindPropertyByName(TEXT("LaneSections")));
				if (!SectionsProp) { return false; } // Can't determine — keep it.

				FScriptArrayHelper Sections(SectionsProp,
					SectionsProp->ContainerPtrToValuePtr<void>(LaneObj));
				if (Sections.Num() == 0) { return false; } // No sections — keep.

				// Read LaneType enum from the first section.
				UScriptStruct* SectionStruct = CastField<FStructProperty>(SectionsProp->Inner)
					? CastField<FStructProperty>(SectionsProp->Inner)->Struct : nullptr;
				if (!SectionStruct) { return false; }

				FProperty* TypeProp = SectionStruct->FindPropertyByName(TEXT("LaneType"));
				if (!TypeProp) { return false; }

				// Read the raw enum value.
				const uint8* ElemPtr = Sections.GetRawPtr(0);
				int64 EnumVal = 0;
				if (FEnumProperty* EP = CastField<FEnumProperty>(TypeProp))
				{
					FNumericProperty* Under = EP->GetUnderlyingProperty();
					EnumVal = Under->GetSignedIntPropertyValue(
						TypeProp->ContainerPtrToValuePtr<void>(ElemPtr));
				}
				else if (FByteProperty* BP = CastField<FByteProperty>(TypeProp))
				{
					EnumVal = BP->GetPropertyValue(
						TypeProp->ContainerPtrToValuePtr<void>(ElemPtr));
				}

				// Resolve the enum value name for robust comparison.
				FString TypeName;
				if (LaneTypeEnum)
				{
					TypeName = LaneTypeEnum->GetNameStringByValue(EnumVal);
				}

				UE_LOG(LogAAATraffic, Verbose,
					TEXT("RoadBLDReflectionProvider: Lane '%s' LaneType=%s (%lld)"),
					*LaneObj->GetName(), *TypeName, EnumVal);

				// Keep only types a vehicle can drive on.
				const bool bDrivable =
					TypeName == TEXT("Normal") ||
					TypeName == TEXT("CenterTurn") ||
					TypeName == TEXT("Restricted");
				return !bDrivable;
			});

			UE_LOG(LogAAATraffic, Log,
				TEXT("RoadBLDReflectionProvider: Road '%s' — %d total lanes, %d drivable after ELaneType filter."),
				*RoadActor->GetName(), PreFilterCount, Lanes.Num());
		}

		// ── Resolve lane-class properties once (first valid lane) ─
		if (Lanes.Num() > 0 && !LeftEdgeProp && Lanes[0])
		{
			UClass* LaneClass = Lanes[0]->GetClass();

			// Validate lane contract.
			FRoadBLDAPIContractResult LaneResult = FRoadBLDAPIContract::ValidateLaneClass(LaneClass);
			if (!LaneResult.bAllPassed)
			{
				UE_LOG(LogAAATraffic, Warning,
					TEXT("RoadBLDReflectionProvider: Lane class contract issues:"));
				for (const FRoadBLDContractFailure& F : LaneResult.Failures)
				{
					UE_LOG(LogAAATraffic, Warning, TEXT("  %s"), *F.ToString());
				}
			}

			LeftEdgeProp  = LaneClass->FindPropertyByName(TEXT("LeftEdgeCurve"));
			RightEdgeProp = LaneClass->FindPropertyByName(TEXT("RightEdgeCurve"));
			LaneWidthProp = LaneClass->FindPropertyByName(TEXT("LaneWidth"));

			// Phase 1A: Resolve lane-class array properties
			LaneSectionsProp  = LaneClass->FindPropertyByName(TEXT("LaneSections"));
			ActiveSegmentsProp = LaneClass->FindPropertyByName(TEXT("ActiveSegments"));

			// Resolve Get3DPositionAtDistance from the first non-null edge curve.
			if (LeftEdgeProp)
			{
				FObjectPropertyBase* ObjProp = CastField<FObjectPropertyBase>(LeftEdgeProp);
				if (ObjProp)
				{
					for (UObject* L : Lanes)
					{
						if (!L) { continue; }
						UObject* Edge = ObjProp->GetObjectPropertyValue_InContainer(L);
						if (Edge)
						{
							// Validate curve contract.
							FRoadBLDAPIContractResult CurveResult = FRoadBLDAPIContract::ValidateCurveClass(Edge->GetClass());
							if (!CurveResult.bAllPassed)
							{
								UE_LOG(LogAAATraffic, Warning,
									TEXT("RoadBLDReflectionProvider: Curve class contract issues:"));
								for (const FRoadBLDContractFailure& F : CurveResult.Failures)
								{
									UE_LOG(LogAAATraffic, Warning, TEXT("  %s"), *F.ToString());
								}
							}

							Get3DPosFunc = Edge->GetClass()->FindFunctionByName(TEXT("Get3DPositionAtDistance"));
							break;
						}
					}
				}
			}

			// ── DIAG: Log class-level function resolution result ─────
			UE_LOG(LogAAATraffic, Log,
				TEXT("RoadBLDReflectionProvider: Function resolution — "
					 "Get3DPosFunc=%s  ConvertDistFunc=%s  LeftEdgeProp=%s  RightEdgeProp=%s"),
				Get3DPosFunc   ? TEXT("OK") : TEXT("NULL"),
				ConvertDistFunc ? TEXT("OK") : TEXT("NULL"),
				LeftEdgeProp   ? TEXT("OK") : TEXT("NULL"),
				RightEdgeProp  ? TEXT("OK") : TEXT("NULL"));
		}

		// ── BUG-5 FIX: Retry Get3DPosFunc on subsequent roads ────
		// The property resolution block above is one-shot (guarded by !LeftEdgeProp).
		// If the first road's lanes all had null edge curves, Get3DPosFunc stays null.
		// This block retries on every road until the function is found.
		if (!Get3DPosFunc && LeftEdgeProp && Lanes.Num() > 0)
		{
			FObjectPropertyBase* ObjProp = CastField<FObjectPropertyBase>(LeftEdgeProp);
			if (ObjProp)
			{
				for (UObject* L : Lanes)
				{
					if (!L) { continue; }
					UObject* Edge = ObjProp->GetObjectPropertyValue_InContainer(L);
					if (Edge)
					{
						Get3DPosFunc = Edge->GetClass()->FindFunctionByName(TEXT("Get3DPositionAtDistance"));
						if (Get3DPosFunc)
						{
							UE_LOG(LogAAATraffic, Log,
								TEXT("RoadBLDReflectionProvider: Get3DPosFunc resolved on road '%s' (deferred resolution)."),
								*RoadActor->GetName());
							break;
						}
					}
				}
			}
		}

		// ── Create lane handles ──────────────────────────────────
		TArray<int32>& RoadLaneIds = RoadToLaneHandles.Add(RoadId);

		for (UObject* LaneObj : Lanes)
		{
			if (!LaneObj) { continue; }

			const int32 LaneId = NextHandleId++;
			RoadLaneIds.Add(LaneId);
			LaneToRoadHandleMap.Add(LaneId, RoadId);

			FReflectionLaneData& LData = LaneHandleMap.Add(LaneId);
			LData.LaneObject = LaneObj;
			LData.RoadActor = RoadActor;

			// Read LaneWidth.
			if (LaneWidthProp)
			{
				if (FDoubleProperty* DblProp = CastField<FDoubleProperty>(LaneWidthProp))
				{
					LData.LaneWidth = static_cast<float>(DblProp->GetPropertyValue_InContainer(LaneObj));
				}
				else if (FFloatProperty* FltProp = CastField<FFloatProperty>(LaneWidthProp))
				{
					LData.LaneWidth = FltProp->GetPropertyValue_InContainer(LaneObj);
				}
			}

			// Read edge curves.
			if (LeftEdgeProp)
			{
				FObjectPropertyBase* ObjP = CastField<FObjectPropertyBase>(LeftEdgeProp);
				if (ObjP) { LData.LeftEdge = ObjP->GetObjectPropertyValue_InContainer(LaneObj); }
			}
			if (RightEdgeProp)
			{
				FObjectPropertyBase* ObjP = CastField<FObjectPropertyBase>(RightEdgeProp);
				if (ObjP) { LData.RightEdge = ObjP->GetObjectPropertyValue_InContainer(LaneObj); }
			}

			// ── Phase 1A: Lane side detection ────────────────────
			LData.bIsLeftLane = LeftLaneSet.Contains(LaneObj);

			// ── Phase 1A: Cache per-section LaneTypes ────────────
			if (LaneSectionsProp)
			{
				FArrayProperty* SectArrProp = CastField<FArrayProperty>(LaneSectionsProp);
				if (SectArrProp)
				{
					FScriptArrayHelper Sections(SectArrProp,
						SectArrProp->ContainerPtrToValuePtr<void>(LaneObj));
					UScriptStruct* SectionStruct = CastField<FStructProperty>(SectArrProp->Inner)
						? CastField<FStructProperty>(SectArrProp->Inner)->Struct : nullptr;
					if (SectionStruct)
					{
						FProperty* SecDistProp = SectionStruct->FindPropertyByName(TEXT("SectionStartDistance"));
						FProperty* SecTypeProp = SectionStruct->FindPropertyByName(TEXT("LaneType"));
						for (int32 i = 0; i < Sections.Num(); ++i)
						{
							const uint8* ElemPtr = Sections.GetRawPtr(i);
							double SectionDist = 0.0;
							if (SecDistProp)
							{
								if (FDoubleProperty* DP = CastField<FDoubleProperty>(SecDistProp))
								{
									SectionDist = DP->GetPropertyValue(SecDistProp->ContainerPtrToValuePtr<void>(ElemPtr));
								}
							}
							uint8 SectionType = 0; // Default: Normal
							if (SecTypeProp)
							{
								if (FEnumProperty* EP = CastField<FEnumProperty>(SecTypeProp))
								{
									FNumericProperty* Under = EP->GetUnderlyingProperty();
									SectionType = static_cast<uint8>(Under->GetSignedIntPropertyValue(
										SecTypeProp->ContainerPtrToValuePtr<void>(ElemPtr)));
								}
								else if (FByteProperty* BP = CastField<FByteProperty>(SecTypeProp))
								{
									SectionType = BP->GetPropertyValue(
										SecTypeProp->ContainerPtrToValuePtr<void>(ElemPtr));
								}
							}
							LData.CachedLaneSections.Add(TPair<double, uint8>(SectionDist, SectionType));
						}
					}
				}
			}

			// ── Phase 1A: Cache ActiveSegments ──────────────────
			if (ActiveSegmentsProp)
			{
				FArrayProperty* SegArrProp = CastField<FArrayProperty>(ActiveSegmentsProp);
				if (SegArrProp)
				{
					FScriptArrayHelper Segments(SegArrProp,
						SegArrProp->ContainerPtrToValuePtr<void>(LaneObj));
					UScriptStruct* SegStruct = CastField<FStructProperty>(SegArrProp->Inner)
						? CastField<FStructProperty>(SegArrProp->Inner)->Struct : nullptr;
					if (SegStruct)
					{
						FProperty* StartDistProp   = SegStruct->FindPropertyByName(TEXT("StartDistance"));
						FProperty* EndDistProp     = SegStruct->FindPropertyByName(TEXT("EndDistance"));
						FProperty* TransInProp     = SegStruct->FindPropertyByName(TEXT("TransitionIn"));
						FProperty* TransOutProp    = SegStruct->FindPropertyByName(TEXT("TransitionOut"));
						for (int32 i = 0; i < Segments.Num(); ++i)
						{
							const uint8* ElemPtr = Segments.GetRawPtr(i);
							FTrafficLaneSegment Seg;
							if (FDoubleProperty* DP = CastField<FDoubleProperty>(StartDistProp))
							{
								Seg.StartDistance = DP->GetPropertyValue(StartDistProp->ContainerPtrToValuePtr<void>(ElemPtr));
							}
							if (FDoubleProperty* DP = CastField<FDoubleProperty>(EndDistProp))
							{
								Seg.EndDistance = DP->GetPropertyValue(EndDistProp->ContainerPtrToValuePtr<void>(ElemPtr));
							}
							if (FDoubleProperty* DP = CastField<FDoubleProperty>(TransInProp))
							{
								Seg.TransitionIn = DP->GetPropertyValue(TransInProp->ContainerPtrToValuePtr<void>(ElemPtr));
							}
							if (FDoubleProperty* DP = CastField<FDoubleProperty>(TransOutProp))
							{
								Seg.TransitionOut = DP->GetPropertyValue(TransOutProp->ContainerPtrToValuePtr<void>(ElemPtr));
							}
							LData.CachedActiveSegments.Add(Seg);
						}
					}
				}
			}

			// ── DIAG: Warn when a drivable lane has null edge curves ─
			if (!LData.LeftEdge.Get() || !LData.RightEdge.Get())
			{
				UE_LOG(LogAAATraffic, Warning,
					TEXT("RoadBLDReflectionProvider: Lane %d ('%s') on road '%s' "
						 "has NULL edge curves — LeftEdge=%s  RightEdge=%s. "
						 "Lane will fall back to road-center polyline."),
					LaneId,
					*LaneObj->GetName(),
					*RoadActor->GetName(),
					LData.LeftEdge.Get()  ? TEXT("OK") : TEXT("NULL"),
					LData.RightEdge.Get() ? TEXT("OK") : TEXT("NULL"));
			}

			LaneToHandleMap.Add(LaneObj, LaneId);
		}
	}

	bCached = true;

	// ── Phase 1A: Resolve road snap connections (second pass) ───
	// All road handles are assigned now, so we can resolve SnappedRoad pointers.
	if (StartSnappedRoadProp || EndSnappedRoadProp)
	{
		// Build reverse lookup: road actor → road handle ID
		TMap<TWeakObjectPtr<UObject>, int32> RoadActorToHandle;
		for (const auto& Pair : RoadHandleMap)
		{
			RoadActorToHandle.Add(Pair.Value, Pair.Key);
		}

		auto ExtractSnappedRoad = [&](FProperty* SnapProp, AActor* OwnerActor) -> int32
		{
			if (!SnapProp) { return 0; }
			// FRoadSnap is a struct with SnappedRoad (ADynamicRoad*)
			FStructProperty* StructProp = CastField<FStructProperty>(SnapProp);
			if (!StructProp) { return 0; }
			const void* StructPtr = SnapProp->ContainerPtrToValuePtr<void>(OwnerActor);
			FProperty* SnappedRoadField = StructProp->Struct->FindPropertyByName(TEXT("SnappedRoad"));
			if (!SnappedRoadField) { return 0; }
			FObjectPropertyBase* ObjProp = CastField<FObjectPropertyBase>(SnappedRoadField);
			if (!ObjProp) { return 0; }
			UObject* SnappedObj = ObjProp->GetObjectPropertyValue(SnappedRoadField->ContainerPtrToValuePtr<void>(StructPtr));
			if (!SnappedObj) { return 0; }
			const int32* Found = RoadActorToHandle.Find(SnappedObj);
			return Found ? *Found : 0;
		};

		for (const auto& Pair : RoadHandleMap)
		{
			AActor* RA = Cast<AActor>(Pair.Value.Get());
			if (!RA) { continue; }
			TArray<int32> Snapped;
			int32 StartSnap = ExtractSnappedRoad(StartSnappedRoadProp, RA);
			int32 EndSnap   = ExtractSnappedRoad(EndSnappedRoadProp, RA);
			if (StartSnap != 0) { Snapped.AddUnique(StartSnap); }
			if (EndSnap != 0)   { Snapped.AddUnique(EndSnap); }
			if (Snapped.Num() > 0)
			{
				Snapped.Sort(); // Deterministic order
				RoadSnapMap.Add(Pair.Key, MoveTemp(Snapped));
			}
		}
	}

	// ── Phase 1A: Summary logging ──────────────────────────────
	{
		int32 RoadsByType[4] = { 0, 0, 0, 0 };
		for (const auto& Pair : RoadTypeMap)
		{
			if (Pair.Value < 4) { RoadsByType[Pair.Value]++; }
		}
		int32 LanesWithMultiSections = 0;
		int32 LanesWithActiveSegments = 0;
		int32 LeftLaneCount = 0;
		int32 RightLaneCount = 0;
		for (const auto& Pair : LaneHandleMap)
		{
			if (Pair.Value.CachedLaneSections.Num() > 1) { LanesWithMultiSections++; }
			if (Pair.Value.CachedActiveSegments.Num() > 0) { LanesWithActiveSegments++; }
			if (Pair.Value.bIsLeftLane) { LeftLaneCount++; } else { RightLaneCount++; }
		}
		int32 TotalSnaps = 0;
		for (const auto& Pair : RoadSnapMap) { TotalSnaps += Pair.Value.Num(); }

		UE_LOG(LogAAATraffic, Log,
			TEXT("RoadBLDReflectionProvider [Phase1A]: RoadTypes: Normal=%d Walkway=%d Railway=%d Other=%d | "
				 "Lanes: Left=%d Right=%d MultiSection=%d WithActiveSegments=%d | "
				 "RoadSnaps: %d roads have %d total connections | "
				 "GetPointTurnRadiusFunc=%s"),
			RoadsByType[0], RoadsByType[1], RoadsByType[2], RoadsByType[3],
			LeftLaneCount, RightLaneCount, LanesWithMultiSections, LanesWithActiveSegments,
			RoadSnapMap.Num(), TotalSnaps,
			GetPointTurnRadiusFunc ? TEXT("OK") : TEXT("NULL"));
	}

	// ── DIAG: Post-cache edge sampling summary ──────────────────
	{
		int32 TotalLanes = 0;
		int32 LanesWithBothEdges = 0;
		int32 LanesWithMissingEdges = 0;
		for (const auto& Pair : LaneHandleMap)
		{
			++TotalLanes;
			if (Pair.Value.LeftEdge.Get() && Pair.Value.RightEdge.Get())
			{
				++LanesWithBothEdges;
			}
			else
			{
				++LanesWithMissingEdges;
			}
		}
		UE_LOG(LogAAATraffic, Log,
			TEXT("RoadBLDReflectionProvider: Cached %d roads, %d lanes. "
				 "EdgeSampling: %d/%d lanes have both edges, %d missing. "
				 "Get3DPosFunc=%s  ConvertDistFunc=%s"),
			RoadHandleMap.Num(), TotalLanes,
			LanesWithBothEdges, TotalLanes, LanesWithMissingEdges,
			Get3DPosFunc    ? TEXT("OK") : TEXT("NULL"),
			ConvertDistFunc ? TEXT("OK") : TEXT("NULL"));

		if (LanesWithMissingEdges > 0 || !Get3DPosFunc || !ConvertDistFunc)
		{
			UE_LOG(LogAAATraffic, Warning,
				TEXT("RoadBLDReflectionProvider: *** Edge sampling will FAIL for %d lane(s). "
					 "Vehicles on those lanes will drive on the road centerline. ***"),
				(!Get3DPosFunc || !ConvertDistFunc) ? TotalLanes : LanesWithMissingEdges);
		}
	}
}
// ---------------------------------------------------------------------------
// CacheLaneEndpoints — pre-compute geometry for every lane
// ---------------------------------------------------------------------------

void URoadBLDReflectionProvider::CacheLaneEndpoints()
{
	LaneEndpointMap.Empty();

	// Sort lane handles deterministically.
	TArray<int32> SortedHandles;
	LaneHandleMap.GetKeys(SortedHandles);
	SortedHandles.Sort();

	for (const int32 HandleId : SortedHandles)
	{
		TArray<FVector> Points;
		float Width = 0.0f;

		// Call the base GetLanePath which resolves polyline from reflection.
		// At this point VirtualLaneMap is empty so it will use original path logic.
		if (!GetLanePath(FTrafficLaneHandle(HandleId), Points, Width) || Points.Num() < 2)
		{
			continue;
		}

		FLaneEndpointCache Cache;
		Cache.Polyline = Points;
		Cache.Width = Width;
		Cache.StartPos = Points[0];
		Cache.EndPos = Points.Last();
		Cache.StartDir = (Points[1] - Points[0]).GetSafeNormal();
		Cache.EndDir = (Points.Last() - Points[Points.Num() - 2]).GetSafeNormal();

		LaneEndpointMap.Add(HandleId, MoveTemp(Cache));
	}

	// Build RoadTotalWidthMap: sum of all lane widths per road.
	RoadTotalWidthMap.Empty();
	for (const int32 HandleId : SortedHandles)
	{
		const FLaneEndpointCache* EP = LaneEndpointMap.Find(HandleId);
		if (!EP) { continue; }
		const int32* Road = LaneToRoadHandleMap.Find(HandleId);
		if (!Road) { continue; }
		float& Total = RoadTotalWidthMap.FindOrAdd(*Road);
		Total += EP->Width;
	}

	UE_LOG(LogAAATraffic, Log,
		TEXT("RoadBLDReflectionProvider: Cached endpoints for %d lanes, road widths for %d roads."),
		LaneEndpointMap.Num(), RoadTotalWidthMap.Num());

	// ── DIAG: Dump per-lane endpoint positions to verify opposing lanes
	// have distinct center lines (not sharing road centerline). ──
	if (GEnableDiagnosticDumps)
	{
	for (const int32 HandleId : SortedHandles)
	{
		const FLaneEndpointCache* EP = LaneEndpointMap.Find(HandleId);
		if (!EP) { continue; }
		const int32* Road = LaneToRoadHandleMap.Find(HandleId);
		UE_LOG(LogAAATraffic, Log,
			TEXT("JDIAG LANE-ENDPOINTS: Lane=%d Road=%d Width=%.1f "
				 "Start=(%.1f,%.1f,%.1f) End=(%.1f,%.1f,%.1f) "
				 "Dir=(%.3f,%.3f,%.3f)"),
			HandleId, Road ? *Road : -1, EP->Width,
			EP->StartPos.X, EP->StartPos.Y, EP->StartPos.Z,
			EP->EndPos.X, EP->EndPos.Y, EP->EndPos.Z,
			EP->StartDir.X, EP->StartDir.Y, EP->StartDir.Z);
	}
	} // GEnableDiagnosticDumps

	RebuildRoadSpeedClassification();
}
// ---------------------------------------------------------------------------
// SetSpeedTiers — update classified speed tiers and rebuild the road speed map.
// Called by TrafficSpawner after the provider is ready.
// ---------------------------------------------------------------------------

void URoadBLDReflectionProvider::SetSpeedTiers(float InResidentialSpeed, float InUrbanSpeed, float InHighwaySpeed)
{
	ConfiguredResidentialSpeed = InResidentialSpeed;
	ConfiguredUrbanSpeed = InUrbanSpeed;
	ConfiguredHighwaySpeed = InHighwaySpeed;

	RebuildRoadSpeedClassification();

	UE_LOG(LogAAATraffic, Log,
		TEXT("RoadBLDReflectionProvider: SetSpeedTiers — Residential=%.0f  Urban=%.0f  Highway=%.0f — reclassified %d roads."),
		ConfiguredResidentialSpeed, ConfiguredUrbanSpeed, ConfiguredHighwaySpeed,
		RoadClassifiedSpeedLimits.Num());
}
// ---------------------------------------------------------------------------
// RebuildRoadSpeedClassification — classify speed by lane count, then cap
// each road at the physics-safe speed for its tightest curve.
//
// Step 1: Assign tier speed based on lane count (residential / urban / highway).
// Step 2: For each road, walk every lane's cached polyline and measure
//         curvature in a sliding window.  Compute the minimum safe speed
//         from v = sqrt(0.3g × R) and cap the road's classified speed.
//
// This means a 2-lane residential road with a hairpin gets a LOWER speed
// than a straight 2-lane road, matching how real speed advisory signs work.
// ---------------------------------------------------------------------------

void URoadBLDReflectionProvider::RebuildRoadSpeedClassification()
{
	RoadClassifiedSpeedLimits.Empty();
	RoadClassificationMap.Empty();

	// Step 1: Classify by lane count + road length + connectivity.
	for (const auto& RoadEntry : RoadToLaneHandles)
	{
		const int32 RoadId = RoadEntry.Key;
		const int32 LaneCount = RoadEntry.Value.Num();
		float ClassifiedSpeed;
		ETrafficRoadClass RoadClass;

		if (LaneCount <= 2)
		{
			ClassifiedSpeed = ConfiguredResidentialSpeed;

			// Distinguish Local vs Collector: a 2-lane road that is long
			// and connects to multiple junctions is a collector, not a
			// dead-end residential street.  Closed-loop roads are always
			// Local (residential cul-de-sac / loop).
			RoadClass = ETrafficRoadClass::Local;

			if (!ClosedLoopRoadHandles.Contains(RoadId))
			{
				// Compute total polyline length for this road.
				float TotalRoadLength = 0.0f;
				for (const int32 LaneId : RoadEntry.Value)
				{
					const FLaneEndpointCache* Cache = LaneEndpointMap.Find(LaneId);
					if (Cache)
					{
						for (int32 i = 0; i < Cache->Polyline.Num() - 1; ++i)
						{
							TotalRoadLength += FVector::Dist(Cache->Polyline[i], Cache->Polyline[i + 1]);
						}
						break; // one lane's length represents the road
					}
				}

				// Count junctions this road participates in.
				int32 JunctionCount = 0;
				TSet<int32> SeenJunctions;
				for (const int32 LaneId : RoadEntry.Value)
				{
					// Check original and virtual lanes
					if (const TArray<int32>* Virtuals = OriginalToVirtualMap.Find(LaneId))
					{
						for (const int32 VH : *Virtuals)
						{
							if (const int32* JId = LaneToJunctionMap.Find(VH))
							{
								SeenJunctions.Add(*JId);
							}
						}
					}
					if (const int32* JId = LaneToJunctionMap.Find(LaneId))
					{
						SeenJunctions.Add(*JId);
					}
				}
				JunctionCount = SeenJunctions.Num();

				// Collector: ≥3000cm (30m) long AND connects to 3+ junctions
				if (TotalRoadLength >= 3000.0f && JunctionCount >= 3)
				{
					RoadClass = ETrafficRoadClass::Collector;
					ClassifiedSpeed = ConfiguredUrbanSpeed;
				}
			}
		}
		else if (LaneCount <= 4)
		{
			ClassifiedSpeed = ConfiguredUrbanSpeed;
			RoadClass = ETrafficRoadClass::Arterial;
		}
		else
		{
			ClassifiedSpeed = ConfiguredHighwaySpeed;
			RoadClass = ETrafficRoadClass::Freeway;
		}

		RoadClassifiedSpeedLimits.Add(RoadId, ClassifiedSpeed);
		RoadClassificationMap.Add(RoadId, RoadClass);
	}

	// Step 2: Cap each road's speed at the geometry-safe limit.
	// For every lane on this road, walk the cached polyline with a
	// sliding window, find the tightest turning radius, and compute
	// the maximum safe speed: v = sqrt(lateralAccelBudget × R).
	constexpr float LatAccelBudget = 294.0f; // 0.3g in cm/s²
	constexpr int32 WindowSize = 10;          // ~10m at 100cm samples
	constexpr float MinRadius = 50.0f;        // 0.5m clamp
	constexpr float MinAngleDeg = 3.0f;       // ignore nearly-straight windows

	int32 CappedCount = 0;
	for (auto& SpeedEntry : RoadClassifiedSpeedLimits)
	{
		const int32 RoadId = SpeedEntry.Key;
		float& RoadSpeed = SpeedEntry.Value;

		const TArray<int32>* LaneHandles = RoadToLaneHandles.Find(RoadId);
		if (!LaneHandles) { continue; }

		float TightestRadius = TNumericLimits<float>::Max();

		for (const int32 LaneId : *LaneHandles)
		{
			const FLaneEndpointCache* Cache = LaneEndpointMap.Find(LaneId);
			if (!Cache || Cache->Polyline.Num() < 3) { continue; }

			const TArray<FVector>& Poly = Cache->Polyline;

			// Sliding window: measure curvature in each window of WindowSize segments.
			for (int32 WinStart = 0; WinStart < Poly.Num() - 2; ++WinStart)
			{
				float WinAngleDeg = 0.0f;
				float WinArc = 0.0f;
				const int32 WinEnd = FMath::Min(WinStart + WindowSize, Poly.Num() - 2);

				for (int32 i = WinStart; i < WinEnd; ++i)
				{
					const FVector Seg0 = Poly[i + 1] - Poly[i];
					const FVector Seg1 = (i + 2 < Poly.Num())
						? (Poly[i + 2] - Poly[i + 1])
						: Seg0;
					WinArc += Seg0.Size();
					const FVector Dir0 = Seg0.GetSafeNormal();
					const FVector Dir1 = Seg1.GetSafeNormal();
					if (!Dir0.IsNearlyZero() && !Dir1.IsNearlyZero())
					{
						const float Dot = FMath::Clamp(FVector::DotProduct(Dir0, Dir1), -1.0f, 1.0f);
						WinAngleDeg += FMath::RadiansToDegrees(FMath::Acos(Dot));
					}
				}

				if (WinAngleDeg > MinAngleDeg && WinArc > KINDA_SMALL_NUMBER)
				{
					const float WinAngleRad = FMath::DegreesToRadians(WinAngleDeg);
					const float Radius = FMath::Max(WinArc / WinAngleRad, MinRadius);
					TightestRadius = FMath::Min(TightestRadius, Radius);
				}
			}
		}

		if (TightestRadius < TNumericLimits<float>::Max())
		{
			const float GeometrySpeedLimit = FMath::Sqrt(LatAccelBudget * TightestRadius);
			if (GeometrySpeedLimit < RoadSpeed)
			{
				RoadSpeed = GeometrySpeedLimit;
				++CappedCount;
			}
		}
	}

	UE_LOG(LogAAATraffic, Log,
		TEXT("RoadBLDReflectionProvider: Classified speed limits for %d roads (%d capped by curve geometry)."),
		RoadClassifiedSpeedLimits.Num(), CappedCount);

	// Log road class distribution.
	int32 LocalCount = 0, CollectorCount = 0, ArterialCount = 0, FreewayCount = 0;
	for (const auto& ClassEntry : RoadClassificationMap)
	{
		switch (ClassEntry.Value)
		{
		case ETrafficRoadClass::Local:     ++LocalCount; break;
		case ETrafficRoadClass::Collector: ++CollectorCount; break;
		case ETrafficRoadClass::Arterial:  ++ArterialCount; break;
		case ETrafficRoadClass::Freeway:   ++FreewayCount; break;
		}
	}
	UE_LOG(LogAAATraffic, Log,
		TEXT("RoadBLDReflectionProvider: Road classes — Local=%d Collector=%d Arterial=%d Freeway=%d"),
		LocalCount, CollectorCount, ArterialCount, FreewayCount);
}

// ---------------------------------------------------------------------------
// DetectAndSplitThroughRoads — create virtual lane segments at intersection
// mask boundaries.  A road with 2+ masks is a through-road; its lanes are
// ---------------------------------------------------------------------------
// BuildLaneAdjacency — detect left/right neighbor lanes per road
// ---------------------------------------------------------------------------

void URoadBLDReflectionProvider::BuildLaneAdjacency()
{
	LeftNeighborMap.Empty();
	RightNeighborMap.Empty();

	// For each road, examine pairs of lanes and detect adjacency via shared edge curves.
	// Lane A's RightEdge == Lane B's LeftEdge ⇒ B is to the right of A (and A is to the left of B).
	for (const auto& RoadPair : RoadToLaneHandles)
	{
		const TArray<int32>& LaneIds = RoadPair.Value;
		if (LaneIds.Num() < 2) { continue; }

		for (int32 i = 0; i < LaneIds.Num(); ++i)
		{
			const FReflectionLaneData* DataI = LaneHandleMap.Find(LaneIds[i]);
			if (!DataI) { continue; }

			for (int32 j = i + 1; j < LaneIds.Num(); ++j)
			{
				const FReflectionLaneData* DataJ = LaneHandleMap.Find(LaneIds[j]);
				if (!DataJ) { continue; }

				UObject* iRight = DataI->RightEdge.Get();
				UObject* jLeft  = DataJ->LeftEdge.Get();
				UObject* iLeft  = DataI->LeftEdge.Get();
				UObject* jRight = DataJ->RightEdge.Get();

				// I's right == J's left ⇒ J is to the right of I.
				if (iRight && jLeft && iRight == jLeft)
				{
					RightNeighborMap.Add(LaneIds[i], LaneIds[j]);
					LeftNeighborMap.Add(LaneIds[j], LaneIds[i]);
				}
				// I's left == J's right ⇒ J is to the left of I.
				else if (iLeft && jRight && iLeft == jRight)
				{
					LeftNeighborMap.Add(LaneIds[i], LaneIds[j]);
					RightNeighborMap.Add(LaneIds[j], LaneIds[i]);
				}
			}
		}
	}

	UE_LOG(LogAAATraffic, Log,
		TEXT("RoadBLDReflectionProvider: Built lane adjacency — %d left links, %d right links."),
		LeftNeighborMap.Num(), RightNeighborMap.Num());
}

// ---------------------------------------------------------------------------
// DetectReversedLanes — identify lanes on the left side of 2-way roads

void URoadBLDReflectionProvider::DetectReversedLanes()
{
	ReversedLaneSet.Empty();

	for (const auto& RoadPair : RoadToLaneHandles)
	{
		if (RoadPair.Value.Num() < 2) { continue; } // Single-lane — cannot be 2-way.

		UObject* RoadActor = RoadHandleMap.FindRef(RoadPair.Key).Get();
		if (!RoadActor) { continue; }

		const double RoadLength = GetRoadLength(RoadActor);
		if (RoadLength < 200.0) { continue; }

		UObject* RefLine = GetReferenceLine(RoadActor);
		if (!RefLine || !Get3DPosFunc) { continue; }

		// Sample reference line at multiple points to handle curved roads.
		// Single-midpoint sampling fails on loops where curvature shifts
		// lanes across the reference line centerpoint.
		struct FRefSample { FVector Pos; FVector Dir; float Fraction; };
		TArray<FRefSample> RefSamples;
		{
			constexpr double Fractions[] = { 0.25, 0.50, 0.75 };
			for (const double Frac : Fractions)
			{
				const double SampleDist = RoadLength * Frac;
				const double NearDist = FMath::Max(SampleDist - 100.0, 0.0);
				FRefSample S;
				S.Pos = Get3DPositionAtDistance(RefLine, RefLine, SampleDist);
				const FVector Near = Get3DPositionAtDistance(RefLine, RefLine, NearDist);
				S.Dir = (S.Pos - Near).GetSafeNormal();
				S.Fraction = static_cast<float>(Frac);
				if (!S.Dir.IsNearlyZero())
				{
					RefSamples.Add(S);
				}
			}
		}
		if (RefSamples.Num() == 0) { continue; }

		// Classify each lane by averaging lateral cross products across samples.
		struct FSideData { int32 LaneId; float Cross; };
		TArray<FSideData> SideInfo;
		int32 LeftCount = 0, RightCount = 0;

		for (int32 LaneId : RoadPair.Value)
		{
			TArray<FVector> Points;
			float Width;
			if (!GetLanePath(FTrafficLaneHandle(LaneId), Points, Width) || Points.Num() < 2) { continue; }

			float CrossSum = 0.0f;
			int32 ValidSamples = 0;
			for (const FRefSample& S : RefSamples)
			{
				const int32 LaneIdx = FMath::Clamp(
					FMath::RoundToInt32(static_cast<float>(Points.Num() - 1) * S.Fraction),
					0, Points.Num() - 1);
				const float Cross = FVector::CrossProduct(S.Dir, (Points[LaneIdx] - S.Pos)).Z;
				CrossSum += Cross;
				++ValidSamples;
			}
			const float AvgCross = (ValidSamples > 0) ? (CrossSum / static_cast<float>(ValidSamples)) : 0.0f;
			SideInfo.Add({ LaneId, AvgCross });
			if (AvgCross > 50.0f) { ++LeftCount; }
			else if (AvgCross < -50.0f) { ++RightCount; }
		}

		// Only mark reversed on 2-way roads (lanes on both sides).
		if (LeftCount == 0 || RightCount == 0) { continue; }

		for (const FSideData& S : SideInfo)
		{
			if (S.Cross > 50.0f) { ReversedLaneSet.Add(S.LaneId); }
		}
	}

	UE_LOG(LogAAATraffic, Log,
		TEXT("RoadBLDReflectionProvider: Detected %d reversed lanes on 2-way roads."),
		ReversedLaneSet.Num());
}

// ---------------------------------------------------------------------------
// DetectClosedLoopRoads — identify roads whose start ≈ end (closed loops)
// ---------------------------------------------------------------------------

void URoadBLDReflectionProvider::DetectClosedLoopRoads()
{
	ClosedLoopRoadHandles.Empty();

	constexpr double ClosureThresholdCm = 500.0; // 5 m — generous for RoadBLD snapping

	for (const auto& RoadPair : RoadHandleMap)
	{
		UObject* RoadActor = RoadPair.Value.Get();
		if (!RoadActor) { continue; }

		const double RoadLength = GetRoadLength(RoadActor);
		if (RoadLength < 1000.0) { continue; } // Ignore very short roads

		UObject* RefLine = GetReferenceLine(RoadActor);
		if (!RefLine || !Get3DPosFunc) { continue; }

		const FVector StartPos = Get3DPositionAtDistance(RefLine, RefLine, 0.0);
		const FVector EndPos = Get3DPositionAtDistance(RefLine, RefLine, RoadLength);

		const double Dist2D = FVector::Dist2D(StartPos, EndPos);
		if (Dist2D < ClosureThresholdCm)
		{
			ClosedLoopRoadHandles.Add(RoadPair.Key);
			UE_LOG(LogAAATraffic, Log,
				TEXT("RoadBLDReflectionProvider: Road %d detected as CLOSED LOOP (start-end dist=%.1f cm, length=%.0f cm)"),
				RoadPair.Key, Dist2D, RoadLength);
		}
	}

	UE_LOG(LogAAATraffic, Log,
		TEXT("RoadBLDReflectionProvider: Detected %d closed-loop roads."),
		ClosedLoopRoadHandles.Num());
}

// ---------------------------------------------------------------------------
bool URoadBLDReflectionProvider::IsRoadClosedLoop(const FTrafficRoadHandle& Road)
{
	return ClosedLoopRoadHandles.Contains(Road.HandleId);
}

// ---------------------------------------------------------------------------
ETrafficRoadClass URoadBLDReflectionProvider::GetRoadClass(const FTrafficRoadHandle& Road)
{
	if (const ETrafficRoadClass* Found = RoadClassificationMap.Find(Road.HandleId))
	{
		return *Found;
	}
	return ETrafficRoadClass::Local;
}

// ---------------------------------------------------------------------------
void URoadBLDReflectionProvider::SetFleetVehicleConstraints(float MinTurnRadiusCm, float MaxHalfWidthCm)
{
	FleetMinTurnRadiusCm = MinTurnRadiusCm;
	FleetMaxHalfWidthCm = MaxHalfWidthCm;

	UE_LOG(LogAAATraffic, Log,
		TEXT("RoadBLDReflectionProvider: Fleet constraints set — MinTurnRadius=%.0f cm, MaxHalfWidth=%.0f cm"),
		FleetMinTurnRadiusCm, FleetMaxHalfWidthCm);
}

float URoadBLDReflectionProvider::GetFleetMinTurnRadiusCm() const
{
	return FleetMinTurnRadiusCm;
}

float URoadBLDReflectionProvider::GetFleetMaxHalfWidthCm() const
{
	return FleetMaxHalfWidthCm;
}