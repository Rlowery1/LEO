// Copyright AAA_Traffic Contributors. All Rights Reserved.

#include "RoadBLDReflectionProvider.h"
#include "TrafficLog.h"
#include "Engine/World.h"
#include "EngineUtils.h"

// CVars defined in RoadBLDReflectionProvider.cpp
extern float GProximityThreshold;
extern float GMinUTurnWidth;
extern float GDirectionDotMin;
extern bool GEnableDiagnosticDumps;
extern int32 GTrafficDiagnosticsLevel;
extern int32 GTrafficDiagnosticsSampleLimit;

namespace
{
	static bool ShouldLogDiagnostics(const int32 Level)
	{
		return GEnableDiagnosticDumps || GTrafficDiagnosticsLevel >= Level;
	}

	static int32 GetDiagnosticsSampleLimit()
	{
		return FMath::Max(1, GTrafficDiagnosticsSampleLimit);
	}
}

// ---------------------------------------------------------------------------
// BuildLaneConnectivity — edge-walking corner discovery
// ---------------------------------------------------------------------------
// Attempt edge-walking via GetNextCornerConnection() first. If that returns
// zero corners (common — the function may require editor-only state), fall
// back to reading the RoadNetworkCorners TArray which CornerBuilder
// populates during RebuildRoadNetworkIncremental.

void URoadBLDReflectionProvider::BuildLaneConnectivity(UWorld* World)
{
	// Note: LaneConnectionMap is NOT cleared here — proximity connections
	// may already be present. Corner-based connections are additive.

	if (!DynNetworkClass)
	{
		UE_LOG(LogAAATraffic, Log,
			TEXT("RoadBLDReflectionProvider: DynamicRoadNetwork class not found — skipping corner-based connectivity."));
		return;
	}

	// Find the road network actor deterministically.
	AActor* NetworkActor = nullptr;
	{
		TArray<AActor*> NetworkCandidates;
		for (FActorIterator It(World); It; ++It)
		{
			if (It->IsA(DynNetworkClass))
			{
				NetworkCandidates.Add(*It);
			}
		}
		if (NetworkCandidates.Num() > 0)
		{
			NetworkCandidates.Sort([](const AActor& A, const AActor& B) { return A.GetName() < B.GetName(); });
			NetworkActor = NetworkCandidates[0];
		}
	}
	if (!NetworkActor)
	{
		UE_LOG(LogAAATraffic, Log,
			TEXT("RoadBLDReflectionProvider: No DynamicRoadNetwork actor — skipping corner-based connectivity."));
		return;
	}

	// Trigger rebuild and optional diagnostic dumps in a separate method
	// to keep this function focused on corner-based connectivity logic.
	TriggerRoadBLDRebuildAndDiagnostics(World, NetworkActor);

	// ── Resolve GetNextCornerConnection UFunction on the network class ──
	UFunction* GetNextCornerFunc = DynNetworkClass->FindFunctionByName(TEXT("GetNextCornerConnection"));
	if (!GetNextCornerFunc)
	{
		UE_LOG(LogAAATraffic, Warning,
			TEXT("RoadBLDReflectionProvider: GetNextCornerConnection not found on DynamicRoadNetwork — "
				 "falling back to RoadNetworkCorners TArray (may be empty)."));
		BuildLaneConnectivityFromCornersArray(World, NetworkActor);
		return;
	}

	// ── Resolve function parameter properties by name ──
	FObjectPropertyBase* ParamEdgeObjProp = nullptr;
	FProperty* ParamDistProp = nullptr;  // May be FDoubleProperty or FFloatProperty
	FBoolProperty*       ParamFoundBoolProp = nullptr;
	FStructProperty*     ReturnProp = nullptr;

	for (TFieldIterator<FProperty> It(GetNextCornerFunc); It; ++It)
	{
		FProperty* Prop = *It;
		const FName PropName = Prop->GetFName();

		if (PropName == TEXT("Edge"))
		{
			ParamEdgeObjProp = CastField<FObjectPropertyBase>(Prop);
		}
		else if (PropName == TEXT("CurrentDistance"))
		{
			ParamDistProp = Prop;
		}
		else if (PropName == TEXT("bFound"))
		{
			ParamFoundBoolProp = CastField<FBoolProperty>(Prop);
		}
		else if (Prop->HasAnyPropertyFlags(CPF_ReturnParm))
		{
			ReturnProp = CastField<FStructProperty>(Prop);
		}
	}

	if (!ParamEdgeObjProp || !ParamDistProp || !ParamFoundBoolProp || !ReturnProp)
	{
		UE_LOG(LogAAATraffic, Warning,
			TEXT("RoadBLDReflectionProvider: GetNextCornerConnection has unexpected parameter layout — "
				 "Edge=%s Dist=%s Found=%s Return=%s. Falling back to TArray."),
			ParamEdgeObjProp ? TEXT("OK") : TEXT("MISSING"),
			ParamDistProp ? TEXT("OK") : TEXT("MISSING"),
			ParamFoundBoolProp ? TEXT("OK") : TEXT("MISSING"),
			ReturnProp ? TEXT("OK") : TEXT("MISSING"));
		BuildLaneConnectivityFromCornersArray(World, NetworkActor);
		return;
	}

	// ── Resolve FRoadNetworkCorner sub-properties from the return struct ──
	UScriptStruct* CornerStruct = ReturnProp->Struct;
	if (!CornerStruct)
	{
		UE_LOG(LogAAATraffic, Warning,
			TEXT("RoadBLDReflectionProvider: Return value struct is null — cannot parse corner data."));
		BuildLaneConnectivityFromCornersArray(World, NetworkActor);
		return;
	}

	FObjectPropertyBase* CornerStartEdgeObj = CastField<FObjectPropertyBase>(
		CornerStruct->FindPropertyByName(TEXT("StartEdge")));
	FObjectPropertyBase* CornerEndEdgeObj = CastField<FObjectPropertyBase>(
		CornerStruct->FindPropertyByName(TEXT("EndEdge")));
	FProperty* CornerStartDistProp = CornerStruct->FindPropertyByName(TEXT("StartDistance"));
	FProperty* CornerEndDistProp = CornerStruct->FindPropertyByName(TEXT("EndDistance"));
	FBoolProperty* CornerStaleBool = CastField<FBoolProperty>(
		CornerStruct->FindPropertyByName(TEXT("bStale")));
	FStructProperty* CornerIDStructProp = CastField<FStructProperty>(
		CornerStruct->FindPropertyByName(TEXT("CornerID")));
	FProperty* CornerIntersectionPtProp = CornerStruct->FindPropertyByName(TEXT("IntersectionPoint"));

	if (!CornerStartEdgeObj || !CornerEndEdgeObj)
	{
		UE_LOG(LogAAATraffic, Warning,
			TEXT("RoadBLDReflectionProvider: FRoadNetworkCorner missing StartEdge/EndEdge — "
				 "cannot build connections. Falling back to TArray."));
		BuildLaneConnectivityFromCornersArray(World, NetworkActor);
		return;
	}

	// ── Resolve edge-curve LeftLane/RightLane properties ──
	FObjectPropertyBase* EdgeLeftLaneObj = nullptr;
	FObjectPropertyBase* EdgeRightLaneObj = nullptr;

	for (auto& Pair : RoadHandleMap)
	{
		UObject* RoadObj = Pair.Value.Get();
		if (!RoadObj) { continue; }

		FArrayProperty* EdgeCurvesProp = CastField<FArrayProperty>(
			RoadObj->GetClass()->FindPropertyByName(TEXT("EdgeCurves")));
		if (!EdgeCurvesProp) { continue; }

		FScriptArrayHelper EdgesHelper(EdgeCurvesProp,
			EdgeCurvesProp->ContainerPtrToValuePtr<void>(RoadObj));
		FObjectPropertyBase* InnerObjProp = CastField<FObjectPropertyBase>(EdgeCurvesProp->Inner);
		if (!InnerObjProp) { continue; }

		for (int32 e = 0; e < EdgesHelper.Num(); ++e)
		{
			UObject* EdgeObj = InnerObjProp->GetObjectPropertyValue(EdgesHelper.GetRawPtr(e));
			if (!EdgeObj) { continue; }

			FProperty* LeftP = EdgeObj->GetClass()->FindPropertyByName(TEXT("LeftLane"));
			FProperty* RightP = EdgeObj->GetClass()->FindPropertyByName(TEXT("RightLane"));
			EdgeLeftLaneObj = CastField<FObjectPropertyBase>(LeftP);
			EdgeRightLaneObj = CastField<FObjectPropertyBase>(RightP);
			if (EdgeLeftLaneObj && EdgeRightLaneObj) { break; }
		}
		if (EdgeLeftLaneObj && EdgeRightLaneObj) { break; }
	}

	if (!EdgeLeftLaneObj || !EdgeRightLaneObj)
	{
		UE_LOG(LogAAATraffic, Warning,
			TEXT("RoadBLDReflectionProvider: Could not resolve LeftLane/RightLane on edge curves — "
				 "no corner-based connections."));
		return;
	}

	// ── Lambda: collect lane handles from an edge object ──
	auto CollectLaneHandles = [&](UObject* Edge, TArray<int32>& OutHandles)
	{
		if (!Edge) { return; }
		UObject* LeftLane = EdgeLeftLaneObj->GetObjectPropertyValue_InContainer(Edge);
		UObject* RightLane = EdgeRightLaneObj->GetObjectPropertyValue_InContainer(Edge);
		if (LeftLane)
		{
			if (const int32* H = LaneToHandleMap.Find(LeftLane)) { OutHandles.AddUnique(*H); }
		}
		if (RightLane)
		{
			if (const int32* H = LaneToHandleMap.Find(RightLane)) { OutHandles.AddUnique(*H); }
		}
	};

	// ── Helper: read a double value from a property that may be double or float ──
	auto ReadDistanceValue = [](FProperty* Prop, const void* Container) -> double
	{
		if (FDoubleProperty* DblProp = CastField<FDoubleProperty>(Prop))
		{
			return DblProp->GetPropertyValue_InContainer(Container);
		}
		if (FFloatProperty* FltProp = CastField<FFloatProperty>(Prop))
		{
			return static_cast<double>(FltProp->GetPropertyValue_InContainer(Container));
		}
		return 0.0;
	};

	auto WriteDistanceValue = [](FProperty* Prop, void* Container, double Value)
	{
		if (FDoubleProperty* DblProp = CastField<FDoubleProperty>(Prop))
		{
			DblProp->SetPropertyValue_InContainer(Container, Value);
		}
		else if (FFloatProperty* FltProp = CastField<FFloatProperty>(Prop))
		{
			FltProp->SetPropertyValue_InContainer(Container, static_cast<float>(Value));
		}
	};

	// ── Walk all edge curves to discover corners ──────────────────
	TSet<FGuid> SeenCornerIDs;
	int32 CornerConnections = 0;
	int32 TotalCornersFound = 0;
	int32 TotalEdgesWalked = 0;

	// Sort road handles for deterministic iteration.
	TArray<int32> SortedRoadHandles;
	RoadHandleMap.GetKeys(SortedRoadHandles);
	SortedRoadHandles.Sort();

	for (const int32 RoadHandle : SortedRoadHandles)
	{
		UObject* RoadObj = RoadHandleMap[RoadHandle].Get();
		if (!RoadObj) { continue; }

		// Access EdgeCurves TArray property on this road.
		FArrayProperty* EdgeCurvesProp = CastField<FArrayProperty>(
			RoadObj->GetClass()->FindPropertyByName(TEXT("EdgeCurves")));
		if (!EdgeCurvesProp) { continue; }

		FScriptArrayHelper EdgesHelper(EdgeCurvesProp,
			EdgeCurvesProp->ContainerPtrToValuePtr<void>(RoadObj));
		FObjectPropertyBase* InnerObjProp = CastField<FObjectPropertyBase>(EdgeCurvesProp->Inner);
		if (!InnerObjProp) { continue; }

		for (int32 e = 0; e < EdgesHelper.Num(); ++e)
		{
			UObject* EdgeObj = InnerObjProp->GetObjectPropertyValue(EdgesHelper.GetRawPtr(e));
			if (!EdgeObj) { continue; }

			++TotalEdgesWalked;

			// Walk corners along this edge using GetNextCornerConnection.
			// Start at distance -1.0 to catch any corner at distance 0.0.
			double WalkDistance = -1.0;
			constexpr int32 MaxCornersPerEdge = 100; // safety limit

			for (int32 Step = 0; Step < MaxCornersPerEdge; ++Step)
			{
				// Allocate params buffer on the stack.
				TArray<uint8> ParamsStorage;
				ParamsStorage.AddZeroed(GetNextCornerFunc->ParmsSize);
				uint8* ParamsBuffer = ParamsStorage.GetData();

				// Set input parameters.
				ParamEdgeObjProp->SetObjectPropertyValue_InContainer(ParamsBuffer, EdgeObj);
				WriteDistanceValue(ParamDistProp, ParamsBuffer, WalkDistance);

				// Call GetNextCornerConnection on the network actor.
				NetworkActor->ProcessEvent(GetNextCornerFunc, ParamsBuffer);

				// Read bFound output.
				const bool bFound = ParamFoundBoolProp->GetPropertyValue_InContainer(ParamsBuffer);
				if (!bFound) { break; } // No more corners on this edge.

				// Read the returned FRoadNetworkCorner from the return value.
				const uint8* CornerPtr = ReturnProp->ContainerPtrToValuePtr<uint8>(ParamsBuffer);

				// ── Advance walk position past this corner ──
				UObject* CornerStartEdge = CornerStartEdgeObj->GetObjectPropertyValue_InContainer(CornerPtr);
				UObject* CornerEndEdge = CornerEndEdgeObj->GetObjectPropertyValue_InContainer(CornerPtr);

				double CornerDistOnThisEdge = WalkDistance + 100.0; // fallback
				if (CornerStartEdge == EdgeObj && CornerStartDistProp)
				{
					CornerDistOnThisEdge = ReadDistanceValue(CornerStartDistProp, CornerPtr);
				}
				else if (CornerEndEdge == EdgeObj && CornerEndDistProp)
				{
					CornerDistOnThisEdge = ReadDistanceValue(CornerEndDistProp, CornerPtr);
				}
				WalkDistance = CornerDistOnThisEdge + 0.1;

				// ── Skip stale corners ──
				if (CornerStaleBool && CornerStaleBool->GetPropertyValue_InContainer(CornerPtr))
				{
					continue;
				}

				// ── Deduplicate by CornerID (FGuid) ──
				// Each corner is seen from both participating edges; only process once.
				if (CornerIDStructProp)
				{
					FGuid ThisCornerID;
					const void* IDPtr = CornerIDStructProp->ContainerPtrToValuePtr<void>(CornerPtr);
					FMemory::Memcpy(&ThisCornerID, IDPtr, sizeof(FGuid));

					if (ThisCornerID.IsValid())
					{
						bool bAlreadySeen = false;
						SeenCornerIDs.Add(ThisCornerID, &bAlreadySeen);
						if (bAlreadySeen) { continue; }
					}
				}

				++TotalCornersFound;

				// ── Build cross-road connections from this corner ──
				if (!CornerStartEdge || !CornerEndEdge) { continue; }

				TArray<int32> StartHandles, EndHandles;
				CollectLaneHandles(CornerStartEdge, StartHandles);
				CollectLaneHandles(CornerEndEdge, EndHandles);

				// Connect StartEdge lanes ↔ EndEdge lanes (bidirectional).
				for (const int32 Src : StartHandles)
				{
					TArray<FTrafficLaneHandle>& Conns = LaneConnectionMap.FindOrAdd(Src);
					for (const int32 Dst : EndHandles)
					{
						if (Src != Dst)
						{
							Conns.AddUnique(FTrafficLaneHandle(Dst));
							++CornerConnections;
						}
					}
				}
				for (const int32 Src : EndHandles)
				{
					TArray<FTrafficLaneHandle>& Conns = LaneConnectionMap.FindOrAdd(Src);
					for (const int32 Dst : StartHandles)
					{
						if (Src != Dst)
						{
							Conns.AddUnique(FTrafficLaneHandle(Dst));
							++CornerConnections;
						}
					}
				}

				if (ShouldLogDiagnostics(2))
				{
					UE_LOG(LogAAATraffic, Log,
						TEXT("  Corner: StartEdge=%s EndEdge=%s StartLanes=[%s] EndLanes=[%s] Dist=%.1f"),
						CornerStartEdge ? *CornerStartEdge->GetName() : TEXT("null"),
						CornerEndEdge ? *CornerEndEdge->GetName() : TEXT("null"),
						*FString::JoinBy(StartHandles, TEXT(","),
							[](int32 H) { return FString::FromInt(H); }),
						*FString::JoinBy(EndHandles, TEXT(","),
							[](int32 H) { return FString::FromInt(H); }),
						CornerDistOnThisEdge);
				}
			}
		}
	}

	UE_LOG(LogAAATraffic, Log,
		TEXT("RoadBLDReflectionProvider: Edge-walking corner discovery — "
			 "walked %d edges across %d roads, found %d unique corners, "
			 "added %d cross-road connections."),
		TotalEdgesWalked, SortedRoadHandles.Num(),
		TotalCornersFound, CornerConnections);

	// ── Fallback: if edge-walking found nothing, try the TArray directly ──
	// After RebuildRoadNetworkIncremental, CornerBuilder may populate
	// RoadNetworkCorners even when GetNextCornerConnection returns empty.
	if (TotalCornersFound == 0)
	{
		UE_LOG(LogAAATraffic, Log,
			TEXT("RoadBLDReflectionProvider: Edge-walking found 0 corners — "
				 "falling back to RoadNetworkCorners TArray."));
		BuildLaneConnectivityFromCornersArray(World, NetworkActor);
	}
}

// ---------------------------------------------------------------------------
// BuildLaneConnectivityFromCornersArray — reads RoadNetworkCorners TArray
// ---------------------------------------------------------------------------
// Primary fallback when GetNextCornerConnection edge-walking returns empty.
// After RebuildRoadNetworkIncremental, CornerBuilder populates this TArray
// with the computed intersection corners.

void URoadBLDReflectionProvider::BuildLaneConnectivityFromCornersArray(
	UWorld* World, AActor* NetworkActor)
{
	if (!NetworkActor || !DynNetworkClass) { return; }

	// Try RoadNetworkCorners first, then CornerEditData as fallback.
	// CornerBuilder may populate either or both arrays depending on
	// RoadBLD version and configuration.
	FArrayProperty* CornerArrayProp = nullptr;
	FName UsedArrayName = NAME_None;

	{
		FProperty* P = DynNetworkClass->FindPropertyByName(TEXT("RoadNetworkCorners"));
		FArrayProperty* AP = P ? CastField<FArrayProperty>(P) : nullptr;
		if (AP)
		{
			FScriptArrayHelper H(AP, AP->ContainerPtrToValuePtr<void>(NetworkActor));
			if (H.Num() > 0)
			{
				CornerArrayProp = AP;
				UsedArrayName = TEXT("RoadNetworkCorners");
			}
			else
			{
				UE_LOG(LogAAATraffic, Log,
					TEXT("RoadBLDReflectionProvider: RoadNetworkCorners has 0 entries — trying CornerEditData."));
			}
		}
	}

	if (!CornerArrayProp)
	{
		FProperty* P = DynNetworkClass->FindPropertyByName(TEXT("CornerEditData"));
		FArrayProperty* AP = P ? CastField<FArrayProperty>(P) : nullptr;
		if (AP)
		{
			FScriptArrayHelper H(AP, AP->ContainerPtrToValuePtr<void>(NetworkActor));
			if (H.Num() > 0)
			{
				CornerArrayProp = AP;
				UsedArrayName = TEXT("CornerEditData");
			}
			else
			{
				UE_LOG(LogAAATraffic, Log,
					TEXT("RoadBLDReflectionProvider: CornerEditData also has 0 entries — no corner-based connectivity."));
			}
		}
	}

	if (!CornerArrayProp)
	{
		UE_LOG(LogAAATraffic, Log,
			TEXT("RoadBLDReflectionProvider: No populated corner array found — skipping corner-based connectivity."));
		return;
	}

	FScriptArrayHelper ArrayHelper(CornerArrayProp, CornerArrayProp->ContainerPtrToValuePtr<void>(NetworkActor));
	const int32 NumCorners = ArrayHelper.Num();

	UE_LOG(LogAAATraffic, Log,
		TEXT("RoadBLDReflectionProvider: Using %s with %d entries for corner connectivity."),
		*UsedArrayName.ToString(), NumCorners);

	FStructProperty* StructProp = CastField<FStructProperty>(CornerArrayProp->Inner);
	if (!StructProp) { return; }

	UScriptStruct* CornerStruct = StructProp->Struct;
	FObjectPropertyBase* StartEdgeObjProp = CastField<FObjectPropertyBase>(
		CornerStruct->FindPropertyByName(TEXT("StartEdge")));
	FObjectPropertyBase* EndEdgeObjProp = CastField<FObjectPropertyBase>(
		CornerStruct->FindPropertyByName(TEXT("EndEdge")));
	FBoolProperty* bStaleNative = CastField<FBoolProperty>(
		CornerStruct->FindPropertyByName(TEXT("bStale")));
	FProperty* IntPtProp = CornerStruct->FindPropertyByName(TEXT("IntersectionPoint"));

	if (!StartEdgeObjProp || !EndEdgeObjProp)
	{
		UE_LOG(LogAAATraffic, Warning,
			TEXT("RoadBLDReflectionProvider: %s entries lack StartEdge/EndEdge — "
				 "cannot build corner connections."),
			*UsedArrayName.ToString());
		return;
	}

	// Resolve edge-curve lane properties from first valid edge.
	FObjectPropertyBase* EdgeLeftLaneObj = nullptr;
	FObjectPropertyBase* EdgeRightLaneObj = nullptr;
	for (int32 i = 0; i < NumCorners; ++i)
	{
		UObject* Edge = StartEdgeObjProp->GetObjectPropertyValue_InContainer(ArrayHelper.GetRawPtr(i));
		if (Edge)
		{
			EdgeLeftLaneObj = CastField<FObjectPropertyBase>(
				Edge->GetClass()->FindPropertyByName(TEXT("LeftLane")));
			EdgeRightLaneObj = CastField<FObjectPropertyBase>(
				Edge->GetClass()->FindPropertyByName(TEXT("RightLane")));
			break;
		}
	}
	if (!EdgeLeftLaneObj || !EdgeRightLaneObj) { return; }

	auto CollectLaneHandles = [&](UObject* Edge, TArray<int32>& OutHandles)
	{
		if (!Edge) { return; }
		UObject* LeftLane  = EdgeLeftLaneObj->GetObjectPropertyValue_InContainer(Edge);
		UObject* RightLane = EdgeRightLaneObj->GetObjectPropertyValue_InContainer(Edge);

		UE_LOG(LogAAATraffic, Log,
			TEXT("    CollectLaneHandles: Edge=%s LeftLane=%s(%p) RightLane=%s(%p) LaneToHandleMap.Num=%d"),
			*Edge->GetName(),
			LeftLane ? *LeftLane->GetName() : TEXT("NULL"), static_cast<void*>(LeftLane),
			RightLane ? *RightLane->GetName() : TEXT("NULL"), static_cast<void*>(RightLane),
			LaneToHandleMap.Num());

		if (LeftLane)
		{
			if (const int32* H = LaneToHandleMap.Find(LeftLane))
			{
				OutHandles.AddUnique(*H);
			}
			else
			{
				// Edge->LeftLane points to a lane UObject not in our map.
				// Try by-name fallback: find any cached lane with the same name.
				for (const auto& MapPair : LaneToHandleMap)
				{
					if (MapPair.Key.IsValid() && MapPair.Key->GetName() == LeftLane->GetName())
					{
						OutHandles.AddUnique(MapPair.Value);
						UE_LOG(LogAAATraffic, Log,
							TEXT("      LeftLane '%s' matched by name to handle %d (cached ptr=%p vs edge ptr=%p)"),
							*LeftLane->GetName(), MapPair.Value, static_cast<void*>(MapPair.Key.Get()), static_cast<void*>(LeftLane));
						break;
					}
				}
			}
		}
		if (RightLane)
		{
			if (const int32* H = LaneToHandleMap.Find(RightLane))
			{
				OutHandles.AddUnique(*H);
			}
			else
			{
				for (const auto& MapPair : LaneToHandleMap)
				{
					if (MapPair.Key.IsValid() && MapPair.Key->GetName() == RightLane->GetName())
					{
						OutHandles.AddUnique(MapPair.Value);
						UE_LOG(LogAAATraffic, Log,
							TEXT("      RightLane '%s' matched by name to handle %d (cached ptr=%p vs edge ptr=%p)"),
							*RightLane->GetName(), MapPair.Value, static_cast<void*>(MapPair.Key.Get()), static_cast<void*>(RightLane));
						break;
					}
				}
			}
		}
	};

	int32 CornerConnections = 0;
	for (int32 i = 0; i < NumCorners; ++i)
	{
		const uint8* ElemPtr = ArrayHelper.GetRawPtr(i);
		if (bStaleNative && bStaleNative->GetPropertyValue_InContainer(ElemPtr)) { continue; }

		UObject* StartEdge = StartEdgeObjProp->GetObjectPropertyValue_InContainer(ElemPtr);
		UObject* EndEdge   = EndEdgeObjProp->GetObjectPropertyValue_InContainer(ElemPtr);
		if (!StartEdge || !EndEdge) { continue; }

		TArray<int32> StartHandles, EndHandles;
		CollectLaneHandles(StartEdge, StartHandles);
		CollectLaneHandles(EndEdge, EndHandles);

		// Connect StartEdge lanes ↔ EndEdge lanes (bidirectional).
		for (const int32 Src : StartHandles)
		{
			TArray<FTrafficLaneHandle>& Conns = LaneConnectionMap.FindOrAdd(Src);
			for (const int32 Dst : EndHandles)
			{
				if (Src != Dst)
				{
					Conns.AddUnique(FTrafficLaneHandle(Dst));
					++CornerConnections;
				}
			}
		}
		for (const int32 Src : EndHandles)
		{
			TArray<FTrafficLaneHandle>& Conns = LaneConnectionMap.FindOrAdd(Src);
			for (const int32 Dst : StartHandles)
			{
				if (Src != Dst)
				{
					Conns.AddUnique(FTrafficLaneHandle(Dst));
					++CornerConnections;
				}
			}
		}

		if (GEnableDiagnosticDumps)
		{
			UE_LOG(LogAAATraffic, Log,
				TEXT("  TArray Corner[%d]: StartEdge=%s EndEdge=%s StartLanes=[%s] EndLanes=[%s]"),
				i,
				*StartEdge->GetName(), *EndEdge->GetName(),
				*FString::JoinBy(StartHandles, TEXT(","),
					[](int32 H) { return FString::FromInt(H); }),
				*FString::JoinBy(EndHandles, TEXT(","),
					[](int32 H) { return FString::FromInt(H); }));
		}
	}

	if (GEnableDiagnosticDumps)
	{
		UE_LOG(LogAAATraffic, Log,
			TEXT("RoadBLDReflectionProvider: TArray corner connectivity added %d links from %d corners."),
			CornerConnections, NumCorners);
	}
}

// ---------------------------------------------------------------------------
// BuildProximityConnections — endpoint matching + U-turn gating
// ---------------------------------------------------------------------------

void URoadBLDReflectionProvider::BuildProximityConnections()
{
	ProximityConnectionList.Empty();

	const float ProxThreshold = GProximityThreshold;
	const float ProxThresholdSq = ProxThreshold * ProxThreshold;
	const float DirectionDotMin = GDirectionDotMin;
	const float MinUTurnWidth = GMinUTurnWidth;

	// Build working set: originals that were NOT split + all virtuals.
	TArray<int32> WorkingSet;
	for (const auto& Pair : LaneEndpointMap)
	{
		if (!ReplacedLaneHandles.Contains(Pair.Key))
		{
			WorkingSet.Add(Pair.Key);
		}
	}
	WorkingSet.Sort(); // Deterministic order.

	// Precompute effective road handle for each lane in the working set.
	TMap<int32, int32> EffectiveRoadMap;
	EffectiveRoadMap.Reserve(WorkingSet.Num());
	for (const int32 Handle : WorkingSet)
	{
		int32 Road = 0;
		if (const FVirtualLaneInfo* V = VirtualLaneMap.Find(Handle))
		{
			const int32* R = LaneToRoadHandleMap.Find(V->OriginalLaneHandle);
			Road = R ? *R : 0;
		}
		else
		{
			const int32* R = LaneToRoadHandleMap.Find(Handle);
			Road = R ? *R : 0;
		}
		EffectiveRoadMap.Add(Handle, Road);
	}

	int32 ProximityLinks = 0;
	int32 PairComparisons = 0;
	int32 SkipSameRoad = 0;
	int32 SkipDistance = 0;
	int32 ForwardAccepted = 0;
	int32 UTurnCandidates = 0;
	int32 UTurnAccepted = 0;
	int32 UTurnRejectedWidth = 0;
	int32 DuplicateConnectionAttempts = 0;

	TArray<FString> SampledRejects;
	TArray<FString> SampledAccepts;
	const bool bSampleDiagnostics = ShouldLogDiagnostics(3);
	const int32 MaxSamples = GetDiagnosticsSampleLimit();

	for (int32 i = 0; i < WorkingSet.Num(); ++i)
	{
		const int32 HandleA = WorkingSet[i];
		const FLaneEndpointCache* CacheA = LaneEndpointMap.Find(HandleA);
		if (!CacheA) { continue; }

		const int32 RoadA = EffectiveRoadMap[HandleA];

		for (int32 j = 0; j < WorkingSet.Num(); ++j)
		{
			if (i == j) { continue; }
			++PairComparisons;

			const int32 HandleB = WorkingSet[j];
			const FLaneEndpointCache* CacheB = LaneEndpointMap.Find(HandleB);
			if (!CacheB) { continue; }

			const int32 RoadB = EffectiveRoadMap[HandleB];

			// Only connect lanes on DIFFERENT roads.
			if (RoadA == RoadB && RoadA != 0)
			{
				++SkipSameRoad;
				continue;
			}

			// Check EndPos(A) → StartPos(B) proximity.
			const float DistSq = FVector::DistSquared(CacheA->EndPos, CacheB->StartPos);
			if (DistSq > ProxThresholdSq)
			{
				++SkipDistance;
				if (bSampleDiagnostics && SampledRejects.Num() < MaxSamples)
				{
					const float Dist = FMath::Sqrt(DistSq);
					if (FMath::Abs(Dist - ProxThreshold) <= 150.0f)
					{
						SampledRejects.Add(FString::Printf(
							TEXT("RejectDistance A=%d B=%d Dist=%.2f Threshold=%.2f RoadA=%d RoadB=%d"),
							HandleA, HandleB, Dist, ProxThreshold, RoadA, RoadB));
					}
				}
				continue;
			}

			// Direction compatibility: EndDir(A) · StartDir(B).
			const float Dot = FVector::DotProduct(CacheA->EndDir, CacheB->StartDir);

			if (Dot >= DirectionDotMin)
			{
				// Forward or angled connection — always allowed.
				TArray<FTrafficLaneHandle>& Conns = LaneConnectionMap.FindOrAdd(HandleA);
				const int32 PrevCount = Conns.Num();
				Conns.AddUnique(FTrafficLaneHandle(HandleB));
				if (Conns.Num() == PrevCount)
				{
					++DuplicateConnectionAttempts;
				}
				else
				{
					++ProximityLinks;
					++ForwardAccepted;
				}

				FProximityConnection PC;
				PC.FromLane = HandleA;
				PC.ToLane = HandleB;
				PC.Midpoint = (CacheA->EndPos + CacheB->StartPos) * 0.5f;
				ProximityConnectionList.Add(MoveTemp(PC));

				if (bSampleDiagnostics && SampledAccepts.Num() < MaxSamples)
				{
					SampledAccepts.Add(FString::Printf(
						TEXT("AcceptForward A=%d B=%d Dist=%.2f Dot=%.3f RoadA=%d RoadB=%d"),
						HandleA, HandleB, FMath::Sqrt(DistSq), Dot, RoadA, RoadB));
				}
			}
			else
			{
				++UTurnCandidates;
				// U-turn candidate — gate by total road width (not individual lane width).
				const float* RoadWidthA = RoadTotalWidthMap.Find(RoadA);
				const float* RoadWidthB = RoadTotalWidthMap.Find(RoadB);
				const float WidthA = RoadWidthA ? *RoadWidthA : CacheA->Width;
				const float WidthB = RoadWidthB ? *RoadWidthB : CacheB->Width;
				const bool bWidthOk = (WidthA >= MinUTurnWidth) && (WidthB >= MinUTurnWidth);

				if (bWidthOk)
				{
					TArray<FTrafficLaneHandle>& Conns = LaneConnectionMap.FindOrAdd(HandleA);
					const int32 PrevCount = Conns.Num();
					Conns.AddUnique(FTrafficLaneHandle(HandleB));
					if (Conns.Num() == PrevCount)
					{
						++DuplicateConnectionAttempts;
					}
					else
					{
						++ProximityLinks;
						++UTurnAccepted;
					}

					FProximityConnection PC;
					PC.FromLane = HandleA;
					PC.ToLane = HandleB;
					PC.Midpoint = (CacheA->EndPos + CacheB->StartPos) * 0.5f;
					ProximityConnectionList.Add(MoveTemp(PC));

					if (bSampleDiagnostics && SampledAccepts.Num() < MaxSamples)
					{
						SampledAccepts.Add(FString::Printf(
							TEXT("AcceptUTurn A=%d B=%d Dist=%.2f Dot=%.3f WidthA=%.1f WidthB=%.1f"),
							HandleA, HandleB, FMath::Sqrt(DistSq), Dot, WidthA, WidthB));
					}
				}
				else
				{
					++UTurnRejectedWidth;
					if (bSampleDiagnostics && SampledRejects.Num() < MaxSamples)
					{
						SampledRejects.Add(FString::Printf(
							TEXT("RejectUTurnWidth A=%d B=%d Dist=%.2f Dot=%.3f WidthA=%.1f WidthB=%.1f MinWidth=%.1f"),
							HandleA, HandleB, FMath::Sqrt(DistSq), Dot, WidthA, WidthB, MinUTurnWidth));
					}
				}
			}
		}
	}

	// Sort all connection lists deterministically.
	for (auto& Pair : LaneConnectionMap)
	{
		Pair.Value.Sort([](const FTrafficLaneHandle& A, const FTrafficLaneHandle& B)
		{
			return A.HandleId < B.HandleId;
		});
	}

	UE_LOG(LogAAATraffic, Log,
		TEXT("RoadBLDReflectionProvider: Proximity connections added %d links (U-turn gating at %.0f cm)."),
		ProximityLinks, MinUTurnWidth);

	if (ShouldLogDiagnostics(2))
	{
		UE_LOG(LogAAATraffic, Log,
			TEXT("RoadBLDReflectionProvider: ProximityDiag PairComparisons=%d WorkingSet=%d SkipSameRoad=%d SkipDistance=%d ForwardAccepted=%d UTurnCandidates=%d UTurnAccepted=%d UTurnRejectedWidth=%d DuplicateAttempts=%d"),
			PairComparisons,
			WorkingSet.Num(),
			SkipSameRoad,
			SkipDistance,
			ForwardAccepted,
			UTurnCandidates,
			UTurnAccepted,
			UTurnRejectedWidth,
			DuplicateConnectionAttempts);
	}

	if (bSampleDiagnostics)
	{
		for (const FString& Line : SampledAccepts)
		{
			UE_LOG(LogAAATraffic, Log, TEXT("RoadBLDReflectionProvider: ProximitySample %s"), *Line);
		}
		for (const FString& Line : SampledRejects)
		{
			UE_LOG(LogAAATraffic, Log, TEXT("RoadBLDReflectionProvider: ProximitySample %s"), *Line);
		}
	}
}