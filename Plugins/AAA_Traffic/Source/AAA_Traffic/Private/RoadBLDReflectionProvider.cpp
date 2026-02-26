// Copyright AAA_Traffic Contributors. All Rights Reserved.

#include "RoadBLDReflectionProvider.h"
#include "RoadBLDAPIContract.h"
#include "TrafficSubsystem.h"
#include "TrafficLog.h"
#include "Engine/World.h"
#include "EngineUtils.h"

// ---------------------------------------------------------------------------
// CVars — tunable proximity connectivity thresholds
// ---------------------------------------------------------------------------

static float GProximityThreshold = 500.0f;
static float GThroughRoadRadius = 500.0f;
static float GMinUTurnWidth = 1100.0f;
static float GDirectionDotMin = -0.5f;

static FAutoConsoleVariableRef CVarProximityThreshold(
	TEXT("traffic.ProximityThreshold"),
	GProximityThreshold,
	TEXT("Max distance (cm) between two lane endpoints to consider them connected. Default 500."),
	ECVF_Default);

static FAutoConsoleVariableRef CVarThroughRoadRadius(
	TEXT("traffic.ThroughRoadRadius"),
	GThroughRoadRadius,
	TEXT("Max distance (cm) from a side-road endpoint to a through-road polyline midpoint for split detection. Default 500."),
	ECVF_Default);

static FAutoConsoleVariableRef CVarMinUTurnWidth(
	TEXT("traffic.MinUTurnWidth"),
	GMinUTurnWidth,
	TEXT("Minimum total road width (cm) at junction to allow U-turn connections. Default 1100 (AASHTO standard)."),
	ECVF_Default);

static FAutoConsoleVariableRef CVarDirectionDotMin(
	TEXT("traffic.DirectionDotMin"),
	GDirectionDotMin,
	TEXT("Minimum dot product between lane directions for non-U-turn connections. Default -0.5."),
	ECVF_Default);

// ---------------------------------------------------------------------------
// USubsystem overrides
// ---------------------------------------------------------------------------

bool URoadBLDReflectionProvider::ShouldCreateSubsystem(UObject* Outer) const
{
	// Only spin up if RoadBLD classes are actually loaded.
	return FindObject<UClass>(nullptr, TEXT("/Script/RoadBLDRuntime.DynamicRoad")) != nullptr;
}

void URoadBLDReflectionProvider::Initialize(FSubsystemCollectionBase& Collection)
{
	Super::Initialize(Collection);
	NextHandleId = 1;
	bCached = false;
}

void URoadBLDReflectionProvider::Deinitialize()
{
	if (UWorld* World = GetWorld())
	{
		if (UTrafficSubsystem* TrafficSub = World->GetSubsystem<UTrafficSubsystem>())
		{
			TrafficSub->UnregisterProvider(this);
		}
	}

	RoadHandleMap.Empty();
	RoadToLaneHandles.Empty();
	LaneHandleMap.Empty();
	LaneToHandleMap.Empty();
	LaneConnectionMap.Empty();
	LeftNeighborMap.Empty();
	RightNeighborMap.Empty();
	LaneToRoadHandleMap.Empty();
	LaneEndpointMap.Empty();
	VirtualLaneMap.Empty();
	OriginalToVirtualMap.Empty();
	ReplacedLaneHandles.Empty();
	ProximityConnectionList.Empty();
	bCached = false;

	DynRoadClass = nullptr;
	DynNetworkClass = nullptr;
	GetLengthFunc = nullptr;
	GetAllLanesFunc = nullptr;
	ConvertDistFunc = nullptr;
	GetWorldPosFunc = nullptr;
	RefLineProp = nullptr;
	Get3DPosFunc = nullptr;
	LeftEdgeProp = nullptr;
	RightEdgeProp = nullptr;
	LaneWidthProp = nullptr;

	Super::Deinitialize();
}

void URoadBLDReflectionProvider::OnWorldBeginPlay(UWorld& InWorld)
{
	Super::OnWorldBeginPlay(InWorld);

	// Yield to an already-registered compiled provider (URoadBLDTrafficProvider).
	UTrafficSubsystem* TrafficSub = InWorld.GetSubsystem<UTrafficSubsystem>();
	if (TrafficSub && TrafficSub->GetProvider() != nullptr)
	{
		UE_LOG(LogAAATraffic, Log,
			TEXT("RoadBLDReflectionProvider: A compiled provider is already registered — yielding."));
		return;
	}

	// ── API Contract Validation ──────────────────────────────────
	FRoadBLDAPIContractResult ContractResult = FRoadBLDAPIContract::Validate();
	if (!ContractResult.bAllPassed)
	{
		UE_LOG(LogAAATraffic, Error,
			TEXT("RoadBLDReflectionProvider: API contract validation FAILED — %d issue(s):"),
			ContractResult.Failures.Num());
		for (const FRoadBLDContractFailure& Fail : ContractResult.Failures)
		{
			UE_LOG(LogAAATraffic, Error, TEXT("  %s"), *Fail.ToString());
		}
		UE_LOG(LogAAATraffic, Error,
			TEXT("RoadBLDReflectionProvider: Provider will NOT register. The installed RoadBLD version may be incompatible."));
		return;
	}
	UE_LOG(LogAAATraffic, Log, TEXT("RoadBLDReflectionProvider: API contract validation passed."));

	// ── Cache road/lane data ─────────────────────────────────────
	CacheRoadData(&InWorld);

	if (!bCached || RoadHandleMap.IsEmpty())
	{
		UE_LOG(LogAAATraffic, Warning,
			TEXT("RoadBLDReflectionProvider: No roads found — provider will NOT register."));
		return;
	}

	// ── Detect reversed lanes on 2-way roads ─────────────────────
	DetectReversedLanes();

	// ── Build same-road lane adjacency (left/right neighbors) ────
	BuildLaneAdjacency();

	// ── Cache lane endpoint geometry ────────────────────────────
	CacheLaneEndpoints();

	// ── Detect through-roads and create virtual lane splits ──────
	DetectAndSplitThroughRoads();

	// ── Corner-based connectivity (RoadNetworkCorners fallback) ──
	BuildLaneConnectivity(&InWorld);

	// ── Proximity-based connectivity ────────────────────────────
	BuildProximityConnections();

	// ── Group connections into junctions ────────────────────────
	BuildJunctionGrouping();

	// ── Register ─────────────────────────────────────────────────
	if (TrafficSub)
	{
		TrafficSub->RegisterProvider(this);
		UE_LOG(LogAAATraffic, Log,
			TEXT("RoadBLDReflectionProvider: Registered — %d roads, %d lanes cached."),
			RoadHandleMap.Num(), LaneHandleMap.Num());
	}
}

// ---------------------------------------------------------------------------
// ITrafficRoadProvider
// ---------------------------------------------------------------------------

TArray<FTrafficRoadHandle> URoadBLDReflectionProvider::GetAllRoads()
{
	TArray<FTrafficRoadHandle> Result;
	Result.Reserve(RoadHandleMap.Num());

	TArray<int32> Keys;
	RoadHandleMap.GetKeys(Keys);
	Keys.Sort();

	for (const int32 Key : Keys)
	{
		if (RoadHandleMap[Key].IsValid())
		{
			Result.Emplace(Key);
		}
	}
	return Result;
}

TArray<FTrafficLaneHandle> URoadBLDReflectionProvider::GetLanesForRoad(const FTrafficRoadHandle& Road)
{
	TArray<FTrafficLaneHandle> Result;

	const TArray<int32>* LaneIds = RoadToLaneHandles.Find(Road.HandleId);
	if (!LaneIds) { return Result; }

	for (const int32 LaneId : *LaneIds)
	{
		// If this lane was split into virtual segments, return those instead.
		if (const TArray<int32>* Virtuals = OriginalToVirtualMap.Find(LaneId))
		{
			for (int32 VId : *Virtuals) { Result.Emplace(VId); }
		}
		else
		{
			Result.Emplace(LaneId);
		}
	}
	return Result;
}

bool URoadBLDReflectionProvider::GetLanePath(
	const FTrafficLaneHandle& Lane,
	TArray<FVector>& OutPoints,
	float& OutWidth)
{
	// Virtual lane — return a subset of the original lane's polyline.
	if (const FVirtualLaneInfo* VInfo = VirtualLaneMap.Find(Lane.HandleId))
	{
		const FLaneEndpointCache* OrigCache = LaneEndpointMap.Find(VInfo->OriginalLaneHandle);
		if (!OrigCache || OrigCache->Polyline.Num() == 0) { return false; }

		const int32 Start = FMath::Clamp(VInfo->StartPointIndex, 0, OrigCache->Polyline.Num() - 1);
		const int32 End = FMath::Clamp(VInfo->EndPointIndex, Start, OrigCache->Polyline.Num() - 1);

		OutPoints.Reset(End - Start + 1);
		for (int32 i = Start; i <= End; ++i)
		{
			OutPoints.Add(OrigCache->Polyline[i]);
		}
		OutWidth = OrigCache->Width;
		return OutPoints.Num() >= 2;
	}

	// Check cached endpoint polyline first (avoids recomputation).
	if (const FLaneEndpointCache* Cached = LaneEndpointMap.Find(Lane.HandleId))
	{
		if (Cached->Polyline.Num() >= 2)
		{
			OutPoints = Cached->Polyline;
			OutWidth = Cached->Width;
			return true;
		}
	}

	const FReflectionLaneData* Data = LaneHandleMap.Find(Lane.HandleId);
	if (!Data) { return false; }

	UObject* RoadActor = Data->RoadActor.Get();
	if (!RoadActor) { return false; }

	OutWidth = Data->LaneWidth;

	const double RoadLength = GetRoadLength(RoadActor);
	if (RoadLength <= 0.0) { return false; }

	UObject* RefLine = GetReferenceLine(RoadActor);
	UObject* LeftEdge = Data->LeftEdge.Get();
	UObject* RightEdge = Data->RightEdge.Get();

	const double SampleInterval = 100.0; // 1 m
	const int32 NumSamples = FMath::Max(2, FMath::CeilToInt(RoadLength / SampleInterval) + 1);

	OutPoints.Reset(NumSamples);

	const bool bCanDoEdgeSampling = LeftEdge && RightEdge && RefLine
		&& Get3DPosFunc && ConvertDistFunc;

	for (int32 i = 0; i < NumSamples; ++i)
	{
		const double Dist = FMath::Min(RoadLength,
			(RoadLength * static_cast<double>(i)) / static_cast<double>(NumSamples - 1));

		FVector Pos;

		if (bCanDoEdgeSampling)
		{
			const double LeftDist = ConvertDistanceBetweenCurves(RoadActor, RefLine, LeftEdge, Dist);
			const double RightDist = ConvertDistanceBetweenCurves(RoadActor, RefLine, RightEdge, Dist);

			const FVector LeftPos = Get3DPositionAtDistance(LeftEdge, RefLine, LeftDist);
			const FVector RightPos = Get3DPositionAtDistance(RightEdge, RefLine, RightDist);
			Pos = (LeftPos + RightPos) * 0.5;
		}
		else
		{
			// Fallback: 2D centerline from road actor.
			const FVector2D Pos2D = GetWorldPositionAtDistance(RoadActor, Dist);
			Pos = FVector(Pos2D.X, Pos2D.Y, RoadActor->IsA<AActor>()
				? CastChecked<AActor>(RoadActor)->GetActorLocation().Z : 0.0);
		}

		OutPoints.Add(Pos);
	}

	// Reverse points for lanes whose travel direction opposes the reference line (C3).
	if (ReversedLaneSet.Contains(Lane.HandleId))
	{
		Algo::Reverse(OutPoints);
	}

	return OutPoints.Num() >= 2;
}

FVector URoadBLDReflectionProvider::GetLaneDirection(const FTrafficLaneHandle& Lane)
{
	// Virtual or cached endpoint — use cached endpoint data.
	if (const FLaneEndpointCache* Cached = LaneEndpointMap.Find(Lane.HandleId))
	{
		return (Cached->EndPos - Cached->StartPos).GetSafeNormal();
	}
	TArray<FVector> Points;
	float Width;
	if (GetLanePath(Lane, Points, Width) && Points.Num() >= 2)
	{
		return (Points.Last() - Points[0]).GetSafeNormal();
	}
	return FVector::ForwardVector;
}

TArray<FTrafficLaneHandle> URoadBLDReflectionProvider::GetConnectedLanes(const FTrafficLaneHandle& Lane)
{
	if (const TArray<FTrafficLaneHandle>* Connected = LaneConnectionMap.Find(Lane.HandleId))
	{
		return *Connected;
	}
	return TArray<FTrafficLaneHandle>();
}

FTrafficLaneHandle URoadBLDReflectionProvider::GetLaneAtLocation(const FVector& Location)
{
	// Check all lane endpoints (including virtual) for closest midpoint.
	float BestDistSq = MAX_flt;
	int32 BestLaneHandle = 0;

	TArray<int32> EndpointKeys;
	LaneEndpointMap.GetKeys(EndpointKeys);
	EndpointKeys.Sort();

	for (const int32 LaneId : EndpointKeys)
	{
		// Skip original lanes that have been replaced by virtual segments.
		if (ReplacedLaneHandles.Contains(LaneId)) { continue; }

		const FLaneEndpointCache& Cache = LaneEndpointMap[LaneId];
		const FVector Midpoint = (Cache.StartPos + Cache.EndPos) * 0.5f;
		const float DistSq = FVector::DistSquared(Location, Midpoint);
		if (DistSq < BestDistSq)
		{
			BestDistSq = DistSq;
			BestLaneHandle = LaneId;
		}
	}

	return FTrafficLaneHandle(BestLaneHandle);
}

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

				UE_LOG(LogAAATraffic, Log,
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

			LaneToHandleMap.Add(LaneObj, LaneId);
		}
	}

	bCached = true;

	UE_LOG(LogAAATraffic, Log,
		TEXT("RoadBLDReflectionProvider: Cached %d roads, %d lanes via reflection."),
		RoadHandleMap.Num(), LaneHandleMap.Num());
}

// ---------------------------------------------------------------------------
// BuildLaneConnectivity — corner-based fallback from RoadNetworkCorners
// ---------------------------------------------------------------------------

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

	// Access RoadNetworkCorners TArray via reflection.
	FProperty* CornersProp = DynNetworkClass->FindPropertyByName(TEXT("RoadNetworkCorners"));
	FArrayProperty* CornerArrayProp = CornersProp ? CastField<FArrayProperty>(CornersProp) : nullptr;
	if (!CornerArrayProp)
	{
		UE_LOG(LogAAATraffic, Log,
			TEXT("RoadBLDReflectionProvider: RoadNetworkCorners property not found or not array — skipping."));
		return;
	}

	FScriptArrayHelper ArrayHelper(CornerArrayProp, CornerArrayProp->ContainerPtrToValuePtr<void>(NetworkActor));
	const int32 NumCorners = ArrayHelper.Num();

	if (NumCorners == 0)
	{
		UE_LOG(LogAAATraffic, Log,
			TEXT("RoadBLDReflectionProvider: RoadNetworkCorners is empty — no corner-based connectivity."));
		return;
	}

	// Resolve FRoadNetworkCorner struct properties.
	FStructProperty* StructProp = CastField<FStructProperty>(CornerArrayProp->Inner);
	if (!StructProp) { return; }

	UScriptStruct* CornerStruct = StructProp->Struct;
	FProperty* StartEdgeProp = CornerStruct->FindPropertyByName(TEXT("StartEdge"));
	FProperty* EndEdgeProp   = CornerStruct->FindPropertyByName(TEXT("EndEdge"));
	FProperty* bStaleProp    = CornerStruct->FindPropertyByName(TEXT("bStale"));
	IntersectionPointProp    = CornerStruct->FindPropertyByName(TEXT("IntersectionPoint"));

	FObjectPropertyBase* StartEdgeObjProp = CastField<FObjectPropertyBase>(StartEdgeProp);
	FObjectPropertyBase* EndEdgeObjProp   = CastField<FObjectPropertyBase>(EndEdgeProp);
	FBoolProperty*       bStaleNative     = CastField<FBoolProperty>(bStaleProp);

	if (!StartEdgeObjProp || !EndEdgeObjProp) { return; }

	// Resolve edge-curve lane properties.
	FProperty* EdgeLeftLaneProp  = nullptr;
	FProperty* EdgeRightLaneProp = nullptr;
	for (int32 i = 0; i < NumCorners; ++i)
	{
		const uint8* ElemPtr = ArrayHelper.GetRawPtr(i);
		UObject* Edge = StartEdgeObjProp->GetObjectPropertyValue_InContainer(ElemPtr);
		if (Edge)
		{
			EdgeLeftLaneProp  = Edge->GetClass()->FindPropertyByName(TEXT("LeftLane"));
			EdgeRightLaneProp = Edge->GetClass()->FindPropertyByName(TEXT("RightLane"));
			break;
		}
	}
	if (!EdgeLeftLaneProp || !EdgeRightLaneProp) { return; }

	FObjectPropertyBase* EdgeLeftLaneObj  = CastField<FObjectPropertyBase>(EdgeLeftLaneProp);
	FObjectPropertyBase* EdgeRightLaneObj = CastField<FObjectPropertyBase>(EdgeRightLaneProp);
	if (!EdgeLeftLaneObj || !EdgeRightLaneObj) { return; }

	auto CollectLaneHandles = [&](UObject* Edge, TArray<int32>& OutHandles)
	{
		if (!Edge) { return; }
		UObject* LeftLane  = EdgeLeftLaneObj->GetObjectPropertyValue_InContainer(Edge);
		UObject* RightLane = EdgeRightLaneObj->GetObjectPropertyValue_InContainer(Edge);
		if (LeftLane)  { if (const int32* H = LaneToHandleMap.Find(LeftLane))  { OutHandles.AddUnique(*H); } }
		if (RightLane) { if (const int32* H = LaneToHandleMap.Find(RightLane)) { OutHandles.AddUnique(*H); } }
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
	}

	UE_LOG(LogAAATraffic, Log,
		TEXT("RoadBLDReflectionProvider: Corner-based connectivity added %d links from %d corners."),
		CornerConnections, NumCorners);
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

	UE_LOG(LogAAATraffic, Log,
		TEXT("RoadBLDReflectionProvider: Cached endpoints for %d lanes."),
		LaneEndpointMap.Num());
}

// ---------------------------------------------------------------------------
// DetectAndSplitThroughRoads — create virtual lane segments
// ---------------------------------------------------------------------------

void URoadBLDReflectionProvider::DetectAndSplitThroughRoads()
{
	VirtualLaneMap.Empty();
	OriginalToVirtualMap.Empty();
	ReplacedLaneHandles.Empty();

	const float SplitRadius = GThroughRoadRadius;
	const float SplitRadiusSq = SplitRadius * SplitRadius;

	// Collect all lane endpoints (start + end) grouped by road.
	struct FEndpointInfo
	{
		int32 LaneHandle;
		int32 RoadHandle;
		FVector Position;
	};
	TArray<FEndpointInfo> AllEndpoints;

	// Sort roads deterministically.
	TArray<int32> SortedRoads;
	RoadToLaneHandles.GetKeys(SortedRoads);
	SortedRoads.Sort();

	for (const int32 RoadId : SortedRoads)
	{
		const TArray<int32>& Lanes = RoadToLaneHandles[RoadId];
		for (const int32 LaneId : Lanes)
		{
			const FLaneEndpointCache* EP = LaneEndpointMap.Find(LaneId);
			if (!EP) { continue; }
			AllEndpoints.Add({ LaneId, RoadId, EP->StartPos });
			AllEndpoints.Add({ LaneId, RoadId, EP->EndPos });
		}
	}

	// For each endpoint on road X, check if it is close to the MIDDLE of any lane
	// on a DIFFERENT road Y. If so, Y's lane is a through-road that needs splitting.
	// Key: through-lane handle → list of split distances (0.0–1.0 parameter along polyline).
	TMap<int32, TArray<float>> SplitCandidates;

	for (const FEndpointInfo& EP : AllEndpoints)
	{
		for (const auto& Pair : LaneEndpointMap)
		{
			const int32 CandidateLane = Pair.Key;
			// Skip same-road lanes.
			const int32* CandRoad = LaneToRoadHandleMap.Find(CandidateLane);
			if (!CandRoad || *CandRoad == EP.RoadHandle) { continue; }

			const FLaneEndpointCache& CandCache = Pair.Value;
			const TArray<FVector>& Poly = CandCache.Polyline;
			if (Poly.Num() < 3) { continue; }

			// Walk the polyline to find closest point.
			float BestDistSq = SplitRadiusSq;
			int32 BestSegIndex = -1;
			float BestSegT = 0.0f;

			for (int32 s = 0; s < Poly.Num() - 1; ++s)
			{
				const FVector& A = Poly[s];
				const FVector& B = Poly[s + 1];
				const FVector AB = B - A;
				const float SegLenSq = AB.SizeSquared();
				if (SegLenSq < 1.0f) { continue; }

				float T = FVector::DotProduct(EP.Position - A, AB) / SegLenSq;
				T = FMath::Clamp(T, 0.0f, 1.0f);
				const FVector ClosestPt = A + AB * T;
				const float DistSq = FVector::DistSquared(EP.Position, ClosestPt);

				if (DistSq < BestDistSq)
				{
					BestDistSq = DistSq;
					BestSegIndex = s;
					BestSegT = T;
				}
			}

			if (BestSegIndex < 0) { continue; }

			// Calculate the parametric position along the full polyline (0..NumSeg).
			const float ParamPos = static_cast<float>(BestSegIndex) + BestSegT;
			const float TotalSegs = static_cast<float>(Poly.Num() - 1);
			const float NormParam = ParamPos / TotalSegs;

			// Only split if in the middle 10%-90% of the polyline (not near endpoints).
			if (NormParam < 0.10f || NormParam > 0.90f) { continue; }

			SplitCandidates.FindOrAdd(CandidateLane).Add(NormParam);
		}
	}

	// Process each candidate: merge nearby split points, create virtual segments.
	int32 TotalVirtuals = 0;
	TArray<int32> SortedCandidates;
	SplitCandidates.GetKeys(SortedCandidates);
	SortedCandidates.Sort();

	for (const int32 OriginalLane : SortedCandidates)
	{
		TArray<float>& Params = SplitCandidates[OriginalLane];
		Params.Sort();

		// Merge nearby split points (within 5% of polyline length).
		TArray<float> MergedParams;
		for (const float P : Params)
		{
			if (MergedParams.Num() == 0 || (P - MergedParams.Last()) > 0.05f)
			{
				MergedParams.Add(P);
			}
		}

		const FLaneEndpointCache* OrigCache = LaneEndpointMap.Find(OriginalLane);
		if (!OrigCache || OrigCache->Polyline.Num() < 3) { continue; }

		const TArray<FVector>& Poly = OrigCache->Polyline;
		const int32 TotalPoints = Poly.Num();

		// Convert normalized params to point indices.
		TArray<int32> SplitIndices;
		for (const float NP : MergedParams)
		{
			int32 Idx = FMath::RoundToInt32(NP * static_cast<float>(TotalPoints - 1));
			Idx = FMath::Clamp(Idx, 1, TotalPoints - 2); // Never split at very first/last point.
			if (SplitIndices.Num() == 0 || SplitIndices.Last() != Idx)
			{
				SplitIndices.Add(Idx);
			}
		}

		if (SplitIndices.Num() == 0) { continue; }

		// Create N+1 virtual segments.
		TArray<int32> VirtualHandles;
		int32 PrevStart = 0;

		for (int32 s = 0; s <= SplitIndices.Num(); ++s)
		{
			const int32 SegEnd = (s < SplitIndices.Num()) ? SplitIndices[s] : (TotalPoints - 1);

			if (SegEnd <= PrevStart) { continue; }

			const int32 VirtualId = NextHandleId++;
			FVirtualLaneInfo VInfo;
			VInfo.OriginalLaneHandle = OriginalLane;
			VInfo.StartPointIndex = PrevStart;
			VInfo.EndPointIndex = SegEnd;
			VirtualLaneMap.Add(VirtualId, VInfo);
			VirtualHandles.Add(VirtualId);

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

			PrevStart = SegEnd; // Next segment starts where this one ends.
		}

		if (VirtualHandles.Num() > 1)
		{
			ReplacedLaneHandles.Add(OriginalLane);
			OriginalToVirtualMap.Add(OriginalLane, MoveTemp(VirtualHandles));
			TotalVirtuals += OriginalToVirtualMap[OriginalLane].Num();

			// Connect consecutive virtual segments internally.
			const TArray<int32>& Virtuals = OriginalToVirtualMap[OriginalLane];
			for (int32 v = 0; v < Virtuals.Num() - 1; ++v)
			{
				TArray<FTrafficLaneHandle>& Conns = LaneConnectionMap.FindOrAdd(Virtuals[v]);
				Conns.AddUnique(FTrafficLaneHandle(Virtuals[v + 1]));
			}
		}
		else
		{
			// Only one segment — splitting didn't help. Clean up.
			for (const int32 VH : VirtualHandles)
			{
				VirtualLaneMap.Remove(VH);
				LaneEndpointMap.Remove(VH);
			}
		}
	}

	UE_LOG(LogAAATraffic, Log,
		TEXT("RoadBLDReflectionProvider: Through-road splitting created %d virtual segments from %d source lanes."),
		TotalVirtuals, ReplacedLaneHandles.Num());
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

	int32 ProximityLinks = 0;

	for (int32 i = 0; i < WorkingSet.Num(); ++i)
	{
		const int32 HandleA = WorkingSet[i];
		const FLaneEndpointCache* CacheA = LaneEndpointMap.Find(HandleA);
		if (!CacheA) { continue; }

		// Determine road of A.
		int32 RoadA = 0;
		if (const FVirtualLaneInfo* VA = VirtualLaneMap.Find(HandleA))
		{
			const int32* RA = LaneToRoadHandleMap.Find(VA->OriginalLaneHandle);
			RoadA = RA ? *RA : 0;
		}
		else
		{
			const int32* RA = LaneToRoadHandleMap.Find(HandleA);
			RoadA = RA ? *RA : 0;
		}

		for (int32 j = 0; j < WorkingSet.Num(); ++j)
		{
			if (i == j) { continue; }

			const int32 HandleB = WorkingSet[j];
			const FLaneEndpointCache* CacheB = LaneEndpointMap.Find(HandleB);
			if (!CacheB) { continue; }

			// Determine road of B.
			int32 RoadB = 0;
			if (const FVirtualLaneInfo* VB = VirtualLaneMap.Find(HandleB))
			{
				const int32* RB = LaneToRoadHandleMap.Find(VB->OriginalLaneHandle);
				RoadB = RB ? *RB : 0;
			}
			else
			{
				const int32* RB = LaneToRoadHandleMap.Find(HandleB);
				RoadB = RB ? *RB : 0;
			}

			// Only connect lanes on DIFFERENT roads.
			if (RoadA == RoadB && RoadA != 0) { continue; }

			// Check EndPos(A) → StartPos(B) proximity.
			const float DistSq = FVector::DistSquared(CacheA->EndPos, CacheB->StartPos);
			if (DistSq > ProxThresholdSq) { continue; }

			// Direction compatibility: EndDir(A) · StartDir(B).
			const float Dot = FVector::DotProduct(CacheA->EndDir, CacheB->StartDir);

			if (Dot >= DirectionDotMin)
			{
				// Forward or angled connection — always allowed.
				TArray<FTrafficLaneHandle>& Conns = LaneConnectionMap.FindOrAdd(HandleA);
				Conns.AddUnique(FTrafficLaneHandle(HandleB));
				++ProximityLinks;

				FProximityConnection PC;
				PC.FromLane = HandleA;
				PC.ToLane = HandleB;
				PC.Midpoint = (CacheA->EndPos + CacheB->StartPos) * 0.5f;
				ProximityConnectionList.Add(MoveTemp(PC));
			}
			else
			{
				// U-turn candidate — gate by road width.
				// Both lanes must be on roads wide enough for the turn.
				const bool bWidthOk = (CacheA->Width >= MinUTurnWidth) && (CacheB->Width >= MinUTurnWidth);

				if (bWidthOk)
				{
					TArray<FTrafficLaneHandle>& Conns = LaneConnectionMap.FindOrAdd(HandleA);
					Conns.AddUnique(FTrafficLaneHandle(HandleB));
					++ProximityLinks;

					FProximityConnection PC;
					PC.FromLane = HandleA;
					PC.ToLane = HandleB;
					PC.Midpoint = (CacheA->EndPos + CacheB->StartPos) * 0.5f;
					ProximityConnectionList.Add(MoveTemp(PC));
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
}

// ---------------------------------------------------------------------------
// BuildJunctionGrouping — cluster proximity connections into junctions
// ---------------------------------------------------------------------------

void URoadBLDReflectionProvider::BuildJunctionGrouping()
{
	LaneToJunctionMap.Empty();
	JunctionCentroids.Empty();

	if (ProximityConnectionList.Num() == 0)
	{
		UE_LOG(LogAAATraffic, Log,
			TEXT("RoadBLDReflectionProvider: No proximity connections — junction grouping skipped."));
		return;
	}

	constexpr float JunctionGroupThreshold = 200.0f; // cm
	const float ThresholdSq = JunctionGroupThreshold * JunctionGroupThreshold;

	const int32 N = ProximityConnectionList.Num();

	// Union-find for clustering connections by midpoint proximity.
	TArray<int32> Parent;
	Parent.SetNumUninitialized(N);
	for (int32 k = 0; k < N; ++k) { Parent[k] = k; }

	TFunction<int32(int32)> Find = [&Parent, &Find](int32 X) -> int32
	{
		if (Parent[X] != X) { Parent[X] = Find(Parent[X]); }
		return Parent[X];
	};

	auto Union = [&Parent, &Find](int32 A, int32 B)
	{
		int32 RA = Find(A);
		int32 RB = Find(B);
		if (RA != RB) { Parent[RA] = RB; }
	};

	// O(N^2) clustering — N is typically small (number of intersection connections).
	for (int32 a = 0; a < N; ++a)
	{
		for (int32 b = a + 1; b < N; ++b)
		{
			if (FVector::DistSquared(ProximityConnectionList[a].Midpoint,
				ProximityConnectionList[b].Midpoint) <= ThresholdSq)
			{
				Union(a, b);
			}
		}
	}

	// Collect clusters.
	TMap<int32, TArray<int32>> Clusters;
	for (int32 k = 0; k < N; ++k)
	{
		Clusters.FindOrAdd(Find(k)).Add(k);
	}

	// Sort clusters by centroid for deterministic ID assignment.
	struct FClusterEntry
	{
		FVector Centroid;
		TArray<int32> ConnIndices; // Indices into ProximityConnectionList.
	};
	TArray<FClusterEntry> SortedClusters;

	for (auto& ClusterPair : Clusters)
	{
		FClusterEntry Entry;
		FVector Sum = FVector::ZeroVector;
		for (int32 Idx : ClusterPair.Value)
		{
			Sum += ProximityConnectionList[Idx].Midpoint;
		}
		Entry.Centroid = Sum / static_cast<float>(ClusterPair.Value.Num());
		Entry.ConnIndices = MoveTemp(ClusterPair.Value);
		SortedClusters.Add(MoveTemp(Entry));
	}

	SortedClusters.Sort([](const FClusterEntry& A, const FClusterEntry& B)
	{
		if (!FMath::IsNearlyEqual(A.Centroid.X, B.Centroid.X, 1.0f)) return A.Centroid.X < B.Centroid.X;
		if (!FMath::IsNearlyEqual(A.Centroid.Y, B.Centroid.Y, 1.0f)) return A.Centroid.Y < B.Centroid.Y;
		return A.Centroid.Z < B.Centroid.Z;
	});

	// Assign junction IDs (1-based) and map all participating lanes.
	for (int32 JIdx = 0; JIdx < SortedClusters.Num(); ++JIdx)
	{
		const int32 JunctionId = JIdx + 1;
		JunctionCentroids.Add(JunctionId, SortedClusters[JIdx].Centroid);

		for (int32 ConnIdx : SortedClusters[JIdx].ConnIndices)
		{
			const FProximityConnection& PC = ProximityConnectionList[ConnIdx];
			LaneToJunctionMap.FindOrAdd(PC.FromLane) = JunctionId;
			LaneToJunctionMap.FindOrAdd(PC.ToLane) = JunctionId;
		}
	}

	UE_LOG(LogAAATraffic, Log,
		TEXT("RoadBLDReflectionProvider: Grouped %d connections into %d junctions, %d lanes mapped."),
		N, SortedClusters.Num(), LaneToJunctionMap.Num());
}

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
// ---------------------------------------------------------------------------

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

		// Sample reference line at midpoint for direction and position.
		const double MidDist = RoadLength * 0.5;
		const double NearDist = FMath::Max(MidDist - 100.0, 0.0);
		const FVector RefMid = Get3DPositionAtDistance(RefLine, RefLine, MidDist);
		const FVector RefNear = Get3DPositionAtDistance(RefLine, RefLine, NearDist);
		const FVector RefDir = (RefMid - RefNear).GetSafeNormal();
		if (RefDir.IsNearlyZero()) { continue; }

		// Classify each lane as left-of-center or right-of-center.
		struct FSideData { int32 LaneId; float Cross; };
		TArray<FSideData> SideInfo;
		int32 LeftCount = 0, RightCount = 0;

		for (int32 LaneId : RoadPair.Value)
		{
			TArray<FVector> Points;
			float Width;
			if (!GetLanePath(FTrafficLaneHandle(LaneId), Points, Width) || Points.Num() < 2) { continue; }
			const FVector LaneMid = Points[Points.Num() / 2];
			const float Cross = FVector::CrossProduct(RefDir, (LaneMid - RefMid)).Z;
			SideInfo.Add({ LaneId, Cross });
			if (Cross > 50.0f) { ++LeftCount; }
			else if (Cross < -50.0f) { ++RightCount; }
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

bool URoadBLDReflectionProvider::IsLaneReversed(const FTrafficLaneHandle& Lane)
{
	// Virtual lane — check the original.
	int32 EffectiveId = Lane.HandleId;
	if (const FVirtualLaneInfo* VInfo = VirtualLaneMap.Find(Lane.HandleId))
	{
		EffectiveId = VInfo->OriginalLaneHandle;
	}
	return ReversedLaneSet.Contains(EffectiveId);
}

// ---------------------------------------------------------------------------
// ITrafficRoadProvider — junction queries
// ---------------------------------------------------------------------------

int32 URoadBLDReflectionProvider::GetJunctionForLane(const FTrafficLaneHandle& Lane)
{
	// Check the handle directly first (works for both virtual and original).
	if (const int32* JId = LaneToJunctionMap.Find(Lane.HandleId))
	{
		return *JId;
	}
	// Virtual lane — also check via original handle.
	if (const FVirtualLaneInfo* VInfo = VirtualLaneMap.Find(Lane.HandleId))
	{
		if (const int32* JId = LaneToJunctionMap.Find(VInfo->OriginalLaneHandle))
		{
			return *JId;
		}
	}
	return 0;
}

bool URoadBLDReflectionProvider::GetJunctionPath(
	const FTrafficLaneHandle& FromLane,
	const FTrafficLaneHandle& ToLane,
	TArray<FVector>& OutPath)
{
	// Resolve junction for FromLane (virtual-aware).
	const int32 FromJunctionId = GetJunctionForLane(FromLane);
	if (FromJunctionId == 0)
	{
		return false;
	}

	// Validate that ToLane maps to the same junction.
	const int32 ToJunctionId = GetJunctionForLane(ToLane);
	if (ToJunctionId != FromJunctionId)
	{
		return false;
	}

	const FVector* Centroid = JunctionCentroids.Find(FromJunctionId);
	if (!Centroid)
	{
		return false;
	}

	// Build a 3-point path: FromLane endpoint → junction centroid → ToLane startpoint.
	TArray<FVector> FromPoints;
	float FromWidth;
	TArray<FVector> ToPoints;
	float ToWidth;

	if (!GetLanePath(FromLane, FromPoints, FromWidth) || FromPoints.Num() == 0)
	{
		return false;
	}
	if (!GetLanePath(ToLane, ToPoints, ToWidth) || ToPoints.Num() == 0)
	{
		return false;
	}

	OutPath.Reset(3);
	OutPath.Add(FromPoints.Last());
	OutPath.Add(*Centroid);
	OutPath.Add(ToPoints[0]);
	return true;
}

// ---------------------------------------------------------------------------
// ITrafficRoadProvider — adjacency & speed limit
// ---------------------------------------------------------------------------

FTrafficLaneHandle URoadBLDReflectionProvider::GetAdjacentLane(
	const FTrafficLaneHandle& Lane, ETrafficLaneSide Side)
{
	// Virtual lane — find the corresponding virtual segment of the neighbor.
	int32 EffectiveId = Lane.HandleId;
	int32 SegmentIndex = -1;
	if (const FVirtualLaneInfo* VInfo = VirtualLaneMap.Find(Lane.HandleId))
	{
		EffectiveId = VInfo->OriginalLaneHandle;
		// Determine which segment index this virtual lane is.
		if (const TArray<int32>* Virtuals = OriginalToVirtualMap.Find(EffectiveId))
		{
			for (int32 i = 0; i < Virtuals->Num(); ++i)
			{
				if ((*Virtuals)[i] == Lane.HandleId) { SegmentIndex = i; break; }
			}
		}
	}

	const TMap<int32, int32>& Map = (Side == ETrafficLaneSide::Left) ? LeftNeighborMap : RightNeighborMap;
	if (const int32* NeighborId = Map.Find(EffectiveId))
	{
		// If the neighbor was also split, return the same-indexed virtual segment.
		if (SegmentIndex >= 0)
		{
			if (const TArray<int32>* NVirtuals = OriginalToVirtualMap.Find(*NeighborId))
			{
				if (SegmentIndex < NVirtuals->Num())
				{
					return FTrafficLaneHandle((*NVirtuals)[SegmentIndex]);
				}
			}
		}
		// Neighbor not split — return as-is.
		return FTrafficLaneHandle(*NeighborId);
	}
	return FTrafficLaneHandle();
}

FTrafficRoadHandle URoadBLDReflectionProvider::GetRoadForLane(const FTrafficLaneHandle& Lane)
{
	// Virtual lane — resolve to original lane's road.
	int32 EffectiveId = Lane.HandleId;
	if (const FVirtualLaneInfo* VInfo = VirtualLaneMap.Find(Lane.HandleId))
	{
		EffectiveId = VInfo->OriginalLaneHandle;
	}
	if (const int32* RoadId = LaneToRoadHandleMap.Find(EffectiveId))
	{
		return FTrafficRoadHandle(*RoadId);
	}
	return FTrafficRoadHandle();
}

float URoadBLDReflectionProvider::GetLaneSpeedLimit(const FTrafficLaneHandle& /*Lane*/)
{
	// RoadBLD does not expose per-lane speed limits. Return -1 so the caller
	// knows to fall back to its own default speed.
	return -1.0f;
}

// ---------------------------------------------------------------------------
// Reflection helpers
// ---------------------------------------------------------------------------

void URoadBLDReflectionProvider::CallReflection(UObject* Target, UFunction* Func, void* Params)
{
	check(Target && Func);
	Target->ProcessEvent(Func, Params);
}

double URoadBLDReflectionProvider::GetRoadLength(UObject* RoadActor) const
{
	if (!RoadActor || !GetLengthFunc) { return 0.0; }

	struct { double ReturnValue; } Params;
	Params.ReturnValue = 0.0;
	RoadActor->ProcessEvent(GetLengthFunc, &Params);
	return Params.ReturnValue;
}

TArray<UObject*> URoadBLDReflectionProvider::GetAllLanesForRoad(UObject* RoadActor) const
{
	if (!RoadActor || !GetAllLanesFunc) { return {}; }

	struct { TArray<UObject*> ReturnValue; } Params;
	RoadActor->ProcessEvent(GetAllLanesFunc, &Params);
	return MoveTemp(Params.ReturnValue);
}

UObject* URoadBLDReflectionProvider::GetReferenceLine(UObject* RoadActor) const
{
	if (!RoadActor || !RefLineProp) { return nullptr; }

	FObjectPropertyBase* ObjProp = CastField<FObjectPropertyBase>(RefLineProp);
	if (!ObjProp) { return nullptr; }

	return ObjProp->GetObjectPropertyValue_InContainer(RoadActor);
}

double URoadBLDReflectionProvider::ConvertDistanceBetweenCurves(
	UObject* RoadActor, UObject* From, UObject* To, double Distance) const
{
	if (!RoadActor || !ConvertDistFunc) { return Distance; }

	uint8* Params = static_cast<uint8*>(FMemory_Alloca(ConvertDistFunc->ParmsSize));
	FMemory::Memzero(Params, ConvertDistFunc->ParmsSize);

	int32 ParamIdx = 0;
	for (TFieldIterator<FProperty> PIt(ConvertDistFunc); PIt && (PIt->PropertyFlags & CPF_Parm); ++PIt)
	{
		if (PIt->HasAnyPropertyFlags(CPF_ReturnParm)) { continue; }
		if (ParamIdx == 0)      { *PIt->ContainerPtrToValuePtr<UObject*>(Params) = From; }
		else if (ParamIdx == 1) { *PIt->ContainerPtrToValuePtr<UObject*>(Params) = To; }
		else if (ParamIdx == 2) { *PIt->ContainerPtrToValuePtr<double>(Params) = Distance; }
		++ParamIdx;
	}

	RoadActor->ProcessEvent(ConvertDistFunc, Params);

	if (FProperty* RetProp = ConvertDistFunc->GetReturnProperty())
	{
		return *RetProp->ContainerPtrToValuePtr<double>(Params);
	}
	return Distance;
}

FVector URoadBLDReflectionProvider::Get3DPositionAtDistance(
	UObject* CurveObj, UObject* RefLine, double Distance) const
{
	if (!CurveObj || !Get3DPosFunc) { return FVector::ZeroVector; }

	uint8* Params = static_cast<uint8*>(FMemory_Alloca(Get3DPosFunc->ParmsSize));
	FMemory::Memzero(Params, Get3DPosFunc->ParmsSize);

	int32 PIdx = 0;
	for (TFieldIterator<FProperty> PIt(Get3DPosFunc); PIt && (PIt->PropertyFlags & CPF_Parm); ++PIt)
	{
		if (PIt->HasAnyPropertyFlags(CPF_ReturnParm)) { continue; }
		if (PIdx == 0)      { *PIt->ContainerPtrToValuePtr<UObject*>(Params) = RefLine; }
		else if (PIdx == 1) { *PIt->ContainerPtrToValuePtr<double>(Params) = Distance; }
		++PIdx;
	}

	CurveObj->ProcessEvent(Get3DPosFunc, Params);

	if (FProperty* RetProp = Get3DPosFunc->GetReturnProperty())
	{
		return *RetProp->ContainerPtrToValuePtr<FVector>(Params);
	}
	return FVector::ZeroVector;
}

FVector2D URoadBLDReflectionProvider::GetWorldPositionAtDistance(
	UObject* RoadActor, double Distance) const
{
	if (!RoadActor || !GetWorldPosFunc) { return FVector2D::ZeroVector; }

	struct { double Distance; FVector2D ReturnValue; } Params;
	Params.Distance = Distance;
	Params.ReturnValue = FVector2D::ZeroVector;
	RoadActor->ProcessEvent(GetWorldPosFunc, &Params);
	return Params.ReturnValue;
}
