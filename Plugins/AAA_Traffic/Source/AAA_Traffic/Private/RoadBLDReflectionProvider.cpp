// Copyright AAA_Traffic Contributors. All Rights Reserved.

#include "RoadBLDReflectionProvider.h"
#include "RoadBLDAPIContract.h"
#include "TrafficSubsystem.h"
#include "TrafficLog.h"
#include "Engine/World.h"
#include "EngineUtils.h"
#include "UObject/FieldPathProperty.h"

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

	// ── Build lane connectivity ──────────────────────────────────
	BuildLaneConnectivity(&InWorld);

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

	Result.Reserve(LaneIds->Num());
	for (const int32 LaneId : *LaneIds)
	{
		Result.Emplace(LaneId);
	}
	return Result;
}

bool URoadBLDReflectionProvider::GetLanePath(
	const FTrafficLaneHandle& Lane,
	TArray<FVector>& OutPoints,
	float& OutWidth)
{
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

	return OutPoints.Num() >= 2;
}

FVector URoadBLDReflectionProvider::GetLaneDirection(const FTrafficLaneHandle& Lane)
{
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
	// Find the lane whose centerline midpoint is closest to the location.
	// This is a simple brute-force approach — adequate for the lane counts
	// typical of RoadBLD scenes (tens to low hundreds of lanes).

	float BestDistSq = MAX_flt;
	int32 BestLaneHandle = 0;

	// Sort keys for deterministic iteration (System.md §4.4).
	TArray<int32> LaneKeys;
	LaneHandleMap.GetKeys(LaneKeys);
	LaneKeys.Sort();

	for (const int32 LaneId : LaneKeys)
	{
		const FReflectionLaneData& Data = LaneHandleMap[LaneId];
		UObject* RoadActor = Data.RoadActor.Get();
		if (!RoadActor) { continue; }

		const double RoadLength = GetRoadLength(RoadActor);
		if (RoadLength <= 0.0) { continue; }

		// Sample at midpoint and quarter-points for reasonable coverage.
		UObject* RefLine = GetReferenceLine(RoadActor);
		UObject* LeftEdge = Data.LeftEdge.Get();
		UObject* RightEdge = Data.RightEdge.Get();

		const bool bCanEdgeSample = LeftEdge && RightEdge && RefLine && Get3DPosFunc && ConvertDistFunc;

		static const double SampleFractions[] = { 0.0, 0.25, 0.5, 0.75, 1.0 };
		for (double Frac : SampleFractions)
		{
			const double Dist = RoadLength * Frac;
			FVector SamplePos;

			if (bCanEdgeSample)
			{
				const double LD = ConvertDistanceBetweenCurves(RoadActor, RefLine, LeftEdge, Dist);
				const double RD = ConvertDistanceBetweenCurves(RoadActor, RefLine, RightEdge, Dist);
				const FVector LP = Get3DPositionAtDistance(LeftEdge, RefLine, LD);
				const FVector RP = Get3DPositionAtDistance(RightEdge, RefLine, RD);
				SamplePos = (LP + RP) * 0.5;
			}
			else
			{
				const FVector2D Pos2D = GetWorldPositionAtDistance(RoadActor, Dist);
				SamplePos = FVector(Pos2D.X, Pos2D.Y, 0.0);
			}

			const float DistSq = FVector::DistSquared(Location, SamplePos);
			if (DistSq < BestDistSq)
			{
				BestDistSq = DistSq;
				BestLaneHandle = LaneId;
			}
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

	for (AActor* RoadActor : RoadActors)
	{
		const double RoadLength = GetRoadLength(RoadActor);
		if (RoadLength <= 0.0) { continue; }

		const int32 RoadId = NextHandleId++;
		RoadHandleMap.Add(RoadId, RoadActor);

		TArray<UObject*> Lanes = GetAllLanesForRoad(RoadActor);
		// Sort lanes by name for deterministic handle ordering.
		Lanes.Sort([](const UObject& A, const UObject& B)
		{
			return A.GetName() < B.GetName();
		});

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
// BuildLaneConnectivity — discover connections from RoadNetworkCorners
// ---------------------------------------------------------------------------

void URoadBLDReflectionProvider::BuildLaneConnectivity(UWorld* World)
{
	LaneConnectionMap.Empty();

	if (!DynNetworkClass)
	{
		UE_LOG(LogAAATraffic, Warning,
			TEXT("RoadBLDReflectionProvider: DynamicRoadNetwork class not found — connectivity unavailable."));
		return;
	}

	// Find the road network actor.
	AActor* NetworkActor = nullptr;
	for (FActorIterator It(World); It; ++It)
	{
		if (It->IsA(DynNetworkClass))
		{
			NetworkActor = *It;
			break;
		}
	}
	if (!NetworkActor)
	{
		UE_LOG(LogAAATraffic, Warning,
			TEXT("RoadBLDReflectionProvider: No DynamicRoadNetwork actor found — connectivity unavailable."));
		return;
	}

	// ── Access RoadNetworkCorners TArray via reflection ──────────
	FProperty* CornersProp = DynNetworkClass->FindPropertyByName(TEXT("RoadNetworkCorners"));
	if (!CornersProp)
	{
		UE_LOG(LogAAATraffic, Warning,
			TEXT("RoadBLDReflectionProvider: RoadNetworkCorners property not found."));
		return;
	}

	FArrayProperty* CornerArrayProp = CastField<FArrayProperty>(CornersProp);
	if (!CornerArrayProp)
	{
		UE_LOG(LogAAATraffic, Warning,
			TEXT("RoadBLDReflectionProvider: RoadNetworkCorners is not an array property."));
		return;
	}

	FScriptArrayHelper ArrayHelper(CornerArrayProp, CornerArrayProp->ContainerPtrToValuePtr<void>(NetworkActor));
	const int32 NumCorners = ArrayHelper.Num();

	if (NumCorners == 0)
	{
		UE_LOG(LogAAATraffic, Log,
			TEXT("RoadBLDReflectionProvider: RoadNetworkCorners is empty — no connectivity."));
		return;
	}

	// ── Resolve FRoadNetworkCorner struct properties ─────────────
	FStructProperty* StructProp = CastField<FStructProperty>(CornerArrayProp->Inner);
	if (!StructProp)
	{
		UE_LOG(LogAAATraffic, Warning,
			TEXT("RoadBLDReflectionProvider: Cannot resolve FRoadNetworkCorner struct."));
		return;
	}

	UScriptStruct* CornerStruct = StructProp->Struct;
	FProperty* StartEdgeProp = CornerStruct->FindPropertyByName(TEXT("StartEdge"));
	FProperty* EndEdgeProp   = CornerStruct->FindPropertyByName(TEXT("EndEdge"));
	FProperty* bStaleProp    = CornerStruct->FindPropertyByName(TEXT("bStale"));

	if (!StartEdgeProp || !EndEdgeProp)
	{
		UE_LOG(LogAAATraffic, Warning,
			TEXT("RoadBLDReflectionProvider: FRoadNetworkCorner missing StartEdge/EndEdge."));
		return;
	}

	FObjectPropertyBase* StartEdgeObjProp = CastField<FObjectPropertyBase>(StartEdgeProp);
	FObjectPropertyBase* EndEdgeObjProp   = CastField<FObjectPropertyBase>(EndEdgeProp);
	FBoolProperty*       bStaleNative     = CastField<FBoolProperty>(bStaleProp);

	if (!StartEdgeObjProp || !EndEdgeObjProp)
	{
		UE_LOG(LogAAATraffic, Warning,
			TEXT("RoadBLDReflectionProvider: StartEdge/EndEdge are not object properties."));
		return;
	}

	// ── Resolve edge-curve lane properties (LeftLane, RightLane) ─
	// These are on UEdgeCurve; we resolve from the first non-null StartEdge.
	FProperty* EdgeLeftLaneProp  = nullptr;
	FProperty* EdgeRightLaneProp = nullptr;

	for (int32 i = 0; i < NumCorners; ++i)
	{
		const uint8* ElemPtr = ArrayHelper.GetRawPtr(i);
		UObject* StartEdge = StartEdgeObjProp->GetObjectPropertyValue_InContainer(ElemPtr);
		if (StartEdge)
		{
			UClass* EdgeClass = StartEdge->GetClass();
			EdgeLeftLaneProp  = EdgeClass->FindPropertyByName(TEXT("LeftLane"));
			EdgeRightLaneProp = EdgeClass->FindPropertyByName(TEXT("RightLane"));
			break;
		}
	}

	if (!EdgeLeftLaneProp || !EdgeRightLaneProp)
	{
		UE_LOG(LogAAATraffic, Warning,
			TEXT("RoadBLDReflectionProvider: Could not resolve LeftLane/RightLane on edge curve class."));
		return;
	}

	FObjectPropertyBase* EdgeLeftLaneObj  = CastField<FObjectPropertyBase>(EdgeLeftLaneProp);
	FObjectPropertyBase* EdgeRightLaneObj = CastField<FObjectPropertyBase>(EdgeRightLaneProp);

	if (!EdgeLeftLaneObj || !EdgeRightLaneObj)
	{
		UE_LOG(LogAAATraffic, Warning,
			TEXT("RoadBLDReflectionProvider: LeftLane/RightLane are not object properties."));
		return;
	}

	// ── Helper: collect lane handles adjacent to an edge curve ───
	auto CollectLaneHandles = [&](UObject* Edge, TArray<int32>& OutHandles)
	{
		if (!Edge) { return; }

		UObject* LeftLane  = EdgeLeftLaneObj->GetObjectPropertyValue_InContainer(Edge);
		UObject* RightLane = EdgeRightLaneObj->GetObjectPropertyValue_InContainer(Edge);

		if (LeftLane)
		{
			if (const int32* Handle = LaneToHandleMap.Find(LeftLane))
			{
				OutHandles.AddUnique(*Handle);
			}
		}
		if (RightLane)
		{
			if (const int32* Handle = LaneToHandleMap.Find(RightLane))
			{
				OutHandles.AddUnique(*Handle);
			}
		}
	};

	// ── Process each corner ──────────────────────────────────────
	for (int32 i = 0; i < NumCorners; ++i)
	{
		const uint8* ElemPtr = ArrayHelper.GetRawPtr(i);

		// Check bStale.
		if (bStaleNative)
		{
			const bool bStale = bStaleNative->GetPropertyValue_InContainer(ElemPtr);
			if (bStale) { continue; }
		}

		UObject* StartEdge = StartEdgeObjProp->GetObjectPropertyValue_InContainer(ElemPtr);
		UObject* EndEdge   = EndEdgeObjProp->GetObjectPropertyValue_InContainer(ElemPtr);

		if (!StartEdge || !EndEdge) { continue; }

		TArray<int32> StartLaneHandles;
		TArray<int32> EndLaneHandles;
		CollectLaneHandles(StartEdge, StartLaneHandles);
		CollectLaneHandles(EndEdge, EndLaneHandles);

		// Connect: StartEdge lanes → EndEdge lanes (directional).
		for (const int32 SrcHandle : StartLaneHandles)
		{
			TArray<FTrafficLaneHandle>& Connections = LaneConnectionMap.FindOrAdd(SrcHandle);
			for (const int32 DstHandle : EndLaneHandles)
			{
				if (SrcHandle == DstHandle) { continue; }
				Connections.AddUnique(FTrafficLaneHandle(DstHandle));
			}
		}
	}

	// Sort each connection list by HandleId for deterministic selection.
	for (auto& Pair : LaneConnectionMap)
	{
		Pair.Value.Sort([](const FTrafficLaneHandle& A, const FTrafficLaneHandle& B)
		{
			return A.HandleId < B.HandleId;
		});
	}

	int32 ConnectedLaneCount = 0;
	for (const auto& Pair : LaneConnectionMap)
	{
		ConnectedLaneCount += Pair.Value.Num();
	}
	UE_LOG(LogAAATraffic, Log,
		TEXT("RoadBLDReflectionProvider: Built lane connectivity — %d lanes with connections, %d total links."),
		LaneConnectionMap.Num(), ConnectedLaneCount);
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
