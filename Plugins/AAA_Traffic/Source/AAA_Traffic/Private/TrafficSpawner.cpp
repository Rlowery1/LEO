// Copyright AAA_Traffic Contributors. All Rights Reserved.

#include "TrafficSpawner.h"
#include "TrafficSubsystem.h"
#include "TrafficRoadProvider.h"
#include "RoadBLDReflectionProvider.h"
#include "TrafficVehicleController.h"
#include "TrafficSignalController.h"
#include "TrafficVehiclePool.h"
#include "TrafficLog.h"
#include "ChaosWheeledVehicleMovementComponent.h"
#include "Engine/World.h"
#include "TimerManager.h"
#include "EngineUtils.h"
#include "GameFramework/PlayerController.h"
#include "GameFramework/Pawn.h"

// Global debug draw CVar — declared in TrafficVehicleController.cpp.
extern int32 GTrafficDebugDraw;

// Junction diagnostics CVar — declared in TrafficVehicleController.cpp.
extern int32 GTrafficJunctionDiagnostics;

ATrafficSpawner::ATrafficSpawner()
	: VehicleCount(1)
	, VehicleSpeed(2012.f)
	, SpawnSeed(42)
	, SpawnZOffset(50.f)
	, SpawnSpacing(1500.f)
	, SpeedVariation(15.f)
	, LaneChangeAggression(0.5f)
	, bEnableRespawn(true)
	, RespawnCheckInterval(2.0f)
	, MinRespawnDistance(10000.f)
	, DefaultSpeedLimit(2012.0f)
	, ResidentialSpeed(1118.0f)
	, UrbanSpeed(2012.0f)
	, HighwaySpeed(2906.0f)
{
#if defined(ENABLE_DRAW_DEBUG) && ENABLE_DRAW_DEBUG
	PrimaryActorTick.bCanEverTick = true;
	PrimaryActorTick.bStartWithTickEnabled = true; // Tick in editor for debug lane draw
#else
	PrimaryActorTick.bCanEverTick = false;
#endif
}

void ATrafficSpawner::BeginPlay()
{
	Super::BeginPlay();

	// Defer to next tick so adapter subsystems have finished OnWorldBeginPlay registration.
	GetWorldTimerManager().SetTimerForNextTick(this, &ATrafficSpawner::SpawnVehicles);

#if defined(ENABLE_DRAW_DEBUG) && ENABLE_DRAW_DEBUG
	if (bDebugDrawLanes || bDebugDrawIntersections)
	{
		SetActorTickEnabled(true);
	}
#endif
}

bool ATrafficSpawner::ShouldTickIfViewportsOnly() const
{
#if defined(ENABLE_DRAW_DEBUG) && ENABLE_DRAW_DEBUG
	return bDebugDrawLanes || bDebugDrawIntersections;
#else
	return false;
#endif
}

void ATrafficSpawner::Tick(float DeltaSeconds)
{
	Super::Tick(DeltaSeconds);

#if defined(ENABLE_DRAW_DEBUG) && ENABLE_DRAW_DEBUG
	if (!bDebugDrawLanes && !bDebugDrawIntersections)
	{
		return;
	}

	const bool bDrawLanes = bDebugDrawLanes || (GTrafficDebugDraw != 0);
	const bool bDrawIntersections = bDebugDrawIntersections || (GTrafficDebugDraw != 0);

	if (bDrawLanes && !bDebugCacheReady && !bDebugCacheAttempted)
	{
		CacheDebugLaneData();
	}

	if (bDrawIntersections && !bIntersectionCacheReady && !bIntersectionCacheAttempted)
	{
		CacheDebugIntersectionData();
	}

	if (bDrawLanes && bDebugCacheReady)
	{
		DrawDebugLanes();
	}

	if (bDrawIntersections && bIntersectionCacheReady)
	{
		DrawDebugIntersections();
	}
#endif
}

void ATrafficSpawner::EndPlay(const EEndPlayReason::Type EndPlayReason)
{
	// Unsubscribe from despawn delegate to prevent dangling callbacks.
	if (UWorld* World = GetWorld())
	{
		if (UTrafficSubsystem* TrafficSub = World->GetSubsystem<UTrafficSubsystem>())
		{
			TrafficSub->OnVehicleDespawned.RemoveAll(this);
			TrafficSub->OnProviderRegistered.RemoveAll(this);
		}
		World->GetTimerManager().ClearTimer(RespawnTimerHandle);
	}

	OwnedVehicles.Empty();

	Super::EndPlay(EndPlayReason);
}

#if defined(ENABLE_DRAW_DEBUG) && ENABLE_DRAW_DEBUG
void ATrafficSpawner::CacheDebugLaneData()
{
	UWorld* World = GetWorld();
	if (!World) { return; }

	UTrafficSubsystem* TrafficSub = World->GetSubsystem<UTrafficSubsystem>();
	ITrafficRoadProvider* Provider = TrafficSub ? TrafficSub->GetProvider() : nullptr;

	if (Provider)
	{
		TArray<FTrafficRoadHandle> Roads = Provider->GetAllRoads();
		for (const FTrafficRoadHandle& Road : Roads)
		{
			TArray<FTrafficLaneHandle> Lanes = Provider->GetLanesForRoad(Road);
			for (const FTrafficLaneHandle& Lane : Lanes)
			{
				TArray<FVector> Points;
				float Width;
				if (!Provider->GetLanePath(Lane, Points, Width) || Points.Num() < 2)
				{
					continue;
				}

				const bool bReverse = Provider->IsLaneReversed(Lane);

				FDebugLaneData& Entry = DebugLanes.AddDefaulted_GetRef();
				Entry.Points = MoveTemp(Points);
				Entry.bIsReverseLane = bReverse;
			}
		}
	}

	// Fallback: discover road geometry via UE reflection (works with precompiled plugins).
	if (DebugLanes.IsEmpty())
	{
		CacheDebugLaneDataViaReflection(World);
	}

	bDebugCacheAttempted = true;
	bDebugCacheReady = DebugLanes.Num() > 0;

	if (bDebugCacheReady)
	{
		UE_LOG(LogAAATraffic, Log,
			TEXT("TrafficSpawner: Cached %d lane polylines for debug draw."),
			DebugLanes.Num());
	}
	else
	{
		UE_LOG(LogAAATraffic, Warning,
			TEXT("TrafficSpawner: Debug draw enabled but no lane data available (no provider, no reflectable roads)."));
	}
}

// ---------------------------------------------------------------------------
// Reflection-based lane discovery — no compile-time RoadBLD dependency.
//
// This allows debug-draw to work when RoadBLD ships as a precompiled binary
// (no source headers on disk).  All calls go through UE's UFUNCTION reflection
// so the only requirement is that the RoadBLD plugin is loaded at runtime.
// ---------------------------------------------------------------------------
void ATrafficSpawner::CacheDebugLaneDataViaReflection(UWorld* World)
{
	// --- Discover RoadBLD classes ------------------------------------------
	UClass* DynRoadClass = FindObject<UClass>(nullptr, TEXT("/Script/RoadBLDRuntime.DynamicRoad"));
	if (!DynRoadClass)
	{
		UE_LOG(LogAAATraffic, Verbose,
			TEXT("TrafficSpawner: RoadBLDRuntime.DynamicRoad class not found — reflection fallback skipped."));
		return;
	}

	// --- Cache UFunction / FProperty pointers once -------------------------
	UFunction* GetLengthFunc    = DynRoadClass->FindFunctionByName(TEXT("GetLength"));
	UFunction* GetAllLanesFunc  = DynRoadClass->FindFunctionByName(TEXT("GetAllLanes"));
	UFunction* ConvertDistFunc  = DynRoadClass->FindFunctionByName(TEXT("ConvertDistanceBetweenCurves"));
	FProperty* RefLineProp      = DynRoadClass->FindPropertyByName(TEXT("ReferenceLine"));

	if (!GetLengthFunc || !GetAllLanesFunc)
	{
		UE_LOG(LogAAATraffic, Warning,
			TEXT("TrafficSpawner: DynamicRoad reflection — missing GetLength or GetAllLanes."));
		return;
	}

	// Lane class properties (resolved lazily on first lane found).
	FProperty* LeftEdgeProp  = nullptr;
	FProperty* RightEdgeProp = nullptr;
	FProperty* LaneWidthProp = nullptr;
	UFunction* Get3DPosFunc  = nullptr;  // UCurveObject::Get3DPositionAtDistance

	// --- Iterate all DynamicRoad actors ------------------------------------
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
		// --- GetLength() → double ------------------------------------------
		struct { double ReturnValue = 0.0; } LengthParams;
		RoadActor->ProcessEvent(GetLengthFunc, &LengthParams);
		const double RoadLength = LengthParams.ReturnValue;
		if (RoadLength <= 0.0) { continue; }

		// --- GetAllLanes() → TArray<UObject*> ------------------------------
		struct { TArray<UObject*> ReturnValue; } LanesParams;
		RoadActor->ProcessEvent(GetAllLanesFunc, &LanesParams);
		TArray<UObject*>& Lanes = LanesParams.ReturnValue;

		// Sort lanes by name for deterministic handle ordering.
		Lanes.Sort([](const UObject& A, const UObject& B)
		{
			return A.GetName() < B.GetName();
		});

		// --- Resolve ReferenceLine property on this road -------------------
		UObject* RefLine = nullptr;
		if (RefLineProp)
		{
			FObjectPropertyBase* ObjProp = CastField<FObjectPropertyBase>(RefLineProp);
			if (ObjProp)
			{
				RefLine = ObjProp->GetObjectPropertyValue_InContainer(RoadActor);
			}
		}

		// --- Resolve lane-class properties once ----------------------------
		if (Lanes.Num() > 0 && !LeftEdgeProp && Lanes[0])
		{
			UClass* LaneClass = Lanes[0]->GetClass();
			LeftEdgeProp  = LaneClass->FindPropertyByName(TEXT("LeftEdgeCurve"));
			RightEdgeProp = LaneClass->FindPropertyByName(TEXT("RightEdgeCurve"));
			LaneWidthProp = LaneClass->FindPropertyByName(TEXT("LaneWidth"));

			// Get3DPositionAtDistance lives on UCurveObject (parent of UEdgeCurve).
			if (LeftEdgeProp)
			{
				FObjectPropertyBase* ObjProp = CastField<FObjectPropertyBase>(LeftEdgeProp);
				if (ObjProp)
				{
					// Resolve from any non-null edge curve instance.
					for (UObject* L : Lanes)
					{
						if (!L) { continue; }
						UObject* Edge = ObjProp->GetObjectPropertyValue_InContainer(L);
						if (Edge)
						{
							Get3DPosFunc = Edge->GetClass()->FindFunctionByName(TEXT("Get3DPositionAtDistance"));
							break;
						}
					}
				}
			}
		}

		// --- Per-lane edge-curve sampling ----------------------------------
		const bool bCanDoPerLane = LeftEdgeProp && RightEdgeProp && RefLine
			&& Get3DPosFunc && ConvertDistFunc;

		for (UObject* LaneObj : Lanes)
		{
			if (!LaneObj) { continue; }

			const double SampleInterval = 100.0; // 1 m
			const int32 NumSamples = FMath::Max(2, FMath::CeilToInt(RoadLength / SampleInterval) + 1);

			FDebugLaneData Entry;
			Entry.bIsReverseLane = false;
			Entry.Points.Reserve(NumSamples);

			if (bCanDoPerLane)
			{
				FObjectPropertyBase* LeftObjProp  = CastField<FObjectPropertyBase>(LeftEdgeProp);
				FObjectPropertyBase* RightObjProp = CastField<FObjectPropertyBase>(RightEdgeProp);

				UObject* LeftEdge  = LeftObjProp  ? LeftObjProp->GetObjectPropertyValue_InContainer(LaneObj)  : nullptr;
				UObject* RightEdge = RightObjProp ? RightObjProp->GetObjectPropertyValue_InContainer(LaneObj) : nullptr;

				if (LeftEdge && RightEdge)
				{
					for (int32 i = 0; i < NumSamples; ++i)
					{
						const double Dist = FMath::Min(RoadLength,
							(RoadLength * static_cast<double>(i)) / static_cast<double>(NumSamples - 1));

						// ConvertDistanceBetweenCurves(From, To, Distance) → double
						auto ConvertDist = [&](UObject* ToEdge) -> double
						{
							uint8* Params = static_cast<uint8*>(FMemory_Alloca(ConvertDistFunc->ParmsSize));
							FMemory::Memzero(Params, ConvertDistFunc->ParmsSize);

							// Set parameters by iterating the function's property chain.
							int32 ParamIdx = 0;
							double Result = Dist;
							for (TFieldIterator<FProperty> PIt(ConvertDistFunc); PIt && (PIt->PropertyFlags & CPF_Parm); ++PIt)
							{
								if (PIt->HasAnyPropertyFlags(CPF_ReturnParm))
								{
									continue;
								}
								if (ParamIdx == 0) // From (UCurveObject*)
								{
									*PIt->ContainerPtrToValuePtr<UObject*>(Params) = RefLine;
								}
								else if (ParamIdx == 1) // To (UCurveObject*)
								{
									*PIt->ContainerPtrToValuePtr<UObject*>(Params) = ToEdge;
								}
								else if (ParamIdx == 2) // Distance (double)
								{
									*PIt->ContainerPtrToValuePtr<double>(Params) = Dist;
								}
								++ParamIdx;
							}

							RoadActor->ProcessEvent(ConvertDistFunc, Params);

							// Read return value.
							if (FProperty* RetProp = ConvertDistFunc->GetReturnProperty())
							{
								Result = *RetProp->ContainerPtrToValuePtr<double>(Params);
							}
							return Result;
						};

						const double LeftDist  = ConvertDist(LeftEdge);
						const double RightDist = ConvertDist(RightEdge);

						// Get3DPositionAtDistance(ReferenceLine, Distance) → FVector
						auto GetPos3D = [&](UObject* Curve, double CurveDist) -> FVector
						{
							uint8* Params = static_cast<uint8*>(FMemory_Alloca(Get3DPosFunc->ParmsSize));
							FMemory::Memzero(Params, Get3DPosFunc->ParmsSize);

							int32 PIdx = 0;
							FVector Result = FVector::ZeroVector;
							for (TFieldIterator<FProperty> PIt(Get3DPosFunc); PIt && (PIt->PropertyFlags & CPF_Parm); ++PIt)
							{
								if (PIt->HasAnyPropertyFlags(CPF_ReturnParm)) { continue; }
								if (PIdx == 0) // ReferenceLine (UCurveObject*)
								{
									*PIt->ContainerPtrToValuePtr<UObject*>(Params) = RefLine;
								}
								else if (PIdx == 1) // Distance (double)
								{
									*PIt->ContainerPtrToValuePtr<double>(Params) = CurveDist;
								}
								++PIdx;
							}

							Curve->ProcessEvent(Get3DPosFunc, Params);

							if (FProperty* RetProp = Get3DPosFunc->GetReturnProperty())
							{
								Result = *RetProp->ContainerPtrToValuePtr<FVector>(Params);
							}
							return Result;
						};

						const FVector LeftPos  = GetPos3D(LeftEdge, LeftDist);
						const FVector RightPos = GetPos3D(RightEdge, RightDist);
						Entry.Points.Add((LeftPos + RightPos) * 0.5);
					}
				}
			}

			// Fallback: road centerline via GetWorldPositionAtDistance.
			if (Entry.Points.IsEmpty())
			{
				UFunction* GetPosFunc = DynRoadClass->FindFunctionByName(TEXT("GetWorldPositionAtDistance"));
				if (!GetPosFunc) { continue; }

				for (int32 i = 0; i < NumSamples; ++i)
				{
					const double Dist = FMath::Min(RoadLength,
						(RoadLength * static_cast<double>(i)) / static_cast<double>(NumSamples - 1));

					struct { double Distance; FVector2D ReturnValue; } PosParams;
					PosParams.Distance = Dist;
					PosParams.ReturnValue = FVector2D::ZeroVector;
					RoadActor->ProcessEvent(GetPosFunc, &PosParams);

					Entry.Points.Add(FVector(PosParams.ReturnValue.X, PosParams.ReturnValue.Y,
						RoadActor->GetActorLocation().Z));
				}
			}

			if (Entry.Points.Num() >= 2)
			{
				DebugLanes.Add(MoveTemp(Entry));
			}
		}

		// If no lanes were found, draw a single road centerline.
		if (Lanes.IsEmpty())
		{
			UFunction* GetPosFunc = DynRoadClass->FindFunctionByName(TEXT("GetWorldPositionAtDistance"));
			if (!GetPosFunc) { continue; }

			const int32 NumSamples = FMath::Max(2, FMath::CeilToInt(RoadLength / 100.0) + 1);
			FDebugLaneData Entry;
			Entry.bIsReverseLane = false;
			Entry.Points.Reserve(NumSamples);

			for (int32 i = 0; i < NumSamples; ++i)
			{
				const double Dist = FMath::Min(RoadLength,
					(RoadLength * static_cast<double>(i)) / static_cast<double>(NumSamples - 1));

				struct { double Distance; FVector2D ReturnValue; } PosParams;
				PosParams.Distance = Dist;
				PosParams.ReturnValue = FVector2D::ZeroVector;
				RoadActor->ProcessEvent(GetPosFunc, &PosParams);

				Entry.Points.Add(FVector(PosParams.ReturnValue.X, PosParams.ReturnValue.Y,
					RoadActor->GetActorLocation().Z));
			}

			if (Entry.Points.Num() >= 2)
			{
				DebugLanes.Add(MoveTemp(Entry));
			}
		}
	}

	if (DebugLanes.Num() > 0)
	{
		UE_LOG(LogAAATraffic, Log,
			TEXT("TrafficSpawner: Reflection fallback — cached %d lane polylines from %d road actors."),
			DebugLanes.Num(), RoadActors.Num());
	}
}

void ATrafficSpawner::DrawDebugLanes() const
{
	UWorld* World = GetWorld();
	if (!World) { return; }

	// Green = forward lanes, Magenta = reverse lanes.
	// Raised above road so lines are clearly visible and not lost in the mesh.
	static constexpr float ZLift = 50.0f; // cm above road surface

	for (const FDebugLaneData& Lane : DebugLanes)
	{
		const FColor Color = Lane.bIsReverseLane
			? FColor(255, 0, 255)   // Magenta — reverse lane
			: FColor(0, 255, 0);    // Green   — forward lane

		for (int32 i = 0; i < Lane.Points.Num() - 1; ++i)
		{
			const FVector A = Lane.Points[i]   + FVector(0, 0, ZLift);
			const FVector B = Lane.Points[i+1] + FVector(0, 0, ZLift);
			DrawDebugLine(World, A, B, Color,
				/*bPersistentLines=*/ false,
				/*LifeTime=*/ -1.0f,
				/*DepthPriority=*/ SDPG_Foreground,
				/*Thickness=*/ 6.0f);
		}
	}
}

// ---------------------------------------------------------------------------
// Intersection debug — cache endpoints, connectivity graph, junction centroids
// ---------------------------------------------------------------------------

void ATrafficSpawner::CacheDebugIntersectionData()
{
	UWorld* World = GetWorld();
	if (!World) { return; }

	UTrafficSubsystem* TrafficSub = World->GetSubsystem<UTrafficSubsystem>();
	ITrafficRoadProvider* Provider = TrafficSub ? TrafficSub->GetProvider() : nullptr;

	if (!Provider)
	{
		bIntersectionCacheAttempted = true;
		UE_LOG(LogAAATraffic, Warning,
			TEXT("TrafficSpawner: Intersection debug enabled but no provider available."));
		return;
	}

	// Collect all lanes across all roads.
	TArray<FTrafficLaneHandle> AllLanes;
	TArray<FTrafficRoadHandle> Roads = Provider->GetAllRoads();
	for (const FTrafficRoadHandle& Road : Roads)
	{
		AllLanes.Append(Provider->GetLanesForRoad(Road));
	}

	// Build a map from lane handle → (start, end) positions for quick lookup.
	TMap<int32, FDebugEndpointData> EndpointLookup;

	for (const FTrafficLaneHandle& Lane : AllLanes)
	{
		TArray<FVector> Points;
		float Width;
		if (!Provider->GetLanePath(Lane, Points, Width) || Points.Num() < 2) { continue; }

		FDebugEndpointData EP;
		EP.StartPos = Points[0];
		EP.EndPos = Points.Last();
		EndpointLookup.Add(Lane.HandleId, EP);
		DebugEndpoints.Add(EP);
	}

	// Build connectivity graph: for each lane, draw arrows to connected lanes.
	// Simultaneously collect junction assignments.
	TMap<int32, TArray<FVector>> JunctionLaneEndpoints; // JunctionId → participating lane endpoints.

	for (const FTrafficLaneHandle& Lane : AllLanes)
	{
		const FDebugEndpointData* SrcEP = EndpointLookup.Find(Lane.HandleId);
		if (!SrcEP) { continue; }

		// Connections.
		TArray<FTrafficLaneHandle> Connected = Provider->GetConnectedLanes(Lane);
		for (const FTrafficLaneHandle& Dst : Connected)
		{
			const FDebugEndpointData* DstEP = EndpointLookup.Find(Dst.HandleId);
			if (!DstEP) { continue; }

			FDebugConnectionData Conn;
			Conn.From = SrcEP->EndPos;
			Conn.To = DstEP->StartPos;
			DebugConnections.Add(Conn);
		}

		// Junction grouping.
		const int32 JId = Provider->GetJunctionForLane(Lane);
		if (JId != 0)
		{
			TArray<FVector>& JEPs = JunctionLaneEndpoints.FindOrAdd(JId);
			JEPs.Add(SrcEP->StartPos);
			JEPs.Add(SrcEP->EndPos);
		}
	}

	// Compute junction centroids.
	// Prefer provider-supplied centroids (derived from mask data at actual
	// intersection positions) over lane-endpoint averages (which can be
	// far from the intersection for through-lanes spanning long roads).
	for (const auto& Pair : JunctionLaneEndpoints)
	{
		FDebugJunctionData JD;
		JD.JunctionId = Pair.Key;

		FVector ProviderCentroid;
		if (Provider->GetJunctionCentroid(Pair.Key, ProviderCentroid))
		{
			JD.Centroid = ProviderCentroid;
		}
		else
		{
			// Fallback: average of participating lane endpoints.
			const int32 NumEndpoints = Pair.Value.Num();
			if (NumEndpoints > 0)
			{
				FVector Sum = FVector::ZeroVector;
				for (const FVector& P : Pair.Value) { Sum += P; }
				JD.Centroid = Sum / static_cast<float>(NumEndpoints);
			}
			else
			{
				// No valid endpoints for this junction; skip to avoid division by zero.
				UE_LOG(LogAAATraffic, Warning,
					TEXT("TrafficSpawner: Junction %d has no valid lane endpoints; skipping debug centroid."),
					JD.JunctionId);
				continue;
			}
		}

		DebugJunctions.Add(JD);
	}

	bIntersectionCacheAttempted = true;
	bIntersectionCacheReady = (DebugEndpoints.Num() > 0);

	UE_LOG(LogAAATraffic, Log,
		TEXT("TrafficSpawner: Intersection debug cached %d endpoints, %d connections, %d junctions."),
		DebugEndpoints.Num(), DebugConnections.Num(), DebugJunctions.Num());
}

void ATrafficSpawner::DrawDebugIntersections() const
{
	UWorld* World = GetWorld();
	if (!World) { return; }

	static constexpr float ZLift = 55.0f; // cm above road surface (5cm above lane debug lines to prevent z-fighting)

	// ── Lane endpoint markers: blue=start, red=end ──
	for (const FDebugEndpointData& EP : DebugEndpoints)
	{
		DrawDebugSphere(World,
			EP.StartPos + FVector(0, 0, ZLift),
			25.0f, 6, FColor::Blue,
			/*bPersistentLines=*/ false, /*LifeTime=*/ -1.0f,
			/*DepthPriority=*/ SDPG_Foreground);

		DrawDebugSphere(World,
			EP.EndPos + FVector(0, 0, ZLift),
			25.0f, 6, FColor::Red,
			/*bPersistentLines=*/ false, /*LifeTime=*/ -1.0f,
			/*DepthPriority=*/ SDPG_Foreground);
	}

	// ── Connectivity arrows: yellow lines from lane end → connected lane start ──
	for (const FDebugConnectionData& Conn : DebugConnections)
	{
		const FVector A = Conn.From + FVector(0, 0, ZLift);
		const FVector B = Conn.To + FVector(0, 0, ZLift);

		DrawDebugLine(World, A, B, FColor::Yellow,
			/*bPersistentLines=*/ false, /*LifeTime=*/ -1.0f,
			/*DepthPriority=*/ SDPG_Foreground, /*Thickness=*/ 3.0f);

		// Arrowhead: small sphere at the destination end.
		DrawDebugSphere(World, B, 15.0f, 4, FColor::Yellow,
			/*bPersistentLines=*/ false, /*LifeTime=*/ -1.0f,
			/*DepthPriority=*/ SDPG_Foreground);
	}

	// ── Junction centroids: white spheres with ID labels ──
	for (const FDebugJunctionData& JD : DebugJunctions)
	{
		const FVector Pos = JD.Centroid + FVector(0, 0, ZLift + 50.0f);

		DrawDebugSphere(World, Pos, 60.0f, 8, FColor::White,
			/*bPersistentLines=*/ false, /*LifeTime=*/ -1.0f,
			/*DepthPriority=*/ SDPG_Foreground);

		DrawDebugString(World, Pos + FVector(0, 0, 70.0f),
			FString::Printf(TEXT("J%d"), JD.JunctionId),
			nullptr, FColor::White, /*Duration=*/ 0.0f,
			/*bDrawShadow=*/ true);
	}
}
#endif // ENABLE_DRAW_DEBUG

void ATrafficSpawner::SpawnSingleVehicle(UWorld* World, ITrafficRoadProvider* Provider,
	const FTrafficLaneHandle& Lane, int32 SlotIndex, int32 VehicleIndex)
{
	TArray<FVector> LanePoints;
	float LaneWidth;
	if (!Provider->GetLanePath(Lane, LanePoints, LaneWidth) || LanePoints.Num() < 2)
	{
		UE_LOG(LogAAATraffic, Warning,
			TEXT("TrafficSpawner: Could not get path for lane %d — skipping."),
			Lane.HandleId);
		return;
	}

	const float TargetOffset = SlotIndex * SpawnSpacing;

	// Walk along lane points to find the staggered spawn position.
	FVector SpawnPos = LanePoints[0];
	FVector SpawnDir = (LanePoints[1] - LanePoints[0]).GetSafeNormal();
	float AccumulatedDist = 0.0f;
	for (int32 p = 0; p < LanePoints.Num() - 1; ++p)
	{
		const float SegLen = FVector::Dist(LanePoints[p], LanePoints[p + 1]);
		if (AccumulatedDist + SegLen >= TargetOffset)
		{
			const float Alpha = (TargetOffset - AccumulatedDist) / FMath::Max(SegLen, KINDA_SMALL_NUMBER);
			SpawnPos = FMath::Lerp(LanePoints[p], LanePoints[p + 1], Alpha);
			SpawnDir = (LanePoints[p + 1] - LanePoints[p]).GetSafeNormal();
			break;
		}
		AccumulatedDist += SegLen;
	}

	// If the requested offset exceeds the lane length, clamp to the lane end.
	if (TargetOffset > AccumulatedDist && LanePoints.Num() >= 2)
	{
		const int32 LastIdx = LanePoints.Num() - 1;
		SpawnPos = LanePoints[LastIdx];
		SpawnDir = (LanePoints[LastIdx] - LanePoints[LastIdx - 1]).GetSafeNormal();
	}

	// Curvature-aware spawn: avoid placing vehicles on tight curves where
	// they'd immediately need emergency braking. If the spawn point is on
	// a curve with radius < 25m (κ > 0.0004), walk forward to find a
	// straighter segment. This prevents freshly spawned vehicles from
	// immediately departing their lane on sharp curves.
	{
		// Find the polyline index closest to the spawn position.
		int32 SpawnIdx = 0;
		float BestDSq = TNumericLimits<float>::Max();
		for (int32 i = 0; i < LanePoints.Num(); ++i)
		{
			const float DSq = FVector::DistSquared(SpawnPos, LanePoints[i]);
			if (DSq < BestDSq)
			{
				BestDSq = DSq;
				SpawnIdx = i;
			}
		}

		// Compute Menger curvature at the spawn point.
		if (LanePoints.Num() >= 3 && SpawnIdx > 0 && SpawnIdx < LanePoints.Num() - 1)
		{
			const FVector& A = LanePoints[FMath::Max(0, SpawnIdx - 2)];
			const FVector& B = LanePoints[SpawnIdx];
			const FVector& C = LanePoints[FMath::Min(LanePoints.Num() - 1, SpawnIdx + 2)];
			const float AB = FVector::Dist2D(A, B);
			const float BC = FVector::Dist2D(B, C);
			const float CA = FVector::Dist2D(C, A);
			const float Denom = AB * BC * CA;
			if (Denom > KINDA_SMALL_NUMBER)
			{
				const float SignedArea2 = (B.X - A.X) * (C.Y - A.Y) - (B.Y - A.Y) * (C.X - A.X);
				const float Curvature = FMath::Abs(2.0f * SignedArea2 / Denom);
				if (Curvature > 0.0004f) // radius < 25m
				{
					UE_LOG(LogAAATraffic, Log,
						TEXT("TrafficSpawner: Skipping vehicle %d on lane %d — "
							 "spawn on tight curve (κ=%.5f, R≈%.0fm)."),
						VehicleIndex, Lane.HandleId, Curvature,
						Curvature > KINDA_SMALL_NUMBER ? 1.0f / Curvature / 100.0f : 999999.0f);
					return;
				}
			}
		}
	}

	const FVector SpawnLocation = SpawnPos + FVector(0.0, 0.0, SpawnZOffset);
	const FRotator SpawnRotation = SpawnDir.Rotation();
	const FTransform SpawnTransform(SpawnRotation, SpawnLocation);

	// FIX (Phase 3): Pre-spawn overlap check — skip this slot if an existing
	// traffic vehicle is already within SpawnSpacing. Without this, the
	// AdjustIfPossibleButAlwaysSpawn policy can stack vehicles on top of each
	// other, causing immediate physics explosions on spawn.
	{
		UTrafficSubsystem* OverlapSub = World->GetSubsystem<UTrafficSubsystem>();
		if (OverlapSub)
		{
			const float MinSeparationSq = SpawnSpacing * SpawnSpacing;
			for (const TWeakObjectPtr<ATrafficVehicleController>& WeakVC : OverlapSub->GetActiveVehicles())
			{
				ATrafficVehicleController* VC = WeakVC.Get();
				if (!VC) { continue; }
				const APawn* ExistingPawn = VC->GetPawn();
				if (!ExistingPawn) { continue; }

				if (FVector::DistSquared(SpawnLocation, ExistingPawn->GetActorLocation()) < MinSeparationSq)
				{
					UE_LOG(LogAAATraffic, Log,
						TEXT("TrafficSpawner: Skipping vehicle %d on lane %d — "
							 "existing vehicle '%s' too close (within %.0f cm)."),
						VehicleIndex, Lane.HandleId,
						*ExistingPawn->GetName(), SpawnSpacing);
					return;
				}
			}
		}
	}

	// Select vehicle class via weighted random (C1: vehicle variety).
	const TSubclassOf<APawn> ChosenClass = SelectVehicleClass(VehicleIndex);
	if (!ChosenClass)
	{
		UE_LOG(LogAAATraffic, Warning,
			TEXT("TrafficSpawner: No valid vehicle class for vehicle %d."), VehicleIndex);
		return;
	}

	// Try to acquire from the vehicle pool first (I1: object pooling).
	APawn* Vehicle = nullptr;
	UTrafficSubsystem* TrafficSub = World->GetSubsystem<UTrafficSubsystem>();
	UTrafficVehiclePool* Pool = TrafficSub ? TrafficSub->GetVehiclePool() : nullptr;
	if (Pool)
	{
		Vehicle = Pool->AcquireVehicle(World, ChosenClass, SpawnTransform);
	}

	if (!Vehicle)
	{
		FActorSpawnParameters SpawnParams;
		SpawnParams.SpawnCollisionHandlingOverride =
			ESpawnActorCollisionHandlingMethod::AdjustIfPossibleButAlwaysSpawn;
		Vehicle = World->SpawnActor<APawn>(ChosenClass, SpawnTransform, SpawnParams);
	}
	if (!Vehicle)
	{
		UE_LOG(LogAAATraffic, Warning,
			TEXT("TrafficSpawner: Failed to spawn vehicle %d."), VehicleIndex);
		return;
	}

	ATrafficVehicleController* Controller =
		World->SpawnActor<ATrafficVehicleController>();
	if (Controller)
	{
		Controller->SetRandomSeed(SpawnSeed + VehicleIndex);

		// Apply deterministic per-vehicle speed variation.
		float FinalSpeed = VehicleSpeed;
		if (SpeedVariation > KINDA_SMALL_NUMBER)
		{
			constexpr int32 SpeedVariationSeedOffset = 10000;
			FRandomStream SpeedRng(SpawnSeed + VehicleIndex + SpeedVariationSeedOffset);
			const float VariationFraction = SpeedVariation / 100.0f;
			const float Offset = SpeedRng.FRandRange(-VariationFraction, VariationFraction);
			FinalSpeed = VehicleSpeed * (1.0f + Offset);
			FinalSpeed = FMath::Max(FinalSpeed, 0.0f);
		}
		Controller->SetTargetSpeed(FinalSpeed);
		Controller->SetLaneChangeAggression(LaneChangeAggression);

		// Per-road speed limit: check the override map via the subsystem,
		// falling back to the spawner's DefaultSpeedLimit (UrbanSpeed by default).
		float EffectiveSpeedLimit = DefaultSpeedLimit;
		{
			UTrafficSubsystem* SpawnSub = World->GetSubsystem<UTrafficSubsystem>();
			ITrafficRoadProvider* SpawnProv = SpawnSub ? SpawnSub->GetProvider() : nullptr;
			if (SpawnProv && SpawnSub)
			{
				const FTrafficRoadHandle Road = SpawnProv->GetRoadForLane(Lane);
				const float RoadLimit = SpawnSub->GetRoadSpeedLimit(Road.HandleId);
				if (RoadLimit > 0.0f)
				{
					EffectiveSpeedLimit = RoadLimit;
				}
			}
		}
		Controller->SetDefaultSpeedLimit(EffectiveSpeedLimit);

		Controller->Possess(Vehicle);
		Controller->InitializeLaneFollowing(Lane);

		OwnedVehicles.Add(Controller);

		// --- Spawner diagnostic: confirm possession and movement readiness ---
		{
			UPawnMovementComponent* MC = Vehicle->GetMovementComponent();
			UChaosWheeledVehicleMovementComponent* ChaosMC =
				Cast<UChaosWheeledVehicleMovementComponent>(MC);
			UE_LOG(LogAAATraffic, Log,
				TEXT("TrafficSpawner: Vehicle %d spawned on lane %d. "
					 "Class='%s' MovementComp='%s' ChaosWheeledCast=%s Possessed=%s"),
				VehicleIndex, Lane.HandleId,
				*Vehicle->GetClass()->GetName(),
				MC ? *MC->GetClass()->GetName() : TEXT("NULL"),
				ChaosMC ? TEXT("OK") : TEXT("FAILED"),
				Controller->GetPawn() ? TEXT("Yes") : TEXT("No"));
		}
	}
}

void ATrafficSpawner::OnVehicleDespawned(ATrafficVehicleController* Controller, const FTrafficLaneHandle& Lane)
{
	if (!bEnableRespawn || !bSpawnComplete) { return; }

	// Only respond to vehicles owned by this spawner.
	if (!Controller || !OwnedVehicles.Remove(Controller))
	{
		return;
	}

	// Queue the lane for respawn.
	if (Lane.IsValid())
	{
		RespawnQueue.Add(Lane);
	}
	else if (CachedAllLanes.Num() > 0)
	{
		// If lane is unknown, pick one deterministically.
		FRandomStream RespawnRng(SpawnSeed + TotalVehiclesSpawned + RespawnCounter);
		const int32 Idx = RespawnRng.RandRange(0, CachedAllLanes.Num() - 1);
		RespawnQueue.Add(CachedAllLanes[Idx]);
	}
}

void ATrafficSpawner::CheckRespawn()
{
	if (RespawnQueue.IsEmpty()) { return; }

	UWorld* World = GetWorld();
	if (!World) { return; }

	UTrafficSubsystem* TrafficSub = World->GetSubsystem<UTrafficSubsystem>();
	ITrafficRoadProvider* Provider = TrafficSub ? TrafficSub->GetProvider() : nullptr;
	if (!Provider) { return; }

	// Gather player positions for distance validation.
	TArray<FVector> PlayerPositions;
	for (FConstPlayerControllerIterator It = World->GetPlayerControllerIterator(); It; ++It)
	{
		if (const APlayerController* PC = It->Get())
		{
			if (const APawn* PlayerPawn = PC->GetPawn())
			{
				PlayerPositions.Add(PlayerPawn->GetActorLocation());
			}
		}
	}

	const float MinDistSq = MinRespawnDistance * MinRespawnDistance;

	// Process queued respawns.
	TArray<FTrafficLaneHandle> Remaining;
	for (const FTrafficLaneHandle& Lane : RespawnQueue)
	{
		// Get lane start position to check against player distances.
		TArray<FVector> LanePoints;
		float LaneWidth;
		if (!Provider->GetLanePath(Lane, LanePoints, LaneWidth) || LanePoints.Num() < 2)
		{
			continue; // Lane no longer valid — drop.
		}

		const FVector SpawnCandidate = LanePoints[0] + FVector(0, 0, SpawnZOffset);

		// Ensure spawn location is far enough from all players.
		bool bTooClose = false;
		for (const FVector& PlayerPos : PlayerPositions)
		{
			if (FVector::DistSquared(SpawnCandidate, PlayerPos) < MinDistSq)
			{
				bTooClose = true;
				break;
			}
		}

		if (bTooClose)
		{
			Remaining.Add(Lane); // Retry next interval.
			continue;
		}

		++RespawnCounter;
		const int32 VehicleIdx = TotalVehiclesSpawned++;
		// Use RespawnCounter as slot offset to stagger respawn positions
		// and avoid clumping when multiple respawns target the same lane.
		SpawnSingleVehicle(World, Provider, Lane, RespawnCounter, VehicleIdx);

		UE_LOG(LogAAATraffic, Log,
			TEXT("TrafficSpawner: Respawned vehicle on lane %d (respawn #%d)."),
			Lane.HandleId, RespawnCounter);
	}

	RespawnQueue = MoveTemp(Remaining);
}

void ATrafficSpawner::OnProviderRegistered(ITrafficRoadProvider* Provider)
{
	if (bSpawnComplete) { return; }

#if defined(ENABLE_DRAW_DEBUG) && ENABLE_DRAW_DEBUG
	// Reset debug cache so it picks up provider data on next tick.
	bDebugCacheReady = false;
	bDebugCacheAttempted = false;
	DebugLanes.Empty();
#endif

	UE_LOG(LogAAATraffic, Log,
		TEXT("TrafficSpawner: Provider registered (deferred) — attempting vehicle spawn."));
	SpawnVehicles();
}

void ATrafficSpawner::SpawnVehicles()
{
	if (bSpawnComplete) { return; }

	if (VehicleClasses.Num() == 0 && !VehicleClass)
	{
		UE_LOG(LogAAATraffic, Error, TEXT("TrafficSpawner: No VehicleClass or VehicleClasses assigned."));
		return;
	}

	UWorld* World = GetWorld();
	if (!World)
	{
		UE_LOG(LogAAATraffic, Error, TEXT("TrafficSpawner: World is null in SpawnVehicles."));
		return;
	}

	UTrafficSubsystem* TrafficSub = World->GetSubsystem<UTrafficSubsystem>();
	ITrafficRoadProvider* Provider = TrafficSub ? TrafficSub->GetProvider() : nullptr;
	if (!Provider)
	{
		// No provider yet — subscribe for deferred retry.
		if (TrafficSub)
		{
			TrafficSub->OnProviderRegistered.AddUObject(this, &ATrafficSpawner::OnProviderRegistered);
			UE_LOG(LogAAATraffic, Log,
				TEXT("TrafficSpawner: No provider yet — subscribed for deferred retry."));
		}
		else
		{
			UE_LOG(LogAAATraffic, Error,
				TEXT("TrafficSpawner: No TrafficSubsystem found."));
		}
		return;
	}

	// Push spawner speed tiers into the provider so road classification
	// uses user-configured values instead of hardcoded defaults.
	if (auto* RoadBLDProvider = World->GetSubsystem<URoadBLDReflectionProvider>())
	{
		RoadBLDProvider->SetSpeedTiers(ResidentialSpeed, UrbanSpeed, HighwaySpeed);
	}

	// Gather all lanes across all roads.
	TArray<FTrafficRoadHandle> Roads = Provider->GetAllRoads();
	if (Roads.IsEmpty())
	{
		UE_LOG(LogAAATraffic, Warning, TEXT("TrafficSpawner: No roads found by the provider."));
		return;
	}

	// Push per-road speed overrides into the subsystem so controllers can
	// query the correct speed limit when transitioning between roads.
	for (const auto& OverrideEntry : RoadSpeedOverrides)
	{
		TrafficSub->SetRoadSpeedLimit(OverrideEntry.Key, OverrideEntry.Value);
	}

	TArray<FTrafficLaneHandle> AllLanes;
	for (const FTrafficRoadHandle& Road : Roads)
	{
		AllLanes.Append(Provider->GetLanesForRoad(Road));
	}

	if (AllLanes.IsEmpty())
	{
		UE_LOG(LogAAATraffic, Warning, TEXT("TrafficSpawner: No lanes found on any road."));
		return;
	}

	// Auto-place traffic signals at discovered junctions (C2).
	// Must happen before filtering so PlaceAutoSignals sees all lanes.
	PlaceAutoSignals(World, Provider, AllLanes);

	// Filter out junction lanes — vehicles must not spawn inside intersections.
	// Junction lanes are short virtual segments (12-14m) and spawning on them
	// causes vehicles to immediately enter junction logic on their first tick.
	{
		const int32 PreFilterCount = AllLanes.Num();
		AllLanes.RemoveAll([Provider](const FTrafficLaneHandle& Lane)
		{
			return Provider->GetJunctionForLane(Lane) != 0;
		});
		UE_LOG(LogAAATraffic, Log,
			TEXT("TrafficSpawner: Filtered %d junction lanes from spawn pool (%d -> %d)."),
			PreFilterCount - AllLanes.Num(), PreFilterCount, AllLanes.Num());
	}

	// Filter out stub lanes that are too short for a vehicle to spawn on.
	// These occur when a road's intersection mask starts at (or very near)
	// the road endpoint, producing a pre-mask segment of only a few cm.
	{
		constexpr float MinSpawnLaneLengthCm = 500.0f;
		const int32 PreStubCount = AllLanes.Num();
		AllLanes.RemoveAll([Provider, MinSpawnLaneLengthCm](const FTrafficLaneHandle& Lane)
		{
			const float LaneLengthCm = Provider->GetLaneLength(Lane);
			// Treat length <= 0 as "unknown" (provider has no data) — keep the
			// lane rather than wrongly filtering it as a stub.
			return LaneLengthCm > 0.0f && LaneLengthCm < MinSpawnLaneLengthCm;
		});
		const int32 StubsRemoved = PreStubCount - AllLanes.Num();
		if (StubsRemoved > 0)
		{
			UE_LOG(LogAAATraffic, Log,
				TEXT("TrafficSpawner: Filtered %d stub lanes (< %.0f cm) from spawn pool (%d -> %d)."),
				StubsRemoved, MinSpawnLaneLengthCm, PreStubCount, AllLanes.Num());
		}
	}

	if (AllLanes.IsEmpty())
	{
		UE_LOG(LogAAATraffic, Warning, TEXT("TrafficSpawner: All lanes are junction lanes — no spawn candidates!"));
		return;
	}

	// Cache filtered lanes for respawn use.
	CachedAllLanes = AllLanes;

	// Allow VehicleCount to exceed lane count — vehicles share lanes with
	// staggered positioning via SpawnSpacing.
	const int32 SpawnCount = VehicleCount;

	UE_LOG(LogAAATraffic, Log,
		TEXT("TrafficSpawner: Spawning %d vehicles across %d available lanes (junction lanes excluded)."),
		SpawnCount, AllLanes.Num());

	// Track how many vehicles have been placed on each lane for staggered positioning.
	TMap<int32, int32> LaneOccupancy;

	for (int32 i = 0; i < SpawnCount; ++i)
	{
		const FTrafficLaneHandle& Lane = AllLanes[i % AllLanes.Num()];
		const int32 SlotIndex = LaneOccupancy.FindOrAdd(Lane.HandleId, 0);
		LaneOccupancy[Lane.HandleId] = SlotIndex + 1;

		SpawnSingleVehicle(World, Provider, Lane, SlotIndex, i);
	}

	TotalVehiclesSpawned = SpawnCount;
	bSpawnComplete = true;

	// Unbind delegate now that spawning is complete (review feedback: keep delegate list clean).
	if (TrafficSub)
	{
		TrafficSub->OnProviderRegistered.RemoveAll(this);
	}

	// Subscribe to despawn events for respawning.
	if (bEnableRespawn && TrafficSub)
	{
		TrafficSub->OnVehicleDespawned.AddUObject(this, &ATrafficSpawner::OnVehicleDespawned);

		World->GetTimerManager().SetTimer(
			RespawnTimerHandle,
			FTimerDelegate::CreateUObject(this, &ATrafficSpawner::CheckRespawn),
			RespawnCheckInterval,
			/*bLoop=*/ true);

		UE_LOG(LogAAATraffic, Log,
			TEXT("TrafficSpawner: Respawn enabled — checking every %.1f seconds."),
			RespawnCheckInterval);
	}
}

TSubclassOf<APawn> ATrafficSpawner::SelectVehicleClass(int32 VehicleIndex) const
{
	if (VehicleClasses.Num() == 0)
	{
		return VehicleClass;
	}

	// Build CDF from valid entries.
	float TotalWeight = 0.0f;
	for (const FVehicleClassEntry& Entry : VehicleClasses)
	{
		if (Entry.VehicleClass)
		{
			TotalWeight += Entry.Weight;
		}
	}

	if (TotalWeight <= KINDA_SMALL_NUMBER)
	{
		return VehicleClass; // Fallback to single class.
	}

	constexpr int32 ClassSelectionSeedOffset = 20000;
	FRandomStream ClassRng(SpawnSeed + VehicleIndex + ClassSelectionSeedOffset);
	const float Roll = ClassRng.FRandRange(0.0f, TotalWeight);

	float Cumulative = 0.0f;
	for (const FVehicleClassEntry& Entry : VehicleClasses)
	{
		if (!Entry.VehicleClass) { continue; }
		Cumulative += Entry.Weight;
		if (Roll <= Cumulative)
		{
			return Entry.VehicleClass;
		}
	}

	// Fallback (floating-point edge case).
	for (int32 i = VehicleClasses.Num() - 1; i >= 0; --i)
	{
		if (VehicleClasses[i].VehicleClass)
		{
			return VehicleClasses[i].VehicleClass;
		}
	}
	return VehicleClass;
}

void ATrafficSpawner::PlaceAutoSignals(UWorld* World, ITrafficRoadProvider* Provider,
	const TArray<FTrafficLaneHandle>& AllLanes)
{
	if (!bAutoPlaceSignals) { return; }

	const bool bSignalDiag = (GTrafficJunctionDiagnostics >= 1);

	UTrafficSubsystem* TrafficSub = World ? World->GetSubsystem<UTrafficSubsystem>() : nullptr;

	// Discover unique junction IDs from lane data.
	TMap<int32, TArray<FTrafficLaneHandle>> JunctionLanes;
	for (const FTrafficLaneHandle& Lane : AllLanes)
	{
		const int32 JId = Provider->GetJunctionForLane(Lane);
		if (JId != 0)
		{
			JunctionLanes.FindOrAdd(JId).Add(Lane);
		}
	}

	if (JunctionLanes.IsEmpty()) { return; }

	// Sort junction IDs for deterministic processing (System.md §4.4).
	TArray<int32> JunctionIds;
	JunctionLanes.GetKeys(JunctionIds);
	JunctionIds.Sort();

	int32 SignalsPlaced = 0;
	for (const int32 JId : JunctionIds)
	{
		// Skip junctions that already have a manually-placed signal controller.
		if (TrafficSub && TrafficSub->GetSignalForJunction(JId))
		{
			continue;
		}

		const TArray<FTrafficLaneHandle>& Lanes = JunctionLanes[JId];

		// Group incoming lanes by road for phase groups.
		TMap<int32, TArray<FTrafficLaneHandle>> LanesByRoad;
		for (const FTrafficLaneHandle& Lane : Lanes)
		{
			const FTrafficRoadHandle Road = Provider->GetRoadForLane(Lane);
			LanesByRoad.FindOrAdd(Road.HandleId).Add(Lane);
		}

		// --- Use deferred spawn so JunctionId and PhaseGroups are set BEFORE
		// BeginPlay fires. Previously, SpawnActor triggered BeginPlay immediately,
		// which saw JunctionId==0, logged a warning, and disabled the signal's tick.
		FTransform SignalTransform = FTransform::Identity;
		ATrafficSignalController* Signal = World->SpawnActorDeferred<ATrafficSignalController>(
			ATrafficSignalController::StaticClass(), SignalTransform);
		if (!Signal) { continue; }

		Signal->JunctionId = JId;

		// Auto-classify by APPROACH ARM count (direction-clustered).
		// On a 2-lane bidirectional road both lanes share the same road
		// ID, so grouping by road under-counts.  Instead count unique
		// approach directions across all junction lanes.
		TArray<FVector> ApproachArmDirs;
		for (const FTrafficLaneHandle& JLane : Lanes)
		{
			const FVector Dir = Provider->GetLaneDirection(JLane);
			bool bMerged = false;
			for (const FVector& Existing : ApproachArmDirs)
			{
				if (FVector::DotProduct(Dir, Existing) > 0.7f) // ~45° merge threshold
				{
					bMerged = true;
					break;
				}
			}
			if (!bMerged)
			{
				ApproachArmDirs.Add(Dir);
			}
			if (bSignalDiag)
			{
				UE_LOG(LogAAATraffic, Warning,
					TEXT("SIGNAL-DIAG: JunctionId=%d ArmScan Lane=%d Dir=(%.3f, %.3f, %.3f) Merged=%s ArmsSoFar=%d"),
					JId, JLane.HandleId, Dir.X, Dir.Y, Dir.Z, bMerged ? TEXT("Y") : TEXT("N"), ApproachArmDirs.Num());
			}
		}
		const int32 ApproachArmCount = ApproachArmDirs.Num();

		// First, check for roundabout topology: if a junction's internal
		// lanes form a cycle (can follow connected lanes back to the start),
		// it's a roundabout. Roundabouts use Yield mode — entering traffic
		// yields to vehicles already in the circle.
		bool bIsRoundabout = false;
		if (ApproachArmCount >= 3)
		{
			// Detect cycle: for each junction lane, follow connected lanes
			// that stay within the same junction. If we can return to the
			// starting lane within MaxHops, it's circular.
			for (const FTrafficLaneHandle& StartLane : Lanes)
			{
				FTrafficLaneHandle Walk = StartLane;
				bool bCycle = false;
				for (int32 Hop = 0; Hop < 16; ++Hop)
				{
					TArray<FTrafficLaneHandle> WalkExits = Provider->GetConnectedLanes(Walk);
					FTrafficLaneHandle NextInJunction;
					for (const FTrafficLaneHandle& E : WalkExits)
					{
						if (Provider->GetJunctionForLane(E) == JId && E.HandleId != Walk.HandleId)
						{
							NextInJunction = E;
							break;
						}
					}
					if (!NextInJunction.IsValid()) { break; }
					Walk = NextInJunction;
					if (Walk.HandleId == StartLane.HandleId)
					{
						bCycle = true;
						break;
					}
				}
				if (bCycle) { bIsRoundabout = true; break; }
			}
		}

		if (bIsRoundabout)
		{
			Signal->ControlMode = EJunctionControlMode::Yield;
			UE_LOG(LogAAATraffic, Log,
				TEXT("PlaceAutoSignals: JunctionId=%d detected as ROUNDABOUT (%d approach arms, circular topology) → Yield"),
				JId, ApproachArmCount);
		}
		else if (ApproachArmCount >= 4)
		{
			Signal->ControlMode = EJunctionControlMode::Signal;
			if (bSignalDiag)
			{
				UE_LOG(LogAAATraffic, Warning,
					TEXT("SIGNAL-DIAG: JunctionId=%d ApproachArmCount=%d → Signal (4+ arms)"),
					JId, ApproachArmCount);
			}
		}
		else if (ApproachArmCount == 3)
		{
			Signal->ControlMode = EJunctionControlMode::StopSign;
			if (bSignalDiag)
			{
				UE_LOG(LogAAATraffic, Warning,
					TEXT("SIGNAL-DIAG: JunctionId=%d ApproachArmCount=%d → StopSign (3 arms)"),
					JId, ApproachArmCount);
			}
		}
		else
		{
			// For 2-arm junctions, examine actual turning angles between
			// approach and exit lanes to distinguish gentle curves from
			// genuine T-junctions that need stop control.
			float MaxTurnAngleDeg = 0.0f;
			for (const FTrafficLaneHandle& JLane : Lanes)
			{
				TArray<FTrafficLaneHandle> Exits = Provider->GetConnectedLanes(JLane);
				if (Exits.Num() == 0) { continue; }
				const float JLaneLen = Provider->GetLaneLength(JLane);
				const FVector ApproachDir = Provider->GetLaneDirectionAtDistance(JLane, JLaneLen);
				for (const FTrafficLaneHandle& Exit : Exits)
				{
					const FVector ExitDir = Provider->GetLaneDirection(Exit);
					const float Dot = FMath::Clamp(FVector::DotProduct(ApproachDir, ExitDir), -1.0f, 1.0f);
					const float AngleDeg = FMath::RadiansToDegrees(FMath::Acos(Dot));
					MaxTurnAngleDeg = FMath::Max(MaxTurnAngleDeg, AngleDeg);
				}
			}
			Signal->ControlMode = (MaxTurnAngleDeg > 45.0f)
				? EJunctionControlMode::StopSign
				: EJunctionControlMode::Yield;
			if (bSignalDiag)
			{
				UE_LOG(LogAAATraffic, Warning,
					TEXT("SIGNAL-DIAG: JunctionId=%d ApproachArmCount=%d MaxTurnAngle=%.1f → %s (2-arm)"),
					JId, ApproachArmCount, MaxTurnAngleDeg,
					MaxTurnAngleDeg > 45.0f ? TEXT("StopSign") : TEXT("Yield"));
			}
		}

		// Build phase groups: cluster opposing-direction roads into the
		// same phase so a standard 4-way gets 2 phases (N+S green
		// together, E+W green together) instead of 4.
		//
		// Algorithm: compute the average approach direction per road,
		// then greedily merge roads whose directions are nearly opposite
		// (dot < -0.3).
		TArray<int32> RoadKeys;
		LanesByRoad.GetKeys(RoadKeys);
		RoadKeys.Sort();

		// Compute average approach direction per road.
		TMap<int32, FVector> RoadApproachDir;
		for (const int32 RoadId : RoadKeys)
		{
			FVector DirSum = FVector::ZeroVector;
			for (const FTrafficLaneHandle& Lane : LanesByRoad[RoadId])
			{
				const float Len = Provider->GetLaneLength(Lane);
				const FVector LaneDir = Provider->GetLaneDirectionAtDistance(Lane, Len);
				DirSum += LaneDir;
				if (bSignalDiag)
				{
					UE_LOG(LogAAATraffic, Warning,
						TEXT("SIGNAL-DIAG: JunctionId=%d Road=%d Lane=%d EndDir=(%.3f, %.3f, %.3f)"),
						JId, RoadId, Lane.HandleId, LaneDir.X, LaneDir.Y, LaneDir.Z);
				}
			}
			const FVector Normalized = DirSum.GetSafeNormal();
			RoadApproachDir.Add(RoadId, Normalized);
			if (bSignalDiag)
			{
				UE_LOG(LogAAATraffic, Warning,
					TEXT("SIGNAL-DIAG: JunctionId=%d Road=%d DirSum=(%.3f, %.3f, %.3f) Normalized=(%.3f, %.3f, %.3f) LaneCount=%d"),
					JId, RoadId, DirSum.X, DirSum.Y, DirSum.Z, Normalized.X, Normalized.Y, Normalized.Z, LanesByRoad[RoadId].Num());
			}
		}

		// Greedy clustering: walk through roads and pair opposing ones.
		TArray<TArray<int32>> RoadClusters;
		TSet<int32> Assigned;
		for (int32 i = 0; i < RoadKeys.Num(); ++i)
		{
			if (Assigned.Contains(RoadKeys[i])) { continue; }
			TArray<int32> Cluster;
			Cluster.Add(RoadKeys[i]);
			Assigned.Add(RoadKeys[i]);
			const FVector& DirA = RoadApproachDir[RoadKeys[i]];
			// Find opposing roads (dot < -0.3 → facing each other).
			for (int32 j = i + 1; j < RoadKeys.Num(); ++j)
			{
				if (Assigned.Contains(RoadKeys[j])) { continue; }
				const FVector& DirB = RoadApproachDir[RoadKeys[j]];
				if (FVector::DotProduct(DirA, DirB) < -0.3f)
				{
					Cluster.Add(RoadKeys[j]);
					Assigned.Add(RoadKeys[j]);
				}
			}
			RoadClusters.Add(MoveTemp(Cluster));
		}

		if (bSignalDiag)
		{
			UE_LOG(LogAAATraffic, Warning,
				TEXT("SIGNAL-DIAG: JunctionId=%d Clustering complete — %d clusters from %d roads"),
				JId, RoadClusters.Num(), RoadKeys.Num());
		}

		// Build one phase group per cluster.
		Signal->PhaseGroups.Empty();
		for (const TArray<int32>& Cluster : RoadClusters)
		{
			FSignalPhaseGroup Group;
			TArray<FString> RoadNames;
			for (const int32 RoadId : Cluster)
			{
				RoadNames.Add(FString::Printf(TEXT("Road_%d"), RoadId));
				Group.GreenLanes.Append(LanesByRoad[RoadId]);
			}
			Group.GroupName = FString::Join(RoadNames, TEXT("+"));
			Signal->PhaseGroups.Add(MoveTemp(Group));
		}

		// ── Protected left-turn phases ──────────────────────────
		// For Signal-mode junctions, identify lanes that have left-turn
		// exits and create a dedicated arrow phase before each through-
		// phase. The arrow phase gets a shorter green (10s default).
		// This prevents left-turners from having to yield to through-
		// traffic — they get their own exclusive green.
		if (Signal->ControlMode == EJunctionControlMode::Signal && Signal->PhaseGroups.Num() > 1)
		{
			TArray<FSignalPhaseGroup> ExpandedGroups;
			for (const FSignalPhaseGroup& ThroughGroup : Signal->PhaseGroups)
			{
				// Find left-turn lanes in this group.
				TArray<FTrafficLaneHandle> LeftTurnLanes;
				for (const FTrafficLaneHandle& GLane : ThroughGroup.GreenLanes)
				{
					TArray<FTrafficLaneHandle> Exits = Provider->GetConnectedLanes(GLane);
					const float GLaneLen = Provider->GetLaneLength(GLane);
					const FVector ApproachDir = Provider->GetLaneDirectionAtDistance(GLane, GLaneLen);
					for (const FTrafficLaneHandle& Exit : Exits)
					{
						if (Provider->GetJunctionForLane(Exit) != 0) { continue; }
						const FVector ExitDir = Provider->GetLaneDirection(Exit);
						const float CrossZ = FVector::CrossProduct(ApproachDir, ExitDir).Z;
						if (CrossZ > 0.26f) // Left turn (sin(15°) threshold)
						{
							LeftTurnLanes.AddUnique(GLane);
							break;
						}
					}
				}

				// If left-turn lanes found, insert an arrow phase before through.
				if (LeftTurnLanes.Num() > 0)
				{
					FSignalPhaseGroup ArrowGroup;
					ArrowGroup.GroupName = ThroughGroup.GroupName + TEXT("_LeftArrow");
					ArrowGroup.GreenLanes = MoveTemp(LeftTurnLanes);
					ArrowGroup.GroupGreenDuration = 10.0f; // Shorter left-turn phase.
					ArrowGroup.bIsProtectedArrow = true;
					ExpandedGroups.Add(MoveTemp(ArrowGroup));
				}

				// Through-phase follows.
				ExpandedGroups.Add(ThroughGroup);
			}
			Signal->PhaseGroups = MoveTemp(ExpandedGroups);
		}

		// Derive all-red clearance from the longest junction lane — the
		// time a vehicle needs to cross the intersection at the intersection
		// speed limit.  Clamp to [2, 8] seconds for sanity.
		{
			float MaxJunctionLaneLen = 0.0f;
			for (const FTrafficLaneHandle& JLane : Lanes)
			{
				MaxJunctionLaneLen = FMath::Max(MaxJunctionLaneLen, Provider->GetLaneLength(JLane));
			}
			// crossing time = distance / speed. IntersectionSpeedLimitCmPerSec default = 2000 (20 m/s ≈ 45 mph).
			constexpr float TypicalIntersectionSpeedCmPerSec = 1500.0f; // ~15 m/s conservative turning speed
			const float CrossingTimeSec = (MaxJunctionLaneLen > 0.0f)
				? MaxJunctionLaneLen / TypicalIntersectionSpeedCmPerSec
				: 2.0f;
			Signal->AllRedClearanceSec = FMath::Clamp(CrossingTimeSec, 2.0f, 8.0f);
		}

		// Stagger signal phases so adjacent junctions don't all start green
		// simultaneously. Each junction offsets by one full phase period.
		Signal->PhaseOffset = static_cast<float>(SignalsPlaced)
			* (Signal->GreenDuration + Signal->YellowDuration);

		// Dump final phase group structure.
		if (bSignalDiag)
		{
			UE_LOG(LogAAATraffic, Warning,
				TEXT("SIGNAL-DIAG: JunctionId=%d FINAL — ControlMode=%d PhaseGroupCount=%d AllRedClearance=%.1fs Offset=%.1fs"),
				JId, static_cast<int32>(Signal->ControlMode), Signal->PhaseGroups.Num(),
				Signal->AllRedClearanceSec, Signal->PhaseOffset);
		}
		for (int32 gi = 0; gi < Signal->PhaseGroups.Num(); ++gi)
		{
			const FSignalPhaseGroup& G = Signal->PhaseGroups[gi];
			FString LaneList;
			for (const FTrafficLaneHandle& L : G.GreenLanes)
			{
				LaneList += FString::Printf(TEXT("%d "), L.HandleId);
			}
			if (bSignalDiag)
			{
				UE_LOG(LogAAATraffic, Warning,
					TEXT("SIGNAL-DIAG: JunctionId=%d Group[%d] '%s' GreenDur=%.1f Lanes=[%s]"),
					JId, gi, *G.GroupName, G.GroupGreenDuration, *LaneList);
			}
		}

		// Finish spawning — now BeginPlay sees the correct JunctionId and registers.
		Signal->FinishSpawning(SignalTransform);

		++SignalsPlaced;
	}

	if (SignalsPlaced > 0)
	{
		UE_LOG(LogAAATraffic, Log,
			TEXT("TrafficSpawner: Auto-placed %d junction controllers across %d junctions "
				 "(mode auto-classified by approach arm count: 4+=Signal, 3=StopSign, 2=angle-check)."),
			SignalsPlaced, JunctionIds.Num());
	}
}
