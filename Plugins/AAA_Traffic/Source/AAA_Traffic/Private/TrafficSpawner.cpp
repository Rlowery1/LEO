// Copyright AAA_Traffic Contributors. All Rights Reserved.

#include "TrafficSpawner.h"
#include "TrafficSubsystem.h"
#include "TrafficRoadProvider.h"
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

ATrafficSpawner::ATrafficSpawner()
	: VehicleCount(1)
	, VehicleSpeed(1500.f)
	, SpawnSeed(42)
	, SpawnZOffset(50.f)
	, SpawnSpacing(1500.f)
	, SpeedVariation(15.f)
	, LaneChangeAggression(0.5f)
	, bEnableRespawn(true)
	, RespawnCheckInterval(2.0f)
	, MinRespawnDistance(10000.f)
	, DefaultSpeedLimit(0.0f)
{
#if ENABLE_DRAW_DEBUG
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

#if ENABLE_DRAW_DEBUG
	if (bDebugDrawLanes || bDebugDrawIntersections)
	{
		SetActorTickEnabled(true);
	}
#endif
}

bool ATrafficSpawner::ShouldTickIfViewportsOnly() const
{
#if ENABLE_DRAW_DEBUG
	return bDebugDrawLanes || bDebugDrawIntersections;
#else
	return false;
#endif
}

void ATrafficSpawner::Tick(float DeltaSeconds)
{
	Super::Tick(DeltaSeconds);

#if ENABLE_DRAW_DEBUG
	if (!bDebugDrawLanes && !bDebugDrawIntersections)
	{
		return;
	}

	if (bDebugDrawLanes && !bDebugCacheReady && !bDebugCacheAttempted)
	{
		CacheDebugLaneData();
	}

	if (bDebugDrawIntersections && !bIntersectionCacheReady && !bIntersectionCacheAttempted)
	{
		CacheDebugIntersectionData();
	}

	if (bDebugDrawLanes && bDebugCacheReady)
	{
		DrawDebugLanes();
	}

	if (bDebugDrawIntersections && bIntersectionCacheReady)
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

#if ENABLE_DRAW_DEBUG
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

	const FVector SpawnLocation = SpawnPos + FVector(0.0, 0.0, SpawnZOffset);
	const FRotator SpawnRotation = SpawnDir.Rotation();
	const FTransform SpawnTransform(SpawnRotation, SpawnLocation);

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
		Controller->SetDefaultSpeedLimit(DefaultSpeedLimit);

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

#if ENABLE_DRAW_DEBUG
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

	// Gather all lanes across all roads.
	TArray<FTrafficRoadHandle> Roads = Provider->GetAllRoads();
	if (Roads.IsEmpty())
	{
		UE_LOG(LogAAATraffic, Warning, TEXT("TrafficSpawner: No roads found by the provider."));
		return;
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

	// Cache for respawn use.
	CachedAllLanes = AllLanes;

	// Auto-place traffic signals at discovered junctions (C2).
	PlaceAutoSignals(World, Provider, AllLanes);

	// Allow VehicleCount to exceed lane count — vehicles share lanes with
	// staggered positioning via SpawnSpacing.
	const int32 SpawnCount = VehicleCount;

	UE_LOG(LogAAATraffic, Log,
		TEXT("TrafficSpawner: Spawning %d vehicles across %d available lanes."),
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

		// Spawn the signal controller at the world origin (no visible component needed).
		ATrafficSignalController* Signal = World->SpawnActor<ATrafficSignalController>();
		if (!Signal) { continue; }

		Signal->JunctionId = JId;

		// Build phase groups: each approach road gets its own green phase.
		TArray<int32> RoadKeys;
		LanesByRoad.GetKeys(RoadKeys);
		RoadKeys.Sort();

		Signal->PhaseGroups.Empty();
		for (const int32 RoadId : RoadKeys)
		{
			FSignalPhaseGroup Group;
			Group.GroupName = FString::Printf(TEXT("Road_%d"), RoadId);
			Group.GreenLanes = LanesByRoad[RoadId];
			Signal->PhaseGroups.Add(MoveTemp(Group));
		}

		++SignalsPlaced;
	}

	if (SignalsPlaced > 0)
	{
		UE_LOG(LogAAATraffic, Log,
			TEXT("TrafficSpawner: Auto-placed %d signal controllers across %d junctions."),
			SignalsPlaced, JunctionIds.Num());
	}
}
