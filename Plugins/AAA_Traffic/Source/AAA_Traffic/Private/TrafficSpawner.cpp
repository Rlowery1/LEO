// Copyright AAA_Traffic Contributors. All Rights Reserved.

#include "TrafficSpawner.h"
#include "TrafficSubsystem.h"
#include "TrafficRoadProvider.h"
#include "TrafficVehicleController.h"
#include "TrafficLog.h"
#include "Engine/World.h"
#include "TimerManager.h"

ATrafficSpawner::ATrafficSpawner()
	: VehicleCount(1)
	, VehicleSpeed(1500.f)
	, SpawnSeed(42)
	, SpawnZOffset(50.f)
	, SpawnSpacing(1500.f)
	, SpeedVariation(15.f)
{
	PrimaryActorTick.bCanEverTick = false;
}

void ATrafficSpawner::BeginPlay()
{
	Super::BeginPlay();

	// Defer to next tick so adapter subsystems have finished OnWorldBeginPlay registration.
	GetWorldTimerManager().SetTimerForNextTick(this, &ATrafficSpawner::SpawnVehicles);
}

void ATrafficSpawner::SpawnVehicles()
{
	if (!VehicleClass)
	{
		UE_LOG(LogAAATraffic, Error, TEXT("TrafficSpawner: No VehicleClass assigned."));
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
		UE_LOG(LogAAATraffic, Error,
			TEXT("TrafficSpawner: No road provider registered. Is a road-kit adapter module loaded?"));
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

		TArray<FVector> LanePoints;
		float LaneWidth;
		if (!Provider->GetLanePath(Lane, LanePoints, LaneWidth) || LanePoints.Num() < 2)
		{
			UE_LOG(LogAAATraffic, Warning,
				TEXT("TrafficSpawner: Could not get path for lane %d — skipping."),
				Lane.HandleId);
			continue;
		}

		// Stagger spawn position along the lane based on how many vehicles already placed on it.
		const int32 SlotIndex = LaneOccupancy.FindOrAdd(Lane.HandleId, 0);
		LaneOccupancy[Lane.HandleId] = SlotIndex + 1;
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

		// If the requested offset exceeds the lane length, clamp to the lane end
		// instead of leaving SpawnPos/SpawnDir at the lane start.
		if (TargetOffset > AccumulatedDist && LanePoints.Num() >= 2)
		{
			const int32 LastIdx = LanePoints.Num() - 1;
			SpawnPos = LanePoints[LastIdx];
			SpawnDir = (LanePoints[LastIdx] - LanePoints[LastIdx - 1]).GetSafeNormal();

			UE_LOG(LogAAATraffic, Warning,
				TEXT("TrafficSpawner: TargetOffset %.2f exceeds length %.2f of lane %d, clamping to lane end."),
				TargetOffset, AccumulatedDist, Lane.HandleId);
		}

		const FVector SpawnLocation = SpawnPos + FVector(0.0, 0.0, SpawnZOffset);
		const FRotator SpawnRotation = SpawnDir.Rotation();
		const FTransform SpawnTransform(SpawnRotation, SpawnLocation);

		FActorSpawnParameters SpawnParams;
		SpawnParams.SpawnCollisionHandlingOverride =
			ESpawnActorCollisionHandlingMethod::AdjustIfPossibleButAlwaysSpawn;

		APawn* Vehicle = World->SpawnActor<APawn>(VehicleClass, SpawnTransform, SpawnParams);
		if (!Vehicle)
		{
			UE_LOG(LogAAATraffic, Warning,
				TEXT("TrafficSpawner: Failed to spawn vehicle %d."), i);
			continue;
		}

		// Spawn our AI controller and possess the vehicle.
		ATrafficVehicleController* Controller =
			World->SpawnActor<ATrafficVehicleController>();
		if (Controller)
		{
			Controller->SetRandomSeed(SpawnSeed + i);

			// Apply deterministic per-vehicle speed variation.
			float FinalSpeed = VehicleSpeed;
			if (SpeedVariation > KINDA_SMALL_NUMBER)
			{
				// Offset keeps this RNG stream disjoint from the controller RNG (SpawnSeed + i),
				// improving seed hygiene and avoiding accidental cross-stream collisions.
				constexpr int32 SpeedVariationSeedOffset = 10000;
				FRandomStream SpeedRng(SpawnSeed + i + SpeedVariationSeedOffset);
				const float VariationFraction = SpeedVariation / 100.0f;
				const float Offset = SpeedRng.FRandRange(-VariationFraction, VariationFraction);
				FinalSpeed = VehicleSpeed * (1.0f + Offset);
				FinalSpeed = FMath::Max(FinalSpeed, 0.0f);
			}
			Controller->SetTargetSpeed(FinalSpeed);

			Controller->Possess(Vehicle);
			Controller->InitializeLaneFollowing(Lane);

			UE_LOG(LogAAATraffic, Log,
				TEXT("TrafficSpawner: Vehicle %d spawned on lane %d."),
				i, Lane.HandleId);
		}
	}
}
