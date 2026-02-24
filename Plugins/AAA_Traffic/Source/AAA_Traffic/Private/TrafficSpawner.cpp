// Copyright AAA_Traffic Contributors. All Rights Reserved.

#include "TrafficSpawner.h"
#include "TrafficSubsystem.h"
#include "TrafficRoadProvider.h"
#include "TrafficVehicleController.h"
#include "Engine/World.h"
#include "TimerManager.h"

ATrafficSpawner::ATrafficSpawner()
	: VehicleCount(1)
	, VehicleSpeed(1500.f)
	, SpawnSeed(42)
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
		UE_LOG(LogTemp, Error, TEXT("TrafficSpawner: No VehicleClass assigned."));
		return;
	}

	UTrafficSubsystem* TrafficSub = GetWorld()->GetSubsystem<UTrafficSubsystem>();
	ITrafficRoadProvider* Provider = TrafficSub ? TrafficSub->GetProvider() : nullptr;
	if (!Provider)
	{
		UE_LOG(LogTemp, Error,
			TEXT("TrafficSpawner: No road provider registered. Is a road-kit adapter module loaded?"));
		return;
	}

	// Gather all lanes across all roads.
	TArray<FTrafficRoadHandle> Roads = Provider->GetAllRoads();
	if (Roads.IsEmpty())
	{
		UE_LOG(LogTemp, Warning, TEXT("TrafficSpawner: No roads found by the provider."));
		return;
	}

	TArray<FTrafficLaneHandle> AllLanes;
	for (const FTrafficRoadHandle& Road : Roads)
	{
		AllLanes.Append(Provider->GetLanesForRoad(Road));
	}

	if (AllLanes.IsEmpty())
	{
		UE_LOG(LogTemp, Warning, TEXT("TrafficSpawner: No lanes found on any road."));
		return;
	}

	FRandomStream SpawnRandom(SpawnSeed);
	const int32 SpawnCount = FMath::Min(VehicleCount, AllLanes.Num());

	UE_LOG(LogTemp, Log,
		TEXT("TrafficSpawner: Spawning %d vehicles across %d available lanes."),
		SpawnCount, AllLanes.Num());

	for (int32 i = 0; i < SpawnCount; ++i)
	{
		const FTrafficLaneHandle& Lane = AllLanes[i % AllLanes.Num()];

		TArray<FVector> LanePoints;
		float LaneWidth;
		if (!Provider->GetLanePath(Lane, LanePoints, LaneWidth) || LanePoints.Num() < 2)
		{
			UE_LOG(LogTemp, Warning,
				TEXT("TrafficSpawner: Could not get path for lane %d — skipping."),
				Lane.HandleId);
			continue;
		}

		// Spawn at lane start, oriented along the lane.
		const FVector SpawnLocation = LanePoints[0] + FVector(0.0, 0.0, 50.0);
		const FRotator SpawnRotation = (LanePoints[1] - LanePoints[0]).Rotation();
		const FTransform SpawnTransform(SpawnRotation, SpawnLocation);

		FActorSpawnParameters SpawnParams;
		SpawnParams.SpawnCollisionHandlingOverride =
			ESpawnActorCollisionHandlingMethod::AdjustIfPossibleButAlwaysSpawn;

		APawn* Vehicle = GetWorld()->SpawnActor<APawn>(VehicleClass, SpawnTransform, SpawnParams);
		if (!Vehicle)
		{
			UE_LOG(LogTemp, Warning,
				TEXT("TrafficSpawner: Failed to spawn vehicle %d."), i);
			continue;
		}

		// Spawn our AI controller and possess the vehicle.
		ATrafficVehicleController* Controller =
			GetWorld()->SpawnActor<ATrafficVehicleController>();
		if (Controller)
		{
			Controller->Possess(Vehicle);
			Controller->InitializeLaneFollowing(Lane);

			UE_LOG(LogTemp, Log,
				TEXT("TrafficSpawner: Vehicle %d spawned on lane %d."),
				i, Lane.HandleId);
		}
	}
}
