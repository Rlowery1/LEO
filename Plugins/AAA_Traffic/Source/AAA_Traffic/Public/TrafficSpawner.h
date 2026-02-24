// Copyright AAA_Traffic Contributors. All Rights Reserved.

#pragma once

#include "CoreMinimal.h"
#include "GameFramework/Actor.h"
#include "TrafficSpawner.generated.h"

/**
 * Place this actor in a level to spawn traffic vehicles on available lanes.
 *
 * On BeginPlay it queries the active ITrafficRoadProvider for lanes,
 * spawns vehicles of the configured class, and assigns each one an
 * ATrafficVehicleController that drives along its lane.
 */
UCLASS()
class AAA_TRAFFIC_API ATrafficSpawner : public AActor
{
	GENERATED_BODY()

public:
	ATrafficSpawner();

protected:
	virtual void BeginPlay() override;

private:
	/** Deferred to next tick so all subsystems (including adapters) are initialized. */
	void SpawnVehicles();

	/** Vehicle pawn class to spawn (e.g. a DD_Vehicles Blueprint). */
	UPROPERTY(EditAnywhere, Category = "Traffic")
	TSubclassOf<APawn> VehicleClass;

	/** Number of vehicles to spawn. Capped to available lane count. */
	UPROPERTY(EditAnywhere, Category = "Traffic", meta = (ClampMin = "1", ClampMax = "100"))
	int32 VehicleCount;

	/** Target speed for spawned vehicles (cm/s). */
	UPROPERTY(EditAnywhere, Category = "Traffic", meta = (ClampMin = "0"))
	float VehicleSpeed;

	/** Seed for deterministic spawn decisions (lane assignment, ordering). */
	UPROPERTY(EditAnywhere, Category = "Traffic")
	int32 SpawnSeed;

	/** Vertical offset added to lane start position when spawning (cm). */
	UPROPERTY(EditAnywhere, Category = "Traffic", meta = (ClampMin = "0"))
	float SpawnZOffset;
};
