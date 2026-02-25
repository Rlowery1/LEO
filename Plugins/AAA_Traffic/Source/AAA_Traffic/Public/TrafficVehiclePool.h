// Copyright AAA_Traffic Contributors. All Rights Reserved.

#pragma once

#include "CoreMinimal.h"
#include "UObject/NoExportTypes.h"
#include "TrafficVehiclePool.generated.h"

/**
 * Object pool for traffic vehicle pawns.
 *
 * Instead of destroying despawned vehicles, they are deactivated and cached.
 * On the next spawn request for the same class, a pooled pawn is reactivated
 * and repositioned — avoiding the cost of full actor construction.
 *
 * Owned by UTrafficSubsystem (world-scoped lifecycle).
 * Determinism: pool acquisition order is FIFO per class (last-released first).
 */
UCLASS()
class AAA_TRAFFIC_API UTrafficVehiclePool : public UObject
{
	GENERATED_BODY()

public:
	/**
	 * Try to acquire a previously pooled pawn of the given class.
	 * If available, hides/deactivates state is reversed (made visible + active)
	 * and the pawn is teleported to SpawnTransform.
	 *
	 * @return A reactivated pawn, or nullptr if the pool has none for this class.
	 */
	APawn* AcquireVehicle(UWorld* World, TSubclassOf<APawn> VehicleClass, const FTransform& SpawnTransform);

	/**
	 * Return a pawn to the pool instead of destroying it.
	 * The pawn is hidden, collision disabled, and ticking paused.
	 */
	void ReleaseVehicle(APawn* Vehicle);

	/** Get the total number of pooled (inactive) pawns across all classes. */
	int32 GetPooledCount() const;

	/** Destroy all pooled actors (called during subsystem teardown). */
	void DrainPool();

private:
	/** Per-class pool of inactive pawns. */
	TMap<UClass*, TArray<TWeakObjectPtr<APawn>>> Pool;
};
