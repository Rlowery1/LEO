// Copyright AAA_Traffic Contributors. All Rights Reserved.

#pragma once

#include "CoreMinimal.h"
#include "GameFramework/Actor.h"
#include "DrawDebugHelpers.h"
#include "TrafficRoadProvider.h"
#include "TrafficSpawner.generated.h"

/**
 * Place this actor in a level to spawn traffic vehicles on available lanes.
 *
 * On BeginPlay it queries the active ITrafficRoadProvider for lanes,
 * spawns vehicles of the configured class, and assigns each one an
 * ATrafficVehicleController that drives along its lane.
 *
 * Enable bDebugDrawLanes to visualize the sampled lane centerlines during
 * Play-In-Editor — useful for verifying that the adapter path data aligns
 * with the painted road markings.
 */
UCLASS()
class AAA_TRAFFIC_API ATrafficSpawner : public AActor
{
	GENERATED_BODY()

public:
	ATrafficSpawner();

	virtual void Tick(float DeltaSeconds) override;

protected:
	virtual void BeginPlay() override;

private:
	/** Deferred to next tick so all subsystems (including adapters) are initialized. */
	void SpawnVehicles();

#if ENABLE_DRAW_DEBUG
	/** Cache lane polylines from the provider so debug draw doesn't re-query every frame. */
	void CacheDebugLaneData();

	/** Draw cached lane centerlines using DrawDebugLine. */
	void DrawDebugLanes() const;

	/** Whether the lane cache has been populated. */
	bool bDebugCacheReady = false;

	/** Cached per-lane polylines for debug rendering. */
	struct FDebugLaneData
	{
		TArray<FVector> Points;
		bool bIsReverseLane = false;
	};
	TArray<FDebugLaneData> DebugLanes;
#endif // ENABLE_DRAW_DEBUG

	// ── Traffic Config ──────────────────────────────────────────────

	/** Vehicle pawn class to spawn (e.g. a DD_Vehicles Blueprint). */
	UPROPERTY(EditAnywhere, Category = "Traffic")
	TSubclassOf<APawn> VehicleClass;

	/** Number of vehicles to spawn. May exceed lane count (vehicles share lanes via SpawnSpacing). */
	UPROPERTY(EditAnywhere, Category = "Traffic", meta = (ClampMin = "1"))
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

	/** Spacing between vehicles spawned on the same lane (cm). */
	UPROPERTY(EditAnywhere, Category = "Traffic", meta = (ClampMin = "200"))
	float SpawnSpacing;

	/**
	 * Maximum percentage of speed variation applied per vehicle (0-100).
	 * Each vehicle gets a deterministic random offset in [-SpeedVariation, +SpeedVariation]%
	 * of VehicleSpeed, using SpawnSeed for reproducibility.
	 */
	UPROPERTY(EditAnywhere, Category = "Traffic", meta = (ClampMin = "0", ClampMax = "100"))
	float SpeedVariation;

	// ── Debug ───────────────────────────────────────────────────────

	/**
	 * When true, draws sampled lane centerlines in the viewport during PIE.
	 * Forward-direction lanes are drawn in cyan, reverse lanes in orange.
	 * Stripped from Shipping builds automatically.
	 */
	UPROPERTY(EditAnywhere, Category = "Traffic|Debug")
	bool bDebugDrawLanes = false;
};
