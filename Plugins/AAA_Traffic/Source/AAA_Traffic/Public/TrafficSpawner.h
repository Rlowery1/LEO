// Copyright AAA_Traffic Contributors. All Rights Reserved.

#pragma once

#include "CoreMinimal.h"
#include "GameFramework/Actor.h"
#include "DrawDebugHelpers.h"
#include "TrafficSpawner.generated.h"

class ITrafficRoadProvider;
class ATrafficVehicleController;
struct FTrafficLaneHandle;

/**
 * Entry in the vehicle class table for varied spawning.
 * When the spawner's VehicleClasses array is non-empty, a class is picked
 * per vehicle using weighted random selection with FRandomStream (deterministic).
 */
USTRUCT(BlueprintType)
struct FVehicleClassEntry
{
	GENERATED_BODY()

	/** Vehicle pawn class to spawn. */
	UPROPERTY(EditAnywhere, BlueprintReadWrite, Category = "Traffic")
	TSubclassOf<APawn> VehicleClass;

	/** Relative weight for selection probability. Higher = more likely. */
	UPROPERTY(EditAnywhere, BlueprintReadWrite, Category = "Traffic", meta = (ClampMin = "0.01"))
	float Weight = 1.0f;
};

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
UCLASS(Blueprintable)
class AAA_TRAFFIC_API ATrafficSpawner : public AActor
{
	GENERATED_BODY()

	friend class FTrafficSpawnerCustomization;

public:
	ATrafficSpawner();

	virtual void Tick(float DeltaSeconds) override;
	virtual bool ShouldTickIfViewportsOnly() const override;

protected:
	virtual void BeginPlay() override;
	virtual void EndPlay(const EEndPlayReason::Type EndPlayReason) override;

private:
	/** Deferred to next tick so all subsystems (including adapters) are initialized. */
	void SpawnVehicles();

	/**
	 * Spawn a single vehicle on the given lane at a position along it.
	 * Used by both initial spawn and respawn logic.
	 * @param World        World context.
	 * @param Provider     Active road provider.
	 * @param Lane         Lane to spawn on.
	 * @param SlotIndex    Position along the lane (multiplied by SpawnSpacing).
	 * @param VehicleIndex Index for seed generation.
	 */
	void SpawnSingleVehicle(UWorld* World, ITrafficRoadProvider* Provider,
		const FTrafficLaneHandle& Lane, int32 SlotIndex, int32 VehicleIndex);

	/** Called when a provider registers after BeginPlay (deferred retry). */
	void OnProviderRegistered(ITrafficRoadProvider* Provider);

	/** Called when a vehicle is despawned — schedules a respawn if enabled. */
	void OnVehicleDespawned(ATrafficVehicleController* Controller, const FTrafficLaneHandle& Lane);

	/** Periodic check: if we're below VehicleCount, spawn a replacement. */
	void CheckRespawn();

	/** Select a vehicle class for the given vehicle index using weighted random. */
	TSubclassOf<APawn> SelectVehicleClass(int32 VehicleIndex) const;

	/** Auto-discover junctions and spawn ATrafficSignalController actors. */
	void PlaceAutoSignals(UWorld* World, ITrafficRoadProvider* Provider, const TArray<FTrafficLaneHandle>& AllLanes);

	/** True once SpawnVehicles has successfully completed. */
	bool bSpawnComplete = false;

	/** Counter incremented each time a vehicle is respawned (used for deterministic seeding). */
	int32 RespawnCounter = 0;

	/** Total number of vehicles ever spawned (initial + respawns), for unique seed assignment. */
	int32 TotalVehiclesSpawned = 0;

	/** Lanes pending respawn (queued by OnVehicleDespawned, consumed by CheckRespawn). */
	TArray<FTrafficLaneHandle> RespawnQueue;

	/** Timer handle for the periodic respawn check. */
	FTimerHandle RespawnTimerHandle;

	/** Cached list of all available lanes (populated once during initial spawn). */
	TArray<FTrafficLaneHandle> CachedAllLanes;

	/** Controllers spawned by this spawner, for filtering shared despawn delegate. */
	TSet<TWeakObjectPtr<ATrafficVehicleController>> OwnedVehicles;

#if ENABLE_DRAW_DEBUG // ── Debug internals (no UPROPERTYs here) ──
	/** Cache lane polylines from the provider so debug draw doesn't re-query every frame. */
	void CacheDebugLaneData();

	/**
	 * Fallback: discover lane geometry via UE reflection when no compiled adapter
	 * is available (e.g. precompiled RoadBLD without source headers).
	 */
	void CacheDebugLaneDataViaReflection(UWorld* World);

	/** Draw cached lane centerlines using DrawDebugLine. */
	void DrawDebugLanes() const;

	/** Whether the lane cache has been populated. */
	bool bDebugCacheReady = false;

	/** Whether a cache attempt has been made (prevents re-querying every frame on failure). */
	bool bDebugCacheAttempted = false;

	/** Cached per-lane polylines for debug rendering. */
	struct FDebugLaneData
	{
		TArray<FVector> Points;
		bool bIsReverseLane = false;
	};
	TArray<FDebugLaneData> DebugLanes;

	// ── Intersection debug data ──

	/** Cache lane endpoints, connections, and junctions from the provider. */
	void CacheDebugIntersectionData();

	/** Draw intersection visualizations (connections, endpoints, junctions). */
	void DrawDebugIntersections() const;

	/** Whether the intersection cache has been populated. */
	bool bIntersectionCacheReady = false;

	/** Whether an intersection cache attempt has been made. */
	bool bIntersectionCacheAttempted = false;

	/** Cached lane endpoint positions for sphere drawing. */
	struct FDebugEndpointData
	{
		FVector StartPos = FVector::ZeroVector;
		FVector EndPos = FVector::ZeroVector;
	};
	TArray<FDebugEndpointData> DebugEndpoints;

	/** Cached connection between two lane endpoints. */
	struct FDebugConnectionData
	{
		FVector From = FVector::ZeroVector;  // End of source lane.
		FVector To = FVector::ZeroVector;    // Start of destination lane.
	};
	TArray<FDebugConnectionData> DebugConnections;

	/** Cached junction centroid with ID for label drawing. */
	struct FDebugJunctionData
	{
		FVector Centroid = FVector::ZeroVector;
		int32 JunctionId = 0;
	};
	TArray<FDebugJunctionData> DebugJunctions;
#endif // ENABLE_DRAW_DEBUG

protected:
	// ── Traffic Config ──────────────────────────────────────────────

	/** Vehicle pawn class to spawn (e.g. a DD_Vehicles Blueprint). Used when VehicleClasses is empty. */
	UPROPERTY(EditAnywhere, BlueprintReadWrite, Category = "Traffic")
	TSubclassOf<APawn> VehicleClass;

	/**
	 * Array of vehicle classes with weights for varied spawning.
	 * When non-empty, overrides VehicleClass. The spawner picks a class per
	 * vehicle via weighted random using SpawnSeed for determinism.
	 */
	UPROPERTY(EditAnywhere, BlueprintReadWrite, Category = "Traffic")
	TArray<FVehicleClassEntry> VehicleClasses;

	/** Number of vehicles to spawn. May exceed lane count (vehicles share lanes via SpawnSpacing). */
	UPROPERTY(EditAnywhere, BlueprintReadWrite, Category = "Traffic", meta = (ClampMin = "1"))
	int32 VehicleCount;

	/** Target speed for spawned vehicles (cm/s). */
	UPROPERTY(EditAnywhere, BlueprintReadWrite, Category = "Traffic", meta = (ClampMin = "0"))
	float VehicleSpeed;

	/** Seed for deterministic spawn decisions (lane assignment, ordering). */
	UPROPERTY(EditAnywhere, BlueprintReadWrite, Category = "Traffic")
	int32 SpawnSeed;

	/** Vertical offset added to lane start position when spawning (cm). */
	UPROPERTY(EditAnywhere, BlueprintReadWrite, Category = "Traffic", meta = (ClampMin = "0"))
	float SpawnZOffset;

	/** Spacing between vehicles spawned on the same lane (cm). */
	UPROPERTY(EditAnywhere, BlueprintReadWrite, Category = "Traffic", meta = (ClampMin = "200"))
	float SpawnSpacing;

	/**
	 * Maximum percentage of speed variation applied per vehicle (0-100).
	 * Each vehicle gets a deterministic random offset in [-SpeedVariation, +SpeedVariation]%
	 * of VehicleSpeed, using SpawnSeed for reproducibility.
	 */
	UPROPERTY(EditAnywhere, BlueprintReadWrite, Category = "Traffic", meta = (ClampMin = "0", ClampMax = "100"))
	float SpeedVariation;

	// ── Lane Change ────────────────────────────────────────────────

	/**
	 * Lane-change aggression (0-1).
	 * 0 = conservative (long blend, high cooldown, strict gap).
	 * 1 = aggressive (short blend, low cooldown, loose gap).
	 * Maps to internal controller tuning knobs automatically.
	 */
	UPROPERTY(EditAnywhere, BlueprintReadWrite, Category = "Traffic|Lane Change", meta = (ClampMin = "0", ClampMax = "1"))
	float LaneChangeAggression;

	// ── Respawn ────────────────────────────────────────────────────

	/** If true, despawned vehicles are replaced to maintain VehicleCount. */
	UPROPERTY(EditAnywhere, BlueprintReadWrite, Category = "Traffic|Respawn")
	bool bEnableRespawn;

	/** Seconds between respawn checks. */
	UPROPERTY(EditAnywhere, BlueprintReadWrite, Category = "Traffic|Respawn", meta = (ClampMin = "0.5", EditCondition = "bEnableRespawn"))
	float RespawnCheckInterval;

	/**
	 * Minimum distance (cm) from any player for a respawn location to be valid.
	 * Prevents vehicles popping in within the player's field of view.
	 */
	UPROPERTY(EditAnywhere, BlueprintReadWrite, Category = "Traffic|Respawn", meta = (ClampMin = "1000", EditCondition = "bEnableRespawn"))
	float MinRespawnDistance;

	/** Default speed limit (cm/s) used if the provider returns no per-lane speed data. */
	UPROPERTY(EditAnywhere, BlueprintReadWrite, Category = "Traffic", meta = (ClampMin = "0"))
	float DefaultSpeedLimit;

	/** Automatically spawn traffic signal controllers at discovered junctions. */
	UPROPERTY(EditAnywhere, BlueprintReadWrite, Category = "Traffic")
	bool bAutoPlaceSignals = true;

	// ── Debug ───────────────────────────────────────────────────────

	/**
	 * When true (in non-Shipping builds), draws sampled lane centerlines in the viewport during PIE.
	 * Forward-direction lanes are drawn in cyan, reverse lanes in orange.
	 * In Shipping builds this flag has no effect because the debug drawing code is compiled out
	 * via ENABLE_DRAW_DEBUG.
	 */
	UPROPERTY(EditAnywhere, Category = "Traffic|Debug")
	bool bDebugDrawLanes = false;

	/**
	 * When true (in non-Shipping builds), draws intersection debug overlays:
	 * lane endpoint markers (blue=start, red=end), connectivity arrows (yellow),
	 * and junction centroid spheres with ID labels (white).
	 */
	UPROPERTY(EditAnywhere, Category = "Traffic|Debug")
	bool bDebugDrawIntersections = false;
};
