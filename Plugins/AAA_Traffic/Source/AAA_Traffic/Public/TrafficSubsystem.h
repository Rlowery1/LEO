// Copyright AAA_Traffic Contributors. All Rights Reserved.

#pragma once

#include "CoreMinimal.h"
#include "Subsystems/WorldSubsystem.h"
#include "Engine/TimerHandle.h"
#include "TrafficRoadProvider.h"
#include "TrafficSubsystem.generated.h"

class ITrafficRoadProvider;
class ATrafficVehicleController;
class ATrafficSignalController;

/**
 * LOD tier for distance-based vehicle simulation detail.
 */
UENUM(BlueprintType)
enum class ETrafficLOD : uint8
{
	/** Full simulation: every-frame tick, physics input, sphere sweep. */
	Full,
	/** Reduced: tick every N frames, skip leader detection. */
	Reduced,
	/** Minimal: tick infrequently, teleport along polyline, skip lane changes. */
	Minimal
};

/** Broadcast when a provider registers (or re-registers) with the subsystem. */
DECLARE_MULTICAST_DELEGATE_OneParam(FOnProviderRegistered, ITrafficRoadProvider* /*Provider*/);

/** Broadcast just before a vehicle is destroyed by the despawn sweep. */
DECLARE_MULTICAST_DELEGATE_TwoParams(FOnVehicleDespawned, ATrafficVehicleController* /*Controller*/, const FTrafficLaneHandle& /*Lane*/);

/**
 * World subsystem that holds a reference to the active road provider.
 * Road-kit adapters register themselves here; the traffic system queries through here.
 *
 * Also manages vehicle lifecycle: periodically despawns vehicles that are
 * too far from any player or have reached a dead-end and stopped.
 */
UCLASS()
class AAA_TRAFFIC_API UTrafficSubsystem : public UWorldSubsystem
{
	GENERATED_BODY()

public:
	UTrafficSubsystem();

	// --- USubsystem interface ---
	virtual void Initialize(FSubsystemCollectionBase& Collection) override;
	virtual void Deinitialize() override;
	virtual bool DoesSupportWorldType(const EWorldType::Type WorldType) const override;

	/** Register a road provider (adapter). Only one active provider is supported at a time. */
	void RegisterProvider(UObject* InProvider);

	/** Unregister the current provider. */
	void UnregisterProvider(UObject* InProvider);

	/** Get the currently active road provider, or nullptr if none is registered. */
	ITrafficRoadProvider* GetProvider() const;

	/** Returns true if a provider is currently registered. */
	UFUNCTION(BlueprintCallable, Category = "Traffic")
	bool HasProvider() const;

	/** Fired when a provider registers. Listeners can use this for deferred initialization. */
	FOnProviderRegistered OnProviderRegistered;

	/** Fired just before a vehicle is destroyed by the despawn sweep. */
	FOnVehicleDespawned OnVehicleDespawned;

	/** Register an active traffic vehicle controller. */
	void RegisterVehicle(ATrafficVehicleController* InController);

	/** Unregister a traffic vehicle controller. */
	void UnregisterVehicle(ATrafficVehicleController* InController);

	/** Notify the subsystem that a vehicle has changed lanes (updates per-lane registry). */
	void UpdateVehicleLane(ATrafficVehicleController* InController, const FTrafficLaneHandle& NewLane);

	/** Get all vehicles currently on a given lane. */
	TArray<TWeakObjectPtr<ATrafficVehicleController>> GetVehiclesOnLane(const FTrafficLaneHandle& Lane) const;

	/** Get the set of all currently active vehicle controllers. */
	const TSet<TWeakObjectPtr<ATrafficVehicleController>>& GetActiveVehicles() const { return ActiveVehicles; }

	// --- Spatial Grid ---

	/** Update a vehicle's position in the spatial grid. Call from vehicle Tick. */
	void UpdateVehiclePosition(ATrafficVehicleController* Controller, const FVector& Position);

	/** Get all vehicles within Radius of Position. */
	UFUNCTION(BlueprintCallable, Category = "Traffic")
	TArray<ATrafficVehicleController*> GetNearbyVehicles(const FVector& Position, float Radius) const;

	// --- LOD ---

	/** Get the current LOD tier for a vehicle. */
	UFUNCTION(BlueprintCallable, Category = "Traffic")
	ETrafficLOD GetVehicleLOD(const ATrafficVehicleController* Controller) const;

	// --- Signal Controllers ---

	/** Register a signal controller for a junction ID. */
	void RegisterSignalController(int32 JunctionId, ATrafficSignalController* Controller);

	/** Unregister a signal controller. */
	void UnregisterSignalController(int32 JunctionId);

	/** Get the signal controller for a junction, or nullptr. */
	UFUNCTION(BlueprintCallable, Category = "Traffic")
	ATrafficSignalController* GetSignalForJunction(int32 JunctionId) const;

	// --- Junction Occupancy ---

	/** Mark a junction as occupied by a vehicle (returns false if already occupied by another). */
	bool TryOccupyJunction(int32 JunctionId, ATrafficVehicleController* Controller);

	/** Release junction occupancy for the given vehicle. */
	void ReleaseJunction(int32 JunctionId, ATrafficVehicleController* Controller);

	// --- Despawn configuration ---

	/** Maximum distance (cm) from the nearest player before a vehicle is despawned. */
	UPROPERTY(EditAnywhere, BlueprintReadWrite, Category = "Traffic|Lifecycle", meta = (ClampMin = "5000"))
	float DespawnDistance;

	/** Seconds between lifecycle sweep passes (lower = more responsive, higher = cheaper). */
	UPROPERTY(EditAnywhere, BlueprintReadWrite, Category = "Traffic|Lifecycle", meta = (ClampMin = "0.1"))
	float DespawnCheckInterval;

	/** If true, vehicles at a dead-end that have fully stopped will be despawned. */
	UPROPERTY(EditAnywhere, BlueprintReadWrite, Category = "Traffic|Lifecycle")
	bool bDespawnDeadEndVehicles;

private:
	/** Periodic lifecycle sweep: destroy vehicles out of range or stopped at dead end. */
	void PerformDespawnSweep();

	/** Timer handle for the despawn sweep. */
	FTimerHandle DespawnTimerHandle;

	/** The UObject that implements ITrafficRoadProvider. Kept as UObject* to prevent GC. */
	UPROPERTY()
	TObjectPtr<UObject> ActiveProviderObject;

	/**
	 * Registry of all active traffic vehicle controllers in the world.
	 *
	 * NOTE:
	 * - The proximity detection system currently relies on direct physics sweeps and does not
	 *   query this registry for neighbor detection.
	 * - This registry exists to support future despawn / lifecycle management and potential
	 *   debugging / analytics features.
	 *
	 * Per System.md Section 8 (minimalism), this is treated as bounded technical debt:
	 * - Keep registration / unregistration overhead minimal and free of heavy per-tick work.
	 * - If this set becomes a performance hotspot or remains unused by concrete features,
	 *   either wire it into those features or remove it instead of growing its responsibilities.
	 *
	 * Not marked UPROPERTY: TSet<TWeakObjectPtr<T>> is not supported by UHT reflection.
	 * GC safety is already guaranteed by TWeakObjectPtr, which nullifies automatically
	 * when the referenced UObject is collected.
	 */
	TSet<TWeakObjectPtr<ATrafficVehicleController>> ActiveVehicles;

	/**
	 * Per-lane vehicle registry: lane handle ID → controllers currently on that lane.
	 * Used by lane-change gap checking for deterministic neighbor queries.
	 */
	TMap<int32, TArray<TWeakObjectPtr<ATrafficVehicleController>>> VehiclesByLane;

	// --- Spatial grid ---

	/** Cell size for spatial hashing (cm). */
	static constexpr float SpatialCellSize = 5000.0f;

	/** Compute the spatial cell key for a world position. */
	static int64 ComputeCellKey(const FVector& Position);

	/** Spatial hash grid: cell key → vehicles in that cell. */
	TMap<int64, TArray<TWeakObjectPtr<ATrafficVehicleController>>> SpatialGrid;

	/** Per-vehicle cached cell key for efficient grid updates. */
	TMap<TWeakObjectPtr<ATrafficVehicleController>, int64> VehicleCellMap;

	// --- LOD ---

	/** Per-vehicle LOD tier, updated during despawn sweep. */
	TMap<TWeakObjectPtr<ATrafficVehicleController>, ETrafficLOD> VehicleLODMap;

	/** Assign LOD tiers based on player distances (called during despawn sweep). */
	void UpdateLODTiers(const TArray<FVector>& PlayerPositions);

	// --- Signal controllers ---

	/** Junction ID → signal controller. */
	TMap<int32, TWeakObjectPtr<ATrafficSignalController>> SignalControllerMap;

	// --- Junction occupancy ---

	/** Junction ID → vehicle currently traversing it. */
	TMap<int32, TWeakObjectPtr<ATrafficVehicleController>> JunctionOccupancy;
};
