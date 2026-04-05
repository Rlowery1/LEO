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
class UTrafficVehiclePool;
struct FCanonicalMovementRecord;

// Forward-declare ETurnSignalState (defined in TrafficVehicleController.h).
enum class ETurnSignalState : uint8;

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

enum class ECanonicalMovementClass : uint8
{
	Straight,
	Left,
	Right,
	Merge,
	Diverge,
	SlipLane,
	Unknown
};

enum class ECanonicalMovementSourceKind : uint8
{
	ProviderDerived,
	SynthesizedDuringCompile
};

/** Broadcast when a provider registers (or re-registers) with the subsystem. */
DECLARE_MULTICAST_DELEGATE_OneParam(FOnProviderRegistered, ITrafficRoadProvider* /*Provider*/);

/** Broadcast just before a vehicle is destroyed by the despawn sweep. */
DECLARE_MULTICAST_DELEGATE_TwoParams(FOnVehicleDespawned, ATrafficVehicleController* /*Controller*/, const FTrafficLaneHandle& /*Lane*/);

/**
 * Surveyed legal exit from a junction approach lane.
 * Produced once at startup by BuildJunctionSurvey(); consumed by vehicles
 * instead of per-tick ad-hoc filtering.
 */
struct FJunctionExitRule
{
	/** The exit lane (post-junction free-flow lane). */
	FTrafficLaneHandle ExitLane;

	/** Turn direction relative to the approach lane (set by BuildJunctionSurvey). */
	ETurnSignalState TurnDirection;

	/** Selection weight (higher = more desirable). 0 = should-not-pick. */
	float Weight = 1.0f;

	/** False if the vehicle physically cannot make this turn (too tight for lane width). */
	bool bPhysicallyFeasible = true;
};

struct FCanonicalMovementRecord
{
	int32 MovementId = 0;
	int32 JunctionId = 0;
	FTrafficLaneHandle FromLane;
	FTrafficLaneHandle ToLane;
	FTrafficLaneHandle ApproachJunctionLane;
	ETurnSignalState TurnDirection{};
	ECanonicalMovementClass MovementClass = ECanonicalMovementClass::Unknown;
	float SelectionWeight = 0.0f;
	bool bLegallyAllowed = false;
	bool bPhysicallyFeasible = false;
	TArray<FVector> CorridorPoints;
	FVector CorridorEntryPoint = FVector::ZeroVector;
	FVector CorridorExitPoint = FVector::ZeroVector;
	int32 EntryLaneAttachIndex = INDEX_NONE;
	int32 ExitLaneAttachIndex = INDEX_NONE;
	float CorridorArcLengthCm = 0.0f;
	float MinTurnRadiusCm = 0.0f;
	int32 TraversalReleaseIndex = INDEX_NONE;
	int32 ExitLaneResumeIndex = INDEX_NONE;
	TArray<int32> ConflictMovementIds;
	bool bHasProximityConflicts = false;
	uint32 ValidationFlags = 0;
	ECanonicalMovementSourceKind SourceKind = ECanonicalMovementSourceKind::ProviderDerived;
#if !UE_BUILD_SHIPPING
	FString BuildNotes;
#endif

	bool IsValid() const
	{
		return MovementId > 0
			&& JunctionId > 0
			&& FromLane.IsValid()
			&& ToLane.IsValid();
	}
	};

/**
 * Record of a vehicle currently traversing a junction.
 * Stores the approach and exit lanes so the conflict test can determine
 * whether two simultaneous movements would geometrically cross.
 */
struct FJunctionOccupant
{
	TWeakObjectPtr<ATrafficVehicleController> Controller;
	int32 MovementId = 0;
	FTrafficLaneHandle FromLane;
	FTrafficLaneHandle ToLane;

	/** Simulation time when occupancy was granted (UWorld::GetTimeSeconds()).
	 *  Used to evict stuck vehicles that never released their occupancy. */
	double OccupiedAtTime = 0.0;
};

/**
 * Record of a vehicle waiting at a stop-sign/flashing-red junction.
 * Used for FIFO ordering: first to complete their mandatory stop,
 * first to proceed. Tracks arrival time for deterministic tie-break.
 */
struct FStopSignArrival
{
	TWeakObjectPtr<ATrafficVehicleController> Controller;
	/** Simulation time when the vehicle completed its mandatory stop.
	 *  Deterministic: sourced from UWorld::GetTimeSeconds(). */
	double ArrivalTime = 0.0;
};

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
	const TArray<TWeakObjectPtr<ATrafficVehicleController>>& GetActiveVehicles() const { return ActiveVehicles; }

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

	/**
	 * Attempt to enter a junction using a precompiled canonical movement.
	 * Returns false if any existing occupant holds a conflicting movement.
	 */
	bool TryOccupyJunction(int32 JunctionId, ATrafficVehicleController* Controller,
		int32 MovementId);

	/** Release junction occupancy for the given vehicle. */
	void ReleaseJunction(int32 JunctionId, ATrafficVehicleController* Controller);

	/** Returns true if the given controller currently has an occupancy entry for JunctionId. */
	bool HasJunctionOccupancy(int32 JunctionId, const ATrafficVehicleController* Controller) const;

	/** Returns true if any vehicle currently occupies the given junction. */
	bool IsJunctionOccupied(int32 JunctionId) const;

	/**
	 * Check whether any other vehicle approaching the same junction has a
	 * conflicting path AND is going straight (higher priority than a turner).
	 * Used by left-turning vehicles at signalized intersections to yield to
	 * oncoming straight-through traffic.
	 *
	 * @return true if a higher-priority conflicting approach exists (caller should yield).
	 */
	bool HasConflictingApproach(int32 JunctionId, ATrafficVehicleController* Self,
		int32 MovementId) const;

	/**
	 * Check if any vehicle approaching this junction from a conflicting direction
	 * is close enough and fast enough to make entry unsafe (gap acceptance).
	 * Used at yield signs for ALL movements (not just left turns).
	 *
	 * @param GapThresholdSec  Minimum acceptable time-gap (seconds). Default 4.0s.
	 * @return true if cross-traffic is too close (caller should wait).
	 */
	bool HasApproachingCrossTraffic(int32 JunctionId, ATrafficVehicleController* Self,
		int32 MovementId,
		float GapThresholdSec = 4.0f) const;

	// --- Junction Exit Survey ---

	/**
	 * Get the pre-computed legal exits for an approach lane.
	 * Returns an empty array if the lane was not surveyed (e.g. not an approach lane).
	 */
	const TArray<FJunctionExitRule>& GetLegalExits(int32 ApproachLaneId) const;

	/** Get a canonical movement by stable MovementId. */
	const FCanonicalMovementRecord* GetCanonicalMovement(int32 MovementId) const;

	/** Get the sorted movement IDs compiled for a junction. */
	const TArray<int32>& GetCanonicalMovementsForJunction(int32 JunctionId) const;

	/** Get the sorted movement IDs compiled for an approach lane. */
	const TArray<int32>& GetCanonicalMovementsForApproachLane(int32 ApproachLaneId) const;

	/** Get read-only access to all canonical movements (for debug drawing). */
	const TMap<int32, FCanonicalMovementRecord>& GetAllCanonicalMovements() const { return CanonicalMovementTable; }

	/** Rebuild the junction survey and canonical movement table using the current provider state. */
	void RebuildJunctionSurvey();

	// --- Stop-Sign FIFO Queue ---

	/** Record that a vehicle has completed its mandatory stop at a stop-sign junction.
	 *  Entries are time-ordered; first to arrive = first to proceed. */
	void RecordStopSignArrival(int32 JunctionId, ATrafficVehicleController* Controller);

	/** Check if this vehicle is at the front of the stop-sign arrival queue.
	 *  Returns true if the vehicle may proceed (first-arrived-first-served). */
	bool IsStopSignTurnToGo(int32 JunctionId, ATrafficVehicleController* Controller) const;

	/** Remove a vehicle from the stop-sign arrival queue (called when occupancy is granted). */
	void RemoveStopSignArrival(int32 JunctionId, ATrafficVehicleController* Controller);

	// --- Road Speed Limits ---

	/** Set a speed limit for a specific road (pushed by the spawner during setup). */
	void SetRoadSpeedLimit(int32 RoadHandleId, float SpeedLimit);

	/** Get the speed limit for a road. Returns -1 if no override exists. */
	float GetRoadSpeedLimit(int32 RoadHandleId) const;

	// --- Vehicle Pool ---

	/** Get the vehicle object pool (used by spawner and despawn sweep). */
	UTrafficVehiclePool* GetVehiclePool() const { return VehiclePool; }

	// --- Safety Despawn ---

	/**
	 * Request immediate despawn of a vehicle for safety reasons (flipped,
	 * stuck, runaway speed). The actual destruction is deferred to next tick
	 * to avoid invalidating iterators or destroying objects mid-Tick.
	 */
	void RequestDespawn(ATrafficVehicleController* Controller, const FString& Reason);

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

	// --- LOD configuration ---

	/** Distance (cm) at which a vehicle is promoted to Full LOD (closest to player). */
	UPROPERTY(EditAnywhere, BlueprintReadWrite, Category = "Traffic|LOD", meta = (ClampMin = "1000"))
	float FullLODPromoteDistance = 10000.0f;

	/** Distance (cm) beyond which a vehicle is demoted from Full LOD (hysteresis band). */
	UPROPERTY(EditAnywhere, BlueprintReadWrite, Category = "Traffic|LOD", meta = (ClampMin = "1000"))
	float FullLODDemoteDistance = 11000.0f;

	/** Distance (cm) at which a vehicle is promoted to Reduced LOD. */
	UPROPERTY(EditAnywhere, BlueprintReadWrite, Category = "Traffic|LOD", meta = (ClampMin = "5000"))
	float ReducedLODPromoteDistance = 30000.0f;

	/** Distance (cm) beyond which a vehicle is demoted to Minimal LOD (hysteresis band). */
	UPROPERTY(EditAnywhere, BlueprintReadWrite, Category = "Traffic|LOD", meta = (ClampMin = "5000"))
	float ReducedLODDemoteDistance = 33000.0f;

	/** Empty array returned by GetLegalExits when no data exists. */
	static const TArray<FJunctionExitRule> EmptyExitRules;

	/** Empty movement-id array returned when no canonical movement data exists. */
	static const TArray<int32> EmptyMovementIds;

private:
	/** Periodic lifecycle sweep: destroy vehicles out of range or stopped at dead end. */
	void PerformDespawnSweep();

	/** Timer handle for the despawn sweep. */
	FTimerHandle DespawnTimerHandle;

	/** The UObject that implements ITrafficRoadProvider. Kept as UObject* to prevent GC. */
	UPROPERTY()
	TObjectPtr<UObject> ActiveProviderObject;

	/** Vehicle object pool for recycling despawned pawns (I1). */
	UPROPERTY()
	TObjectPtr<UTrafficVehiclePool> VehiclePool;

	/**
	 * Registry of all active traffic vehicle controllers in the world.
	 * Stored as TArray ordered by DeterministicSpawnIndex for stable iteration.
	 *
	 * Not marked UPROPERTY: TArray<TWeakObjectPtr<T>> is not supported by UHT reflection.
	 * GC safety is already guaranteed by TWeakObjectPtr, which nullifies automatically
	 * when the referenced UObject is collected.
	 */
	TArray<TWeakObjectPtr<ATrafficVehicleController>> ActiveVehicles;

	/** Monotonically increasing counter for deterministic vehicle ordering. */
	int32 NextSpawnIndex = 0;

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

	/** Junction ID → vehicles currently traversing it (multi-vehicle). */
	TMap<int32, TArray<FJunctionOccupant>> JunctionOccupancy;

	// --- Stop-sign FIFO queues ---

	/** Junction ID → ordered list of vehicles that completed their mandatory stop.
	 *  First entry = first to arrive = first to proceed. */
	TMap<int32, TArray<FStopSignArrival>> StopSignQueues;

	// --- Road speed limits ---

	/** Road handle ID → speed limit (cm/s), pushed by the spawner. */
	TMap<int32, float> RoadSpeedLimits;

	// --- Junction exit survey ---

	/** Approach lane HandleId → array of legal exits (built once at provider registration). */
	TMap<int32, TArray<FJunctionExitRule>> JunctionExitRules;

	/** Stable movement id → canonical movement record. */
	TMap<int32, FCanonicalMovementRecord> CanonicalMovementTable;

	/** Junction id → sorted movement ids. */
	TMap<int32, TArray<int32>> JunctionToMovementIds;

	/** Approach lane id → sorted movement ids. */
	TMap<int32, TArray<int32>> ApproachLaneToMovementIds;

	/** Clear all canonical movement tables before a rebuild or teardown. */
	void ResetCanonicalMovementTable();

	/** One-time analysis: build the junction exit table from road provider data. */
	void BuildJunctionSurvey();
};
