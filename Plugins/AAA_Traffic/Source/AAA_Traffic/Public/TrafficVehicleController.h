// Copyright AAA_Traffic Contributors. All Rights Reserved.

#pragma once

#include "CoreMinimal.h"
#include "AIController.h"
#include "TrafficRoadProvider.h"
#include "TrafficVehicleController.generated.h"

/**
 * State of in-progress lane change.
 */
UENUM(BlueprintType)
enum class ELaneChangeState : uint8
{
	/** Not changing lanes. */
	None,
	/** Actively blending from source to target lane. */
	Executing,
	/** Blend complete, settling onto new lane. */
	Completing
};

/**
 * AI controller that drives a Chaos vehicle along a lane
 * obtained from an ITrafficRoadProvider adapter.
 *
 * Uses a pure-pursuit steering model: picks a look-ahead point on the
 * lane centerline and feeds throttle / steering / brake to the vehicle
 * movement component.
 */
UCLASS(Blueprintable)
class AAA_TRAFFIC_API ATrafficVehicleController : public AAIController
{
	GENERATED_BODY()

public:
	ATrafficVehicleController();

	/**
	 * Tell this controller which lane to follow.
	 * Fetches the lane path from the active road provider and begins driving.
	 */
	UFUNCTION(BlueprintCallable, Category = "Traffic")
	void InitializeLaneFollowing(const FTrafficLaneHandle& InLane);

	/** Set the target speed for this controller (cm/s). */
	UFUNCTION(BlueprintCallable, Category = "Traffic")
	void SetTargetSpeed(float InSpeed);

	/** Set the random seed before possession (determines RandomStream). */
	UFUNCTION(BlueprintCallable, Category = "Traffic")
	void SetRandomSeed(int32 InSeed);

	/** Returns true if the vehicle has reached a dead-end with no connected lanes. */
	UFUNCTION(BlueprintPure, Category = "Traffic")
	bool IsAtDeadEnd() const { return bAtDeadEnd; }

	/** Get the lane this vehicle is currently following. */
	UFUNCTION(BlueprintPure, Category = "Traffic")
	const FTrafficLaneHandle& GetCurrentLane() const { return CurrentLane; }

	/**
	 * Configure lane-change behavior from spawner aggression slider (0-1).
	 * Maps aggression to internal thresholds and cooldown timings.
	 */
	UFUNCTION(BlueprintCallable, Category = "Traffic")
	void SetLaneChangeAggression(float Aggression);

	/** Set the base (default) speed limit used when the provider has no lane speed data. */
	UFUNCTION(BlueprintCallable, Category = "Traffic")
	void SetDefaultSpeedLimit(float InSpeedLimit);

protected:
	virtual void OnPossess(APawn* InPawn) override;
	virtual void OnUnPossess() override;
	virtual void Tick(float DeltaSeconds) override;

private:
	/** Feed throttle / steering / brake into the vehicle movement component. */
	void UpdateVehicleInput(float DeltaSeconds);

	/**
	 * Find the index of the closest lane point to the given world position.
	 * Uses a cached last-known index with bounded search for O(1) amortized performance.
	 */
	int32 FindClosestPointIndex(const FVector& VehicleLocation);

	/** Walk forward along lane points by LookAheadDistance and return the target point. */
	FVector GetLookAheadPoint(const FVector& VehicleLocation, int32 ClosestIndex) const;

	/** Compute remaining path distance from a point index to the lane end. */
	float GetRemainingDistance(int32 FromIndex) const;

	/**
	 * Called when the vehicle is nearing lane-end.
	 * Queries connected lanes and transitions, or flags dead-end for braking.
	 */
	void CheckLaneTransition();

	/**
	 * Detect the nearest traffic vehicle ahead via a sphere sweep along the lane direction.
	 * Returns the distance to the leader (cm), or -1.0 if no leader detected.
	 * OutLeaderSpeed receives the leader's forward speed if found.
	 *
	 * NOTE — Physics boundary (System.md §4.5): sweep results depend on Chaos physics
	 * state, which is not guaranteed deterministic across machines. This is acceptable
	 * because proximity detection is a reactive safety system, not a decision-logic
	 * path — minor variations affect smoothness, not correctness.
	 *
	 * Performance: O(N) sweeps per frame where N is number of active vehicles.
	 * Acceptable for traffic counts up to low hundreds; if hotspot profiling shows
	 * this dominating frame time, consider switching to analytical lane-position
	 * checks via the vehicle registry.
	 */
	float GetLeaderDistance(float& OutLeaderSpeed) const;

	// ── Lane change ─────────────────────────────────────────

	/**
	 * Evaluate whether to begin a lane change (slow leader ahead, gap available).
	 * Called every tick when LaneChangeState == None and cooldown has expired.
	 */
	void EvaluateLaneChange();

	/**
	 * Advance the active lane-change blend.
	 * Interpolates the steering target between source and target lane polylines.
	 * Returns the blended look-ahead point for this frame.
	 */
	FVector UpdateLaneChangeBlend(const FVector& VehicleLocation, int32 ClosestIndex);

	/**
	 * Complete a lane change: adopt target lane as current, reset state.
	 */
	void FinalizeLaneChange();

	/**
	 * Abort a lane change: return to source lane, reset state.
	 * Called when continuous gap check detects it's unsafe to continue.
	 */
	void AbortLaneChange();

	// ----- State -----

	/** Handle to the lane currently being followed. */
	FTrafficLaneHandle CurrentLane;

	/** Cached world-space centerline points of the current lane. */
	TArray<FVector> LanePoints;

	/** Cached lane width in cm. */
	float LaneWidth;

	/** Set to true once lane data has been fetched from the provider. */
	bool bLaneDataReady;

	/** Cached road provider pointer for lane transition queries. */
	ITrafficRoadProvider* CachedProvider;

	/** True when vehicle has reached lane-end with no connected lanes. */
	bool bAtDeadEnd;

	/**
	 * Junction transition path: smooth curve points connecting the end of one
	 * lane to the start of the next. Consumed before following the new lane.
	 */
	TArray<FVector> JunctionTransitionPoints;

	/** Index along JunctionTransitionPoints currently being followed. */
	int32 JunctionTransitionIndex;

	/** True when waiting at an intersection for right-of-way. */
	bool bWaitingAtIntersection;

	/** Junction ID of the intersection this vehicle is waiting/in. 0 = none. */
	int32 IntersectionJunctionId;

	/** Cumulative distance traveled on the current lane (prevents short-lane transition loops). */
	float DistanceTraveledOnLane;

	/** Previous vehicle location for distance tracking. */
	FVector PreviousVehicleLocation;

	/** Distance traveled by the vehicle this tick (computed once, consumed by blend). */
	float DistanceThisTick;

	/** Cached index from last FindClosestPointIndex call (for O(1) amortized search). */
	int32 LastClosestIndex;

	/** Frame counter for LOD-based tick gating. */
	uint32 LODFrameCounter;

	// ── Lane-change state ───────────────────────────────────

	/** Current lane-change phase. */
	ELaneChangeState LaneChangeState;

	/** Handle to the target lane during an active lane change. */
	FTrafficLaneHandle TargetLaneHandle;

	/** Cached centerline of the target lane (for blending). */
	TArray<FVector> TargetLanePoints;

	/** Width of the target lane (cm). */
	float TargetLaneWidth;

	/** Progress of the lane-change blend (0 = source, 1 = target). */
	float LaneChangeProgress;

	/** Time remaining in the Completing settling phase (seconds). */
	float LaneChangeSettleTimer;

	/** Time remaining before another lane change can be considered (seconds). */
	float LaneChangeCooldownRemaining;

	/** Base target speed before lane speed-limit adjustments. */
	float BaseTargetSpeed;

protected:
	// ----- Tuning -----

	/** Target speed in cm/s (default ~54 km/h). */
	UPROPERTY(EditAnywhere, BlueprintReadWrite, Category = "Traffic", meta = (ClampMin = "0"))
	float TargetSpeed;

	/** Distance ahead on the lane path (cm) used as the steering target. */
	UPROPERTY(EditAnywhere, BlueprintReadWrite, Category = "Traffic", meta = (ClampMin = "100"))
	float LookAheadDistance;

	/** Minimum safe following distance behind a leader (cm). */
	UPROPERTY(EditAnywhere, BlueprintReadWrite, Category = "Traffic|Proximity", meta = (ClampMin = "100"))
	float FollowingDistance;

	/** Maximum forward detection range for vehicles ahead (cm). */
	UPROPERTY(EditAnywhere, BlueprintReadWrite, Category = "Traffic|Proximity", meta = (ClampMin = "500"))
	float DetectionDistance;

	// ── Lane Change Tuning ──────────────────────────────────

	/**
	 * Distance over which the lane-change lateral blend occurs (cm).
	 * Shorter = snappier, longer = smoother.
	 */
	UPROPERTY(EditAnywhere, BlueprintReadWrite, Category = "Traffic|LaneChange", meta = (ClampMin = "200"))
	float LaneChangeDistance;

	/** Minimum time between successive lane changes (seconds). */
	UPROPERTY(EditAnywhere, BlueprintReadWrite, Category = "Traffic|LaneChange", meta = (ClampMin = "0"))
	float LaneChangeCooldownTime;

	/**
	 * Speed ratio threshold to trigger a lane change (0-1).
	 * A change is considered when CurrentSpeed / TargetSpeed < this value.
	 */
	UPROPERTY(EditAnywhere, BlueprintReadWrite, Category = "Traffic|LaneChange", meta = (ClampMin = "0", ClampMax = "1"))
	float LaneChangeSpeedThreshold;

	/** Minimum gap (cm) required on the target lane for a safe merge. */
	UPROPERTY(EditAnywhere, BlueprintReadWrite, Category = "Traffic|LaneChange", meta = (ClampMin = "100"))
	float LaneChangeGapRequired;

	/** Default speed limit (cm/s) used when the provider returns no speed data. */
	UPROPERTY(EditAnywhere, BlueprintReadWrite, Category = "Traffic", meta = (ClampMin = "0"))
	float DefaultSpeedLimit;

	/**
	 * Seed for deterministic random decisions.
	 * Reserved for future use (e.g. lane choice at intersections).
	 */
	UPROPERTY(EditAnywhere, BlueprintReadWrite, Category = "Traffic")
	int32 RandomSeed;

	/** When true (non-Shipping builds), draws the lane polyline, look-ahead target, and leader detection in-game. Has no effect in Shipping. */
	UPROPERTY(EditAnywhere, Category = "Traffic|Debug")
	bool bDebugDraw = false;

	// --- One-shot diagnostic flags (prevent log spam) ---
	bool bDiagLoggedNoMovement = false;
	bool bDiagLoggedTickSkip = false;
	bool bDiagLoggedFirstInput = false;

	/**
	 * Deterministic random stream (seeded from RandomSeed).
	 * Reserved for future lane-choice / gap-acceptance randomization.
	 */
	FRandomStream RandomStream;
};
