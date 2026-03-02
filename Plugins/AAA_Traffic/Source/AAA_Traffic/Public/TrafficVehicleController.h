// Copyright AAA_Traffic Contributors. All Rights Reserved.

#pragma once

#include "CoreMinimal.h"
#include "AIController.h"
#include "DrawDebugHelpers.h"
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

	/** Append one lane-decision trace record into the bounded diagnostics ring buffer. */
	void AddLaneDecisionTrace(const TCHAR* EventName, int32 CandidateLane, float MetricA, float MetricB, const FString& Detail = FString());

	/** Flush the current decision trace buffer to log with a reason label, then clear it. */
	void FlushLaneDecisionTrace(const TCHAR* Reason, bool bAsWarning);

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
	bool bWaitingAtIntersection = false;

	/** Cooldown timer for intersection retry attempts (seconds). */
	float IntersectionRetryTimer = 0.0f;

	/** Junction ID of the intersection this vehicle is waiting/in. 0 = none. */
	int32 IntersectionJunctionId = 0;

	/** Junction ID that was just released on the current lane.
	 *  Prevents re-detection of the same junction after the vehicle releases
	 *  occupancy mid-lane (e.g. curve-complete fires before lane-end).
	 *  Reset in InitializeLaneFollowing when moving to a new lane. */
	int32 LastReleasedJunctionId = 0;

	/** The junction lane handle used for signal queries.
	 *  When a junction is detected via look-ahead, this is the NEXT lane
	 *  (the one inside the intersection) — the same handle that appears
	 *  in the signal's PhaseGroup.GreenLanes. Using this instead of
	 *  CurrentLane ensures IsLaneGreen matches correctly. */
	FTrafficLaneHandle IntersectionJunctionLane = FTrafficLaneHandle();

	/** The lane the vehicle approaches the junction from (for conflict detection). */
	FTrafficLaneHandle IntersectionFromLane = FTrafficLaneHandle();

	/** The lane the vehicle exits the junction to (for conflict detection). */
	FTrafficLaneHandle IntersectionToLane = FTrafficLaneHandle();

	/** Exact 3D world position of the intersection entry boundary.
	 *  Derived from intersection mask geometry (not quantized polyline).
	 *  Used as the brake target when waiting at an intersection. */
	FVector IntersectionEntryWorldPos = FVector::ZeroVector;

	/** True if IntersectionEntryWorldPos is valid (entry point was resolved). */
	bool bHasIntersectionEntryPos = false;

	// ── Junction Approach Scan State ─────────────────────────

	/** True when the multi-hop scan has detected a junction within braking range. */
	bool bApproachingIntersection;

	/** Distance (cm) to the detected upcoming junction (from current position along lane graph). */
	float ApproachJunctionDistanceCm = 0.0f;

	/** Computed approach speed limit (cm/s) based on stopping-distance envelope. */
	float ApproachSpeedLimitCmPerSec = 0.0f;

	/** Junction ID detected by the approach scan (0 = none). */
	int32 ApproachJunctionId = 0;

	/** The junction lane handle from the approach scan (for debug display). */
	FTrafficLaneHandle ApproachJunctionLane = FTrafficLaneHandle();

	/** Cumulative distance traveled on the current lane (prevents short-lane transition loops). */
	float DistanceTraveledOnLane = 0.0f;

	/** Elapsed time (seconds) spent waiting at the current intersection. */
	float IntersectionWaitElapsed = 0.0f;

	/** Per-instance log throttle counter for waiting-state diagnostics.
	 *  Replaces the old static int32 that was shared across all instances
	 *  (determinism violation). */
	int32 WaitLogThrottleCounter = 0;

	/** Accumulated DeltaTime from LOD-skipped frames.
	 *  Passed to UpdateVehicleInput on the next active tick so behavior
	 *  is frame-rate independent regardless of LOD gating. */
	float LODAccumulatedDeltaTime = 0.0f;

	/** Timer for periodic junction diagnostic dumps (traffic.JunctionDiagnostics CVar). */
	float DiagJunctionTimer;

	/** Previous vehicle location for distance tracking. */
	FVector PreviousVehicleLocation;

	/** Distance traveled by the vehicle this tick (computed once, consumed by blend). */
	float DistanceThisTick;

	/** Previous-tick cross-track error (CrossZ) for the steering derivative term. */
	float PreviousCrossTrackError = 0.0f;

	// ── Computed adaptive distances (set each tick in UpdateVehicleInput) ─
	// These are derived from the time-based tuning properties × current speed.
	// Internal code (GetLookAheadPoint, GetLeaderDistance, etc.) reads these
	// instead of the old fixed-distance UPROPERTYs.

	/** Computed look-ahead distance (cm) for this tick. */
	float LookAheadDistance = 300.0f;

	/** Computed following distance (cm) for this tick. */
	float FollowingDistance = 200.0f;

	/** Computed detection distance (cm) for this tick. */
	float DetectionDistance = 1500.0f;

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
	float TargetLaneWidth = 0.0f;

	/** Progress of the lane-change blend (0 = source, 1 = target). */
	float LaneChangeProgress = 0.0f;

	/** Time remaining in the Completing settling phase (seconds). */
	float LaneChangeSettleTimer = 0.0f;

	/** Time remaining before another lane change can be considered (seconds). */
	float LaneChangeCooldownRemaining = 0.0f;

	/** Base target speed before lane speed-limit adjustments. */
	float BaseTargetSpeed = 0.0f;

	/** Single lane-decision trace record for forensic debugging. */
	struct FLaneDecisionTrace
	{
		double WorldTimeSeconds = 0.0;
		FString EventName;
		int32 CurrentLaneId = 0;
		int32 CandidateLaneId = 0;
		float MetricA = 0.0f;
		float MetricB = 0.0f;
		FString Detail;
	};

	/** Bounded decision trace buffer (ring behavior by removing oldest). */
	TArray<FLaneDecisionTrace> LaneDecisionTraceBuffer;

	/** Effective max entries for LaneDecisionTraceBuffer (read from CVar). */
	int32 LaneDecisionTraceMaxEntries = 256;

protected:
	// ----- Tuning -----

	/** Target speed in cm/s (default ~54 km/h). */
	UPROPERTY(EditAnywhere, BlueprintReadWrite, Category = "Traffic", meta = (ClampMin = "0"))
	float TargetSpeed;

	// ── Adaptive Driving Parameters (time-based) ───────────
	// These replace the old fixed-distance properties. At runtime the
	// controller computes: EffectiveDistance = Speed * TimeSec, clamped
	// to a minimum. This means distances grow with speed automatically.

	/**
	 * Time (seconds) of look-ahead along the lane centerline for the
	 * steering target. Effective distance = CurrentSpeed * this value,
	 * clamped to MinLookAheadDistanceCm. Default 0.6s.
	 */
	UPROPERTY(EditAnywhere, BlueprintReadWrite, Category = "Traffic|Steering", meta = (ClampMin = "0.1"))
	float LookAheadTimeSec;

	/** Floor for look-ahead distance (cm) at very low speeds. Default 300. */
	UPROPERTY(EditAnywhere, BlueprintReadWrite, Category = "Traffic|Steering", meta = (ClampMin = "50"))
	float MinLookAheadDistanceCm;

	/**
	 * Derivative (damping) gain for the PD steering controller.
	 * Larger values reduce steering oscillation on straights.
	 * 0 = pure P controller (old behavior). Default 0.5.
	 */
	UPROPERTY(EditAnywhere, BlueprintReadWrite, Category = "Traffic|Steering", meta = (ClampMin = "0", ClampMax = "2"))
	float SteeringDampingFactor;

	/**
	 * Time (seconds) of safe following gap behind a leader vehicle.
	 * Effective distance = CurrentSpeed * this value, clamped to
	 * MinFollowingDistanceCm. Default 1.5s (standard 1.5-second rule).
	 */
	UPROPERTY(EditAnywhere, BlueprintReadWrite, Category = "Traffic|Proximity", meta = (ClampMin = "0.3"))
	float FollowingTimeSec;

	/** Floor for following distance (cm) at very low speeds. Default 200. */
	UPROPERTY(EditAnywhere, BlueprintReadWrite, Category = "Traffic|Proximity", meta = (ClampMin = "50"))
	float MinFollowingDistanceCm;

	/**
	 * Time (seconds) of forward detection range for leader vehicles.
	 * Effective distance = CurrentSpeed * this value, clamped to
	 * MinDetectionDistanceCm. Default 4.0s.
	 */
	UPROPERTY(EditAnywhere, BlueprintReadWrite, Category = "Traffic|Proximity", meta = (ClampMin = "1.0"))
	float DetectionTimeSec;

	/** Floor for detection distance (cm) at very low speeds. Default 1500. */
	UPROPERTY(EditAnywhere, BlueprintReadWrite, Category = "Traffic|Proximity", meta = (ClampMin = "500"))
	float MinDetectionDistanceCm;

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
	 * Global margin (cm) added between the vehicle's front bumper and the
	 * intersection entry boundary. Purely cosmetic comfort distance.
	 * Defaults to 0 (exact geometric stop). Not per-intersection.
	 * @todo Wire into brake-targeting offset once stop-line snapping is implemented.
	 */
	UPROPERTY(EditAnywhere, BlueprintReadWrite, Category = "Traffic|Intersection", meta = (ClampMin = "0"))
	float StopLineMarginCm;

	// ── Junction Approach Scan Tuning ───────────────────────

	/**
	 * Maximum forward distance (cm) to scan for upcoming junctions.
	 * Higher values give more warning at higher speeds.
	 * Default 50000 = 500m (sufficient for highway stopping from 120 km/h).
	 */
	UPROPERTY(EditAnywhere, BlueprintReadWrite, Category = "Traffic|Intersection", meta = (ClampMin = "5000"))
	float JunctionScanMaxDistanceCm;

	/**
	 * Maximum lane-graph hops to follow when scanning for junctions.
	 * Prevents runaway traversal on complex networks.
	 */
	UPROPERTY(EditAnywhere, BlueprintReadWrite, Category = "Traffic|Intersection", meta = (ClampMin = "1", ClampMax = "20"))
	int32 MaxJunctionScanHops;

	/**
	 * Safety margin (cm) added beyond the physics stopping distance.
	 * Vehicles begin braking StopDist + this margin before the junction.
	 */
	UPROPERTY(EditAnywhere, BlueprintReadWrite, Category = "Traffic|Intersection", meta = (ClampMin = "0"))
	float ApproachSafetyMarginCm;

	/**
	 * Comfort deceleration (cm/s²) used for the approach speed envelope.
	 * Must match or be close to StopDecel used in the actual braking logic.
	 */
	UPROPERTY(EditAnywhere, BlueprintReadWrite, Category = "Traffic|Intersection", meta = (ClampMin = "100"))
	float ApproachDecelCmPerSec2;

	/**
	 * Base speed limit (cm/s) while traversing a junction.
	 * Actual cap may be lower if the junction path has tight curvature.
	 * Default 2000 = ~45 mph (reasonable straight-through speed).
	 * Set lower for stop-sign networks, higher for highway merges.
	 */
	UPROPERTY(EditAnywhere, BlueprintReadWrite, Category = "Traffic|Intersection", meta = (ClampMin = "100"))
	float IntersectionSpeedLimitCmPerSec;

	/**
	 * Maximum time (seconds) a vehicle will wait at an intersection before
	 * force-proceeding to break potential deadlocks. 0 = wait forever (not
	 * recommended). Default 30s.
	 */
	UPROPERTY(EditAnywhere, BlueprintReadWrite, Category = "Traffic|Intersection", meta = (ClampMin = "0"))
	float MaxIntersectionWaitTimeSec;

	/** Cached forward extent of the vehicle mesh bounding box (cm from actor
	 *  origin to front bumper). Computed once on possess from the pawn's
	 *  component bounds. Currently used for diagnostics/logging and
	 *  reserved for future stop-line alignment logic. */
	float VehicleFrontExtent = 0.0f;

	// ── Physics Safety Net State ────────────────────────────

	/** Consecutive frames the vehicle has been detected as flipped / airborne.
	 *  After ConsecutiveFlipFramesThreshold frames, the vehicle is safety-despawned. */
	int32 ConsecutiveFlipFrames = 0;

	/** Consecutive frames the vehicle has been stuck (throttle applied but near-zero movement).
	 *  After ConsecutiveStuckFramesThreshold frames, the vehicle is safety-despawned. */
	int32 ConsecutiveStuckFrames = 0;

	/** Throttle timer for the wake guard so it fires at most once per second
	 *  instead of every single tick (avoids energy injection). */
	float WakeGuardCheckTimer = 0.0f;

	/** True once a safety despawn has been requested. Prevents driving input
	 *  and further despawn requests while the deferred destroy is pending. */
	bool bPendingRecoveryDespawn = false;

	/**
	 * Maximum allowed vehicle speed (cm/s). If the physics body's linear
	 * velocity exceeds this, the velocity is zeroed as an emergency brake.
	 * Default 8000 cm/s ≈ 180 mph — well above any realistic traffic speed.
	 */
	UPROPERTY(EditAnywhere, BlueprintReadWrite, Category = "Traffic|Safety", meta = (ClampMin = "1000"))
	float MaxAllowedSpeedCmPerSec;

	/** Frames a vehicle must be flipped / airborne before being despawned.
	 *  At 60 fps, 90 frames ≈ 1.5 seconds — enough to filter brief bumps. */
	static constexpr int32 ConsecutiveFlipFramesThreshold = 90;

	/** Frames a vehicle must be stuck before being despawned.
	 *  At 60 fps, 300 frames ≈ 5 seconds — generous grace period for temporary stops. */
	static constexpr int32 ConsecutiveStuckFramesThreshold = 300;

	/**
	 * Seed for deterministic random decisions.
	 * Reserved for future use (e.g. lane choice at intersections).
	 */
	UPROPERTY(EditAnywhere, BlueprintReadWrite, Category = "Traffic")
	int32 RandomSeed;

	/** When true (non-Shipping builds), draws the lane polyline, look-ahead target, and leader detection in-game. Has no effect in Shipping. Can also be toggled globally via console: traffic.DebugDraw 1 */
	UPROPERTY(EditAnywhere, Category = "Traffic|Debug")
	bool bDebugDraw = false;

	// --- Cached debug state (populated each tick in UpdateVehicleInput for debug draw) ---
#if defined(ENABLE_DRAW_DEBUG) && ENABLE_DRAW_DEBUG
	FString DbgStateName = TEXT("INIT");
	float DbgThrottle = 0.0f;
	float DbgBrake = 0.0f;
	float DbgSteering = 0.0f;
	float DbgEffectiveTargetSpeed = 0.0f;
	float DbgCurrentSpeed = 0.0f;
	float DbgDistToEntry = -1.0f; // -1 = not approaching intersection
	float DbgDesiredStopSpeed = 0.0f;
	float DbgLeaderDist = -1.0f;  // -1 = no leader detected
	float DbgLeaderSpeed = 0.0f;
	float DbgRemainingDist = 0.0f;
	float DbgTransitionThreshold = 0.0f;
	float DbgStoppingDist = 0.0f; // v^2 / (2*300)
	FVector DbgTargetPoint = FVector::ZeroVector;
	float DbgApproachJunctionDist = -1.0f;  // -1 = no junction detected by scan
	float DbgApproachSpeedLimit = 0.0f;
	int32 DbgApproachJunctionId = 0;
#endif // ENABLE_DRAW_DEBUG

	// --- One-shot diagnostic flags (prevent log spam, gated by traffic.VehicleDiagnostics CVar) ---
	bool bDiagLoggedNoMovement = false;
	bool bDiagLoggedTickSkip = false;
	bool bDiagLoggedFirstInput = false;
	bool bDiagLoggedMovementCheck = false;
	float DiagElapsedTime = 0.0f;
	FVector DiagSpawnLocation = FVector::ZeroVector;

	/** Periodic diagnostic timer — fires every DiagPeriodicInterval seconds. */
	float DiagPeriodicTimer = 0.0f;
	static constexpr float DiagPeriodicInterval = 2.0f;

	/** True once the vehicle has been observed moving (Speed > 10 cm/s). Enables STOPPED one-shot. */
	bool bDiagWasMoving = false;
	/** True once the STOPPED one-shot has fired. Prevents spam. */
	bool bDiagLoggedStopped = false;

	/**
	 * Deterministic random stream (seeded from RandomSeed).
	 * Reserved for future lane-choice / gap-acceptance randomization.
	 */
	FRandomStream RandomStream;
};
