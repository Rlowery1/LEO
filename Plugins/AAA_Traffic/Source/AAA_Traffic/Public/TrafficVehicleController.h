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
 * Turn signal indicator state — exposed to Blueprint for visual binding
 * (e.g. activating brake-light / turn-signal meshes or materials).
 */
UENUM(BlueprintType)
enum class ETurnSignalState : uint8
{
	/** No signal active. */
	Off,
	/** Left turn signal. */
	Left,
	/** Right turn signal. */
	Right,
	/** Hazard lights (both sides). */
	Hazard
};

/** Delegate broadcast when the vehicle's turn signal state changes. */
DECLARE_DYNAMIC_MULTICAST_DELEGATE_OneParam(FOnTurnSignalChanged, ETurnSignalState, NewState);

/** Delegate broadcast when the vehicle's brake light state changes. */
DECLARE_DYNAMIC_MULTICAST_DELEGATE_OneParam(FOnBrakeLightChanged, bool, bBraking);

/**
 * Consolidated junction interaction phase.
 * Single source of truth — replaces multiple independent booleans/IDs.
 */
UENUM()
enum class EJunctionPhase : uint8
{
	/** No junction interaction. */
	Idle,
	/** Approach scan detected junction within braking range. */
	Approaching,
	/** At junction entry: signal red, occupancy denied, or stop-sign wait. */
	Waiting,
	/** Occupied junction, following transition curve. */
	Traversing
};

/**
 * Consolidated junction state for a traffic vehicle.
 * Replaces 18+ independent member variables with a single struct
 * that enforces phase-transition invariants.
 */
USTRUCT()
struct FVehicleJunctionState
{
	GENERATED_BODY()

	/** Current phase of junction interaction. */
	EJunctionPhase Phase = EJunctionPhase::Idle;

	/** Junction ID being interacted with. Valid when Phase != Idle. */
	int32 JunctionId = 0;

	/** Junction lane handle (for signal queries). Valid when Phase >= Waiting. */
	FTrafficLaneHandle JunctionLane;

	/** Approach lane (from). Valid when Phase >= Waiting. */
	FTrafficLaneHandle FromLane;

	/** Exit lane (to). Valid when Phase >= Waiting. */
	FTrafficLaneHandle ToLane;

	/** World position of junction entry boundary. */
	FVector EntryWorldPos = FVector::ZeroVector;

	/** True if EntryWorldPos has been resolved. */
	bool bHasEntryPos = false;

	/** True when waiting for right-of-way (Phase == Waiting). */
	bool bWaiting = false;

	/** Cooldown timer for retry attempts (seconds, Phase == Waiting). */
	float RetryTimer = 0.0f;

	/** Elapsed wait time at junction (seconds, Phase == Waiting). */
	float WaitElapsed = 0.0f;

	/** Last released junction ID — prevents re-detection on same lane. */
	int32 LastReleasedId = 0;

	/** Distance to detected junction (cm, Phase == Approaching). */
	float ApproachDistanceCm = 0.0f;

	/** Computed approach speed limit (cm/s, Phase == Approaching). */
	float ApproachSpeedLimitCmPerSec = 0.0f;

	/** Junction lane from approach scan (Phase == Approaching). */
	FTrafficLaneHandle ApproachJunctionLane;

	/** Smooth Hermite curve path through junction (Phase == Traversing). */
	TArray<FVector> TransitionPoints;

	/** Current index along TransitionPoints. */
	int32 TransitionIndex = 0;

	/** Index where actual junction curve begins (after prepended tail points). */
	int32 CurveStartIndex = 0;

	/** Stop-sign mandatory wait elapsed (seconds, Phase == Waiting). */
	float StopSignStopElapsed = 0.0f;

	/** True when stop-sign wait is complete, waiting for clearance. */
	bool bStopSignWaitComplete = false;

	// ── Phase transitions (implemented in TrafficVehicleController.cpp) ──

	/** Idle → Approaching. Sets JunctionId and prepares approach fields. */
	void BeginApproach(int32 InJunctionId);

	/** Idle|Approaching → Waiting. Caller fills JunctionLane/FromLane/ToLane/EntryWorldPos. */
	void BeginWaiting();

	/** Waiting → Traversing. TransitionPoints must already be filled. */
	void BeginTraversing();

	/** Waiting|Traversing → Released. Clears engagement data, keeps LastReleasedId. */
	void Release();

	/** Any → Idle. Unconditional full reset (zeroes everything including LastReleasedId). */
	void Reset();

	/** Returns true if interacting with a junction (Approaching, Waiting, or Traversing). */
	bool IsActive() const { return Phase >= EJunctionPhase::Approaching && Phase <= EJunctionPhase::Traversing; }

	/** Returns true if the junction is engaged (Waiting or Traversing — occupancy relevant). */
	bool IsEngaged() const { return Phase >= EJunctionPhase::Waiting && Phase <= EJunctionPhase::Traversing; }
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

	friend class UTrafficSubsystem;

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

	/** Returns true if lane data (polyline, width) has been fetched from the provider. */
	bool IsLaneDataReady() const { return bLaneDataReady; }

	/** Get the cached lane polyline (world-space centerline points). */
	const TArray<FVector>& GetLanePoints() const { return LanePoints; }

	/** Get the cached lane width (cm). */
	float GetLaneWidth() const { return LaneWidth; }

	/**
	 * Configure lane-change behavior from spawner aggression slider (0-1).
	 * Maps aggression to internal thresholds and cooldown timings.
	 */
	UFUNCTION(BlueprintCallable, Category = "Traffic")
	void SetLaneChangeAggression(float Aggression);

	/** Set the base (default) speed limit used when the provider has no lane speed data. */
	UFUNCTION(BlueprintCallable, Category = "Traffic")
	void SetDefaultSpeedLimit(float InSpeedLimit);

	/** Get the current turn signal state (Off, Left, Right, Hazard). */
	UFUNCTION(BlueprintPure, Category = "Traffic")
	ETurnSignalState GetTurnSignalState() const { return CurrentTurnSignal; }

	/** Returns true when the vehicle's brake lights are illuminated. */
	UFUNCTION(BlueprintPure, Category = "Traffic")
	bool AreBrakeLightsOn() const { return bBrakeLightsOn; }

	/** Fired whenever the turn signal state changes. Bind in Blueprint to
	 *  activate turn signal lights on the vehicle mesh. */
	UPROPERTY(BlueprintAssignable, Category = "Traffic")
	FOnTurnSignalChanged OnTurnSignalChanged;

	/** Fired whenever the brake light state changes. Bind in Blueprint to
	 *  toggle brake light materials/emissives. */
	UPROPERTY(BlueprintAssignable, Category = "Traffic")
	FOnBrakeLightChanged OnBrakeLightChanged;

protected:
	virtual void OnPossess(APawn* InPawn) override;
	virtual void OnUnPossess() override;
	virtual void Tick(float DeltaSeconds) override;

	/** Reactive collision handler — called by OnActorHit delegate. */
	UFUNCTION()
	void HandleActorHit(AActor* SelfActor, AActor* OtherActor,
		FVector NormalImpulse, const FHitResult& Hit);

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

	/**
	 * Pick a junction exit using the centralized surveyor table.
	 * Looks up the pre-computed legal exits for the approach lane, then
	 * uses the seeded random stream for weighted selection.
	 * Falls back to raw connected-lane data if the surveyor has no entry.
	 *
	 * @param ApproachLane  The approach lane entering the junction.
	 * @param FallbackExits Raw exit list to use if no survey data exists.
	 * @return Chosen exit lane, or invalid handle if no exits available.
	 */
	FTrafficLaneHandle PickSurveyedExit(
		const FTrafficLaneHandle& ApproachLane,
		const TArray<FTrafficLaneHandle>& FallbackExits);

	/** Append one lane-decision trace record into the bounded diagnostics ring buffer. */
	void AddLaneDecisionTrace(const TCHAR* EventName, int32 CandidateLane, float MetricA, float MetricB, const FString& Detail = FString());

	/** Flush the current decision trace buffer to log with a reason label, then clear it. */
	void FlushLaneDecisionTrace(const TCHAR* Reason, bool bAsWarning);

	/**
	 * Compute signed Menger curvature from nearby polyline points.
	 * Positive = turning left, negative = turning right (Z-up right-hand rule).
	 * Uses widened point spacing (half CurveScanWindowSize) to suppress
	 * noise from the 100cm polyline sampling interval.
	 *
	 * @param Points      The lane polyline (LanePoints or JnctState.TransitionPoints).
	 * @param CenterIndex Index of the vehicle's closest point.
	 * @return Signed curvature in 1/cm.
	 */
	float ComputeLocalCurvature(const TArray<FVector>& Points, int32 CenterIndex) const;

	// ----- State -----

	/** Handle to the lane currently being followed. */
	FTrafficLaneHandle CurrentLane;

	/** Cached world-space centerline points of the current lane. */
	TArray<FVector> LanePoints;

	/** Cached lane width in cm. */
	float LaneWidth;

	/** Set to true once lane data has been fetched from the provider. */
	bool bLaneDataReady;

	/** True on the first tick after entering a new lane. Used to trigger
	 *  a full polyline curvature scan for braking envelope setup. */
	bool bFirstTickOnLane = true;

	/** Cached road provider pointer for lane transition queries. */
	ITrafficRoadProvider* CachedProvider;

	/** True when vehicle has reached lane-end with no connected lanes. */
	bool bAtDeadEnd;

	/** Consolidated junction interaction state — single source of truth. */
	FVehicleJunctionState JnctState;

	/** Monotonically increasing index assigned by UTrafficSubsystem::RegisterVehicle.
	 *  Used to guarantee deterministic iteration order across frames. */
	int32 DeterministicSpawnIndex = -1;

	/** Cumulative distance traveled on the current lane (prevents short-lane transition loops). */
	float DistanceTraveledOnLane = 0.0f;

	// ── Turn signal state ───────────────────────────────────

	/** Current turn signal state. */
	ETurnSignalState CurrentTurnSignal = ETurnSignalState::Off;

	/** Countdown timer (seconds) for delayed turn-signal deactivation.
	 *  When > 0, the signal stays on; ticked down each frame. */
	float TurnSignalOffDelayRemaining = 0.0f;

	/** Set the turn signal and broadcast delegate (no-op if state unchanged). */
	void SetTurnSignal(ETurnSignalState NewState);

	/**
	 * Compute the turn direction for a junction transition.
	 * Uses cross product of approach direction × exit direction.
	 * Returns Left, Right, or Off (straight-through).
	 */
	ETurnSignalState ComputeTurnDirection(const FTrafficLaneHandle& FromLane, const FTrafficLaneHandle& ToLane) const;

	// ── Brake light state ───────────────────────────────────

	/** True when the vehicle's brake lights should be illuminated. */
	bool bBrakeLightsOn = false;

	/** Updates brake light state and broadcasts delegate if changed. */
	void SetBrakeLights(bool bNewState);

	// ── Lane pre-positioning state ──────────────────────────

	/** Target lane handle for navigational lane change (pre-positioning before junction). */
	FTrafficLaneHandle NavigationalTargetLane;

	/** True when a navigational (junction pre-positioning) lane change is pending or active. */
	bool bNavigationalLaneChange = false;

	// ── Dead-end despawn state ──────────────────────────────

	/** Countdown (seconds) before a vehicle stopped at a dead-end is
	 *  recycled via RequestDespawn.  Gives a brief realistic pause
	 *  (brake lights on, stopped) before the vehicle disappears. */
	float DeadEndDespawnTimer = 0.0f;

	/** Duration (seconds) to wait at a dead-end before despawning.
	 *  Configurable per-class; keep short for map-edge roads. */
	static constexpr float DeadEndDespawnDelaySec = 3.0f;

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

	/** Previous-tick heading cross product (CrossZ) for the steering derivative term. */
	float PreviousHeadingCrossZ = 0.0f;

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
	 * Proportional gain for cross-track error (CTE) correction.
	 * Adds a lateral offset correction to the pure pursuit steering,
	 * pushing the vehicle back toward the lane centerline.
	 * Must be LOW relative to pure pursuit — too high overpowers
	 * heading correction on curves, causing the car to steer opposite
	 * to the curve direction and leave the road.
	 * 0 = no CTE correction. Default 0.3.
	 */
	UPROPERTY(EditAnywhere, BlueprintReadWrite, Category = "Traffic|Steering", meta = (ClampMin = "0", ClampMax = "2"))
	float CTECorrectionGain;

	/**
	 * Lateral acceleration budget (cm/s²) for curve speed limits.
	 * Determines how fast vehicles take curves: v = sqrt(budget * radius).
	 * Default 294 = 0.3g (comfortable passenger car). Increase for
	 * sportier vehicles, decrease for heavy trucks.
	 */
	UPROPERTY(EditAnywhere, BlueprintReadWrite, Category = "Traffic|Steering", meta = (ClampMin = "50", ClampMax = "980"))
	float LateralAccelBudgetCmPerSec2;

	/**
	 * Safety multiplier applied to the physics-derived curve speed limit.
	 * Accounts for the gap between theoretical lateral-g and what the
	 * tire model can actually sustain. Lower = more conservative.
	 * Default 0.85 (15% margin below theoretical limit).
	 */
	UPROPERTY(EditAnywhere, BlueprintReadWrite, Category = "Traffic|Steering", meta = (ClampMin = "0.5", ClampMax = "1.0"))
	float CurveSpeedSafetyFactor;

	/**
	 * Number of polyline segments accumulated in the sliding window
	 * when scanning for upcoming curvature. Larger windows detect
	 * gradual curves on dense polylines; smaller windows respond
	 * to sharp kinks. Default 5. Must be >= 2.
	 */
	UPROPERTY(EditAnywhere, BlueprintReadWrite, Category = "Traffic|Steering", meta = (ClampMin = "2", ClampMax = "20"))
	int32 CurveScanWindowSize;

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

	// ── IDM (Intelligent Driver Model) Tuning ───────────────
	// FollowingTimeSec  → IDM time headway T  (above, "Traffic|Proximity")
	// MinFollowingDistanceCm → IDM jam distance s₀ (above, "Traffic|Proximity")

	/**
	 * Maximum comfortable acceleration (cm/s²).
	 * Higher = more aggressive starts, lower = smoother roll-up.
	 * Default 150 ≈ 1.5 m/s² (typical passenger sedan).
	 */
	UPROPERTY(EditAnywhere, BlueprintReadWrite, Category = "Traffic|IDM", meta = (ClampMin = "50"))
	float IDMMaxAccelCmPerSec2;

	/**
	 * Comfortable braking deceleration (cm/s²).
	 * IDM avoids braking harder than this except in emergencies.
	 * Default 300 ≈ 3.0 m/s² (comfortable stop).
	 */
	UPROPERTY(EditAnywhere, BlueprintReadWrite, Category = "Traffic|IDM", meta = (ClampMin = "50"))
	float IDMComfortDecelCmPerSec2;

	/**
	 * Perception-reaction delay (seconds). Leader state is buffered and
	 * read back this many seconds later, so vehicles in a platoon brake
	 * with a realistic cascade instead of all simultaneously.
	 * 0 = instant reaction.  Default 0.3s (typical alert driver).
	 */
	UPROPERTY(EditAnywhere, BlueprintReadWrite, Category = "Traffic|IDM", meta = (ClampMin = "0", ClampMax = "2.0"))
	float IDMReactionDelaySec;

	/**
	 * AEB (Automatic Emergency Braking) time-to-collision threshold (seconds).
	 * If the raw (undelayed) TTC to the leader drops below this value,
	 * the vehicle bypasses the reaction delay buffer and applies maximum
	 * braking immediately. Default 1.5s.
	 */
	UPROPERTY(EditAnywhere, BlueprintReadWrite, Category = "Traffic|IDM", meta = (ClampMin = "0.3", ClampMax = "5.0"))
	float AEBTimeToCollisionThresholdSec;

	/**
	 * Per-vehicle personality spread (0-1). Each vehicle's IDM parameters
	 * (a, b, T) are randomly varied by ± this fraction using the
	 * deterministic RandomStream. 0 = all identical, 0.15 = ±15% variation.
	 * Default 0.15 for natural-looking traffic.
	 */
	UPROPERTY(EditAnywhere, BlueprintReadWrite, Category = "Traffic|IDM", meta = (ClampMin = "0", ClampMax = "0.5"))
	float IDMPersonalitySpread;

	/**
	 * Smoothing time constant (seconds) for the IDM acceleration output.
	 * Prevents rapid throttle/brake oscillation at equilibrium.
	 * 0 = no smoothing.  Default 0.15s.
	 */
	UPROPERTY(EditAnywhere, BlueprintReadWrite, Category = "Traffic|IDM", meta = (ClampMin = "0", ClampMax = "1.0"))
	float IDMSmoothingTauSec;

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
	 * Target length (cm) per segment when synthesizing junction Hermite curves.
	 * Lower values produce smoother curves (more segments) at minor memory cost.
	 * Default 200 = 2m per segment. Increase for performance, decrease for
	 * smoother interchange ramps.
	 */
	UPROPERTY(EditAnywhere, BlueprintReadWrite, Category = "Traffic|Intersection", meta = (ClampMin = "50", ClampMax = "1000"))
	float JunctionCurveResolutionCm;

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
	 * force-proceeding to break potential deadlocks. 0 = wait forever (no
	 * timeout). Default 90s.
	 */
	UPROPERTY(EditAnywhere, BlueprintReadWrite, Category = "Traffic|Intersection", meta = (ClampMin = "0"))
	float MaxIntersectionWaitTimeSec;

	/** Cached forward extent of the vehicle mesh bounding box (cm from actor
	 *  origin to front bumper). Computed once on possess from the pawn's
	 *  component bounds. Currently used for diagnostics/logging and
	 *  reserved for future stop-line alignment logic. */
	float VehicleFrontExtent = 0.0f;

	/** Rear extent (cm) — distance from actor origin to rear bumper.
	 *  Computed once on possess from the pawn's component bounds. */
	float VehicleRearExtent = 0.0f;

	/** Lateral half-width (cm) — distance from actor origin to side.
	 *  Computed once on possess. Used for lane-change gap checks. */
	float VehicleLateralHalfWidth = 0.0f;

	/** Vehicle wheelbase (cm) — distance between front and rear axles.
	 *  Computed from Chaos wheel setup in OnPossess; fallback 280 cm. */
	float VehicleWheelbaseCm = 280.0f;

	/** Maximum steering angle (radians) from the front wheels' Chaos config.
	 *  Computed in OnPossess; fallback 35°. */
	float VehicleMaxSteerAngleRad = 35.0f * UE_PI / 180.0f;

	/** Tick counter for throttled steering diagnostics (logs every N ticks). */
	int32 SteeringDiagCounter = 0;

	// ── Physics Safety Net State ────────────────────────────

	/** Accumulated seconds the vehicle has been continuously flipped / airborne.
	 *  After FlipDespawnTimeSec, the vehicle is safety-despawned. */
	float FlipTimeAccumulator = 0.0f;

	/** Accumulated seconds the vehicle has been continuously stuck.
	 *  After StuckDespawnTimeSec, the vehicle is safety-despawned. */
	float StuckTimeAccumulator = 0.0f;

	/** Throttle timer for the wake guard so it fires at most once per second
	 *  instead of every single tick (avoids energy injection). */
	float WakeGuardCheckTimer = 0.0f;

	/** Remaining seconds of full-braking forced by the reactive collision handler. */
	float CollisionBrakeTimer = 0.0f;

	// ── IDM Runtime State ───────────────────────────────────

	/** Per-vehicle personality multiplier for max acceleration (set in SetRandomSeed). */
	float IDMPersonalityAccelScale = 1.0f;

	/** Per-vehicle personality multiplier for comfortable decel (set in SetRandomSeed). */
	float IDMPersonalityDecelScale = 1.0f;

	/** Per-vehicle personality multiplier for time headway (set in SetRandomSeed). */
	float IDMPersonalityTimeHeadwayScale = 1.0f;

	/** Smoothed IDM acceleration from previous tick (for exponential filter). */
	float SmoothedIDMAccel = 0.0f;

	/** Vehicle-specific maximum brake deceleration (cm/s²), computed from
	 *  Chaos wheel MaxBrakeTorque and vehicle mass in OnPossess.
	 *  Used for emergency braking normalization.  Fallback = 600 (~6 m/s²). */
	float MaxBrakeDecelCmPerSec2 = 600.0f;

	/** Ring-buffer of (Time, LeaderDist, LeaderSpeed) for reaction delay. */
	struct FLeaderSample
	{
		float Time = 0.0f;
		float Dist = -1.0f;
		float Speed = 0.0f;
	};
	TArray<FLeaderSample> LeaderDelayBuffer;

	/** Monotonic clock for indexing into the reaction-delay buffer. */
	float IDMTimeClock = 0.0f;

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

	/** Seconds a vehicle must be continuously flipped / airborne before despawn.
	 *  1.5 s is enough to filter brief bumps from terrain. */
	static constexpr float FlipDespawnTimeSec = 1.5f;

	/** Seconds a vehicle must be continuously stuck before despawn.
	 *  5.0 s is a generous grace period for temporary stops. */
	static constexpr float StuckDespawnTimeSec = 5.0f;

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
