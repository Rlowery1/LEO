// Copyright AAA_Traffic Contributors. All Rights Reserved.

#pragma once

#include "CoreMinimal.h"
#include "TrafficRoadProvider.h" // FTrafficLaneHandle, ITrafficRoadProvider, ETrafficLaneSide

// Forward declarations — full definitions live in TrafficVehicleController.h
// which is always included before this header reaches a translation unit.
enum class ELaneChangeState : uint8;
enum class ETurnSignalState : uint8;
class ATrafficVehicleController;
class UTrafficSubsystem;

/** Phase of an overtaking maneuver (lateral offset, no actual lane change). */
enum class EOvertakePhase : uint8
{
	None,       // No active overtake.
	PullingOut, // Blending lateral offset toward oncoming lane.
	Passing,    // Holding offset, traveling past the slow leader.
	PullingIn   // Blending offset back to zero.
};

// ---------------------------------------------------------------------------
// Input / output structs
// ---------------------------------------------------------------------------

/** Context needed to evaluate whether a lane change should begin. */
struct AAA_TRAFFIC_API FLaneChangeEvalContext
{
	FVector VehicleLocation = FVector::ZeroVector;
	FVector EgoForward     = FVector::ForwardVector;
	float   CurrentSpeed   = 0.0f;
	float   TargetSpeed    = 0.0f;
	float   EgoForwardSpeed = 0.0f; // Abs(VehicleMovement->GetForwardSpeed())

	float VehicleFrontExtent = 0.0f;
	float VehicleRearExtent  = 0.0f;
	float DetectionDistance   = 0.0f;

	// Lane context
	FTrafficLaneHandle      CurrentLane;
	const TArray<FVector>*  LanePoints      = nullptr;
	int32                   LastClosestIndex = 0;
	bool                    bLaneDataReady   = false;

	// Junction suppression (replaces direct EJunctionPhase dependency)
	bool bJunctionApproaching = false;

	// Leader info (caller must compute via GetLeaderDistance before calling)
	float LeaderDist  = -1.0f;
	float LeaderSpeed = 0.0f;

	// External dependencies (not owned)
	ITrafficRoadProvider*  RoadProvider  = nullptr;
	UTrafficSubsystem*     TrafficSub   = nullptr;
};

/** Result of a lane change evaluation. */
struct AAA_TRAFFIC_API FLaneChangeEvalResult
{
	bool bStarted;
	ETurnSignalState TurnSignal; // only meaningful when bStarted == true
};

/** Context needed for per-tick lane change blending. */
struct AAA_TRAFFIC_API FLaneChangeBlendContext
{
	FVector VehicleLocation = FVector::ZeroVector;
	FVector SourcePoint     = FVector::ZeroVector; // GetLookAheadPoint result on source lane
	int32   ClosestIndex    = 0;
	float   LookAheadDistance = 0.0f;
	float   DistanceThisTick  = 0.0f;

	UTrafficSubsystem* TrafficSub = nullptr;
};

/** Result of per-tick blending. */
struct AAA_TRAFFIC_API FLaneChangeBlendResult
{
	FVector Point = FVector::ZeroVector;
	bool bAborted = false; // true if safety check forced abort during blend
};

/** Data returned when a lane change finalizes — caller applies to controller. */
struct AAA_TRAFFIC_API FLaneChangeFinalizeResult
{
	FTrafficLaneHandle  NewLane;
	TArray<FVector>     NewLanePoints;
	float               NewLaneWidth     = 0.0f;
	float               NewTargetSpeed   = 0.0f;
};

// ---------------------------------------------------------------------------
// Coordinator struct — owns the lane-change state machine
// ---------------------------------------------------------------------------

/**
 * Stateful lane-change coordinator: evaluate candidates, blend between
 * source/target lanes, settle, and finalize.
 * Plain struct (not a UObject).  Lives as a member of the controller.
 */
struct AAA_TRAFFIC_API FLaneChangeCoordinator
{
	// ── Public state (readable by controller + other vehicles for blind-spot) ──

	ELaneChangeState State;
	float            Progress           = 0.0f;
	float            SettleTimer        = 0.0f;
	float            CooldownRemaining  = 0.0f;

	FTrafficLaneHandle TargetLane;
	TArray<FVector>    TargetLanePoints;
	float              TargetLaneWidth = 0.0f;

	// Navigation pre-positioning (shared with junction approach logic).
	// NOTE: These fields are set by JunctionNegotiator to reuse lane-change
	// blend machinery for junction pre-positioning. This is an intentional
	// coupling — the negotiator sets bNavigationalLaneChange + target lane,
	// and the coordinator's Evaluate() prioritizes it over comfort changes.
	bool               bNavigationalLaneChange = false;
	FTrafficLaneHandle NavigationalTargetLane;

	// ── Config (synced from UPROPERTY tuning knobs on the controller) ──

	float Distance       = 1500.0f;
	float CooldownTime   = 5.0f;
	float SpeedThreshold = 0.6f;
	float GapRequired    = 800.0f;

	// ── Owner ──

	ATrafficVehicleController* Owner = nullptr;

	// ── Constructor ──

	FLaneChangeCoordinator();

	// ── Methods ──

	/** Map 0-1 aggression to internal config knobs. */
	void SetAggression(float Aggression);

	/** Decrement cooldown timer. Call once per tick at the start. */
	void TickCooldown(float DeltaSeconds);

	/** True if the state machine is idle and cooldown has expired. */
	bool ShouldEvaluate() const;

	/** Evaluate adjacent lanes and potentially begin a lane change. */
	FLaneChangeEvalResult Evaluate(const FLaneChangeEvalContext& Ctx);

	/**
	 * Per-tick blend between source and target lanes.
	 * May internally abort (sets bAborted in result) if continuous safety check fails.
	 */
	FLaneChangeBlendResult Blend(const FLaneChangeBlendContext& Ctx);

	/** Decrement settle timer.  Returns true when the timer expires (finalize needed). */
	bool TickSettle(float DeltaSeconds);

	/**
	 * Finalise the lane change — populate result with new lane data.
	 * Resets internal state, starts cooldown.
	 */
	FLaneChangeFinalizeResult Finalize(
		ITrafficRoadProvider* Provider,
		float DefaultSpeedLimit,
		float BaseTargetSpeed);

	/** Abort the lane change — resets state and starts cooldown. */
	void Abort();

	/** Quick state reset (no cooldown, no logging).  Used by junction / transition gates. */
	void Reset();

	/** Clear only navigational pre-positioning state. */
	void ClearNav();

	// ── Overtaking ──
	// Two modes: (a) lateral-offset into oncoming lane (legacy, disabled on 2-lane 2-way),
	// (b) same-direction lane change tagged as overtake for tracking.

	bool             bSameDirectionOvertake = false; // True when overtaking via same-dir lane change
	EOvertakePhase   OvertakePhase        = EOvertakePhase::None;
	float            OvertakeLateralOffset = 0.0f;   // Current smooth-blended offset (cm)
	float            OvertakeBlendProgress = 0.0f;   // 0..1 for pull-out / pull-in
	float            OvertakeStuckTimer    = 0.0f;   // Accumulated stuck-behind-leader time
	float            OvertakePassDistAccum = 0.0f;   // Forward distance in Passing phase
	float            OvertakeCooldownRemaining = 0.0f;
	float            OvertakeTargetOffset  = 0.0f;   // Full offset magnitude (≈ lane width)
	ETrafficLaneSide OvertakeSide  = ETrafficLaneSide::Left;
	FTrafficLaneHandle OvertakeOncomingLane;

	// Config (synced from controller UPROPERTYs)
	float OvertakeStuckTimeSec    = 4.0f;     // Oncoming-lane overtake
	float SameDirOvertakeStuckTimeSec = 1.5f; // Same-direction lane change
	float OvertakeMinClearanceCm  = 5000.0f;  // 50m to start
	float OvertakeAbortClearanceCm = 5000.0f;  // 50m to abort
	float OvertakeSpeedBoostPct   = 15.0f;     // +15%
	float OvertakeMinPassDistCm   = 2000.0f;   // 20m in Passing before pull-in
	float OvertakeBlendDistCm     = 1200.0f;   // Lateral blend distance
	float OvertakeCooldownTimeSec = 20.0f;

	/** Decrement overtake cooldown. */
	void TickOvertakeCooldown(float DeltaSeconds);

	/** True if any overtake phase is active. */
	bool IsOvertaking() const;

	/** Current lateral offset (cm) for steering target adjustment. */
	float GetOvertakeLateralOffset() const;

	/** Speed boost factor (e.g. 1.15) while overtaking, 1.0 otherwise. */
	float GetOvertakeSpeedBoostFactor() const;

	/**
	 * Accumulate stuck time and evaluate whether to begin an overtake.
	 * Called from the controller's UpdateInput when no lane-change is active.
	 */
	void EvaluateOvertake(const FLaneChangeEvalContext& Ctx, float DeltaSeconds);

	/**
	 * Per-tick overtake phase advancement: blend offset, check clearance,
	 * transition between phases.
	 */
	void TickOvertake(float DeltaSeconds, float DistanceThisTick,
		const FLaneChangeEvalContext& Ctx);

	/** Abort an active overtake — immediately begins pulling in. */
	void AbortOvertake();
};
