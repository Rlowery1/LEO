// SpeedEnvelope.h — Speed-cap stack: approach braking, intersection cap,
// curvature-based anticipatory braking, speed-zone lookahead.
// Pure computation; no UObject dependencies.

#pragma once

#include "CoreMinimal.h"
#include "TrafficRoadProvider.h"

class ITrafficRoadProvider;

/** Read-only inputs assembled by the controller each tick. */
struct AAA_TRAFFIC_API FSpeedEnvelopeInput
{
	float TargetSpeed           = 0.0f;   // road speed limit (cm/s)
	float CurrentSpeed          = 0.0f;   // |velocity| (cm/s)

	// ── Approach braking ─────────────────────────────────────
	bool  bApproachBraking      = false;
	float ApproachSpeedLimitCmPerSec = 0.0f;

	// ── Speed-zone anticipation (needs provider queries) ─────
	ITrafficRoadProvider* RoadProvider   = nullptr;
	FTrafficLaneHandle    CurrentLane;
	bool  bLaneDataReady                 = false;
	int32 CurrentJunctionId              = 0;
	float RemainingDistOnLane            = 0.0f;
	float ApproachDecelCmPerSec2         = 0.0f;
	float ApproachSafetyMarginCm         = 0.0f;

	// ── Intersection speed cap ───────────────────────────────
	float IntersectionSpeedLimitCmPerSec = 0.0f;
	float LateralAccelBudgetCmPerSec2    = 0.0f;
	TArrayView<const FVector> JunctionTransitionPoints;
	int32 JunctionCurveStartIndex        = 0;

	// ── Predictive curve speed reduction ─────────────────────
	TArrayView<const FVector> LanePoints;
	int32 ClosestIndex                   = 0;
	bool  bFirstTickOnLane               = false;
	float LookAheadDistance              = 0.0f;
	float MaxBrakeDecelCmPerSec2         = 0.0f;
	int32 CurveScanWindowSize            = 3;
	float CurveSpeedSafetyFactor         = 1.0f;
	float ComfortDecelCmPerSec2          = 0.0f;
};

struct AAA_TRAFFIC_API FSpeedEnvelopeOutput
{
	float EffectiveTargetSpeed       = 0.0f;
	bool  bFirstTickOnLaneConsumed   = false;  // caller should clear flag

	// Debug
	FString DbgAppliedCap;  // e.g. "APPROACH", empty if none
};

/** Stateless speed-cap stack. */
struct AAA_TRAFFIC_API FSpeedEnvelope
{
	FSpeedEnvelopeOutput Compute(const FSpeedEnvelopeInput& In) const;
};
