// Copyright AAA_Traffic Contributors. All Rights Reserved.

#pragma once

#include "CoreMinimal.h"

/**
 * Output from the steering computation — includes final input plus
 * individual component terms for diagnostics.
 */
struct FSteeringOutput
{
	/** Final steering input in [-1, 1]. */
	float SteeringInput = 0.0f;

	/** Curvature feedforward component. */
	float FeedforwardTerm = 0.0f;

	/** Pure pursuit heading component. */
	float PurePursuitTerm = 0.0f;

	/** Cross-track error correction component. */
	float CTETerm = 0.0f;

	/** Derivative (damping) component. */
	float DerivativeTerm = 0.0f;

	/** Signed cross-track error in cm (positive = right of centerline). */
	float SignedCTE = 0.0f;

	/** Local curvature at vehicle position (1/cm, signed). */
	float LocalCurvature = 0.0f;

	/** Half lane width used for CTE normalization (cm). */
	float HalfLaneWidth = 175.0f;
};

/**
 * Snapshot of all inputs the steering computer needs for one tick.
 * Assembled by the controller, consumed by FSteeringComputer::Compute.
 */
struct FSteeringInput
{
	FVector VehicleLocation = FVector::ZeroVector;
	FVector VehicleForward = FVector::ForwardVector;
	FVector TargetPoint = FVector::ZeroVector;

	/** Lane centerline polyline (for CTE and curvature). */
	TArrayView<const FVector> LanePoints;
	int32 ClosestIndex = 0;

	/** Junction transition curve (empty when not traversing). */
	TArrayView<const FVector> JunctionCurvePoints;
	int32 JunctionCurveIndex = 0;

	/** Vehicle geometry. */
	float WheelbaseCm = 280.0f;
	float MaxSteerAngleRad = 0.6108f; // ~35 degrees

	/** Tuning parameters. */
	float EffectiveLaneWidth = 350.0f;
	float CTECorrectionGain = 0.3f;
	float SteeringDampingFactor = 0.5f;
	int32 CurveScanWindowSize = 5;

	float DeltaSeconds = 0.0f;
};

/**
 * Pure-math steering computer. Computes steering input from:
 *   - Curvature feedforward (road shape anticipation)
 *   - Pure pursuit geometry (heading toward look-ahead point)
 *   - Cross-track error correction (centering on lane)
 *   - Derivative damping (suppresses oscillation)
 *
 * Stateful: holds PreviousHeadingCrossZ across ticks for the
 * derivative term. All other computation is stateless per tick.
 */
struct AAA_TRAFFIC_API FSteeringComputer
{
	/** Compute steering for one tick. */
	FSteeringOutput Compute(const FSteeringInput& In);

	/**
	 * Menger curvature at a polyline index.
	 * Returns signed curvature in 1/cm. Positive = left turn.
	 * @param Points     Polyline to evaluate.
	 * @param CenterIndex  Index around which to sample.
	 * @param WindowSize   Number of segments for the sampling spread.
	 */
	static float ComputeLocalCurvature(
		TArrayView<const FVector> Points,
		int32 CenterIndex,
		int32 WindowSize = 5);

	/** Reset inter-tick state (call on lane change, respawn, etc.). */
	void Reset();

	/** Previous-tick heading cross product for derivative term. */
	float PreviousHeadingCrossZ = 0.0f;
};
