// Copyright AAA_Traffic Contributors. All Rights Reserved.

#include "SteeringComputer.h"

// ---------------------------------------------------------------------------
// FSteeringComputer::Compute
// ---------------------------------------------------------------------------

FSteeringOutput FSteeringComputer::Compute(const FSteeringInput& In)
{
	FSteeringOutput Out;

	// ── Curvature feedforward ────────────────────────────────
	// Compute local curvature from the active polyline.
	if (In.JunctionCurvePoints.Num() > 0
		&& In.JunctionCurveIndex > 0
		&& In.JunctionCurveIndex < In.JunctionCurvePoints.Num())
	{
		Out.LocalCurvature = ComputeLocalCurvature(
			In.JunctionCurvePoints, In.JunctionCurveIndex, In.CurveScanWindowSize);
	}
	else if (In.LanePoints.Num() >= 3)
	{
		Out.LocalCurvature = ComputeLocalCurvature(
			In.LanePoints, In.ClosestIndex, In.CurveScanWindowSize);
	}

	// δ_ff = atan(Wheelbase × κ) / MaxSteerAngle
	Out.FeedforwardTerm = FMath::Atan(In.WheelbaseCm * Out.LocalCurvature)
		/ In.MaxSteerAngleRad;

	// ── Pure pursuit ─────────────────────────────────────────
	const FVector ToTargetVec = In.TargetPoint - In.VehicleLocation;
	const float Ld = ToTargetVec.Size2D();
	const FVector ToTargetDir = ToTargetVec.GetSafeNormal2D();
	const FVector Forward2D = In.VehicleForward.GetSafeNormal2D();
	const float CrossZ = FVector::CrossProduct(Forward2D, ToTargetDir).Z;
	Out.TargetDistance2D = Ld;
	Out.HeadingCrossZ = CrossZ;

	// Guard against noise amplification when target is very close.
	constexpr float MinLd = 50.0f;
	const float PurePursuitAngle = (Ld > MinLd)
		? FMath::Atan(2.0f * In.WheelbaseCm * CrossZ / Ld)
		: 0.0f;
	Out.PurePursuitTerm = PurePursuitAngle / In.MaxSteerAngleRad;

	// ── Cross-track error (CTE) ──────────────────────────────
	// Compute CTE against the ACTIVE path — junction curve when
	// traversing, road lane otherwise.  During junction traversal the
	// vehicle must stay on the junction curve, not the road lane.
	const bool bFollowingJunctionCurve = (In.JunctionCurvePoints.Num() > 0
		&& In.JunctionCurveIndex < In.JunctionCurvePoints.Num() - 1);
	Out.bFollowingJunctionCurve = bFollowingJunctionCurve;

	if (bFollowingJunctionCurve)
	{
		// CTE against the junction curve polyline.
		// JunctionCurveIndex is the NEXT target point, so the vehicle
		// is between [Index-1, Index].  Clamp to 0 for the first point.
		const int32 SegIdx = FMath::Clamp(
			In.JunctionCurveIndex - 1, 0, In.JunctionCurvePoints.Num() - 2);
		const FVector& SegA = In.JunctionCurvePoints[SegIdx];
		const FVector& SegB = In.JunctionCurvePoints[SegIdx + 1];
		const FVector SegDir = (SegB - SegA).GetSafeNormal2D();
		if (!SegDir.IsNearlyZero())
		{
			const FVector VehicleToSeg = In.VehicleLocation - SegA;
			Out.SignedCTE = FVector2D::CrossProduct(
				FVector2D(SegDir), FVector2D(VehicleToSeg));
		}
	}
	else if (In.LanePoints.Num() >= 2)
	{
		const int32 SegIdx = FMath::Clamp(In.ClosestIndex, 0, In.LanePoints.Num() - 2);
		const FVector& SegA = In.LanePoints[SegIdx];
		const FVector& SegB = In.LanePoints[SegIdx + 1];
		const FVector SegDir = (SegB - SegA).GetSafeNormal2D();
		if (!SegDir.IsNearlyZero())
		{
			const FVector VehicleToSeg = In.VehicleLocation - SegA;
			Out.SignedCTE = FVector2D::CrossProduct(
				FVector2D(SegDir), FVector2D(VehicleToSeg));
		}
	}

	// CTE → steering correction via atan soft saturation.
	Out.HalfLaneWidth = FMath::Max(In.EffectiveLaneWidth * 0.5f, 50.0f);
	const float CTENormalized = Out.SignedCTE / Out.HalfLaneWidth;
	Out.CTETerm = (In.CTECorrectionGain > KINDA_SMALL_NUMBER)
		? FMath::Atan(In.CTECorrectionGain * CTENormalized) / In.MaxSteerAngleRad
		: 0.0f;

	// Ramp CTE correction in gradually at junction curve start.
	// When entering a junction curve the vehicle heading may be misaligned
	// from the curve tangent.  Full CTE correction immediately would fight
	// pure pursuit and push the vehicle off-path.  Ramp over 0.5s lets
	// pure pursuit align the heading first.
	if (bFollowingJunctionCurve)
	{
		JunctionCurveFollowElapsed += In.DeltaSeconds;
		constexpr float RampDurationSec = 0.5f;
		Out.CTETerm *= FMath::Clamp(JunctionCurveFollowElapsed / RampDurationSec, 0.0f, 1.0f);
	}
	else
	{
		JunctionCurveFollowElapsed = 0.0f;
	}

	// ── Derivative damping (heading error rate) ─────────────
	// Dampens the rate of change of the pure-pursuit heading error
	// (CrossZ), NOT the CTE. This suppresses steering oscillation.
	const float HeadingErrorDerivative = (In.DeltaSeconds > KINDA_SMALL_NUMBER)
		? (CrossZ - PreviousHeadingCrossZ) / In.DeltaSeconds
		: 0.0f;
	PreviousHeadingCrossZ = CrossZ;

	Out.DerivativeTerm = FMath::Clamp(
		HeadingErrorDerivative * In.SteeringDampingFactor,
		-0.3f, 0.3f);

	// ── Combine ──────────────────────────────────────────────
	// CTE is SUBTRACTED — positive CTE (right of center) steers left.
	Out.SteeringInput = FMath::Clamp(
		Out.FeedforwardTerm + Out.PurePursuitTerm - Out.CTETerm + Out.DerivativeTerm,
		-1.0f, 1.0f);

	return Out;
}

// ---------------------------------------------------------------------------
// FSteeringComputer::ComputeLocalCurvature
// ---------------------------------------------------------------------------

float FSteeringComputer::ComputeLocalCurvature(
	TArrayView<const FVector> Points,
	int32 CenterIndex,
	int32 WindowSize)
{
	if (Points.Num() < 3) { return 0.0f; }

	const int32 Spread = FMath::Max(WindowSize / 2, 1);
	const int32 IdxA = FMath::Max(0, CenterIndex - Spread);
	const int32 IdxC = FMath::Min(Points.Num() - 1, CenterIndex + Spread);
	const int32 IdxB = FMath::Clamp(CenterIndex, IdxA + 1, IdxC - 1);

	if (IdxA == IdxB || IdxB == IdxC) { return 0.0f; }

	const FVector& A = Points[IdxA];
	const FVector& B = Points[IdxB];
	const FVector& C = Points[IdxC];

	const float AB = FVector::Dist2D(A, B);
	const float BC = FVector::Dist2D(B, C);
	const float CA = FVector::Dist2D(C, A);
	const float Denom = AB * BC * CA;

	if (Denom < KINDA_SMALL_NUMBER) { return 0.0f; }

	const float SignedArea2 = (B.X - A.X) * (C.Y - A.Y)
		- (B.Y - A.Y) * (C.X - A.X);
	return (2.0f * SignedArea2) / Denom;
}

// ---------------------------------------------------------------------------
// FSteeringComputer::Reset
// ---------------------------------------------------------------------------

void FSteeringComputer::Reset()
{
	PreviousHeadingCrossZ = 0.0f;
	JunctionCurveFollowElapsed = 0.0f;
}
