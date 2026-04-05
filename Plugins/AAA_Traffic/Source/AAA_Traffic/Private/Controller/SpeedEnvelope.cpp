// SpeedEnvelope.cpp — Speed-cap stack implementation.

#include "SpeedEnvelope.h"
#include "TrafficRoadProvider.h"
#include "TrafficLog.h"

// ---------------------------------------------------------------------------
// Shared helper: compute MINIMUM turn radius from a polyline segment range
// using 3-point circumradius at each interior vertex.
// Returns the tightest radius in cm (clamped to a minimum of 50 cm).
// Using the minimum (not average) ensures the speed cap accounts for the
// sharpest section of the curve, preventing corner-clipping.
// ---------------------------------------------------------------------------
static float ComputePolylineTurnRadius(
	const FVector* Points, int32 StartIdx, int32 EndIdx)
{
	float MinRadius = TNumericLimits<float>::Max();

	for (int32 i = StartIdx + 1; i < EndIdx; ++i)
	{
		const FVector SegA = Points[i] - Points[i - 1];
		const FVector SegB = Points[i + 1] - Points[i];
		const float CrossMag = FMath::Abs(FVector::CrossProduct(SegA, SegB).Z);
		if (CrossMag <= KINDA_SMALL_NUMBER)
		{
			continue; // Nearly collinear — infinite radius, skip.
		}

		const float ChordLength = FVector::Dist(Points[i - 1], Points[i + 1]);
		const float Numerator = SegA.Size() * SegB.Size() * ChordLength;
		if (Numerator <= KINDA_SMALL_NUMBER)
		{
			continue;
		}

		const float Radius = Numerator / (2.0f * CrossMag);
		if (Radius > 0.0f)
		{
			MinRadius = FMath::Min(MinRadius, Radius);
		}
	}

	// If all segments were collinear, fall back to a large-radius estimate
	// using total arc/angle (straight-ish path).
	if (MinRadius == TNumericLimits<float>::Max())
	{
		float TotalAngleRad = 0.0f;
		float TotalArcLength = 0.0f;
		for (int32 i = StartIdx + 1; i < EndIdx; ++i)
		{
			const FVector Seg0 = Points[i] - Points[i - 1];
			const FVector Seg1 = Points[i + 1] - Points[i];
			TotalArcLength += Seg0.Size();
			const FVector Dir0 = Seg0.GetSafeNormal();
			const FVector Dir1 = Seg1.GetSafeNormal();
			if (!Dir0.IsNearlyZero() && !Dir1.IsNearlyZero())
			{
				const float Dot = FMath::Clamp(
					FVector::DotProduct(Dir0, Dir1), -1.0f, 1.0f);
				TotalAngleRad += FMath::Acos(Dot);
			}
		}
		TotalArcLength += (Points[EndIdx] - Points[EndIdx - 1]).Size();
		const float SafeAngle = FMath::Max(TotalAngleRad, FMath::DegreesToRadians(1.0f));
		MinRadius = TotalArcLength / SafeAngle;
	}

	return FMath::Max(MinRadius, 50.0f);
}

FSpeedEnvelopeOutput FSpeedEnvelope::Compute(const FSpeedEnvelopeInput& In) const
{
	FSpeedEnvelopeOutput Out;
	Out.EffectiveTargetSpeed = In.TargetSpeed;

	// Speed-cap layer stack (each layer may only LOWER EffectiveTargetSpeed):
	//   1. Approach braking — decelerate toward junction entry point
	//   2. Speed-zone anticipation — brake ahead of lower-limit road segments
	//   3. Intersection curvature cap — physics-safe speed through junction curve
	//   4. Predictive curve cap — look-ahead curvature on regular lane
	// All layers are independent; the final speed is the MIN of all active caps.

	// ── Approach braking ─────────────────────────────────────
	// Clamp DOWN only — never raise speed above current effective.
	if (In.bApproachBraking)
	{
		// Negative speed limit signals a full-stop request (proximity-conflict
		// junction that is currently occupied).  Bypass the 100 cm/s floor.
		const float ApproachCap = (In.ApproachSpeedLimitCmPerSec < 0.0f)
			? 0.0f
			: FMath::Max(In.ApproachSpeedLimitCmPerSec, 100.0f);
		if (ApproachCap < Out.EffectiveTargetSpeed)
		{
			Out.EffectiveTargetSpeed = ApproachCap;
		}
		Out.DbgAppliedCap = TEXT("APPROACH");
	}

	// ── Speed-zone anticipatory braking ──────────────────────
	// Walk connected lanes to find a lower speed limit ahead. If found,
	// compute braking envelope to start slowing before the zone boundary.
	if (In.RoadProvider && In.bLaneDataReady && In.CurrentJunctionId == 0)
	{
		float ScanDist = In.RemainingDistOnLane;
		FTrafficLaneHandle ScanLane = In.CurrentLane;
		float NextSpeedLimit = -1.0f;

		for (int32 Hop = 0; Hop < In.SpeedZoneHopLimit && NextSpeedLimit < 0.0f; ++Hop)
		{
			TArray<FTrafficLaneHandle> Connected =
				In.RoadProvider->GetConnectedLanes(ScanLane);
			if (Connected.IsEmpty()) { break; }

			Connected.Sort([](const FTrafficLaneHandle& A, const FTrafficLaneHandle& B)
			{ return A.HandleId < B.HandleId; });

			bool bFoundNext = false;
			for (const FTrafficLaneHandle& Next : Connected)
			{
				if (In.RoadProvider->GetJunctionForLane(Next) != 0) { continue; }
				const float Limit = In.RoadProvider->GetLaneSpeedLimit(Next);
				if (Limit > 0.0f && Limit < In.TargetSpeed - 50.0f)
				{
					NextSpeedLimit = Limit;
					bFoundNext = true;
					break;
				}
				ScanDist += In.RoadProvider->GetLaneLength(Next);
				ScanLane = Next;
				bFoundNext = true;
				break;
			}
			if (!bFoundNext) { break; }
		}

		if (NextSpeedLimit > 0.0f && In.CurrentSpeed > NextSpeedLimit)
		{
			const float BrakeDist =
				(In.CurrentSpeed * In.CurrentSpeed - NextSpeedLimit * NextSpeedLimit)
				/ (2.0f * FMath::Max(In.ApproachDecelCmPerSec2, 100.0f));

			if (ScanDist < BrakeDist + In.ApproachSafetyMarginCm)
			{
				const float SafeDist = FMath::Max(
					ScanDist - In.ApproachSafetyMarginCm, 0.0f);
				const float EnvSpeed = FMath::Sqrt(
					NextSpeedLimit * NextSpeedLimit
					+ 2.0f * In.ApproachDecelCmPerSec2 * SafeDist);
				if (EnvSpeed < Out.EffectiveTargetSpeed)
				{
					Out.EffectiveTargetSpeed = FMath::Max(EnvSpeed, NextSpeedLimit);
				}
			}
		}
	}

	// ── Intersection speed cap ───────────────────────────────
	// While traversing a junction, cap speed using the tunable baseline
	// and curvature-derived limit from the junction polyline.
	if (In.CurrentJunctionId != 0)
	{
		float JunctionSpeedCap = In.IntersectionSpeedLimitCmPerSec;

		const int32 NumPts = In.JunctionTransitionPoints.Num();
		if (NumPts >= 3)
		{
			// Use the ENTIRE curve for minimum-radius computation — not just
			// ahead of the vehicle.  The old approach (starting from the
			// vehicle's current curve index) missed the tightest apex once
			// the vehicle moved past it, causing the speed cap to jump from
			// e.g. 181 → 748 mid-turn and allowing the vehicle to accelerate
			// right when it still had large heading error.
			constexpr int32 CurveStart = 0;

			const float TurnRadius = ComputePolylineTurnRadius(
				In.JunctionTransitionPoints.GetData(), CurveStart, NumPts - 1);

			// Recompute arc/angle for diagnostics.
			float TotalAngleDeg = 0.0f;
			float TotalArcLength = 0.0f;
			for (int32 i = CurveStart + 1; i < NumPts - 1; ++i)
			{
				const FVector Seg0 = In.JunctionTransitionPoints[i]
					- In.JunctionTransitionPoints[i - 1];
				const FVector Seg1 = In.JunctionTransitionPoints[i + 1]
					- In.JunctionTransitionPoints[i];
				TotalArcLength += Seg0.Size();
				const FVector Dir0 = Seg0.GetSafeNormal();
				const FVector Dir1 = Seg1.GetSafeNormal();
				if (!Dir0.IsNearlyZero() && !Dir1.IsNearlyZero())
				{
					const float Dot = FMath::Clamp(
						FVector::DotProduct(Dir0, Dir1), -1.0f, 1.0f);
					TotalAngleDeg += FMath::RadiansToDegrees(FMath::Acos(Dot));
				}
			}
			TotalArcLength += (In.JunctionTransitionPoints[NumPts - 1]
				- In.JunctionTransitionPoints[NumPts - 2]).Size();

			const float CurvatureSpeed = FMath::Sqrt(
				In.LateralAccelBudgetCmPerSec2 * TurnRadius)
				* In.CurveSpeedSafetyFactor;

			JunctionSpeedCap = FMath::Min(JunctionSpeedCap, CurvatureSpeed);
			Out.DbgJunctionTurnRadiusCm = TurnRadius;
			Out.DbgJunctionArcLengthCm = TotalArcLength;
			Out.DbgJunctionTotalAngleDeg = TotalAngleDeg;
			Out.DbgJunctionSpeedCapCmPerSec = JunctionSpeedCap;

			UE_LOG(LogAAATraffic, Verbose,
				TEXT("JNCT SPEED-CAP: CurveStart=%d pts=%d arc=%.0f "
					 "angle=%.1f° R=%.0f speed=%.0f cap=%.0f"),
				CurveStart, NumPts, TotalArcLength, TotalAngleDeg,
				TurnRadius, CurvatureSpeed, JunctionSpeedCap);
		}

		Out.EffectiveTargetSpeed = FMath::Min(
			Out.EffectiveTargetSpeed, JunctionSpeedCap);
	}

	// ── Predictive curve speed reduction ─────────────────────
	// Scan upcoming polyline curvature, compute physics-safe speed,
	// then apply kinematic braking envelope to decelerate in time.
	const int32 NumLanePts = In.LanePoints.Num();
	if (NumLanePts >= 3 && In.ClosestIndex < NumLanePts - 2)
	{
		const float AbsSpeed = FMath::Abs(In.CurrentSpeed);
		const float StoppingDist = (AbsSpeed > KINDA_SMALL_NUMBER)
			? (AbsSpeed * AbsSpeed) / (2.0f * In.MaxBrakeDecelCmPerSec2)
			: 0.0f;

		float ScanRange;
		if (In.bFirstTickOnLane)
		{
			ScanRange = 10000.0f; // 100m — sufficient for full braking from max speed
			Out.bFirstTickOnLaneConsumed = true;
		}
		else
		{
			ScanRange = FMath::Max(
				StoppingDist + In.LookAheadDistance, 800.0f);
		}

		const int32 CurveStart = FMath::Clamp(
			In.ClosestIndex, 0, NumLanePts - 2);
		const int32 WinSize = FMath::Max(In.CurveScanWindowSize, 2);
		const int32 LastSegIdx = NumLanePts - 2;

		// Pre-compute cumulative distance from CurveStart.
		TArray<float, TInlineAllocator<128>> CumDist;
		{
			float Accum = 0.0f;
			CumDist.Add(0.0f);
			for (int32 i = CurveStart; i < LastSegIdx
				&& Accum < ScanRange + 2000.0f; ++i)
			{
				Accum += FVector::Dist(In.LanePoints[i], In.LanePoints[i + 1]);
				CumDist.Add(Accum);
			}
		}
		const int32 NumScannableSegs = CumDist.Num() - 1;

		float MinTurnRadius = TNumericLimits<float>::Max();
		float DistToTightestCurve = 0.0f;

		for (int32 w = 0; w < NumScannableSegs; ++w)
		{
			const float WinStartDist = CumDist[w];
			if (WinStartDist > ScanRange) { break; }

			const int32 WinStartIdx = CurveStart + w;
			const int32 WinEndIdx = FMath::Min(WinStartIdx + WinSize, LastSegIdx);
			if (WinEndIdx <= WinStartIdx + 1) { break; }

			const float Radius = ComputePolylineTurnRadius(
				In.LanePoints.GetData(), WinStartIdx, WinEndIdx);

			// Only consider windows with meaningful deflection (> ~3 degrees).
			float WinAngleRad = 0.0f;
			for (int32 i = WinStartIdx; i < WinEndIdx; ++i)
			{
				const FVector Seg = In.LanePoints[i + 1] - In.LanePoints[i];
				if (i + 2 <= LastSegIdx + 1)
				{
					const FVector NextSeg =
						In.LanePoints[i + 2] - In.LanePoints[i + 1];
					const FVector Dir0 = Seg.GetSafeNormal();
					const FVector Dir1 = NextSeg.GetSafeNormal();
					if (!Dir0.IsNearlyZero() && !Dir1.IsNearlyZero())
					{
						const float Dot = FMath::Clamp(
							FVector::DotProduct(Dir0, Dir1), -1.0f, 1.0f);
						WinAngleRad += FMath::Acos(Dot);
					}
				}
			}

			if (WinAngleRad > 0.052f && Radius < MinTurnRadius)
			{
				MinTurnRadius = Radius;
				DistToTightestCurve = WinStartDist;
			}
		}

		if (MinTurnRadius < TNumericLimits<float>::Max())
		{
			const float RawCurveSpeed = FMath::Sqrt(
				In.LateralAccelBudgetCmPerSec2 * MinTurnRadius);
			const float CurveSpeedLimit =
				RawCurveSpeed * In.CurveSpeedSafetyFactor;
			Out.DbgPredictiveCurveRadiusCm = MinTurnRadius;
			Out.DbgPredictiveCurveDistCm = DistToTightestCurve;
			Out.DbgPredictiveCurveSpeedCmPerSec = CurveSpeedLimit;

			const float Decel = In.ComfortDecelCmPerSec2;
			const float Dist = FMath::Max(DistToTightestCurve, 0.0f);
			const float EnvelopeSpeed = FMath::Sqrt(
				CurveSpeedLimit * CurveSpeedLimit + 2.0f * Decel * Dist);

			if (EnvelopeSpeed < Out.EffectiveTargetSpeed)
			{
				Out.EffectiveTargetSpeed = EnvelopeSpeed;
			}
		}
	}

	// ── Merge zone deceleration ─────────────────────────────────
	// If the current lane has active segments (variable-width zones),
	// check whether the vehicle is approaching the end of its active
	// region. When the lane is about to narrow to zero width (merge
	// point), decelerate to give the vehicle time to lane-change.
	if (In.RoadProvider && In.bLaneDataReady && In.CurrentJunctionId == 0
		&& NumLanePts >= 2)
	{
		const TArray<FTrafficLaneSegment> ActiveSegs =
			In.RoadProvider->GetLaneActiveSegments(In.CurrentLane);
		if (!ActiveSegs.IsEmpty())
		{
			// Compute current distance along lane.
			float DistAtVehicle = 0.0f;
			const int32 VehIdx = FMath::Clamp(In.ClosestIndex, 0, NumLanePts - 1);
			for (int32 i = 0; i < VehIdx && i < NumLanePts - 1; ++i)
			{
				DistAtVehicle += FVector::Dist(In.LanePoints[i], In.LanePoints[i + 1]);
			}

			// Find the nearest active-segment end that is ahead of us.
			float NearestEndAhead = TNumericLimits<float>::Max();
			for (const FTrafficLaneSegment& Seg : ActiveSegs)
			{
				const float SegEnd = static_cast<float>(Seg.EndDistance);
				if (SegEnd > DistAtVehicle)
				{
					const float Ahead = SegEnd - DistAtVehicle;
					if (Ahead < NearestEndAhead)
					{
						NearestEndAhead = Ahead;
					}
				}
			}

			if (NearestEndAhead < TNumericLimits<float>::Max())
			{
				// Apply braking envelope toward a low speed at the merge end.
				// Use comfort deceleration for smooth merge behavior.
				// Don't command a full stop — let the lane-change coordinator
				// handle escape. Minimum speed 100 cm/s (walking pace).
				constexpr float MergeEndMinSpeed = 100.0f;
				const float Decel = In.ComfortDecelCmPerSec2;
				const float MergeDist = FMath::Max(NearestEndAhead, 0.0f);
				const float MergeEnvelopeSpeed = FMath::Sqrt(
					MergeEndMinSpeed * MergeEndMinSpeed
					+ 2.0f * Decel * MergeDist);

				if (MergeEnvelopeSpeed < Out.EffectiveTargetSpeed)
				{
					Out.EffectiveTargetSpeed = MergeEnvelopeSpeed;
				}
			}
		}
	}

	return Out;
}
