// SpeedEnvelope.cpp — Speed-cap stack implementation.

#include "SpeedEnvelope.h"
#include "TrafficRoadProvider.h"
#include "TrafficLog.h"

FSpeedEnvelopeOutput FSpeedEnvelope::Compute(const FSpeedEnvelopeInput& In) const
{
	FSpeedEnvelopeOutput Out;
	Out.EffectiveTargetSpeed = In.TargetSpeed;

	// ── Approach braking ─────────────────────────────────────
	// Clamp DOWN only — never raise speed above current effective.
	if (In.bApproachBraking)
	{
		const float ApproachCap = FMath::Max(In.ApproachSpeedLimitCmPerSec, 100.0f);
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

		for (int32 Hop = 0; Hop < 3 && NextSpeedLimit < 0.0f; ++Hop)
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
			const int32 CurveStart = FMath::Clamp(
				In.JunctionCurveStartIndex, 0, NumPts - 3);

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

			const float TotalAngleRad = FMath::DegreesToRadians(
				FMath::Max(TotalAngleDeg, 1.0f));
			const float TurnRadius = FMath::Max(
				TotalArcLength / TotalAngleRad, 50.0f);
			const float CurvatureSpeed = FMath::Sqrt(
				In.LateralAccelBudgetCmPerSec2 * TurnRadius);

			JunctionSpeedCap = FMath::Min(JunctionSpeedCap, CurvatureSpeed);

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
			ScanRange = TNumericLimits<float>::Max();
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
			if (WinEndIdx <= WinStartIdx) { break; }

			float WinAngleRad = 0.0f;
			float WinArc = 0.0f;
			for (int32 i = WinStartIdx; i < WinEndIdx; ++i)
			{
				const FVector Seg = In.LanePoints[i + 1] - In.LanePoints[i];
				WinArc += Seg.Size();
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

			if (WinAngleRad > 0.052f && WinArc > KINDA_SMALL_NUMBER)
			{
				const float Radius = FMath::Max(WinArc / WinAngleRad, 50.0f);
				if (Radius < MinTurnRadius)
				{
					MinTurnRadius = Radius;
					DistToTightestCurve = WinStartDist;
				}
			}
		}

		if (MinTurnRadius < TNumericLimits<float>::Max())
		{
			const float RawCurveSpeed = FMath::Sqrt(
				In.LateralAccelBudgetCmPerSec2 * MinTurnRadius);
			const float CurveSpeedLimit =
				RawCurveSpeed * In.CurveSpeedSafetyFactor;

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

	return Out;
}
