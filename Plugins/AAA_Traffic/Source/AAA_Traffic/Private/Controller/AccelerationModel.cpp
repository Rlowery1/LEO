// Copyright AAA_Traffic Contributors. All Rights Reserved.

#include "AccelerationModel.h"

// ---------------------------------------------------------------------------
// FAccelerationModel::SetPersonality
// ---------------------------------------------------------------------------

void FAccelerationModel::SetPersonality(float AccelScale, float DecelScale, float HeadwayScale)
{
	PersonalityAccelScale = AccelScale;
	PersonalityDecelScale = DecelScale;
	PersonalityHeadwayScale = HeadwayScale;
}

// ---------------------------------------------------------------------------
// FAccelerationModel::ClearDelayBuffer
// ---------------------------------------------------------------------------

void FAccelerationModel::ClearDelayBuffer()
{
	DelayBuffer.Empty();
	TimeClock = 0.0f;
	SmoothedAccel = 0.0f;
}

// ---------------------------------------------------------------------------
// FAccelerationModel::ProcessDelayBuffer
// ---------------------------------------------------------------------------

void FAccelerationModel::ProcessDelayBuffer(
	float DeltaSeconds,
	float ReactionDelay,
	float RawDist,
	float RawSpeed,
	float& OutDist,
	float& OutSpeed)
{
	TimeClock += DeltaSeconds;
	OutDist = RawDist;
	OutSpeed = RawSpeed;

	if (ReactionDelay <= KINDA_SMALL_NUMBER) { return; }

	// Push current sample.
	FLeaderSample Sample;
	Sample.Time = TimeClock;
	Sample.Dist = RawDist;
	Sample.Speed = RawSpeed;
	DelayBuffer.Add(Sample);

	// Read delayed sample with linear interpolation.
	const float ReadTime = TimeClock - ReactionDelay;
	if (DelayBuffer.Num() <= 1) { return; }

	int32 UpperIdx = DelayBuffer.Num() - 1;
	for (int32 i = DelayBuffer.Num() - 1; i >= 0; --i)
	{
		if (DelayBuffer[i].Time <= ReadTime)
		{
			UpperIdx = FMath::Min(i + 1, DelayBuffer.Num() - 1);
			break;
		}
	}
	const int32 LowerIdx = FMath::Max(UpperIdx - 1, 0);

	if (LowerIdx == UpperIdx)
	{
		OutDist = DelayBuffer[LowerIdx].Dist;
		OutSpeed = DelayBuffer[LowerIdx].Speed;
	}
	else
	{
		// Guard: straddles -1 ↔ positive boundary → snap to nearest.
		const bool bLowerValid = DelayBuffer[LowerIdx].Dist >= 0.0f;
		const bool bUpperValid = DelayBuffer[UpperIdx].Dist >= 0.0f;
		if (bLowerValid != bUpperValid)
		{
			const float T0 = DelayBuffer[LowerIdx].Time;
			const float T1 = DelayBuffer[UpperIdx].Time;
			const int32 SnapIdx = (FMath::Abs(ReadTime - T0) <= FMath::Abs(ReadTime - T1))
				? LowerIdx : UpperIdx;
			OutDist = DelayBuffer[SnapIdx].Dist;
			OutSpeed = DelayBuffer[SnapIdx].Speed;
		}
		else
		{
			const float T0 = DelayBuffer[LowerIdx].Time;
			const float T1 = DelayBuffer[UpperIdx].Time;
			const float Frac = (T1 > T0)
				? FMath::Clamp((ReadTime - T0) / (T1 - T0), 0.0f, 1.0f)
				: 0.0f;
			OutDist = FMath::Lerp(DelayBuffer[LowerIdx].Dist,
				DelayBuffer[UpperIdx].Dist, Frac);
			OutSpeed = FMath::Lerp(DelayBuffer[LowerIdx].Speed,
				DelayBuffer[UpperIdx].Speed, Frac);
		}
	}

	// Prune old entries (keep one before the read window).
	const int32 PruneUpTo = FMath::Max(0, LowerIdx - 1);
	if (PruneUpTo > 0)
	{
		DelayBuffer.RemoveAt(0, PruneUpTo, EAllowShrinking::No);
	}
}

// ---------------------------------------------------------------------------
// FAccelerationModel::Compute
// ---------------------------------------------------------------------------

FAccelerationOutput FAccelerationModel::Compute(const FAccelerationInput& In)
{
	FAccelerationOutput Out;

	// ── AEB: Automatic Emergency Braking ─────────────────────
	if (In.RawLeaderDist >= 0.0f)
	{
		const float ClosingSpeed = FMath::Abs(In.CurrentSpeed) - In.RawLeaderSpeed;
		if (ClosingSpeed > 50.0f)
		{
			const float TTC = In.RawLeaderDist / ClosingSpeed;
			if (TTC < In.AEBThresholdSec)
			{
				Out.bAEBActive = true;
			}

			// Stopping-distance AEB: fire emergency brake when comfort
			// deceleration cannot stop within 1.5× the kinematic distance.
			// Accounts for Chaos physics braking being weaker than the
			// parameterised comfort decel.
			if (!Out.bAEBActive)
			{
				const float StopDist = (ClosingSpeed * ClosingSpeed)
					/ (2.0f * FMath::Max(In.ComfortDecelCmPerSec2, 1.0f));
				if (StopDist * 1.5f >= In.RawLeaderDist)
				{
					Out.bAEBActive = true;
				}
			}
		}
	}

	// ── Reaction delay buffer ────────────────────────────────
	float DelayedDist = In.RawLeaderDist;
	float DelayedSpeed = In.RawLeaderSpeed;
	ProcessDelayBuffer(In.DeltaSeconds, In.ReactionDelaySec,
		In.RawLeaderDist, In.RawLeaderSpeed,
		DelayedDist, DelayedSpeed);

	// AEB or brake-light bypass: use instantaneous data.
	if (Out.bAEBActive || In.bLeaderBrakeLightsVisible)
	{
		DelayedDist = In.RawLeaderDist;
		DelayedSpeed = In.RawLeaderSpeed;
	}

	Out.DelayedLeaderDist = DelayedDist;
	Out.DelayedLeaderSpeed = DelayedSpeed;

	// ── IDM+ acceleration ────────────────────────────────────
	const float v = FMath::Max(FMath::Abs(In.CurrentSpeed), 0.0f);
	const float v0 = In.DesiredSpeed;

	const float a = In.MaxAccelCmPerSec2 * PersonalityAccelScale;
	const float b = In.ComfortDecelCmPerSec2 * PersonalityDecelScale;
	const float T = In.TimeHeadwaySec * PersonalityHeadwayScale;

	const float s0 = In.JamDistanceCm;

	const float vRatio = v / FMath::Max(v0, 1.0f);
	const float vRatio2 = vRatio * vRatio;
	const float FreeRoad = vRatio2 * vRatio2; // (v/v₀)⁴

	float IDMAccel;
	if (DelayedDist >= 0.0f)
	{
		const float s = FMath::Max(DelayedDist, 1.0f);
		const float DeltaV = v - DelayedSpeed;
		const float Interaction = v * DeltaV
			/ (2.0f * FMath::Sqrt(FMath::Max(a * b, 1.0f)));
		const float DesiredGap = s0 + FMath::Max(0.0f, v * T + Interaction);
		const float GapRatio = DesiredGap / s;
		const float GapTerm = GapRatio * GapRatio;

		if (v <= v0)
		{
			IDMAccel = a * (1.0f - FreeRoad - GapTerm);
		}
		else
		{
			const float AccelFree = -a * (FreeRoad - 1.0f);
			const float AccelGap = a * (1.0f - GapTerm);
			IDMAccel = FMath::Min(AccelFree, AccelGap);
		}
	}
	else
	{
		if (v <= v0)
		{
			IDMAccel = a * (1.0f - FreeRoad);
		}
		else
		{
			IDMAccel = -a * (FreeRoad - 1.0f);
		}
	}

	// ── Smoothing ────────────────────────────────────────────
	if (In.SmoothingTauSec > KINDA_SMALL_NUMBER)
	{
		const float Alpha = In.DeltaSeconds / (In.DeltaSeconds + In.SmoothingTauSec);
		SmoothedAccel = FMath::Lerp(SmoothedAccel, IDMAccel, Alpha);
		IDMAccel = SmoothedAccel;
	}

	Out.IDMAccel = IDMAccel;

	// ── Throttle / brake mapping ─────────────────────────────
	constexpr float DeadZoneFraction = 0.05f;
	const float DeadZone = In.MaxAccelCmPerSec2 * DeadZoneFraction;

	if (IDMAccel > DeadZone)
	{
		Out.ThrottleInput = FMath::Clamp(
			IDMAccel / FMath::Max(In.MaxAccelCmPerSec2, 1.0f), 0.0f, 1.0f);
	}
	else if (IDMAccel < -DeadZone)
	{
		const float BrakeNorm = (IDMAccel < -In.ComfortDecelCmPerSec2)
			? In.MaxBrakeDecelCmPerSec2
			: In.ComfortDecelCmPerSec2;
		Out.BrakeInput = FMath::Clamp(
			-IDMAccel / FMath::Max(BrakeNorm, 1.0f), 0.0f, 1.0f);
	}

	// Minimum throttle nudge at near-standstill.
	if (IDMAccel > 0.0f && FMath::Abs(In.CurrentSpeed) < 50.0f
		&& Out.ThrottleInput < 0.08f)
	{
		Out.ThrottleInput = 0.08f;
	}

	// Brake light state with hysteresis to prevent flickering at idle.
	// When waiting at a junction: on below 50 cm/s, off above 150 cm/s.
	const bool bBrakeInputActive = (Out.BrakeInput > 0.0f);
	if (In.bWaitingAtJunction)
	{
		const float AbsSpeed = FMath::Abs(In.CurrentSpeed);
		if (bPrevBraking)
		{
			Out.bBraking = bBrakeInputActive || AbsSpeed < 150.0f;
		}
		else
		{
			Out.bBraking = bBrakeInputActive || AbsSpeed < 50.0f;
		}
	}
	else
	{
		Out.bBraking = bBrakeInputActive;
	}
	bPrevBraking = Out.bBraking;

	// AEB hard override.
	if (Out.bAEBActive)
	{
		Out.ThrottleInput = 0.0f;
		Out.BrakeInput = 1.0f;
		Out.bBraking = true;
	}

	return Out;
}
