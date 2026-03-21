// Copyright AAA_Traffic Contributors. All Rights Reserved.

#pragma once

#include "CoreMinimal.h"

/**
 * Input snapshot for the IDM acceleration model, assembled each tick
 * by the controller.
 */
struct FAccelerationInput
{
	/** Current vehicle speed (cm/s, absolute). */
	float CurrentSpeed = 0.0f;

	/** Desired free-road speed (cm/s) — the IDM v₀. */
	float DesiredSpeed = 0.0f;

	/** Raw leader distance from detector (cm, -1 = no leader). */
	float RawLeaderDist = -1.0f;

	/** Raw leader speed along ego forward axis (cm/s). */
	float RawLeaderSpeed = 0.0f;

	/** True if the leader vehicle's brake lights are visible,
	 *  signalling the reaction delay should be bypassed. */
	bool bLeaderBrakeLightsVisible = false;

	/** True when vehicle is waiting at a junction (for brake light logic). */
	bool bWaitingAtJunction = false;

	float DeltaSeconds = 0.0f;

	// ── IDM tuning parameters (base values) ─────────────────
	float MaxAccelCmPerSec2 = 150.0f;
	float ComfortDecelCmPerSec2 = 300.0f;
	float TimeHeadwaySec = 1.5f;
	float JamDistanceCm = 200.0f;
	float ReactionDelaySec = 0.3f;
	float SmoothingTauSec = 0.15f;
	float AEBThresholdSec = 1.5f;

	/** Maximum brake deceleration from physics (cm/s², for emergency normalization). */
	float MaxBrakeDecelCmPerSec2 = 600.0f;
};

/**
 * Output from the acceleration model.
 */
struct FAccelerationOutput
{
	float ThrottleInput = 0.0f;
	float BrakeInput = 0.0f;

	/** True when Automatic Emergency Braking overrode normal IDM. */
	bool bAEBActive = false;

	/** True when brake lights should be on (braking or junction-waiting at low speed). */
	bool bBraking = false;

	/** Raw IDM acceleration (cm/s², for diagnostics). */
	float IDMAccel = 0.0f;

	/** Delayed leader distance after reaction buffer (cm, for diagnostics). */
	float DelayedLeaderDist = -1.0f;

	/** Delayed leader speed after reaction buffer (cm/s, for diagnostics). */
	float DelayedLeaderSpeed = 0.0f;
};

/**
 * IDM+ acceleration model with reaction-delay buffer, AEB, and
 * per-vehicle personality scaling.
 *
 * Stateful: owns the leader delay ring buffer, smoothing EMA,
 * and personality multipliers.
 */
struct AAA_TRAFFIC_API FAccelerationModel
{
	/** Compute throttle/brake for one tick. */
	FAccelerationOutput Compute(const FAccelerationInput& In);

	/** Set per-vehicle personality multipliers (call once after spawn). */
	void SetPersonality(float AccelScale, float DecelScale, float HeadwayScale);

	/** Clear the delay buffer (call on lane transitions). */
	void ClearDelayBuffer();

	// ── Internal state (public for save/restore by controller) ──
	float PersonalityAccelScale = 1.0f;
	float PersonalityDecelScale = 1.0f;
	float PersonalityHeadwayScale = 1.0f;
	float SmoothedAccel = 0.0f;

private:
	struct FLeaderSample
	{
		float Time = 0.0f;
		float Dist = -1.0f;
		float Speed = 0.0f;
	};
	TArray<FLeaderSample> DelayBuffer;
	float TimeClock = 0.0f;

	/** Apply reaction delay buffer: push raw sample, read delayed sample. */
	void ProcessDelayBuffer(
		float DeltaSeconds,
		float ReactionDelay,
		float RawDist,
		float RawSpeed,
		float& OutDist,
		float& OutSpeed);
};
