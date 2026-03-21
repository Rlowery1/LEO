// Copyright AAA_Traffic Contributors. All Rights Reserved.

#include "TrafficVehicleController.h"
#include "TrafficSubsystem.h"
#include "TrafficSignalController.h"
#include "TrafficLog.h"
#include "ChaosWheeledVehicleMovementComponent.h"
#include "ChaosVehicleWheel.h"                           // UChaosVehicleWheel CDO for brake torque / wheel radius
#include "GameFramework/Pawn.h"
#include "Engine/World.h"
#include "DrawDebugHelpers.h"
#include "PhysicsEngine/BodyInstance.h"
#include "Components/PrimitiveComponent.h"              // Full definition (forward-declared only in Game target)
// Needed for FPhysicsActorHandle->GetGameThreadAPI().SetSleepType(Chaos::ESleepType::NeverSleep).
// This header is engine-internal; Chaos is a private dependency to limit downstream coupling.
#include "PhysicsProxy/SingleParticlePhysicsProxy.h"
#include "HAL/IConsoleManager.h"
#include "UObject/UnrealType.h"                        // FBoolProperty for BP reflection

int32 GTrafficVehicleDecisionTrace = 0;
static FAutoConsoleVariableRef CVarTrafficVehicleDecisionTrace(
	TEXT("traffic.VehicleDecisionTrace"),
	GTrafficVehicleDecisionTrace,
	TEXT("Vehicle decision tracing: 0=off, 1=trace lane transitions/lane changes and flush on failures, 2=verbose per-candidate traces."),
	ECVF_Default);

int32 GTrafficVehicleDecisionTraceMax = 256;
static FAutoConsoleVariableRef CVarTrafficVehicleDecisionTraceMax(
	TEXT("traffic.VehicleDecisionTraceMax"),
	GTrafficVehicleDecisionTraceMax,
	TEXT("Max decision-trace entries held per vehicle before oldest records are evicted."),
	ECVF_Default);

bool GTrafficVehicleTraceFlushOnSuccess = false;
static FAutoConsoleVariableRef CVarTrafficVehicleTraceFlushOnSuccess(
	TEXT("traffic.VehicleTraceFlushOnSuccess"),
	GTrafficVehicleTraceFlushOnSuccess,
	TEXT("Flush lane decision trace buffer on successful transitions/lane changes (default false)."),
	ECVF_Default);

float GTrafficWakeGuardMinThrottle = 0.20f;
static FAutoConsoleVariableRef CVarTrafficWakeGuardMinThrottle(
	TEXT("traffic.WakeGuardMinThrottle"),
	GTrafficWakeGuardMinThrottle,
	TEXT("Minimum throttle request to trigger anti-sleep wake guard."),
	ECVF_Default);

float GTrafficWakeGuardMaxSpeed = 120.0f;
static FAutoConsoleVariableRef CVarTrafficWakeGuardMaxSpeed(
	TEXT("traffic.WakeGuardMaxSpeed"),
	GTrafficWakeGuardMaxSpeed,
	TEXT("Maximum absolute speed (cm/s) where anti-sleep wake guard is allowed to trigger."),
	ECVF_Default);

int32 GTrafficVehicleDiagnostics = 0;
static FAutoConsoleVariableRef CVarTrafficVehicleDiagnostics(
	TEXT("traffic.VehicleDiagnostics"),
	GTrafficVehicleDiagnostics,
	TEXT("Enable deep vehicle diagnostics logging: 0=off (default), 1=on. Keep off in production to avoid log spam and per-tick overhead."),
	ECVF_Default);

int32 GTrafficDebugDraw = 0;
static FAutoConsoleVariableRef CVarTrafficDebugDraw(
	TEXT("traffic.DebugDraw"),
	GTrafficDebugDraw,
	TEXT("Global toggle for in-world debug visualization: 0=off (default), 1=on for ALL vehicles. Overrides per-vehicle bDebugDraw."),
	ECVF_Default);

int32 GTrafficJunctionDiagnostics = 0;
static FAutoConsoleVariableRef CVarTrafficJunctionDiagnostics(
	TEXT("traffic.JunctionDiagnostics"),
	GTrafficJunctionDiagnostics,
	TEXT("Junction diagnostic flood logging: 0=off, 1=per-tick approach scan + detection gate, 2=verbose every-branch trace. Use 1 to diagnose why vehicles ignore intersections."),
	ECVF_Default);

// IsVehicleTraceEnabled moved to TrafficVehicleController_Diagnostics.cpp
// â”€â”€ FVehicleJunctionState phase transitions â”€â”€â”€â”€â”€â”€â”€â”€â”€â”€â”€â”€â”€â”€â”€â”€

void FVehicleJunctionState::BeginApproach(int32 InJunctionId)
{
	checkf(Phase == EJunctionPhase::Idle,
		TEXT("BeginApproach: expected Idle, got %d"), (int32)Phase);
	Phase = EJunctionPhase::Approaching;
	JunctionId = InJunctionId;
	LastReleasedId = 0; // New engagement â€” clear stale release marker.
}

void FVehicleJunctionState::BeginWaiting()
{
	checkf(Phase == EJunctionPhase::Idle || Phase == EJunctionPhase::Approaching,
		TEXT("BeginWaiting: expected Idle|Approaching, got %d"), (int32)Phase);
	Phase = EJunctionPhase::Waiting;
	bWaiting = true;
}

void FVehicleJunctionState::BeginTraversing()
{
	checkf(Phase == EJunctionPhase::Approaching || Phase == EJunctionPhase::Waiting,
		TEXT("BeginTraversing: expected Approaching|Waiting, got %d"), (int32)Phase);
	Phase = EJunctionPhase::Traversing;
	bWaiting = false;
	RetryTimer = 0.0f;
	WaitElapsed = 0.0f;
	StopSignStopElapsed = 0.0f;
	bStopSignWaitComplete = false;
}

void FVehicleJunctionState::Release()
{
	checkf(Phase >= EJunctionPhase::Approaching && Phase <= EJunctionPhase::Traversing,
		TEXT("Release: expected Approaching|Waiting|Traversing, got %d"), (int32)Phase);
	LastReleasedId = JunctionId;
	Phase = EJunctionPhase::Idle;
	JunctionId = 0;
	JunctionLane = FTrafficLaneHandle();
	FromLane = FTrafficLaneHandle();
	ToLane = FTrafficLaneHandle();
	EntryWorldPos = FVector::ZeroVector;
	bHasEntryPos = false;
	bWaiting = false;
	RetryTimer = 0.0f;
	WaitElapsed = 0.0f;
	ApproachDistanceCm = 0.0f;
	ApproachSpeedLimitCmPerSec = 0.0f;
	ApproachJunctionLane = FTrafficLaneHandle();
	TransitionPoints.Empty();
	TransitionIndex = 0;
	CurveStartIndex = 0;
	StopSignStopElapsed = 0.0f;
	bStopSignWaitComplete = false;
}

void FVehicleJunctionState::Reset()
{
	Phase = EJunctionPhase::Idle;
	JunctionId = 0;
	JunctionLane = FTrafficLaneHandle();
	FromLane = FTrafficLaneHandle();
	ToLane = FTrafficLaneHandle();
	EntryWorldPos = FVector::ZeroVector;
	bHasEntryPos = false;
	bWaiting = false;
	RetryTimer = 0.0f;
	WaitElapsed = 0.0f;
	LastReleasedId = 0;
	ApproachDistanceCm = 0.0f;
	ApproachSpeedLimitCmPerSec = 0.0f;
	ApproachJunctionLane = FTrafficLaneHandle();
	TransitionPoints.Empty();
	TransitionIndex = 0;
	CurveStartIndex = 0;
	StopSignStopElapsed = 0.0f;
	bStopSignWaitComplete = false;
}

ATrafficVehicleController::ATrafficVehicleController()
	: LaneWidth(0.f)
	, bLaneDataReady(false)
	, CachedProvider(nullptr)
	, bAtDeadEnd(false)
	, DistanceTraveledOnLane(0.0f)
	, WaitLogThrottleCounter(0)
	, LODAccumulatedDeltaTime(0.0f)
	, DiagJunctionTimer(0.0f)
	, PreviousVehicleLocation(FVector::ZeroVector)
	, DistanceThisTick(0.0f)
	, LastClosestIndex(0)
	, LODFrameCounter(0)
	, BaseTargetSpeed(0.0f)
	, TargetSpeed(1500.f)
	, LookAheadTimeSec(0.6f)
	, MinLookAheadDistanceCm(300.0f)
	, SteeringDampingFactor(0.5f)
	, CTECorrectionGain(0.3f)
	, LateralAccelBudgetCmPerSec2(294.0f)
	, CurveSpeedSafetyFactor(0.85f)
	, CurveScanWindowSize(5)
	, FollowingTimeSec(1.5f)
	, MinFollowingDistanceCm(200.0f)
	, DetectionTimeSec(4.0f)
	, MinDetectionDistanceCm(1500.0f)
	, IDMMaxAccelCmPerSec2(150.0f)
	, IDMComfortDecelCmPerSec2(300.0f)
	, IDMReactionDelaySec(0.3f)
	, AEBTimeToCollisionThresholdSec(1.5f)
	, IDMPersonalitySpread(0.15f)
	, IDMSmoothingTauSec(0.15f)
	, LaneChangeDistance(1500.f)
	, LaneChangeCooldownTime(5.0f)
	, LaneChangeSpeedThreshold(0.6f)
	, LaneChangeGapRequired(800.f)
	, DefaultSpeedLimit(0.0f)
	, JunctionScanMaxDistanceCm(50000.0f)
	, MaxJunctionScanHops(10)
	, ApproachSafetyMarginCm(500.0f)
	, ApproachDecelCmPerSec2(300.0f)
	, JunctionCurveResolutionCm(200.0f)
	, IntersectionSpeedLimitCmPerSec(2000.0f)
	, MaxIntersectionWaitTimeSec(90.0f)
	, VehicleFrontExtent(0.0f)
	, MaxAllowedSpeedCmPerSec(8000.0f) // ~180 mph, well above any traffic speed
	, RandomSeed(0)
{
	PrimaryActorTick.bCanEverTick = true;
	LaneChangeCoord_.Owner = this;
	TransitionEngine_.Owner = this;
	JunctionNeg_.Owner = this;
}

void ATrafficVehicleController::SetTargetSpeed(float InSpeed)
{
	TargetSpeed = FMath::Max(0.0f, InSpeed);
}

void ATrafficVehicleController::SetRandomSeed(int32 InSeed)
{
	RandomSeed = InSeed;
	RandomStream.Initialize(InSeed);

	// Per-vehicle personality: vary IDM parameters deterministically.
	// Each multiplier is in [1 - spread, 1 + spread].
	if (IDMPersonalitySpread > KINDA_SMALL_NUMBER)
	{
		const float S = IDMPersonalitySpread;
		AccelModel.SetPersonality(
			1.0f + RandomStream.FRandRange(-S, S),
			1.0f + RandomStream.FRandRange(-S, S),
			1.0f + RandomStream.FRandRange(-S, S));
	}
	else
	{
		AccelModel.SetPersonality(1.0f, 1.0f, 1.0f);
	}
}

void ATrafficVehicleController::SetLaneChangeAggression(float Aggression)
{
	LaneChangeCoord_.SetAggression(Aggression);

	// Sync UPROPERTY mirrors so Blueprint readback stays consistent.
	LaneChangeDistance       = LaneChangeCoord_.Distance;
	LaneChangeCooldownTime   = LaneChangeCoord_.CooldownTime;
	LaneChangeSpeedThreshold = LaneChangeCoord_.SpeedThreshold;
	LaneChangeGapRequired    = LaneChangeCoord_.GapRequired;
}

void ATrafficVehicleController::SetDefaultSpeedLimit(float InSpeedLimit)
{
	DefaultSpeedLimit = FMath::Max(0.0f, InSpeedLimit);
}

// ---------------------------------------------------------------------------
// Turn signal helpers
// ---------------------------------------------------------------------------

void ATrafficVehicleController::SetTurnSignal(ETurnSignalState NewState)
{
	// Cancel any pending delayed-off when a new active signal is set.
	if (NewState != ETurnSignalState::Off)
	{
		TurnSignalOffDelayRemaining = 0.0f;
	}
	if (CurrentTurnSignal == NewState) { return; }
	CurrentTurnSignal = NewState;
	OnTurnSignalChanged.Broadcast(NewState);
}

void ATrafficVehicleController::SetBrakeLights(bool bNewState)
{
	if (bBrakeLightsOn == bNewState) { return; }
	bBrakeLightsOn = bNewState;
	OnBrakeLightChanged.Broadcast(bNewState);
}

ETurnSignalState ATrafficVehicleController::ComputeTurnDirection(
	const FTrafficLaneHandle& FromLane,
	const FTrafficLaneHandle& ToLane) const
{
	if (!CachedProvider || !FromLane.IsValid() || !ToLane.IsValid())
	{
		return ETurnSignalState::Off;
	}

	// Use end-of-lane tangent for approach (where the junction is) and
	// start-of-lane tangent for exit (where the lane begins).
	const float FromLength = CachedProvider->GetLaneLength(FromLane);
	const FVector ApproachDir = CachedProvider->GetLaneDirectionAtDistance(FromLane, FromLength);
	const FVector ExitDir = CachedProvider->GetLaneDirection(ToLane);

	// Cross product Z: positive = left turn, negative = right turn.
	const float CrossZ = FVector::CrossProduct(ApproachDir, ExitDir).Z;

	// Threshold: ~15Â° deviation from straight â€” below this, treat as straight-through.
	constexpr float StraightThreshold = 0.26f; // sin(15Â°) â‰ˆ 0.259
	if (FMath::Abs(CrossZ) < StraightThreshold)
	{
		return ETurnSignalState::Off;
	}

	return (CrossZ > 0.0f) ? ETurnSignalState::Left : ETurnSignalState::Right;
}


FTrafficLaneHandle ATrafficVehicleController::PickSurveyedExit(
	const FTrafficLaneHandle& ApproachLane,
	const TArray<FTrafficLaneHandle>& FallbackExits)
{
	// Query the centralized surveyor table.
	UWorld* World = GetWorld();
	const UTrafficSubsystem* TrafficSub = World ? World->GetSubsystem<UTrafficSubsystem>() : nullptr;
	const TArray<FJunctionExitRule>& Rules = TrafficSub
		? TrafficSub->GetLegalExits(ApproachLane.HandleId)
		: UTrafficSubsystem::EmptyExitRules;

	// Build a list of exits with positive weight from the survey.
	TArray<FJunctionExitRule> ValidRules;
	for (const FJunctionExitRule& R : Rules)
	{
		if (R.Weight > 0.0f)
		{
			ValidRules.Add(R);
		}
	}

	// Fallback: if the surveyor has no data for this approach lane, use raw exits
	// with the same direction-weighted formula (e.g. lanes without a junction scan).
	if (ValidRules.IsEmpty() && FallbackExits.Num() > 0)
	{
		if (FallbackExits.Num() == 1)
		{
			return FallbackExits[0];
		}

		// Ad-hoc weighted selection for unsurveyed lanes.
		TArray<FTrafficLaneHandle> Sorted = FallbackExits;
		Sorted.Sort([](const FTrafficLaneHandle& A, const FTrafficLaneHandle& B)
		{ return A.HandleId < B.HandleId; });

		const float CurLen = CachedProvider ? CachedProvider->GetLaneLength(ApproachLane) : 0.0f;
		const FVector CurDir = CachedProvider
			? CachedProvider->GetLaneDirectionAtDistance(ApproachLane, CurLen) : FVector::ForwardVector;
		TArray<float> Weights;
		float TotalW = 0.0f;
		for (const FTrafficLaneHandle& E : Sorted)
		{
			const FVector ExitDir = CachedProvider ? CachedProvider->GetLaneDirection(E) : FVector::ForwardVector;
			const float Dot = FVector::DotProduct(CurDir, ExitDir);
			const float W = FMath::Max(FMath::Sqrt(FMath::Max(Dot + 1.0f, 0.0f)), 0.01f);
			Weights.Add(W);
			TotalW += W;
		}
		const float Roll = RandomStream.FRandRange(0.0f, TotalW);
		float Cum = 0.0f;
		for (int32 Idx = 0; Idx < Sorted.Num(); ++Idx)
		{
			Cum += Weights[Idx];
			if (Roll <= Cum) { return Sorted[Idx]; }
		}
		return Sorted.Last();
	}

	if (ValidRules.IsEmpty())
	{
		return FTrafficLaneHandle();
	}

	if (ValidRules.Num() == 1)
	{
		return ValidRules[0].ExitLane;
	}

	// Deterministic weighted CDF selection using seeded random stream.
	ValidRules.Sort([](const FJunctionExitRule& A, const FJunctionExitRule& B)
	{ return A.ExitLane.HandleId < B.ExitLane.HandleId; });

	float TotalWeight = 0.0f;
	for (const FJunctionExitRule& R : ValidRules)
	{
		TotalWeight += R.Weight;
	}

	const float Roll = RandomStream.FRandRange(0.0f, TotalWeight);
	float Cumulative = 0.0f;
	for (const FJunctionExitRule& R : ValidRules)
	{
		Cumulative += R.Weight;
		if (Roll <= Cumulative)
		{
			UE_LOG(LogAAATraffic, Log,
				TEXT("SURVEY-PICK: Pawn='%s' Approach=%d Exit=%d Turn=%s W=%.3f (roll=%.3f/%.3f)"),
				GetPawn() ? *GetPawn()->GetName() : TEXT("NULL"),
				ApproachLane.HandleId, R.ExitLane.HandleId,
				R.TurnDirection == ETurnSignalState::Left ? TEXT("L") : (R.TurnDirection == ETurnSignalState::Right ? TEXT("R") : TEXT("S")),
				R.Weight, Roll, TotalWeight);
			return R.ExitLane;
		}
	}
	return ValidRules.Last().ExitLane;
}


void ATrafficVehicleController::InitializeLaneFollowing(const FTrafficLaneHandle& InLane)
{
	TransitionEngine_.InitializeLane(InLane);
}


void ATrafficVehicleController::OnUnPossess()
{
	if (UWorld* World = GetWorld())
	{
		if (UTrafficSubsystem* TrafficSub = World->GetSubsystem<UTrafficSubsystem>())
		{
			// Release any junction occupancy before unregistering.
			if (JnctState.IsEngaged())
			{
				UE_LOG(LogAAATraffic, Warning,
					TEXT("JNCT RELEASE-UNPOSSESS: Pawn='%s' releasing junction %d (phase=%d) on unpossess/destroy"),
					GetPawn() ? *GetPawn()->GetName() : TEXT("NULL"),
					JnctState.JunctionId, (int32)JnctState.Phase);
				TrafficSub->ReleaseJunction(JnctState.JunctionId, this);
				JnctState.Reset();
			}
			else
			{
				UE_LOG(LogAAATraffic, Warning,
					TEXT("JNCT RELEASE-UNPOSSESS: Pawn='%s' unpossess â€” Phase=%d, nothing to release"),
					GetPawn() ? *GetPawn()->GetName() : TEXT("NULL"),
					(int32)JnctState.Phase);
			}
			TrafficSub->UnregisterVehicle(this);
		}
	}
	Super::OnUnPossess();
}


void ATrafficVehicleController::HandleActorHit(
	AActor* SelfActor, AActor* OtherActor, FVector NormalImpulse, const FHitResult& Hit)
{
	// Ignore trivial or self-hits.
	if (!OtherActor || OtherActor == SelfActor) return;

	// Only react to meaningful impacts (impulse > 5 kN).
	if (NormalImpulse.SizeSquared() < 5000.0f * 5000.0f) return;

	// Set the collision brake timer (seconds of full braking).
	CollisionBrakeTimer = FMath::Max(CollisionBrakeTimer, 1.0f);

	UE_LOG(LogAAATraffic, Warning,
		TEXT("COLLISION: Pawn='%s' hit '%s' impulse=%.0f â€” braking for 1s"),
		SelfActor ? *SelfActor->GetName() : TEXT("NULL"),
		*OtherActor->GetName(),
		NormalImpulse.Size());
}


void ATrafficVehicleController::Tick(float DeltaSeconds)
{
	Super::Tick(DeltaSeconds);

	// --- Turn signal delayed-off timer ---
	if (TurnSignalOffDelayRemaining > 0.0f)
	{
		TurnSignalOffDelayRemaining -= DeltaSeconds;
		if (TurnSignalOffDelayRemaining <= 0.0f)
		{
			TurnSignalOffDelayRemaining = 0.0f;
			SetTurnSignal(ETurnSignalState::Off);
		}
	}

	// --- Delayed movement diagnostic: after 2 seconds, check if vehicle moved ---

	// --- Vehicle diagnostics (delegated to separate translation unit) ---
	TickDiagnostics(DeltaSeconds);

	if (!bLaneDataReady || !GetPawn())
	{
		// Log once per controller to help diagnose stuck vehicles.
		if (!bDiagLoggedTickSkip)
		{
			bDiagLoggedTickSkip = true;
			UE_LOG(LogAAATraffic, Warning,
				TEXT("VehicleController::Tick: Skipping â€” bLaneDataReady=%s, Pawn=%s. "
					 "Lane will not be followed until both are valid."),
				bLaneDataReady ? TEXT("true") : TEXT("false"),
				GetPawn() ? *GetPawn()->GetName() : TEXT("NULL"));
		}
		return;
	}

	// --- Safety net: skip driving if despawn is pending ---
	if (bPendingRecoveryDespawn)
	{
		return;
	}

	++LODFrameCounter;
	if (LODFrameCounter >= 20) { LODFrameCounter = 0; } // Wrap at LCM(4,10) to prevent overflow.

	// --- LOD-based tick gating (Feature 7) ---
	// Reduced: tick every 4 frames. Minimal: tick every 10 frames.
	//
	// FIX (was MAJOR): Previously skipped frames entirely with bare `return`,
	// so DeltaTime was lost â€” vehicle behavior became frame-rate dependent.
	// Now we accumulate DeltaTime across skipped frames and pass the total
	// to UpdateVehicleInput on the tick that fires, making the update
	// frame-rate independent.
	UTrafficSubsystem* TrafficSub = nullptr;
	ETrafficLOD CurrentLOD = ETrafficLOD::Full;
	if (UWorld* World = GetWorld())
	{
		TrafficSub = World->GetSubsystem<UTrafficSubsystem>();
		if (TrafficSub)
		{
			CurrentLOD = TrafficSub->GetVehicleLOD(this);
			TrafficSub->UpdateVehiclePosition(this, GetPawn()->GetActorLocation());
		}
	}

	if (CurrentLOD == ETrafficLOD::Reduced && (LODFrameCounter % 4) != 0)
	{
		LODAccumulatedDeltaTime += DeltaSeconds;
		return;
	}
	if (CurrentLOD == ETrafficLOD::Minimal && (LODFrameCounter % 10) != 0)
	{
		// Minimal LOD: teleport along polyline using accumulated time and
		// speed to advance the correct number of points (frame-rate safe).
		LODAccumulatedDeltaTime += DeltaSeconds;
		if (LanePoints.Num() >= 2)
		{
			const float TeleportDist = TargetSpeed * LODAccumulatedDeltaTime;
			float DistRemaining = TeleportDist;
			int32 Idx = LastClosestIndex;
			while (Idx < LanePoints.Num() - 1 && DistRemaining > 0.0f)
			{
				const float SegLen = FVector::Dist(LanePoints[Idx], LanePoints[Idx + 1]);
				if (SegLen <= DistRemaining)
				{
					DistRemaining -= SegLen;
					++Idx;
				}
				else
				{
					break;
				}
			}
			if (Idx != LastClosestIndex)
			{
				// Zero velocity before teleport so the body doesn't carry
				// residual momentum into the new location (prevents launch).
				if (UChaosWheeledVehicleMovementComponent* MC =
					GetPawn()->FindComponentByClass<UChaosWheeledVehicleMovementComponent>())
				{
					if (UPrimitiveComponent* Prim = MC->UpdatedPrimitive)
					{
						if (FBodyInstance* BI = Prim->GetBodyInstance())
						{
							BI->SetLinearVelocity(FVector::ZeroVector, false);
							BI->SetAngularVelocityInRadians(FVector::ZeroVector, false);
						}
					}
				}
				GetPawn()->SetActorLocation(LanePoints[Idx], /*bSweep*/ false, /*OutSweepHitResult*/ nullptr, ETeleportType::TeleportPhysics);
				LastClosestIndex = Idx;
			}
		}
		LODAccumulatedDeltaTime = 0.0f; // consumed by teleport
		return;
	}

	// Active tick: pass accumulated time from skipped frames + this frame's delta.
	const float EffectiveDeltaSeconds = DeltaSeconds + LODAccumulatedDeltaTime;
	LODAccumulatedDeltaTime = 0.0f;

	// â”€â”€ Physics Safety Net â”€â”€â”€â”€â”€â”€â”€â”€â”€â”€â”€â”€â”€â”€â”€â”€â”€â”€â”€â”€â”€â”€â”€â”€â”€â”€â”€â”€â”€â”€â”€â”€â”€â”€
	// Detect vehicles that are flipped, airborne, or stuck and despawn them
	// to prevent physics explosions, traffic jams, and visual artifacts.
	{
		APawn* SafetyPawn = GetPawn();

		// --- Flip / Airborne detection ---
		// If the vehicle's up vector Z drops below 0.3, it's significantly
		// tilted or upside-down. Accumulate time and despawn after the
		// threshold to filter brief bumps from terrain.
		const float UpZ = SafetyPawn->GetActorUpVector().Z;
		if (UpZ < 0.3f)
		{
			FlipTimeAccumulator += EffectiveDeltaSeconds;
			if (FlipTimeAccumulator >= FlipDespawnTimeSec)
			{
				if (TrafficSub)
				{
					bPendingRecoveryDespawn = true;
					TrafficSub->RequestDespawn(this,
						FString::Printf(TEXT("flipped/airborne for %.1fs (UpZ=%.2f)"),
							FlipTimeAccumulator, UpZ));
				}
				else
				{
					UE_LOG(LogAAATraffic, Warning,
						TEXT("Safety: flipped vehicle has no subsystem for despawn â€” resetting accumulator."));
					FlipTimeAccumulator = 0.0f;
				}
				return;
			}
		}
		else
		{
			FlipTimeAccumulator = 0.0f;
		}

		// --- Stuck vehicle detection ---
		// If throttle is being requested (TargetSpeed > 0) but the vehicle
		// has barely moved for many consecutive frames, it's stuck in
		// geometry or a collision deadlock. Despawn to clear the blockage.
		const FVector CurrentLoc = SafetyPawn->GetActorLocation();
		const float MovedDist = FVector::Dist(CurrentLoc, PreviousVehicleLocation);
		// Only count stuck frames when the vehicle SHOULD be moving.
		const bool bShouldBeMoving = (TargetSpeed > 10.0f) && !JnctState.bWaiting && !bAtDeadEnd;
		if (bShouldBeMoving && MovedDist < 5.0f)
		{
			StuckTimeAccumulator += EffectiveDeltaSeconds;
			if (StuckTimeAccumulator >= StuckDespawnTimeSec)
			{
				if (TrafficSub)
				{
					bPendingRecoveryDespawn = true;
					TrafficSub->RequestDespawn(this,
						FString::Printf(TEXT("stuck for %.1fs (moved %.1f cm this tick)"),
							StuckTimeAccumulator, MovedDist));
				}
				else
				{
					UE_LOG(LogAAATraffic, Warning,
						TEXT("Safety: stuck vehicle has no subsystem for despawn â€” resetting accumulator."));
					StuckTimeAccumulator = 0.0f;
				}
				return;
			}
		}
		else
		{
			StuckTimeAccumulator = 0.0f;
		}
	}

	UpdateVehicleInput(EffectiveDeltaSeconds);

	// --- Debug draw (delegated to separate translation unit) ---
	DrawVehicleDebug();
}

int32 ATrafficVehicleController::FindClosestPointIndex(const FVector& VehicleLocation)
{
	if (LanePoints.Num() == 0) { return 0; }

	// Forward-only monotonic search: the index can only advance.
	// This prevents snapping to the wrong side of a hairpin curve
	// where the outgoing and incoming legs are close in 3D space.
	// Allow a small backward look (-2) to handle minor overshoot on
	// curve exits, but overwhelmingly search forward (+12).
	constexpr int32 BackwardRadius = 2;
	constexpr int32 ForwardRadius = 12;
	LastClosestIndex = FMath::Clamp(LastClosestIndex, 0, LanePoints.Num() - 1);
	const int32 StartIdx = FMath::Max(0, LastClosestIndex - BackwardRadius);
	const int32 EndIdx = FMath::Min(LanePoints.Num() - 1, LastClosestIndex + ForwardRadius);

	int32 BestIndex = LastClosestIndex;
	float BestDistSq = FVector::DistSquared(VehicleLocation, LanePoints[LastClosestIndex]);

	for (int32 i = StartIdx; i <= EndIdx; ++i)
	{
		const float DistSq = FVector::DistSquared(VehicleLocation, LanePoints[i]);
		if (DistSq < BestDistSq)
		{
			BestDistSq = DistSq;
			BestIndex = i;
		}
	}

	// Fall back to a forward-only full scan if the bounded search result
	// is far from the vehicle (teleport, respawn, or initial placement).
	// Search only from the current index forward to maintain monotonicity
	// except on the very first call (LastClosestIndex == 0) where we scan all.
	constexpr float FullScanThresholdSq = 500.0f * 500.0f; // 5m
	if (BestDistSq > FullScanThresholdSq)
	{
		const int32 ScanStart = (LastClosestIndex == 0) ? 0 : LastClosestIndex;
		for (int32 i = ScanStart; i < LanePoints.Num(); ++i)
		{
			const float DistSq = FVector::DistSquared(VehicleLocation, LanePoints[i]);
			if (DistSq < BestDistSq)
			{
				BestDistSq = DistSq;
				BestIndex = i;
			}
		}
	}

	LastClosestIndex = BestIndex;
	return BestIndex;
}


FVector ATrafficVehicleController::GetLookAheadPoint(
	const FVector& VehicleLocation, int32 ClosestIndex) const
{
	// Walk forward along lane points until LookAheadDistance is reached.
	float AccumulatedDist = 0.0f;
	int32 Index = ClosestIndex;

	while (Index < LanePoints.Num() - 1)
	{
		// Use 2D distance so look-ahead accumulation matches the 2D
		// steering computation. On hills, 3D distance is longer than
		// 2D, causing the look-ahead point to be closer than intended.
		const float SegmentDist = FVector::Dist2D(LanePoints[Index], LanePoints[Index + 1]);
		AccumulatedDist += SegmentDist;

		if (AccumulatedDist >= LookAheadDistance)
		{
			const float Overshoot = AccumulatedDist - LookAheadDistance;
			const float Alpha = 1.0f - (Overshoot / FMath::Max(SegmentDist, KINDA_SMALL_NUMBER));
			return FMath::Lerp(LanePoints[Index], LanePoints[Index + 1], Alpha);
		}

		++Index;
	}

	// If the look-ahead extends beyond the lane, extrapolate along the
	// final segment direction. Returning LanePoints.Last() would make
	// Ldâ†’0 as the vehicle approaches, causing a steering spike.
	if (LanePoints.Num() >= 2)
	{
		const int32 Last = LanePoints.Num() - 1;
		const FVector FinalDir = (LanePoints[Last] - LanePoints[Last - 1]).GetSafeNormal();
		const float Remaining = LookAheadDistance - AccumulatedDist;
		return LanePoints[Last] + FinalDir * Remaining;
	}
	return LanePoints.Last();
}


float ATrafficVehicleController::GetRemainingDistance(int32 FromIndex) const
{
	// Need at least two lane points to measure any distance.
	if (LanePoints.Num() < 2)
	{
		return 0.0f;
	}

	// Clamp to valid range so FromIndex + 1 is a valid index.
	if (FromIndex < 0)
	{
		FromIndex = 0;
	}
	if (FromIndex >= LanePoints.Num() - 1)
	{
		return 0.0f; // Already at or past the last point.
	}

	float Distance = 0.0f;

	// Use the vehicle's actual location for the first partial segment so
	// the measurement reflects how far the vehicle truly is from the next point.
	const APawn* ControlledPawn = GetPawn();
	const FVector StartPos = (ControlledPawn != nullptr)
		? ControlledPawn->GetActorLocation()
		: LanePoints[FromIndex];

	Distance += FVector::Dist(StartPos, LanePoints[FromIndex + 1]);

	// Sum remaining full segments.
	for (int32 i = FromIndex + 1; i < LanePoints.Num() - 1; ++i)
	{
		Distance += FVector::Dist(LanePoints[i], LanePoints[i + 1]);
	}
	return Distance;
}


float ATrafficVehicleController::ComputeLocalCurvature(
	const TArray<FVector>& Points, int32 CenterIndex) const
{
	return FSteeringComputer::ComputeLocalCurvature(Points, CenterIndex, CurveScanWindowSize);
}


void ATrafficVehicleController::CheckLaneTransition()
{
	TransitionEngine_.CheckTransition();
}

