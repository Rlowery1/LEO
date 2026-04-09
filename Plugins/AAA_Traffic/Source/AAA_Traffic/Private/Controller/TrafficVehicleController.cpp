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

float GTrafficCTERecoveryThreshold = 0.6f;
static FAutoConsoleVariableRef CVarTrafficCTERecoveryThreshold(
	TEXT("traffic.CTERecoveryThreshold"),
	GTrafficCTERecoveryThreshold,
	TEXT("CTE fraction (of half-lane) that triggers post-collision throttle reduction (default 0.6)."),
	ECVF_Default);

float GTrafficCTERecoveryScale = 1.25f;
static FAutoConsoleVariableRef CVarTrafficCTERecoveryScale(
	TEXT("traffic.CTERecoveryScale"),
	GTrafficCTERecoveryScale,
	TEXT("Throttle reduction per CTE unit above threshold (default 1.25). Higher = more aggressive."),
	ECVF_Default);

float GTrafficCTERecoveryMinThrottle = 0.4f;
static FAutoConsoleVariableRef CVarTrafficCTERecoveryMinThrottle(
	TEXT("traffic.CTERecoveryMinThrottle"),
	GTrafficCTERecoveryMinThrottle,
	TEXT("Minimum throttle floor during CTE recovery to preserve steering authority (default 0.4)."),
	ECVF_Default);

float GTrafficCTERecoveryBrakeFactor = 0.5f;
static FAutoConsoleVariableRef CVarTrafficCTERecoveryBrakeFactor(
	TEXT("traffic.CTERecoveryBrakeFactor"),
	GTrafficCTERecoveryBrakeFactor,
	TEXT("Supplemental brake multiplier during CTE recovery (default 0.5). brake = (1-recoveryScale)*factor."),
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

int32 GTrafficDebugDrawPaths = 0;
static FAutoConsoleVariableRef CVarTrafficDebugDrawPaths(
	TEXT("traffic.DebugDrawPaths"),
	GTrafficDebugDrawPaths,
	TEXT("Path-only debug visualization: 0=off, 1=show only the path each vehicle follows (lane polyline + junction curve), 2=also show all precomputed junction corridors."),
	ECVF_Default);

int32 GTrafficJunctionDiagnostics = 0;
static FAutoConsoleVariableRef CVarTrafficJunctionDiagnostics(
	TEXT("traffic.JunctionDiagnostics"),
	GTrafficJunctionDiagnostics,
	TEXT("Junction diagnostic flood logging: 0=off, 1=per-tick approach scan + detection gate, 2=verbose every-branch trace. Use 1 to diagnose why vehicles ignore intersections."),
	ECVF_Default);

// IsVehicleTraceEnabled moved to TrafficVehicleController_Diagnostics.cpp
// -- FVehicleJunctionState phase transitions ----------------

void FVehicleJunctionState::BeginApproach(int32 InJunctionId)
{
	checkf(Phase == EJunctionPhase::Idle,
		TEXT("BeginApproach: expected Idle, got %d"), (int32)Phase);
	Phase = EJunctionPhase::Approaching;
	JunctionId = InJunctionId;
	CanonicalMovementId = 0;
	bOwnsJunctionOccupancy = false;
	bTransitionPathFromProvider = false;
	// Only clear stale release marker when engaging a DIFFERENT junction.
	// If the vehicle just released this junction (e.g. it is still on a
	// junction lane), preserving LastReleasedId prevents TickDetectAndOccupy
	// from re-acquiring the same junction as a new encounter.
	if (InJunctionId != LastReleasedId)
	{
		LastReleasedId = 0;
	}
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
	TimeoutRetryCount = 0;
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
	CanonicalMovementId = 0;
	EntryWorldPos = FVector::ZeroVector;
	bHasEntryPos = false;
	bWaiting = false;
	bOwnsJunctionOccupancy = false;
	RetryTimer = 0.0f;
	WaitElapsed = 0.0f;
	TimeoutRetryCount = 0;
	ApproachDistanceCm = 0.0f;
	ApproachSpeedLimitCmPerSec = 0.0f;
	ApproachJunctionLane = FTrafficLaneHandle();
	TransitionPoints.Empty();
	TransitionIndex = 0;
	CurveStartIndex = 0;
	TransitionReleaseIndex = INDEX_NONE;
	ExitLaneResumeIndex = INDEX_NONE;
	bTransitionPathFromProvider = false;
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
	CanonicalMovementId = 0;
	EntryWorldPos = FVector::ZeroVector;
	bHasEntryPos = false;
	bWaiting = false;
	bOwnsJunctionOccupancy = false;
	RetryTimer = 0.0f;
	WaitElapsed = 0.0f;
	TimeoutRetryCount = 0;
	LastReleasedId = 0;
	ApproachDistanceCm = 0.0f;
	ApproachSpeedLimitCmPerSec = 0.0f;
	ApproachJunctionLane = FTrafficLaneHandle();
	TransitionPoints.Empty();
	TransitionIndex = 0;
	CurveStartIndex = 0;
	TransitionReleaseIndex = INDEX_NONE;
	ExitLaneResumeIndex = INDEX_NONE;
	bTransitionPathFromProvider = false;
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
	, CTECorrectionGain(2.0f)
	, LateralAccelBudgetCmPerSec2(294.0f)
	, CurveSpeedSafetyFactor(0.85f)
	, CurveScanWindowSize(5)
	, FollowingTimeSec(1.5f)
	, MinFollowingDistanceCm(200.0f)
	, DetectionTimeSec(4.0f)
	, MinDetectionDistanceCm(2500.0f)
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
	, OvertakeStuckTimeSec(2.0f)
	, OvertakeMinClearanceCm(3000.0f)
	, OvertakeAbortClearanceCm(3000.0f)
	, OvertakeSpeedBoostPct(15.0f)
	, OvertakeMinPassDistCm(2000.0f)
	, OvertakeBlendDistCm(1200.0f)
	, OvertakeCooldownTimeSec(20.0f)
	, DefaultSpeedLimit(0.0f)
	, JunctionScanMaxDistanceCm(50000.0f)
	, MaxJunctionScanHops(10)
	, ApproachSafetyMarginCm(500.0f)
	, ApproachDecelCmPerSec2(300.0f)
	, JunctionCurveResolutionCm(200.0f)
	, IntersectionSpeedLimitCmPerSec(2000.0f)
	, MaxIntersectionWaitTimeSec(30.0f)
	, VehicleFrontExtent(0.0f)
	, MaxAllowedSpeedCmPerSec(8000.0f) // ~180 mph, well above any traffic speed
	, RandomSeed(0)
{
	PrimaryActorTick.bCanEverTick = true;
	LaneChangeCoord_.Owner = this;
	TransitionEngine_.Owner = this;
	JunctionNeg_.Owner = this;

	// Sync overtaking config to coordinator.
	LaneChangeCoord_.OvertakeStuckTimeSec    = OvertakeStuckTimeSec;
	LaneChangeCoord_.OvertakeMinClearanceCm  = OvertakeMinClearanceCm;
	LaneChangeCoord_.OvertakeAbortClearanceCm = OvertakeAbortClearanceCm;
	LaneChangeCoord_.OvertakeSpeedBoostPct   = OvertakeSpeedBoostPct;
	LaneChangeCoord_.OvertakeMinPassDistCm   = OvertakeMinPassDistCm;
	LaneChangeCoord_.OvertakeBlendDistCm     = OvertakeBlendDistCm;
	LaneChangeCoord_.OvertakeCooldownTimeSec = OvertakeCooldownTimeSec;
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

ETurnSignalState ATrafficVehicleController::GetSelectedTurnDirection() const
{
	if (const FCanonicalMovementRecord* Movement = GetSelectedCanonicalMovement())
	{
		return Movement->TurnDirection;
	}

	return ETurnSignalState::Off;
}


bool ATrafficVehicleController::IsUsableExitLane(const FTrafficLaneHandle& Lane) const
{
	if (!Lane.IsValid()) { return false; }

	// Reject lanes on non-drivable road types (Walkway, Railway, etc.).
	if (CachedProvider)
	{
		const FTrafficRoadHandle Road = CachedProvider->GetRoadForLane(Lane);
		if (Road.IsValid())
		{
			const ETrafficRoadType RoadType = CachedProvider->GetRoadType(Road);
			if (RoadType != ETrafficRoadType::Normal)
			{
				return false;
			}
		}

		// Reject lanes whose primary type is non-drivable.
		const TArray<FTrafficLaneSection> Sections = CachedProvider->GetLaneSections(Lane);
		if (!Sections.IsEmpty())
		{
			const ETrafficLaneType PrimaryType = Sections[0].Type;
			if (PrimaryType != ETrafficLaneType::Normal
				&& PrimaryType != ETrafficLaneType::CenterTurn)
			{
				return false;
			}
		}
	}

	return true;
}


FTrafficLaneHandle ATrafficVehicleController::PickSurveyedExit(
	const FTrafficLaneHandle& ApproachLane,
	const TArray<FTrafficLaneHandle>& FallbackExits)
{
	// Query the centralized surveyor table.
	UWorld* World = GetWorld();
	const UTrafficSubsystem* TrafficSub = World ? World->GetSubsystem<UTrafficSubsystem>() : nullptr;
	const TArray<int32>& CanonicalMovementIds = TrafficSub
		? TrafficSub->GetCanonicalMovementsForApproachLane(ApproachLane.HandleId)
		: UTrafficSubsystem::EmptyMovementIds;
	const TArray<FJunctionExitRule>& Rules = TrafficSub
		? TrafficSub->GetLegalExits(ApproachLane.HandleId)
		: UTrafficSubsystem::EmptyExitRules;
	auto IsAllowedCandidate = [this, &FallbackExits](const FTrafficLaneHandle& CandidateLane)
	{
		if (!CandidateLane.IsValid())
		{
			return false;
		}

		if (!FallbackExits.IsEmpty())
		{
			bool bFoundInFallback = false;
			for (const FTrafficLaneHandle& FallbackLane : FallbackExits)
			{
				if (FallbackLane.HandleId == CandidateLane.HandleId)
				{
					bFoundInFallback = true;
					break;
				}
			}

			if (!bFoundInFallback)
			{
				return false;
			}
		}

		return IsUsableExitLane(CandidateLane);
	};

	if (!CanonicalMovementIds.IsEmpty() && TrafficSub)
	{
		if (const FCanonicalMovementRecord* ExistingMovement = GetSelectedCanonicalMovement())
		{
			if (ExistingMovement->FromLane.HandleId == ApproachLane.HandleId
				&& IsAllowedCandidate(ExistingMovement->ToLane))
			{
				JnctState.CanonicalMovementId = ExistingMovement->MovementId;
				return ExistingMovement->ToLane;
			}
		}

		TArray<const FCanonicalMovementRecord*> ValidMovements;
		for (const int32 MovementId : CanonicalMovementIds)
		{
			const FCanonicalMovementRecord* Movement = TrafficSub->GetCanonicalMovement(MovementId);
			if (!Movement
				|| !Movement->IsValid()
				|| Movement->SelectionWeight <= 0.0f
				|| !Movement->bLegallyAllowed
				|| !Movement->bPhysicallyFeasible
				|| !IsAllowedCandidate(Movement->ToLane))
			{
				continue;
			}
			ValidMovements.Add(Movement);
		}

		// Fallback: if all canonical movements were filtered (typically
		// because every curve is physically infeasible), admit the
		// infeasible ones so the vehicle is not orphaned at the junction.
		if (ValidMovements.IsEmpty())
		{
			const FCanonicalMovementRecord* BestInfeasible = nullptr;
			float BestRadius = -1.0f;
			for (const int32 MovementId : CanonicalMovementIds)
			{
				const FCanonicalMovementRecord* Movement = TrafficSub->GetCanonicalMovement(MovementId);
				if (!Movement || !Movement->IsValid() || !Movement->bLegallyAllowed
					|| !IsAllowedCandidate(Movement->ToLane))
				{
					continue;
				}
				if (Movement->MinTurnRadiusCm > BestRadius)
				{
					BestRadius = Movement->MinTurnRadiusCm;
					BestInfeasible = Movement;
				}
			}
			if (BestInfeasible)
			{
				UE_LOG(LogAAATraffic, Warning,
					TEXT("CANONICAL-PICK-INFEASIBLE-FALLBACK: Pawn='%s' Approach=%d — admitting Movement=%d Exit=%d (R=%.0f) as last resort"),
					GetPawn() ? *GetPawn()->GetName() : TEXT("NULL"),
					ApproachLane.HandleId,
					BestInfeasible->MovementId,
					BestInfeasible->ToLane.HandleId,
					BestRadius);
				ValidMovements.Add(BestInfeasible);
			}
		}

		if (!ValidMovements.IsEmpty())
		{
			ValidMovements.Sort([](const FCanonicalMovementRecord& A, const FCanonicalMovementRecord& B)
			{
				return A.MovementId < B.MovementId;
			});

			float TotalWeight = 0.0f;
			for (const FCanonicalMovementRecord* Movement : ValidMovements)
			{
				TotalWeight += Movement->SelectionWeight;
			}

			const float Roll = RandomStream.FRandRange(0.0f, TotalWeight);
			float Cumulative = 0.0f;
			for (const FCanonicalMovementRecord* Movement : ValidMovements)
			{
				Cumulative += Movement->SelectionWeight;
				if (Roll <= Cumulative)
				{
					JnctState.CanonicalMovementId = Movement->MovementId;
					UE_LOG(LogAAATraffic, Log,
						TEXT("CANONICAL-PICK: Pawn='%s' Movement=%d Approach=%d Exit=%d Turn=%s W=%.3f (roll=%.3f/%.3f)"),
						GetPawn() ? *GetPawn()->GetName() : TEXT("NULL"),
						Movement->MovementId,
						ApproachLane.HandleId,
						Movement->ToLane.HandleId,
						Movement->TurnDirection == ETurnSignalState::Left ? TEXT("L") : (Movement->TurnDirection == ETurnSignalState::Right ? TEXT("R") : TEXT("S")),
						Movement->SelectionWeight,
						Roll,
						TotalWeight);
					return Movement->ToLane;
				}
			}

			const FCanonicalMovementRecord* Movement = ValidMovements.Last();
			JnctState.CanonicalMovementId = Movement->MovementId;
			return Movement->ToLane;
		}

		UE_LOG(LogAAATraffic, Warning,
			TEXT("CANONICAL-PICK-EMPTY: Pawn='%s' Approach=%d has canonical movements but none survived runtime filtering — falling through to Rules"),
			GetPawn() ? *GetPawn()->GetName() : TEXT("NULL"),
			ApproachLane.HandleId);
		JnctState.CanonicalMovementId = 0;
		// Fall through to Rules-based selection below instead of
		// returning invalid — the survey Rules table may still have
		// a usable exit even when canonical movements are all culled.
	}

	JnctState.CanonicalMovementId = 0;

	// Build a list of exits with positive weight from the survey.
	TArray<FJunctionExitRule> ValidRules;
	for (const FJunctionExitRule& R : Rules)
	{
		if (R.Weight > 0.0f && IsAllowedCandidate(R.ExitLane))
		{
			ValidRules.Add(R);
		}
	}

	// Fallback: if the surveyor has no data for this approach lane, use raw exits
	// with the same direction-weighted formula (e.g. lanes without a junction scan).
	if (ValidRules.IsEmpty() && FallbackExits.Num() > 0)
	{
		TArray<FTrafficLaneHandle> Sorted;
		Sorted.Reserve(FallbackExits.Num());
		for (const FTrafficLaneHandle& ExitLane : FallbackExits)
		{
			if (IsAllowedCandidate(ExitLane))
			{
				Sorted.Add(ExitLane);
			}
		}

		if (Sorted.Num() == 1)
		{
			return Sorted[0];
		}

		if (Sorted.IsEmpty())
		{
			return FTrafficLaneHandle();
		}

		// Ad-hoc weighted selection for unsurveyed lanes.
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

const FCanonicalMovementRecord* ATrafficVehicleController::GetSelectedCanonicalMovement() const
{
	if (JnctState.CanonicalMovementId <= 0)
	{
		return nullptr;
	}

	const UWorld* World = GetWorld();
	const UTrafficSubsystem* TrafficSub = World ? World->GetSubsystem<UTrafficSubsystem>() : nullptr;
	if (!TrafficSub)
	{
		return nullptr;
	}

	return TrafficSub->GetCanonicalMovement(JnctState.CanonicalMovementId);
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
			if (JnctState.Phase == EJunctionPhase::Traversing)
			{
				UE_LOG(LogAAATraffic, Warning,
					TEXT("JNCT RELEASE-UNPOSSESS: Pawn='%s' preserving traversing junction %d (phase=%d) until unregister cleanup"),
					GetPawn() ? *GetPawn()->GetName() : TEXT("NULL"),
					JnctState.JunctionId, (int32)JnctState.Phase);
			}
			else if (JnctState.IsActive())
			{
				UE_LOG(LogAAATraffic, Warning,
					TEXT("JNCT RELEASE-UNPOSSESS: Pawn='%s' preserving phase=%d junction %d until unregister cleanup"),
					GetPawn() ? *GetPawn()->GetName() : TEXT("NULL"),
					(int32)JnctState.Phase,
					JnctState.JunctionId);
			}
			else
			{
				UE_LOG(LogAAATraffic, Warning,
					TEXT("JNCT RELEASE-UNPOSSESS: Pawn='%s' unpossess -- Phase=%d, nothing to release"),
					GetPawn() ? *GetPawn()->GetName() : TEXT("NULL"),
					(int32)JnctState.Phase);
			}
			TrafficSub->UnregisterVehicle(this);
			// Release any deferred junction occupancy before cleanup.
			if (bDeferredJunctionRelease)
			{
				UE_LOG(LogAAATraffic, Warning,
					TEXT("JNCT DEFERRED-RELEASE-UNPOSSESS: Pawn='%s' junction %d — releasing on despawn"),
					GetPawn() ? *GetPawn()->GetName() : TEXT("NULL"),
					DeferredJunctionReleaseId);
				TrafficSub->ReleaseJunction(DeferredJunctionReleaseId, this);
				bDeferredJunctionRelease = false;
				DeferredJunctionReleaseId = 0;
			}
			if (JnctState.IsActive())
			{
				JnctState.Reset();
			}
		}
	}
	Super::OnUnPossess();
}


void ATrafficVehicleController::HandleActorHit(
	AActor* SelfActor, AActor* OtherActor, FVector NormalImpulse, const FHitResult& Hit)
{
	// Ignore trivial or self-hits.
	if (!OtherActor || OtherActor == SelfActor) return;

	// Ignore physics debris (destructible pieces from vehicle damage).
	// These are not traffic-relevant obstacles.  Catch both the legacy
	// "DestructiblePiece" naming and the DD_Vehicles "DP_" prefix for
	// detached doors/panels.
	{
		const FString& OtherName = OtherActor->GetName();
		if (OtherName.Contains(TEXT("DestructiblePiece")) || OtherName.StartsWith(TEXT("DP_")))
		{
			return;
		}
	}

	// Only brake for vehicle-vehicle collisions (other Pawns).
	// Collisions with road geometry, guardrails, or static meshes should
	// not trigger emergency braking — the steering PID handles recovery.
	if (!OtherActor->IsA<APawn>()) return;

	// Ignore collisions during spawn grace window. Vehicles can spawn
	// overlapping other actors, generating impulses on the first frame
	// that would trap the vehicle at zero throttle via CollisionBrakeTimer.
	if (SpawnGraceTimer > 0.0f) return;

	// If already collision-braking, ignore new impacts.  Without this,
	// overlapping vehicles generate continuous impulses every physics
	// frame, resetting CollisionBrakeTimer to 1.0 via the Max() below
	// and preventing it from ever reaching zero — permanently trapping
	// both vehicles at full brake.
	if (CollisionBrakeTimer > 0.0f) return;

	// Only react to meaningful impacts (impulse > 5 kN).
	if (NormalImpulse.SizeSquared() < 5000.0f * 5000.0f) return;

	// Proportional brake: low-impulse queue bumps get minimal brake to
	// prevent bump→brake→resume→bump oscillation.  Real crashes still
	// receive the full 1 s emergency brake.
	const float Impulse = NormalImpulse.Size();
	const float BrakeTime = FMath::GetMappedRangeValueClamped(
		FVector2f(5000.0f, 50000.0f),
		FVector2f(0.1f, 1.0f),
		Impulse);
	CollisionBrakeTimer = BrakeTime;

	// Track whether this was a vehicle-vehicle collision (other actor is a Pawn).
	bCollisionWithVehicle = bCollisionWithVehicle || OtherActor->IsA<APawn>();

	UE_LOG(LogAAATraffic, Warning,
		TEXT("COLLISION: Pawn='%s' hit '%s' impulse=%.0f Lane=%d Jnct=%d Phase=%d Path=%s CurvePts=%d CurveIdx=%d CTE=%.1f/%.1f HeadCross=%.3f TargetDist=%.0f SpeedCap=%.0f Steer=%.3f JCurveR=%.0f JCurveCap=%.0f PredR=%.0f PredDist=%.0f PredCap=%.0f LeaderDist=%.0f BrakeTime=%.2f"),
		SelfActor ? *SelfActor->GetName() : TEXT("NULL"),
		*OtherActor->GetName(),
		NormalImpulse.Size(),
		CurrentLane.HandleId,
		JnctState.JunctionId,
		(int32)JnctState.Phase,
		JnctState.TransitionPoints.Num() > 0
			? (JnctState.bTransitionPathFromProvider ? TEXT("PROVIDER-JNCT") : TEXT("SYNTH-JNCT"))
			: TEXT("LANE"),
		JnctState.TransitionPoints.Num(),
		JnctState.TransitionIndex,
		LastDiagSignedCTE,
		LastDiagHalfLaneWidth,
		LastDiagHeadingCrossZ,
		LastDiagTargetDistance2D,
		LastDiagEffectiveTargetSpeed,
		LastDiagSteeringInput,
		LastDiagJunctionCurveRadiusCm,
		LastDiagJunctionCurveCapCmPerSec,
		LastDiagPredictiveCurveRadiusCm,
		LastDiagPredictiveCurveDistCm,
		LastDiagPredictiveCurveSpeedCmPerSec,
		DbgLeaderDist,
		BrakeTime);
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
				TEXT("VehicleController::Tick: Skipping -- bLaneDataReady=%s, Pawn=%s. "
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
	// so DeltaTime was lost -- vehicle behavior became frame-rate dependent.
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

	// ── Deferred junction release ──
	// When a vehicle exits a junction onto a lane not registered in the
	// junction system, occupancy release is deferred until the vehicle
	// clears the junction area (distance-based).
	if (bDeferredJunctionRelease && TrafficSub && GetPawn())
	{
		const float DistSq = FVector::DistSquared(GetPawn()->GetActorLocation(), DeferredJunctionReleaseOrigin);
		if (DistSq > DeferredJunctionReleaseDistCm * DeferredJunctionReleaseDistCm)
		{
			UE_LOG(LogAAATraffic, Warning,
				TEXT("JNCT DEFERRED-RELEASE-FIRE: Pawn='%s' junction %d — "
					 "traveled %.0f cm from exit, releasing"),
				*GetPawn()->GetName(), DeferredJunctionReleaseId,
				FMath::Sqrt(DistSq));
			TrafficSub->ReleaseJunction(DeferredJunctionReleaseId, this);
			bDeferredJunctionRelease = false;
			DeferredJunctionReleaseId = 0;
			DeferredJunctionReleaseOrigin = FVector::ZeroVector;
			DeferredJunctionReleaseDistCm = 1500.0f;
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

	// -- Physics Safety Net ----------------------------------
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
						TEXT("Safety: flipped vehicle has no subsystem for despawn -- resetting accumulator."));
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
		// Cars in any junction phase (approaching, waiting, traversing) are
		// legitimately stopped or slow — don't count them as stuck.
		// Also exempt cars that have a detected leader ahead (queued behind
		// another vehicle and correctly car-following at near-zero speed).
		const bool bInJunction = (JnctState.Phase != EJunctionPhase::Idle);
		const bool bHasLeader = (LastLeaderDist > 0.0f);
		// Exempt ALL junction phases. Approaching vehicles legitimately
		// brake to near-zero at the stop line before the detection gate
		// transitions them to Waiting. Without this exemption the 2-second
		// stuck timer fires during the Approaching→Waiting gap, triggering
		// false stuck recovery on every car that stops for a red light.
		const bool bShouldBeMoving = (TargetSpeed > 10.0f) && !bInJunction
			&& !bAtDeadEnd && !bHasLeader;
		if (bShouldBeMoving && MovedDist < 5.0f)
		{
			StuckTimeAccumulator += EffectiveDeltaSeconds;

			// Phase 1: After StuckRecoveryTimeSec, attempt reverse + counter-steer.
			// Skip if recovery was already exhausted (micro-wiggle detected).
			if (StuckTimeAccumulator >= StuckRecoveryTimeSec && !bStuckRecoveryActive
				&& !bStuckRecoveryExhausted && StuckTimeAccumulator < StuckDespawnTimeSec)
			{
				bStuckRecoveryActive = true;
				StuckRecoveryElapsed = 0.0f;
				UE_LOG(LogAAATraffic, Log,
					TEXT("Safety: Vehicle '%s' Lane=%d — initiating stuck recovery (reverse + counter-steer)"),
					SafetyPawn ? *SafetyPawn->GetName() : TEXT("NULL"),
					CurrentLane.HandleId);
			}

			// Phase 2: After StuckDespawnTimeSec, give up and despawn.
			if (StuckTimeAccumulator >= StuckDespawnTimeSec)
			{
				if (TrafficSub)
				{
					bPendingRecoveryDespawn = true;
					bStuckRecoveryActive = false;
					bStuckRecoveryExhausted = false;
					TrafficSub->RequestDespawn(this,
						FString::Printf(TEXT("stuck for %.1fs (moved %.1f cm this tick, recovery failed)"),
							StuckTimeAccumulator, MovedDist));
				}
				else
				{
					UE_LOG(LogAAATraffic, Warning,
						TEXT("Safety: stuck vehicle has no subsystem for despawn -- resetting accumulator."));
					StuckTimeAccumulator = 0.0f;
					bStuckRecoveryActive = false;
					bStuckRecoveryExhausted = false;
				}
				return;
			}
		}
		else
		{
			// Vehicle is moving — check if this is genuine recovery or
			// just a micro-wiggle from the reverse maneuver.
			if (bStuckRecoveryActive && StuckTimeAccumulator > StuckRecoveryTimeSec)
			{
				if (MovedDist < 50.0f)
				{
					// Micro-move: recovery failed to truly unstick.
					// Don't reset the timer — let it keep counting toward despawn.
					UE_LOG(LogAAATraffic, Log,
						TEXT("Safety: Vehicle '%s' — stuck recovery micro-move (%.1f cm), marking exhausted"),
						SafetyPawn ? *SafetyPawn->GetName() : TEXT("NULL"), MovedDist);
					bStuckRecoveryActive = false;
					bStuckRecoveryExhausted = true;
					StuckRecoveryElapsed = 0.0f;
					// DO NOT reset StuckTimeAccumulator
				}
				else
				{
					// Genuine recovery — vehicle moved substantially.
					UE_LOG(LogAAATraffic, Log,
						TEXT("Safety: Vehicle '%s' — stuck recovery succeeded (moved %.1f cm)"),
						SafetyPawn ? *SafetyPawn->GetName() : TEXT("NULL"), MovedDist);
					StuckTimeAccumulator = 0.0f;
					bStuckRecoveryActive = false;
					bStuckRecoveryExhausted = false;
					StuckRecoveryElapsed = 0.0f;
				}
			}
			else
			{
				// Normal movement — reset all stuck state.
				StuckTimeAccumulator = 0.0f;
				bStuckRecoveryActive = false;
				bStuckRecoveryExhausted = false;
				StuckRecoveryElapsed = 0.0f;
			}
		}

		// --- Off-road stuck detection (CTE safety net) ---
		// The normal stuck detector exempts vehicles in junction phases,
		// but a vehicle can end up 5+ meters off-road after a bad junction
		// exit and immediately enter Approaching phase for the next junction,
		// bypassing the stuck detector forever. This secondary check fires
		// regardless of junction phase.
		if (bLaneDataReady && LastDiagHalfLaneWidth > 0.0f)
		{
			const float CTEFraction = FMath::Abs(LastDiagSignedCTE) / LastDiagHalfLaneWidth;
			const float CurrentSpeed = SafetyPawn->GetVelocity().Size();
			if (CTEFraction > 1.5f && CurrentSpeed < 10.0f)
			{
				OffRoadStuckTimer += EffectiveDeltaSeconds;
				if (OffRoadStuckTimer >= StuckDespawnTimeSec)
				{
					if (TrafficSub && !bPendingRecoveryDespawn)
					{
						bPendingRecoveryDespawn = true;
						UE_LOG(LogAAATraffic, Warning,
							TEXT("Safety: off-road stuck vehicle '%s' Lane=%d CTE=%.1fcm (%.0f%% of half-lane) "
							     "Speed=%.1f — despawning after %.1fs"),
							SafetyPawn ? *SafetyPawn->GetName() : TEXT("NULL"),
							CurrentLane.HandleId,
							LastDiagSignedCTE, CTEFraction * 100.0f,
							CurrentSpeed, OffRoadStuckTimer);
						TrafficSub->RequestDespawn(this,
							FString::Printf(TEXT("off-road stuck for %.1fs (CTE=%.0f%% of half-lane)"),
								OffRoadStuckTimer, CTEFraction * 100.0f));
					}
					return;
				}
			}
			else if (CTEFraction < 1.2f || CurrentSpeed >= 10.0f)
			{
				// Hysteresis: once timer starts counting (CTE > 150%),
				// don't reset until vehicle definitively recovers
				// (CTE < 120% OR regains meaningful speed).
				OffRoadStuckTimer = 0.0f;
			}
		}
	}

	UpdateVehicleInput(EffectiveDeltaSeconds);

	// --- Debug draw (delegated to separate translation unit) ---
	DrawVehicleDebug();
}

int32 ATrafficVehicleController::FindClosestPointIndex(const FVector& VehicleLocation, bool bForceFullScan)
{
	if (LanePoints.Num() == 0) { return 0; }

	if (bForceFullScan)
	{
		int32 BestIndex = 0;
		float BestDistSq = FVector::DistSquared(VehicleLocation, LanePoints[0]);
		for (int32 i = 1; i < LanePoints.Num(); ++i)
		{
			const float DistSq = FVector::DistSquared(VehicleLocation, LanePoints[i]);
			if (DistSq < BestDistSq)
			{
				BestDistSq = DistSq;
				BestIndex = i;
			}
		}

		LastClosestIndex = BestIndex;
		return BestIndex;
	}

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


bool ATrafficVehicleController::ProjectOntoLaneAtIndex(
	const FVector& ReferenceLocation,
	int32 ClosestIndex,
	FVector& OutProjectedPoint,
	int32& OutSegmentStartIndex) const
{
	if (LanePoints.Num() == 0)
	{
		OutProjectedPoint = FVector::ZeroVector;
		OutSegmentStartIndex = 0;
		return false;
	}

	if (LanePoints.Num() == 1)
	{
		OutProjectedPoint = LanePoints[0];
		OutSegmentStartIndex = 0;
		return true;
	}

	const int32 ClampedIndex = FMath::Clamp(ClosestIndex, 0, LanePoints.Num() - 1);
	const FVector Reference2D(ReferenceLocation.X, ReferenceLocation.Y, 0.0f);

	bool bFoundProjection = false;
	float BestDistSq2D = TNumericLimits<float>::Max();
	FVector BestProjectedPoint = LanePoints[ClampedIndex];
	int32 BestSegmentStartIndex = FMath::Clamp(ClampedIndex, 0, LanePoints.Num() - 2);

	auto ConsiderSegment = [&](int32 SegmentStartIndex)
	{
		if (SegmentStartIndex < 0 || SegmentStartIndex >= LanePoints.Num() - 1)
		{
			return;
		}

		const FVector SegmentStart = LanePoints[SegmentStartIndex];
		const FVector SegmentEnd = LanePoints[SegmentStartIndex + 1];
		const FVector SegmentStart2D(SegmentStart.X, SegmentStart.Y, 0.0f);
		const FVector SegmentEnd2D(SegmentEnd.X, SegmentEnd.Y, 0.0f);
		const FVector SegmentDelta2D = SegmentEnd2D - SegmentStart2D;
		const float SegmentLenSq2D = SegmentDelta2D.SizeSquared();

		FVector CandidateProjectedPoint = SegmentStart;
		if (SegmentLenSq2D > KINDA_SMALL_NUMBER)
		{
			const float Alpha = FMath::Clamp(
				FVector::DotProduct(Reference2D - SegmentStart2D, SegmentDelta2D) / SegmentLenSq2D,
				0.0f,
				1.0f);
			CandidateProjectedPoint = FMath::Lerp(SegmentStart, SegmentEnd, Alpha);
		}

		const FVector CandidateProjectedPoint2D(
			CandidateProjectedPoint.X,
			CandidateProjectedPoint.Y,
			0.0f);
		const float CandidateDistSq2D = FVector::DistSquared(Reference2D, CandidateProjectedPoint2D);
		if (!bFoundProjection || CandidateDistSq2D < BestDistSq2D)
		{
			bFoundProjection = true;
			BestDistSq2D = CandidateDistSq2D;
			BestProjectedPoint = CandidateProjectedPoint;
			BestSegmentStartIndex = SegmentStartIndex;
		}
	};

	ConsiderSegment(ClampedIndex - 1);
	ConsiderSegment(ClampedIndex);

	if (!bFoundProjection)
	{
		BestProjectedPoint = LanePoints[ClampedIndex];
		BestSegmentStartIndex = FMath::Clamp(ClampedIndex, 0, LanePoints.Num() - 2);
	}

	OutProjectedPoint = BestProjectedPoint;
	OutSegmentStartIndex = BestSegmentStartIndex;
	return true;
}


FVector ATrafficVehicleController::GetLookAheadPoint(
	const FVector& VehicleLocation, int32 ClosestIndex) const
{
	if (LanePoints.Num() == 0)
	{
		return FVector::ZeroVector;
	}

	if (LanePoints.Num() == 1)
	{
		return LanePoints[0];
	}

	FVector ProjectedStartPoint = VehicleLocation;
	int32 SegmentStartIndex = FMath::Clamp(ClosestIndex, 0, LanePoints.Num() - 2);
	ProjectOntoLaneAtIndex(VehicleLocation, ClosestIndex, ProjectedStartPoint, SegmentStartIndex);

	// Walk forward from the projected lane position until LookAheadDistance is reached.
	float AccumulatedDist = 0.0f;
	int32 Index = SegmentStartIndex;

	while (Index < LanePoints.Num() - 1)
	{
		const FVector SegmentStart = (Index == SegmentStartIndex)
			? ProjectedStartPoint
			: LanePoints[Index];

		// Use 2D distance so look-ahead accumulation matches the 2D
		// steering computation. On hills, 3D distance is longer than
		// 2D, causing the look-ahead point to be closer than intended.
		const float SegmentDist = FVector::Dist2D(SegmentStart, LanePoints[Index + 1]);
		AccumulatedDist += SegmentDist;

		if (AccumulatedDist >= LookAheadDistance)
		{
			const float Overshoot = AccumulatedDist - LookAheadDistance;
			const float Alpha = 1.0f - (Overshoot / FMath::Max(SegmentDist, KINDA_SMALL_NUMBER));
			return FMath::Lerp(SegmentStart, LanePoints[Index + 1], Alpha);
		}

		++Index;
	}

	// If the look-ahead extends beyond the lane, extrapolate along the
	// final segment direction. Returning LanePoints.Last() would make
	// Ld->0 as the vehicle approaches, causing a steering spike.
	if (LanePoints.Num() >= 2)
	{
		const int32 Last = LanePoints.Num() - 1;
		const FVector FinalSegmentStart = (SegmentStartIndex == Last - 1)
			? ProjectedStartPoint
			: LanePoints[Last - 1];
		const FVector FinalDir = (LanePoints[Last] - FinalSegmentStart).GetSafeNormal();
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

