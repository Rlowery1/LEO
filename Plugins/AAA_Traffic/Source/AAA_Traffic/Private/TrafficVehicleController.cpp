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

static int32 GTrafficVehicleDecisionTrace = 0;
static FAutoConsoleVariableRef CVarTrafficVehicleDecisionTrace(
	TEXT("traffic.VehicleDecisionTrace"),
	GTrafficVehicleDecisionTrace,
	TEXT("Vehicle decision tracing: 0=off, 1=trace lane transitions/lane changes and flush on failures, 2=verbose per-candidate traces."),
	ECVF_Default);

static int32 GTrafficVehicleDecisionTraceMax = 256;
static FAutoConsoleVariableRef CVarTrafficVehicleDecisionTraceMax(
	TEXT("traffic.VehicleDecisionTraceMax"),
	GTrafficVehicleDecisionTraceMax,
	TEXT("Max decision-trace entries held per vehicle before oldest records are evicted."),
	ECVF_Default);

static bool GTrafficVehicleTraceFlushOnSuccess = false;
static FAutoConsoleVariableRef CVarTrafficVehicleTraceFlushOnSuccess(
	TEXT("traffic.VehicleTraceFlushOnSuccess"),
	GTrafficVehicleTraceFlushOnSuccess,
	TEXT("Flush lane decision trace buffer on successful transitions/lane changes (default false)."),
	ECVF_Default);

static float GTrafficWakeGuardMinThrottle = 0.20f;
static FAutoConsoleVariableRef CVarTrafficWakeGuardMinThrottle(
	TEXT("traffic.WakeGuardMinThrottle"),
	GTrafficWakeGuardMinThrottle,
	TEXT("Minimum throttle request to trigger anti-sleep wake guard."),
	ECVF_Default);

static float GTrafficWakeGuardMaxSpeed = 120.0f;
static FAutoConsoleVariableRef CVarTrafficWakeGuardMaxSpeed(
	TEXT("traffic.WakeGuardMaxSpeed"),
	GTrafficWakeGuardMaxSpeed,
	TEXT("Maximum absolute speed (cm/s) where anti-sleep wake guard is allowed to trigger."),
	ECVF_Default);

static int32 GTrafficVehicleDiagnostics = 0;
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

namespace
{
	static bool IsVehicleTraceEnabled(const int32 RequiredLevel)
	{
		return GTrafficVehicleDecisionTrace >= RequiredLevel;
	}
}

// ── FVehicleJunctionState phase transitions ────────────────

void FVehicleJunctionState::BeginApproach(int32 InJunctionId)
{
	checkf(Phase == EJunctionPhase::Idle,
		TEXT("BeginApproach: expected Idle, got %d"), (int32)Phase);
	Phase = EJunctionPhase::Approaching;
	JunctionId = InJunctionId;
	LastReleasedId = 0; // New engagement — clear stale release marker.
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
	, LaneChangeState(ELaneChangeState::None)
	, TargetLaneWidth(0.0f)
	, LaneChangeProgress(0.0f)
	, LaneChangeSettleTimer(0.0f)
	, LaneChangeCooldownRemaining(0.0f)
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
		IDMPersonalityAccelScale       = 1.0f + RandomStream.FRandRange(-S, S);
		IDMPersonalityDecelScale       = 1.0f + RandomStream.FRandRange(-S, S);
		IDMPersonalityTimeHeadwayScale = 1.0f + RandomStream.FRandRange(-S, S);
	}
	else
	{
		IDMPersonalityAccelScale       = 1.0f;
		IDMPersonalityDecelScale       = 1.0f;
		IDMPersonalityTimeHeadwayScale = 1.0f;
	}
}

void ATrafficVehicleController::SetLaneChangeAggression(float Aggression)
{
	Aggression = FMath::Clamp(Aggression, 0.0f, 1.0f);

	// Map 0-1 aggression to internal tuning knobs.
	// Aggression 0: conservative — long blend, high cooldown, strict gap, only if very slow.
	// Aggression 1: aggressive — short blend, low cooldown, loose gap, eager trigger.
	LaneChangeDistance      = FMath::Lerp(2500.0f, 800.0f, Aggression);
	LaneChangeCooldownTime  = FMath::Lerp(8.0f, 2.0f, Aggression);
	LaneChangeSpeedThreshold = FMath::Lerp(0.4f, 0.8f, Aggression);
	LaneChangeGapRequired   = FMath::Lerp(1200.0f, 500.0f, Aggression);
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

	// Threshold: ~15° deviation from straight — below this, treat as straight-through.
	constexpr float StraightThreshold = 0.26f; // sin(15°) ≈ 0.259
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
	// ── Phase-aware junction state handling ──
	// Traversing: keep junction state intact (save/restore no longer needed).
	// Waiting: release occupancy before resetting.
	// Other: full reset.
	if (JnctState.Phase == EJunctionPhase::Traversing)
	{
		UE_LOG(LogAAATraffic, Log,
			TEXT("JNCT INIT-KEEP: Pawn='%s' Phase=Traversing JunctionId=%d NewLane=%d — preserving junction state"),
			GetPawn() ? *GetPawn()->GetName() : TEXT("NULL"),
			JnctState.JunctionId, InLane.HandleId);
	}
	else
	{
		if (JnctState.IsActive())
		{
			UE_LOG(LogAAATraffic, Warning,
				TEXT("JNCT INIT-RESET: Pawn='%s' Phase=%d JunctionId=%d bWaiting=%s TransPts=%d NewLane=%d — resetting"),
				GetPawn() ? *GetPawn()->GetName() : TEXT("NULL"),
				(int32)JnctState.Phase, JnctState.JunctionId,
				JnctState.bWaiting ? TEXT("YES") : TEXT("NO"),
				JnctState.TransitionPoints.Num(),
				InLane.HandleId);

			// If waiting with occupancy, release it before resetting.
			if (JnctState.Phase == EJunctionPhase::Waiting && JnctState.JunctionId != 0)
			{
				if (UWorld* W = GetWorld())
				{
					if (UTrafficSubsystem* TrafficSub = W->GetSubsystem<UTrafficSubsystem>())
					{
						TrafficSub->ReleaseJunction(JnctState.JunctionId, this);
					}
				}
			}
		}
		JnctState.Reset();
	}

	CurrentLane = InLane;
	bLaneDataReady = false;
	bAtDeadEnd = false;
	DeadEndDespawnTimer = 0.0f;
	DistanceTraveledOnLane = 0.0f;
	PreviousVehicleLocation = FVector::ZeroVector;
	LastClosestIndex = 0;
	PreviousHeadingCrossZ = 0.0f;
	WaitLogThrottleCounter = 0;

	// Clear IDM reaction delay state so stale leader readings from a
	// previous lane don't bleed into the new lane's decision-making.
	LeaderDelayBuffer.Empty();
	IDMTimeClock = 0.0f;

	// Reset turn signal.
	CurrentTurnSignal = ETurnSignalState::Off;

	// Reset navigational pre-positioning.
	bNavigationalLaneChange = false;
	NavigationalTargetLane = FTrafficLaneHandle();

	// Reset lane-change state when entering a new lane.
	LaneChangeState = ELaneChangeState::None;
	LaneChangeProgress = 0.0f;
	LaneChangeSettleTimer = 0.0f;
	TargetLanePoints.Empty();

	// Flag first tick on the new lane so the curvature scan covers
	// the full polyline for braking envelope setup.
	bFirstTickOnLane = true;

	UWorld* World = GetWorld();
	if (!World)
	{
		UE_LOG(LogAAATraffic, Warning, TEXT("TrafficVehicleController: World is null, cannot initialize lane following."));
		return;
	}
	UTrafficSubsystem* TrafficSub = World->GetSubsystem<UTrafficSubsystem>();
	ITrafficRoadProvider* Provider = TrafficSub ? TrafficSub->GetProvider() : nullptr;
	if (!Provider)
	{
		UE_LOG(LogAAATraffic, Warning, TEXT("TrafficVehicleController: No road provider available."));
		return;
	}

	CachedProvider = Provider;

	LanePoints.Reset();
	if (Provider->GetLanePath(CurrentLane, LanePoints, LaneWidth) && LanePoints.Num() >= 2)
	{
		bLaneDataReady = true;

		// --- JUNCTION DIAGNOSTIC: Immediately check what the precomputed map
		// says about this lane so we know ON SPAWN whether this vehicle will
		// ever detect junctions ---
		{
			const float TotalLen = Provider->GetLaneLength(CurrentLane);
			const ITrafficRoadProvider::FJunctionScanResult InitScan =
				Provider->GetDistanceToNextJunction(CurrentLane, TotalLen, 50000.0f, 10);
			const int32 DirectJunction = Provider->GetJunctionForLane(CurrentLane);
			TArray<FTrafficLaneHandle> InitConnected = Provider->GetConnectedLanes(CurrentLane);

			FString ConnStr;
			for (const FTrafficLaneHandle& C : InitConnected)
			{
				const int32 CJnct = Provider->GetJunctionForLane(C);
				ConnStr += FString::Printf(TEXT(" %d(jnct=%d)"), C.HandleId, CJnct);
			}

			UE_LOG(LogAAATraffic, Warning,
				TEXT("JDIAG LANE-INIT: Pawn='%s' Lane=%d Length=%.0f DirectJunction=%d "
					 "PrecomputedScan=%s ScanJnctId=%d ScanDist=%.0f ScanJnctLane=%d "
					 "NumConnections=%d Connections=[%s]"),
				GetPawn() ? *GetPawn()->GetName() : TEXT("NULL"),
				CurrentLane.HandleId,
				TotalLen,
				DirectJunction,
				InitScan.IsValid() ? TEXT("VALID") : TEXT("INVALID"),
				InitScan.JunctionId,
				InitScan.DistanceCm,
				InitScan.JunctionLane.HandleId,
				InitConnected.Num(),
				*ConnStr);
		}

		// Compute total lane length for diagnostics.
		float TotalLaneLength = 0.0f;
		for (int32 i = 0; i < LanePoints.Num() - 1; ++i)
		{
			TotalLaneLength += FVector::Dist(LanePoints[i], LanePoints[i + 1]);
		}

		UE_LOG(LogAAATraffic, Log,
			TEXT("TrafficVehicleController: Lane loaded — %d points, width %.1f cm, length %.1f cm (%.1f m)."),
			LanePoints.Num(), LaneWidth, TotalLaneLength, TotalLaneLength / 100.0f);

		// Query lane speed limit from the provider.
		const float LaneSpeedLimit = Provider->GetLaneSpeedLimit(CurrentLane);
		if (LaneSpeedLimit > 0.0f)
		{
			TargetSpeed = LaneSpeedLimit;
		}
		else if (DefaultSpeedLimit > 0.0f)
		{
			TargetSpeed = DefaultSpeedLimit;
		}
		// else: keep the TargetSpeed set by SetTargetSpeed (spawner-configured).
		// Preserve the base speed for restoration after speed-zone transitions.
		if (BaseTargetSpeed <= 0.0f)
		{
			BaseTargetSpeed = TargetSpeed;
		}
	}
	else
	{
		UE_LOG(LogAAATraffic, Warning, TEXT("TrafficVehicleController: Failed to load lane path data."));
	}

	// Notify the subsystem of our lane assignment.
	if (TrafficSub)
	{
		TrafficSub->UpdateVehicleLane(this, CurrentLane);
	}
}

void ATrafficVehicleController::OnPossess(APawn* InPawn)
{
	Super::OnPossess(InPawn);
	RandomStream.Initialize(RandomSeed);

	// --- Diagnostic: log pawn class and movement component details ---
	if (InPawn)
	{
		const FString PawnClass = InPawn->GetClass()->GetName();
		UPawnMovementComponent* GenericMC = InPawn->GetMovementComponent();
		const FString MCClass = GenericMC ? GenericMC->GetClass()->GetName() : TEXT("NULL");

		UChaosWheeledVehicleMovementComponent* ChaosMC =
			Cast<UChaosWheeledVehicleMovementComponent>(GenericMC);

		UE_LOG(LogAAATraffic, Log,
			TEXT("VehicleController::OnPossess: Pawn='%s' Class='%s' MovementComponent='%s' ChaosWheeledCast=%s"),
			*InPawn->GetName(), *PawnClass, *MCClass,
			ChaosMC ? TEXT("OK") : TEXT("FAILED — vehicles will NOT move"));

		if (!ChaosMC)
		{
			UE_LOG(LogAAATraffic, Error,
				TEXT("VehicleController::OnPossess: CRITICAL — Pawn '%s' (class '%s') does not have a UChaosWheeledVehicleMovementComponent. "
					 "GetMovementComponent() returned '%s'. The vehicle controller requires ChaosWheeledVehicleMovementComponent to set throttle/steering/brake. "
					 "Ensure the vehicle Blueprint inherits from AWheeledVehiclePawn or has this component added."),
				*InPawn->GetName(), *PawnClass, *MCClass);

			// List all components on the pawn for further diagnosis.
			TArray<UActorComponent*> AllComps;
			InPawn->GetComponents(AllComps);
			for (const UActorComponent* Comp : AllComps)
			{
				UE_LOG(LogAAATraffic, Log,
					TEXT("  Component: '%s' Class='%s'"),
					*Comp->GetName(), *Comp->GetClass()->GetName());
			}
		}
		else
		{
			// --- Ensure Chaos vehicle is in a drivable state ---
			// Marketplace Blueprints often default to parked/sleeping/handbrake-on.
			// Explicitly wake and unpark the vehicle so throttle input takes effect.
			ChaosMC->SetParked(false);
			ChaosMC->SetSleeping(false);
			ChaosMC->SetHandbrakeInput(false);
			ChaosMC->SetTargetGear(1, /*bImmediate=*/ true);
			ChaosMC->SetUseAutomaticGears(true);

			// --- Prevent ProcessSleeping from re-sleeping the physics body ---
			// PROVEN by diagnostic: BodyAwake=NO at T=2s despite per-tick
			// SetSleeping(false). The engine's ProcessSleeping uses a speed-
			// based sleep threshold (default 10 cm/s) and an interpolated
			// ThrottleInput (gated by bRequiresControllerForInputs). Two
			// belt-and-suspenders fixes:
			//
			// 1) SetRequiresControllerForInputs(false): ensures ThrottleInput
			//    interpolation always runs in UpdateState regardless of
			//    controller type, so ProcessSleeping sees throttle > wake
			//    tolerance and keeps the body awake.
			//
			// 2) SleepThreshold = 0: the speed-based sleep path requires
			//    speed² < threshold². With threshold=0 this is never true,
			//    so ProcessSleeping's sleep counter never increments even if
			//    the vehicle is momentarily stationary.
			ChaosMC->SetRequiresControllerForInputs(false);
			ChaosMC->SleepThreshold = 0.0f;

			// --- Prevent Chaos solver from putting the physics body to sleep ---
			// PROVEN through 3 test runs: GT-level sleep prevention (SleepThreshold,
			// RequiresControllerForInputs, per-tick SetSleeping) all failed because
			// the Chaos physics solver independently sleeps rigid bodies based on
			// kinetic energy via the Island Manager. The Island Manager only skips
			// sleep for particles where SleepType == NeverSleep.
			if (UPrimitiveComponent* Prim = ChaosMC->UpdatedPrimitive)
			{
				if (FBodyInstance* BI = Prim->GetBodyInstance())
				{
					FPhysicsActorHandle PhysHandle = BI->GetPhysicsActorHandle();
					if (PhysHandle)
					{
						PhysHandle->GetGameThreadAPI().SetSleepType(Chaos::ESleepType::NeverSleep);
						UE_LOG(LogAAATraffic, Log,
							TEXT("VehicleController::OnPossess: Set Chaos ESleepType::NeverSleep on '%s'"),
							*InPawn->GetName());
					}
				}
			}

			// --- Neutralize the handbrake/parking brake torque per wheel ---
			// The Chaos physics sim applies HandbrakeTorque on any rear wheel
			// with HandbrakeEnabled whenever EITHER the handbrake input is
			// nonzero OR ParkingEnabled is true (SetParked). Marketplace
			// Blueprints can re-assert parking/handbrake state via their own
			// Tick or EventGraph, overriding our SetParked(false) call above.
			//
			// Rather than fighting per-frame to clear that state, we zero out
			// the HandbrakeTorque magnitude on every wheel. This is a one-time
			// config change: 0 torque * any input = 0 force. Our AI never
			// uses the handbrake; we use SetBrakeInput() for all deceleration.
			const int32 NumWheels = ChaosMC->WheelSetups.Num();
			for (int32 WIdx = 0; WIdx < NumWheels; ++WIdx)
			{
				ChaosMC->SetWheelHandbrakeTorque(WIdx, 0.0f);
			}

			UE_LOG(LogAAATraffic, Log,
				TEXT("VehicleController::OnPossess: Vehicle '%s' initialized — "
					 "Parked=false, Sleeping=false, Handbrake=false, Gear=1, AutoGears=true, "
					 "HandbrakeTorque zeroed on %d wheels, "
					 "RequiresControllerForInputs=false, SleepThreshold=0"),
				*InPawn->GetName(), NumWheels);

			// I3 FIX: Compute vehicle-specific max braking deceleration from
			// the Chaos wheel setup so emergency braking maps to reality.
			// Formula: TotalBrakeTorque / (avgWheelRadius * vehicleMass) → cm/s².
			{
				float TotalBrakeTorque = 0.0f;  // Nm across all wheels
				float RadiusSum = 0.0f;         // accumulate wheel radii (cm)
				int32 RadiusCount = 0;
				for (int32 WI = 0; WI < NumWheels; ++WI)
				{
					if (WI < ChaosMC->WheelSetups.Num())
					{
						const auto& WS = ChaosMC->WheelSetups[WI];
						if (WS.WheelClass)
						{
							const UChaosVehicleWheel* WheelCDO = WS.WheelClass->GetDefaultObject<UChaosVehicleWheel>();
							if (WheelCDO)
							{
								TotalBrakeTorque += WheelCDO->MaxBrakeTorque;
								RadiusSum += WheelCDO->WheelRadius;
								++RadiusCount;
							}
						}
					}
				}
				// Average wheel radius: use measured data, else 30 cm fallback.
				const float AvgWheelRadius = (RadiusCount > 0)
					? (RadiusSum / static_cast<float>(RadiusCount))
					: 30.0f;
				// Vehicle mass from Chaos physics body.
				float VehicleMassKg = 1500.0f; // fallback
				if (UPrimitiveComponent* Prim = ChaosMC->UpdatedPrimitive)
				{
					if (FBodyInstance* BI = Prim->GetBodyInstance())
					{
						VehicleMassKg = FMath::Max(BI->GetBodyMass(), 100.0f);
					}
				}
				// Torque(Nm) / Radius(m) = Force(N), Force/Mass = accel(m/s²) → *100 for cm/s²
				const float RadiusM = FMath::Max(AvgWheelRadius * 0.01f, 0.1f);
				MaxBrakeDecelCmPerSec2 = FMath::Clamp(
					(TotalBrakeTorque / RadiusM / VehicleMassKg) * 100.0f,
					300.0f, 2000.0f); // clamp to sane range
				UE_LOG(LogAAATraffic, Log,
					TEXT("VehicleController::OnPossess: Vehicle '%s' MaxBrakeDecel=%.0f cm/s² "
						 "(TotalBrakeTorque=%.0f Nm, AvgWheelRadius=%.1f cm, Mass=%.0f kg)"),
					*InPawn->GetName(), MaxBrakeDecelCmPerSec2,
					TotalBrakeTorque, AvgWheelRadius, VehicleMassKg);
			}

			// --- Compute wheelbase and max steer angle from Chaos wheel setup ---
			// Walk the wheel CDOs: group by AxleType (Front vs Rear), get bone
			// positions from the skeletal mesh to compute axle separation.
			// Also extract MaxSteerAngle from the first steered (Front) wheel.
			{
				float FrontAxleX = 0.0f, RearAxleX = 0.0f;
				int32 FrontCount = 0, RearCount = 0;
				float MaxSteerDeg = 0.0f;

				// Get the skeletal mesh for bone position lookups.
				USkeletalMeshComponent* MeshComp = nullptr;
				if (InPawn)
				{
					MeshComp = InPawn->FindComponentByClass<USkeletalMeshComponent>();
				}

				for (int32 WI = 0; WI < ChaosMC->WheelSetups.Num(); ++WI)
				{
					const auto& WS = ChaosMC->WheelSetups[WI];
					if (!WS.WheelClass) continue;
					const UChaosVehicleWheel* WheelCDO = WS.WheelClass->GetDefaultObject<UChaosVehicleWheel>();
					if (!WheelCDO) continue;

					// Get wheel position from the bone name on the skeletal mesh.
					float WheelLocalX = WS.AdditionalOffset.X; // fallback
					if (MeshComp && !WS.BoneName.IsNone())
					{
						const int32 BoneIdx = MeshComp->GetBoneIndex(WS.BoneName);
						if (BoneIdx != INDEX_NONE)
						{
							// GetBoneTransform returns world space; convert to local.
							const FTransform BoneWorld = MeshComp->GetBoneTransform(BoneIdx);
							const FTransform ActorInv = InPawn->GetActorTransform().Inverse();
							const FVector LocalPos = ActorInv.TransformPosition(BoneWorld.GetLocation());
							WheelLocalX = LocalPos.X + WS.AdditionalOffset.X;
						}
					}

					if (WheelCDO->AxleType == EAxleType::Front)
					{
						FrontAxleX += WheelLocalX;
						++FrontCount;
						MaxSteerDeg = FMath::Max(MaxSteerDeg, WheelCDO->MaxSteerAngle);
					}
					else if (WheelCDO->AxleType == EAxleType::Rear)
					{
						RearAxleX += WheelLocalX;
						++RearCount;
					}
				}
				if (FrontCount > 0 && RearCount > 0)
				{
					FrontAxleX /= static_cast<float>(FrontCount);
					RearAxleX /= static_cast<float>(RearCount);
					VehicleWheelbaseCm = FMath::Clamp(FMath::Abs(FrontAxleX - RearAxleX), 150.0f, 600.0f);
				}
				if (MaxSteerDeg > 1.0f)
				{
					VehicleMaxSteerAngleRad = FMath::DegreesToRadians(FMath::Clamp(MaxSteerDeg, 15.0f, 55.0f));
				}
				UE_LOG(LogAAATraffic, Log,
					TEXT("VehicleController::OnPossess: Vehicle '%s' Wheelbase=%.1f cm, MaxSteerAngle=%.1f°"),
					*InPawn->GetName(), VehicleWheelbaseCm,
					FMath::RadiansToDegrees(VehicleMaxSteerAngleRad));
			}
		}

		// --- Compute front-bumper extent from actor bounds ---
		// Distance from actor origin to the front-most point of the
		// bounding box along the forward axis.  Uses the AABB support
		// function: for each world axis pick Max or Min depending on
		// the sign of the forward vector, giving the corner that
		// projects furthest forward.  This is orientation-correct
		// regardless of which world axis the car faces at spawn.
		{
			const FBox ActorBounds = InPawn->GetComponentsBoundingBox(/*bNonColliding=*/ false);
			if (ActorBounds.IsValid)
			{
				const FVector ActorOrigin = InPawn->GetActorLocation();
				const FVector Fwd = InPawn->GetActorForwardVector();
				const FVector Right = InPawn->GetActorRightVector();

				// Support point: the AABB corner furthest along Fwd.
				FVector FrontCorner;
				FrontCorner.X = (Fwd.X >= 0.0f) ? ActorBounds.Max.X : ActorBounds.Min.X;
				FrontCorner.Y = (Fwd.Y >= 0.0f) ? ActorBounds.Max.Y : ActorBounds.Min.Y;
				FrontCorner.Z = (Fwd.Z >= 0.0f) ? ActorBounds.Max.Z : ActorBounds.Min.Z;

				VehicleFrontExtent = FVector::DotProduct(FrontCorner - ActorOrigin, Fwd);
				// Clamp to reasonable range to handle edge cases.
				VehicleFrontExtent = FMath::Clamp(VehicleFrontExtent, 50.0f, 1000.0f);

				// Rear extent: AABB corner furthest along -Fwd.
				FVector RearCorner;
				RearCorner.X = (Fwd.X < 0.0f) ? ActorBounds.Max.X : ActorBounds.Min.X;
				RearCorner.Y = (Fwd.Y < 0.0f) ? ActorBounds.Max.Y : ActorBounds.Min.Y;
				RearCorner.Z = (Fwd.Z < 0.0f) ? ActorBounds.Max.Z : ActorBounds.Min.Z;
				VehicleRearExtent = FMath::Abs(FVector::DotProduct(RearCorner - ActorOrigin, Fwd));
				VehicleRearExtent = FMath::Clamp(VehicleRearExtent, 50.0f, 1000.0f);

				// Lateral half-width: AABB corner furthest along Right.
				FVector RightCorner;
				RightCorner.X = (Right.X >= 0.0f) ? ActorBounds.Max.X : ActorBounds.Min.X;
				RightCorner.Y = (Right.Y >= 0.0f) ? ActorBounds.Max.Y : ActorBounds.Min.Y;
				RightCorner.Z = (Right.Z >= 0.0f) ? ActorBounds.Max.Z : ActorBounds.Min.Z;
				VehicleLateralHalfWidth = FVector::DotProduct(RightCorner - ActorOrigin, Right);
				VehicleLateralHalfWidth = FMath::Clamp(VehicleLateralHalfWidth, 50.0f, 300.0f);
			}
			else
			{
				// Fallback: typical sedan half-length.
				VehicleFrontExtent = 250.0f;
				VehicleRearExtent = 250.0f;
				VehicleLateralHalfWidth = 100.0f;
			}
			UE_LOG(LogAAATraffic, Log,
				TEXT("VehicleController::OnPossess: Vehicle '%s' front=%.1f rear=%.1f halfWidth=%.1f cm"),
				*InPawn->GetName(), VehicleFrontExtent, VehicleRearExtent, VehicleLateralHalfWidth);
		}

		// --- Neutralize marketplace BP physics-optimization parking ---
		// PROVEN ROOT CAUSE: The parent vehicle Blueprint (DD_Vehicles_Advanced)
		// runs "Physics optimization" every tick via EventTick → Sequence Then 0.
		// Entry gate: (Base Vehicle Initialized AND Optimized AND GameTime > BeginPlayMargin)
		// Once this gate opens (~2s), the Optimization Unpossessed block detects
		// that the vehicle has no PlayerController (only our AIController), treats
		// it as "unpossessed", and calls CE_StopCar → CE Set Parked (Parked=true),
		// killing the drivetrain every frame.
		//
		// Fix: use UE reflection to set the BP's "Optimized" variable to false.
		// This disables the entry gate, preventing the entire parking chain from
		// ever firing. All other parent EventTick behavior (engine sound, wheel
		// effects, smoke VFX, crash impacts, lights) continues unaffected because
		// those are on Sequence Then 1-4, which do not check 'Optimized'.
		//
		// Also set "Can Sleep" to false as a secondary safety net — this is the
		// inner gate checked by CE_CheckVehicleSleep before evaluating wake/sleep.
		{
			UClass* VehicleBPClass = InPawn->GetClass();
			int32 PropertiesNeutralized = 0;

			// Disable the 'Optimized' gate (prevents Physics optimization from running).
			if (FBoolProperty* OptimizedProp = FindFProperty<FBoolProperty>(VehicleBPClass, TEXT("Optimized")))
			{
				OptimizedProp->SetPropertyValue_InContainer(InPawn, false);
				++PropertiesNeutralized;
			}

			// Disable the 'Can Sleep' inner gate (prevents CE_CheckVehicleSleep from
			// evaluating wake/sleep even if the Optimized gate is bypassed).
			// BP variable names with spaces use the space-free internal FName — search
			// by both common internal names and DisplayName metadata for robustness.
			FBoolProperty* CanSleepProp = FindFProperty<FBoolProperty>(VehicleBPClass, TEXT("CanSleep"));
			if (!CanSleepProp)
			{
				CanSleepProp = FindFProperty<FBoolProperty>(VehicleBPClass, TEXT("bCanSleep"));
			}
			if (!CanSleepProp)
			{
				CanSleepProp = FindFProperty<FBoolProperty>(VehicleBPClass, TEXT("Can Sleep"));
			}
#if WITH_EDITORONLY_DATA
			if (!CanSleepProp)
			{
				// Last resort: iterate all bool properties and match by DisplayName metadata.
				// HasMetaData/GetMetaData are editor-only APIs (metadata stripped in Game builds).
				for (TFieldIterator<FBoolProperty> It(VehicleBPClass); It; ++It)
				{
					if (It->HasMetaData(TEXT("DisplayName")))
					{
						const FString& DisplayName = It->GetMetaData(TEXT("DisplayName"));
						if (DisplayName.Equals(TEXT("Can Sleep"), ESearchCase::IgnoreCase))
						{
							CanSleepProp = *It;
							break;
						}
					}
				}
			}
#endif // WITH_EDITORONLY_DATA
			if (CanSleepProp)
			{
				CanSleepProp->SetPropertyValue_InContainer(InPawn, false);
				++PropertiesNeutralized;
			}

			if (PropertiesNeutralized > 0)
			{
				UE_LOG(LogAAATraffic, Log,
					TEXT("VehicleController::OnPossess: Neutralized %d BP optimization properties on '%s' "
						 "(Optimized=false, CanSleep=false) — parent BP parking chain disabled."),
					PropertiesNeutralized, *InPawn->GetName());
			}
			else
			{
				// Property names may differ across marketplace pack versions. Log all
				// boolean properties so the developer can identify the correct FName.
				UE_LOG(LogAAATraffic, Warning,
					TEXT("VehicleController::OnPossess: Could not find 'Optimized' or 'Can Sleep' "
						 "boolean properties on '%s' (class '%s'). The parent BP may still park this vehicle. "
						 "Listing all boolean properties for diagnosis:"),
					*InPawn->GetName(), *VehicleBPClass->GetName());

				for (TFieldIterator<FBoolProperty> It(VehicleBPClass); It; ++It)
				{
					UE_LOG(LogAAATraffic, Warning, TEXT("  Bool property: '%s'"), *It->GetName());
				}
			}
		}

		// --- Bind collision response ---
		// When the pawn physically hits another actor, start a brief
		// full-braking period so the vehicle doesn't just keep pushing.
		InPawn->OnActorHit.AddDynamic(this, &ATrafficVehicleController::HandleActorHit);
	}
	else
	{
		UE_LOG(LogAAATraffic, Error, TEXT("VehicleController::OnPossess: InPawn is NULL."));
	}

	if (UWorld* World = GetWorld())
	{
		if (UTrafficSubsystem* TrafficSub = World->GetSubsystem<UTrafficSubsystem>())
		{
			TrafficSub->RegisterVehicle(this);
		}
	}
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
					TEXT("JNCT RELEASE-UNPOSSESS: Pawn='%s' unpossess — Phase=%d, nothing to release"),
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
		TEXT("COLLISION: Pawn='%s' hit '%s' impulse=%.0f — braking for 1s"),
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
	if (GTrafficVehicleDiagnostics > 0 && !bDiagLoggedMovementCheck && GetPawn())
	{
		if (DiagSpawnLocation.IsZero())
		{
			DiagSpawnLocation = GetPawn()->GetActorLocation();
		}
		DiagElapsedTime += DeltaSeconds;
		if (DiagElapsedTime >= 2.0f)
		{
			bDiagLoggedMovementCheck = true;
			const FVector CurrentLoc = GetPawn()->GetActorLocation();
			const float DistMoved = FVector::Dist(DiagSpawnLocation, CurrentLoc);
			UChaosWheeledVehicleMovementComponent* DiagMC =
				Cast<UChaosWheeledVehicleMovementComponent>(GetPawn()->GetMovementComponent());
			const float FwdSpeed = DiagMC ? DiagMC->GetForwardSpeed() : 0.0f;

			// Deep physics state diagnostic.
			if (DiagMC)
			{
				const float EngineRPM = DiagMC->GetEngineRotationSpeed();
				const int32 CurGear = DiagMC->GetCurrentGear();
				const int32 TgtGear = DiagMC->GetTargetGear();
				const bool bMechSim = DiagMC->bMechanicalSimEnabled;
				const int32 NumWheels = DiagMC->WheelSetups.Num();
				const int32 NumWheelInstances = DiagMC->Wheels.Num();
				const bool bHasOutput = (DiagMC->PhysicsVehicleOutput().Get() != nullptr);
				const bool bCompActive = DiagMC->IsActive();
				const bool bCompRegistered = DiagMC->IsRegistered();

				// Check physics body simulation state.
				UPrimitiveComponent* UpdatedPrim = DiagMC->UpdatedPrimitive;
				bool bSimulatingPhysics = false;
				bool bBodyAwake = false;
				if (UpdatedPrim)
				{
					bSimulatingPhysics = UpdatedPrim->IsSimulatingPhysics();
					FBodyInstance* BI = UpdatedPrim->GetBodyInstance();
					bBodyAwake = BI ? BI->IsInstanceAwake() : false;
				}

				// Query internal Chaos vehicle input state (what physics sim actually sees)
				const bool bIsParked = DiagMC->IsParked();
				const bool bRawHandbrake = DiagMC->GetHandbrakeInput();
				// Get the interpolated handbrake value that feeds physics
				// HandbrakeInput is a protected float member, but GetHandbrakeInput()
				// returns the raw bool. We can't read the interpolated value directly,
				// so we check GetParkedState and the raw bool.

				UE_LOG(LogAAATraffic, Warning,
					TEXT("DEEP DIAG (2s): Pawn='%s' DistFromSpawn=%.1f FwdSpeed=%.1f "
						 "EngineRPM=%.0f Gear=%d/%d bMechSimEnabled=%s "
						 "WheelSetups=%d WheelInstances=%d bHasPhysOutput=%s "
						 "CompActive=%s CompRegistered=%s "
						 "SimulatingPhysics=%s BodyAwake=%s "
						 "bParked=%s bRawHandbrake=%s %s"),
					*GetPawn()->GetName(), DistMoved, FwdSpeed,
					EngineRPM, CurGear, TgtGear,
					bMechSim ? TEXT("true") : TEXT("FALSE"),
					NumWheels, NumWheelInstances,
					bHasOutput ? TEXT("true") : TEXT("FALSE"),
					bCompActive ? TEXT("true") : TEXT("FALSE"),
					bCompRegistered ? TEXT("true") : TEXT("FALSE"),
					bSimulatingPhysics ? TEXT("true") : TEXT("FALSE"),
					bBodyAwake ? TEXT("true") : TEXT("FALSE"),
					bIsParked ? TEXT("PARKED") : TEXT("unparked"),
					bRawHandbrake ? TEXT("HANDBRAKE-ON") : TEXT("handbrake-off"),
					(DistMoved > 100.0f) ? TEXT("DRIVING OK") : TEXT("NOT DRIVING"));

				// Per-wheel diagnostics — are wheels touching the ground?
				for (int32 WIdx = 0; WIdx < NumWheelInstances; ++WIdx)
				{
					const FWheelStatus& WS = DiagMC->GetWheelState(WIdx);
					UE_LOG(LogAAATraffic, Warning,
						TEXT("  WHEEL %d: InContact=%s DriveTorque=%.1f BrakeTorque=%.1f "
							 "SlipAngle=%.2f NormSuspLen=%.2f SpringForce=%.1f"),
						WIdx,
						WS.bInContact ? TEXT("YES") : TEXT("NO"),
						WS.DriveTorque, WS.BrakeTorque,
						WS.SlipAngle,
						WS.NormalizedSuspensionLength, WS.SpringForce);
				}
			}
			else
			{
				UE_LOG(LogAAATraffic, Warning,
					TEXT("DEEP DIAG (2s): Pawn='%s' DistFromSpawn=%.1f — "
						 "No ChaosWheeledVehicleMovementComponent"),
					*GetPawn()->GetName(), DistMoved);
			}
		}
	}

	// --- Periodic diagnostic: comprehensive vehicle state every 2 seconds ---
	// Gated behind traffic.VehicleDiagnostics CVar to avoid log spam and per-tick overhead.
	if (GTrafficVehicleDiagnostics > 0 && GetPawn() && bLaneDataReady)
	{
		DiagPeriodicTimer += DeltaSeconds;

		UChaosWheeledVehicleMovementComponent* PeriodicMC =
			Cast<UChaosWheeledVehicleMovementComponent>(GetPawn()->GetMovementComponent());
		const float CurSpeed = PeriodicMC ? PeriodicMC->GetForwardSpeed() : 0.0f;

		// --- STOPPED one-shot: fires the instant speed drops to 0 after moving ---
		if (!bDiagWasMoving && FMath::Abs(CurSpeed) > 10.0f)
		{
			bDiagWasMoving = true;
		}
		if (bDiagWasMoving && !bDiagLoggedStopped && FMath::Abs(CurSpeed) < 1.0f)
		{
			bDiagLoggedStopped = true;

			// Gather comprehensive state at the exact moment of stop.
			bool bStopBodyAwake = false;
			float StopRPM = 0.0f;
			int32 StopGear = 0, StopTgtGear = 0;
			float StopRawThrottle = 0.0f, StopRawBrake = 0.0f, StopRawSteering = 0.0f;
			bool StopHandbrake = false, StopParked = false;
			bool StopMechSim = false, StopAutoGears = false;
			if (PeriodicMC)
			{
				StopRPM = PeriodicMC->GetEngineRotationSpeed();
				StopGear = PeriodicMC->GetCurrentGear();
				StopTgtGear = PeriodicMC->GetTargetGear();
				StopRawThrottle = PeriodicMC->GetThrottleInput();
				StopRawBrake = PeriodicMC->GetBrakeInput();
				StopRawSteering = PeriodicMC->GetSteeringInput();
				StopHandbrake = PeriodicMC->GetHandbrakeInput();
				StopParked = PeriodicMC->IsParked();
				StopMechSim = PeriodicMC->bMechanicalSimEnabled;
				StopAutoGears = PeriodicMC->GetUseAutoGears();

				UPrimitiveComponent* UpdPrim = PeriodicMC->UpdatedPrimitive;
				if (UpdPrim)
				{
					FBodyInstance* BI = UpdPrim->GetBodyInstance();
					bStopBodyAwake = BI ? BI->IsInstanceAwake() : false;
				}
			}

			const FVector VehLoc = GetPawn()->GetActorLocation();
			const int32 CIdx = FindClosestPointIndex(VehLoc);

			UE_LOG(LogAAATraffic, Warning,
				TEXT(">>STOPPED<< Pawn='%s' at T=%.1fs | "
					 "Speed=%.1f Loc=(%.0f,%.0f,%.0f) ClosestIdx=%d/%d DistTraveled=%.1f | "
					 "BodyAwake=%s | "
					 "Parked=%s Handbrake=%s MechSim=%s AutoGears=%s | "
					 "RPM=%.0f Gear=%d/%d RawThrottle=%.2f RawBrake=%.2f RawSteering=%.2f | "
					 "bAtDeadEnd=%s JnctState.bWaiting=%s LaneChangeState=%d"),
				*GetPawn()->GetName(), DiagElapsedTime,
				CurSpeed, VehLoc.X, VehLoc.Y, VehLoc.Z,
				CIdx, LanePoints.Num(), DistanceTraveledOnLane,
				bStopBodyAwake ? TEXT("YES") : TEXT("NO"),
				StopParked ? TEXT("YES") : TEXT("no"),
				StopHandbrake ? TEXT("YES") : TEXT("no"),
				StopMechSim ? TEXT("yes") : TEXT("NO"),
				StopAutoGears ? TEXT("yes") : TEXT("NO"),
				StopRPM, StopGear, StopTgtGear,
				StopRawThrottle, StopRawBrake, StopRawSteering,
				bAtDeadEnd ? TEXT("YES") : TEXT("no"),
				JnctState.bWaiting ? TEXT("YES") : TEXT("no"),
				static_cast<int32>(LaneChangeState));

			// Per-wheel state at stop
			if (PeriodicMC)
			{
				for (int32 WIdx = 0; WIdx < PeriodicMC->Wheels.Num(); ++WIdx)
				{
					const FWheelStatus& WS = PeriodicMC->GetWheelState(WIdx);
					UE_LOG(LogAAATraffic, Warning,
						TEXT(">>STOPPED<< %s WHEEL %d: InContact=%s DriveTorque=%.1f "
							 "BrakeTorque=%.1f SlipAngle=%.2f SpringForce=%.1f"),
						*GetPawn()->GetName(), WIdx,
						WS.bInContact ? TEXT("YES") : TEXT("NO"),
						WS.DriveTorque, WS.BrakeTorque,
						WS.SlipAngle, WS.SpringForce);
				}
			}
		}

		// --- Periodic log every DiagPeriodicInterval seconds ---
		if (DiagPeriodicTimer >= DiagPeriodicInterval)
		{
			DiagPeriodicTimer -= DiagPeriodicInterval;

			const FVector VehLoc = GetPawn()->GetActorLocation();
			const int32 CIdx = FindClosestPointIndex(VehLoc);
			const float RemDist = GetRemainingDistance(CIdx);
			const float ThrshVal = FMath::Max(LookAheadDistance, FMath::Abs(CurSpeed) * 1.0f);

			bool bBodyAwake = false;
			float DiagRPM = 0.0f;
			int32 DiagGear = 0, DiagTgtGear = 0;
			float DiagThrottle = 0.0f, DiagBrake = 0.0f, DiagSteering = 0.0f;
			bool bDiagParked = false, bDiagHandbrake = false;
			if (PeriodicMC)
			{
				DiagRPM = PeriodicMC->GetEngineRotationSpeed();
				DiagGear = PeriodicMC->GetCurrentGear();
				DiagTgtGear = PeriodicMC->GetTargetGear();
				DiagThrottle = PeriodicMC->GetThrottleInput();
				DiagBrake = PeriodicMC->GetBrakeInput();
				DiagSteering = PeriodicMC->GetSteeringInput();
				bDiagParked = PeriodicMC->IsParked();
				bDiagHandbrake = PeriodicMC->GetHandbrakeInput();

				UPrimitiveComponent* UpdPrim = PeriodicMC->UpdatedPrimitive;
				if (UpdPrim)
				{
					FBodyInstance* BI = UpdPrim->GetBodyInstance();
					bBodyAwake = BI ? BI->IsInstanceAwake() : false;
				}
			}

			UE_LOG(LogAAATraffic, Log,
				TEXT("LANE PROGRESS: Pawn='%s' Speed=%.1f ClosestIdx=%d/%d "
					 "RemainDist=%.1f DistTraveled=%.1f Threshold=%.1f "
					 "bAtDeadEnd=%s JnctState.bWaiting=%s | "
					 "BodyAwake=%s "
					 "Parked=%s Handbrake=%s "
					 "RPM=%.0f Gear=%d/%d Throttle=%.2f Brake=%.2f Steering=%.2f"),
				*GetPawn()->GetName(), CurSpeed,
				CIdx, LanePoints.Num(),
				RemDist, DistanceTraveledOnLane, ThrshVal,
				bAtDeadEnd ? TEXT("YES") : TEXT("no"),
				JnctState.bWaiting ? TEXT("YES") : TEXT("no"),
				bBodyAwake ? TEXT("YES") : TEXT("NO"),
				bDiagParked ? TEXT("YES") : TEXT("no"),
				bDiagHandbrake ? TEXT("YES") : TEXT("no"),
				DiagRPM, DiagGear, DiagTgtGear,
				DiagThrottle, DiagBrake, DiagSteering);
		}
	}

	if (!bLaneDataReady || !GetPawn())
	{
		// Log once per controller to help diagnose stuck vehicles.
		if (!bDiagLoggedTickSkip)
		{
			bDiagLoggedTickSkip = true;
			UE_LOG(LogAAATraffic, Warning,
				TEXT("VehicleController::Tick: Skipping — bLaneDataReady=%s, Pawn=%s. "
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
	// so DeltaTime was lost — vehicle behavior became frame-rate dependent.
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

	// ── Physics Safety Net ──────────────────────────────────
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
				bPendingRecoveryDespawn = true;
				if (TrafficSub)
				{
					TrafficSub->RequestDespawn(this,
						FString::Printf(TEXT("flipped/airborne for %.1fs (UpZ=%.2f)"),
							FlipTimeAccumulator, UpZ));
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
				bPendingRecoveryDespawn = true;
				if (TrafficSub)
				{
					TrafficSub->RequestDespawn(this,
						FString::Printf(TEXT("stuck for %.1fs (moved %.1f cm)"),
							StuckTimeAccumulator, MovedDist));
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

#if defined(ENABLE_DRAW_DEBUG) && ENABLE_DRAW_DEBUG
	if ((bDebugDraw || GTrafficDebugDraw != 0) && GetPawn())
	{
		const UWorld* DbgWorld = GetWorld();
		const FVector VehicleLoc = GetPawn()->GetActorLocation();
		const FVector VehicleFwd = GetPawn()->GetActorForwardVector();
		const FVector TextBase = VehicleLoc + FVector(0, 0, 180.0f); // Above the car roof

		// --- Color based on state ---
		FColor StateColor = FColor::Green; // CRUISING
		if (DbgStateName == TEXT("WAITING"))          { StateColor = FColor::Red; }
		else if (DbgStateName == TEXT("BRAKING"))     { StateColor = FColor(255, 165, 0); } // Orange
		else if (DbgStateName == TEXT("FOLLOWING"))   { StateColor = FColor::Yellow; }
		else if (DbgStateName == TEXT("IN-JNCT"))     { StateColor = FColor(0, 200, 200); } // Teal
		else if (DbgStateName == TEXT("APPROACH"))    { StateColor = FColor(255, 200, 0); } // Amber

		// --- LINE 1: Vehicle name + state ---
		const FString VehicleName = GetPawn()->GetName();
		// Strip BP_Sedan_child_base_ prefix for readability.
		FString ShortName = VehicleName;
		ShortName.ReplaceInline(TEXT("BP_Sedan_child_base_"), TEXT(""));
		const FString Line1 = FString::Printf(TEXT("%s [%s]"), *ShortName, *DbgStateName);
		DrawDebugString(DbgWorld, TextBase, Line1, nullptr, StateColor, 0.0f, true, 1.2f);

		// --- LINE 2: Speed (mph) + Brake% ---
		const float SpeedMph = FMath::Abs(DbgCurrentSpeed) * 0.0223694f; // cm/s -> mph
		const FString Line2 = FString::Printf(TEXT("%.0f mph  Brake:%.0f%%  Throttle:%.0f%%"),
			SpeedMph, DbgBrake * 100.0f, DbgThrottle * 100.0f);
		DrawDebugString(DbgWorld, TextBase + FVector(0, 0, -20), Line2, nullptr, FColor::White, 0.0f, true, 1.0f);

		// --- LINE 3: Lane + RemainingDist + TransitionThreshold ---
		const FString Line3 = FString::Printf(TEXT("Lane:%d  Rem:%.0f  Thresh:%.0f  StopDist:%.0f"),
			CurrentLane.HandleId, DbgRemainingDist, DbgTransitionThreshold, DbgStoppingDist);
		DrawDebugString(DbgWorld, TextBase + FVector(0, 0, -40), Line3, nullptr, FColor::White, 0.0f, true, 0.9f);

		// --- LINE 4: Intersection-specific info (only when approaching/waiting) ---
		if (DbgDistToEntry >= 0.0f || JnctState.bWaiting)
		{
			const FString Line4 = FString::Printf(TEXT("DistToEntry:%.0f  DesiredStop:%.0f  Jnct:%d"),
				DbgDistToEntry, DbgDesiredStopSpeed, JnctState.JunctionId);
			DrawDebugString(DbgWorld, TextBase + FVector(0, 0, -58), Line4, nullptr,
				FColor::Red, 0.0f, true, 0.9f);
		}

		// --- LINE 4b: Junction Approach Scan info (when scan detected a junction ahead) ---
		if (DbgApproachJunctionDist >= 0.0f)
		{
			const bool bDbgApproaching = (JnctState.Phase == EJunctionPhase::Approaching);
			const float ApproachMph = DbgApproachSpeedLimit * 0.0223694f;
			const FString Line4b = FString::Printf(TEXT("SCAN: Jnct:%d  Dist:%.0f  VLimit:%.0fmph  %s"),
				DbgApproachJunctionId,
				DbgApproachJunctionDist,
				ApproachMph,
				bDbgApproaching ? TEXT("BRAKING") : TEXT("OK"));
			const FColor ScanColor = bDbgApproaching ? FColor(255, 165, 0) : FColor(0, 200, 100);
			DrawDebugString(DbgWorld, TextBase + FVector(0, 0, -94), Line4b, nullptr,
				ScanColor, 0.0f, true, 0.9f);
		}

		// --- LINE 5: Leader info (only when following) ---
		if (DbgLeaderDist >= 0.0f)
		{
			const float LeaderMph = FMath::Abs(DbgLeaderSpeed) * 0.0223694f;
			const FString Line5 = FString::Printf(TEXT("Leader:%.0fcm  LeaderSpeed:%.0fmph  FollowDist:%.0f"),
				DbgLeaderDist, LeaderMph, FollowingDistance);
			DrawDebugString(DbgWorld, TextBase + FVector(0, 0, -76), Line5, nullptr,
				FColor::Yellow, 0.0f, true, 0.9f);

			// Orange line from car to detected leader position.
			const FVector LeaderPos = VehicleLoc + VehicleFwd * DbgLeaderDist;
			DrawDebugLine(DbgWorld, VehicleLoc, LeaderPos, FColor::Orange, false, -1.0f, 0, 2.5f);
			DrawDebugSphere(DbgWorld, LeaderPos, 40.0f, 6, FColor::Orange, false, -1.0f, 0, 2.0f);
		}

		// --- INTERSECTION ENTRY POINT: Red sphere where the car thinks it must stop ---
		if (JnctState.bHasEntryPos)
		{
			DrawDebugSphere(DbgWorld, JnctState.EntryWorldPos, 60.0f, 8, FColor::Red, false, -1.0f, 0, 3.0f);
			// Yellow line from car to entry point — shows actual stopping gap.
			DrawDebugLine(DbgWorld, VehicleLoc, JnctState.EntryWorldPos, FColor::Yellow, false, -1.0f, 0, 2.5f);

			// Blue sphere: where the decel curve predicts the car WILL stop
			// (v^2 / (2*300) cm from current position along forward vector).
			if (DbgStoppingDist > 10.0f)
			{
				const FVector PredictedStop = VehicleLoc + VehicleFwd * DbgStoppingDist;
				DrawDebugSphere(DbgWorld, PredictedStop, 40.0f, 6, FColor::Blue, false, -1.0f, 0, 2.5f);
				DrawDebugLine(DbgWorld, VehicleLoc, PredictedStop, FColor::Blue, false, -1.0f, 0, 1.5f);
			}
		}

		// --- STOPPING DISTANCE vs REMAINING DISTANCE comparison bar ---
		// Visualize whether the car CAN physically stop before lane end.
		// Green bar = can stop. Red bar = cannot stop (will overshoot).
		if (DbgRemainingDist > 0.0f && DbgStoppingDist > 0.0f)
		{
			const bool bCanStop = (DbgStoppingDist <= DbgRemainingDist);
			const FColor BarColor = bCanStop ? FColor::Green : FColor::Red;
			// Draw a line along forward vector showing stopping distance,
			// with the length clamped to remaining distance for comparison.
			const float BarLen = FMath::Min(DbgStoppingDist, DbgRemainingDist + 500.0f);
			const FVector BarEnd = VehicleLoc + VehicleFwd * BarLen;
			DrawDebugLine(DbgWorld, VehicleLoc + FVector(0,0,50), BarEnd + FVector(0,0,50),
				BarColor, false, -1.0f, 0, 4.0f);
		}

		// --- LANE POLYLINE (cyan) ---
		for (int32 i = 0; i < LanePoints.Num() - 1; ++i)
		{
			DrawDebugLine(DbgWorld, LanePoints[i], LanePoints[i + 1], FColor::Cyan, false, -1.0f, 0, 2.0f);
		}

		// --- LOOK-AHEAD TARGET (green sphere + line) ---
		if (!DbgTargetPoint.IsZero())
		{
			DrawDebugSphere(DbgWorld, DbgTargetPoint, 30.0f, 6, FColor::Green, false, -1.0f, 0, 2.0f);
			DrawDebugLine(DbgWorld, VehicleLoc, DbgTargetPoint, FColor::Green, false, -1.0f, 0, 1.5f);
		}

		// --- TRANSITION THRESHOLD RING (white circle at lane end minus threshold) ---
		if (DbgTransitionThreshold > 0.0f && LanePoints.Num() >= 2)
		{
			// Walk backward from lane end to find the threshold position.
			float DistFromEnd = 0.0f;
			FVector ThreshPt = LanePoints.Last();
			for (int32 i = LanePoints.Num() - 1; i > 0; --i)
			{
				const float Seg = FVector::Dist(LanePoints[i], LanePoints[i - 1]);
				if (DistFromEnd + Seg >= DbgTransitionThreshold)
				{
					const float Alpha = (DbgTransitionThreshold - DistFromEnd) / FMath::Max(Seg, 1.0f);
					ThreshPt = FMath::Lerp(LanePoints[i], LanePoints[i - 1], Alpha);
					break;
				}
				DistFromEnd += Seg;
			}
			DrawDebugSphere(DbgWorld, ThreshPt, 50.0f, 8, FColor::White, false, -1.0f, 0, 2.0f);
		}

		// --- APPROACH SCAN: Junction marker (orange diamond at scan distance ahead) ---
		if (DbgApproachJunctionDist > 0.0f)
		{
			const FVector ApproachPt = VehicleLoc + VehicleFwd * FMath::Min(DbgApproachJunctionDist, 10000.0f);
			const FColor ApproachColor = (JnctState.Phase == EJunctionPhase::Approaching) ? FColor(255, 100, 0) : FColor(100, 255, 100);
			DrawDebugSphere(DbgWorld, ApproachPt, 70.0f, 4, ApproachColor, false, -1.0f, 0, 3.0f);
			DrawDebugLine(DbgWorld, VehicleLoc + FVector(0, 0, 70), ApproachPt + FVector(0, 0, 70),
				ApproachColor, false, -1.0f, 0, 2.5f);
		}

		// --- LANE-CHANGE TARGET LANE (magenta) ---
		if (LaneChangeState != ELaneChangeState::None)
		{
			for (int32 i = 0; i < TargetLanePoints.Num() - 1; ++i)
			{
				DrawDebugLine(DbgWorld, TargetLanePoints[i], TargetLanePoints[i + 1], FColor::Magenta, false, -1.0f, 0, 2.0f);
			}
		}
	}
#endif
}

void ATrafficVehicleController::UpdateVehicleInput(float DeltaSeconds)
{
	APawn* ControlledPawn = GetPawn();
	if (!ControlledPawn)
	{
		return;
	}

	UChaosWheeledVehicleMovementComponent* VehicleMovement =
		Cast<UChaosWheeledVehicleMovementComponent>(ControlledPawn->GetMovementComponent());
	if (!VehicleMovement)
	{
		// Log once per controller to avoid spam.
		if (!bDiagLoggedNoMovement)
		{
			bDiagLoggedNoMovement = true;
			UPawnMovementComponent* GenericMC = ControlledPawn->GetMovementComponent();
			UE_LOG(LogAAATraffic, Error,
				TEXT("VehicleController::UpdateVehicleInput: ChaosWheeledVehicleMovementComponent cast FAILED for pawn '%s' (class '%s'). "
					 "GetMovementComponent() returned: %s (class '%s'). Vehicle will be stuck. "
					 "This pawn must have UChaosWheeledVehicleMovementComponent (inherits AWheeledVehiclePawn)."),
				*ControlledPawn->GetName(),
				*ControlledPawn->GetClass()->GetName(),
				GenericMC ? *GenericMC->GetName() : TEXT("NULL"),
				GenericMC ? *GenericMC->GetClass()->GetName() : TEXT("N/A"));
		}
		return;
	}

	// --- Safety net: undo unexpected parking/sleep if detected ---
	// PRIMARY FIX is in OnPossess: we set the parent BP's 'Optimized' variable
	// to false via reflection, which disables the entire Physics optimization
	// → Optimization Unpossessed → CE_StopCar → CE Set Parked(true) chain.
	// These conditional checks remain as a cheap safety net in case a future
	// marketplace pack update re-introduces parking through a different path.
	if (VehicleMovement->IsParked())
	{
		VehicleMovement->SetParked(false);
	}
	if (VehicleMovement->GetHandbrakeInput())
	{
		VehicleMovement->SetHandbrakeInput(false);
	}
	// SetSleeping(false) calls WakeAllEnabledRigidBodies — only invoke when actually sleeping.
	{
		UPrimitiveComponent* WakePrim = VehicleMovement->UpdatedPrimitive;
		if (WakePrim)
		{
			FBodyInstance* WakeBI = WakePrim->GetBodyInstance();
			if (WakeBI && !WakeBI->IsInstanceAwake())
			{
				VehicleMovement->SetSleeping(false);
			}
		}
	}

	const FVector VehicleLocation = ControlledPawn->GetActorLocation();
	const FVector VehicleForward = ControlledPawn->GetActorForwardVector();
	const float CurrentSpeed = VehicleMovement->GetForwardSpeed();

	// ── Compute adaptive distances from time-based parameters ──
	// Each distance = |CurrentSpeed| × TimeSec, clamped to a floor so
	// behaviour remains stable even when the vehicle is nearly stopped.
	const float AbsSpeed = FMath::Abs(CurrentSpeed);
	LookAheadDistance  = FMath::Max(AbsSpeed * LookAheadTimeSec,  MinLookAheadDistanceCm);
	FollowingDistance  = FMath::Max(AbsSpeed * FollowingTimeSec,  MinFollowingDistanceCm);
	DetectionDistance  = FMath::Max(AbsSpeed * DetectionTimeSec,  MinDetectionDistanceCm);

	// Approach-scan derived flag: true when the vehicle needs to decelerate
	// toward a junction entry point. Set in the approach scan block below;
	// consumed by speed-limiting and lane-change inhibitor sections.
	bool bApproachBraking = false;

#if defined(ENABLE_DRAW_DEBUG) && ENABLE_DRAW_DEBUG
	// Initialize debug cache each tick — will be overwritten by decision branches.
	DbgCurrentSpeed = CurrentSpeed;
	DbgStateName = TEXT("CRUISING");
	DbgThrottle = 0.0f;
	DbgBrake = 0.0f;
	DbgSteering = 0.0f;
	DbgDistToEntry = -1.0f;
	DbgDesiredStopSpeed = 0.0f;
	DbgLeaderDist = -1.0f;
	DbgLeaderSpeed = 0.0f;
	DbgRemainingDist = 0.0f;
	DbgTransitionThreshold = 0.0f;
	DbgStoppingDist = (FMath::Abs(CurrentSpeed) * FMath::Abs(CurrentSpeed)) / (2.0f * ApproachDecelCmPerSec2);
#endif

	// Track cumulative distance traveled on this lane to prevent short-lane transition loops.
	DistanceThisTick = 0.0f;
	if (!PreviousVehicleLocation.IsZero())
	{
		DistanceThisTick = FVector::Dist(PreviousVehicleLocation, VehicleLocation);
		DistanceTraveledOnLane += DistanceThisTick;
	}
	PreviousVehicleLocation = VehicleLocation;

	// Tick lane-change cooldown.
	if (LaneChangeCooldownRemaining > 0.0f)
	{
		LaneChangeCooldownRemaining -= DeltaSeconds;
	}

	// Consider a lane change if idle and cooldown expired.
	if (LaneChangeState == ELaneChangeState::None && LaneChangeCooldownRemaining <= 0.0f)
	{
		EvaluateLaneChange();
	}

	// Find where we are on the lane and where to aim
	const int32 ClosestIndex = FindClosestPointIndex(VehicleLocation);

	// ── PERIODIC JUNCTION DIAGNOSTIC DUMP (every ~2s per vehicle) ──
	// Provides a full status snapshot so you can see at a glance which vehicles
	// are aware of junctions and which are "blind."
	if (GTrafficJunctionDiagnostics >= 1)
	{
		DiagJunctionTimer += DeltaSeconds;
		if (DiagJunctionTimer >= 2.0f)
		{
			DiagJunctionTimer = 0.0f;
			const float RemDist = GetRemainingDistance(ClosestIndex);
			const float AbsSp = FMath::Abs(CurrentSpeed);
			const float StopD = (AbsSp * AbsSp) / (2.0f * FMath::Max(ApproachDecelCmPerSec2, 1.0f));
			UE_LOG(LogAAATraffic, Warning,
				TEXT("JDIAG STATUS: Pawn='%s' Lane=%d LanePts=%d Speed=%.0f "
					 "RemDist=%.0f StopDist=%.0f DistTraveled=%.0f "
					 "bDeadEnd=%s Provider=%s "
					 "bApproaching=%s ApproachDist=%.0f ApproachJnct=%d ApproachLimit=%.0f "
					 "IntersectionId=%d bWaiting=%s WaitElapsed=%.1f "
					 "JunctionTransPts=%d"),
				GetPawn() ? *GetPawn()->GetName() : TEXT("NULL"),
				CurrentLane.HandleId,
				LanePoints.Num(),
				CurrentSpeed,
				RemDist,
				StopD,
				DistanceTraveledOnLane,
				bAtDeadEnd ? TEXT("YES") : TEXT("NO"),
				CachedProvider ? TEXT("OK") : TEXT("NULL!!!"),
				bApproachBraking ? TEXT("YES") : TEXT("NO"),
				JnctState.ApproachDistanceCm,
				JnctState.JunctionId,
				JnctState.ApproachSpeedLimitCmPerSec,
				JnctState.JunctionId,
				JnctState.bWaiting ? TEXT("YES") : TEXT("NO"),
				JnctState.WaitElapsed,
				JnctState.TransitionPoints.Num());
		}
	}

	// ────────────────────────────────────────────────────────────────
	// JUNCTION APPROACH — Uses the full precomputed road/junction map.
	// The provider pre-built distance-to-next-junction for EVERY lane
	// at world startup, so this is an O(1) lookup — the vehicle has
	// instant knowledge of the entire road network layout.
	// ────────────────────────────────────────────────────────────────
	if (!bAtDeadEnd && CachedProvider && JnctState.Phase <= EJunctionPhase::Approaching)
	{
		const float RemainingOnCurrent = GetRemainingDistance(ClosestIndex);
		const ITrafficRoadProvider::FJunctionScanResult ScanResult =
			CachedProvider->GetDistanceToNextJunction(
				CurrentLane,
				RemainingOnCurrent,
				JunctionScanMaxDistanceCm,
				MaxJunctionScanHops);

		if (GTrafficJunctionDiagnostics >= 2)
		{
			UE_LOG(LogAAATraffic, Warning,
				TEXT("JDIAG SCAN: Pawn='%s' Lane=%d Remaining=%.1f ScanValid=%s "
					 "JunctionId=%d Dist=%.1f JunctionLane=%d ApproachLane=%d"),
				GetPawn() ? *GetPawn()->GetName() : TEXT("NULL"),
				CurrentLane.HandleId,
				RemainingOnCurrent,
				ScanResult.IsValid() ? TEXT("YES") : TEXT("NO"),
				ScanResult.JunctionId,
				ScanResult.DistanceCm,
				ScanResult.JunctionLane.HandleId,
				ScanResult.ApproachLane.HandleId);
		}

		if (ScanResult.IsValid())
		{
			// Transition Idle → Approaching on first detection.
			if (JnctState.Phase == EJunctionPhase::Idle)
			{
				JnctState.BeginApproach(ScanResult.JunctionId);
			}
			JnctState.ApproachDistanceCm = ScanResult.DistanceCm;
			JnctState.ApproachJunctionLane = ScanResult.JunctionLane;

			// Compute the approach speed envelope:
			//   v_approach = sqrt(2 * decel * distToJunction)
			// This is the maximum speed at which the vehicle can still stop
			// before the junction using comfort deceleration.
			// Subtract VehicleFrontExtent so the front bumper (not actor
			// origin) is the reference — matches the decel-curve offset
			// in the JnctState.bWaiting braking code.
			const float SafeDist = FMath::Max(JnctState.ApproachDistanceCm - ApproachSafetyMarginCm - VehicleFrontExtent, 0.0f);
			JnctState.ApproachSpeedLimitCmPerSec = FMath::Sqrt(
				FMath::Max(2.0f * ApproachDecelCmPerSec2 * SafeDist, 0.0f));

			// Only activate approach slowdown when the current speed would
			// require braking — avoids unnecessary throttle limiting when
			// the vehicle is already going slow enough.
			const float AbsSpeedNow = FMath::Abs(CurrentSpeed);
			bApproachBraking = (AbsSpeedNow > JnctState.ApproachSpeedLimitCmPerSec + 50.0f) ||
									   (JnctState.ApproachDistanceCm < LookAheadDistance * 3.0f);

#if defined(ENABLE_DRAW_DEBUG) && ENABLE_DRAW_DEBUG
			DbgApproachJunctionDist = JnctState.ApproachDistanceCm;
			DbgApproachSpeedLimit = JnctState.ApproachSpeedLimitCmPerSec;
			DbgApproachJunctionId = JnctState.JunctionId;
#endif

			// --- Lane pre-positioning for upcoming turn ---
			// If the vehicle is far enough from the junction and has not yet
			// started a navigational lane change, pre-select the exit direction
			// and move toward the appropriate side lane.
			//
			// Dynamic pre-position distance: based on current speed and the
			// physical distance needed to execute a lane change + stop.
			// This adapts automatically to all road types:
			//   - Short urban blocks at 25 mph → ~30 m threshold
			//   - Highway at 65 mph → ~120 m threshold
			const float PrePosStopDist = (AbsSpeed * AbsSpeed) / (2.0f * FMath::Max(ApproachDecelCmPerSec2, 1.0f));
			const float DynamicPrePositionDist = PrePosStopDist + LaneChangeDistance * 2.0f + ApproachSafetyMarginCm;
			if (!bNavigationalLaneChange
				&& JnctState.ApproachDistanceCm > DynamicPrePositionDist
				&& LaneChangeState == ELaneChangeState::None
				&& JnctState.ApproachJunctionLane.IsValid())
			{
				// Use centralized surveyor to pick exit lane.
				TArray<FTrafficLaneHandle> FallbackExits;
				if (JnctState.JunctionId > 0)
				{
					TArray<FTrafficLaneHandle> AllJL = CachedProvider->GetLanesForJunction(JnctState.JunctionId);
					for (const FTrafficLaneHandle& JL : AllJL)
					{
						for (const FTrafficLaneHandle& E : CachedProvider->GetConnectedLanes(JL))
						{
							if (CachedProvider->GetJunctionForLane(E) == 0)
							{
								bool bDup = false;
								for (const FTrafficLaneHandle& X : FallbackExits)
								{
									if (X.HandleId == E.HandleId) { bDup = true; break; }
								}
								if (!bDup) { FallbackExits.Add(E); }
							}
						}
					}
				}
				if (FallbackExits.IsEmpty())
				{
					FallbackExits = CachedProvider->GetConnectedLanes(JnctState.ApproachJunctionLane);
				}

				const FTrafficLaneHandle ChosenExit = PickSurveyedExit(CurrentLane, FallbackExits);

				if (ChosenExit.IsValid())
				{
					// Determine turn direction.
					const ETurnSignalState TurnDir = ComputeTurnDirection(CurrentLane, ChosenExit);

					// Early turn signal activation.
					if (TurnDir != ETurnSignalState::Off)
					{
						SetTurnSignal(TurnDir);
					}

					// Map turn direction to target lane for pre-positioning.
					FTrafficLaneHandle TargetSideLane = FTrafficLaneHandle();
					const FVector PrePosRefDir = CachedProvider->GetLaneDirection(CurrentLane);
					if (TurnDir == ETurnSignalState::Right)
					{
						FTrafficLaneHandle Walk = CachedProvider->GetAdjacentLane(CurrentLane, ETrafficLaneSide::Right);
						for (int32 Safety = 0; Safety < 8 && Walk.IsValid(); ++Safety)
						{
							const FVector WalkDir = CachedProvider->GetLaneDirection(Walk);
							if (FVector::DotProduct(PrePosRefDir, WalkDir) < 0.5f) { break; }
							TargetSideLane = Walk;
							Walk = CachedProvider->GetAdjacentLane(Walk, ETrafficLaneSide::Right);
						}
					}
					else if (TurnDir == ETurnSignalState::Left)
					{
						FTrafficLaneHandle Walk = CachedProvider->GetAdjacentLane(CurrentLane, ETrafficLaneSide::Left);
						for (int32 Safety = 0; Safety < 8 && Walk.IsValid(); ++Safety)
						{
							const FVector WalkDir = CachedProvider->GetLaneDirection(Walk);
							if (FVector::DotProduct(PrePosRefDir, WalkDir) < 0.5f) { break; }
							TargetSideLane = Walk;
							Walk = CachedProvider->GetAdjacentLane(Walk, ETrafficLaneSide::Left);
						}
					}

					if (TargetSideLane.IsValid() && TargetSideLane != CurrentLane)
					{
						NavigationalTargetLane = TargetSideLane;
						bNavigationalLaneChange = true;
						UE_LOG(LogAAATraffic, Log,
							TEXT("TrafficVehicleController: Pre-positioning %s from lane %d → lane %d "
								 "(turn=%s, junction dist=%.0f cm)."),
							GetPawn() ? *GetPawn()->GetName() : TEXT("NULL"),
							CurrentLane.HandleId, TargetSideLane.HandleId,
							TurnDir == ETurnSignalState::Left ? TEXT("LEFT") : TEXT("RIGHT"),
							JnctState.ApproachDistanceCm);
					}
				}
			}
		}
		else
		{
			// Scan returned invalid — no junction downstream for this lane.
			if (GTrafficJunctionDiagnostics >= 1)
			{
				UE_LOG(LogAAATraffic, Warning,
					TEXT("JDIAG SCAN-INVALID: Pawn='%s' Lane=%d — GetDistanceToNextJunction "
						 "returned INVALID (no junction downstream). Vehicle will NOT slow for any intersection."),
					GetPawn() ? *GetPawn()->GetName() : TEXT("NULL"),
					CurrentLane.HandleId);
			}
			bApproachBraking = false;
			if (JnctState.Phase == EJunctionPhase::Approaching)
			{
				JnctState.Reset();
			}
#if defined(ENABLE_DRAW_DEBUG) && ENABLE_DRAW_DEBUG
			DbgApproachJunctionDist = -1.0f;
			DbgApproachSpeedLimit = 0.0f;
			DbgApproachJunctionId = 0;
#endif
		}
	}
	else if (JnctState.Phase >= EJunctionPhase::Waiting)
	{
		// Already handling an intersection — disable approach scan.
		bApproachBraking = false;
		if (GTrafficJunctionDiagnostics >= 2)
		{
			UE_LOG(LogAAATraffic, Warning,
				TEXT("JDIAG SCAN-SKIP-HANDLING: Pawn='%s' Lane=%d — approach scan disabled "
					 "(already handling intersection: JunctionId=%d bWaiting=%s)"),
				GetPawn() ? *GetPawn()->GetName() : TEXT("NULL"),
				CurrentLane.HandleId,
				JnctState.JunctionId,
				JnctState.bWaiting ? TEXT("YES") : TEXT("NO"));
		}
#if defined(ENABLE_DRAW_DEBUG) && ENABLE_DRAW_DEBUG
		DbgApproachJunctionDist = -1.0f;
		DbgApproachSpeedLimit = 0.0f;
		DbgApproachJunctionId = 0;
#endif
	}
	else
	{
		// Guard blocked approach scan for a reason OTHER than handling intersection.
		if (GTrafficJunctionDiagnostics >= 1)
		{
			UE_LOG(LogAAATraffic, Warning,
				TEXT("JDIAG SCAN-BLOCKED: Pawn='%s' Lane=%d — approach scan guard FAILED. "
					 "bAtDeadEnd=%s CachedProvider=%s bWaiting=%s JnctState.JunctionId=%d. "
					 "THIS VEHICLE CANNOT SEE JUNCTIONS."),
				GetPawn() ? *GetPawn()->GetName() : TEXT("NULL"),
				CurrentLane.HandleId,
				bAtDeadEnd ? TEXT("YES") : TEXT("NO"),
				CachedProvider ? TEXT("VALID") : TEXT("NULL!!!"),
				JnctState.bWaiting ? TEXT("YES") : TEXT("NO"),
				JnctState.JunctionId);
		}
	}

	// --- Lane-end detection + intersection right-of-way ---
	// Allow entry even during an active lane change: if we're approaching a
	// lane end, the intersection check is more important than the lane change.
	// The lane change will be aborted below if an intersection is detected.
	if (!bAtDeadEnd)
	{
		const float RemainingDist = GetRemainingDistance(ClosestIndex);
		// Use physics-based stopping-distance threshold so fast vehicles get
		// enough advance warning to actually stop before the intersection.
		// Old formula: max(LookAhead, |Speed| * 1.0s) — only 1s of reaction time.
		// New formula: max(LookAhead, v²/(2*decel) + margin) — matches actual braking physics.
		// AbsSpeed already computed at the top of UpdateVehicleInput.
		const float StoppingDist = (AbsSpeed * AbsSpeed) / (2.0f * ApproachDecelCmPerSec2);
		const float TransitionThreshold = FMath::Max(LookAheadDistance, StoppingDist + ApproachSafetyMarginCm);
#if defined(ENABLE_DRAW_DEBUG) && ENABLE_DRAW_DEBUG
		DbgRemainingDist = RemainingDist;
		DbgTransitionThreshold = TransitionThreshold;
#endif
		// MinTransitionGuard: small constant to prevent re-triggering on the
		// same frame, but low enough that short virtual segments (created by
		// through-road splitting) still trigger intersection detection.
		constexpr float MinTransitionGuard = 50.0f; // cm — ~1 polyline sample

		// DIAGNOSTIC: Log why the detection gate doesn't fire.
		const bool bLaneEndClose = (RemainingDist < TransitionThreshold);
		const bool bApproachScanClose = (JnctState.ApproachDistanceCm > 0.0f
			&& JnctState.ApproachDistanceCm < TransitionThreshold
			&& JnctState.JunctionId != 0);

		if (GTrafficJunctionDiagnostics >= 1 && !((bLaneEndClose || bApproachScanClose) && DistanceTraveledOnLane > MinTransitionGuard))
		{
			// Only log once per second per vehicle to avoid total log flood.
			static TMap<uint32, double> LastGateLogTime;
			static TWeakObjectPtr<UWorld> LastGateLogWorld;
			// Clear on new PIE session so diagnostics re-fire for fresh play.
			UWorld* CurrentWorld = GetWorld();
			if (!LastGateLogWorld.IsValid() || LastGateLogWorld.Get() != CurrentWorld)
			{
				LastGateLogTime.Empty();
				LastGateLogWorld = CurrentWorld;
			}
			const uint32 VehicleId = GetUniqueID();
			const double Now = FPlatformTime::Seconds();
			double& LastTime = LastGateLogTime.FindOrAdd(VehicleId, 0.0);
			if (Now - LastTime > 1.0)
			{
				LastTime = Now;
				UE_LOG(LogAAATraffic, Warning,
					TEXT("JDIAG GATE-NOT-FIRING: Pawn='%s' Lane=%d RemainingDist=%.1f "
						 "TransitionThreshold=%.1f DistTraveled=%.1f MinGuard=%.1f "
						 "Speed=%.1f bApproaching=%s ApproachDist=%.1f ApproachJnct=%d "
						 "— Vehicle is NOT close enough to lane end for junction detection."),
					GetPawn() ? *GetPawn()->GetName() : TEXT("NULL"),
					CurrentLane.HandleId,
					RemainingDist,
					TransitionThreshold,
					DistanceTraveledOnLane,
					MinTransitionGuard,
					CurrentSpeed,
					bApproachBraking ? TEXT("YES") : TEXT("NO"),
					JnctState.ApproachDistanceCm,
					JnctState.JunctionId);
			}
		}

		if ((bLaneEndClose || bApproachScanClose) && DistanceTraveledOnLane > MinTransitionGuard)
		{
			// --- Intersection approach ---
			// Detect junctions via two methods:
			//   1. Current lane IS a junction lane (GetJunctionForLane(CurrentLane) != 0)
			//   2. Look-ahead: current lane is free-flow but NEXT lane is a junction
			//      (through-road split — free-flow segment preceding intersection)
			if (JnctState.Phase < EJunctionPhase::Waiting && CachedProvider)
			{
				int32 DetectedJunctionId = CachedProvider->GetJunctionForLane(CurrentLane);
				// Track which lane handle to pass to IsLaneGreen — must be
				// the lane that appears in the signal's PhaseGroup.GreenLanes.
				FTrafficLaneHandle DetectedJunctionLane = CurrentLane;

				if (GTrafficJunctionDiagnostics >= 1)
				{
					UE_LOG(LogAAATraffic, Log,
						TEXT("JNCT DETECT: Pawn='%s' CurrentLane=%d GetJunctionForLane(current)=%d "
							 "RemainingDist=%.1f TransitionThreshold=%.1f Speed=%.1f"),
						GetPawn() ? *GetPawn()->GetName() : TEXT("NULL"),
						CurrentLane.HandleId, DetectedJunctionId,
						RemainingDist, TransitionThreshold, CurrentSpeed);
				}

				// --- Look-ahead: peek at next lanes for junction presence ---
				if (DetectedJunctionId == 0)
				{
					TArray<FTrafficLaneHandle> NextLanes = CachedProvider->GetConnectedLanes(CurrentLane);
					for (const FTrafficLaneHandle& NextLane : NextLanes)
					{
						const int32 NextJId = CachedProvider->GetJunctionForLane(NextLane);
						if (GTrafficJunctionDiagnostics >= 1)
						{
							UE_LOG(LogAAATraffic, Log,
								TEXT("JNCT LOOKAHEAD: Pawn='%s' CurrentLane=%d NextLane=%d "
									 "GetJunctionForLane(next)=%d"),
								GetPawn() ? *GetPawn()->GetName() : TEXT("NULL"),
								CurrentLane.HandleId, NextLane.HandleId, NextJId);
						}
						if (NextJId != 0)
						{
							DetectedJunctionId = NextJId;
							DetectedJunctionLane = NextLane; // Use the junction lane for signal queries.
							break; // Any matching junction suffices.
						}
					}
				}

				// --- Extended look-ahead (precomputed map fallback) ---
				// The 1-hop LOOKAHEAD above only finds junctions on the immediately
				// connected lane. For lanes that are 2+ hops from a junction lane
				// (e.g., lane 20 → 21 → 22(J1)), it misses the junction entirely.
				//
				// The approach scan already walked the full precomputed graph and
				// stored the result in JnctState.JunctionId / JnctState.ApproachJunctionLane.
				// Use that data as a fallback so these vehicles still check
				// signals and occupancy before entering the junction path.
				// IMPORTANT: Only use this fallback when the junction is actually
				// within braking distance. On loop roads the scan always finds
				// the junction (because connectivity wraps), so without this
				// distance gate every lane-end falsely triggers junction waiting.
				if (DetectedJunctionId == 0 && JnctState.Phase == EJunctionPhase::Approaching
					&& JnctState.ApproachDistanceCm > 0.0f
					&& JnctState.ApproachDistanceCm < TransitionThreshold)
				{
					DetectedJunctionId = JnctState.JunctionId;
					DetectedJunctionLane = JnctState.ApproachJunctionLane;
					if (GTrafficJunctionDiagnostics >= 1)
					{
						UE_LOG(LogAAATraffic, Log,
							TEXT("JNCT PRECOMPUTED-HIT: Pawn='%s' CurrentLane=%d — 1-hop LOOKAHEAD "
								 "missed, using precomputed map: JunctionId=%d Dist=%.1f JnctLane=%d"),
							GetPawn() ? *GetPawn()->GetName() : TEXT("NULL"),
							CurrentLane.HandleId,
							JnctState.JunctionId,
							JnctState.ApproachDistanceCm,
							JnctState.ApproachJunctionLane.HandleId);
					}
				}

				// Skip re-acquisition when we already hold this junction's occupancy.
				// This prevents re-entering the signal/occupy flow when the vehicle
				// transitions to a new lane still inside the same junction.
				if (DetectedJunctionId != 0 && JnctState.IsEngaged() && DetectedJunctionId == JnctState.JunctionId)
				{
					if (GTrafficJunctionDiagnostics >= 1)
					{
						UE_LOG(LogAAATraffic, Log,
							TEXT("JNCT SKIP-REDETECT: Pawn='%s' JunctionId=%d — already holds "
								 "occupancy, skipping signal check, proceeding to CheckLaneTransition"),
							GetPawn() ? *GetPawn()->GetName() : TEXT("NULL"),
							DetectedJunctionId);
					}
				}
				else if (DetectedJunctionId != 0 && DetectedJunctionId == JnctState.LastReleasedId)
				{
					// Junction was just released on this lane (curve-complete fired
					// before lane-end). Don't re-acquire — the vehicle is exiting.
					if (GTrafficJunctionDiagnostics >= 1)
					{
						UE_LOG(LogAAATraffic, Log,
							TEXT("JNCT SKIP-RELEASED: Pawn='%s' JunctionId=%d — junction was "
								 "just released on this lane, skipping re-acquisition"),
							GetPawn() ? *GetPawn()->GetName() : TEXT("NULL"),
							DetectedJunctionId);
					}
				}
				else if (DetectedJunctionId != 0)
				{
					// Abort any active lane change — intersection takes priority.
					if (LaneChangeState != ELaneChangeState::None)
					{
						if (GTrafficJunctionDiagnostics >= 1)
						{
							UE_LOG(LogAAATraffic, Log,
								TEXT("JNCT LANE-CHANGE-ABORT: Pawn='%s' JunctionId=%d — "
									 "aborting lane change (state=%d) to handle intersection"),
								GetPawn() ? *GetPawn()->GetName() : TEXT("NULL"),
								DetectedJunctionId, static_cast<int32>(LaneChangeState));
						}
						LaneChangeState = ELaneChangeState::None;
						LaneChangeProgress = 0.0f;
						LaneChangeSettleTimer = 0.0f;
						TargetLanePoints.Empty();
					}

					// Transition to Waiting phase (from Idle or Approaching).
					// JunctionId may already match if approaching; BeginWaiting validates the transition.
					if (JnctState.Phase == EJunctionPhase::Idle)
					{
						JnctState.BeginApproach(DetectedJunctionId);
					}
					JnctState.JunctionLane = DetectedJunctionLane;

					// Resolve exact intersection entry point for brake targeting.
					if (!JnctState.bHasEntryPos)
					{
						FVector EntryPt;
						if (CachedProvider->GetIntersectionEntryPoint(CurrentLane, EntryPt))
						{
							JnctState.EntryWorldPos = EntryPt;
							JnctState.bHasEntryPos = true;
							UE_LOG(LogAAATraffic, Warning,
								TEXT("JNCT ENTRYPOINT: Pawn='%s' Lane=%d EntryPt=(%.1f,%.1f,%.1f) Source=ExactMask"),
								GetPawn() ? *GetPawn()->GetName() : TEXT("NULL"),
								CurrentLane.HandleId, EntryPt.X, EntryPt.Y, EntryPt.Z);
						}
						else
						{
							// Fallback: prefer junction centroid projected onto approach
							// direction over raw LanePoints.Last(), which can extend
							// into the junction area for non-split roads.
							FVector Centroid;
							if (DetectedJunctionId != 0
								&& CachedProvider->GetJunctionCentroid(DetectedJunctionId, Centroid)
								&& LanePoints.Num() >= 2)
							{
								// Project centroid onto the last lane segment to get the
								// point where the lane intersects the junction boundary.
								const FVector& P0 = LanePoints[LanePoints.Num() - 2];
								const FVector& P1 = LanePoints.Last();
								const FVector SegDir = (P1 - P0).GetSafeNormal();
								const float Proj = FVector::DotProduct(Centroid - P0, SegDir);
								const float SegLen = FVector::Dist(P0, P1);
								// Clamp to segment — entry can't be before P0 or after P1.
								const float ClampedProj = FMath::Clamp(Proj, 0.0f, SegLen);
								JnctState.EntryWorldPos = P0 + SegDir * ClampedProj;
								JnctState.bHasEntryPos = true;
								UE_LOG(LogAAATraffic, Warning,
									TEXT("JNCT ENTRYPOINT: Pawn='%s' Lane=%d EntryPt=(%.1f,%.1f,%.1f) Source=CentroidProjection"),
									GetPawn() ? *GetPawn()->GetName() : TEXT("NULL"),
									CurrentLane.HandleId, JnctState.EntryWorldPos.X, JnctState.EntryWorldPos.Y, JnctState.EntryWorldPos.Z);
							}
							else
							{
								// Ultimate fallback: last polyline point.
								JnctState.EntryWorldPos = LanePoints.Last();
								JnctState.bHasEntryPos = true;
								UE_LOG(LogAAATraffic, Warning,
									TEXT("JNCT ENTRYPOINT: Pawn='%s' Lane=%d EntryPt=(%.1f,%.1f,%.1f) Source=FallbackLast"),
									GetPawn() ? *GetPawn()->GetName() : TEXT("NULL"),
									CurrentLane.HandleId, LanePoints.Last().X, LanePoints.Last().Y, LanePoints.Last().Z);
							}
						}
					}

					UWorld* World = GetWorld();
					UTrafficSubsystem* TrafficSub = World ? World->GetSubsystem<UTrafficSubsystem>() : nullptr;
					if (TrafficSub)
					{
						// Check for signal — vehicles stop on Red.
						// On Yellow: apply "dilemma zone" logic — if the vehicle
						// cannot safely stop before the entry point, commit through.
						ATrafficSignalController* Signal = TrafficSub->GetSignalForJunction(DetectedJunctionId);
						bool bSignalAllows = true;
						if (Signal)
						{
							bSignalAllows = Signal->IsLaneGreen(JnctState.JunctionLane);

							// Yellow dilemma zone: IsLaneGreen returns false for
							// Yellow, but a vehicle too close to stop comfortably
							// should commit through rather than slam the brakes.
							if (!bSignalAllows
								&& Signal->GetControlMode() == EJunctionControlMode::Signal
								&& Signal->GetCurrentPhase() == ETrafficSignalPhase::Yellow)
							{
								// Stopping distance from current speed:
								// d_stop = v² / (2 × comfortable_decel)
								const float StopDist = (AbsSpeed * AbsSpeed)
									/ (2.0f * FMath::Max(ApproachDecelCmPerSec2, 1.0f));
								const float DistToJunction = JnctState.bHasEntryPos
									? FVector::Dist(VehicleLocation, JnctState.EntryWorldPos)
									: RemainingDist;
								if (StopDist > DistToJunction)
								{
									// Can't stop in time — commit through Yellow.
									bSignalAllows = true;
									UE_LOG(LogAAATraffic, Log,
										TEXT("SIGNAL YELLOW-COMMIT: Pawn='%s' JunctionId=%d "
											 "StopDist=%.0f > DistToJunction=%.0f — committing through Yellow"),
										GetPawn() ? *GetPawn()->GetName() : TEXT("NULL"),
										DetectedJunctionId, StopDist, DistToJunction);
								}
							}
						}

						// Compute exit lane for the 4-arg conflict-detection occupy call.
						// Pre-select the exit using weighted random so conflict
						// detection and actual driving use the SAME exit lane.
						//
						// Use centralized surveyor for exit selection.
						JnctState.FromLane = CurrentLane;
						JnctState.ToLane = FTrafficLaneHandle();
						{
							TArray<FTrafficLaneHandle> FallbackExits;
							TArray<FTrafficLaneHandle> AllJunctionLanes = CachedProvider->GetLanesForJunction(DetectedJunctionId);
							for (const FTrafficLaneHandle& JL : AllJunctionLanes)
							{
								TArray<FTrafficLaneHandle> Exits = CachedProvider->GetConnectedLanes(JL);
								for (const FTrafficLaneHandle& E : Exits)
								{
									if (CachedProvider->GetJunctionForLane(E) == 0)
									{
										bool bAlready = false;
										for (const FTrafficLaneHandle& Existing : FallbackExits)
										{
											if (Existing.HandleId == E.HandleId) { bAlready = true; break; }
										}
										if (!bAlready) { FallbackExits.Add(E); }
									}
								}
							}
							if (FallbackExits.IsEmpty())
							{
								FallbackExits = CachedProvider->GetConnectedLanes(DetectedJunctionLane);
							}

							JnctState.ToLane = PickSurveyedExit(CurrentLane, FallbackExits);

							if (JnctState.ToLane.IsValid())
							{
								UE_LOG(LogAAATraffic, Warning,
									TEXT("JNCT EXIT-PICKED: Pawn='%s' JunctionId=%d Picked=%d (via surveyor)"),
									GetPawn() ? *GetPawn()->GetName() : TEXT("NULL"),
									DetectedJunctionId, JnctState.ToLane.HandleId);
							}
						}

						// ── Turn-aware exit lane matching ──────────────────────────
						// Left turns should target the innermost (leftmost) exit
						// lane; right turns should target the outermost (rightmost).
						// Straight-through mirrors the approach lateral position.
						//
						// IMPORTANT: adjacency maps cross the centre line on 2-way
						// roads (lanes share a boundary curve). Each adjacency step
						// must validate that the neighbor faces the same direction
						// as the original exit; stop immediately if it doesn't.
						if (CachedProvider && JnctState.ToLane.IsValid())
						{
							const ETurnSignalState TurnDir = ComputeTurnDirection(JnctState.FromLane, JnctState.ToLane);
							if (TurnDir == ETurnSignalState::Left || TurnDir == ETurnSignalState::Right)
							{
								const FVector ExitRefDir = CachedProvider->GetLaneDirection(JnctState.ToLane);

								// Walk to the rightmost same-direction exit lane.
								FTrafficLaneHandle RightmostExit = JnctState.ToLane;
								{
									FTrafficLaneHandle Walk = CachedProvider->GetAdjacentLane(JnctState.ToLane, ETrafficLaneSide::Right);
									for (int32 S = 0; S < 8 && Walk.IsValid(); ++S)
									{
										const FVector WalkDir = CachedProvider->GetLaneDirection(Walk);
										if (FVector::DotProduct(ExitRefDir, WalkDir) < 0.5f) { break; }
										RightmostExit = Walk;
										Walk = CachedProvider->GetAdjacentLane(Walk, ETrafficLaneSide::Right);
									}
								}
								// Walk to the leftmost same-direction exit lane.
								FTrafficLaneHandle LeftmostExit = JnctState.ToLane;
								{
									FTrafficLaneHandle Walk = CachedProvider->GetAdjacentLane(JnctState.ToLane, ETrafficLaneSide::Left);
									for (int32 S = 0; S < 8 && Walk.IsValid(); ++S)
									{
										const FVector WalkDir = CachedProvider->GetLaneDirection(Walk);
										if (FVector::DotProduct(ExitRefDir, WalkDir) < 0.5f) { break; }
										LeftmostExit = Walk;
										Walk = CachedProvider->GetAdjacentLane(Walk, ETrafficLaneSide::Left);
									}
								}

								const FTrafficLaneHandle TargetLane = (TurnDir == ETurnSignalState::Left)
									? LeftmostExit   // Left turn → innermost (nearest to centre)
									: RightmostExit; // Right turn → outermost (kerb-side)
								if (TargetLane.HandleId != JnctState.ToLane.HandleId)
								{
									UE_LOG(LogAAATraffic, Log,
										TEXT("JNCT TURN-LANE-MATCH: Pawn='%s' Turn=%s "
											 "Exit %d -> %d (matched to %s lane)"),
										GetPawn() ? *GetPawn()->GetName() : TEXT("NULL"),
										TurnDir == ETurnSignalState::Left ? TEXT("LEFT") : TEXT("RIGHT"),
										JnctState.ToLane.HandleId, TargetLane.HandleId,
										TurnDir == ETurnSignalState::Left ? TEXT("innermost") : TEXT("outermost"));
									JnctState.ToLane = TargetLane;
								}
							}
						}

						// Activate turn signal based on approach → exit direction.
						SetTurnSignal(ComputeTurnDirection(JnctState.FromLane, JnctState.ToLane));

						UE_LOG(LogAAATraffic, Warning,
							TEXT("JNCT OCCUPY-ATTEMPT: Pawn='%s' JunctionId=%d HasSignal=%s "
								 "SignalAllowsGreen=%s DesiredNext=%d ExitLane=%d ControlMode=%d — about to TryOccupyJunction"),
							GetPawn() ? *GetPawn()->GetName() : TEXT("NULL"),
							DetectedJunctionId,
							Signal ? TEXT("YES") : TEXT("NO"),
							bSignalAllows ? TEXT("YES") : TEXT("NO"),
							DetectedJunctionLane.HandleId,
							JnctState.ToLane.HandleId,
							Signal ? static_cast<int32>(Signal->GetControlMode()) : -1);

						// Determine junction control mode.
						const EJunctionControlMode JunctionMode = Signal
							? Signal->GetControlMode()
							: EJunctionControlMode::Yield; // No controller = uncontrolled (yield-like).

						if (JunctionMode == EJunctionControlMode::Yield)
						{
							// Yield sign: check all conflict sources before proceeding.
							// 1) Left-turn yield to oncoming straight-through.
							// 2) Cross-traffic gap acceptance (all movements).
							bool bMustYield = false;
							{
								const ETurnSignalState YieldTurnDir = ComputeTurnDirection(JnctState.FromLane, JnctState.ToLane);
								if (YieldTurnDir == ETurnSignalState::Left
									&& TrafficSub->HasConflictingApproach(DetectedJunctionId, this, JnctState.FromLane, JnctState.ToLane))
								{
									bMustYield = true;
								}
							}
							if (!bMustYield)
							{
								bMustYield = TrafficSub->HasApproachingCrossTraffic(
									DetectedJunctionId, this, JnctState.FromLane, JnctState.ToLane);
							}

							// Yield: try to occupy immediately. If clear, proceed without stopping.
							if (!bMustYield && TrafficSub->TryOccupyJunction(DetectedJunctionId, this, JnctState.FromLane, JnctState.ToLane))
							{
								JnctState.BeginTraversing();
								UE_LOG(LogAAATraffic, Warning,
									TEXT("JNCT YIELD-PROCEED: Pawn='%s' JunctionId=%d — clear, proceeding without stop"),
									GetPawn() ? *GetPawn()->GetName() : TEXT("NULL"),
									DetectedJunctionId);
							}
							else
							{
								// Not clear — must wait (will slow down via approach braking).
								JnctState.BeginWaiting();
								JnctState.bStopSignWaitComplete = true; // No mandatory stop for yield.
								JnctState.StopSignStopElapsed = 0.0f;
								UE_LOG(LogAAATraffic, Warning,
									TEXT("JNCT YIELD-WAIT: Pawn='%s' JunctionId=%d — %s, waiting for clearance"),
									GetPawn() ? *GetPawn()->GetName() : TEXT("NULL"),
									DetectedJunctionId,
									bMustYield ? TEXT("yield/cross-traffic") : TEXT("occupied"));
							}
						}
						else if (JunctionMode == EJunctionControlMode::StopSign || JunctionMode == EJunctionControlMode::FlashingRed)
						{
							// StopSign/FlashingRed: always wait — vehicle must come to a full stop first.
							JnctState.BeginWaiting();
							JnctState.bStopSignWaitComplete = false;
							JnctState.StopSignStopElapsed = 0.0f;
							UE_LOG(LogAAATraffic, Warning,
								TEXT("JNCT STOPSIGN-WAIT: Pawn='%s' JunctionId=%d — must stop and wait %.1fs"),
								GetPawn() ? *GetPawn()->GetName() : TEXT("NULL"),
								DetectedJunctionId,
								Signal ? Signal->StopSignWaitTimeSec : 2.0f);
						}
						else // Signal mode
						{
						// Left-turn yield: if making a left turn, yield to
						// oncoming straight-through traffic (standard traffic
						// law — left-turners have lowest priority on green).
						bool bLeftTurnYield = false;
						if (bSignalAllows)
						{
							const ETurnSignalState OccupyTurnDir = ComputeTurnDirection(JnctState.FromLane, JnctState.ToLane);
							if (OccupyTurnDir == ETurnSignalState::Left
								&& !(Signal && Signal->IsLaneProtectedGreen(JnctState.JunctionLane))
								&& TrafficSub->HasConflictingApproach(DetectedJunctionId, this, JnctState.FromLane, JnctState.ToLane))
							{
								bLeftTurnYield = true;
							}
						}

						if (bSignalAllows && !bLeftTurnYield && TrafficSub->TryOccupyJunction(DetectedJunctionId, this, JnctState.FromLane, JnctState.ToLane))
						{
							JnctState.BeginTraversing();
							UE_LOG(LogAAATraffic, Warning,
								TEXT("JNCT OCCUPY-GRANTED: Pawn='%s' JunctionId=%d — PROCEEDING through intersection"),
								GetPawn() ? *GetPawn()->GetName() : TEXT("NULL"),
								DetectedJunctionId);
						}
						else
						{
							JnctState.BeginWaiting();
							JnctState.bStopSignWaitComplete = true; // N/A for signals.
							JnctState.StopSignStopElapsed = 0.0f;
							UE_LOG(LogAAATraffic, Warning,
								TEXT("JNCT OCCUPY-DENIED: Pawn='%s' JunctionId=%d SignalAllows=%s "
									 "— WAITING at intersection"),
								GetPawn() ? *GetPawn()->GetName() : TEXT("NULL"),
								DetectedJunctionId,
								bSignalAllows ? TEXT("YES-occupied") : TEXT("NO-signal-red"));
						}
						} // end Signal mode
					}
				}
				else
				{
					UE_LOG(LogAAATraffic, Warning,
						TEXT("JNCT DETECT-NONE: Pawn='%s' CurrentLane=%d — no junction detected "
							 "(current=0, all next=0). Proceeding directly to CheckLaneTransition."),
						GetPawn() ? *GetPawn()->GetName() : TEXT("NULL"),
						CurrentLane.HandleId);
				}
			}
		} // Close detection gate — junction detection only fires near lane end.

		// --- Intersection waiting: fires UNCONDITIONALLY when JnctState.bWaiting ---
		// FIX: Moved outside the (RemainingDist < TransitionThreshold) gate.
		// Previously, braking dropped out when the threshold shrank with speed,
		// causing vehicles to coast through lane ends at ~470 cm/s.
		if (JnctState.bWaiting)
		{
			UWorld* World = GetWorld();
			UTrafficSubsystem* TrafficSub = World ? World->GetSubsystem<UTrafficSubsystem>() : nullptr;

			// Check for stuck bug: waiting flag is true but junction ID was
			// erased (e.g., by an external InitializeLaneFollowing call).
			//
			// FIX (was CRITICAL): Previously only logged the error and left the
			// vehicle stuck FOREVER. Now auto-recovers by clearing the waiting
			// flag so the vehicle resumes lane following on its current lane.
			if (JnctState.JunctionId == 0)
			{
				UE_LOG(LogAAATraffic, Error,
					TEXT("JNCT BUG-RECOVERED: Pawn='%s' JnctState.bWaiting=YES but "
						 "JnctState.JunctionId=0! Clearing waiting flag to resume driving. "
						 "Root cause: InitializeLaneFollowing erased the ID while waiting."),
					GetPawn() ? *GetPawn()->GetName() : TEXT("NULL"));
				JnctState.Reset();
				// Fall through — vehicle will resume normal lane following.
			}
			else if (TrafficSub)
			{
				// Accumulate wait time for timeout detection.
				JnctState.WaitElapsed += DeltaSeconds;

				// ── Stop-sign elapsed timer (every frame, not just on retry) ──
				// Accumulate real DeltaSeconds so the required-wait comparison
				// is frame-rate independent.  The old code added a hardcoded
				// 0.25 s per retry tick, coupling the timer to retry cadence.
				{
					ATrafficSignalController* StopTimerSignal = TrafficSub->GetSignalForJunction(JnctState.JunctionId);
					const EJunctionControlMode StopTimerMode = StopTimerSignal
						? StopTimerSignal->GetControlMode()
						: EJunctionControlMode::Yield;
					if (!JnctState.bStopSignWaitComplete
						&& (StopTimerMode == EJunctionControlMode::StopSign
							|| StopTimerMode == EJunctionControlMode::FlashingRed))
					{
						if (FMath::Abs(CurrentSpeed) < 10.0f)
						{
							JnctState.StopSignStopElapsed += DeltaSeconds;
							const float RequiredWait = StopTimerSignal ? StopTimerSignal->StopSignWaitTimeSec : 2.0f;
							if (JnctState.StopSignStopElapsed >= RequiredWait)
							{
								JnctState.bStopSignWaitComplete = true;
								// Register in FIFO queue — first to complete mandatory stop = first to go.
								TrafficSub->RecordStopSignArrival(JnctState.JunctionId, this);
								UE_LOG(LogAAATraffic, Warning,
									TEXT("JNCT STOPSIGN-WAIT-DONE: Pawn='%s' JunctionId=%d — "
										 "waited %.1fs at stop, now checking occupancy (FIFO queued)"),
									GetPawn() ? *GetPawn()->GetName() : TEXT("NULL"),
									JnctState.JunctionId, JnctState.StopSignStopElapsed);
							}
						}
						else
						{
							JnctState.StopSignStopElapsed = 0.0f;
						}
					}
				}

				// Throttle retry attempts to ~4 per second to avoid log spam.
				// Without this, TryOccupyJunction fires + logs DENIED every tick.
				JnctState.RetryTimer -= DeltaSeconds;
				if (JnctState.RetryTimer <= 0.0f)
				{
					JnctState.RetryTimer = 0.25f; // retry every 250ms

					bool bSignalAllows = true;
					ATrafficSignalController* Signal = TrafficSub->GetSignalForJunction(JnctState.JunctionId);
					const EJunctionControlMode JunctionMode = Signal
						? Signal->GetControlMode()
						: EJunctionControlMode::Yield;

					if (JunctionMode == EJunctionControlMode::Signal)
					{
						// Signal mode: wait for green.
					if (Signal)
					{
						bSignalAllows = Signal->IsLaneGreen(JnctState.JunctionLane);
					}

					// Right-on-red: if signal is red and this vehicle is turning
					// right, allow proceeding after a full stop if no conflicting
					// traffic. Standard North American traffic law.
					if (!bSignalAllows && Signal
						&& Signal->GetCurrentPhase() == ETrafficSignalPhase::Red
						&& FMath::Abs(CurrentSpeed) < 10.0f)
					{
						const ETurnSignalState RoRTurn = ComputeTurnDirection(JnctState.FromLane, JnctState.ToLane);
						if (RoRTurn == ETurnSignalState::Right
							&& !TrafficSub->HasConflictingApproach(JnctState.JunctionId, this, JnctState.FromLane, JnctState.ToLane))
						{
							bSignalAllows = true;
							UE_LOG(LogAAATraffic, Log,
								TEXT("JNCT RIGHT-ON-RED: Pawn='%s' JunctionId=%d — "
									 "stopped + no conflicts, proceeding with right turn on red"),
								GetPawn() ? *GetPawn()->GetName() : TEXT("NULL"),
								JnctState.JunctionId);
						}
					}
					}
					else if (JunctionMode == EJunctionControlMode::StopSign || JunctionMode == EJunctionControlMode::FlashingRed)
					{
						// Stop sign: wait timer is now accumulated every frame
						// (above the retry gate) using real DeltaSeconds.
						if (!JnctState.bStopSignWaitComplete)
						{
							bSignalAllows = false; // Don't proceed until wait is complete.
						}
						else if (!TrafficSub->IsStopSignTurnToGo(JnctState.JunctionId, this))
						{
							// FIFO ordering: another vehicle arrived first — must wait our turn.
							bSignalAllows = false;
						}
					}
					// Yield mode: also check approaching cross-traffic gap acceptance.
					if (bSignalAllows && JunctionMode == EJunctionControlMode::Yield)
					{
						if (TrafficSub->HasApproachingCrossTraffic(
								JnctState.JunctionId, this, JnctState.FromLane, JnctState.ToLane))
						{
							bSignalAllows = false;
						}
					}

					// Left-turn yield in retry: left-turners yield to oncoming
					// straight-through traffic at ALL intersection types, not
					// just signals. Standard traffic law: left turn = lowest
					// priority regardless of control mode (Signal/StopSign/Yield).
					bool bLeftTurnYieldRetry = false;
					if (bSignalAllows)
					{
						const ETurnSignalState RetryTurnDir = ComputeTurnDirection(JnctState.FromLane, JnctState.ToLane);
						if (RetryTurnDir == ETurnSignalState::Left
							&& !(Signal && Signal->IsLaneProtectedGreen(JnctState.JunctionLane))
							&& TrafficSub->HasConflictingApproach(JnctState.JunctionId, this, JnctState.FromLane, JnctState.ToLane))
						{
							bLeftTurnYieldRetry = true;
						}
					}

					if (bSignalAllows && !bLeftTurnYieldRetry && TrafficSub->TryOccupyJunction(JnctState.JunctionId, this, JnctState.FromLane, JnctState.ToLane))
					{
						JnctState.BeginTraversing();
						// Remove from stop-sign FIFO queue now that occupancy is granted.
						TrafficSub->RemoveStopSignArrival(JnctState.JunctionId, this);

						// Note: old overshoot-teleport removed. The old-lane-tail
						// prepend in CheckLaneTransition now gives the new lane
						// continuous polyline coverage from the vehicle's actual
						// position, making teleport unnecessary and visually jarring.

						JnctState.bHasEntryPos = false;
						UE_LOG(LogAAATraffic, Warning,
							TEXT("JNCT RETRY-GRANTED: Pawn='%s' JunctionId=%d — CLEARED to proceed (was waiting, elapsed %.1fs)"),
							GetPawn() ? *GetPawn()->GetName() : TEXT("NULL"),
							JnctState.JunctionId,
							JnctState.WaitElapsed);
						// Fall through to lane transition gate below.
					}
				}

				// FIX (was MAJOR): Timeout — if the vehicle has been waiting
				// longer than MaxIntersectionWaitTimeSec, force-proceed to
				// prevent permanent deadlocks. The junction may be blocked by
				// a stuck vehicle, broken signal, or mutual denial cycle.
				//
				// TryOccupyJunction now auto-evicts occupants that have held
				// a junction for >20 s (stuck vehicles), so a clean retry is
				// usually enough.  ForceOccupy is a last resort.
				if (JnctState.bWaiting && MaxIntersectionWaitTimeSec > 0.0f && JnctState.WaitElapsed >= MaxIntersectionWaitTimeSec)
				{
					UE_LOG(LogAAATraffic, Warning,
						TEXT("JNCT TIMEOUT-FORCE-PROCEED: Pawn='%s' JunctionId=%d — "
							 "waited %.1fs (max %.1fs). Retrying TryOccupy (stale occupants auto-evicted)."),
						GetPawn() ? *GetPawn()->GetName() : TEXT("NULL"),
						JnctState.JunctionId,
						JnctState.WaitElapsed,
						MaxIntersectionWaitTimeSec);

					if (TrafficSub)
					{
						// TryOccupy now evicts stale occupants (>20 s) before
						// checking conflicts, so this will succeed if the blocker
						// was stuck.  If it still fails (e.g. genuine heavy
						// traffic), force-occupy as last resort so the vehicle
						// isn't lost forever.
						if (!TrafficSub->TryOccupyJunction(JnctState.JunctionId,
							this, JnctState.FromLane, JnctState.ToLane))
						{
							TrafficSub->ForceOccupyJunction(JnctState.JunctionId,
								this, JnctState.FromLane, JnctState.ToLane);
						}
					}

					JnctState.BeginTraversing();
					JnctState.bHasEntryPos = false;
				}
			}

			if (JnctState.bWaiting)
			{
				// Per-instance throttled logging: show stuck/waiting vehicles periodically.
				// FIX (was MAJOR): Replaced static int32 shared across all instances
				// with per-instance member to avoid determinism violations.
				++WaitLogThrottleCounter;
				if ((WaitLogThrottleCounter % 60) == 0)
				{
					UE_LOG(LogAAATraffic, Warning,
						TEXT("JNCT RETRY-STILL-WAITING: Pawn='%s' JunctionId=%d "
							 "bWaiting=YES DistToEntry=%.1f Speed=%.1f WaitTime=%.1fs (throttled log, every 60 ticks)"),
						GetPawn() ? *GetPawn()->GetName() : TEXT("NULL"),
						JnctState.JunctionId,
						JnctState.bHasEntryPos
							? FVector::Dist(VehicleLocation, JnctState.EntryWorldPos)
							: RemainingDist,
						CurrentSpeed,
						JnctState.WaitElapsed);
				}

				// Decel-curve brake toward intersection entry point.
				// Compute distance to entry, checking for overshoot.
				// Prefer polyline-based approach distance (arc) over Euclidean
				// when available — Euclidean is too short on curved roads.
				//
				// C3 FIX: Also detect any vehicle stopped between ego and
				// the entry point. If a leader is closer than the entry point,
				// stop behind the leader (bumper-to-bumper) instead of driving
				// into it while targeting the entry position.
				float DistToEntry = RemainingDist;
				{
					float WaitLeaderSpeed = 0.0f;
					float WaitLeaderDist = GetLeaderDistance(WaitLeaderSpeed);
					if (WaitLeaderDist >= 0.0f)
					{
						WaitLeaderDist = FMath::Max(WaitLeaderDist - VehicleFrontExtent, 1.0f);
						if (WaitLeaderDist < DistToEntry)
						{
							// Leader is closer than the intersection entry — stop behind leader.
							DistToEntry = WaitLeaderDist;
						}
					}
				}
				if (JnctState.bHasEntryPos)
				{
					const FVector ToEntry = JnctState.EntryWorldPos - VehicleLocation;
					const float DotFwd = FVector::DotProduct(ToEntry, VehicleForward);
					if (DotFwd <= 0.0f)
					{
						DistToEntry = 0.0f; // Overshot — treat as zero.
					}
					else
					{
						// FIX: Always use fresh Euclidean distance to entry.
						// The old code preferred JnctState.ApproachDistanceCm
						// (polyline arc from the approach scan), but that value
						// FREEZES when JnctState.bWaiting is set (the scan
						// condition requires !bWaiting).  The stale distance
						// made the decel curve think the vehicle was much
						// farther from the stop line than it actually was,
						// producing a high DesiredStopSpeed and near-zero
						// brake input — causing consistent overshoot.
						// Euclidean updates every frame and at junction-approach
						// distances (< ~7000 cm) the arc vs Euclidean gap is
						// negligible, always erring on the safe side (shorter
						// distance → more conservative braking).
						DistToEntry = FVector::Dist(VehicleLocation, JnctState.EntryWorldPos);
					}
				}

				// Offset so the front bumper (not vehicle center) aligns with
				// the entry point.  VehicleFrontExtent is the distance from
				// the actor origin to the front of the mesh, computed during
				// OnPossess from the bounding box.
				if (DistToEntry > 0.0f)
				{
					DistToEntry = FMath::Max(DistToEntry - VehicleFrontExtent, 0.0f);
				}

				// Desired speed from constant-decel stopping curve: v = sqrt(2*a*d)
				// Uses the same tunable decel as the approach speed envelope
				// so both systems agree on braking physics.
				const float DesiredStopSpeed = FMath::Sqrt(
					FMath::Max(2.0f * ApproachDecelCmPerSec2 * DistToEntry, 0.0f));

				// Compute basic lane-following steering even while braking for
				// the intersection.  Without this, vehicles on curved approach
				// roads drift off-lane because steering was zeroed.
				float WaitSteer = 0.0f;
				{
					const FVector WaitTarget = GetLookAheadPoint(VehicleLocation, ClosestIndex);
					const FVector ToTarget = WaitTarget - VehicleLocation;
					const float Ld = ToTarget.Size2D();
					if (Ld > KINDA_SMALL_NUMBER)
					{
						const FVector ToTargetDir = ToTarget / Ld;
						const float CrossZ = FVector::CrossProduct(VehicleForward, ToTargetDir).Z;
						WaitSteer = FMath::Clamp(
							FMath::Atan2(2.0f * VehicleWheelbaseCm * CrossZ, Ld) / VehicleMaxSteerAngleRad,
							-1.0f, 1.0f);
					}
				}

				float WaitBrake = 0.0f;
				if (DistToEntry < 50.0f || DesiredStopSpeed <= KINDA_SMALL_NUMBER)
				{
					// At or past entry — full stop.
					WaitBrake = 1.0f;
					VehicleMovement->SetThrottleInput(0.0f);
					VehicleMovement->SetSteeringInput(0.0f);
					VehicleMovement->SetBrakeInput(1.0f);
				}
				else if (FMath::Abs(CurrentSpeed) > DesiredStopSpeed)
				{
					// Over the decel envelope — brake proportionally.
					WaitBrake = FMath::Clamp(
						(FMath::Abs(CurrentSpeed) - DesiredStopSpeed) /
						FMath::Max(DesiredStopSpeed, 100.0f),
						0.0f, 1.0f);
					VehicleMovement->SetThrottleInput(0.0f);
					VehicleMovement->SetSteeringInput(WaitSteer);
					VehicleMovement->SetBrakeInput(WaitBrake);
				}
				else
				{
					// Under envelope — coast if still rolling, hold brake if stopped.
					// Without holding brake a stopped vehicle on a slope rolls freely
					// (Throttle=0, Brake=0 = no forces).
					const bool bEffectivelyStopped = FMath::Abs(CurrentSpeed) < 10.0f;
					VehicleMovement->SetThrottleInput(0.0f);
					VehicleMovement->SetSteeringInput(bEffectivelyStopped ? 0.0f : WaitSteer);
					VehicleMovement->SetBrakeInput(bEffectivelyStopped ? 1.0f : 0.0f);
					WaitBrake = bEffectivelyStopped ? 1.0f : 0.0f;
				}

#if defined(ENABLE_DRAW_DEBUG) && ENABLE_DRAW_DEBUG
				DbgStateName = TEXT("WAITING");
				DbgBrake = WaitBrake;
				DbgThrottle = 0.0f;
				DbgDistToEntry = DistToEntry;
				DbgDesiredStopSpeed = DesiredStopSpeed;
#endif
				return;
			}
		}

		// --- Lane transition gate ---
		// Re-evaluated separately from detection so post-RETRY-GRANTED vehicles
		// can proceed to CheckLaneTransition immediately.
		// Bypass MinTransitionGuard for junction lanes to prevent vehicles from
		// being stuck for 30+ seconds on short junction polylines.
		{
			const bool bJunctionBypass = JnctState.IsActive();
			if (RemainingDist < TransitionThreshold
				&& (DistanceTraveledOnLane > MinTransitionGuard || bJunctionBypass))
			{
				if (LaneChangeState != ELaneChangeState::None)
				{
					// Lane change was still in progress when we reached the
					// transition threshold.  Abort the change so the vehicle
					// can proceed to CheckLaneTransition on the current lane
					// instead of being stuck in an incomplete merge.
					LaneChangeState = ELaneChangeState::None;
					LaneChangeProgress = 0.0f;
					LaneChangeSettleTimer = 0.0f;
					TargetLanePoints.Empty();
				}

				CheckLaneTransition();
				if (!bLaneDataReady || !GetPawn())
				{
					return;
				}
				if (!bAtDeadEnd)
				{
					if (JnctState.IsActive())
					{
						const int32 NewLaneJunction = CachedProvider
							? CachedProvider->GetJunctionForLane(CurrentLane) : 0;

						if (NewLaneJunction != JnctState.JunctionId
							&& JnctState.TransitionPoints.Num() == 0)
						{
							// Exit lane is outside the junction AND no curve is
							// pending — safe to release immediately.
							UWorld* World = GetWorld();
							UTrafficSubsystem* TrafficSub = World
								? World->GetSubsystem<UTrafficSubsystem>() : nullptr;
							if (TrafficSub)
							{
								UE_LOG(LogAAATraffic, Warning,
									TEXT("JNCT RELEASE: Pawn='%s' RELEASING junction %d — new lane %d "
										 "is in junction %d (exited)"),
									GetPawn() ? *GetPawn()->GetName() : TEXT("NULL"),
									JnctState.JunctionId, CurrentLane.HandleId, NewLaneJunction);
								TrafficSub->ReleaseJunction(JnctState.JunctionId, this);
							}
							JnctState.Release();
						}
						else if (NewLaneJunction != JnctState.JunctionId)
						{
							// Exit lane is outside the junction but a junction
							// curve is active — defer release to PATH2-CHECK
							// so the junction stays occupied while the vehicle
							// physically traverses the intersection.
							UE_LOG(LogAAATraffic, Warning,
								TEXT("JNCT RELEASE-DEFERRED: Pawn='%s' junction %d — "
									 "new lane %d is in junction %d but curve active (%d pts), "
									 "deferring release to PATH2"),
								GetPawn() ? *GetPawn()->GetName() : TEXT("NULL"),
								JnctState.JunctionId, CurrentLane.HandleId,
								NewLaneJunction, JnctState.TransitionPoints.Num());
						}
						else
						{
							if (GTrafficJunctionDiagnostics >= 1)
							{
								UE_LOG(LogAAATraffic, Log,
									TEXT("JNCT HOLD: Pawn='%s' junction %d — new lane %d still in "
										 "same junction, keeping occupancy"),
									GetPawn() ? *GetPawn()->GetName() : TEXT("NULL"),
									JnctState.JunctionId, CurrentLane.HandleId);
							}
						}
					}
					return;
				}
			}
		}
	}

	// --- Dead-end braking & despawn ---
	// Vehicle brakes to a stop, then despawns after a short, realistic
	// pause. Handles map-edge roads, cul-de-sacs, and any lane with no
	// forward connections.
	if (bAtDeadEnd)
	{
		VehicleMovement->SetThrottleInput(0.0f);
		VehicleMovement->SetSteeringInput(0.0f);
		VehicleMovement->SetBrakeInput(1.0f);

		DeadEndDespawnTimer += DeltaSeconds;
		if (DeadEndDespawnTimer >= DeadEndDespawnDelaySec && !bPendingRecoveryDespawn)
		{
			bPendingRecoveryDespawn = true;
			if (UTrafficSubsystem* TrafficSub = GetWorld()->GetSubsystem<UTrafficSubsystem>())
			{
				TrafficSub->RequestDespawn(this,
					FString::Printf(TEXT("dead-end on lane %d (no forward connections)"),
						CurrentLane.HandleId));
			}
		}
		return;
	}

	// --- Determine target point (junction transition, lane-change blended, or normal) ---
	FVector TargetPoint;

	// Junction transition takes priority — follow synthesized curve first.
	if (JnctState.TransitionPoints.Num() > 0)
	{
		// Advance along junction points based on proximity or forward progress.
		// Use both distance check and "closer to next" test so high-speed vehicles
		// that skip past points still advance correctly.
		while (JnctState.TransitionIndex < JnctState.TransitionPoints.Num() - 1)
		{
			const float DistToCurrentSq = FVector::DistSquared(VehicleLocation, JnctState.TransitionPoints[JnctState.TransitionIndex]);
			const float DistToNextSq = FVector::DistSquared(VehicleLocation, JnctState.TransitionPoints[JnctState.TransitionIndex + 1]);
			if (DistToCurrentSq < 10000.0f || DistToNextSq < DistToCurrentSq) // 1m or closer to next
			{
				++JnctState.TransitionIndex;
			}
			else
			{
				break;
			}
		}

		if (JnctState.TransitionIndex >= JnctState.TransitionPoints.Num() - 1)
		{
			// Finished junction transition — release junction occupancy and proceed.
			JnctState.TransitionPoints.Empty();
			JnctState.TransitionIndex = 0;

			// Turn signal off (deferred) — keep signal on briefly after turn completes.
			TurnSignalOffDelayRemaining = 1.5f;

			// Clear navigational pre-positioning — now on a new road.
			bNavigationalLaneChange = false;
			NavigationalTargetLane = FTrafficLaneHandle();

			UE_LOG(LogAAATraffic, Warning,
				TEXT("JNCT RELEASE-PATH2-CHECK: Pawn='%s' JnctState.JunctionId=%d "
					 "— junction curve complete, checking release"),
				GetPawn() ? *GetPawn()->GetName() : TEXT("NULL"),
				JnctState.JunctionId);

			if (UWorld* World = GetWorld())
			{
				if (UTrafficSubsystem* TrafficSub = World->GetSubsystem<UTrafficSubsystem>())
				{
					UE_LOG(LogAAATraffic, Warning,
						TEXT("JNCT RELEASE-PATH2-FIRE: Pawn='%s' RELEASING junction %d (curve complete)"),
						GetPawn() ? *GetPawn()->GetName() : TEXT("NULL"),
						JnctState.JunctionId);
					TrafficSub->ReleaseJunction(JnctState.JunctionId, this);
				}
			}
			JnctState.Release();

			TargetPoint = GetLookAheadPoint(VehicleLocation, ClosestIndex);
		}
		else
		{
			// Speed-adaptive look-ahead along junction curve: walk forward
			// by LookAheadDistance rather than a fixed +2 index offset.
			// For turns, clamp the look-ahead to prevent the pure-pursuit
			// target from overshooting past the bend apex (cutting the corner).
			float JctLookAhead = LookAheadDistance;
			if (JnctState.TransitionPoints.Num() >= 3)
			{
				// Estimate turn radius from total arc length / total angle change.
				float TotalArc = 0.0f;
				float TotalAngle = 0.0f;
				for (int32 k = 0; k < JnctState.TransitionPoints.Num() - 1; ++k)
				{
					TotalArc += FVector::Dist(JnctState.TransitionPoints[k], JnctState.TransitionPoints[k + 1]);
				}
				if (JnctState.TransitionPoints.Num() >= 2)
				{
					const FVector StartDir = (JnctState.TransitionPoints[1] - JnctState.TransitionPoints[0]).GetSafeNormal();
					const FVector EndDir = (JnctState.TransitionPoints.Last() - JnctState.TransitionPoints[JnctState.TransitionPoints.Num() - 2]).GetSafeNormal();
					TotalAngle = FMath::Acos(FMath::Clamp(FVector::DotProduct(StartDir, EndDir), -1.0f, 1.0f));
				}
				if (TotalAngle > 0.15f) // ~8.6° — actual turn, not straight-through
				{
					const float EstRadius = TotalArc / FMath::Max(TotalAngle, KINDA_SMALL_NUMBER);
					JctLookAhead = FMath::Min(JctLookAhead, EstRadius * 0.5f);
					// Do not go below the minimum look-ahead.
					JctLookAhead = FMath::Max(JctLookAhead, MinLookAheadDistanceCm);
				}
			}

			float JctAccumDist = 0.0f;
			int32 JctIdx = JnctState.TransitionIndex;
			while (JctIdx < JnctState.TransitionPoints.Num() - 1)
			{
				const float SegDist = FVector::Dist2D(
					JnctState.TransitionPoints[JctIdx],
					JnctState.TransitionPoints[JctIdx + 1]);
				JctAccumDist += SegDist;
				if (JctAccumDist >= JctLookAhead)
				{
					const float Overshoot = JctAccumDist - JctLookAhead;
					const float Alpha = 1.0f - (Overshoot / FMath::Max(SegDist, KINDA_SMALL_NUMBER));
					TargetPoint = FMath::Lerp(
						JnctState.TransitionPoints[JctIdx],
						JnctState.TransitionPoints[JctIdx + 1], Alpha);
					break;
				}
				++JctIdx;
			}
			if (JctAccumDist < JctLookAhead)
			{
				// Extrapolate past the end of the junction curve.
				const int32 Last = JnctState.TransitionPoints.Num() - 1;
				if (Last >= 1)
				{
					const FVector Dir = (JnctState.TransitionPoints[Last] - JnctState.TransitionPoints[Last - 1]).GetSafeNormal();
					TargetPoint = JnctState.TransitionPoints[Last] + Dir * (JctLookAhead - JctAccumDist);
				}
				else
				{
					TargetPoint = JnctState.TransitionPoints[Last];
				}
			}
		}
	}
	else if (LaneChangeState == ELaneChangeState::Executing)
	{
		TargetPoint = UpdateLaneChangeBlend(VehicleLocation, ClosestIndex);
	}
	else if (LaneChangeState == ELaneChangeState::Completing)
	{
		// Settling phase: drive purely on target lane, count down settle timer.
		LaneChangeSettleTimer -= DeltaSeconds;
		if (LaneChangeSettleTimer <= 0.0f)
		{
			FinalizeLaneChange();
			// FinalizeLaneChange() clears TargetLanePoints and sets state to None.
			// Use normal look-ahead on the (now current) lane.
			TargetPoint = GetLookAheadPoint(VehicleLocation, ClosestIndex);
		}
		else
		{
			// While settling, steer toward target lane look-ahead.
			TargetPoint = UpdateLaneChangeBlend(VehicleLocation, ClosestIndex);
		}
	}
	else
	{
		TargetPoint = GetLookAheadPoint(VehicleLocation, ClosestIndex);
	}

	// --- Pure Pursuit Steering with Curvature Feedforward ---
	// 0. Curvature feedforward: proactive steering based on road shape.
	//    δ_ff = atan(Wheelbase × κ) / MaxSteerAngle
	//    This tells the car what the road IS, so it turns proactively
	//    instead of chasing a point and reacting late.
	// 1. Pure pursuit: geometric arc from rear axle to look-ahead point.
	//    δ = atan(2 · L · sin(α) / Ld)
	// 2. CTE correction: lateral offset from lane centerline adds a
	//    proportional steering push back toward the lane center.
	// 3. Derivative damping: rate of change of heading error for stability.

	// Compute local curvature at vehicle position from polyline.
	// Uses the active polyline (junction transition or lane polyline).
	float LocalCurvature = 0.0f;
	if (JnctState.TransitionIndex > 0 && JnctState.TransitionIndex < JnctState.TransitionPoints.Num())
	{
		LocalCurvature = ComputeLocalCurvature(JnctState.TransitionPoints, JnctState.TransitionIndex);
	}
	else if (LanePoints.Num() >= 3)
	{
		LocalCurvature = ComputeLocalCurvature(LanePoints, ClosestIndex);
	}

	// Feedforward term: atan(Wheelbase * κ) normalized to [-1, 1].
	// Zero new tuning params — uses only physics-measured Wheelbase and MaxSteerAngle.
	const float FFTerm = FMath::Atan(VehicleWheelbaseCm * LocalCurvature) / VehicleMaxSteerAngleRad;

	const FVector ToTargetVec = TargetPoint - VehicleLocation;
	const float Ld = ToTargetVec.Size2D();
	const FVector ToTargetDir = ToTargetVec.GetSafeNormal2D();
	const FVector Forward2D = VehicleForward.GetSafeNormal2D();
	const float CrossZ = FVector::CrossProduct(Forward2D, ToTargetDir).Z;

	// Pure pursuit arc curvature → steering angle.
	// Wheelbase and max steer angle read from Chaos vehicle data (OnPossess).
	const float PurePursuitAngle = (Ld > KINDA_SMALL_NUMBER)
		? FMath::Atan(2.0f * VehicleWheelbaseCm * CrossZ / Ld)
		: 0.0f;
	const float PTerm = PurePursuitAngle / VehicleMaxSteerAngleRad;

	// Cross-track error (CTE): signed perpendicular distance from vehicle
	// to the nearest lane segment. Positive = vehicle is to the RIGHT of
	// lane centerline, negative = LEFT.
	// SUPPRESS CTE during junction curve following: the vehicle is
	// intentionally off the new lane's centerline while executing the turn.
	// CTE against the new lane's polyline fights the junction steering and
	// prevents the vehicle from completing the turn.
	const bool bFollowingJunctionCurve = (JnctState.TransitionPoints.Num() > 0
		&& JnctState.TransitionIndex < JnctState.TransitionPoints.Num() - 1);
	float SignedCTE = 0.0f;
	if (!bFollowingJunctionCurve && LanePoints.Num() >= 2)
	{
		const int32 SegIdx = FMath::Clamp(ClosestIndex, 0, LanePoints.Num() - 2);
		const FVector& SegA = LanePoints[SegIdx];
		const FVector& SegB = LanePoints[SegIdx + 1];
		const FVector SegDir = (SegB - SegA).GetSafeNormal2D();
		if (!SegDir.IsNearlyZero())
		{
			// Perpendicular offset: cross product of segment direction and
			// vehicle-to-segment vector gives signed lateral distance.
			const FVector VehicleToSeg = VehicleLocation - SegA;
			SignedCTE = FVector2D::CrossProduct(
				FVector2D(SegDir), FVector2D(VehicleToSeg));
		}
	}

	// CTE steering correction: proportional to lateral offset, normalized
	// by half lane width. Uses atan for soft saturation so large offsets
	// don't cause violent steering swings.
	//
	// The CTE gain must be LOW relative to the pure pursuit heading term.
	// On a curve with radius R, PTerm ≈ Wheelbase/(R·MaxSteerAngle).
	// For a 15m curve: PTerm ≈ 0.30. If CTETerm exceeds this even at
	// small lateral offsets, the CTE overpowers the heading correction
	// and the car steers OPPOSITE to the curve → leaves the road.
	// CTECorrectionGain = 0.3 keeps CTETerm safely below PTerm on
	// all but extreme offsets, while still providing meaningful
	// centering on straights (0.48 steering at full lane offset).
	//
	// Use positional lane width when available (variable-width lanes,
	// turn pockets, tapered merges). Falls back to uniform LaneWidth.
	float EffectiveLaneWidth = LaneWidth;
	if (CachedProvider && CurrentLane.IsValid())
	{
		// Approximate distance along lane from closest polyline index.
		float DistOnLane = 0.0f;
		const int32 ClampedIdx = FMath::Clamp(ClosestIndex, 0, LanePoints.Num() - 1);
		for (int32 i = 0; i < ClampedIdx && i < LanePoints.Num() - 1; ++i)
		{
			DistOnLane += FVector::Dist(LanePoints[i], LanePoints[i + 1]);
		}
		const float PositionalWidth = CachedProvider->GetLaneWidthAtDistance(CurrentLane, DistOnLane);
		if (PositionalWidth > 0.0f)
		{
			EffectiveLaneWidth = PositionalWidth;
		}
	}
	const float HalfLaneWidth = FMath::Max(EffectiveLaneWidth * 0.5f, 50.0f);
	const float CTENormalized = SignedCTE / HalfLaneWidth;
	const float CTETerm = (CTECorrectionGain > KINDA_SMALL_NUMBER)
		? FMath::Atan(CTECorrectionGain * CTENormalized) / VehicleMaxSteerAngleRad
		: 0.0f;

	// Derivative term: rate of change of heading cross product provides
	// damping. Prevents oscillatory steering on long straights.
	const float CrossTrackDerivative = (DeltaSeconds > KINDA_SMALL_NUMBER)
		? (CrossZ - PreviousHeadingCrossZ) / DeltaSeconds
		: 0.0f;
	PreviousHeadingCrossZ = CrossZ;

	// Damping contribution: scale derivative by the tunable factor and
	// normalize against a reference rate so the gain is intuitive (1.0 =
	// moderate damping at 60 fps). Clamp to prevent derivative kick from
	// sudden lane-change blend transitions.
	const float DTerm = FMath::Clamp(CrossTrackDerivative * SteeringDampingFactor * 0.016f, -0.3f, 0.3f);

	// CTE correction is SUBTRACTED: positive CTE (vehicle RIGHT of center)
	// must steer LEFT (negative), opposing the pure pursuit term.
	// Feedforward (FFTerm) is ADDED: it provides the base steering angle
	// that the road curvature demands, independent of heading error.
	const float SteeringInput = FMath::Clamp(FFTerm + PTerm - CTETerm + DTerm, -1.0f, 1.0f);

	// --- Steering diagnostic logging (throttled) ---
	// Emitted every ~60 ticks (~1 second at 60fps) to avoid log spam.
	// Provides CTE, steering breakdown, speed, look-ahead distance.
	++SteeringDiagCounter;
	if (SteeringDiagCounter >= 60)
	{
		SteeringDiagCounter = 0;
		const float CTEcm = SignedCTE;
		const float CTEpct = (HalfLaneWidth > 0.0f) ? (FMath::Abs(SignedCTE) / HalfLaneWidth * 100.0f) : 0.0f;
		UE_LOG(LogAAATraffic, Log,
			TEXT("STEER DIAG: Pawn='%s' Lane=%d CTE=%.1fcm (%.0f%%) "
				 "FF=%.3f P=%.3f CTE=%.3f D=%.3f Total=%.3f Speed=%.0f LookAhead=%.0f κ=%.6f"),
			GetPawn() ? *GetPawn()->GetName() : TEXT("NULL"),
			CurrentLane.HandleId,
			CTEcm, CTEpct,
			FFTerm, PTerm, CTETerm, DTerm, SteeringInput,
			CurrentSpeed, LookAheadDistance, LocalCurvature);
	}

	// Lane departure warning: log when CTE exceeds 80% of half lane width.
	if (FMath::Abs(SignedCTE) > HalfLaneWidth * 0.8f)
	{
		UE_LOG(LogAAATraffic, Warning,
			TEXT("LANE DEPARTURE: Pawn='%s' Lane=%d CTE=%.1fcm (%.0f%% of half-lane) "
				 "Speed=%.0f Steer=%.3f"),
			GetPawn() ? *GetPawn()->GetName() : TEXT("NULL"),
			CurrentLane.HandleId,
			SignedCTE, (HalfLaneWidth > 0.0f ? FMath::Abs(SignedCTE) / HalfLaneWidth * 100.0f : 0.0f),
			CurrentSpeed, SteeringInput);
	}

	// --- Throttle / Brake (IDM — Intelligent Driver Model) ---
	// Step 1: Detect leader vehicle (LOD-dependent method).
	// Step 2: Compute desired speed v₀ (road limit + junction/curve caps).
	// Step 3: IDM acceleration: ẍ = a[1 - (v/v₀)⁴ - (s*/s)²],
	//         s* = s₀ + max(0, vT + vΔv/(2√(ab))).
	// Step 4: Map acceleration to throttle/brake inputs.

	// ── Step 1: Leader detection ─────────────────────────────
	// On Reduced LOD, use spatial grid instead of physics sweep.
	float LeaderDist = -1.0f;
	float LeaderSpeed = 0.0f;
	{
		ETrafficLOD TickLOD = ETrafficLOD::Full;
		if (UWorld* LODWorld = GetWorld())
		{
			if (UTrafficSubsystem* LODSub = LODWorld->GetSubsystem<UTrafficSubsystem>())
			{
				TickLOD = LODSub->GetVehicleLOD(this);
			}
		}

		if (TickLOD == ETrafficLOD::Full)
		{
			LeaderDist = GetLeaderDistance(LeaderSpeed);
			// C1 FIX: Convert to approximate bumper-to-bumper gap.
			// GetLeaderDistance returns sweep hit distance from an offset
			// start (past ego collision shape). Subtract ego front extent
			// to approximate true bumper-to-bumper spacing.
			if (LeaderDist >= 0.0f)
			{
				LeaderDist = FMath::Max(LeaderDist - VehicleFrontExtent, 1.0f);
			}
		}
		else if (TickLOD == ETrafficLOD::Reduced)
		{
			// Analytical leader detection via spatial grid — cheaper for distant vehicles.
			UTrafficSubsystem* SpSub = nullptr;
			if (UWorld* SpWorld = GetWorld())
			{
				SpSub = SpWorld->GetSubsystem<UTrafficSubsystem>();
			}
			if (SpSub)
			{
				const FVector Loc = ControlledPawn->GetActorLocation();
				const FVector Fwd = ControlledPawn->GetActorForwardVector();
				TArray<ATrafficVehicleController*> Nearby = SpSub->GetNearbyVehicles(Loc, DetectionDistance);

				float BestDist = MAX_FLT;
				float BestSpeed = 0.0f;
				for (const ATrafficVehicleController* Other : Nearby)
				{
					if (Other == this) { continue; }
					const APawn* OP = Other->GetPawn();
					if (!OP) { continue; }

					// I2 FIX: Lane affinity filter — only consider vehicles on
					// our lane, an immediately adjacent same-direction lane, or
					// a connected lane (junction lanes connected to our lane).
					// Prevents phantom braking from parallel/diagonal roads while
					// still detecting vehicles that entered a junction ahead of us.
					if (CachedProvider && CurrentLane.IsValid())
					{
						const FTrafficLaneHandle OtherLane = Other->GetCurrentLane();
						if (OtherLane.IsValid()
							&& OtherLane != CurrentLane
							&& OtherLane != CachedProvider->GetAdjacentLane(CurrentLane, ETrafficLaneSide::Left)
							&& OtherLane != CachedProvider->GetAdjacentLane(CurrentLane, ETrafficLaneSide::Right))
						{
							// Also accept vehicles on lanes directly connected
							// to our current lane (e.g., junction lanes ahead).
							bool bConnected = false;
							TArray<FTrafficLaneHandle> Connected = CachedProvider->GetConnectedLanes(CurrentLane);
							for (const FTrafficLaneHandle& CL : Connected)
							{
								if (CL == OtherLane) { bConnected = true; break; }
							}
							if (!bConnected)
							{
								continue; // Different road or non-adjacent lane.
							}
						}
					}

					const FVector Delta = OP->GetActorLocation() - Loc;
					const float Dist = Delta.Size();
					if (Dist < KINDA_SMALL_NUMBER || FVector::DotProduct(Delta / Dist, Fwd) < 0.5f)
					{
						continue;
					}
					if (Dist < BestDist)
					{
						BestDist = Dist;
						if (const UPawnMovementComponent* OtherMC = OP->GetMovementComponent())
						{
							BestSpeed = FVector::DotProduct(OtherMC->Velocity, Fwd);
						}
						else
						{
							BestSpeed = 0.0f;
						}
					}
				}
				if (BestDist < DetectionDistance)
				{
					// C1 FIX (Reduced LOD): Convert to approximate bumper-to-bumper gap,
					// mirroring the full-LOD correction after GetLeaderDistance.
					LeaderDist = FMath::Max(BestDist - VehicleFrontExtent, 1.0f);
					LeaderSpeed = BestSpeed;
				}
			}
		}
	}
	// ── AEB: Automatic Emergency Braking (TTC override) ──────
	// If the raw (undelayed) leader data shows an imminent collision,
	// bypass the reaction delay entirely. This is a separate safety
	// layer that fires BEFORE the IDM model, directly setting the
	// braking flag so the vehicle stops as fast as physics allows.
	// TTC = gap / closing_speed.  Threshold: 1.5s.
	bool bAEBActive = false;
	if (LeaderDist >= 0.0f)
	{
		const float ClosingSpeed = FMath::Abs(CurrentSpeed) - LeaderSpeed;
		if (ClosingSpeed > 50.0f) // Only when actually closing (>0.5 m/s)
		{
			const float TTC = LeaderDist / ClosingSpeed;
			if (TTC < AEBTimeToCollisionThresholdSec)
			{
				bAEBActive = true;
				UE_LOG(LogAAATraffic, Warning,
					TEXT("AEB ACTIVE: Pawn='%s' TTC=%.2fs Gap=%.0fcm Closing=%.0fcm/s — EMERGENCY BRAKE"),
					GetPawn() ? *GetPawn()->GetName() : TEXT("NULL"),
					TTC, LeaderDist, ClosingSpeed);
			}
		}
	}

	// ── Reaction delay buffer ────────────────────────────────
	// Push raw leader readings into a ring buffer; read back from
	// IDMReactionDelaySec ago. This creates a realistic braking cascade
	// in platoons instead of all vehicles braking simultaneously.
	IDMTimeClock += DeltaSeconds;
	float DelayedLeaderDist  = LeaderDist;
	float DelayedLeaderSpeed = LeaderSpeed;
	if (IDMReactionDelaySec > KINDA_SMALL_NUMBER)
	{
		// Push current sample.
		FLeaderSample Sample;
		Sample.Time  = IDMTimeClock;
		Sample.Dist  = LeaderDist;
		Sample.Speed = LeaderSpeed;
		LeaderDelayBuffer.Add(Sample);

		// Read delayed sample with linear interpolation.
		const float ReadTime = IDMTimeClock - IDMReactionDelaySec;
		if (LeaderDelayBuffer.Num() > 1)
		{
			// Find the first sample at or past ReadTime.
			int32 UpperIdx = LeaderDelayBuffer.Num() - 1;
			for (int32 i = LeaderDelayBuffer.Num() - 1; i >= 0; --i)
			{
				if (LeaderDelayBuffer[i].Time <= ReadTime)
				{
					UpperIdx = FMath::Min(i + 1, LeaderDelayBuffer.Num() - 1);
					break;
				}
			}
			const int32 LowerIdx = FMath::Max(UpperIdx - 1, 0);

			if (LowerIdx == UpperIdx)
			{
				// Only one relevant sample.
				DelayedLeaderDist  = LeaderDelayBuffer[LowerIdx].Dist;
				DelayedLeaderSpeed = LeaderDelayBuffer[LowerIdx].Speed;
			}
			else
			{
				// I5 FIX: Linear interpolation between bracketing samples.
				// Guard: if one sample is "no leader" (-1) and the other is a
				// real distance, LERP would produce a phantom leader at an
				// intermediate value. Use nearest-neighbor in that case.
				const bool bLowerValid = LeaderDelayBuffer[LowerIdx].Dist >= 0.0f;
				const bool bUpperValid = LeaderDelayBuffer[UpperIdx].Dist >= 0.0f;
				if (bLowerValid != bUpperValid)
				{
					// Straddles -1↔positive boundary — snap to nearest.
					const float T0 = LeaderDelayBuffer[LowerIdx].Time;
					const float T1 = LeaderDelayBuffer[UpperIdx].Time;
					const int32 SnapIdx = (FMath::Abs(ReadTime - T0) <= FMath::Abs(ReadTime - T1))
						? LowerIdx : UpperIdx;
					DelayedLeaderDist  = LeaderDelayBuffer[SnapIdx].Dist;
					DelayedLeaderSpeed = LeaderDelayBuffer[SnapIdx].Speed;
				}
				else
				{
					const float T0 = LeaderDelayBuffer[LowerIdx].Time;
					const float T1 = LeaderDelayBuffer[UpperIdx].Time;
					const float Frac = (T1 > T0)
						? FMath::Clamp((ReadTime - T0) / (T1 - T0), 0.0f, 1.0f)
						: 0.0f;
					DelayedLeaderDist  = FMath::Lerp(LeaderDelayBuffer[LowerIdx].Dist,
						LeaderDelayBuffer[UpperIdx].Dist, Frac);
					DelayedLeaderSpeed = FMath::Lerp(LeaderDelayBuffer[LowerIdx].Speed,
						LeaderDelayBuffer[UpperIdx].Speed, Frac);
				}
			}

			// Prune old entries (keep one before the read window).
			const int32 PruneUpTo = FMath::Max(0, LowerIdx - 1);
			if (PruneUpTo > 0)
			{
				LeaderDelayBuffer.RemoveAt(0, PruneUpTo, EAllowShrinking::No);
			}
		}
	}
	// C2 FIX: When delay is 0, no buffer needed — use raw values (already set above).

	// AEB bypass — when TTC < threshold, use instantaneous (raw) leader data
	// so the delay buffer cannot mask an imminent collision.
	if (bAEBActive)
	{
		DelayedLeaderDist  = LeaderDist;
		DelayedLeaderSpeed = LeaderSpeed;
	}

	// ── Brake light perception ──────────────────────────────
	// If the leader vehicle's brake lights are visible, bypass the reaction
	// delay and use real-time readings. This models the driver noticing the
	// red lights ahead and reacting faster than the default cognitive delay.
	if (!bAEBActive && LeaderDist >= 0.0f && IDMReactionDelaySec > KINDA_SMALL_NUMBER)
	{
		UTrafficSubsystem* BLSub = nullptr;
		if (UWorld* BLWorld = GetWorld())
		{
			BLSub = BLWorld->GetSubsystem<UTrafficSubsystem>();
		}
		if (BLSub)
		{
			const FVector Loc = ControlledPawn->GetActorLocation();
			const FVector Fwd = ControlledPawn->GetActorForwardVector();
			TArray<TWeakObjectPtr<ATrafficVehicleController>> LaneVehicles = BLSub->GetVehiclesOnLane(CurrentLane);
			float NearestAheadDist = MAX_FLT;
			const ATrafficVehicleController* NearestLeader = nullptr;
			for (const TWeakObjectPtr<ATrafficVehicleController>& WeakOther : LaneVehicles)
			{
				const ATrafficVehicleController* Other = WeakOther.Get();
				if (!Other || Other == this) { continue; }
				const APawn* OP = Other->GetPawn();
				if (!OP) { continue; }
				const FVector Delta = OP->GetActorLocation() - Loc;
				const float Dist = Delta.Size();
				if (Dist < KINDA_SMALL_NUMBER) { continue; }
				if (FVector::DotProduct(Delta / Dist, Fwd) < 0.5f) { continue; }
				if (Dist < NearestAheadDist)
				{
					NearestAheadDist = Dist;
					NearestLeader = Other;
				}
			}
			if (NearestLeader && NearestLeader->AreBrakeLightsOn())
			{
				// Leader is braking — use real-time data instead of delayed.
				DelayedLeaderDist  = LeaderDist;
				DelayedLeaderSpeed = LeaderSpeed;
			}
		}
	}

#if defined(ENABLE_DRAW_DEBUG) && ENABLE_DRAW_DEBUG
	DbgLeaderDist = DelayedLeaderDist;
	DbgLeaderSpeed = DelayedLeaderSpeed;
#endif

	// ── Step 2: Desired speed (IDM v₀) ──────────────────────
	// Start with road speed limit, then clamp by junction / intersection /
	// curve constraints. Leader following is handled by the IDM gap term.
	float EffectiveTargetSpeed = TargetSpeed;

	// Junction approach speed limit: progressively slow down as a junction
	// enters the scan range. Uses the decel-envelope speed computed by the
	// approach scan earlier this tick. This fires BEFORE the detection gate
	// so the vehicle decelerates smoothly instead of slamming the brakes.
	//
	// FIX (was CRITICAL): The old code used:
	//   EffectiveTargetSpeed = FMath::Max(JnctState.ApproachSpeedLimitCmPerSec, 100.0f);
	// This could RAISE speed above leader-following (e.g., leader = 80,
	// approach = 50 → max(50,100) = 100 → override leader to 100 → rear-end
	// collision). Now we only clamp DOWN, never up.
	if (bApproachBraking)
	{
		const float ApproachCap = FMath::Max(JnctState.ApproachSpeedLimitCmPerSec, 100.0f);
		if (ApproachCap < EffectiveTargetSpeed)
		{
			EffectiveTargetSpeed = ApproachCap;
		}
#if defined(ENABLE_DRAW_DEBUG) && ENABLE_DRAW_DEBUG
		DbgStateName = TEXT("APPROACH");
#endif
	}

	// ── Speed-zone anticipatory braking ──────────────────────
	// Look ahead along connected lanes for a speed limit reduction.
	// If the next lane has a lower limit, compute the comfortable
	// braking distance and start slowing before the zone boundary.
	if (CachedProvider && bLaneDataReady && JnctState.JunctionId == 0)
	{
		const float RemDist = GetRemainingDistance(FindClosestPointIndex(VehicleLocation));
		float ScanDist = RemDist;
		FTrafficLaneHandle ScanLane = CurrentLane;
		float NextSpeedLimit = -1.0f;

		// Walk up to 3 hops to find a lane with a different speed limit.
		for (int32 Hop = 0; Hop < 3 && NextSpeedLimit < 0.0f; ++Hop)
		{
			TArray<FTrafficLaneHandle> Connected = CachedProvider->GetConnectedLanes(ScanLane);
			if (Connected.IsEmpty()) { break; }
			// Sort for determinism, then pick first non-junction lane.
			Connected.Sort([](const FTrafficLaneHandle& A, const FTrafficLaneHandle& B)
			{ return A.HandleId < B.HandleId; });
			bool bFoundNext = false;
			for (const FTrafficLaneHandle& Next : Connected)
			{
				if (CachedProvider->GetJunctionForLane(Next) != 0) { continue; }
				const float Limit = CachedProvider->GetLaneSpeedLimit(Next);
				if (Limit > 0.0f && Limit < TargetSpeed - 50.0f)
				{
					// Found a lower-speed zone.
					NextSpeedLimit = Limit;
					bFoundNext = true;
					break;
				}
				// Continue scanning through this lane.
				ScanDist += CachedProvider->GetLaneLength(Next);
				ScanLane = Next;
				bFoundNext = true;
				break;
			}
			if (!bFoundNext) { break; }
		}

		if (NextSpeedLimit > 0.0f && AbsSpeed > NextSpeedLimit)
		{
			// Kinematic braking distance: d = (v² - v_target²) / (2 * decel)
			const float BrakeDist = (AbsSpeed * AbsSpeed - NextSpeedLimit * NextSpeedLimit)
				/ (2.0f * FMath::Max(ApproachDecelCmPerSec2, 100.0f));
			if (ScanDist < BrakeDist + ApproachSafetyMarginCm)
			{
				// Envelope speed: v = sqrt(v_target² + 2 * decel * remainingDist)
				const float EnvSpeed = FMath::Sqrt(
					NextSpeedLimit * NextSpeedLimit
					+ 2.0f * ApproachDecelCmPerSec2 * FMath::Max(ScanDist - ApproachSafetyMarginCm, 0.0f));
				if (EnvSpeed < EffectiveTargetSpeed)
				{
					EffectiveTargetSpeed = FMath::Max(EnvSpeed, NextSpeedLimit);
				}
			}
		}
	}

	// Intersection speed limit: cap speed while traversing a junction.
	// Uses the tunable IntersectionSpeedLimitCmPerSec as a baseline,
	// then modulates based on the junction path's curvature.
	//
	// FIX (was MAJOR): Replaced hardcoded 500 cm/s (~11 mph) that applied
	// uniformly to ALL intersections (highway merge, 4-way stop, etc.)
	// with a tunable property + curvature-derived speed so straight-through
	// traffic keeps moving and tight turns slow appropriately.
	if (JnctState.JunctionId != 0)
	{
		float JunctionSpeedCap = IntersectionSpeedLimitCmPerSec;

		// Derive speed from junction path curvature when available:
		// walk the junction transition polyline, measure the tightest
		// turning angle, convert to a centripetal-safe speed.
		if (JnctState.TransitionPoints.Num() >= 3)
		{
			// Compute total arc length and cumulative turn angle.
			// Start from JnctState.CurveStartIndex so prepended old-lane-tail
			// approach points don't dilute the curvature measurement.
			const int32 CurveStart = FMath::Clamp(JnctState.CurveStartIndex,
				0, JnctState.TransitionPoints.Num() - 3);
			float TotalAngleDeg = 0.0f;
			float TotalArcLength = 0.0f;
			for (int32 i = CurveStart + 1; i < JnctState.TransitionPoints.Num() - 1; ++i)
			{
				const FVector Seg0 = (JnctState.TransitionPoints[i] - JnctState.TransitionPoints[i - 1]);
				const FVector Seg1 = (JnctState.TransitionPoints[i + 1] - JnctState.TransitionPoints[i]);
				TotalArcLength += Seg0.Size();
				const FVector Dir0 = Seg0.GetSafeNormal();
				const FVector Dir1 = Seg1.GetSafeNormal();
				if (!Dir0.IsNearlyZero() && !Dir1.IsNearlyZero())
				{
					const float Dot = FMath::Clamp(FVector::DotProduct(Dir0, Dir1), -1.0f, 1.0f);
					TotalAngleDeg += FMath::RadiansToDegrees(FMath::Acos(Dot));
				}
			}
			// Add last segment length.
			TotalArcLength += (JnctState.TransitionPoints.Last() - JnctState.TransitionPoints[JnctState.TransitionPoints.Num() - 2]).Size();

			// Approximate turning radius: R = arcLength / totalAngle(radians).
			// Then centripetal speed limit: v = sqrt(lateralAccel * R).
			const float TotalAngleRad = FMath::DegreesToRadians(FMath::Max(TotalAngleDeg, 1.0f));
			const float TurnRadius = FMath::Max(TotalArcLength / TotalAngleRad, 50.0f); // clamp to 0.5m min

			const float CurvatureSpeed = FMath::Sqrt(LateralAccelBudgetCmPerSec2 * TurnRadius);

			// Use the tighter of curvature-derived and baseline limit.
			JunctionSpeedCap = FMath::Min(JunctionSpeedCap, CurvatureSpeed);

			UE_LOG(LogAAATraffic, Verbose,
				TEXT("JNCT SPEED-CAP: CurveStart=%d pts=%d arc=%.0f angle=%.1f° R=%.0f speed=%.0f cap=%.0f"),
				CurveStart, JnctState.TransitionPoints.Num(),
				TotalArcLength, TotalAngleDeg, TurnRadius,
				CurvatureSpeed, JunctionSpeedCap);
		}

		EffectiveTargetSpeed = FMath::Min(EffectiveTargetSpeed, JunctionSpeedCap);
	}

	// ── Predictive curve speed reduction ─────────────────────────
	// Scan upcoming polyline curvature and compute a physics-safe
	// speed limit: v = sqrt(lateralAccelBudget * turningRadius).
	// Uses a sliding window of CurveScanWindowSize segments to
	// accumulate curvature — matching the RoadBLD static classifier
	// approach. Single-vertex angles on dense polylines are tiny
	// even on tight curves; the window accumulates them correctly.
	//
	// After finding the tightest curve and its distance, a kinematic
	// braking envelope ensures the vehicle starts decelerating early
	// enough: v_now = sqrt(v_curve² + 2 · decel · distToCurve).
	// This mirrors the proven junction-approach pattern.
	if (LanePoints.Num() >= 3 && LastClosestIndex < LanePoints.Num() - 2)
	{
		const float StoppingDistForScan = (AbsSpeed > KINDA_SMALL_NUMBER)
			? (AbsSpeed * AbsSpeed) / (2.0f * MaxBrakeDecelCmPerSec2)
			: 0.0f;

		// On first tick after lane transition, scan the FULL polyline so the
		// braking envelope covers curves anywhere on the lane — not just
		// the normal speed-based look-ahead range that may be too short
		// at low transition speeds.
		float ScanRange;
		if (bFirstTickOnLane)
		{
			// Scan entire remaining lane.
			ScanRange = TNumericLimits<float>::Max();
			bFirstTickOnLane = false;
		}
		else
		{
			ScanRange = FMath::Max(StoppingDistForScan + LookAheadDistance, 800.0f);
		}
		const int32 CurveStart = FMath::Clamp(LastClosestIndex, 0, LanePoints.Num() - 2);
		const int32 WinSize = FMath::Max(CurveScanWindowSize, 2);

		// Last valid index for polyline segment start: we need at least
		// i+1 to form a segment, so stop at Num()-2.
		const int32 LastSegIdx = LanePoints.Num() - 2;

		// Pre-compute cumulative distance from CurveStart for each point
		// so we can look up the distance to any window start cheaply.
		// Only compute up to what we need (scan range worth of segments).
		TArray<float, TInlineAllocator<128>> CumDist;
		{
			float Accum = 0.0f;
			CumDist.Add(0.0f); // CumDist[0] = distance from CurveStart to CurveStart = 0
			for (int32 i = CurveStart; i < LastSegIdx && Accum < ScanRange + 2000.0f; ++i)
			{
				Accum += FVector::Dist(LanePoints[i], LanePoints[i + 1]);
				CumDist.Add(Accum);
			}
		}
		const int32 NumScannableSegs = CumDist.Num() - 1; // segments we have distance data for

		float MinTurnRadius = TNumericLimits<float>::Max();
		float DistToTightestCurve = 0.0f;

		// Slide a window of WinSize segments across the scannable range.
		for (int32 w = 0; w < NumScannableSegs; ++w)
		{
			const float WinStartDist = CumDist[w];
			if (WinStartDist > ScanRange) { break; }

			const int32 WinStartIdx = CurveStart + w;
			const int32 WinEndIdx = FMath::Min(WinStartIdx + WinSize, LastSegIdx);

			if (WinEndIdx <= WinStartIdx) { break; }

			// Accumulate angle and arc over the window segments.
			float WinAngleRad = 0.0f;
			float WinArc = 0.0f;
			for (int32 i = WinStartIdx; i < WinEndIdx; ++i)
			{
				const FVector Seg = LanePoints[i + 1] - LanePoints[i];
				WinArc += Seg.Size();

				// Angle between consecutive segments (needs i+2).
				if (i + 2 <= LastSegIdx + 1) // i+2 is a valid point index
				{
					const FVector NextSeg = LanePoints[i + 2] - LanePoints[i + 1];
					const FVector Dir0 = Seg.GetSafeNormal();
					const FVector Dir1 = NextSeg.GetSafeNormal();
					if (!Dir0.IsNearlyZero() && !Dir1.IsNearlyZero())
					{
						const float Dot = FMath::Clamp(FVector::DotProduct(Dir0, Dir1), -1.0f, 1.0f);
						WinAngleRad += FMath::Acos(Dot);
					}
				}
			}

			// ~3° minimum to filter near-straight windows.
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
			// Physics-safe curve speed with safety margin.
			const float RawCurveSpeed = FMath::Sqrt(LateralAccelBudgetCmPerSec2 * MinTurnRadius);
			const float CurveSpeedLimit = RawCurveSpeed * CurveSpeedSafetyFactor;

			// Kinematic braking envelope: the maximum speed NOW that still
			// allows decelerating to CurveSpeedLimit by the time the curve
			// is reached. v_now = sqrt(v_curve² + 2 · decel · dist).
			// Uses comfort deceleration so braking looks natural.
			const float Decel = IDMComfortDecelCmPerSec2;
			const float Dist = FMath::Max(DistToTightestCurve, 0.0f);
			const float EnvelopeSpeed = FMath::Sqrt(
				CurveSpeedLimit * CurveSpeedLimit + 2.0f * Decel * Dist);

			if (EnvelopeSpeed < EffectiveTargetSpeed)
			{
				EffectiveTargetSpeed = EnvelopeSpeed;
			}
		}
	}

	// NOTE: TurnSpeedScale (I6) removed — the predictive curve speed
	// reduction above already limits speed via physics-based lateral accel
	// budget. The redundant steering-magnitude multiplier was crushing
	// speed to 10% at full lock on top of the already-reduced curve speed.

	// ── Merge cooperation ────────────────────────────────────
	// If a vehicle on an adjacent lane is actively lane-changing toward
	// this vehicle's lane AND is alongside (within detection range),
	// reduce effective speed to create a merge gap.
	if (CachedProvider && bLaneDataReady && LaneChangeState == ELaneChangeState::None)
	{
		UWorld* MergeWorld = GetWorld();
		UTrafficSubsystem* MergeSub = MergeWorld ? MergeWorld->GetSubsystem<UTrafficSubsystem>() : nullptr;
		if (MergeSub)
		{
			// Check both adjacent lanes.
			for (ETrafficLaneSide Side : { ETrafficLaneSide::Left, ETrafficLaneSide::Right })
			{
				FTrafficLaneHandle AdjLane = CachedProvider->GetAdjacentLane(CurrentLane, Side);
				if (!AdjLane.IsValid()) { continue; }

				TArray<TWeakObjectPtr<ATrafficVehicleController>> AdjVehicles = MergeSub->GetVehiclesOnLane(AdjLane);
				for (const TWeakObjectPtr<ATrafficVehicleController>& WeakAdj : AdjVehicles)
				{
					ATrafficVehicleController* Adj = WeakAdj.Get();
					if (!Adj || Adj == this) { continue; }
					// Is the adjacent vehicle merging toward our lane?
					if (Adj->LaneChangeState != ELaneChangeState::Executing) { continue; }
					if (Adj->TargetLaneHandle != CurrentLane) { continue; }
					// Is it alongside us (within ±DetectionDistance)?
					const APawn* AdjPawn = Adj->GetPawn();
					if (!AdjPawn) { continue; }
					const float LongDist = FVector::DotProduct(
						AdjPawn->GetActorLocation() - VehicleLocation,
						ControlledPawn->GetActorForwardVector());
					// Adjacent vehicle is ahead but close — slow down to create gap.
					if (LongDist > -200.0f && LongDist < DetectionDistance * 0.5f)
					{
						// Cooperative decel: reduce target by 15%.
						EffectiveTargetSpeed *= 0.85f;
						break;
					}
				}
			}
		}
	}

	// Guard against EffectiveTargetSpeed == 0 to prevent division by zero.
	if (EffectiveTargetSpeed <= KINDA_SMALL_NUMBER)
	{
		VehicleMovement->SetThrottleInput(0.0f);
		VehicleMovement->SetSteeringInput(SteeringInput);
		VehicleMovement->SetBrakeInput(1.0f);
		return;
	}

	// ── Step 3: IDM+ acceleration (Treiber & Kesting 2013) ──────
	// Improved IDM that handles v > v₀ gracefully. When a speed cap
	// drops v₀ below current speed, standard IDM double-penalizes
	// via (v/v₀)⁴ > 1 plus the gap term. IDM+ separates these:
	//   If v ≤ v₀: standard IDM   ẍ = a[1 - (v/v₀)⁴ - (s*/s)²]
	//   If v > v₀:
	//     Free-road: ẍ = -a(v/v₀)⁴ - 1)    (decel only by speed excess)
	//     With leader: ẍ = a[1 - (s*/s)²]   if that's worse, else free-road
	// Uses delayed leader state for reaction delay, and personality-scaled
	// parameters for per-vehicle variation.
	float IDMAccel;
	{
		const float v  = FMath::Max(FMath::Abs(CurrentSpeed), 0.0f);
		const float v0 = EffectiveTargetSpeed;          // desired speed

		// Apply per-vehicle personality scaling.
		const float a  = IDMMaxAccelCmPerSec2     * IDMPersonalityAccelScale;
		const float b  = IDMComfortDecelCmPerSec2 * IDMPersonalityDecelScale;
		const float T  = FollowingTimeSec         * IDMPersonalityTimeHeadwayScale;

		// Queue compaction: when both ego and leader are nearly stopped
		// (e.g. waiting at a red light), reduce jam distance to allow
		// tighter bumper-to-bumper queuing. Blends from full s₀ at
		// 300 cm/s down to half s₀ at standstill.
		float s0 = MinFollowingDistanceCm;
		if (DelayedLeaderDist >= 0.0f && v < 300.0f && DelayedLeaderSpeed < 300.0f)
		{
			const float QueueBlend = FMath::Clamp(FMath::Max(v, DelayedLeaderSpeed) / 300.0f, 0.0f, 1.0f);
			s0 = FMath::Lerp(MinFollowingDistanceCm * 0.5f, MinFollowingDistanceCm, QueueBlend);
		}

		// Free-road term.
		const float vRatio   = v / FMath::Max(v0, 1.0f);
		const float vRatio2  = vRatio * vRatio;
		const float FreeRoad = vRatio2 * vRatio2;  // (v/v₀)⁴

		if (DelayedLeaderDist >= 0.0f)
		{
			// Car-following: compute desired minimum gap s*.
			const float s      = FMath::Max(DelayedLeaderDist, 1.0f);
			const float DeltaV = v - DelayedLeaderSpeed;

			const float Interaction = v * DeltaV
				/ (2.0f * FMath::Sqrt(FMath::Max(a * b, 1.0f)));
			const float DesiredGap  = s0 + FMath::Max(0.0f, v * T + Interaction);
			const float GapRatio    = DesiredGap / s;
			const float GapTerm     = GapRatio * GapRatio;

			// IDM+ branching: avoid double-penalty when v > v₀.
			if (v <= v0)
			{
				IDMAccel = a * (1.0f - FreeRoad - GapTerm);
			}
			else
			{
				// Over desired speed: use whichever demands more deceleration.
				const float AccelFree = -a * (FreeRoad - 1.0f);  // ≤ 0
				const float AccelGap  = a * (1.0f - GapTerm);     // may be + or -
				IDMAccel = FMath::Min(AccelFree, AccelGap);
			}
		}
		else
		{
			// Free-road: no leader detected.
			if (v <= v0)
			{
				IDMAccel = a * (1.0f - FreeRoad);
			}
			else
			{
				// Over desired speed — brake proportionally.
				IDMAccel = -a * (FreeRoad - 1.0f);
			}
		}
	}

	// ── Apply acceleration smoothing ─────────────────────────
	// Exponential moving average prevents throttle/brake oscillation
	// at the equilibrium gap distance where IDM output hovers near zero.
	if (IDMSmoothingTauSec > KINDA_SMALL_NUMBER)
	{
		const float Alpha = DeltaSeconds / (DeltaSeconds + IDMSmoothingTauSec);
		SmoothedIDMAccel = FMath::Lerp(SmoothedIDMAccel, IDMAccel, Alpha);
		IDMAccel = SmoothedIDMAccel;
	}

	// ── Step 4: Map IDM acceleration to throttle/brake ───────
	// N7 FIX: Dead zone around zero prevents oscillatory throttle↔brake
	// switching at equilibrium. Accel within ±5% of max is coasted.
	constexpr float IDMDeadZoneFraction = 0.05f;
	const float IDMDeadZone = IDMMaxAccelCmPerSec2 * IDMDeadZoneFraction;

	float ThrottleInput = 0.0f;
	float BrakeInput = 0.0f;
	if (IDMAccel > IDMDeadZone)
	{
		ThrottleInput = FMath::Clamp(IDMAccel / FMath::Max(IDMMaxAccelCmPerSec2, 1.0f),
			0.0f, 1.0f);
	}
	else if (IDMAccel < -IDMDeadZone)
	{
		// I3 FIX: Normalize against the vehicle-specific MaxBrakeDecelCmPerSec2
		// for emergency braking (values beyond comfortable decel). This uses
		// real brake torque data from the Chaos wheel setup rather than a
		// fixed comfortable decel value, allowing proper hard stops.
		const float BrakeNorm = (IDMAccel < -IDMComfortDecelCmPerSec2)
			? MaxBrakeDecelCmPerSec2   // emergency: use full brake capability
			: IDMComfortDecelCmPerSec2; // comfort: normal IDM range
		BrakeInput = FMath::Clamp(-IDMAccel / FMath::Max(BrakeNorm, 1.0f),
			0.0f, 1.0f);
	}
	// else: within dead zone → coast (throttle=0, brake=0)

	// N5 FIX: Minimum throttle nudge at near-standstill to overcome
	// Chaos static friction. Without this vehicles stall when IDM
	// commands a tiny positive acceleration insufficient to move.
	if (IDMAccel > 0.0f && FMath::Abs(CurrentSpeed) < 50.0f && ThrottleInput < 0.08f)
	{
		ThrottleInput = 0.08f;
	}

	// ── Brake light state ────────────────────────────────────
	// Lights on when actively braking (beyond dead zone) or at standstill
	// waiting at intersection/stop-sign.
	{
		const bool bShouldBrake = (BrakeInput > 0.0f) ||
			(JnctState.bWaiting && FMath::Abs(CurrentSpeed) < 100.0f);
		SetBrakeLights(bShouldBrake);
	}

	// AEB hard override — when TTC is below threshold, force max braking
	// regardless of IDM output. This is the last safety layer.
	if (bAEBActive)
	{
		ThrottleInput = 0.0f;
		BrakeInput    = 1.0f;
	}

	// I6 FIX: Turn-speed reduction factored into EffectiveTargetSpeed
	// (before IDM) is handled above in the speed cap section. The
	// old post-IDM throttle scaling violated IDM equilibrium assumptions
	// and is removed.

	// Wake guard: keep the GT-level vehicle movement awake so throttle input
	// takes effect. Throttled to once per second to avoid per-tick energy
	// injection that was causing flying-car physics explosions.
	// NOTE: Chaos-level NeverSleep is already set once in OnPossess.
	// This guard only handles the GT SetSleeping/SetParked path.
	WakeGuardCheckTimer += DeltaSeconds;
	if (WakeGuardCheckTimer >= 1.0f
		&& ThrottleInput >= GTrafficWakeGuardMinThrottle
		&& BrakeInput <= KINDA_SMALL_NUMBER
		&& FMath::Abs(CurrentSpeed) <= GTrafficWakeGuardMaxSpeed)
	{
		WakeGuardCheckTimer = 0.0f;
		VehicleMovement->SetParked(false);
		VehicleMovement->SetSleeping(false);
		VehicleMovement->SetHandbrakeInput(false);
	}

	// Reactive collision handler — if we physically collided, override to
	// full braking for the duration of CollisionBrakeTimer.
	if (CollisionBrakeTimer > 0.0f)
	{
		CollisionBrakeTimer -= DeltaSeconds;
		ThrottleInput = 0.0f;
		BrakeInput    = 1.0f;
	}

	VehicleMovement->SetThrottleInput(ThrottleInput);
	VehicleMovement->SetSteeringInput(SteeringInput);
	VehicleMovement->SetBrakeInput(BrakeInput);

	// --- Velocity sanity clamp ---
	// If the physics body somehow exceeds MaxAllowedSpeedCmPerSec (e.g. from
	// a collision penetration-resolution impulse), zero the velocity entirely.
	// This is a last-resort safety net — normal driving never reaches this.
	if (UPrimitiveComponent* Prim = VehicleMovement->UpdatedPrimitive)
	{
		if (FBodyInstance* BI = Prim->GetBodyInstance())
		{
			const FVector LinearVel = BI->GetUnrealWorldVelocity();
			if (LinearVel.SizeSquared() > MaxAllowedSpeedCmPerSec * MaxAllowedSpeedCmPerSec)
			{
				BI->SetLinearVelocity(FVector::ZeroVector, false);
				BI->SetAngularVelocityInRadians(FVector::ZeroVector, false);
				UE_LOG(LogAAATraffic, Warning,
					TEXT("SAFETY VELOCITY-CLAMP: Pawn='%s' had runaway speed %.0f cm/s (max %.0f) — zeroed"),
					GetPawn() ? *GetPawn()->GetName() : TEXT("NULL"),
					LinearVel.Size(), MaxAllowedSpeedCmPerSec);
			}
		}
	}

#if defined(ENABLE_DRAW_DEBUG) && ENABLE_DRAW_DEBUG
	DbgThrottle = ThrottleInput;
	DbgBrake = BrakeInput;
	DbgSteering = SteeringInput;
	DbgEffectiveTargetSpeed = EffectiveTargetSpeed;
	DbgTargetPoint = TargetPoint;
	// State priority: junction > following > braking > cruising.
	if (JnctState.JunctionId != 0)
	{
		// Distinguish: "IN-JNCT" when on the actual junction lane,
		// "INT" when holding occupancy but on an approach or exit lane.
		const int32 CurrLaneJnct = CachedProvider
			? CachedProvider->GetJunctionForLane(CurrentLane) : 0;
		DbgStateName = (CurrLaneJnct == JnctState.JunctionId)
			? TEXT("IN-JNCT") : TEXT("INT");
	}
	else if (DbgLeaderDist >= 0.0f && BrakeInput > 0.05f)
	{
		DbgStateName = TEXT("FOLLOWING");
	}
	else if (BrakeInput > 0.05f)
	{
		DbgStateName = TEXT("BRAKING");
	}
#endif

	// --- Diagnostic: log first tick's driving values per vehicle ---
	if (!bDiagLoggedFirstInput)
	{
		bDiagLoggedFirstInput = true;
		UE_LOG(LogAAATraffic, Log,
			TEXT("VehicleController::UpdateVehicleInput FIRST TICK: Pawn='%s' "
				 "TargetSpeed=%.1f EffectiveTarget=%.1f CurrentSpeed=%.1f "
				 "Throttle=%.3f Steering=%.3f Brake=%.3f "
				 "bAtDeadEnd=%s JnctState.bWaiting=%s LanePoints=%d ClosestIdx=%d"),
			*ControlledPawn->GetName(),
			TargetSpeed, EffectiveTargetSpeed, CurrentSpeed,
			ThrottleInput, SteeringInput, BrakeInput,
			bAtDeadEnd ? TEXT("true") : TEXT("false"),
			JnctState.bWaiting ? TEXT("true") : TEXT("false"),
			LanePoints.Num(), ClosestIndex);
	}
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
	// Ld→0 as the vehicle approaches, causing a steering spike.
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

void ATrafficVehicleController::AddLaneDecisionTrace(const TCHAR* EventName, int32 CandidateLane, float MetricA, float MetricB, const FString& Detail)
{
	if (!IsVehicleTraceEnabled(1))
	{
		return;
	}

	LaneDecisionTraceMaxEntries = FMath::Max(32, GTrafficVehicleDecisionTraceMax);
	while (LaneDecisionTraceBuffer.Num() >= LaneDecisionTraceMaxEntries)
	{
		LaneDecisionTraceBuffer.RemoveAt(0, 1, EAllowShrinking::No);
	}

	FLaneDecisionTrace Record;
	if (const UWorld* World = GetWorld())
	{
		Record.WorldTimeSeconds = World->GetTimeSeconds();
	}
	Record.EventName = EventName;
	Record.CurrentLaneId = CurrentLane.HandleId;
	Record.CandidateLaneId = CandidateLane;
	Record.MetricA = MetricA;
	Record.MetricB = MetricB;
	Record.Detail = Detail;

	LaneDecisionTraceBuffer.Add(MoveTemp(Record));
}

void ATrafficVehicleController::FlushLaneDecisionTrace(const TCHAR* Reason, bool bAsWarning)
{
	if (!IsVehicleTraceEnabled(1) || LaneDecisionTraceBuffer.Num() == 0)
	{
		return;
	}

	if (bAsWarning)
	{
		UE_LOG(LogAAATraffic, Warning,
			TEXT("TrafficVehicleController: DecisionTraceFlush Reason=%s Entries=%d Pawn=%s CurrentLane=%d"),
			Reason,
			LaneDecisionTraceBuffer.Num(),
			GetPawn() ? *GetPawn()->GetName() : TEXT("NULL"),
			CurrentLane.HandleId);
	}
	else
	{
		UE_LOG(LogAAATraffic, Log,
			TEXT("TrafficVehicleController: DecisionTraceFlush Reason=%s Entries=%d Pawn=%s CurrentLane=%d"),
			Reason,
			LaneDecisionTraceBuffer.Num(),
			GetPawn() ? *GetPawn()->GetName() : TEXT("NULL"),
			CurrentLane.HandleId);
	}

	for (const FLaneDecisionTrace& Record : LaneDecisionTraceBuffer)
	{
		if (bAsWarning)
		{
			UE_LOG(LogAAATraffic, Warning,
				TEXT("  [%.2f] Event=%s Cur=%d Cand=%d A=%.3f B=%.3f Detail=%s"),
				Record.WorldTimeSeconds,
				*Record.EventName,
				Record.CurrentLaneId,
				Record.CandidateLaneId,
				Record.MetricA,
				Record.MetricB,
				Record.Detail.IsEmpty() ? TEXT("-") : *Record.Detail);
		}
		else
		{
			UE_LOG(LogAAATraffic, Log,
				TEXT("  [%.2f] Event=%s Cur=%d Cand=%d A=%.3f B=%.3f Detail=%s"),
				Record.WorldTimeSeconds,
				*Record.EventName,
				Record.CurrentLaneId,
				Record.CandidateLaneId,
				Record.MetricA,
				Record.MetricB,
				Record.Detail.IsEmpty() ? TEXT("-") : *Record.Detail);
		}
	}

	LaneDecisionTraceBuffer.Reset();
}

// ---------------------------------------------------------------------------
// Curvature computation — Menger curvature from polyline points
// ---------------------------------------------------------------------------
float ATrafficVehicleController::ComputeLocalCurvature(
	const TArray<FVector>& Points, int32 CenterIndex) const
{
	if (Points.Num() < 3) { return 0.0f; }

	// Widen point spacing to suppress noise from 100cm polyline sampling.
	// Use half CurveScanWindowSize for spread (default CurveScanWindowSize=5 → spread=2).
	const int32 Spread = FMath::Max(CurveScanWindowSize / 2, 1);
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

	// Signed area of triangle (2D cross product / 2).
	// Positive = A→B→C turns left (counterclockwise).
	const float SignedArea2 = (B.X - A.X) * (C.Y - A.Y) - (B.Y - A.Y) * (C.X - A.X);
	return (2.0f * SignedArea2) / Denom;
}

void ATrafficVehicleController::CheckLaneTransition()
{
	// Guard: never re-enter while already handling a dead-end.
	// The caller (lane-end gate) fires every tick once RemainingDist drops
	// below the threshold, which would trigger this repeatedly.
	if (bAtDeadEnd)
	{
		return;
	}

	AddLaneDecisionTrace(TEXT("TransitionCheck.Start"), 0, static_cast<float>(CurrentLane.HandleId), 0.0f, TEXT("Entering lane-end transition check"));

	if (!CachedProvider)
	{
		bAtDeadEnd = true;
		SetTurnSignal(ETurnSignalState::Hazard);
		AddLaneDecisionTrace(TEXT("TransitionCheck.NoProvider"), 0, 0.0f, 0.0f, TEXT("No cached provider"));
		FlushLaneDecisionTrace(TEXT("NoProviderDeadEnd"), true);
		UE_LOG(LogAAATraffic, Log,
			TEXT("TrafficVehicleController: No provider cached — dead end on lane %d."),
			CurrentLane.HandleId);
		return;
	}

	TArray<FTrafficLaneHandle> Connected = CachedProvider->GetConnectedLanes(CurrentLane);

	// ── One-way / turn-restriction filter ────────────────────
	// Remove exit lanes that flow backward relative to the approach
	// direction (would place the vehicle on a wrong-way / one-way road).
	// A lane whose direction dot product with the approach tangent is
	// strongly negative (< -0.3) is flowing back toward us — reject it.
	if (Connected.Num() > 1)
	{
		const float CurLaneLen = CachedProvider->GetLaneLength(CurrentLane);
		const FVector ApproachTangent = CachedProvider->GetLaneDirectionAtDistance(CurrentLane, CurLaneLen);
		TArray<FTrafficLaneHandle> Filtered;
		Filtered.Reserve(Connected.Num());
		for (const FTrafficLaneHandle& C : Connected)
		{
			const FVector ExitDir = CachedProvider->GetLaneDirection(C);
			if (FVector::DotProduct(ApproachTangent, ExitDir) >= 0.0f)
			{
				Filtered.Add(C);
			}
		}
		// Only apply filter if it leaves at least one valid exit.
		if (Filtered.Num() > 0)
		{
			Connected = MoveTemp(Filtered);
		}
	}

	// One-way filter above already handled directional filtering.
	// No additional lane-permission filter needed — surveyor table covers it.

	AddLaneDecisionTrace(TEXT("TransitionCheck.ConnectedFetched"), 0, static_cast<float>(Connected.Num()), 0.0f, TEXT("Fetched connected lanes"));
	if (Connected.IsEmpty())
	{
		// Dead-end: no forward connections from this lane. Stop the vehicle
		// and let the dead-end despawn timer in UpdateVehicleInput recycle it.
		// This handles map-edge roads, cul-de-sacs, and any topology gap.
		bAtDeadEnd = true;
		DeadEndDespawnTimer = 0.0f;
		SetTurnSignal(ETurnSignalState::Hazard);
		AddLaneDecisionTrace(TEXT("TransitionCheck.DeadEnd"), 0, 0.0f, 0.0f, TEXT("No connected lanes — dead end, will despawn"));
		FlushLaneDecisionTrace(TEXT("DeadEndDespawn"), true);
		UE_LOG(LogAAATraffic, Warning,
			TEXT("TrafficVehicleController: Lane %d is a dead-end (no forward connections). "
				 "Vehicle will stop and despawn in %.1fs."),
			CurrentLane.HandleId, DeadEndDespawnDelaySec);
		return;
	}

	// --- Weighted lane selection (Feature 4) ---
	// If a junction exit was pre-selected at detection time (JnctState.ToLane),
	// reuse it to guarantee conflict detection and actual driving agree.
	// Otherwise fall back to weighted random selection.
	FTrafficLaneHandle NextLane;

	// Check if the pre-selected exit is valid and present in the connected list.
	bool bUsedPreselected = false;
	if (JnctState.ToLane.HandleId != 0
		&& JnctState.ToLane.HandleId != CurrentLane.HandleId)
	{
		for (const FTrafficLaneHandle& C : Connected)
		{
			if (C.HandleId == JnctState.ToLane.HandleId)
			{
				NextLane = JnctState.ToLane;
				bUsedPreselected = true;
				AddLaneDecisionTrace(TEXT("TransitionCheck.UsedPreselectedExit"), NextLane.HandleId, 0.0f, 0.0f, TEXT("Reused junction pre-selected exit"));
				break;
			}
		}
	}

	// ── Turn bypass: if the pre-selected exit lane is a cross-junction turn,
	// it won't appear in Connected (which only has the directly-connected
	// junction lane for straight-through).  Detect this and skip the junction
	// lane, transitioning directly to the departure exit.  The Hermite curve
	// generator downstream will bridge the gap from approach to exit. ──
	if (!bUsedPreselected && JnctState.ToLane.HandleId != 0
		&& JnctState.ToLane.HandleId != CurrentLane.HandleId
		&& CachedProvider)
	{
		bool bReachableViaDirectJunction = false;
		for (const FTrafficLaneHandle& C : Connected)
		{
			if (CachedProvider->GetJunctionForLane(C) != 0)
			{
				TArray<FTrafficLaneHandle> JExits = CachedProvider->GetConnectedLanes(C);
				for (const FTrafficLaneHandle& E : JExits)
				{
					if (E.HandleId == JnctState.ToLane.HandleId)
					{
						bReachableViaDirectJunction = true;
						break;
					}
				}
			}
			if (bReachableViaDirectJunction) { break; }
		}

		if (!bReachableViaDirectJunction)
		{
			// This is a TURN — the exit lane belongs to a different road's
			// junction lane.  Bypass the intermediate junction lane entirely.
			NextLane = JnctState.ToLane;
			bUsedPreselected = true;
			UE_LOG(LogAAATraffic, Warning,
				TEXT("JNCT TURN-BYPASS: Pawn='%s' Approach=%d -> Exit=%d "
					 "(skipping junction lane, will generate turn curve)"),
				GetPawn() ? *GetPawn()->GetName() : TEXT("NULL"),
				CurrentLane.HandleId, JnctState.ToLane.HandleId);
			AddLaneDecisionTrace(TEXT("TransitionCheck.TurnBypass"), NextLane.HandleId, 0.0f, 0.0f, TEXT("Cross-junction turn: bypassed junction lane"));
		}
	}

	if (!bUsedPreselected)
	{
	// Use centralized surveyor for fallback exit selection.
	NextLane = PickSurveyedExit(CurrentLane, Connected);
	AddLaneDecisionTrace(TEXT("TransitionCheck.SurveyorPick"), NextLane.HandleId, 0.0f, 0.0f, TEXT("Fallback via surveyor"));

	// Exit-side lane matching: align to the same lateral position on the
	// exit road as the vehicle occupied on the approach road. Prevents
	// ending up in the wrong lane on multi-lane exit roads.
	//
	// IMPORTANT: adjacency maps can cross the centre line on 2-way roads
	// (opposing lanes share a boundary curve). Every adjacency step must
	// validate that the neighbor faces the same direction (dot > 0.5);
	// stop immediately if it doesn't.
	if (CachedProvider)
	{
		// Count approach lane's index from right side (same-direction only).
		const FVector ApproachRefDir = CachedProvider->GetLaneDirection(CurrentLane);
		int32 ApproachRightIdx = 0;
		{
			FTrafficLaneHandle Walk = CachedProvider->GetAdjacentLane(CurrentLane, ETrafficLaneSide::Right);
			for (int32 S = 0; S < 8 && Walk.IsValid(); ++S)
			{
				const FVector WalkDir = CachedProvider->GetLaneDirection(Walk);
				if (FVector::DotProduct(ApproachRefDir, WalkDir) < 0.5f) { break; }
				++ApproachRightIdx;
				Walk = CachedProvider->GetAdjacentLane(Walk, ETrafficLaneSide::Right);
			}
		}

		// Walk NextLane to rightmost same-direction exit lane.
		const FVector ExitRefDir = CachedProvider->GetLaneDirection(NextLane);
		FTrafficLaneHandle RightmostExit = NextLane;
		{
			FTrafficLaneHandle Walk = CachedProvider->GetAdjacentLane(NextLane, ETrafficLaneSide::Right);
			for (int32 S = 0; S < 8 && Walk.IsValid(); ++S)
			{
				const FVector WalkDir = CachedProvider->GetLaneDirection(Walk);
				if (FVector::DotProduct(ExitRefDir, WalkDir) < 0.5f) { break; }
				RightmostExit = Walk;
				Walk = CachedProvider->GetAdjacentLane(Walk, ETrafficLaneSide::Right);
			}
		}

		// Walk left from rightmost by ApproachRightIdx to mirror position.
		FTrafficLaneHandle MatchedLane = RightmostExit;
		{
			FTrafficLaneHandle Walk = RightmostExit;
			for (int32 i = 0; i < ApproachRightIdx; ++i)
			{
				const FTrafficLaneHandle Left = CachedProvider->GetAdjacentLane(Walk, ETrafficLaneSide::Left);
				if (!Left.IsValid()) { break; }
				const FVector LeftDir = CachedProvider->GetLaneDirection(Left);
				if (FVector::DotProduct(ExitRefDir, LeftDir) < 0.5f) { break; }
				Walk = Left;
				MatchedLane = Walk;
			}
		}

		// Only override if matched lane is different AND reachable (in Connected list).
		if (MatchedLane.HandleId != NextLane.HandleId)
		{
			bool bMatchedIsConnected = false;
			for (const FTrafficLaneHandle& C : Connected)
			{
				if (C.HandleId == MatchedLane.HandleId)
				{
					bMatchedIsConnected = true;
					break;
				}
			}
			if (bMatchedIsConnected)
			{
				AddLaneDecisionTrace(TEXT("TransitionCheck.ExitLaneMatched"),
					MatchedLane.HandleId,
					static_cast<float>(ApproachRightIdx), 0.0f,
					FString::Printf(TEXT("Overrode %d -> %d to match approach lateral pos"),
						NextLane.HandleId, MatchedLane.HandleId));
				NextLane = MatchedLane;
			}
		}
	}

	AddLaneDecisionTrace(
		TEXT("TransitionCheck.SelectedNextLane"),
		NextLane.HandleId,
		0.0f,
		0.0f,
		FString::Printf(TEXT("CandidateCount=%d"), Connected.Num()));
	} // end !bUsedPreselected

	UE_LOG(LogAAATraffic, Log,
		TEXT("TrafficVehicleController: Transitioning from lane %d to lane %d (%d candidates)."),
		CurrentLane.HandleId, NextLane.HandleId, Connected.Num());

	// --- Junction smoothing (Feature 3) ---
	// Cache the end of the current lane before switching, so we can synthesize
	// a smooth transition curve from old-lane-end to new-lane-start.
	FVector OldLaneEnd = FVector::ZeroVector;
	FVector OldLaneTangent = FVector::ZeroVector;
	if (LanePoints.Num() >= 2)
	{
		OldLaneEnd = LanePoints.Last();
		// Average the last few segments for robust tangent direction.
		// A single short/noisy final segment can produce an inaccurate
		// tangent, causing S-curves at the junction entry.
		const int32 NumAvgOld = FMath::Min(3, LanePoints.Num() - 1);
		FVector AvgOldDir = FVector::ZeroVector;
		for (int32 k = LanePoints.Num() - NumAvgOld; k < LanePoints.Num(); ++k)
		{
			AvgOldDir += (LanePoints[k] - LanePoints[k - 1]);
		}
		OldLaneTangent = AvgOldDir.GetSafeNormal();
	}

	// Check if provider has a junction path.
	TArray<FVector> ProviderJunctionPath;
	const bool bHasProviderPath = CachedProvider && CachedProvider->GetJunctionPath(CurrentLane, NextLane, ProviderJunctionPath) && ProviderJunctionPath.Num() >= 2;

	// ── Save old-lane tail for polyline continuity ──────────────────
	// The transition gate fires with TransitionThreshold (often 800–1500 cm)
	// of remaining distance, so the vehicle is physically still ON the old
	// lane's polyline.  Save the tail from the vehicle's current position
	// so we can prepend it to the new lane after init, giving the vehicle
	// continuous CTE reference data from its current location through the
	// lane boundary into the new lane.
	TArray<FVector> OldLaneTail;
	{
		const int32 OldIdx = FMath::Clamp(LastClosestIndex, 0,
			FMath::Max(LanePoints.Num() - 1, 0));
		OldLaneTail.Reserve(LanePoints.Num() - OldIdx);
		for (int32 k = OldIdx; k < LanePoints.Num(); ++k)
		{
			OldLaneTail.Add(LanePoints[k]);
		}
	}
	const float SavedHeadingCrossZ = PreviousHeadingCrossZ;

	if (GTrafficJunctionDiagnostics >= 1)
	{
		UE_LOG(LogAAATraffic, Log,
			TEXT("JNCT TRANSITION-PRE-INIT: Pawn='%s' OldLane=%d NextLane=%d "
				 "JnctState.JunctionId=%d Phase=%d bWaiting=%s JunctionTransPts=%d "
				 "— about to call InitializeLaneFollowing (Phase-aware)"),
			GetPawn() ? *GetPawn()->GetName() : TEXT("NULL"),
			CurrentLane.HandleId, NextLane.HandleId,
			JnctState.JunctionId,
			(int32)JnctState.Phase,
			JnctState.bWaiting ? TEXT("YES") : TEXT("NO"),
			JnctState.TransitionPoints.Num());
	}

	InitializeLaneFollowing(NextLane);

	if (GTrafficJunctionDiagnostics >= 1)
	{
		UE_LOG(LogAAATraffic, Log,
			TEXT("JNCT TRANSITION-POST-INIT: Pawn='%s' NewLane=%d "
				 "JnctState.JunctionId=%d Phase=%d bWaiting=%s"),
			GetPawn() ? *GetPawn()->GetName() : TEXT("NULL"),
			NextLane.HandleId,
			JnctState.JunctionId,
			(int32)JnctState.Phase,
			JnctState.bWaiting ? TEXT("YES") : TEXT("NO"));
	}

	// If lane following could not be initialized for the selected connected lane,
	// treat this as a dead end so braking logic engages.
	if (!bLaneDataReady)
	{
		bAtDeadEnd = true;
		SetTurnSignal(ETurnSignalState::Hazard);
		AddLaneDecisionTrace(TEXT("TransitionCheck.InitializeFailed"), NextLane.HandleId, 0.0f, 0.0f, TEXT("InitializeLaneFollowing failed"));
		FlushLaneDecisionTrace(TEXT("TransitionInitializeFailed"), true);
		UE_LOG(LogAAATraffic, Warning,
			TEXT("TrafficVehicleController: Failed to initialize lane following for lane %d from lane %d — treating as dead end."),
			NextLane.HandleId, CurrentLane.HandleId);
		return;
	}

	AddLaneDecisionTrace(TEXT("TransitionCheck.InitializeSucceeded"), NextLane.HandleId, static_cast<float>(LanePoints.Num()), 0.0f, TEXT("Lane initialized"));
	if (GTrafficVehicleTraceFlushOnSuccess)
	{
		FlushLaneDecisionTrace(TEXT("TransitionSuccess"), false);
	}

	// Generate junction transition points.
	JnctState.TransitionIndex = 0;
	JnctState.CurveStartIndex = 0;
	if (bHasProviderPath)
	{
		JnctState.TransitionPoints = MoveTemp(ProviderJunctionPath);
	}
	else if (!OldLaneTangent.IsNearlyZero() && LanePoints.Num() >= 2)
	{
		// Synthesize a cubic Hermite curve from old-lane-end to new-lane-start.
		const FVector NewLaneStart = LanePoints[0];
		// Average the first few segments for robust entry tangent.
		const int32 NumAvgNew = FMath::Min(3, LanePoints.Num() - 1);
		FVector AvgNewDir = FVector::ZeroVector;
		for (int32 k = 0; k < NumAvgNew; ++k)
		{
			AvgNewDir += (LanePoints[k + 1] - LanePoints[k]);
		}
		const FVector NewLaneTangent = AvgNewDir.GetSafeNormal();
		const float SpanDist = FVector::Dist(OldLaneEnd, NewLaneStart);

		if (SpanDist > 50.0f) // Only if there's meaningful gap to smooth over.
		{
			// Angle-aware tangent scaling: sharp turns need shorter tangents
			// to prevent overshooting into the oncoming lane.
			const float DotFactor = FVector::DotProduct(OldLaneTangent, NewLaneTangent);
			const float Alpha = FMath::Lerp(0.25f, 0.5f,
				FMath::Clamp((DotFactor + 1.0f) * 0.5f, 0.0f, 1.0f));
			const float TangentScale = SpanDist * Alpha;
			const FVector P0 = OldLaneEnd;
			const FVector P1 = NewLaneStart;
			const FVector M0 = OldLaneTangent * TangentScale;
			const FVector M1 = NewLaneTangent * TangentScale;

			// Scale segment count with span distance using the tunable
			// JunctionCurveResolutionCm for smooth curves on any junction.
			// Clamped to [6, 32] for minimum quality and memory bound.
			const int32 NumSegments = FMath::Clamp(
				FMath::CeilToInt32(SpanDist / JunctionCurveResolutionCm), 6, 32);
			JnctState.TransitionPoints.Reserve(NumSegments + 1);
			for (int32 i = 0; i <= NumSegments; ++i)
			{
				const float T = static_cast<float>(i) / static_cast<float>(NumSegments);
				const float T2 = T * T;
				const float T3 = T2 * T;
				// Hermite basis functions.
				const float H00 = 2.0f * T3 - 3.0f * T2 + 1.0f;
				const float H10 = T3 - 2.0f * T2 + T;
				const float H01 = -2.0f * T3 + 3.0f * T2;
				const float H11 = T3 - T2;
				JnctState.TransitionPoints.Add(H00 * P0 + H10 * M0 + H01 * P1 + H11 * M1);
			}
		}
	}

	// ── Junction curve: prepend old-lane tail ──────────────────────────
	// The junction curve (from either provider or fallback Hermite) starts
	// at the old lane's END, but the vehicle is physically hundreds of cm
	// behind that point when the transition gate fires.  Prepending the
	// saved old-lane tail gives the junction curve a smooth lead-in from
	// the vehicle's actual position → old lane end → through the turn.
	// Without this, the vehicle drives straight into the intersection
	// before the curve steering kicks in, making the turn physically
	// impossible.
	if (JnctState.TransitionPoints.Num() > 0 && OldLaneTail.Num() > 1)
	{
		// Remove the last old-tail point if it's near the curve start
		// (they're both at the old lane endpoint).
		if (FVector::Dist(OldLaneTail.Last(), JnctState.TransitionPoints[0]) < 150.0f)
		{
			OldLaneTail.Pop();
		}
		if (OldLaneTail.Num() > 0)
		{
			TArray<FVector> Combined;
			Combined.Reserve(OldLaneTail.Num() + JnctState.TransitionPoints.Num());
			Combined.Append(OldLaneTail);
			Combined.Append(JnctState.TransitionPoints);
			JnctState.CurveStartIndex = OldLaneTail.Num();
			JnctState.TransitionPoints = MoveTemp(Combined);

			UE_LOG(LogAAATraffic, Log,
				TEXT("JNCT CURVE-PREPEND: Pawn='%s' prepended %d old-tail points "
					 "to junction curve (%d total, CurveStartIdx=%d)."),
				GetPawn() ? *GetPawn()->GetName() : TEXT("NULL"),
				OldLaneTail.Num(),
				JnctState.TransitionPoints.Num(),
				JnctState.CurveStartIndex);
		}
	}

	// Transition to Traversing when junction curves are committed.
	if (JnctState.TransitionPoints.Num() > 0
		&& (JnctState.Phase == EJunctionPhase::Approaching || JnctState.Phase == EJunctionPhase::Waiting))
	{
		JnctState.BeginTraversing();
	}

	// ── Non-junction boundary: prepend old-lane tail ────────────────
	// When no junction bridge curve was generated (SpanDist < 50 cm at a
	// non-junction boundary), the new LanePoints start at the lane boundary
	// but the vehicle is still hundreds of cm behind on the old lane's arc.
	// Prepending the saved old-lane tail gives the vehicle continuous
	// polyline coverage from its current physical position through the
	// boundary into the new lane — eliminating the CTE spike that causes
	// the "knocked off course" steering kick at every lane boundary.
	if (JnctState.TransitionPoints.Num() == 0 && OldLaneTail.Num() > 1 && LanePoints.Num() > 0)
	{
		// Deduplicate boundary point: old tail end ≈ new lane start.
		if (FVector::Dist(OldLaneTail.Last(), LanePoints[0]) < 150.0f)
		{
			OldLaneTail.Pop();
		}
		if (OldLaneTail.Num() > 0)
		{
			TArray<FVector> Combined;
			Combined.Reserve(OldLaneTail.Num() + LanePoints.Num());
			Combined.Append(OldLaneTail);
			Combined.Append(LanePoints);
			LanePoints = MoveTemp(Combined);
			LastClosestIndex = 0;
			// Restore heading derivative state that InitializeLaneFollowing
			// reset to zero — the vehicle is still on the same physical road,
			// so the heading state should be continuous.  Zeroing it causes a
			// derivative kick (DTerm spike) on the first tick.
			PreviousHeadingCrossZ = SavedHeadingCrossZ;

			UE_LOG(LogAAATraffic, Log,
				TEXT("LANE-PREPEND: Pawn='%s' prepended %d old-tail points to "
					 "new lane %d (%d total points) — continuous polyline "
					 "from vehicle position through lane boundary."),
				GetPawn() ? *GetPawn()->GetName() : TEXT("NULL"),
				OldLaneTail.Num(),
				CurrentLane.HandleId,
				LanePoints.Num());
		}
	}
}

// ---------------------------------------------------------------------------
// Lane Change — evaluation, blend, finalize
// ---------------------------------------------------------------------------

void ATrafficVehicleController::EvaluateLaneChange()
{
	if (!CachedProvider || !bLaneDataReady) { return; }
	AddLaneDecisionTrace(TEXT("LaneChange.EvaluateStart"), 0, static_cast<float>(CurrentLane.HandleId), 0.0f, TEXT("Lane-change evaluation"));

	const APawn* ControlledPawn = GetPawn();
	if (!ControlledPawn) { return; }

	const UChaosWheeledVehicleMovementComponent* VehicleMovement =
		Cast<UChaosWheeledVehicleMovementComponent>(ControlledPawn->GetMovementComponent());
	if (!VehicleMovement) { return; }

	const float CurrentSpeed = FMath::Abs(VehicleMovement->GetForwardSpeed());
	const float EffectiveTarget = FMath::Max(TargetSpeed, 1.0f);

	// Suppress non-navigational lane changes when approaching a junction.
	// The approach braking envelope is active — passing maneuvers would be
	// immediately aborted by the junction detection gate, causing turn-signal
	// jitter and wasted cooldown cycles.
	if (JnctState.Phase == EJunctionPhase::Approaching && !bNavigationalLaneChange)
	{
		if (IsVehicleTraceEnabled(2))
		{
			AddLaneDecisionTrace(TEXT("LaneChange.RejectApproaching"), 0,
				JnctState.ApproachDistanceCm, 0.0f, TEXT("Approaching intersection — suppressed"));
		}
		return;
	}

	// --- Navigational pre-positioning ---
	// If the vehicle needs to change lanes before a junction turn, bypass the
	// speed-ratio and leader-distance motivation checks.  Only the safety
	// checks (direction, gap, blind-spot) still apply.
	bool bNavigationalAttempt = false;
	ETrafficLaneSide NavigationalSide = ETrafficLaneSide::Left;
	if (bNavigationalLaneChange && NavigationalTargetLane.IsValid() && NavigationalTargetLane != CurrentLane)
	{
		bNavigationalAttempt = true;
		// Walk left from current lane to see if the target is on the left side.
		bool bFoundLeft = false;
		FTrafficLaneHandle Walk = CachedProvider->GetAdjacentLane(CurrentLane, ETrafficLaneSide::Left);
		for (int32 Safety = 0; Safety < 8 && Walk.IsValid(); ++Safety)
		{
			if (Walk == NavigationalTargetLane) { bFoundLeft = true; break; }
			Walk = CachedProvider->GetAdjacentLane(Walk, ETrafficLaneSide::Left);
		}
		NavigationalSide = bFoundLeft ? ETrafficLaneSide::Left : ETrafficLaneSide::Right;
		AddLaneDecisionTrace(TEXT("LaneChange.NavPrePos"), NavigationalTargetLane.HandleId,
			bFoundLeft ? -1.0f : 1.0f, 0.0f, TEXT("Navigational pre-positioning attempt"));
	}
	else if (bNavigationalLaneChange)
	{
		// Already in target lane or target became invalid — clear.
		bNavigationalLaneChange = false;
		NavigationalTargetLane = FTrafficLaneHandle();
	}

	float LeaderDist = -1.0f;
	if (!bNavigationalAttempt)
	{
		// Only consider lane change if we're significantly slower than desired.
		if (CurrentSpeed / EffectiveTarget >= LaneChangeSpeedThreshold)
		{
			if (IsVehicleTraceEnabled(2))
			{
				AddLaneDecisionTrace(TEXT("LaneChange.RejectSpeedRatio"), 0, CurrentSpeed / EffectiveTarget, LaneChangeSpeedThreshold, TEXT("Not slow enough"));
			}
			return;
		}

		// Verify a slow leader is actually ahead (avoid false triggers from turns/hills).
		float LeaderSpeed = 0.0f;
		LeaderDist = GetLeaderDistance(LeaderSpeed);
		if (LeaderDist < 0.0f || LeaderDist > DetectionDistance * 0.75f)
		{
			if (IsVehicleTraceEnabled(2))
			{
				AddLaneDecisionTrace(TEXT("LaneChange.RejectLeaderDistance"), 0, LeaderDist, DetectionDistance * 0.75f, TEXT("No close slow leader"));
			}
			return; // No close leader — speed issue is not from congestion.
		}
	}

	// Suppress lane changes on tight curves — vehicles should hold their
	// lane through curves to avoid cutting across lanes.  κ > 0.002 ≈ R < 5m.
	if (LanePoints.Num() >= 3)
	{
		const float CurCurvature = FMath::Abs(ComputeLocalCurvature(LanePoints, LastClosestIndex));
		if (CurCurvature > 0.002f)
		{
			if (IsVehicleTraceEnabled(2))
			{
				AddLaneDecisionTrace(TEXT("LaneChange.RejectCurvature"), 0, CurCurvature, 0.002f, TEXT("On tight curve"));
			}
			return;
		}
	}

	const FVector MyDirection = CachedProvider->GetLaneDirection(CurrentLane);

	// Choose sides to evaluate:
	// - Navigational: only the computed side toward the target lane.
	// - Normal: both sides in deterministic order (Left first, then Right).
	ETrafficLaneSide SidesBuffer[2] = { ETrafficLaneSide::Left, ETrafficLaneSide::Right };
	int32 NumSides = 2;
	if (bNavigationalAttempt)
	{
		SidesBuffer[0] = NavigationalSide;
		NumSides = 1;
	}

	for (int32 SideIdx = 0; SideIdx < NumSides; ++SideIdx)
	{
		const ETrafficLaneSide Side = SidesBuffer[SideIdx];
		FTrafficLaneHandle CandidateLane = CachedProvider->GetAdjacentLane(CurrentLane, Side);
		if (!CandidateLane.IsValid())
		{
			if (IsVehicleTraceEnabled(2))
			{
				AddLaneDecisionTrace(TEXT("LaneChange.RejectNoAdjacent"), 0, Side == ETrafficLaneSide::Left ? -1.0f : 1.0f, 0.0f, TEXT("No adjacent lane"));
			}
			continue;
		}

		// Direction safety: only allow same-direction lanes (dot > 0).
		const FVector CandidateDir = CachedProvider->GetLaneDirection(CandidateLane);
		const float DirDot = FVector::DotProduct(MyDirection, CandidateDir);
		if (DirDot <= 0.0f)
		{
			if (IsVehicleTraceEnabled(2))
			{
				AddLaneDecisionTrace(TEXT("LaneChange.RejectDirection"), CandidateLane.HandleId, DirDot, 0.0f, TEXT("Opposite direction lane"));
			}
			continue;
		}

		// Gap check via per-lane registry (deterministic — no physics sweep).
		// Account for vehicle dimensions: minimum safe gap = configured gap
		// or vehicle total length (front+rear), whichever is larger.
		UWorld* World = GetWorld();
		UTrafficSubsystem* TrafficSub = World ? World->GetSubsystem<UTrafficSubsystem>() : nullptr;
		if (!TrafficSub) { continue; }

		const float EffectiveGapRequired = FMath::Max(LaneChangeGapRequired,
			VehicleFrontExtent + VehicleRearExtent + 100.0f);
		const FVector VehicleLocation = ControlledPawn->GetActorLocation();
		bool bGapClear = true;

		// Check vehicles currently registered on the candidate lane.
		TArray<TWeakObjectPtr<ATrafficVehicleController>> Neighbors = TrafficSub->GetVehiclesOnLane(CandidateLane);
		for (const TWeakObjectPtr<ATrafficVehicleController>& WeakNeighbor : Neighbors)
		{
			ATrafficVehicleController* Neighbor = WeakNeighbor.Get();
			if (!Neighbor || Neighbor == this) { continue; }

			const APawn* NeighborPawn = Neighbor->GetPawn();
			if (!NeighborPawn) { continue; }

			const float DistToNeighbor = FVector::Dist(VehicleLocation, NeighborPawn->GetActorLocation());
			if (DistToNeighbor < EffectiveGapRequired)
			{
				bGapClear = false;
				if (IsVehicleTraceEnabled(2))
				{
					AddLaneDecisionTrace(TEXT("LaneChange.RejectGap"), CandidateLane.HandleId, DistToNeighbor, LaneChangeGapRequired, TEXT("Neighbor too close"));
				}
				break;
			}
		}

		// FIX (Phase 3): Blind-spot check — also reject if any vehicle is
		// currently mid-lane-change INTO the candidate lane. The per-lane
		// registry only tracks a vehicle's source lane during a change, so
		// without this check two vehicles can converge onto the same lane.
		// Also check Completing state — the 0.5s settle phase where the
		// vehicle is physically on the target lane but not yet registered.
		if (bGapClear)
		{
			for (const TWeakObjectPtr<ATrafficVehicleController>& WeakOther : TrafficSub->GetActiveVehicles())
			{
				ATrafficVehicleController* Other = WeakOther.Get();
				if (!Other || Other == this) { continue; }
				if (Other->LaneChangeState != ELaneChangeState::Executing
					&& Other->LaneChangeState != ELaneChangeState::Completing) { continue; }
				if (Other->TargetLaneHandle != CandidateLane) { continue; }

				const APawn* OtherPawn = Other->GetPawn();
				if (!OtherPawn) { continue; }

				const float DistToOther = FVector::Dist(VehicleLocation, OtherPawn->GetActorLocation());
				if (DistToOther < LaneChangeGapRequired)
				{
					bGapClear = false;
					if (IsVehicleTraceEnabled(2))
					{
						AddLaneDecisionTrace(TEXT("LaneChange.RejectBlindSpot"), CandidateLane.HandleId, DistToOther, LaneChangeGapRequired, TEXT("Vehicle changing into target lane"));
					}
					break;
				}
			}
		}

		// Lateral closing-speed check: reject merge if a vehicle on the
		// candidate lane behind us is closing fast. This prevents cut-in
		// merges in front of fast-approaching traffic.
		if (bGapClear && VehicleMovement)
		{
			const float EgoSpeed = FMath::Abs(VehicleMovement->GetForwardSpeed());
			const FVector EgoFwd = ControlledPawn->GetActorForwardVector();

			for (const TWeakObjectPtr<ATrafficVehicleController>& WeakNeighbor : Neighbors)
			{
				ATrafficVehicleController* Neighbor = WeakNeighbor.Get();
				if (!Neighbor || Neighbor == this) { continue; }

				const APawn* NeighborPawn = Neighbor->GetPawn();
				if (!NeighborPawn) { continue; }

				const FVector ToNeighbor = NeighborPawn->GetActorLocation() - VehicleLocation;
				const float DotAlong = FVector::DotProduct(ToNeighbor, EgoFwd);

				// Only check vehicles behind us (within 2× gap).
				if (DotAlong >= 0.0f || FMath::Abs(DotAlong) > LaneChangeGapRequired * 2.0f)
				{
					continue;
				}

				// Compute the neighbor's closing speed toward us.
				const UChaosWheeledVehicleMovementComponent* NeighborMC =
					Cast<UChaosWheeledVehicleMovementComponent>(NeighborPawn->GetMovementComponent());
				if (!NeighborMC) { continue; }

				const float NeighborSpeed = FMath::Abs(NeighborMC->GetForwardSpeed());
				const float ClosingSpeed = NeighborSpeed - EgoSpeed;

				// If neighbor is closing at > 500 cm/s (5 m/s = 18 km/h),
				// the merge is unsafe — they'd rear-end us.
				if (ClosingSpeed > 500.0f)
				{
					bGapClear = false;
					if (IsVehicleTraceEnabled(2))
					{
						AddLaneDecisionTrace(TEXT("LaneChange.RejectClosingSpeed"),
							CandidateLane.HandleId, ClosingSpeed, 500.0f,
							TEXT("Fast approaching vehicle behind"));
					}
					break;
				}
			}
		}

		if (!bGapClear) { continue; }

		// Candidate lane is valid — begin lane change.
		TArray<FVector> CandidatePoints;
		float CandidateWidth;
		if (!CachedProvider->GetLanePath(CandidateLane, CandidatePoints, CandidateWidth) || CandidatePoints.Num() < 2)
		{
			if (IsVehicleTraceEnabled(2))
			{
				AddLaneDecisionTrace(TEXT("LaneChange.RejectPath"), CandidateLane.HandleId, static_cast<float>(CandidatePoints.Num()), 2.0f, TEXT("Target lane path unavailable"));
			}
			continue;
		}

		LaneChangeState = ELaneChangeState::Executing;
		TargetLaneHandle = CandidateLane;
		TargetLanePoints = MoveTemp(CandidatePoints);
		TargetLaneWidth = CandidateWidth;
		LaneChangeProgress = 0.0f;

		// Activate turn signal for lane change direction.
		SetTurnSignal(Side == ETrafficLaneSide::Left ? ETurnSignalState::Left : ETurnSignalState::Right);

		UE_LOG(LogAAATraffic, Log,
			TEXT("TrafficVehicleController: Beginning lane change from lane %d to lane %d (%s)."),
			CurrentLane.HandleId, CandidateLane.HandleId,
			Side == ETrafficLaneSide::Left ? TEXT("left") : TEXT("right"));
		AddLaneDecisionTrace(TEXT("LaneChange.Begin"), CandidateLane.HandleId, LeaderDist, CurrentSpeed, Side == ETrafficLaneSide::Left ? TEXT("Left") : TEXT("Right"));
		return;
	}

	if (IsVehicleTraceEnabled(2))
	{
		AddLaneDecisionTrace(TEXT("LaneChange.NoCandidateAccepted"), 0, LeaderDist, CurrentSpeed, TEXT("All adjacent lanes rejected"));
	}
}

FVector ATrafficVehicleController::UpdateLaneChangeBlend(const FVector& VehicleLocation, int32 ClosestIndex)
{
	// --- Continuous merge safety check (Feature 5) ---
	// Re-check gap on target lane each tick. Abort early if unsafe.
	if (LaneChangeState == ELaneChangeState::Executing && LaneChangeProgress < 0.8f)
	{
		const float AbortGapThreshold = LaneChangeGapRequired * 0.7f;
		UWorld* World = GetWorld();
		UTrafficSubsystem* TrafficSub = World ? World->GetSubsystem<UTrafficSubsystem>() : nullptr;
		if (TrafficSub)
		{
			TArray<TWeakObjectPtr<ATrafficVehicleController>> Neighbors = TrafficSub->GetVehiclesOnLane(TargetLaneHandle);
			for (const TWeakObjectPtr<ATrafficVehicleController>& WeakNeighbor : Neighbors)
			{
				ATrafficVehicleController* Neighbor = WeakNeighbor.Get();
				if (!Neighbor || Neighbor == this) continue;
				const APawn* NeighborPawn = Neighbor->GetPawn();
				if (!NeighborPawn) continue;
				if (FVector::Dist(VehicleLocation, NeighborPawn->GetActorLocation()) < AbortGapThreshold)
				{
					AddLaneDecisionTrace(TEXT("LaneChange.AbortUnsafeGap"), TargetLaneHandle.HandleId, FVector::Dist(VehicleLocation, NeighborPawn->GetActorLocation()), AbortGapThreshold, TEXT("Continuous merge check failed"));
					AbortLaneChange();
					// Return source lane point after aborting.
					return GetLookAheadPoint(VehicleLocation, ClosestIndex);
				}
			}
		}
	}

	// Advance progress based on distance traveled this tick.
	// DistanceThisTick is computed once at the top of UpdateVehicleInput
	// (before PreviousVehicleLocation is overwritten).
	{
		const float ProgressDelta = DistanceThisTick / FMath::Max(LaneChangeDistance, 1.0f);
		LaneChangeProgress = FMath::Min(LaneChangeProgress + ProgressDelta, 1.0f);
	}

	// Get look-ahead on source lane.
	const FVector SourcePoint = GetLookAheadPoint(VehicleLocation, ClosestIndex);

	// Find closest index on target lane and get look-ahead there.
	int32 TargetClosestIndex = 0;
	float BestDistSq = MAX_flt;
	for (int32 i = 0; i < TargetLanePoints.Num(); ++i)
	{
		const float DSq = FVector::DistSquared(VehicleLocation, TargetLanePoints[i]);
		if (DSq < BestDistSq)
		{
			BestDistSq = DSq;
			TargetClosestIndex = i;
		}
	}

	// Walk forward on target lane by LookAheadDistance.
	FVector TargetPoint = TargetLanePoints.Last();
	{
		float AccDist = 0.0f;
		int32 Idx = TargetClosestIndex;
		while (Idx < TargetLanePoints.Num() - 1)
		{
			// Use 2D distance to match main GetLookAheadPoint behavior.
			// 3D distance on hills makes the look-ahead point closer than intended.
			const float SegDist = FVector::Dist2D(TargetLanePoints[Idx], TargetLanePoints[Idx + 1]);
			AccDist += SegDist;
			if (AccDist >= LookAheadDistance)
			{
				const float Overshoot = AccDist - LookAheadDistance;
				const float Alpha = 1.0f - (Overshoot / FMath::Max(SegDist, KINDA_SMALL_NUMBER));
				TargetPoint = FMath::Lerp(TargetLanePoints[Idx], TargetLanePoints[Idx + 1], Alpha);
				break;
			}
			++Idx;
		}
	}

	// Smooth blend with ease-in-out curve.
	const float BlendAlpha = FMath::SmoothStep(0.0f, 1.0f, LaneChangeProgress);
	const FVector BlendedPoint = FMath::Lerp(SourcePoint, TargetPoint, BlendAlpha);

	// Check for completion — enter settling phase.
	if (LaneChangeProgress >= 1.0f && LaneChangeState == ELaneChangeState::Executing)
	{
		LaneChangeState = ELaneChangeState::Completing;
		LaneChangeSettleTimer = 0.5f; // 0.5s settling phase.
		UE_LOG(LogAAATraffic, Verbose,
			TEXT("TrafficVehicleController: Lane change blend complete, entering settling phase on lane %d."),
			TargetLaneHandle.HandleId);
	}

	return BlendedPoint;
}

void ATrafficVehicleController::FinalizeLaneChange()
{
	AddLaneDecisionTrace(TEXT("LaneChange.Finalize"), TargetLaneHandle.HandleId, LaneChangeProgress, LaneChangeCooldownTime, TEXT("Lane change finalized"));
	UE_LOG(LogAAATraffic, Log,
		TEXT("TrafficVehicleController: Lane change complete — now on lane %d."),
		TargetLaneHandle.HandleId);

	// Adopt target lane as current.
	CurrentLane = TargetLaneHandle;
	LanePoints = MoveTemp(TargetLanePoints);
	LaneWidth = TargetLaneWidth;

	// Reset distance tracking for new lane.
	DistanceTraveledOnLane = 0.0f;
	LastClosestIndex = 0;

	// Query speed limit for the new lane.
	if (CachedProvider)
	{
		const float LaneSpeedLimit = CachedProvider->GetLaneSpeedLimit(CurrentLane);
		if (LaneSpeedLimit > 0.0f)
		{
			TargetSpeed = LaneSpeedLimit;
		}
		else if (DefaultSpeedLimit > 0.0f)
		{
			TargetSpeed = DefaultSpeedLimit;
		}
		else
		{
			TargetSpeed = BaseTargetSpeed;
		}
	}

	// Reset lane-change state.
	LaneChangeState = ELaneChangeState::None;
	TargetLaneHandle = FTrafficLaneHandle();
	TargetLanePoints.Empty();
	LaneChangeProgress = 0.0f;
	LaneChangeCooldownRemaining = LaneChangeCooldownTime;
	PreviousHeadingCrossZ = 0.0f;

	// Turn signal off (deferred) — keep signal on briefly after lane change completes.
	TurnSignalOffDelayRemaining = 1.0f;

	// Clear navigational lane change flag.
	bNavigationalLaneChange = false;
	NavigationalTargetLane = FTrafficLaneHandle();

	// Notify subsystem of new lane.
	if (UWorld* World = GetWorld())
	{
		if (UTrafficSubsystem* TrafficSub = World->GetSubsystem<UTrafficSubsystem>())
		{
			TrafficSub->UpdateVehicleLane(this, CurrentLane);
		}
	}

	if (GTrafficVehicleTraceFlushOnSuccess)
	{
		FlushLaneDecisionTrace(TEXT("LaneChangeSuccess"), false);
	}
}

void ATrafficVehicleController::AbortLaneChange()
{
	AddLaneDecisionTrace(TEXT("LaneChange.Abort"), CurrentLane.HandleId, LaneChangeProgress, LaneChangeCooldownTime, TEXT("AbortLaneChange called"));
	UE_LOG(LogAAATraffic, Log,
		TEXT("TrafficVehicleController: Lane change ABORTED (unsafe gap) — staying on lane %d."),
		CurrentLane.HandleId);

	LaneChangeState = ELaneChangeState::None;
	TargetLaneHandle = FTrafficLaneHandle();
	TargetLanePoints.Empty();
	LaneChangeProgress = 0.0f;
	LaneChangeSettleTimer = 0.0f;
	LaneChangeCooldownRemaining = LaneChangeCooldownTime;

	// Turn signal off — lane change aborted.
	SetTurnSignal(ETurnSignalState::Off);

	// Clear navigational lane change flag.
	bNavigationalLaneChange = false;
	NavigationalTargetLane = FTrafficLaneHandle();
	FlushLaneDecisionTrace(TEXT("LaneChangeAbort"), true);
}

float ATrafficVehicleController::GetLeaderDistance(float& OutLeaderSpeed) const
{
	OutLeaderSpeed = 0.0f;

	const APawn* ControlledPawn = GetPawn();
	if (!ControlledPawn) return -1.0f;

	const UWorld* World = GetWorld();
	if (!World) return -1.0f;

	const FVector VehicleLocation = ControlledPawn->GetActorLocation();

	// I4 FIX: Polyline-following leader sweep.
	// Instead of one straight-line sweep (which misses leaders around
	// curves), walk the lane polyline and fire short sweeps along each
	// segment until DetectionDistance is consumed or a hit is found.
	//
	// During lane changes, fall back to a single straight sweep along
	// the vehicle's forward vector (the polyline is for the source lane
	// and would miss leaders on the target lane).

	const float SweepRadiusFraction = 0.4f;
	const float MinSweepRadius = 50.0f;
	const float SweepRadius = FMath::Max(LaneWidth * SweepRadiusFraction, MinSweepRadius);
	const float SelfCollisionBuffer = 10.0f;

	FCollisionQueryParams QueryParams;
	QueryParams.AddIgnoredActor(ControlledPawn);
	QueryParams.bTraceComplex = false;

	// Choose which polyline to walk.
	const TArray<FVector>* Polyline = nullptr;
	int32 StartIdx = 0;
	if (LaneChangeState != ELaneChangeState::None)
	{
		// During lane change: single straight sweep along vehicle forward.
		Polyline = nullptr; // signal for straight-line fallback
	}
	else if (JnctState.JunctionId != 0 && JnctState.TransitionPoints.Num() >= 2)
	{
		// Traversing a junction: walk the junction polyline.
		Polyline = &JnctState.TransitionPoints;
		StartIdx = FMath::Clamp(JnctState.TransitionIndex, 0, JnctState.TransitionPoints.Num() - 1);
	}
	else if (LanePoints.Num() >= 2)
	{
		Polyline = &LanePoints;
		StartIdx = FMath::Clamp(LastClosestIndex, 0, LanePoints.Num() - 1);
	}

	if (!Polyline)
	{
		// Fallback: single straight sweep along vehicle forward.
		const FVector SweepDir = ControlledPawn->GetActorForwardVector();
		const FVector SweepStart = VehicleLocation + SweepDir * (SweepRadius + SelfCollisionBuffer);
		const FVector SweepEnd = SweepStart + SweepDir * DetectionDistance;

		FHitResult HitResult;
		const bool bHit = World->SweepSingleByChannel(
			HitResult, SweepStart, SweepEnd, FQuat::Identity,
			ECC_Pawn, FCollisionShape::MakeSphere(SweepRadius), QueryParams);

		if (!bHit || !HitResult.GetActor()) return -1.0f;
		const APawn* HitPawn = Cast<APawn>(HitResult.GetActor());
		if (!HitPawn) return -1.0f;
		if (FVector::DotProduct(HitPawn->GetActorForwardVector(), SweepDir) < -0.3f) return -1.0f;

		if (const UPawnMovementComponent* LM = HitPawn->GetMovementComponent())
		{
			OutLeaderSpeed = FVector::DotProduct(LM->Velocity, SweepDir);
		}
		return HitResult.Distance;
	}

	// ── Polyline-following sweep ─────────────────────────────
	// Walk segments from StartIdx forward, firing a sphere sweep
	// along each segment. Accumulate distance and return total
	// on first hit.
	const TArray<FVector>& Pts = *Polyline;
	float AccumDist = 0.0f;
	FVector SegStart = VehicleLocation;
	bool bFirstSeg = true;

	for (int32 i = StartIdx; i < Pts.Num() - 1; ++i)
	{
		const FVector& SegEnd = Pts[i + 1];
		FVector SegDir = (SegEnd - SegStart).GetSafeNormal();
		if (SegDir.IsNearlyZero())
		{
			SegStart = SegEnd;
			continue;
		}

		const float SegLen = FVector::Dist(SegStart, SegEnd);
		const float BudgetRemaining = DetectionDistance - AccumDist;
		if (BudgetRemaining <= 0.0f) break;

		// On the first segment, offset start past our own collision.
		const FVector SwStart = bFirstSeg
			? (SegStart + SegDir * (SweepRadius + SelfCollisionBuffer))
			: SegStart;
		const float SweepLen = FMath::Min(SegLen, BudgetRemaining);
		const FVector SwEnd = SwStart + SegDir * SweepLen;

		FHitResult HitResult;
		const bool bHit = World->SweepSingleByChannel(
			HitResult, SwStart, SwEnd, FQuat::Identity,
			ECC_Pawn, FCollisionShape::MakeSphere(SweepRadius), QueryParams);

		if (bHit && HitResult.GetActor())
		{
			const APawn* HitPawn = Cast<APawn>(HitResult.GetActor());
			if (HitPawn)
			{
				// Oncoming filter.
				if (FVector::DotProduct(HitPawn->GetActorForwardVector(), SegDir) >= -0.3f)
				{
					if (const UPawnMovementComponent* LM = HitPawn->GetMovementComponent())
					{
						OutLeaderSpeed = FVector::DotProduct(LM->Velocity, SegDir);
					}
					return AccumDist + HitResult.Distance;
				}
			}
		}

		AccumDist += SweepLen;
		SegStart = SegEnd;
		bFirstSeg = false;
	}

	// ── Straight-line extension past polyline end ────────────
	// If we exhausted the polyline but still have detection budget,
	// fire one more sweep from the last polyline point in the direction
	// of the last segment.  Without this, vehicles just past the lane
	// end (e.g., entering a junction) are invisible to following vehicles
	// — the leading cause of rear-end collisions at junction entries.
	{
		const float ExtBudget = DetectionDistance - AccumDist;
		if (ExtBudget > 0.0f && Pts.Num() >= 2)
		{
			const FVector LastDir = (Pts.Last() - Pts[Pts.Num() - 2]).GetSafeNormal();
			if (!LastDir.IsNearlyZero())
			{
				const FVector ExtStart = SegStart; // == last polyline point after loop
				const FVector ExtEnd   = ExtStart + LastDir * ExtBudget;

				FHitResult HitResult;
				const bool bHit = World->SweepSingleByChannel(
					HitResult, ExtStart, ExtEnd, FQuat::Identity,
					ECC_Pawn, FCollisionShape::MakeSphere(SweepRadius), QueryParams);

				if (bHit && HitResult.GetActor())
				{
					const APawn* HitPawn = Cast<APawn>(HitResult.GetActor());
					if (HitPawn && FVector::DotProduct(HitPawn->GetActorForwardVector(), LastDir) >= -0.3f)
					{
						if (const UPawnMovementComponent* LM = HitPawn->GetMovementComponent())
						{
							OutLeaderSpeed = FVector::DotProduct(LM->Velocity, LastDir);
						}
						return AccumDist + HitResult.Distance;
					}
				}
			}
		}
	}

	return -1.0f; // No leader found within detection distance.
}
