// Copyright AAA_Traffic Contributors. All Rights Reserved.

#include "TrafficVehicleController.h"
#include "TrafficSubsystem.h"
#include "TrafficSignalController.h"
#include "TrafficLog.h"
#include "ChaosWheeledVehicleMovementComponent.h"
#include "GameFramework/Pawn.h"
#include "Engine/World.h"
#include "DrawDebugHelpers.h"
#include "PhysicsEngine/BodyInstance.h"
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

namespace
{
	static bool IsVehicleTraceEnabled(const int32 RequiredLevel)
	{
		return GTrafficVehicleDecisionTrace >= RequiredLevel;
	}
}

ATrafficVehicleController::ATrafficVehicleController()
	: LaneWidth(0.f)
	, bLaneDataReady(false)
	, CachedProvider(nullptr)
	, bAtDeadEnd(false)
	, JunctionTransitionIndex(0)
	, bWaitingAtIntersection(false)
	, IntersectionJunctionId(0)
	, DistanceTraveledOnLane(0.0f)
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
	, LookAheadDistance(500.f)
	, FollowingDistance(300.f)
	, DetectionDistance(2000.f)
	, LaneChangeDistance(1500.f)
	, LaneChangeCooldownTime(5.0f)
	, LaneChangeSpeedThreshold(0.6f)
	, LaneChangeGapRequired(800.f)
	, DefaultSpeedLimit(0.0f)
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

void ATrafficVehicleController::InitializeLaneFollowing(const FTrafficLaneHandle& InLane)
{
	CurrentLane = InLane;
	bLaneDataReady = false;
	bAtDeadEnd = false;
	DistanceTraveledOnLane = 0.0f;
	PreviousVehicleLocation = FVector::ZeroVector;
	LastClosestIndex = 0;
	JunctionTransitionPoints.Empty();
	JunctionTransitionIndex = 0;
	bWaitingAtIntersection = false;
	IntersectionJunctionId = 0;

	// Reset lane-change state when entering a new lane.
	LaneChangeState = ELaneChangeState::None;
	LaneChangeProgress = 0.0f;
	LaneChangeSettleTimer = 0.0f;
	TargetLanePoints.Empty();

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
			if (!CanSleepProp)
			{
				// Last resort: iterate all bool properties and match by DisplayName metadata.
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
			if (IntersectionJunctionId != 0)
			{
				TrafficSub->ReleaseJunction(IntersectionJunctionId, this);
				IntersectionJunctionId = 0;
			}
			TrafficSub->UnregisterVehicle(this);
		}
	}
	Super::OnUnPossess();
}

void ATrafficVehicleController::Tick(float DeltaSeconds)
{
	Super::Tick(DeltaSeconds);

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
					 "bAtDeadEnd=%s bWaitingAtIntersection=%s LaneChangeState=%d"),
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
				bWaitingAtIntersection ? TEXT("YES") : TEXT("no"),
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
					 "bAtDeadEnd=%s bWaitingAtIntersection=%s | "
					 "BodyAwake=%s "
					 "Parked=%s Handbrake=%s "
					 "RPM=%.0f Gear=%d/%d Throttle=%.2f Brake=%.2f Steering=%.2f"),
				*GetPawn()->GetName(), CurSpeed,
				CIdx, LanePoints.Num(),
				RemDist, DistanceTraveledOnLane, ThrshVal,
				bAtDeadEnd ? TEXT("YES") : TEXT("no"),
				bWaitingAtIntersection ? TEXT("YES") : TEXT("no"),
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

	++LODFrameCounter;
	if (LODFrameCounter >= 20) { LODFrameCounter = 0; } // Wrap at LCM(4,10) to prevent overflow.

	// --- LOD-based tick gating (Feature 7) ---
	// Reduced: tick every 4 frames. Minimal: tick every 10 frames.
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
		return;
	}
	if (CurrentLOD == ETrafficLOD::Minimal && (LODFrameCounter % 10) != 0)
	{
		// Minimal LOD: teleport along polyline instead of physics input.
		if (LanePoints.Num() >= 2)
		{
			const int32 NextIdx = FMath::Min(LastClosestIndex + 1, LanePoints.Num() - 1);
			if (NextIdx != LastClosestIndex)
			{
				GetPawn()->SetActorLocation(LanePoints[NextIdx], /*bSweep*/ false, /*OutSweepHitResult*/ nullptr, ETeleportType::TeleportPhysics);
				LastClosestIndex = NextIdx;
			}
		}
		return;
	}

	UpdateVehicleInput(DeltaSeconds);

#if ENABLE_DRAW_DEBUG
	if (bDebugDraw && GetPawn())
	{
		const UWorld* DbgWorld = GetWorld();
		const FVector VehicleLoc = GetPawn()->GetActorLocation();

		// Draw lane polyline (cyan).
		for (int32 i = 0; i < LanePoints.Num() - 1; ++i)
		{
			DrawDebugLine(DbgWorld, LanePoints[i], LanePoints[i + 1], FColor::Cyan, false, -1.0f, 0, 2.0f);
		}

		// Draw look-ahead target (green sphere).
		const int32 DbgClosest = FindClosestPointIndex(VehicleLoc);
		const FVector LookPt = GetLookAheadPoint(VehicleLoc, DbgClosest);
		DrawDebugSphere(DbgWorld, LookPt, 30.0f, 6, FColor::Green, false, -1.0f, 0, 2.0f);
		DrawDebugLine(DbgWorld, VehicleLoc, LookPt, FColor::Green, false, -1.0f, 0, 1.5f);

		// Draw detection range (orange line).
		const FVector DetEnd = VehicleLoc + GetPawn()->GetActorForwardVector() * DetectionDistance;
		DrawDebugLine(DbgWorld, VehicleLoc, DetEnd, FColor::Orange, false, -1.0f, 0, 1.0f);

		// Draw lane-change target lane (magenta) if active.
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

	// --- Lane-end detection + intersection right-of-way ---
	if (!bAtDeadEnd && LaneChangeState == ELaneChangeState::None)
	{
		const float RemainingDist = GetRemainingDistance(ClosestIndex);
		// Use speed-based look-ahead so fast vehicles get more advance notice.
		const float ReactionTime = 1.0f; // seconds
		const float TransitionThreshold = FMath::Max(LookAheadDistance, FMath::Abs(CurrentSpeed) * ReactionTime);
		if (RemainingDist < TransitionThreshold && DistanceTraveledOnLane > TransitionThreshold)
		{
			// --- Intersection approach (Feature 1) ---
			// Query junction ID at lane end. If non-zero, check occupancy first.
			if (!bWaitingAtIntersection && CachedProvider)
			{
				const int32 JunctionId = CachedProvider->GetJunctionForLane(CurrentLane);
				if (JunctionId != 0)
				{
					IntersectionJunctionId = JunctionId;

					UWorld* World = GetWorld();
					UTrafficSubsystem* TrafficSub = World ? World->GetSubsystem<UTrafficSubsystem>() : nullptr;
					if (TrafficSub)
					{
// Check for signal first — vehicles stop on Red/Yellow.
					ATrafficSignalController* Signal = TrafficSub->GetSignalForJunction(JunctionId);
					bool bSignalAllows = true;
					if (Signal)
					{
						bSignalAllows = Signal->IsLaneGreen(CurrentLane);
						}

						if (bSignalAllows && TrafficSub->TryOccupyJunction(JunctionId, this))
						{
							// We have right-of-way — proceed with transition.
						}
						else
						{
							// Junction occupied — wait.
							bWaitingAtIntersection = true;
						}
					}
				}
			}

			// If waiting at intersection, brake proportionally to remaining distance.
		if (bWaitingAtIntersection)
			{
				UWorld* World = GetWorld();
				UTrafficSubsystem* TrafficSub = World ? World->GetSubsystem<UTrafficSubsystem>() : nullptr;

				// Retry signal + occupancy each tick.
				if (TrafficSub && IntersectionJunctionId != 0)
				{
					// Re-check the traffic signal phase before attempting occupancy.
					bool bSignalAllows = true;
					ATrafficSignalController* Signal = TrafficSub->GetSignalForJunction(IntersectionJunctionId);
					if (Signal)
					{
						bSignalAllows = Signal->IsLaneGreen(CurrentLane);
					}

					if (bSignalAllows && TrafficSub->TryOccupyJunction(IntersectionJunctionId, this))
					{
						bWaitingAtIntersection = false;
						// Fall through to CheckLaneTransition below.
					}
				}

				if (bWaitingAtIntersection)
				{
					// Gradually brake to a stop at the lane end.
					const float BrakeFactor = FMath::Clamp(1.0f - (RemainingDist / FMath::Max(TransitionThreshold, 1.0f)), 0.0f, 1.0f);
					VehicleMovement->SetThrottleInput(FMath::Max(0.0f, 0.3f - BrakeFactor * 0.3f));
					VehicleMovement->SetSteeringInput(0.0f);
					VehicleMovement->SetBrakeInput(BrakeFactor);
					return;
				}
			}

			CheckLaneTransition();
			if (!bLaneDataReady || !GetPawn())
			{
				// InitializeLaneFollowing may have failed — bail.
				return;
			}
			if (!bAtDeadEnd)
			{
				// Successfully transitioned. Release junction immediately if no
				// transition points were generated (junction span too small).
				if (IntersectionJunctionId != 0 && JunctionTransitionPoints.Num() == 0)
				{
					UWorld* World = GetWorld();
					UTrafficSubsystem* TrafficSub = World ? World->GetSubsystem<UTrafficSubsystem>() : nullptr;
					if (TrafficSub)
					{
						TrafficSub->ReleaseJunction(IntersectionJunctionId, this);
					}
					IntersectionJunctionId = 0;
				}
				// Recalculate on new lane data.
				return;
			}
		}
	}

	// --- Dead-end braking ---
	if (bAtDeadEnd)
	{
		VehicleMovement->SetThrottleInput(0.0f);
		VehicleMovement->SetSteeringInput(0.0f);
		VehicleMovement->SetBrakeInput(1.0f);
		return;
	}

	// --- Determine target point (junction transition, lane-change blended, or normal) ---
	FVector TargetPoint;

	// Junction transition takes priority — follow synthesized curve first.
	if (JunctionTransitionPoints.Num() > 0)
	{
		// Advance along junction points based on proximity or forward progress.
		// Use both distance check and "closer to next" test so high-speed vehicles
		// that skip past points still advance correctly.
		while (JunctionTransitionIndex < JunctionTransitionPoints.Num() - 1)
		{
			const float DistToCurrentSq = FVector::DistSquared(VehicleLocation, JunctionTransitionPoints[JunctionTransitionIndex]);
			const float DistToNextSq = FVector::DistSquared(VehicleLocation, JunctionTransitionPoints[JunctionTransitionIndex + 1]);
			if (DistToCurrentSq < 10000.0f || DistToNextSq < DistToCurrentSq) // 1m or closer to next
			{
				++JunctionTransitionIndex;
			}
			else
			{
				break;
			}
		}

		if (JunctionTransitionIndex >= JunctionTransitionPoints.Num() - 1)
		{
			// Finished junction transition — release junction occupancy and proceed.
			JunctionTransitionPoints.Empty();
			JunctionTransitionIndex = 0;

			if (IntersectionJunctionId != 0)
			{
				if (UWorld* World = GetWorld())
				{
					if (UTrafficSubsystem* TrafficSub = World->GetSubsystem<UTrafficSubsystem>())
					{
						TrafficSub->ReleaseJunction(IntersectionJunctionId, this);
					}
				}
				IntersectionJunctionId = 0;
			}

			TargetPoint = GetLookAheadPoint(VehicleLocation, ClosestIndex);
		}
		else
		{
			// Look ahead a bit on the junction curve.
			const int32 LookIdx = FMath::Min(JunctionTransitionIndex + 2, JunctionTransitionPoints.Num() - 1);
			TargetPoint = JunctionTransitionPoints[LookIdx];
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

	// --- Steering ---
	const FVector ToTarget = (TargetPoint - VehicleLocation).GetSafeNormal2D();
	const FVector Forward2D = VehicleForward.GetSafeNormal2D();
	const float CrossZ = FVector::CrossProduct(Forward2D, ToTarget).Z;

	// CrossZ is positive when target is to the right, negative when left.
	// Scale for reasonable responsiveness.
	const float SteeringInput = FMath::Clamp(CrossZ * 2.0f, -1.0f, 1.0f);

	// --- Throttle / Brake ---
	// Compute effective target speed accounting for vehicle ahead.
	// On Reduced LOD, skip the expensive sphere-sweep leader detection.
	float EffectiveTargetSpeed = TargetSpeed;
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
			float LeaderSpeed = 0.0f;
			const float LeaderDist = GetLeaderDistance(LeaderSpeed);
			if (LeaderDist >= 0.0f)
			{
				// Proportional: full stop at FollowingDistance, full speed at 2x FollowingDistance.
				const float GapFactor = FMath::Clamp(
					(LeaderDist - FollowingDistance) / FMath::Max(FollowingDistance, 1.0f),
					0.0f, 1.0f);
				const float GapSpeed = TargetSpeed * GapFactor;
				// Blend between leader speed (close) and gap-based speed (far).
				// At GapFactor=0 (FollowingDistance): match leader speed.
				// At GapFactor=1 (2x FollowingDistance): use full gap-based speed.
				// This prevents hard-braking for a distant stopped leader.
				const float ClampedLeaderSpeed = FMath::Max(LeaderSpeed, 0.0f);
				EffectiveTargetSpeed = FMath::Lerp(ClampedLeaderSpeed, GapSpeed, GapFactor);
				EffectiveTargetSpeed = FMath::Min(EffectiveTargetSpeed, TargetSpeed);
			}
		}
		// Reduced LOD: analytical leader detection via spatial grid instead of
		// a physics sweep — much cheaper for distant vehicles.
		else if (TickLOD == ETrafficLOD::Reduced)
		{
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
					const FVector Delta = OP->GetActorLocation() - Loc;
					const float Dist = Delta.Size();
					// Must be ahead of us (positive dot with forward).
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
					const float GapFactor = FMath::Clamp(
						(BestDist - FollowingDistance) / FMath::Max(FollowingDistance, 1.0f),
						0.0f, 1.0f);
					const float GapSpeed = TargetSpeed * GapFactor;
					const float ClampedLeaderSpeed = FMath::Max(BestSpeed, 0.0f);
					EffectiveTargetSpeed = FMath::Lerp(ClampedLeaderSpeed, GapSpeed, GapFactor);
					EffectiveTargetSpeed = FMath::Min(EffectiveTargetSpeed, TargetSpeed);
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

	const float SpeedError = EffectiveTargetSpeed - FMath::Abs(CurrentSpeed);
	float ThrottleInput = FMath::Clamp(SpeedError / FMath::Max(EffectiveTargetSpeed, 1.0f), 0.0f, 1.0f);
	float BrakeInput = 0.0f;

	// Brake if significantly over effective target speed
	if (CurrentSpeed > EffectiveTargetSpeed * 1.1f)
	{
		ThrottleInput = 0.0f;
		BrakeInput = FMath::Clamp((CurrentSpeed - EffectiveTargetSpeed) / FMath::Max(EffectiveTargetSpeed, 1.0f), 0.0f, 1.0f);
	}

	// Back off throttle in sharp turns
	if (FMath::Abs(SteeringInput) > 0.5f)
	{
		ThrottleInput *= 0.5f;
	}

	if (ThrottleInput >= GTrafficWakeGuardMinThrottle
		&& BrakeInput <= KINDA_SMALL_NUMBER
		&& FMath::Abs(CurrentSpeed) <= GTrafficWakeGuardMaxSpeed)
	{
		VehicleMovement->SetParked(false);
		VehicleMovement->SetSleeping(false);
		VehicleMovement->SetHandbrakeInput(false);

		if (UPrimitiveComponent* Prim = VehicleMovement->UpdatedPrimitive)
		{
			Prim->WakeAllRigidBodies();
			if (FBodyInstance* BI = Prim->GetBodyInstance())
			{
				BI->WakeInstance();
				FPhysicsActorHandle PhysHandle = BI->GetPhysicsActorHandle();
				if (PhysHandle)
				{
					PhysHandle->GetGameThreadAPI().SetSleepType(Chaos::ESleepType::NeverSleep);
				}
			}
		}
	}

	VehicleMovement->SetThrottleInput(ThrottleInput);
	VehicleMovement->SetSteeringInput(SteeringInput);
	VehicleMovement->SetBrakeInput(BrakeInput);

	// --- Diagnostic: log first tick's driving values per vehicle ---
	if (!bDiagLoggedFirstInput)
	{
		bDiagLoggedFirstInput = true;
		UE_LOG(LogAAATraffic, Log,
			TEXT("VehicleController::UpdateVehicleInput FIRST TICK: Pawn='%s' "
				 "TargetSpeed=%.1f EffectiveTarget=%.1f CurrentSpeed=%.1f "
				 "Throttle=%.3f Steering=%.3f Brake=%.3f "
				 "bAtDeadEnd=%s bWaitingAtIntersection=%s LanePoints=%d ClosestIdx=%d"),
			*ControlledPawn->GetName(),
			TargetSpeed, EffectiveTargetSpeed, CurrentSpeed,
			ThrottleInput, SteeringInput, BrakeInput,
			bAtDeadEnd ? TEXT("true") : TEXT("false"),
			bWaitingAtIntersection ? TEXT("true") : TEXT("false"),
			LanePoints.Num(), ClosestIndex);
	}
}

int32 ATrafficVehicleController::FindClosestPointIndex(const FVector& VehicleLocation)
{
	if (LanePoints.Num() == 0) { return 0; }

	// Bounded search around the last-known index for O(1) amortized performance.
	// Vehicles advance monotonically along the lane, so the closest index is
	// usually within a few steps of the cached value.
	constexpr int32 SearchRadius = 8;
	LastClosestIndex = FMath::Clamp(LastClosestIndex, 0, LanePoints.Num() - 1);
	const int32 StartIdx = FMath::Max(0, LastClosestIndex - 2);
	const int32 EndIdx = FMath::Min(LanePoints.Num() - 1, LastClosestIndex + SearchRadius);

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

	// Fall back to a full scan if the bounded search result is far from the vehicle,
	// indicating a teleport, respawn, or lane switch.
	constexpr float FullScanThresholdSq = 500.0f * 500.0f; // 5m
	if (BestDistSq > FullScanThresholdSq)
	{
		for (int32 i = 0; i < LanePoints.Num(); ++i)
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
		const float SegmentDist = FVector::Dist(LanePoints[Index], LanePoints[Index + 1]);
		AccumulatedDist += SegmentDist;

		if (AccumulatedDist >= LookAheadDistance)
		{
			const float Overshoot = AccumulatedDist - LookAheadDistance;
			const float Alpha = 1.0f - (Overshoot / FMath::Max(SegmentDist, KINDA_SMALL_NUMBER));
			return FMath::Lerp(LanePoints[Index], LanePoints[Index + 1], Alpha);
		}

		++Index;
	}

	// If the look-ahead extends beyond the lane, return the last point.
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

void ATrafficVehicleController::CheckLaneTransition()
{
	AddLaneDecisionTrace(TEXT("TransitionCheck.Start"), 0, static_cast<float>(CurrentLane.HandleId), 0.0f, TEXT("Entering lane-end transition check"));

	if (!CachedProvider)
	{
		bAtDeadEnd = true;
		AddLaneDecisionTrace(TEXT("TransitionCheck.NoProvider"), 0, 0.0f, 0.0f, TEXT("No cached provider"));
		FlushLaneDecisionTrace(TEXT("NoProviderDeadEnd"), true);
		UE_LOG(LogAAATraffic, Log,
			TEXT("TrafficVehicleController: No provider cached — dead end on lane %d."),
			CurrentLane.HandleId);
		return;
	}

	TArray<FTrafficLaneHandle> Connected = CachedProvider->GetConnectedLanes(CurrentLane);
	AddLaneDecisionTrace(TEXT("TransitionCheck.ConnectedFetched"), 0, static_cast<float>(Connected.Num()), 0.0f, TEXT("Fetched connected lanes"));
	if (Connected.IsEmpty())
	{
		bAtDeadEnd = true;
		AddLaneDecisionTrace(TEXT("TransitionCheck.NoConnected"), 0, 0.0f, 0.0f, TEXT("No connected lanes"));
		FlushLaneDecisionTrace(TEXT("NoConnectedDeadEnd"), true);
		UE_LOG(LogAAATraffic, Log,
			TEXT("TrafficVehicleController: No connected lanes from lane %d — dead end."),
			CurrentLane.HandleId);
		return;
	}

	// --- Weighted lane selection (Feature 4) ---
	// Weight by direction alignment with current lane to prefer "going straight."
	// Sort by HandleId first for determinism, then build a CDF.
	Connected.Sort([](const FTrafficLaneHandle& A, const FTrafficLaneHandle& B)
	{
		return A.HandleId < B.HandleId;
	});

	const FVector CurrentDir = CachedProvider->GetLaneDirection(CurrentLane);

	TArray<float> Weights;
	Weights.Reserve(Connected.Num());
	float TotalWeight = 0.0f;
	for (const FTrafficLaneHandle& Candidate : Connected)
	{
		const FVector CandDir = CachedProvider->GetLaneDirection(Candidate);
		// Weight = dot + 1 shifted to [0,2], clamped with a small minimum so no lane has zero chance,
		// but strongly favoring well-aligned (straight-ahead) lanes.
		const float Dot = FVector::DotProduct(CurrentDir, CandDir);
		const float W = FMath::Max(Dot + 1.0f, 0.01f);
		Weights.Add(W);
		TotalWeight += W;
		if (IsVehicleTraceEnabled(2))
		{
			AddLaneDecisionTrace(
				TEXT("TransitionCheck.CandidateWeight"),
				Candidate.HandleId,
				Dot,
				W,
				FString::Printf(TEXT("CandidateDir=(%.3f,%.3f,%.3f)"), CandDir.X, CandDir.Y, CandDir.Z));
		}
	}

	// Pick via CDF with the seeded random stream.
	const float Roll = RandomStream.FRandRange(0.0f, TotalWeight);
	float CumulativeWeight = 0.0f;
	int32 LaneIndex = Connected.Num() - 1; // fallback to last
	for (int32 i = 0; i < Connected.Num(); ++i)
	{
		CumulativeWeight += Weights[i];
		if (Roll <= CumulativeWeight)
		{
			LaneIndex = i;
			break;
		}
	}

	const FTrafficLaneHandle NextLane = Connected[LaneIndex];
	AddLaneDecisionTrace(
		TEXT("TransitionCheck.SelectedNextLane"),
		NextLane.HandleId,
		Roll,
		TotalWeight,
		FString::Printf(TEXT("LaneIndex=%d CandidateCount=%d"), LaneIndex, Connected.Num()));

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
		OldLaneTangent = (LanePoints.Last() - LanePoints[LanePoints.Num() - 2]).GetSafeNormal();
	}

	// Check if provider has a junction path.
	TArray<FVector> ProviderJunctionPath;
	const bool bHasProviderPath = CachedProvider && CachedProvider->GetJunctionPath(CurrentLane, NextLane, ProviderJunctionPath) && ProviderJunctionPath.Num() >= 2;

	InitializeLaneFollowing(NextLane);

	// If lane following could not be initialized for the selected connected lane,
	// treat this as a dead end so braking logic engages.
	if (!bLaneDataReady)
	{
		bAtDeadEnd = true;
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
	JunctionTransitionIndex = 0;
	if (bHasProviderPath)
	{
		JunctionTransitionPoints = MoveTemp(ProviderJunctionPath);
	}
	else if (!OldLaneTangent.IsNearlyZero() && LanePoints.Num() >= 2)
	{
		// Synthesize a cubic Hermite curve from old-lane-end to new-lane-start.
		const FVector NewLaneStart = LanePoints[0];
		const FVector NewLaneTangent = (LanePoints[1] - LanePoints[0]).GetSafeNormal();
		const float SpanDist = FVector::Dist(OldLaneEnd, NewLaneStart);

		if (SpanDist > 50.0f) // Only if there's meaningful gap to smooth over.
		{
			const float TangentScale = SpanDist * 0.5f;
			const FVector P0 = OldLaneEnd;
			const FVector P1 = NewLaneStart;
			const FVector M0 = OldLaneTangent * TangentScale;
			const FVector M1 = NewLaneTangent * TangentScale;

			constexpr int32 NumSegments = 6;
			JunctionTransitionPoints.Reserve(NumSegments + 1);
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
				JunctionTransitionPoints.Add(H00 * P0 + H10 * M0 + H01 * P1 + H11 * M1);
			}
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
	const float LeaderDist = GetLeaderDistance(LeaderSpeed);
	if (LeaderDist < 0.0f || LeaderDist > DetectionDistance * 0.75f)
	{
		if (IsVehicleTraceEnabled(2))
		{
			AddLaneDecisionTrace(TEXT("LaneChange.RejectLeaderDistance"), 0, LeaderDist, DetectionDistance * 0.75f, TEXT("No close slow leader"));
		}
		return; // No close leader — speed issue is not from congestion.
	}

	const FVector MyDirection = CachedProvider->GetLaneDirection(CurrentLane);

	// Try both sides — deterministic order: Left first, then Right.
	const ETrafficLaneSide Sides[] = { ETrafficLaneSide::Left, ETrafficLaneSide::Right };

	for (ETrafficLaneSide Side : Sides)
	{
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
		UWorld* World = GetWorld();
		UTrafficSubsystem* TrafficSub = World ? World->GetSubsystem<UTrafficSubsystem>() : nullptr;
		if (!TrafficSub) { continue; }

		const FVector VehicleLocation = ControlledPawn->GetActorLocation();
		bool bGapClear = true;

		TArray<TWeakObjectPtr<ATrafficVehicleController>> Neighbors = TrafficSub->GetVehiclesOnLane(CandidateLane);
		for (const TWeakObjectPtr<ATrafficVehicleController>& WeakNeighbor : Neighbors)
		{
			ATrafficVehicleController* Neighbor = WeakNeighbor.Get();
			if (!Neighbor || Neighbor == this) { continue; }

			const APawn* NeighborPawn = Neighbor->GetPawn();
			if (!NeighborPawn) { continue; }

			const float DistToNeighbor = FVector::Dist(VehicleLocation, NeighborPawn->GetActorLocation());
			if (DistToNeighbor < LaneChangeGapRequired)
			{
				bGapClear = false;
				if (IsVehicleTraceEnabled(2))
				{
					AddLaneDecisionTrace(TEXT("LaneChange.RejectGap"), CandidateLane.HandleId, DistToNeighbor, LaneChangeGapRequired, TEXT("Neighbor too close"));
				}
				break;
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
			const float SegDist = FVector::Dist(TargetLanePoints[Idx], TargetLanePoints[Idx + 1]);
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

	// Use direction toward the look-ahead point (lane-aligned) instead of
	// the vehicle's forward vector, which may be temporarily misaligned.
	FVector SweepDirection = ControlledPawn->GetActorForwardVector();
	if (LanePoints.Num() >= 2)
	{
		// Use the cached closest index (updated earlier this frame in UpdateVehicleInput)
		// to avoid calling the non-const FindClosestPointIndex from this const method.
		const int32 ClosestIdx = FMath::Clamp(LastClosestIndex, 0, LanePoints.Num() - 1);
		const int32 NextIdx = FMath::Min(ClosestIdx + 1, LanePoints.Num() - 1);
		SweepDirection = (LanePoints[NextIdx] - VehicleLocation).GetSafeNormal();
		if (SweepDirection.IsNearlyZero())
		{
			SweepDirection = ControlledPawn->GetActorForwardVector();
		}
	}

	// Sphere sweep along the lane direction.
	// Sweep radius is 40% of lane width — wide enough to catch same-lane vehicles
	// without reaching into adjacent lanes (which are typically 100% width apart).
	const float SweepRadiusFraction = 0.4f;
	const float MinSweepRadius = 50.0f;
	const float SweepRadius = FMath::Max(LaneWidth * SweepRadiusFraction, MinSweepRadius);

	// Small offset to push sweep start past our own collision shape.
	const float SelfCollisionBuffer = 10.0f;
	const FVector SweepStart = VehicleLocation + SweepDirection * (SweepRadius + SelfCollisionBuffer);
	const FVector SweepEnd = SweepStart + SweepDirection * DetectionDistance;

	FHitResult HitResult;
	FCollisionQueryParams QueryParams;
	QueryParams.AddIgnoredActor(ControlledPawn);
	QueryParams.bTraceComplex = false;

	const bool bHit = World->SweepSingleByChannel(
		HitResult,
		SweepStart,
		SweepEnd,
		FQuat::Identity,
		ECC_Pawn,
		FCollisionShape::MakeSphere(SweepRadius),
		QueryParams);

	if (!bHit || !HitResult.GetActor())
	{
		return -1.0f;
	}

	// Accept any pawn as a leader — this includes player-driven vehicles so
	// traffic reacts naturally to the player without special-casing.
	const APawn* HitPawn = Cast<APawn>(HitResult.GetActor());
	if (!HitPawn)
	{
		return -1.0f;
	}

	// Extract leader's forward speed.
	if (const UPawnMovementComponent* LeaderMovement = HitPawn->GetMovementComponent())
	{
		// Project leader velocity onto our sweep direction for relative speed.
		OutLeaderSpeed = FVector::DotProduct(LeaderMovement->Velocity, SweepDirection);
	}

	return HitResult.Distance;
}
