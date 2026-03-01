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

static int32 GTrafficJunctionDiagnostics = 0;
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

ATrafficVehicleController::ATrafficVehicleController()
	: LaneWidth(0.f)
	, bLaneDataReady(false)
	, CachedProvider(nullptr)
	, bAtDeadEnd(false)
	, JunctionTransitionIndex(0)
	, bWaitingAtIntersection(false)
	, IntersectionRetryTimer(0.0f)
	, IntersectionJunctionId(0)
	, IntersectionEntryWorldPos(FVector::ZeroVector)
	, bHasIntersectionEntryPos(false)
	, DistanceTraveledOnLane(0.0f)
	, IntersectionWaitElapsed(0.0f)
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
	, LookAheadDistance(500.f)
	, FollowingDistance(300.f)
	, DetectionDistance(2000.f)
	, LaneChangeDistance(1500.f)
	, LaneChangeCooldownTime(5.0f)
	, LaneChangeSpeedThreshold(0.6f)
	, LaneChangeGapRequired(800.f)
	, DefaultSpeedLimit(0.0f)
	, StopLineMarginCm(0.0f)
	, JunctionScanMaxDistanceCm(50000.0f)
	, MaxJunctionScanHops(10)
	, ApproachSafetyMarginCm(500.0f)
	, ApproachDecelCmPerSec2(300.0f)
	, IntersectionSpeedLimitCmPerSec(2000.0f)
	, MaxIntersectionWaitTimeSec(90.0f)
	, VehicleFrontExtent(0.0f)
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
	// ── JUNCTION DIAG: log the state BEFORE we reset it ──
	if (IntersectionJunctionId != 0 || bWaitingAtIntersection || JunctionTransitionPoints.Num() > 0)
	{
		UE_LOG(LogAAATraffic, Warning,
			TEXT("JNCT INIT-RESET: Pawn='%s' BEFORE reset — IntersectionJunctionId=%d "
				 "bWaiting=%s JunctionTransPts=%d NewLane=%d (these values are about to be ERASED)"),
			GetPawn() ? *GetPawn()->GetName() : TEXT("NULL"),
			IntersectionJunctionId,
			bWaitingAtIntersection ? TEXT("YES") : TEXT("NO"),
			JunctionTransitionPoints.Num(),
			InLane.HandleId);
	}

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
	LastReleasedJunctionId = 0;
	IntersectionJunctionLane = FTrafficLaneHandle();
	IntersectionEntryWorldPos = FVector::ZeroVector;
	bHasIntersectionEntryPos = false;
	IntersectionWaitElapsed = 0.0f;
	WaitLogThrottleCounter = 0;

	// Reset approach scan state.
	bApproachingIntersection = false;
	ApproachJunctionDistanceCm = 0.0f;
	ApproachSpeedLimitCmPerSec = 0.0f;
	ApproachJunctionId = 0;
	ApproachJunctionLane = FTrafficLaneHandle();

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
		}

		// --- Compute front-bumper extent from actor bounds ---
		// The bounding box forward half-length tells us how far the front
		// bumper is from the actor origin along the forward axis.
		// This adapts to any vehicle mesh — sedans, trucks, buses.
		{
			const FBox ActorBounds = InPawn->GetComponentsBoundingBox(/*bNonColliding=*/ false);
			if (ActorBounds.IsValid)
			{
				const FVector ActorOrigin = InPawn->GetActorLocation();
				const FVector ActorForward = InPawn->GetActorForwardVector();
				// Project bounding box extents onto the forward axis.
				const FVector HalfExtents = ActorBounds.GetExtent();
				// Use the box center-to-max projected onto forward direction.
				// For a typical vehicle, the forward extent is roughly half the length.
				const FVector BoxCenter = ActorBounds.GetCenter();
				const FVector OriginToMax = ActorBounds.Max - ActorOrigin;
				VehicleFrontExtent = FMath::Abs(FVector::DotProduct(OriginToMax, ActorForward));
				// Clamp to reasonable range to handle edge cases.
				VehicleFrontExtent = FMath::Clamp(VehicleFrontExtent, 50.0f, 1000.0f);
			}
			else
			{
				// Fallback: typical sedan half-length.
				VehicleFrontExtent = 250.0f;
			}
			UE_LOG(LogAAATraffic, Log,
				TEXT("VehicleController::OnPossess: Vehicle '%s' front bumper extent = %.1f cm"),
				*InPawn->GetName(), VehicleFrontExtent);
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
				UE_LOG(LogAAATraffic, Warning,
					TEXT("JNCT RELEASE-UNPOSSESS: Pawn='%s' releasing junction %d on unpossess/destroy"),
					GetPawn() ? *GetPawn()->GetName() : TEXT("NULL"),
					IntersectionJunctionId);
				TrafficSub->ReleaseJunction(IntersectionJunctionId, this);
				IntersectionJunctionId = 0;
				bHasIntersectionEntryPos = false;
			}
			else
			{
				UE_LOG(LogAAATraffic, Warning,
					TEXT("JNCT RELEASE-UNPOSSESS: Pawn='%s' unpossess — IntersectionJunctionId=0, nothing to release"),
					GetPawn() ? *GetPawn()->GetName() : TEXT("NULL"));
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
	UpdateVehicleInput(EffectiveDeltaSeconds);

#if ENABLE_DRAW_DEBUG
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
		if (DbgDistToEntry >= 0.0f || bWaitingAtIntersection)
		{
			const FString Line4 = FString::Printf(TEXT("DistToEntry:%.0f  DesiredStop:%.0f  Jnct:%d"),
				DbgDistToEntry, DbgDesiredStopSpeed, IntersectionJunctionId);
			DrawDebugString(DbgWorld, TextBase + FVector(0, 0, -58), Line4, nullptr,
				FColor::Red, 0.0f, true, 0.9f);
		}

		// --- LINE 4b: Junction Approach Scan info (when scan detected a junction ahead) ---
		if (DbgApproachJunctionDist >= 0.0f)
		{
			const float ApproachMph = DbgApproachSpeedLimit * 0.0223694f;
			const FString Line4b = FString::Printf(TEXT("SCAN: Jnct:%d  Dist:%.0f  VLimit:%.0fmph  %s"),
				DbgApproachJunctionId,
				DbgApproachJunctionDist,
				ApproachMph,
				bApproachingIntersection ? TEXT("BRAKING") : TEXT("OK"));
			const FColor ScanColor = bApproachingIntersection ? FColor(255, 165, 0) : FColor(0, 200, 100);
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
		if (bHasIntersectionEntryPos)
		{
			DrawDebugSphere(DbgWorld, IntersectionEntryWorldPos, 60.0f, 8, FColor::Red, false, -1.0f, 0, 3.0f);
			// Yellow line from car to entry point — shows actual stopping gap.
			DrawDebugLine(DbgWorld, VehicleLoc, IntersectionEntryWorldPos, FColor::Yellow, false, -1.0f, 0, 2.5f);

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
			const FColor ApproachColor = bApproachingIntersection ? FColor(255, 100, 0) : FColor(100, 255, 100);
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

#if ENABLE_DRAW_DEBUG
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
				bApproachingIntersection ? TEXT("YES") : TEXT("NO"),
				ApproachJunctionDistanceCm,
				ApproachJunctionId,
				ApproachSpeedLimitCmPerSec,
				IntersectionJunctionId,
				bWaitingAtIntersection ? TEXT("YES") : TEXT("NO"),
				IntersectionWaitElapsed,
				JunctionTransitionPoints.Num());
		}
	}

	// ────────────────────────────────────────────────────────────────
	// JUNCTION APPROACH — Uses the full precomputed road/junction map.
	// The provider pre-built distance-to-next-junction for EVERY lane
	// at world startup, so this is an O(1) lookup — the vehicle has
	// instant knowledge of the entire road network layout.
	// ────────────────────────────────────────────────────────────────
	if (!bAtDeadEnd && CachedProvider && !bWaitingAtIntersection && IntersectionJunctionId == 0)
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
			ApproachJunctionDistanceCm = ScanResult.DistanceCm;
			ApproachJunctionId = ScanResult.JunctionId;
			ApproachJunctionLane = ScanResult.JunctionLane;

			// Compute the approach speed envelope:
			//   v_approach = sqrt(2 * decel * distToJunction)
			// This is the maximum speed at which the vehicle can still stop
			// before the junction using comfort deceleration.
			const float SafeDist = FMath::Max(ApproachJunctionDistanceCm - ApproachSafetyMarginCm, 0.0f);
			ApproachSpeedLimitCmPerSec = FMath::Sqrt(
				FMath::Max(2.0f * ApproachDecelCmPerSec2 * SafeDist, 0.0f));

			// Only activate approach slowdown when the current speed would
			// require braking — avoids unnecessary throttle limiting when
			// the vehicle is already going slow enough.
			const float AbsSpeedNow = FMath::Abs(CurrentSpeed);
			bApproachingIntersection = (AbsSpeedNow > ApproachSpeedLimitCmPerSec + 50.0f) ||
									   (ApproachJunctionDistanceCm < LookAheadDistance * 3.0f);

#if ENABLE_DRAW_DEBUG
			DbgApproachJunctionDist = ApproachJunctionDistanceCm;
			DbgApproachSpeedLimit = ApproachSpeedLimitCmPerSec;
			DbgApproachJunctionId = ApproachJunctionId;
#endif
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
			bApproachingIntersection = false;
			ApproachJunctionDistanceCm = 0.0f;
			ApproachSpeedLimitCmPerSec = 0.0f;
			ApproachJunctionId = 0;
			ApproachJunctionLane = FTrafficLaneHandle();
#if ENABLE_DRAW_DEBUG
			DbgApproachJunctionDist = -1.0f;
			DbgApproachSpeedLimit = 0.0f;
			DbgApproachJunctionId = 0;
#endif
		}
	}
	else if (IntersectionJunctionId != 0 || bWaitingAtIntersection)
	{
		// Already handling an intersection — disable approach scan.
		bApproachingIntersection = false;
		if (GTrafficJunctionDiagnostics >= 2)
		{
			UE_LOG(LogAAATraffic, Warning,
				TEXT("JDIAG SCAN-SKIP-HANDLING: Pawn='%s' Lane=%d — approach scan disabled "
					 "(already handling intersection: JunctionId=%d bWaiting=%s)"),
				GetPawn() ? *GetPawn()->GetName() : TEXT("NULL"),
				CurrentLane.HandleId,
				IntersectionJunctionId,
				bWaitingAtIntersection ? TEXT("YES") : TEXT("NO"));
		}
#if ENABLE_DRAW_DEBUG
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
					 "bAtDeadEnd=%s CachedProvider=%s bWaiting=%s IntersectionJunctionId=%d. "
					 "THIS VEHICLE CANNOT SEE JUNCTIONS."),
				GetPawn() ? *GetPawn()->GetName() : TEXT("NULL"),
				CurrentLane.HandleId,
				bAtDeadEnd ? TEXT("YES") : TEXT("NO"),
				CachedProvider ? TEXT("VALID") : TEXT("NULL!!!"),
				bWaitingAtIntersection ? TEXT("YES") : TEXT("NO"),
				IntersectionJunctionId);
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
		const float AbsSpeed = FMath::Abs(CurrentSpeed);
		const float StoppingDist = (AbsSpeed * AbsSpeed) / (2.0f * ApproachDecelCmPerSec2);
		const float TransitionThreshold = FMath::Max(LookAheadDistance, StoppingDist + ApproachSafetyMarginCm);
#if ENABLE_DRAW_DEBUG
		DbgRemainingDist = RemainingDist;
		DbgTransitionThreshold = TransitionThreshold;
#endif
		// MinTransitionGuard: small constant to prevent re-triggering on the
		// same frame, but low enough that short virtual segments (created by
		// through-road splitting) still trigger intersection detection.
		constexpr float MinTransitionGuard = 50.0f; // cm — ~1 polyline sample

		// DIAGNOSTIC: Log why the detection gate doesn't fire.
		if (GTrafficJunctionDiagnostics >= 1 && !(RemainingDist < TransitionThreshold && DistanceTraveledOnLane > MinTransitionGuard))
		{
			// Only log once per second per vehicle to avoid total log flood.
			static TMap<uint32, double> LastGateLogTime;
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
					bApproachingIntersection ? TEXT("YES") : TEXT("NO"),
					ApproachJunctionDistanceCm,
					ApproachJunctionId);
			}
		}

		if (RemainingDist < TransitionThreshold && DistanceTraveledOnLane > MinTransitionGuard)
		{
			// --- Intersection approach ---
			// Detect junctions via two methods:
			//   1. Current lane IS a junction lane (GetJunctionForLane(CurrentLane) != 0)
			//   2. Look-ahead: current lane is free-flow but NEXT lane is a junction
			//      (through-road split — free-flow segment preceding intersection)
			if (!bWaitingAtIntersection && CachedProvider)
			{
				int32 DetectedJunctionId = CachedProvider->GetJunctionForLane(CurrentLane);
				// Track which lane handle to pass to IsLaneGreen — must be
				// the lane that appears in the signal's PhaseGroup.GreenLanes.
				FTrafficLaneHandle DetectedJunctionLane = CurrentLane;

				UE_LOG(LogAAATraffic, Warning,
					TEXT("JNCT DETECT: Pawn='%s' CurrentLane=%d GetJunctionForLane(current)=%d "
						 "RemainingDist=%.1f TransitionThreshold=%.1f Speed=%.1f"),
					GetPawn() ? *GetPawn()->GetName() : TEXT("NULL"),
					CurrentLane.HandleId, DetectedJunctionId,
					RemainingDist, TransitionThreshold, CurrentSpeed);

				// --- Look-ahead: peek at next lanes for junction presence ---
				if (DetectedJunctionId == 0)
				{
					TArray<FTrafficLaneHandle> NextLanes = CachedProvider->GetConnectedLanes(CurrentLane);
					for (const FTrafficLaneHandle& NextLane : NextLanes)
					{
						const int32 NextJId = CachedProvider->GetJunctionForLane(NextLane);
						UE_LOG(LogAAATraffic, Warning,
							TEXT("JNCT LOOKAHEAD: Pawn='%s' CurrentLane=%d NextLane=%d "
								 "GetJunctionForLane(next)=%d"),
							GetPawn() ? *GetPawn()->GetName() : TEXT("NULL"),
							CurrentLane.HandleId, NextLane.HandleId, NextJId);
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
				// stored the result in ApproachJunctionId / ApproachJunctionLane.
				// Use that data as a fallback so these vehicles still check
				// signals and occupancy before entering the junction path.
				if (DetectedJunctionId == 0 && ApproachJunctionId != 0)
				{
					DetectedJunctionId = ApproachJunctionId;
					DetectedJunctionLane = ApproachJunctionLane;
					UE_LOG(LogAAATraffic, Warning,
						TEXT("JNCT PRECOMPUTED-HIT: Pawn='%s' CurrentLane=%d — 1-hop LOOKAHEAD "
							 "missed, using precomputed map: JunctionId=%d Dist=%.1f JnctLane=%d"),
						GetPawn() ? *GetPawn()->GetName() : TEXT("NULL"),
						CurrentLane.HandleId,
						ApproachJunctionId,
						ApproachJunctionDistanceCm,
						ApproachJunctionLane.HandleId);
				}

				// Skip re-acquisition when we already hold this junction's occupancy.
				// This prevents re-entering the signal/occupy flow when the vehicle
				// transitions to a new lane still inside the same junction.
				if (DetectedJunctionId != 0 && DetectedJunctionId == IntersectionJunctionId)
				{
					UE_LOG(LogAAATraffic, Warning,
						TEXT("JNCT SKIP-REDETECT: Pawn='%s' JunctionId=%d — already holds "
							 "occupancy, skipping signal check, proceeding to CheckLaneTransition"),
						GetPawn() ? *GetPawn()->GetName() : TEXT("NULL"),
						DetectedJunctionId);
				}
				else if (DetectedJunctionId != 0 && DetectedJunctionId == LastReleasedJunctionId)
				{
					// Junction was just released on this lane (curve-complete fired
					// before lane-end). Don't re-acquire — the vehicle is exiting.
					UE_LOG(LogAAATraffic, Warning,
						TEXT("JNCT SKIP-RELEASED: Pawn='%s' JunctionId=%d — junction was "
							 "just released on this lane, skipping re-acquisition"),
						GetPawn() ? *GetPawn()->GetName() : TEXT("NULL"),
						DetectedJunctionId);
				}
				else if (DetectedJunctionId != 0)
				{
					// Abort any active lane change — intersection takes priority.
					if (LaneChangeState != ELaneChangeState::None)
					{
						UE_LOG(LogAAATraffic, Warning,
							TEXT("JNCT LANE-CHANGE-ABORT: Pawn='%s' JunctionId=%d — "
								 "aborting lane change (state=%d) to handle intersection"),
							GetPawn() ? *GetPawn()->GetName() : TEXT("NULL"),
							DetectedJunctionId, static_cast<int32>(LaneChangeState));
						LaneChangeState = ELaneChangeState::None;
						LaneChangeProgress = 0.0f;
						LaneChangeSettleTimer = 0.0f;
						TargetLanePoints.Empty();
					}

					IntersectionJunctionId = DetectedJunctionId;
					IntersectionJunctionLane = DetectedJunctionLane;

					// Resolve exact intersection entry point for brake targeting.
					if (!bHasIntersectionEntryPos)
					{
						FVector EntryPt;
						if (CachedProvider->GetIntersectionEntryPoint(CurrentLane, EntryPt))
						{
							IntersectionEntryWorldPos = EntryPt;
							bHasIntersectionEntryPos = true;
							UE_LOG(LogAAATraffic, Warning,
								TEXT("JNCT ENTRYPOINT: Pawn='%s' Lane=%d EntryPt=(%.1f,%.1f,%.1f) Source=ExactMask"),
								GetPawn() ? *GetPawn()->GetName() : TEXT("NULL"),
								CurrentLane.HandleId, EntryPt.X, EntryPt.Y, EntryPt.Z);
						}
						else
						{
							// Fallback: use the last polyline point (non-split lanes
							// or lanes without mask data). This is within ±50cm of the
							// actual boundary for 100cm polyline spacing.
							IntersectionEntryWorldPos = LanePoints.Last();
							bHasIntersectionEntryPos = true;
							UE_LOG(LogAAATraffic, Warning,
								TEXT("JNCT ENTRYPOINT: Pawn='%s' Lane=%d EntryPt=(%.1f,%.1f,%.1f) Source=FallbackLast"),
								GetPawn() ? *GetPawn()->GetName() : TEXT("NULL"),
								CurrentLane.HandleId, LanePoints.Last().X, LanePoints.Last().Y, LanePoints.Last().Z);
						}
					}

					UWorld* World = GetWorld();
					UTrafficSubsystem* TrafficSub = World ? World->GetSubsystem<UTrafficSubsystem>() : nullptr;
					if (TrafficSub)
					{
						// Check for signal first — vehicles stop on Red/Yellow.
						// Check for signal first — vehicles stop on Red/Yellow.
						// Use IntersectionJunctionLane (the lane inside the junction) for
						// signal queries — that is the handle stored in GreenLanes.
						ATrafficSignalController* Signal = TrafficSub->GetSignalForJunction(DetectedJunctionId);
						bool bSignalAllows = true;
						if (Signal)
						{
							bSignalAllows = Signal->IsLaneGreen(IntersectionJunctionLane);
						}

						// Compute exit lane for the 4-arg conflict-detection occupy call.
						// From = current lane, To = first connected lane from junction lane.
						IntersectionFromLane = CurrentLane;
						IntersectionToLane = FTrafficLaneHandle();
						{
							TArray<FTrafficLaneHandle> JunctionExits = CachedProvider->GetConnectedLanes(DetectedJunctionLane);
							if (JunctionExits.Num() > 0)
							{
								// Sort for determinism, pick first (direction-weighted selection
								// happens later in CheckLaneTransition, but for conflict detection
								// any consistent exit suffices).
								JunctionExits.Sort([](const FTrafficLaneHandle& A, const FTrafficLaneHandle& B)
								{ return A.HandleId < B.HandleId; });
								IntersectionToLane = JunctionExits[0];
							}
						}

						UE_LOG(LogAAATraffic, Warning,
							TEXT("JNCT OCCUPY-ATTEMPT: Pawn='%s' JunctionId=%d HasSignal=%s "
								 "SignalAllowsGreen=%s DesiredNext=%d ExitLane=%d — about to TryOccupyJunction"),
							GetPawn() ? *GetPawn()->GetName() : TEXT("NULL"),
							DetectedJunctionId,
							Signal ? TEXT("YES") : TEXT("NO"),
							bSignalAllows ? TEXT("YES") : TEXT("NO"),
							DetectedJunctionLane.HandleId,
							IntersectionToLane.HandleId);

						if (bSignalAllows && TrafficSub->TryOccupyJunction(DetectedJunctionId, this, IntersectionFromLane, IntersectionToLane))
						{
							UE_LOG(LogAAATraffic, Warning,
								TEXT("JNCT OCCUPY-GRANTED: Pawn='%s' JunctionId=%d — PROCEEDING through intersection"),
								GetPawn() ? *GetPawn()->GetName() : TEXT("NULL"),
								DetectedJunctionId);
						}
						else
						{
							bWaitingAtIntersection = true;
							UE_LOG(LogAAATraffic, Warning,
								TEXT("JNCT OCCUPY-DENIED: Pawn='%s' JunctionId=%d SignalAllows=%s "
									 "— WAITING at intersection"),
								GetPawn() ? *GetPawn()->GetName() : TEXT("NULL"),
								DetectedJunctionId,
								bSignalAllows ? TEXT("YES-occupied") : TEXT("NO-signal-red"));
						}
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

		// --- Intersection waiting: fires UNCONDITIONALLY when bWaitingAtIntersection ---
		// FIX: Moved outside the (RemainingDist < TransitionThreshold) gate.
		// Previously, braking dropped out when the threshold shrank with speed,
		// causing vehicles to coast through lane ends at ~470 cm/s.
		if (bWaitingAtIntersection)
		{
			UWorld* World = GetWorld();
			UTrafficSubsystem* TrafficSub = World ? World->GetSubsystem<UTrafficSubsystem>() : nullptr;

			// Check for stuck bug: waiting flag is true but junction ID was
			// erased (e.g., by an external InitializeLaneFollowing call).
			//
			// FIX (was CRITICAL): Previously only logged the error and left the
			// vehicle stuck FOREVER. Now auto-recovers by clearing the waiting
			// flag so the vehicle resumes lane following on its current lane.
			if (IntersectionJunctionId == 0)
			{
				UE_LOG(LogAAATraffic, Error,
					TEXT("JNCT BUG-RECOVERED: Pawn='%s' bWaitingAtIntersection=YES but "
						 "IntersectionJunctionId=0! Clearing waiting flag to resume driving. "
						 "Root cause: InitializeLaneFollowing erased the ID while waiting."),
					GetPawn() ? *GetPawn()->GetName() : TEXT("NULL"));
				bWaitingAtIntersection = false;
				bHasIntersectionEntryPos = false;
				IntersectionRetryTimer = 0.0f;
				// Fall through — vehicle will resume normal lane following.
			}
			else if (TrafficSub)
			{
				// Accumulate wait time for timeout detection.
				IntersectionWaitElapsed += DeltaSeconds;

				// Throttle retry attempts to ~4 per second to avoid log spam.
				// Without this, TryOccupyJunction fires + logs DENIED every tick.
				IntersectionRetryTimer -= DeltaSeconds;
				if (IntersectionRetryTimer <= 0.0f)
				{
					IntersectionRetryTimer = 0.25f; // retry every 250ms

					bool bSignalAllows = true;
					ATrafficSignalController* Signal = TrafficSub->GetSignalForJunction(IntersectionJunctionId);
					if (Signal)
					{
						bSignalAllows = Signal->IsLaneGreen(IntersectionJunctionLane);
					}

					if (bSignalAllows && TrafficSub->TryOccupyJunction(IntersectionJunctionId, this, IntersectionFromLane, IntersectionToLane))
					{
						bWaitingAtIntersection = false;
						IntersectionWaitElapsed = 0.0f;

						// Safety: if the vehicle overshot the entry point while waiting,
						// teleport it back so FindClosestPointIndex maps correctly on the
						// next lane.
						if (bHasIntersectionEntryPos)
						{
							const float OvershootDist = FVector::Dist(VehicleLocation, IntersectionEntryWorldPos);
							if (OvershootDist > 500.0f) // > 5m overshoot
							{
								ControlledPawn->SetActorLocation(
									IntersectionEntryWorldPos + FVector(0.0f, 0.0f, 10.0f),
									false, nullptr, ETeleportType::TeleportPhysics);
								UE_LOG(LogAAATraffic, Warning,
									TEXT("JNCT OVERSHOOT-TELEPORT: Pawn='%s' was %.1f cm from entry — "
										 "teleported to entry point"),
									GetPawn() ? *GetPawn()->GetName() : TEXT("NULL"),
									OvershootDist);
							}
						}

						bHasIntersectionEntryPos = false;
						UE_LOG(LogAAATraffic, Warning,
							TEXT("JNCT RETRY-GRANTED: Pawn='%s' JunctionId=%d — CLEARED to proceed (was waiting, elapsed %.1fs)"),
							GetPawn() ? *GetPawn()->GetName() : TEXT("NULL"),
							IntersectionJunctionId,
							IntersectionWaitElapsed);
						// Fall through to lane transition gate below.
					}
				}

				// FIX (was MAJOR): Timeout — if the vehicle has been waiting
				// longer than MaxIntersectionWaitTimeSec, force-proceed to
				// prevent permanent deadlocks. The junction may be blocked by
				// a stuck vehicle, broken signal, or mutual denial cycle.
				if (bWaitingAtIntersection && IntersectionWaitElapsed >= MaxIntersectionWaitTimeSec)
				{
					UE_LOG(LogAAATraffic, Warning,
						TEXT("JNCT TIMEOUT-FORCE-PROCEED: Pawn='%s' JunctionId=%d — "
							 "waited %.1fs (max %.1fs). Force-occupying to break deadlock."),
						GetPawn() ? *GetPawn()->GetName() : TEXT("NULL"),
						IntersectionJunctionId,
						IntersectionWaitElapsed,
						MaxIntersectionWaitTimeSec);

					// Force-occupy: bypass TryOccupyJunction — just mark as
					// proceeding. The junction system will still track us for
					// release, but we won't wait for permission anymore.
					bWaitingAtIntersection = false;
					IntersectionWaitElapsed = 0.0f;
					bHasIntersectionEntryPos = false;
				}
			}

			if (bWaitingAtIntersection)
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
						IntersectionJunctionId,
						bHasIntersectionEntryPos
							? FVector::Dist(VehicleLocation, IntersectionEntryWorldPos)
							: RemainingDist,
						CurrentSpeed,
						IntersectionWaitElapsed);
				}

				// Decel-curve brake toward intersection entry point.
				// Compute distance to entry, checking for overshoot.
				float DistToEntry = RemainingDist;
				if (bHasIntersectionEntryPos)
				{
					const FVector ToEntry = IntersectionEntryWorldPos - VehicleLocation;
					const float DotFwd = FVector::DotProduct(ToEntry, VehicleForward);
					DistToEntry = (DotFwd > 0.0f)
						? FVector::Dist(VehicleLocation, IntersectionEntryWorldPos)
						: 0.0f; // Overshot — treat as zero.
				}

				// Desired speed from constant-decel stopping curve: v = sqrt(2*a*d)
				// Uses the same tunable decel as the approach speed envelope
				// so both systems agree on braking physics.
				const float DesiredStopSpeed = FMath::Sqrt(
					FMath::Max(2.0f * ApproachDecelCmPerSec2 * DistToEntry, 0.0f));

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
					VehicleMovement->SetSteeringInput(0.0f);
					VehicleMovement->SetBrakeInput(WaitBrake);
				}
				else
				{
					// Under envelope — coast (no acceleration toward intersection).
					VehicleMovement->SetThrottleInput(0.0f);
					VehicleMovement->SetSteeringInput(0.0f);
					VehicleMovement->SetBrakeInput(0.0f);
				}

#if ENABLE_DRAW_DEBUG
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
			const bool bJunctionBypass = (IntersectionJunctionId != 0);
			if (RemainingDist < TransitionThreshold
				&& (DistanceTraveledOnLane > MinTransitionGuard || bJunctionBypass))
			{
				if (LaneChangeState != ELaneChangeState::None)
				{
					return;
				}

				CheckLaneTransition();
				if (!bLaneDataReady || !GetPawn())
				{
					return;
				}
				if (!bAtDeadEnd)
				{
					if (IntersectionJunctionId != 0)
					{
						const int32 NewLaneJunction = CachedProvider
							? CachedProvider->GetJunctionForLane(CurrentLane) : 0;

						if (NewLaneJunction != IntersectionJunctionId)
						{
							UWorld* World = GetWorld();
							UTrafficSubsystem* TrafficSub = World
								? World->GetSubsystem<UTrafficSubsystem>() : nullptr;
							if (TrafficSub)
							{
								UE_LOG(LogAAATraffic, Warning,
									TEXT("JNCT RELEASE: Pawn='%s' RELEASING junction %d — new lane %d "
										 "is in junction %d (exited)"),
									GetPawn() ? *GetPawn()->GetName() : TEXT("NULL"),
									IntersectionJunctionId, CurrentLane.HandleId, NewLaneJunction);
								TrafficSub->ReleaseJunction(IntersectionJunctionId, this);
							}
							IntersectionJunctionId = 0;
							bHasIntersectionEntryPos = false;
						}
						else
						{
							UE_LOG(LogAAATraffic, Warning,
								TEXT("JNCT HOLD: Pawn='%s' junction %d — new lane %d still in "
									 "same junction, keeping occupancy"),
								GetPawn() ? *GetPawn()->GetName() : TEXT("NULL"),
								IntersectionJunctionId, CurrentLane.HandleId);
						}
					}
					return;
				}
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

			UE_LOG(LogAAATraffic, Warning,
				TEXT("JNCT RELEASE-PATH2-CHECK: Pawn='%s' IntersectionJunctionId=%d "
					 "— junction curve complete, checking release"),
				GetPawn() ? *GetPawn()->GetName() : TEXT("NULL"),
				IntersectionJunctionId);

			if (IntersectionJunctionId != 0)
			{
				if (UWorld* World = GetWorld())
				{
					if (UTrafficSubsystem* TrafficSub = World->GetSubsystem<UTrafficSubsystem>())
					{
						UE_LOG(LogAAATraffic, Warning,
							TEXT("JNCT RELEASE-PATH2-FIRE: Pawn='%s' RELEASING junction %d (curve complete)"),
							GetPawn() ? *GetPawn()->GetName() : TEXT("NULL"),
							IntersectionJunctionId);
						TrafficSub->ReleaseJunction(IntersectionJunctionId, this);
					}
				}
				// Record the released junction so the detection code won't
				// re-acquire it on the remainder of this lane.
				LastReleasedJunctionId = IntersectionJunctionId;
				IntersectionJunctionId = 0;
				bHasIntersectionEntryPos = false;
			}
			else
			{
				UE_LOG(LogAAATraffic, Warning,
					TEXT("JNCT RELEASE-PATH2-DEAD: Pawn='%s' IntersectionJunctionId=0 "
						 "— CANNOT release (ID was erased by InitializeLaneFollowing)"),
					GetPawn() ? *GetPawn()->GetName() : TEXT("NULL"));
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
#if ENABLE_DRAW_DEBUG
			DbgLeaderDist = LeaderDist;
			DbgLeaderSpeed = LeaderSpeed;
#endif
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

	// Junction approach speed limit: progressively slow down as a junction
	// enters the scan range. Uses the decel-envelope speed computed by the
	// approach scan earlier this tick. This fires BEFORE the detection gate
	// so the vehicle decelerates smoothly instead of slamming the brakes.
	//
	// FIX (was CRITICAL): The old code used:
	//   EffectiveTargetSpeed = FMath::Max(ApproachSpeedLimitCmPerSec, 100.0f);
	// This could RAISE speed above leader-following (e.g., leader = 80,
	// approach = 50 → max(50,100) = 100 → override leader to 100 → rear-end
	// collision). Now we only clamp DOWN, never up.
	if (bApproachingIntersection)
	{
		const float ApproachCap = FMath::Max(ApproachSpeedLimitCmPerSec, 100.0f);
		if (ApproachCap < EffectiveTargetSpeed)
		{
			EffectiveTargetSpeed = ApproachCap;
		}
#if ENABLE_DRAW_DEBUG
		DbgStateName = TEXT("APPROACH");
#endif
	}

	// Intersection speed limit: cap speed while traversing a junction.
	// Uses the tunable IntersectionSpeedLimitCmPerSec as a baseline,
	// then modulates based on the junction path's curvature.
	//
	// FIX (was MAJOR): Replaced hardcoded 500 cm/s (~11 mph) that applied
	// uniformly to ALL intersections (highway merge, 4-way stop, etc.)
	// with a tunable property + curvature-derived speed so straight-through
	// traffic keeps moving and tight turns slow appropriately.
	if (IntersectionJunctionId != 0)
	{
		float JunctionSpeedCap = IntersectionSpeedLimitCmPerSec;

		// Derive speed from junction path curvature when available:
		// walk the junction transition polyline, measure the tightest
		// turning angle, convert to a centripetal-safe speed.
		if (JunctionTransitionPoints.Num() >= 3)
		{
			// Compute total arc length and cumulative turn angle.
			float TotalAngleDeg = 0.0f;
			float TotalArcLength = 0.0f;
			for (int32 i = 1; i < JunctionTransitionPoints.Num() - 1; ++i)
			{
				const FVector Seg0 = (JunctionTransitionPoints[i] - JunctionTransitionPoints[i - 1]);
				const FVector Seg1 = (JunctionTransitionPoints[i + 1] - JunctionTransitionPoints[i]);
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
			TotalArcLength += (JunctionTransitionPoints.Last() - JunctionTransitionPoints[JunctionTransitionPoints.Num() - 2]).Size();

			// Approximate turning radius: R = arcLength / totalAngle(radians).
			// Then centripetal speed limit: v = sqrt(lateralAccel * R).
			const float TotalAngleRad = FMath::DegreesToRadians(FMath::Max(TotalAngleDeg, 1.0f));
			const float TurnRadius = FMath::Max(TotalArcLength / TotalAngleRad, 50.0f); // clamp to 0.5m min

			// Lateral accel budget: ~0.3g comfortable, in cm/s².
			constexpr float LateralAccelBudget = 294.0f; // 0.3 * 980 cm/s²
			const float CurvatureSpeed = FMath::Sqrt(LateralAccelBudget * TurnRadius);

			// Use the tighter of curvature-derived and baseline limit.
			JunctionSpeedCap = FMath::Min(JunctionSpeedCap, CurvatureSpeed);
		}

		EffectiveTargetSpeed = FMath::Min(EffectiveTargetSpeed, JunctionSpeedCap);
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

#if ENABLE_DRAW_DEBUG
	DbgThrottle = ThrottleInput;
	DbgBrake = BrakeInput;
	DbgSteering = SteeringInput;
	DbgEffectiveTargetSpeed = EffectiveTargetSpeed;
	DbgTargetPoint = TargetPoint;
	// State priority: junction > following > braking > cruising.
	if (IntersectionJunctionId != 0)
	{
		// Distinguish: "IN-JNCT" when on the actual junction lane,
		// "INT" when holding occupancy but on an approach or exit lane.
		const int32 CurrLaneJnct = CachedProvider
			? CachedProvider->GetJunctionForLane(CurrentLane) : 0;
		DbgStateName = (CurrLaneJnct == IntersectionJunctionId)
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

	// ── FIX: Save junction state BEFORE InitializeLaneFollowing erases it ──
	// InitializeLaneFollowing resets IntersectionJunctionId to 0, which
	// prevents the caller's release paths from ever firing. Save the ID
	// so we can restore it after the reset.
	const int32 SavedJunctionId = IntersectionJunctionId;
	const FTrafficLaneHandle SavedJunctionLane = IntersectionJunctionLane;
	const bool bSavedHasEntryPos = bHasIntersectionEntryPos;

	UE_LOG(LogAAATraffic, Warning,
		TEXT("JNCT TRANSITION-PRE-INIT: Pawn='%s' OldLane=%d NextLane=%d "
			 "IntersectionJunctionId=%d bWaiting=%s bHasEntryPos=%s JunctionTransPts=%d "
			 "— about to call InitializeLaneFollowing (junction state saved)"),
		GetPawn() ? *GetPawn()->GetName() : TEXT("NULL"),
		CurrentLane.HandleId, NextLane.HandleId,
		SavedJunctionId,
		bWaitingAtIntersection ? TEXT("YES") : TEXT("NO"),
		bSavedHasEntryPos ? TEXT("YES") : TEXT("NO"),
		JunctionTransitionPoints.Num());

	InitializeLaneFollowing(NextLane);

	// Restore the junction ID and lane so the caller's release paths can fire.
	IntersectionJunctionId = SavedJunctionId;
	IntersectionJunctionLane = SavedJunctionLane;
	bHasIntersectionEntryPos = bSavedHasEntryPos;

	UE_LOG(LogAAATraffic, Warning,
		TEXT("JNCT TRANSITION-POST-INIT: Pawn='%s' NewLane=%d "
			 "IntersectionJunctionId=%d bWaiting=%s (junction ID restored)"),
		GetPawn() ? *GetPawn()->GetName() : TEXT("NULL"),
		NextLane.HandleId,
		IntersectionJunctionId,
		bWaitingAtIntersection ? TEXT("YES") : TEXT("NO"));

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
