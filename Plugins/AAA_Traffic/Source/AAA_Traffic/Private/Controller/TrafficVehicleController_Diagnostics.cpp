// TrafficVehicleController_Diagnostics.cpp — Diagnostic logging and decision tracing.
// Split from TrafficVehicleController.cpp for maintainability.

#include "TrafficVehicleController.h"
#include "TrafficSubsystem.h"
#include "TrafficLog.h"
#include "ChaosWheeledVehicleMovementComponent.h"
#include "GameFramework/Pawn.h"
#include "Engine/World.h"
#include "PhysicsEngine/BodyInstance.h"
#include "Components/PrimitiveComponent.h"

extern int32 GTrafficVehicleDiagnostics;
extern int32 GTrafficVehicleDecisionTraceMax;
extern int32 GTrafficVehicleDecisionTrace;

namespace
{
	static bool IsVehicleTraceEnabled(const int32 RequiredLevel)
	{
		return GTrafficVehicleDecisionTrace >= RequiredLevel;
	}
}

void ATrafficVehicleController::TickDiagnostics(float DeltaSeconds)
{	if (GTrafficVehicleDiagnostics > 0 && !bDiagLoggedMovementCheck && GetPawn())
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

				// Per-wheel diagnostics â€” are wheels touching the ground?
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
					TEXT("DEEP DIAG (2s): Pawn='%s' DistFromSpawn=%.1f â€” "
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
				static_cast<int32>(LaneChangeCoord_.State));

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
// Curvature computation â€” Menger curvature from polyline points
// ---------------------------------------------------------------------------

