// TrafficVehicleController_UpdateInput.cpp -- Per-tick driving AI and leader detection.
// Split from TrafficVehicleController.cpp for maintainability.

#include "TrafficVehicleController.h"
#include "TrafficSubsystem.h"
#include "TrafficSignalController.h"
#include "TrafficLog.h"
#include "ChaosWheeledVehicleMovementComponent.h"
#include "GameFramework/Pawn.h"
#include "Engine/World.h"
#include "PhysicsEngine/BodyInstance.h"
#include "Components/PrimitiveComponent.h"

extern float GTrafficWakeGuardMinThrottle;
extern float GTrafficWakeGuardMaxSpeed;
extern int32 GTrafficJunctionDiagnostics;
extern int32 GTrafficDebugDraw;
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
	// -> Optimization Unpossessed -> CE_StopCar -> CE Set Parked(true) chain.
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
	// SetSleeping(false) calls WakeAllEnabledRigidBodies -- only invoke when actually sleeping.
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

	// -- Compute adaptive distances from time-based parameters --
	// Each distance = |CurrentSpeed| * TimeSec, clamped to a floor so
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
	// Initialize debug cache each tick -- will be overwritten by decision branches.
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

	// Tick lane-change cooldown + evaluate.
	LaneChangeCoord_.TickCooldown(DeltaSeconds);
	if (LaneChangeCoord_.ShouldEvaluate())
	{
		// Assemble eval context.
		FLaneChangeEvalContext LCCtx;
		LCCtx.VehicleLocation   = VehicleLocation;
		LCCtx.EgoForward        = ControlledPawn->GetActorForwardVector();
		LCCtx.CurrentSpeed      = FMath::Abs(CurrentSpeed);
		LCCtx.TargetSpeed       = TargetSpeed;
		LCCtx.EgoForwardSpeed   = FMath::Abs(CurrentSpeed);
		LCCtx.VehicleFrontExtent = VehicleFrontExtent;
		LCCtx.VehicleRearExtent  = VehicleRearExtent;
		LCCtx.DetectionDistance   = DetectionDistance;
		LCCtx.CurrentLane        = CurrentLane;
		LCCtx.LanePoints         = &LanePoints;
		LCCtx.LastClosestIndex   = LastClosestIndex;
		LCCtx.bLaneDataReady     = bLaneDataReady;
		LCCtx.bJunctionApproaching = (JnctState.Phase == EJunctionPhase::Approaching);

		// Pre-compute leader distance for non-navigational evaluation.
		float LeaderSpeedForLC = 0.0f;
		LCCtx.LeaderDist = GetLeaderDistance(LeaderSpeedForLC);

		LCCtx.RoadProvider = CachedProvider;
		if (UWorld* W = GetWorld())
		{
			LCCtx.TrafficSub = W->GetSubsystem<UTrafficSubsystem>();
		}

		const FLaneChangeEvalResult LCResult = LaneChangeCoord_.Evaluate(LCCtx);
		if (LCResult.bStarted)
		{
			SetTurnSignal(LCResult.TurnSignal);
		}
	}

	// Find where we are on the lane and where to aim
	const int32 ClosestIndex = FindClosestPointIndex(VehicleLocation);

	// -- PERIODIC JUNCTION DIAGNOSTIC DUMP (every ~2s per vehicle) --
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


	// -- Junction approach scan (delegated to FJunctionNegotiator) --
	bApproachBraking = JunctionNeg_.TickApproach(ClosestIndex, AbsSpeed, CurrentSpeed);

	// --- Lane-end detection + intersection right-of-way ---
	// Allow entry even during an active lane change: if we're approaching a
	// lane end, the intersection check is more important than the lane change.
	// The lane change will be aborted below if an intersection is detected.
	if (!bAtDeadEnd)
	{
		const float RemainingDist = GetRemainingDistance(ClosestIndex);
		// Use physics-based stopping-distance threshold so fast vehicles get
		// enough advance warning to actually stop before the intersection.
		// Old formula: max(LookAhead, |Speed| * 1.0s) -- only 1s of reaction time.
		// New formula: max(LookAhead, v^2/(2*decel) + margin) -- matches actual braking physics.
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
		constexpr float MinTransitionGuard = 50.0f; // cm -- ~1 polyline sample

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
						 "-- Vehicle is NOT close enough to lane end for junction detection."),
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
			JunctionNeg_.TickDetectAndOccupy(RemainingDist, TransitionThreshold, AbsSpeed, CurrentSpeed, VehicleLocation);
		}


// --- Intersection waiting (delegated to TickIntersectionWaiting) ---
if (TickIntersectionWaiting(DeltaSeconds, CurrentSpeed, VehicleLocation, VehicleForward, ClosestIndex, RemainingDist, VehicleMovement))
{
return;
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
				if (LaneChangeCoord_.State != ELaneChangeState::None)
				{
					// Lane change was still in progress when we reached the
					// transition threshold.  Abort the change so the vehicle
					// can proceed to CheckLaneTransition on the current lane
					// instead of being stuck in an incomplete merge.
					LaneChangeCoord_.Reset();
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
						JunctionNeg_.TickPostTransitionRelease();
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

	// Junction transition takes priority -- follow synthesized curve first.
	if (JunctionNeg_.TickTraverse(VehicleLocation, ClosestIndex, TargetPoint))
	{
		// TargetPoint set by junction traversal.
	}
	else if (LaneChangeCoord_.State == ELaneChangeState::Executing
		|| LaneChangeCoord_.State == ELaneChangeState::Completing)
	{
		// Assemble blend context.
		FLaneChangeBlendContext BlendCtx;
		BlendCtx.VehicleLocation   = VehicleLocation;
		BlendCtx.SourcePoint       = GetLookAheadPoint(VehicleLocation, ClosestIndex);
		BlendCtx.ClosestIndex      = ClosestIndex;
		BlendCtx.LookAheadDistance = LookAheadDistance;
		BlendCtx.DistanceThisTick  = DistanceThisTick;
		if (UWorld* W = GetWorld())
		{
			BlendCtx.TrafficSub = W->GetSubsystem<UTrafficSubsystem>();
		}

		if (LaneChangeCoord_.State == ELaneChangeState::Completing)
		{
			if (LaneChangeCoord_.TickSettle(DeltaSeconds))
			{
				// Settle timer expired -- finalize.
				FLaneChangeFinalizeResult FinalResult =
					LaneChangeCoord_.Finalize(CachedProvider, DefaultSpeedLimit, BaseTargetSpeed);
				CurrentLane           = FinalResult.NewLane;
				LanePoints            = MoveTemp(FinalResult.NewLanePoints);
				LaneWidth             = FinalResult.NewLaneWidth;
				TargetSpeed           = FinalResult.NewTargetSpeed;
				DistanceTraveledOnLane = 0.0f;
				LastClosestIndex      = 0;
				SteeringComputer.Reset();
				TurnSignalOffDelayRemaining = 1.0f;
				if (UWorld* W = GetWorld())
				{
					if (UTrafficSubsystem* TSub = W->GetSubsystem<UTrafficSubsystem>())
					{
						TSub->UpdateVehicleLane(this, CurrentLane);
					}
				}
				TargetPoint = GetLookAheadPoint(VehicleLocation, ClosestIndex);
			}
			else
			{
				// Still settling -- keep blending toward target lane.
				const FLaneChangeBlendResult BR = LaneChangeCoord_.Blend(BlendCtx);
				TargetPoint = BR.Point;
			}
		}
		else
		{
			// Executing: run blend (may abort internally).
			const FLaneChangeBlendResult BR = LaneChangeCoord_.Blend(BlendCtx);
			TargetPoint = BR.Point;
			if (BR.bAborted)
			{
				SetTurnSignal(ETurnSignalState::Off);
			}
		}
	}
	else
	{
		TargetPoint = GetLookAheadPoint(VehicleLocation, ClosestIndex);
	}

	// --- Steering (delegated to FSteeringComputer) ---
	// Resolve effective lane width from provider before calling.
	float EffectiveLaneWidth = LaneWidth;
	if (CachedProvider && CurrentLane.IsValid())
	{
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

	FSteeringInput SteerIn;
	SteerIn.VehicleLocation    = VehicleLocation;
	SteerIn.VehicleForward     = VehicleForward;
	SteerIn.TargetPoint        = TargetPoint;
	SteerIn.LanePoints         = LanePoints;
	SteerIn.ClosestIndex       = ClosestIndex;
	SteerIn.JunctionCurvePoints = JnctState.TransitionPoints;
	SteerIn.JunctionCurveIndex = JnctState.TransitionIndex;
	SteerIn.WheelbaseCm        = VehicleWheelbaseCm;
	SteerIn.MaxSteerAngleRad   = VehicleMaxSteerAngleRad;
	SteerIn.EffectiveLaneWidth = EffectiveLaneWidth;
	SteerIn.CTECorrectionGain  = CTECorrectionGain;
	SteerIn.SteeringDampingFactor = SteeringDampingFactor;
	SteerIn.CurveScanWindowSize = CurveScanWindowSize;
	SteerIn.DeltaSeconds       = DeltaSeconds;

	const FSteeringOutput SteerOut = SteeringComputer.Compute(SteerIn);
	const float SteeringInput = SteerOut.SteeringInput;

	// --- Steering diagnostic logging (throttled) ---
	++SteeringDiagCounter;
	if (SteeringDiagCounter >= 60)
	{
		SteeringDiagCounter = 0;
		const float CTEpct = (SteerOut.HalfLaneWidth > 0.0f)
			? (FMath::Abs(SteerOut.SignedCTE) / SteerOut.HalfLaneWidth * 100.0f) : 0.0f;
		UE_LOG(LogAAATraffic, Log,
			TEXT("STEER DIAG: Pawn='%s' Lane=%d CTE=%.1fcm (%.0f%%) "
				 "FF=%.3f P=%.3f CTE=%.3f D=%.3f Total=%.3f Speed=%.0f LookAhead=%.0f k=%.6f"),
			GetPawn() ? *GetPawn()->GetName() : TEXT("NULL"),
			CurrentLane.HandleId,
			SteerOut.SignedCTE, CTEpct,
			SteerOut.FeedforwardTerm, SteerOut.PurePursuitTerm,
			SteerOut.CTETerm, SteerOut.DerivativeTerm, SteeringInput,
			CurrentSpeed, LookAheadDistance, SteerOut.LocalCurvature);
	}

	// Lane departure warning: log when CTE exceeds 80% of half lane width.
	if (FMath::Abs(SteerOut.SignedCTE) > SteerOut.HalfLaneWidth * 0.8f)
	{
		UE_LOG(LogAAATraffic, Warning,
			TEXT("LANE DEPARTURE: Pawn='%s' Lane=%d CTE=%.1fcm (%.0f%% of half-lane) "
				 "Speed=%.0f Steer=%.3f"),
			GetPawn() ? *GetPawn()->GetName() : TEXT("NULL"),
			CurrentLane.HandleId,
			SteerOut.SignedCTE,
			(SteerOut.HalfLaneWidth > 0.0f ? FMath::Abs(SteerOut.SignedCTE) / SteerOut.HalfLaneWidth * 100.0f : 0.0f),
			CurrentSpeed, SteeringInput);
	}

	// --- Throttle / Brake (IDM -- Intelligent Driver Model) ---
	// Step 1: Detect leader vehicle (LOD-dependent method).
// Step 2: Compute desired speed v0 (road limit + junction/curve caps).
	// Step 3: IDM acceleration: a_idm = a[1 - (v/v0)^4 - (s*/s)^2],
	//         s* = s0 + max(0, vT + v*dv/(2*sqrt(ab))).
	// Step 4: Map acceleration to throttle/brake inputs.

	// -- Step 1: Leader detection (delegated to LeaderDetector) --
	FLeaderDetectorInput LdrIn;
	LdrIn.VehicleLocation        = VehicleLocation;
	LdrIn.VehicleForward         = ControlledPawn->GetActorForwardVector();
	LdrIn.VehicleFrontExtent     = VehicleFrontExtent;
	LdrIn.DetectionDistance      = DetectionDistance;
	LdrIn.LaneWidth              = LaneWidth;
	LdrIn.LanePoints             = LanePoints;
	LdrIn.ClosestIndex           = LastClosestIndex;
	LdrIn.LaneChangeState        = LaneChangeCoord_.State;
	LdrIn.CurrentLane            = CurrentLane;
	LdrIn.RoadProvider           = CachedProvider;
	LdrIn.bLaneDataReady         = bLaneDataReady;
	LdrIn.JunctionId             = JnctState.JunctionId;
	LdrIn.JunctionTransitionPoints = JnctState.TransitionPoints;
	LdrIn.JunctionTransitionIndex  = JnctState.TransitionIndex;
	{
		ETrafficLOD TickLOD = ETrafficLOD::Full;
		UTrafficSubsystem* LdrSub = nullptr;
		if (UWorld* LdrWorld = GetWorld())
		{
			LdrSub = LdrWorld->GetSubsystem<UTrafficSubsystem>();
			if (LdrSub) { TickLOD = LdrSub->GetVehicleLOD(this); }
		}
		LdrIn.TickLOD            = TickLOD;
		LdrIn.TrafficSubsystem   = LdrSub;
	}
	LdrIn.World                  = GetWorld();
	LdrIn.ControlledPawn         = ControlledPawn;
	LdrIn.IDMReactionDelaySec    = IDMReactionDelaySec;

	const FLeaderDetectorOutput LdrOut = LeaderDetector_.Detect(LdrIn);
	float LeaderDist = LdrOut.LeaderDist;
	float LeaderSpeed = LdrOut.LeaderSpeed;
	bool bLeaderBrakeLightsVisible = LdrOut.bLeaderBrakeLightsVisible;

	// -- Step 2: Desired speed (IDM v0) -- delegated to SpeedEnvelope --
	FSpeedEnvelopeInput SpeedIn;
	SpeedIn.TargetSpeed                    = TargetSpeed;
	SpeedIn.CurrentSpeed                   = AbsSpeed;
	SpeedIn.bApproachBraking               = bApproachBraking;
	SpeedIn.ApproachSpeedLimitCmPerSec     = JnctState.ApproachSpeedLimitCmPerSec;
	SpeedIn.RoadProvider                   = CachedProvider;
	SpeedIn.CurrentLane                    = CurrentLane;
	SpeedIn.bLaneDataReady                 = bLaneDataReady;
	SpeedIn.CurrentJunctionId              = JnctState.JunctionId;
	SpeedIn.RemainingDistOnLane            = GetRemainingDistance(FindClosestPointIndex(VehicleLocation));
	SpeedIn.ApproachDecelCmPerSec2         = ApproachDecelCmPerSec2;
	SpeedIn.ApproachSafetyMarginCm         = ApproachSafetyMarginCm;
	SpeedIn.IntersectionSpeedLimitCmPerSec = IntersectionSpeedLimitCmPerSec;
	SpeedIn.LateralAccelBudgetCmPerSec2    = LateralAccelBudgetCmPerSec2;
	SpeedIn.JunctionTransitionPoints       = JnctState.TransitionPoints;
	SpeedIn.JunctionCurveStartIndex        = JnctState.CurveStartIndex;
	SpeedIn.LanePoints                     = LanePoints;
	SpeedIn.ClosestIndex                   = LastClosestIndex;
	SpeedIn.bFirstTickOnLane               = bFirstTickOnLane;
	SpeedIn.LookAheadDistance              = LookAheadDistance;
	SpeedIn.MaxBrakeDecelCmPerSec2         = MaxBrakeDecelCmPerSec2;
	SpeedIn.CurveScanWindowSize            = CurveScanWindowSize;
	SpeedIn.CurveSpeedSafetyFactor         = CurveSpeedSafetyFactor;
	SpeedIn.ComfortDecelCmPerSec2          = IDMComfortDecelCmPerSec2;

	const FSpeedEnvelopeOutput SpeedOut = SpeedEnvelope_.Compute(SpeedIn);
	float EffectiveTargetSpeed = SpeedOut.EffectiveTargetSpeed;

	if (SpeedOut.bFirstTickOnLaneConsumed)
	{
		bFirstTickOnLane = false;
	}
#if defined(ENABLE_DRAW_DEBUG) && ENABLE_DRAW_DEBUG
	if (!SpeedOut.DbgAppliedCap.IsEmpty())
	{
		DbgStateName = SpeedOut.DbgAppliedCap;
	}
#endif

	// -- Merge cooperation ------------------------------------
	// If a vehicle on an adjacent lane is actively lane-changing toward
	// this vehicle's lane AND is alongside (within detection range),
	// reduce effective speed to create a merge gap.
	if (CachedProvider && bLaneDataReady && LaneChangeCoord_.State == ELaneChangeState::None)
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
					if (Adj->LaneChangeCoord_.State != ELaneChangeState::Executing) { continue; }
					if (Adj->LaneChangeCoord_.TargetLane != CurrentLane) { continue; }
					// Is it alongside us (within +/-DetectionDistance)?
					const APawn* AdjPawn = Adj->GetPawn();
					if (!AdjPawn) { continue; }
					const float LongDist = FVector::DotProduct(
						AdjPawn->GetActorLocation() - VehicleLocation,
						ControlledPawn->GetActorForwardVector());
					// Adjacent vehicle is ahead but close -- slow down to create gap.
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

	// -- Step 3+4: IDM acceleration -> throttle/brake (delegated) --
	FAccelerationInput AccelIn;
	AccelIn.CurrentSpeed             = CurrentSpeed;
	AccelIn.DesiredSpeed             = EffectiveTargetSpeed;
	AccelIn.RawLeaderDist            = LeaderDist;
	AccelIn.RawLeaderSpeed           = LeaderSpeed;
	AccelIn.bLeaderBrakeLightsVisible = bLeaderBrakeLightsVisible;
	AccelIn.bWaitingAtJunction       = JnctState.bWaiting;
	AccelIn.DeltaSeconds             = DeltaSeconds;
	AccelIn.MaxAccelCmPerSec2        = IDMMaxAccelCmPerSec2;
	AccelIn.ComfortDecelCmPerSec2    = IDMComfortDecelCmPerSec2;
	AccelIn.TimeHeadwaySec           = FollowingTimeSec;
	AccelIn.JamDistanceCm            = MinFollowingDistanceCm;
	AccelIn.ReactionDelaySec         = IDMReactionDelaySec;
	AccelIn.SmoothingTauSec          = IDMSmoothingTauSec;
	AccelIn.AEBThresholdSec          = AEBTimeToCollisionThresholdSec;
	AccelIn.MaxBrakeDecelCmPerSec2   = MaxBrakeDecelCmPerSec2;

	const FAccelerationOutput AccelOut = AccelModel.Compute(AccelIn);
	float ThrottleInput = AccelOut.ThrottleInput;
	float BrakeInput    = AccelOut.BrakeInput;

	if (AccelOut.bAEBActive)
	{
		UE_LOG(LogAAATraffic, Warning,
			TEXT("AEB ACTIVE: Pawn='%s' Gap=%.0fcm -- EMERGENCY BRAKE"),
			GetPawn() ? *GetPawn()->GetName() : TEXT("NULL"),
			LeaderDist);
	}

	SetBrakeLights(AccelOut.bBraking);

#if defined(ENABLE_DRAW_DEBUG) && ENABLE_DRAW_DEBUG
	DbgLeaderDist = AccelOut.DelayedLeaderDist;
	DbgLeaderSpeed = AccelOut.DelayedLeaderSpeed;
#endif

	// Wake guard: keep the GT-level vehicle movement awake so throttle input
	// takes effect. Throttled to once per second to avoid per-tick energy
	// injection that was causing flying-car physics explosions.
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

	// Reactive collision handler -- if we physically collided, override to
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
	// This is a last-resort safety net -- normal driving never reaches this.
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
					TEXT("SAFETY VELOCITY-CLAMP: Pawn='%s' had runaway speed %.0f cm/s (max %.0f) -- zeroed"),
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
	if (LaneChangeCoord_.State != ELaneChangeState::None)
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

	// -- Polyline-following sweep -----------------------------
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

	// -- Straight-line extension past polyline end ------------
	// If we exhausted the polyline but still have detection budget,
	// fire one more sweep from the last polyline point in the direction
	// of the last segment.  Without this, vehicles just past the lane
	// end (e.g., entering a junction) are invisible to following vehicles
	// -- the leading cause of rear-end collisions at junction entries.
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

