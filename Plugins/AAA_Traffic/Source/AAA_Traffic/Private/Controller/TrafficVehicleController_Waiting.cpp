// TrafficVehicleController_Waiting.cpp -- Intersection waiting and right-of-way logic.
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

bool ATrafficVehicleController::TickIntersectionWaiting(
float DeltaSeconds, float CurrentSpeed,
const FVector& VehicleLocation, const FVector& VehicleForward,
int32 ClosestIndex, float RemainingDist,
UChaosWheeledVehicleMovementComponent* VehicleMovement)
{
if (!JnctState.bWaiting)
{
return false;
}
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
				// Fall through -- vehicle will resume normal lane following.
			}
			else if (TrafficSub)
			{
				// Accumulate wait time for timeout detection.
				JnctState.WaitElapsed += DeltaSeconds;

				// -- Stop-sign elapsed timer (every frame, not just on retry) --
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
								// Register in FIFO queue -- first to complete mandatory stop = first to go.
								TrafficSub->RecordStopSignArrival(JnctState.JunctionId, this);
								UE_LOG(LogAAATraffic, Warning,
									TEXT("JNCT STOPSIGN-WAIT-DONE: Pawn='%s' JunctionId=%d -- "
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
						const ETurnSignalState RoRTurn = GetSelectedTurnDirection();
						if (RoRTurn == ETurnSignalState::Right
							&& !TrafficSub->HasConflictingApproach(JnctState.JunctionId, this, JnctState.CanonicalMovementId))
						{
							bSignalAllows = true;
							UE_LOG(LogAAATraffic, Log,
								TEXT("JNCT RIGHT-ON-RED: Pawn='%s' JunctionId=%d -- "
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
							// FIFO ordering: another vehicle arrived first -- must wait our turn.
							bSignalAllows = false;
						}
					}
					// Yield mode: also check approaching cross-traffic gap acceptance.
					if (bSignalAllows && JunctionMode == EJunctionControlMode::Yield)
					{
						if (TrafficSub->HasApproachingCrossTraffic(
								JnctState.JunctionId, this, JnctState.CanonicalMovementId))
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
						const ETurnSignalState RetryTurnDir = GetSelectedTurnDirection();
						if (RetryTurnDir == ETurnSignalState::Left
							&& !(Signal && Signal->IsLaneProtectedGreen(JnctState.JunctionLane))
							&& TrafficSub->HasConflictingApproach(JnctState.JunctionId, this, JnctState.CanonicalMovementId))
						{
							bLeftTurnYieldRetry = true;
						}
					}

					if (bSignalAllows && !bLeftTurnYieldRetry && TrafficSub->TryOccupyJunction(JnctState.JunctionId, this, JnctState.CanonicalMovementId))
					{
						JnctState.bOwnsJunctionOccupancy = true;
						JnctState.BeginTraversing();
						// Remove from stop-sign FIFO queue now that occupancy is granted.
						TrafficSub->RemoveStopSignArrival(JnctState.JunctionId, this);

						// Note: old overshoot-teleport removed. The old-lane-tail
						// prepend in CheckLaneTransition now gives the new lane
						// continuous polyline coverage from the vehicle's actual
						// position, making teleport unnecessary and visually jarring.

						JnctState.bHasEntryPos = false;
						UE_LOG(LogAAATraffic, Warning,
							TEXT("JNCT RETRY-GRANTED: Pawn='%s' JunctionId=%d -- CLEARED to proceed (was waiting, elapsed %.1fs)"),
							GetPawn() ? *GetPawn()->GetName() : TEXT("NULL"),
							JnctState.JunctionId,
							JnctState.WaitElapsed);
						// Fall through to lane transition gate below.
					}
				}

				// Timeout: retry after stale-occupant eviction, but never force
				// entry through a live conflict set.
				// Gated by RetryTimer to avoid per-frame log spam; the normal
				// retry gate above already resets RetryTimer to 0.25f.
				if (JnctState.bWaiting && MaxIntersectionWaitTimeSec > 0.0f && JnctState.WaitElapsed >= MaxIntersectionWaitTimeSec)
				{
					// Extended-timeout despawn: if stuck for 15s past MaxWait, give up.
					constexpr float ExtendedTimeoutMargin = 15.0f;
					if (JnctState.WaitElapsed >= MaxIntersectionWaitTimeSec + ExtendedTimeoutMargin)
					{
						if (TrafficSub && !bPendingRecoveryDespawn)
						{
							bPendingRecoveryDespawn = true;
							UE_LOG(LogAAATraffic, Warning,
								TEXT("JNCT TIMEOUT-DESPAWN: Pawn='%s' JunctionId=%d -- "
									 "waited %.1fs (max %.1fs + %.0fs margin). Despawning."),
								GetPawn() ? *GetPawn()->GetName() : TEXT("NULL"),
								JnctState.JunctionId,
								JnctState.WaitElapsed,
								MaxIntersectionWaitTimeSec,
								ExtendedTimeoutMargin);
							TrafficSub->RequestDespawn(this,
								FString::Printf(TEXT("junction wait timeout %.1fs"),
									JnctState.WaitElapsed));
						}
						return true;
					}

					// Throttle timeout logs to ~1/sec.
					// RetryTimer fires at 0.25f cadence (4/sec); only log every 4th.
					static constexpr int32 TimeoutLogInterval = 4;
					const bool bShouldLogTimeout = (++JnctState.TimeoutRetryCount % TimeoutLogInterval) == 1;

					if (bShouldLogTimeout)
					{
						UE_LOG(LogAAATraffic, Warning,
							TEXT("JNCT TIMEOUT-RETRY: Pawn='%s' JunctionId=%d -- "
								 "waited %.1fs (max %.1fs). Retrying TryOccupy after stale-occupant eviction."),
							GetPawn() ? *GetPawn()->GetName() : TEXT("NULL"),
							JnctState.JunctionId,
							JnctState.WaitElapsed,
							MaxIntersectionWaitTimeSec);
					}

					if (TrafficSub)
					{
						if (!TrafficSub->TryOccupyJunction(JnctState.JunctionId,
							this, JnctState.CanonicalMovementId))
						{
							JnctState.RetryTimer = 0.25f;
							return true;
						}
						JnctState.bOwnsJunctionOccupancy = true;
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
				// when available -- Euclidean is too short on curved roads.
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
							// Leader is closer than the intersection entry -- stop behind leader.
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
						DistToEntry = 0.0f; // Overshot -- treat as zero.
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
						// brake input -- causing consistent overshoot.
						// Euclidean updates every frame and at junction-approach
						// distances (< ~7000 cm) the arc vs Euclidean gap is
						// negligible, always erring on the safe side (shorter
						// distance -> more conservative braking).
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
					// At or past entry -- full stop.
					WaitBrake = 1.0f;
					VehicleMovement->SetThrottleInput(0.0f);
					VehicleMovement->SetSteeringInput(0.0f);
					VehicleMovement->SetBrakeInput(1.0f);
				}
				else if (FMath::Abs(CurrentSpeed) > DesiredStopSpeed)
				{
					// Over the decel envelope -- brake proportionally.
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
					// Under envelope -- coast if still rolling, hold brake if stopped.
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
				return true;
			}

	return false;
}

