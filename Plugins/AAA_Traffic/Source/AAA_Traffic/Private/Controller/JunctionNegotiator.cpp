// JunctionNegotiator.cpp — Junction approach, detection, occupy, release, traversal.
#include "JunctionNegotiator.h"
#include "TrafficVehicleController.h"
#include "TrafficSubsystem.h"
#include "TrafficSignalController.h"
#include "TrafficLog.h"
#include "GameFramework/Pawn.h"
#include "Engine/World.h"

extern int32 GTrafficJunctionDiagnostics;

// ---------------------------------------------------------------------------
// TickApproach — junction approach scan + braking envelope + pre-positioning.
// ---------------------------------------------------------------------------

bool FJunctionNegotiator::TickApproach(int32 ClosestIndex, float AbsSpeed, float CurrentSpeed)
{
	bool bApproachBraking = false;

	if (!Owner->bAtDeadEnd && Owner->CachedProvider && Owner->JnctState.Phase <= EJunctionPhase::Approaching)
	{
		const float RemainingOnCurrent = Owner->GetRemainingDistance(ClosestIndex);
		const ITrafficRoadProvider::FJunctionScanResult ScanResult =
			Owner->CachedProvider->GetDistanceToNextJunction(
				Owner->CurrentLane,
				RemainingOnCurrent,
				Owner->JunctionScanMaxDistanceCm,
				Owner->MaxJunctionScanHops);

		if (GTrafficJunctionDiagnostics >= 2)
		{
			UE_LOG(LogAAATraffic, Warning,
				TEXT("JDIAG SCAN: Pawn='%s' Lane=%d Remaining=%.1f ScanValid=%s "
					 "JunctionId=%d Dist=%.1f JunctionLane=%d ApproachLane=%d"),
				Owner->GetPawn() ? *Owner->GetPawn()->GetName() : TEXT("NULL"),
				Owner->CurrentLane.HandleId,
				RemainingOnCurrent,
				ScanResult.IsValid() ? TEXT("YES") : TEXT("NO"),
				ScanResult.JunctionId,
				ScanResult.DistanceCm,
				ScanResult.JunctionLane.HandleId,
				ScanResult.ApproachLane.HandleId);
		}

		if (ScanResult.IsValid())
		{
			if (Owner->JnctState.Phase == EJunctionPhase::Idle)
			{
				Owner->JnctState.BeginApproach(ScanResult.JunctionId);
			}
			Owner->JnctState.ApproachDistanceCm = ScanResult.DistanceCm;
			Owner->JnctState.ApproachJunctionLane = ScanResult.JunctionLane;

			const float SafeDist = FMath::Max(Owner->JnctState.ApproachDistanceCm - Owner->ApproachSafetyMarginCm - Owner->VehicleFrontExtent, 0.0f);
			Owner->JnctState.ApproachSpeedLimitCmPerSec = FMath::Sqrt(
				FMath::Max(2.0f * Owner->ApproachDecelCmPerSec2 * SafeDist, 0.0f));

			const float AbsSpeedNow = FMath::Abs(CurrentSpeed);
			bApproachBraking = (AbsSpeedNow > Owner->JnctState.ApproachSpeedLimitCmPerSec + 50.0f) ||
									   (Owner->JnctState.ApproachDistanceCm < Owner->LookAheadDistance * 3.0f);

#if defined(ENABLE_DRAW_DEBUG) && ENABLE_DRAW_DEBUG
			Owner->DbgApproachJunctionDist = Owner->JnctState.ApproachDistanceCm;
			Owner->DbgApproachSpeedLimit = Owner->JnctState.ApproachSpeedLimitCmPerSec;
			Owner->DbgApproachJunctionId = Owner->JnctState.JunctionId;
#endif

			// --- Lane pre-positioning for upcoming turn ---
			const float PrePosStopDist = (AbsSpeed * AbsSpeed) / (2.0f * FMath::Max(Owner->ApproachDecelCmPerSec2, 1.0f));
			const float DynamicPrePositionDist = PrePosStopDist + Owner->LaneChangeCoord_.Distance * 2.0f + Owner->ApproachSafetyMarginCm;
			if (!Owner->LaneChangeCoord_.bNavigationalLaneChange
				&& Owner->JnctState.ApproachDistanceCm > DynamicPrePositionDist
				&& Owner->LaneChangeCoord_.State == ELaneChangeState::None
				&& Owner->JnctState.ApproachJunctionLane.IsValid())
			{
				TArray<FTrafficLaneHandle> FallbackExits;
				if (Owner->JnctState.JunctionId > 0)
				{
					TArray<FTrafficLaneHandle> AllJL = Owner->CachedProvider->GetLanesForJunction(Owner->JnctState.JunctionId);
					for (const FTrafficLaneHandle& JL : AllJL)
					{
						for (const FTrafficLaneHandle& E : Owner->CachedProvider->GetConnectedLanes(JL))
						{
							if (Owner->CachedProvider->GetJunctionForLane(E) == 0)
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
					FallbackExits = Owner->CachedProvider->GetConnectedLanes(Owner->JnctState.ApproachJunctionLane);
				}

				const FTrafficLaneHandle ChosenExit = Owner->PickSurveyedExit(Owner->CurrentLane, FallbackExits);

				if (ChosenExit.IsValid())
				{
					const ETurnSignalState TurnDir = Owner->ComputeTurnDirection(Owner->CurrentLane, ChosenExit);

					if (TurnDir != ETurnSignalState::Off)
					{
						Owner->SetTurnSignal(TurnDir);
					}

					FTrafficLaneHandle TargetSideLane = FTrafficLaneHandle();
					const FVector PrePosRefDir = Owner->CachedProvider->GetLaneDirection(Owner->CurrentLane);
					if (TurnDir == ETurnSignalState::Right)
					{
						FTrafficLaneHandle Walk = Owner->CachedProvider->GetAdjacentLane(Owner->CurrentLane, ETrafficLaneSide::Right);
						for (int32 Safety = 0; Safety < 8 && Walk.IsValid(); ++Safety)
						{
							const FVector WalkDir = Owner->CachedProvider->GetLaneDirection(Walk);
							if (FVector::DotProduct(PrePosRefDir, WalkDir) < 0.5f) { break; }
							TargetSideLane = Walk;
							Walk = Owner->CachedProvider->GetAdjacentLane(Walk, ETrafficLaneSide::Right);
						}
					}
					else if (TurnDir == ETurnSignalState::Left)
					{
						FTrafficLaneHandle Walk = Owner->CachedProvider->GetAdjacentLane(Owner->CurrentLane, ETrafficLaneSide::Left);
						for (int32 Safety = 0; Safety < 8 && Walk.IsValid(); ++Safety)
						{
							const FVector WalkDir = Owner->CachedProvider->GetLaneDirection(Walk);
							if (FVector::DotProduct(PrePosRefDir, WalkDir) < 0.5f) { break; }
							TargetSideLane = Walk;
							Walk = Owner->CachedProvider->GetAdjacentLane(Walk, ETrafficLaneSide::Left);
						}
					}

					if (TargetSideLane.IsValid() && TargetSideLane != Owner->CurrentLane)
					{
						Owner->LaneChangeCoord_.NavigationalTargetLane = TargetSideLane;
						Owner->LaneChangeCoord_.bNavigationalLaneChange = true;
						UE_LOG(LogAAATraffic, Log,
							TEXT("TrafficVehicleController: Pre-positioning %s from lane %d → lane %d "
								 "(turn=%s, junction dist=%.0f cm)."),
							Owner->GetPawn() ? *Owner->GetPawn()->GetName() : TEXT("NULL"),
							Owner->CurrentLane.HandleId, TargetSideLane.HandleId,
							TurnDir == ETurnSignalState::Left ? TEXT("LEFT") : TEXT("RIGHT"),
							Owner->JnctState.ApproachDistanceCm);
					}
				}
			}
		}
		else
		{
			if (GTrafficJunctionDiagnostics >= 1)
			{
				UE_LOG(LogAAATraffic, Warning,
					TEXT("JDIAG SCAN-INVALID: Pawn='%s' Lane=%d — GetDistanceToNextJunction "
						 "returned INVALID (no junction downstream). Vehicle will NOT slow for any intersection."),
					Owner->GetPawn() ? *Owner->GetPawn()->GetName() : TEXT("NULL"),
					Owner->CurrentLane.HandleId);
			}
			bApproachBraking = false;
			if (Owner->JnctState.Phase == EJunctionPhase::Approaching)
			{
				Owner->JnctState.Reset();
			}
#if defined(ENABLE_DRAW_DEBUG) && ENABLE_DRAW_DEBUG
			Owner->DbgApproachJunctionDist = -1.0f;
			Owner->DbgApproachSpeedLimit = 0.0f;
			Owner->DbgApproachJunctionId = 0;
#endif
		}
	}
	else if (Owner->JnctState.Phase >= EJunctionPhase::Waiting)
	{
		bApproachBraking = false;
		if (GTrafficJunctionDiagnostics >= 2)
		{
			UE_LOG(LogAAATraffic, Warning,
				TEXT("JDIAG SCAN-SKIP-HANDLING: Pawn='%s' Lane=%d — approach scan disabled "
					 "(already handling intersection: JunctionId=%d bWaiting=%s)"),
				Owner->GetPawn() ? *Owner->GetPawn()->GetName() : TEXT("NULL"),
				Owner->CurrentLane.HandleId,
				Owner->JnctState.JunctionId,
				Owner->JnctState.bWaiting ? TEXT("YES") : TEXT("NO"));
		}
#if defined(ENABLE_DRAW_DEBUG) && ENABLE_DRAW_DEBUG
		Owner->DbgApproachJunctionDist = -1.0f;
		Owner->DbgApproachSpeedLimit = 0.0f;
		Owner->DbgApproachJunctionId = 0;
#endif
	}
	else
	{
		if (GTrafficJunctionDiagnostics >= 1)
		{
			UE_LOG(LogAAATraffic, Warning,
				TEXT("JDIAG SCAN-BLOCKED: Pawn='%s' Lane=%d — approach scan guard FAILED. "
					 "bAtDeadEnd=%s CachedProvider=%s bWaiting=%s JnctState.JunctionId=%d. "
					 "THIS VEHICLE CANNOT SEE JUNCTIONS."),
				Owner->GetPawn() ? *Owner->GetPawn()->GetName() : TEXT("NULL"),
				Owner->CurrentLane.HandleId,
				Owner->bAtDeadEnd ? TEXT("YES") : TEXT("NO"),
				Owner->CachedProvider ? TEXT("VALID") : TEXT("NULL!!!"),
				Owner->JnctState.bWaiting ? TEXT("YES") : TEXT("NO"),
				Owner->JnctState.JunctionId);
		}
	}

	return bApproachBraking;
}

// ---------------------------------------------------------------------------
// TickDetectAndOccupy — junction detection at lane-end + right-of-way.
// ---------------------------------------------------------------------------

void FJunctionNegotiator::TickDetectAndOccupy(float RemainingDist, float TransitionThreshold,
	float AbsSpeed, float CurrentSpeed, const FVector& VehicleLocation)
{
	if (Owner->JnctState.Phase >= EJunctionPhase::Waiting || !Owner->CachedProvider)
	{
		return;
	}

	int32 DetectedJunctionId = Owner->CachedProvider->GetJunctionForLane(Owner->CurrentLane);
	FTrafficLaneHandle DetectedJunctionLane = Owner->CurrentLane;

	if (GTrafficJunctionDiagnostics >= 1)
	{
		UE_LOG(LogAAATraffic, Log,
			TEXT("JNCT DETECT: Pawn='%s' CurrentLane=%d GetJunctionForLane(current)=%d "
				 "RemainingDist=%.1f TransitionThreshold=%.1f Speed=%.1f"),
			Owner->GetPawn() ? *Owner->GetPawn()->GetName() : TEXT("NULL"),
			Owner->CurrentLane.HandleId, DetectedJunctionId,
			RemainingDist, TransitionThreshold, CurrentSpeed);
	}

	// --- Look-ahead: peek at next lanes for junction presence ---
	if (DetectedJunctionId == 0)
	{
		TArray<FTrafficLaneHandle> NextLanes = Owner->CachedProvider->GetConnectedLanes(Owner->CurrentLane);
		for (const FTrafficLaneHandle& NextLane : NextLanes)
		{
			const int32 NextJId = Owner->CachedProvider->GetJunctionForLane(NextLane);
			if (GTrafficJunctionDiagnostics >= 1)
			{
				UE_LOG(LogAAATraffic, Log,
					TEXT("JNCT LOOKAHEAD: Pawn='%s' CurrentLane=%d NextLane=%d "
						 "GetJunctionForLane(next)=%d"),
					Owner->GetPawn() ? *Owner->GetPawn()->GetName() : TEXT("NULL"),
					Owner->CurrentLane.HandleId, NextLane.HandleId, NextJId);
			}
			if (NextJId != 0)
			{
				DetectedJunctionId = NextJId;
				DetectedJunctionLane = NextLane;
				break;
			}
		}
	}

	// --- Extended look-ahead (precomputed map fallback) ---
	if (DetectedJunctionId == 0 && Owner->JnctState.Phase == EJunctionPhase::Approaching
		&& Owner->JnctState.ApproachDistanceCm > 0.0f
		&& Owner->JnctState.ApproachDistanceCm < TransitionThreshold)
	{
		DetectedJunctionId = Owner->JnctState.JunctionId;
		DetectedJunctionLane = Owner->JnctState.ApproachJunctionLane;
		if (GTrafficJunctionDiagnostics >= 1)
		{
			UE_LOG(LogAAATraffic, Log,
				TEXT("JNCT PRECOMPUTED-HIT: Pawn='%s' CurrentLane=%d — 1-hop LOOKAHEAD "
					 "missed, using precomputed map: JunctionId=%d Dist=%.1f JnctLane=%d"),
				Owner->GetPawn() ? *Owner->GetPawn()->GetName() : TEXT("NULL"),
				Owner->CurrentLane.HandleId,
				Owner->JnctState.JunctionId,
				Owner->JnctState.ApproachDistanceCm,
				Owner->JnctState.ApproachJunctionLane.HandleId);
		}
	}

	// Skip re-acquisition when we already hold this junction's occupancy.
	if (DetectedJunctionId != 0 && Owner->JnctState.IsEngaged() && DetectedJunctionId == Owner->JnctState.JunctionId)
	{
		if (GTrafficJunctionDiagnostics >= 1)
		{
			UE_LOG(LogAAATraffic, Log,
				TEXT("JNCT SKIP-REDETECT: Pawn='%s' JunctionId=%d — already holds "
					 "occupancy, skipping signal check, proceeding to CheckLaneTransition"),
				Owner->GetPawn() ? *Owner->GetPawn()->GetName() : TEXT("NULL"),
				DetectedJunctionId);
		}
	}
	else if (DetectedJunctionId != 0 && DetectedJunctionId == Owner->JnctState.LastReleasedId)
	{
		if (GTrafficJunctionDiagnostics >= 1)
		{
			UE_LOG(LogAAATraffic, Log,
				TEXT("JNCT SKIP-RELEASED: Pawn='%s' JunctionId=%d — junction was "
					 "just released on this lane, skipping re-acquisition"),
				Owner->GetPawn() ? *Owner->GetPawn()->GetName() : TEXT("NULL"),
				DetectedJunctionId);
		}
	}
	else if (DetectedJunctionId != 0)
	{
		// Abort any active lane change — intersection takes priority.
		if (Owner->LaneChangeCoord_.State != ELaneChangeState::None)
		{
			if (GTrafficJunctionDiagnostics >= 1)
			{
				UE_LOG(LogAAATraffic, Log,
					TEXT("JNCT LANE-CHANGE-ABORT: Pawn='%s' JunctionId=%d — "
						 "aborting lane change (state=%d) to handle intersection"),
					Owner->GetPawn() ? *Owner->GetPawn()->GetName() : TEXT("NULL"),
					DetectedJunctionId, static_cast<int32>(Owner->LaneChangeCoord_.State));
			}
			Owner->LaneChangeCoord_.Reset();
		}

		if (Owner->JnctState.Phase == EJunctionPhase::Idle)
		{
			Owner->JnctState.BeginApproach(DetectedJunctionId);
		}
		Owner->JnctState.JunctionLane = DetectedJunctionLane;

		// Resolve exact intersection entry point for brake targeting.
		if (!Owner->JnctState.bHasEntryPos)
		{
			FVector EntryPt;
			if (Owner->CachedProvider->GetIntersectionEntryPoint(Owner->CurrentLane, EntryPt))
			{
				Owner->JnctState.EntryWorldPos = EntryPt;
				Owner->JnctState.bHasEntryPos = true;
				UE_LOG(LogAAATraffic, Warning,
					TEXT("JNCT ENTRYPOINT: Pawn='%s' Lane=%d EntryPt=(%.1f,%.1f,%.1f) Source=ExactMask"),
					Owner->GetPawn() ? *Owner->GetPawn()->GetName() : TEXT("NULL"),
					Owner->CurrentLane.HandleId, EntryPt.X, EntryPt.Y, EntryPt.Z);
			}
			else
			{
				FVector Centroid;
				if (DetectedJunctionId != 0
					&& Owner->CachedProvider->GetJunctionCentroid(DetectedJunctionId, Centroid)
					&& Owner->LanePoints.Num() >= 2)
				{
					const FVector& P0 = Owner->LanePoints[Owner->LanePoints.Num() - 2];
					const FVector& P1 = Owner->LanePoints.Last();
					const FVector SegDir = (P1 - P0).GetSafeNormal();
					const float Proj = FVector::DotProduct(Centroid - P0, SegDir);
					const float SegLen = FVector::Dist(P0, P1);
					const float ClampedProj = FMath::Clamp(Proj, 0.0f, SegLen);
					Owner->JnctState.EntryWorldPos = P0 + SegDir * ClampedProj;
					Owner->JnctState.bHasEntryPos = true;
					UE_LOG(LogAAATraffic, Warning,
						TEXT("JNCT ENTRYPOINT: Pawn='%s' Lane=%d EntryPt=(%.1f,%.1f,%.1f) Source=CentroidProjection"),
						Owner->GetPawn() ? *Owner->GetPawn()->GetName() : TEXT("NULL"),
						Owner->CurrentLane.HandleId, Owner->JnctState.EntryWorldPos.X, Owner->JnctState.EntryWorldPos.Y, Owner->JnctState.EntryWorldPos.Z);
				}
				else
				{
					Owner->JnctState.EntryWorldPos = Owner->LanePoints.Last();
					Owner->JnctState.bHasEntryPos = true;
					UE_LOG(LogAAATraffic, Warning,
						TEXT("JNCT ENTRYPOINT: Pawn='%s' Lane=%d EntryPt=(%.1f,%.1f,%.1f) Source=FallbackLast"),
						Owner->GetPawn() ? *Owner->GetPawn()->GetName() : TEXT("NULL"),
						Owner->CurrentLane.HandleId, Owner->LanePoints.Last().X, Owner->LanePoints.Last().Y, Owner->LanePoints.Last().Z);
				}
			}
		}

		UWorld* World = Owner->GetWorld();
		UTrafficSubsystem* TrafficSub = World ? World->GetSubsystem<UTrafficSubsystem>() : nullptr;
		if (TrafficSub)
		{
			ATrafficSignalController* Signal = TrafficSub->GetSignalForJunction(DetectedJunctionId);
			bool bSignalAllows = true;
			if (Signal)
			{
				bSignalAllows = Signal->IsLaneGreen(Owner->JnctState.JunctionLane);

				if (!bSignalAllows
					&& Signal->GetControlMode() == EJunctionControlMode::Signal
					&& Signal->GetCurrentPhase() == ETrafficSignalPhase::Yellow)
				{
					const float StopDist = (AbsSpeed * AbsSpeed)
						/ (2.0f * FMath::Max(Owner->ApproachDecelCmPerSec2, 1.0f));
					const float DistToJunction = Owner->JnctState.bHasEntryPos
						? FVector::Dist(VehicleLocation, Owner->JnctState.EntryWorldPos)
						: RemainingDist;
					if (StopDist > DistToJunction)
					{
						bSignalAllows = true;
						UE_LOG(LogAAATraffic, Log,
							TEXT("SIGNAL YELLOW-COMMIT: Pawn='%s' JunctionId=%d "
								 "StopDist=%.0f > DistToJunction=%.0f — committing through Yellow"),
							Owner->GetPawn() ? *Owner->GetPawn()->GetName() : TEXT("NULL"),
							DetectedJunctionId, StopDist, DistToJunction);
					}
				}
			}

			// Compute exit lane.
			Owner->JnctState.FromLane = Owner->CurrentLane;
			Owner->JnctState.ToLane = FTrafficLaneHandle();
			{
				TArray<FTrafficLaneHandle> FallbackExits;
				TArray<FTrafficLaneHandle> AllJunctionLanes = Owner->CachedProvider->GetLanesForJunction(DetectedJunctionId);
				for (const FTrafficLaneHandle& JL : AllJunctionLanes)
				{
					TArray<FTrafficLaneHandle> Exits = Owner->CachedProvider->GetConnectedLanes(JL);
					for (const FTrafficLaneHandle& E : Exits)
					{
						if (Owner->CachedProvider->GetJunctionForLane(E) == 0)
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
					FallbackExits = Owner->CachedProvider->GetConnectedLanes(DetectedJunctionLane);
				}

				Owner->JnctState.ToLane = Owner->PickSurveyedExit(Owner->CurrentLane, FallbackExits);

				if (Owner->JnctState.ToLane.IsValid())
				{
					UE_LOG(LogAAATraffic, Warning,
						TEXT("JNCT EXIT-PICKED: Pawn='%s' JunctionId=%d Picked=%d (via surveyor)"),
						Owner->GetPawn() ? *Owner->GetPawn()->GetName() : TEXT("NULL"),
						DetectedJunctionId, Owner->JnctState.ToLane.HandleId);
				}
			}

			// ── Turn-aware exit lane matching ──
			if (Owner->CachedProvider && Owner->JnctState.ToLane.IsValid())
			{
				const ETurnSignalState TurnDir = Owner->ComputeTurnDirection(Owner->JnctState.FromLane, Owner->JnctState.ToLane);
				if (TurnDir == ETurnSignalState::Left || TurnDir == ETurnSignalState::Right)
				{
					const FVector ExitRefDir = Owner->CachedProvider->GetLaneDirection(Owner->JnctState.ToLane);

					FTrafficLaneHandle RightmostExit = Owner->JnctState.ToLane;
					{
						FTrafficLaneHandle Walk = Owner->CachedProvider->GetAdjacentLane(Owner->JnctState.ToLane, ETrafficLaneSide::Right);
						for (int32 S = 0; S < 8 && Walk.IsValid(); ++S)
						{
							const FVector WalkDir = Owner->CachedProvider->GetLaneDirection(Walk);
							if (FVector::DotProduct(ExitRefDir, WalkDir) < 0.5f) { break; }
							RightmostExit = Walk;
							Walk = Owner->CachedProvider->GetAdjacentLane(Walk, ETrafficLaneSide::Right);
						}
					}
					FTrafficLaneHandle LeftmostExit = Owner->JnctState.ToLane;
					{
						FTrafficLaneHandle Walk = Owner->CachedProvider->GetAdjacentLane(Owner->JnctState.ToLane, ETrafficLaneSide::Left);
						for (int32 S = 0; S < 8 && Walk.IsValid(); ++S)
						{
							const FVector WalkDir = Owner->CachedProvider->GetLaneDirection(Walk);
							if (FVector::DotProduct(ExitRefDir, WalkDir) < 0.5f) { break; }
							LeftmostExit = Walk;
							Walk = Owner->CachedProvider->GetAdjacentLane(Walk, ETrafficLaneSide::Left);
						}
					}

					const FTrafficLaneHandle TargetLane = (TurnDir == ETurnSignalState::Left)
						? LeftmostExit
						: RightmostExit;
					if (TargetLane.HandleId != Owner->JnctState.ToLane.HandleId)
					{
						UE_LOG(LogAAATraffic, Log,
							TEXT("JNCT TURN-LANE-MATCH: Pawn='%s' Turn=%s "
								 "Exit %d -> %d (matched to %s lane)"),
							Owner->GetPawn() ? *Owner->GetPawn()->GetName() : TEXT("NULL"),
							TurnDir == ETurnSignalState::Left ? TEXT("LEFT") : TEXT("RIGHT"),
							Owner->JnctState.ToLane.HandleId, TargetLane.HandleId,
							TurnDir == ETurnSignalState::Left ? TEXT("innermost") : TEXT("outermost"));
						Owner->JnctState.ToLane = TargetLane;
					}
				}
			}

			Owner->SetTurnSignal(Owner->ComputeTurnDirection(Owner->JnctState.FromLane, Owner->JnctState.ToLane));

			UE_LOG(LogAAATraffic, Warning,
				TEXT("JNCT OCCUPY-ATTEMPT: Pawn='%s' JunctionId=%d HasSignal=%s "
					 "SignalAllowsGreen=%s DesiredNext=%d ExitLane=%d ControlMode=%d — about to TryOccupyJunction"),
				Owner->GetPawn() ? *Owner->GetPawn()->GetName() : TEXT("NULL"),
				DetectedJunctionId,
				Signal ? TEXT("YES") : TEXT("NO"),
				bSignalAllows ? TEXT("YES") : TEXT("NO"),
				DetectedJunctionLane.HandleId,
				Owner->JnctState.ToLane.HandleId,
				Signal ? static_cast<int32>(Signal->GetControlMode()) : -1);

			const EJunctionControlMode JunctionMode = Signal
				? Signal->GetControlMode()
				: EJunctionControlMode::Yield;

			if (JunctionMode == EJunctionControlMode::Yield)
			{
				bool bMustYield = false;
				{
					const ETurnSignalState YieldTurnDir = Owner->ComputeTurnDirection(Owner->JnctState.FromLane, Owner->JnctState.ToLane);
					if (YieldTurnDir == ETurnSignalState::Left
						&& TrafficSub->HasConflictingApproach(DetectedJunctionId, Owner, Owner->JnctState.FromLane, Owner->JnctState.ToLane))
					{
						bMustYield = true;
					}
				}
				if (!bMustYield)
				{
					bMustYield = TrafficSub->HasApproachingCrossTraffic(
						DetectedJunctionId, Owner, Owner->JnctState.FromLane, Owner->JnctState.ToLane);
				}

				if (!bMustYield && TrafficSub->TryOccupyJunction(DetectedJunctionId, Owner, Owner->JnctState.FromLane, Owner->JnctState.ToLane))
				{
					Owner->JnctState.BeginTraversing();
					UE_LOG(LogAAATraffic, Warning,
						TEXT("JNCT YIELD-PROCEED: Pawn='%s' JunctionId=%d — clear, proceeding without stop"),
						Owner->GetPawn() ? *Owner->GetPawn()->GetName() : TEXT("NULL"),
						DetectedJunctionId);
				}
				else
				{
					Owner->JnctState.BeginWaiting();
					Owner->JnctState.bStopSignWaitComplete = true;
					Owner->JnctState.StopSignStopElapsed = 0.0f;
					UE_LOG(LogAAATraffic, Warning,
						TEXT("JNCT YIELD-WAIT: Pawn='%s' JunctionId=%d — %s, waiting for clearance"),
						Owner->GetPawn() ? *Owner->GetPawn()->GetName() : TEXT("NULL"),
						DetectedJunctionId,
						bMustYield ? TEXT("yield/cross-traffic") : TEXT("occupied"));
				}
			}
			else if (JunctionMode == EJunctionControlMode::StopSign || JunctionMode == EJunctionControlMode::FlashingRed)
			{
				Owner->JnctState.BeginWaiting();
				Owner->JnctState.bStopSignWaitComplete = false;
				Owner->JnctState.StopSignStopElapsed = 0.0f;
				UE_LOG(LogAAATraffic, Warning,
					TEXT("JNCT STOPSIGN-WAIT: Pawn='%s' JunctionId=%d — must stop and wait %.1fs"),
					Owner->GetPawn() ? *Owner->GetPawn()->GetName() : TEXT("NULL"),
					DetectedJunctionId,
					Signal ? Signal->StopSignWaitTimeSec : 2.0f);
			}
			else // Signal mode
			{
			bool bLeftTurnYield = false;
			if (bSignalAllows)
			{
				const ETurnSignalState OccupyTurnDir = Owner->ComputeTurnDirection(Owner->JnctState.FromLane, Owner->JnctState.ToLane);
				if (OccupyTurnDir == ETurnSignalState::Left
					&& !(Signal && Signal->IsLaneProtectedGreen(Owner->JnctState.JunctionLane))
					&& TrafficSub->HasConflictingApproach(DetectedJunctionId, Owner, Owner->JnctState.FromLane, Owner->JnctState.ToLane))
				{
					bLeftTurnYield = true;
				}
			}

			if (bSignalAllows && !bLeftTurnYield && TrafficSub->TryOccupyJunction(DetectedJunctionId, Owner, Owner->JnctState.FromLane, Owner->JnctState.ToLane))
			{
				Owner->JnctState.BeginTraversing();
				UE_LOG(LogAAATraffic, Warning,
					TEXT("JNCT OCCUPY-GRANTED: Pawn='%s' JunctionId=%d — PROCEEDING through intersection"),
					Owner->GetPawn() ? *Owner->GetPawn()->GetName() : TEXT("NULL"),
					DetectedJunctionId);
			}
			else
			{
				Owner->JnctState.BeginWaiting();
				Owner->JnctState.bStopSignWaitComplete = true;
				Owner->JnctState.StopSignStopElapsed = 0.0f;
				UE_LOG(LogAAATraffic, Warning,
					TEXT("JNCT OCCUPY-DENIED: Pawn='%s' JunctionId=%d SignalAllows=%s "
						 "— WAITING at intersection"),
					Owner->GetPawn() ? *Owner->GetPawn()->GetName() : TEXT("NULL"),
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
			Owner->GetPawn() ? *Owner->GetPawn()->GetName() : TEXT("NULL"),
			Owner->CurrentLane.HandleId);
	}
}

// ---------------------------------------------------------------------------
// TickPostTransitionRelease — release junction occupancy after lane switch.
// ---------------------------------------------------------------------------

void FJunctionNegotiator::TickPostTransitionRelease()
{
	const int32 NewLaneJunction = Owner->CachedProvider
		? Owner->CachedProvider->GetJunctionForLane(Owner->CurrentLane) : 0;

	if (NewLaneJunction != Owner->JnctState.JunctionId
		&& Owner->JnctState.TransitionPoints.Num() == 0)
	{
		UWorld* World = Owner->GetWorld();
		UTrafficSubsystem* TrafficSub = World
			? World->GetSubsystem<UTrafficSubsystem>() : nullptr;
		if (TrafficSub)
		{
			UE_LOG(LogAAATraffic, Warning,
				TEXT("JNCT RELEASE: Pawn='%s' RELEASING junction %d — new lane %d "
					 "is in junction %d (exited)"),
				Owner->GetPawn() ? *Owner->GetPawn()->GetName() : TEXT("NULL"),
				Owner->JnctState.JunctionId, Owner->CurrentLane.HandleId, NewLaneJunction);
			TrafficSub->ReleaseJunction(Owner->JnctState.JunctionId, Owner);
		}
		Owner->JnctState.Release();
	}
	else if (NewLaneJunction != Owner->JnctState.JunctionId)
	{
		UE_LOG(LogAAATraffic, Warning,
			TEXT("JNCT RELEASE-DEFERRED: Pawn='%s' junction %d — "
				 "new lane %d is in junction %d but curve active (%d pts), "
				 "deferring release to PATH2"),
			Owner->GetPawn() ? *Owner->GetPawn()->GetName() : TEXT("NULL"),
			Owner->JnctState.JunctionId, Owner->CurrentLane.HandleId,
			NewLaneJunction, Owner->JnctState.TransitionPoints.Num());
	}
	else
	{
		if (GTrafficJunctionDiagnostics >= 1)
		{
			UE_LOG(LogAAATraffic, Log,
				TEXT("JNCT HOLD: Pawn='%s' junction %d — new lane %d still in "
					 "same junction, keeping occupancy"),
				Owner->GetPawn() ? *Owner->GetPawn()->GetName() : TEXT("NULL"),
				Owner->JnctState.JunctionId, Owner->CurrentLane.HandleId);
		}
	}
}

// ---------------------------------------------------------------------------
// TickTraverse — walk along junction transition curve, release on completion.
// ---------------------------------------------------------------------------

bool FJunctionNegotiator::TickTraverse(const FVector& VehicleLocation, int32 ClosestIndex,
	FVector& OutTargetPoint)
{
	if (Owner->JnctState.TransitionPoints.Num() == 0)
	{
		return false;
	}

	// Advance along junction points based on proximity or forward progress.
	while (Owner->JnctState.TransitionIndex < Owner->JnctState.TransitionPoints.Num() - 1)
	{
		const float DistToCurrentSq = FVector::DistSquared(VehicleLocation, Owner->JnctState.TransitionPoints[Owner->JnctState.TransitionIndex]);
		const float DistToNextSq = FVector::DistSquared(VehicleLocation, Owner->JnctState.TransitionPoints[Owner->JnctState.TransitionIndex + 1]);
		if (DistToCurrentSq < 10000.0f || DistToNextSq < DistToCurrentSq)
		{
			++Owner->JnctState.TransitionIndex;
		}
		else
		{
			break;
		}
	}

	if (Owner->JnctState.TransitionIndex >= Owner->JnctState.TransitionPoints.Num() - 1)
	{
		// Finished junction transition — release junction occupancy and proceed.
		Owner->JnctState.TransitionPoints.Empty();
		Owner->JnctState.TransitionIndex = 0;

		Owner->TurnSignalOffDelayRemaining = 1.5f;
		Owner->LaneChangeCoord_.ClearNav();

		UE_LOG(LogAAATraffic, Warning,
			TEXT("JNCT RELEASE-PATH2-CHECK: Pawn='%s' JnctState.JunctionId=%d "
				 "— junction curve complete, checking release"),
			Owner->GetPawn() ? *Owner->GetPawn()->GetName() : TEXT("NULL"),
			Owner->JnctState.JunctionId);

		if (UWorld* World = Owner->GetWorld())
		{
			if (UTrafficSubsystem* TrafficSub = World->GetSubsystem<UTrafficSubsystem>())
			{
				UE_LOG(LogAAATraffic, Warning,
					TEXT("JNCT RELEASE-PATH2-FIRE: Pawn='%s' RELEASING junction %d (curve complete)"),
					Owner->GetPawn() ? *Owner->GetPawn()->GetName() : TEXT("NULL"),
					Owner->JnctState.JunctionId);
				TrafficSub->ReleaseJunction(Owner->JnctState.JunctionId, Owner);
			}
		}
		Owner->JnctState.Release();

		OutTargetPoint = Owner->GetLookAheadPoint(VehicleLocation, ClosestIndex);
		return true;
	}

	// Speed-adaptive look-ahead along junction curve.
	float JctLookAhead = Owner->LookAheadDistance;
	if (Owner->JnctState.TransitionPoints.Num() >= 3)
	{
		float TotalArc = 0.0f;
		float TotalAngle = 0.0f;
		for (int32 k = 0; k < Owner->JnctState.TransitionPoints.Num() - 1; ++k)
		{
			TotalArc += FVector::Dist(Owner->JnctState.TransitionPoints[k], Owner->JnctState.TransitionPoints[k + 1]);
		}
		if (Owner->JnctState.TransitionPoints.Num() >= 2)
		{
			const FVector StartDir = (Owner->JnctState.TransitionPoints[1] - Owner->JnctState.TransitionPoints[0]).GetSafeNormal();
			const FVector EndDir = (Owner->JnctState.TransitionPoints.Last() - Owner->JnctState.TransitionPoints[Owner->JnctState.TransitionPoints.Num() - 2]).GetSafeNormal();
			TotalAngle = FMath::Acos(FMath::Clamp(FVector::DotProduct(StartDir, EndDir), -1.0f, 1.0f));
		}
		if (TotalAngle > 0.15f)
		{
			const float EstRadius = TotalArc / FMath::Max(TotalAngle, KINDA_SMALL_NUMBER);
			JctLookAhead = FMath::Min(JctLookAhead, EstRadius * 0.5f);
			JctLookAhead = FMath::Max(JctLookAhead, Owner->MinLookAheadDistanceCm);
		}
	}

	float JctAccumDist = 0.0f;
	int32 JctIdx = Owner->JnctState.TransitionIndex;
	while (JctIdx < Owner->JnctState.TransitionPoints.Num() - 1)
	{
		const float SegDist = FVector::Dist2D(
			Owner->JnctState.TransitionPoints[JctIdx],
			Owner->JnctState.TransitionPoints[JctIdx + 1]);
		JctAccumDist += SegDist;
		if (JctAccumDist >= JctLookAhead)
		{
			const float Overshoot = JctAccumDist - JctLookAhead;
			const float Alpha = 1.0f - (Overshoot / FMath::Max(SegDist, KINDA_SMALL_NUMBER));
			OutTargetPoint = FMath::Lerp(
				Owner->JnctState.TransitionPoints[JctIdx],
				Owner->JnctState.TransitionPoints[JctIdx + 1], Alpha);
			return true;
		}
		++JctIdx;
	}

	// Extrapolate past the end of the junction curve.
	const int32 Last = Owner->JnctState.TransitionPoints.Num() - 1;
	if (Last >= 1)
	{
		const FVector Dir = (Owner->JnctState.TransitionPoints[Last] - Owner->JnctState.TransitionPoints[Last - 1]).GetSafeNormal();
		OutTargetPoint = Owner->JnctState.TransitionPoints[Last] + Dir * (JctLookAhead - JctAccumDist);
	}
	else
	{
		OutTargetPoint = Owner->JnctState.TransitionPoints[Last];
	}
	return true;
}
