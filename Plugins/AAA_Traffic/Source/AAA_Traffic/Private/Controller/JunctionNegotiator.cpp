// JunctionNegotiator.cpp — Junction approach, detection, occupy, release, traversal.
#include "JunctionNegotiator.h"
#include "TrafficVehicleController.h"
#include "TrafficSubsystem.h"
#include "TrafficSignalController.h"
#include "TrafficLog.h"
#include "GameFramework/Pawn.h"
#include "Engine/World.h"

extern int32 GTrafficJunctionDiagnostics;

namespace
{
	void CheckJunctionOccupancyContract(
		const TCHAR* Context,
		const APawn* Pawn,
		int32 JunctionId,
		EJunctionPhase Phase,
		bool bOwnsJunctionOccupancy,
		const UTrafficSubsystem* TrafficSub,
		const ATrafficVehicleController* Controller)
	{
		if (!TrafficSub || !Controller || JunctionId == 0)
		{
			return;
		}

		const bool bSubsystemHasOccupancy = TrafficSub->HasJunctionOccupancy(JunctionId, Controller);
		ensureAlwaysMsgf(
			bOwnsJunctionOccupancy == bSubsystemHasOccupancy,
			TEXT("JNCT CONTRACT VIOLATION [%s]: Pawn='%s' JunctionId=%d Phase=%d Token=%s Subsystem=%s"),
			Context,
			Pawn ? *Pawn->GetName() : TEXT("NULL"),
			JunctionId,
			(int32)Phase,
			bOwnsJunctionOccupancy ? TEXT("YES") : TEXT("NO"),
			bSubsystemHasOccupancy ? TEXT("YES") : TEXT("NO"));
	}
}

// ---------------------------------------------------------------------------
// TickApproach — junction approach scan + braking envelope + pre-positioning.
// ---------------------------------------------------------------------------

bool FJunctionNegotiator::TickApproach(int32 ClosestIndex, float AbsSpeed, float CurrentSpeed)
{
	bool bApproachBraking = false;

	if (!Owner->bAtDeadEnd && Owner->CachedProvider && Owner->JnctState.Phase <= EJunctionPhase::Approaching)
	{
		// Skip approach scan if the vehicle is currently on a junction lane.
		// After curve completion the vehicle may land on the junction lane;
		// scanning from it would re-detect the same junction, clear the
		// LastReleasedId guard, and trap the vehicle with no canonical
		// movements for the junction-lane approach.
		const int32 CurrentLaneJunction = Owner->CachedProvider->GetJunctionForLane(Owner->CurrentLane);
		if (CurrentLaneJunction != 0)
		{
			return false;
		}

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
			const FTrafficLaneHandle CanonicalApproachLane = ScanResult.ApproachLane.IsValid()
				? ScanResult.ApproachLane
				: Owner->CurrentLane;

			const float SafeDist = FMath::Max(Owner->JnctState.ApproachDistanceCm - Owner->ApproachSafetyMarginCm - Owner->VehicleFrontExtent, 0.0f);
			Owner->JnctState.ApproachSpeedLimitCmPerSec = FMath::Sqrt(
				FMath::Max(2.0f * Owner->ApproachDecelCmPerSec2 * SafeDist, 0.0f));

			// Proximity-conflict junction full-stop: when the junction has
			// converging exit/approach lanes AND is currently occupied, use
			// an enlarged margin so the approaching vehicle fully stops
			// before the lane-overlap zone.  The speed limit is set negative
			// to signal SpeedEnvelope to bypass its 100 cm/s floor.
			if (UWorld* ProxWorld = Owner->GetWorld())
			{
				if (const UTrafficSubsystem* ProxSub = ProxWorld->GetSubsystem<UTrafficSubsystem>())
				{
					const bool bJnctOccupied = ProxSub->IsJunctionOccupied(ScanResult.JunctionId);
					bool bJnctProximity = false;
					if (bJnctOccupied)
					{
						const TArray<int32>& MvIds = ProxSub->GetCanonicalMovementsForJunction(ScanResult.JunctionId);
						for (int32 MvId : MvIds)
						{
							if (const FCanonicalMovementRecord* Mv = ProxSub->GetCanonicalMovement(MvId))
							{
								if (Mv->bHasProximityConflicts) { bJnctProximity = true; break; }
							}
						}
						if (bJnctProximity)
						{
							Owner->JnctState.ApproachSpeedLimitCmPerSec = -1.0f;
						}
					}
				}
			}

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
								if (!Owner->IsUsableExitLane(E))
								{
									continue;
								}

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

				FTrafficLaneHandle ChosenExit = Owner->JnctState.ToLane;
				const FCanonicalMovementRecord* SelectedMovement = Owner->GetSelectedCanonicalMovement();
				if (SelectedMovement && SelectedMovement->IsValid())
				{
					if (ChosenExit.IsValid() && ChosenExit.HandleId != SelectedMovement->ToLane.HandleId)
					{
						UE_LOG(LogAAATraffic, Error,
							TEXT("JNCT ROUTE-CONTRACT: Pawn='%s' approach cached exit %d disagrees with canonical exit %d for movement %d -- realigning to canonical authority"),
							Owner->GetPawn() ? *Owner->GetPawn()->GetName() : TEXT("NULL"),
							ChosenExit.HandleId,
							SelectedMovement->ToLane.HandleId,
							SelectedMovement->MovementId);
					}

					ChosenExit = SelectedMovement->ToLane;
					Owner->JnctState.ToLane = ChosenExit;
				}
				else if (ChosenExit.IsValid())
				{
					// Canonical authority has no valid movement, but a cached
					// exit exists. Keep it so the vehicle can still navigate
					// via the Rules-based fallback in PickSurveyedExit.
					UE_LOG(LogAAATraffic, Warning,
						TEXT("JNCT ROUTE-FALLBACK: Pawn='%s' approach cached exit %d exists without canonical movement authority — keeping for fallback"),
						Owner->GetPawn() ? *Owner->GetPawn()->GetName() : TEXT("NULL"),
						ChosenExit.HandleId);
				}
				bool bKeepChosenExit = false;
				if (ChosenExit.IsValid())
				{
					for (const FTrafficLaneHandle& CandidateExit : FallbackExits)
					{
						if (CandidateExit.HandleId == ChosenExit.HandleId
							&& Owner->IsUsableExitLane(CandidateExit))
						{
							bKeepChosenExit = true;
							break;
						}
					}
				}

				if (!bKeepChosenExit)
				{
					ChosenExit = Owner->PickSurveyedExit(CanonicalApproachLane, FallbackExits);
					if (ChosenExit.IsValid())
					{
						Owner->JnctState.ToLane = ChosenExit;
					}
				}

				if (ChosenExit.IsValid())
				{
					const ETurnSignalState TurnDir = Owner->GetSelectedTurnDirection();

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
			CheckJunctionOccupancyContract(TEXT("DetectAndOccupy.PreSignal"), Owner->GetPawn(), Owner->JnctState.JunctionId, Owner->JnctState.Phase, Owner->JnctState.bOwnsJunctionOccupancy, TrafficSub, Owner);
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
							if (!Owner->IsUsableExitLane(E))
							{
								continue;
							}

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

				bool bKeepChosenExit = false;
				if (Owner->JnctState.ToLane.IsValid())
				{
					const FCanonicalMovementRecord* SelectedMovement = Owner->GetSelectedCanonicalMovement();
					if (SelectedMovement && SelectedMovement->IsValid())
					{
						if (Owner->JnctState.ToLane.HandleId != SelectedMovement->ToLane.HandleId)
						{
							UE_LOG(LogAAATraffic, Error,
								TEXT("JNCT ROUTE-CONTRACT: Pawn='%s' occupy cached exit %d disagrees with canonical exit %d for movement %d -- realigning to canonical authority"),
								Owner->GetPawn() ? *Owner->GetPawn()->GetName() : TEXT("NULL"),
								Owner->JnctState.ToLane.HandleId,
								SelectedMovement->ToLane.HandleId,
								SelectedMovement->MovementId);
						}

						Owner->JnctState.ToLane = SelectedMovement->ToLane;
					}
					else
					{
						// Canonical authority has no valid movement, but a
						// cached exit exists. Keep it for fallback navigation.
						UE_LOG(LogAAATraffic, Warning,
							TEXT("JNCT ROUTE-FALLBACK: Pawn='%s' occupy cached exit %d exists without canonical movement authority — keeping for fallback"),
							Owner->GetPawn() ? *Owner->GetPawn()->GetName() : TEXT("NULL"),
							Owner->JnctState.ToLane.HandleId);
					}
				}

				if (Owner->JnctState.ToLane.IsValid())
				{
					if (FallbackExits.IsEmpty())
					{
						bKeepChosenExit = Owner->IsUsableExitLane(Owner->JnctState.ToLane);
					}
					else
					{
						for (const FTrafficLaneHandle& CandidateExit : FallbackExits)
						{
							if (CandidateExit.HandleId == Owner->JnctState.ToLane.HandleId
								&& Owner->IsUsableExitLane(CandidateExit))
							{
								bKeepChosenExit = true;
								break;
							}
						}
					}
				}

				if (!bKeepChosenExit)
				{
					const FTrafficLaneHandle CanonicalApproachLane = Owner->JnctState.FromLane.IsValid()
						? Owner->JnctState.FromLane
						: Owner->CurrentLane;
					Owner->JnctState.ToLane = Owner->PickSurveyedExit(CanonicalApproachLane, FallbackExits);
				}

				if (Owner->JnctState.ToLane.IsValid())
				{
					if (!Owner->GetSelectedCanonicalMovement())
					{
						UE_LOG(LogAAATraffic, Warning,
							TEXT("JNCT CANONICAL-MISSING: Pawn='%s' JunctionId=%d selected exit %d without canonical movement authority — proceeding via Rules fallback"),
							Owner->GetPawn() ? *Owner->GetPawn()->GetName() : TEXT("NULL"),
							DetectedJunctionId,
							Owner->JnctState.ToLane.HandleId);
					}

					UE_LOG(LogAAATraffic, Warning,
						TEXT("JNCT EXIT-PICKED: Pawn='%s' JunctionId=%d Picked=%d (%s)"),
						Owner->GetPawn() ? *Owner->GetPawn()->GetName() : TEXT("NULL"),
						DetectedJunctionId, Owner->JnctState.ToLane.HandleId,
						bKeepChosenExit ? TEXT("latched") : TEXT("via surveyor"));
				}
				else
				{
					UE_LOG(LogAAATraffic, Warning,
						TEXT("JNCT EXIT-BLOCKED: Pawn='%s' JunctionId=%d has no usable non-terminal exit from lane %d"),
						Owner->GetPawn() ? *Owner->GetPawn()->GetName() : TEXT("NULL"),
						DetectedJunctionId,
						DetectedJunctionLane.HandleId);
					return;
				}
			}

			Owner->SetTurnSignal(Owner->GetSelectedTurnDirection());

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
					const ETurnSignalState YieldTurnDir = Owner->GetSelectedTurnDirection();
					if (YieldTurnDir == ETurnSignalState::Left
						&& TrafficSub->HasConflictingApproach(DetectedJunctionId, Owner, Owner->JnctState.CanonicalMovementId))
					{
						bMustYield = true;
					}
				}
				if (!bMustYield)
				{
					bMustYield = TrafficSub->HasApproachingCrossTraffic(
						DetectedJunctionId, Owner, Owner->JnctState.CanonicalMovementId);
				}

				if (!bMustYield && TrafficSub->TryOccupyJunction(DetectedJunctionId, Owner, Owner->JnctState.CanonicalMovementId))
				{
					Owner->JnctState.bOwnsJunctionOccupancy = true;
					CheckJunctionOccupancyContract(TEXT("DetectAndOccupy.YieldGranted"), Owner->GetPawn(), Owner->JnctState.JunctionId, Owner->JnctState.Phase, Owner->JnctState.bOwnsJunctionOccupancy, TrafficSub, Owner);
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
				const ETurnSignalState OccupyTurnDir = Owner->GetSelectedTurnDirection();
				if (OccupyTurnDir == ETurnSignalState::Left
					&& !(Signal && Signal->IsLaneProtectedGreen(Owner->JnctState.JunctionLane))
					&& TrafficSub->HasConflictingApproach(DetectedJunctionId, Owner, Owner->JnctState.CanonicalMovementId))
				{
					bLeftTurnYield = true;
				}
			}

			if (bSignalAllows && !bLeftTurnYield && TrafficSub->TryOccupyJunction(DetectedJunctionId, Owner, Owner->JnctState.CanonicalMovementId))
			{
				Owner->JnctState.bOwnsJunctionOccupancy = true;
				CheckJunctionOccupancyContract(TEXT("DetectAndOccupy.SignalGranted"), Owner->GetPawn(), Owner->JnctState.JunctionId, Owner->JnctState.Phase, Owner->JnctState.bOwnsJunctionOccupancy, TrafficSub, Owner);
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

	// Only release when the vehicle was Traversing (actually inside the
	// junction).  During the Approaching phase the vehicle may transition
	// between feeder / approach lanes whose GetJunctionForLane() returns 0;
	// releasing here would destroy the canonical movement prematurely.
	if (NewLaneJunction != Owner->JnctState.JunctionId
		&& Owner->JnctState.TransitionPoints.Num() == 0
		&& Owner->JnctState.Phase >= EJunctionPhase::Traversing)
	{
		UWorld* World = Owner->GetWorld();
		UTrafficSubsystem* TrafficSub = World
			? World->GetSubsystem<UTrafficSubsystem>() : nullptr;
		if (TrafficSub)
		{
			CheckJunctionOccupancyContract(TEXT("PostTransitionRelease.PreRelease"), Owner->GetPawn(), Owner->JnctState.JunctionId, Owner->JnctState.Phase, Owner->JnctState.bOwnsJunctionOccupancy, TrafficSub, Owner);
			if (Owner->JnctState.bOwnsJunctionOccupancy)
			{
				UE_LOG(LogAAATraffic, Warning,
					TEXT("JNCT RELEASE: Pawn='%s' RELEASING junction %d — new lane %d "
						 "is in junction %d (exited)"),
					Owner->GetPawn() ? *Owner->GetPawn()->GetName() : TEXT("NULL"),
					Owner->JnctState.JunctionId, Owner->CurrentLane.HandleId, NewLaneJunction);
				TrafficSub->ReleaseJunction(Owner->JnctState.JunctionId, Owner);
			}
			else
			{
				UE_LOG(LogAAATraffic, Warning,
					TEXT("JNCT RELEASE-SKIP: Pawn='%s' junction %d exited on lane %d with no occupancy token"),
					Owner->GetPawn() ? *Owner->GetPawn()->GetName() : TEXT("NULL"),
					Owner->JnctState.JunctionId,
					Owner->CurrentLane.HandleId);
			}
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
	// Guard: only allow relative-distance advancement when the vehicle is
	// within meaningful range of the curve.  Without this, a vehicle that
	// is far from ALL curve points (e.g. at the start of a long approach
	// lane) can complete the entire curve in a single tick because the
	// small inter-point spacing makes DistToNextSq < DistToCurrentSq true
	// for every consecutive pair.
	constexpr float kMaxRelativeAdvanceDistSq = 1000.0f * 1000.0f; // 10 m
	while (Owner->JnctState.TransitionIndex < Owner->JnctState.TransitionPoints.Num() - 1)
	{
		const float DistToCurrentSq = FVector::DistSquared(VehicleLocation, Owner->JnctState.TransitionPoints[Owner->JnctState.TransitionIndex]);
		const float DistToNextSq = FVector::DistSquared(VehicleLocation, Owner->JnctState.TransitionPoints[Owner->JnctState.TransitionIndex + 1]);
		if (DistToCurrentSq < 10000.0f || (DistToCurrentSq < kMaxRelativeAdvanceDistSq && DistToNextSq < DistToCurrentSq))
		{
			++Owner->JnctState.TransitionIndex;
		}
		else
		{
			break;
		}
	}

	if (Owner->JnctState.bOwnsJunctionOccupancy
		&& Owner->JnctState.TransitionReleaseIndex != INDEX_NONE
		&& Owner->JnctState.TransitionIndex >= Owner->JnctState.TransitionReleaseIndex)
	{
		if (UWorld* World = Owner->GetWorld())
		{
			if (UTrafficSubsystem* TrafficSub = World->GetSubsystem<UTrafficSubsystem>())
			{
				CheckJunctionOccupancyContract(TEXT("TickTraverse.EarlyRelease"), Owner->GetPawn(), Owner->JnctState.JunctionId, Owner->JnctState.Phase, Owner->JnctState.bOwnsJunctionOccupancy, TrafficSub, Owner);

				// Check if exit lane is a short dead-end. On short dead-ends
				// the vehicle stops inside the junction area before clearing
				// it, so defer the release to prevent collisions. Long dead-
				// ends are safe because the vehicle drives away quickly.
				// Also defer for proximity-conflict movements where approach/
				// exit lanes are physically close, requiring clearance distance
				// before another vehicle enters.
				const FTrafficLaneHandle EarlyExitLane = Owner->JnctState.ToLane;
				const bool bEarlyExitDeadEnd = EarlyExitLane.IsValid()
					&& Owner->CachedProvider
					&& Owner->CachedProvider->GetConnectedLanes(EarlyExitLane).Num() == 0
					&& Owner->CachedProvider->GetLaneLength(EarlyExitLane) < 2000.0f;

				bool bProximityDefer = false;
				if (const FCanonicalMovementRecord* MvRec = TrafficSub->GetCanonicalMovement(Owner->JnctState.CanonicalMovementId))
				{
					bProximityDefer = MvRec->bHasProximityConflicts;
				}

				if (bEarlyExitDeadEnd || bProximityDefer)
				{
					// Proximity-conflict exits: defer until the vehicle clears the
					// entire exit lane.  The approach lane of the conflicting
					// movement overlaps the exit lane's full length, so 3000 cm
					// was insufficient (Lane 77 ≈ 4270 cm in ActiveSegments map).
					const float ProxDeferDist = (bProximityDefer && Owner->CachedProvider)
						? Owner->CachedProvider->GetLaneLength(EarlyExitLane)
						: 3000.0f;
					const float DeferDist = bProximityDefer ? ProxDeferDist : 1500.0f;
					UE_LOG(LogAAATraffic, Warning,
						TEXT("JNCT EARLY-RELEASE-DEFERRED: Pawn='%s' junction %d — "
							 "ExitLane=%d %s, deferring release (dist=%.0f)"),
						Owner->GetPawn() ? *Owner->GetPawn()->GetName() : TEXT("NULL"),
						Owner->JnctState.JunctionId, EarlyExitLane.HandleId,
						bEarlyExitDeadEnd ? TEXT("is dead-end") : TEXT("has proximity conflicts"),
						DeferDist);
					Owner->bDeferredJunctionRelease = true;
					Owner->DeferredJunctionReleaseId = Owner->JnctState.JunctionId;
					Owner->DeferredJunctionReleaseOrigin = VehicleLocation;
					Owner->DeferredJunctionReleaseDistCm = DeferDist;
					Owner->JnctState.bOwnsJunctionOccupancy = false;
				}
				else
				{
					TrafficSub->ReleaseJunction(Owner->JnctState.JunctionId, Owner);
					Owner->JnctState.bOwnsJunctionOccupancy = false;
				}
			}
		}
	}

	if (Owner->JnctState.TransitionIndex >= Owner->JnctState.TransitionPoints.Num() - 1)
	{
		// Finished junction transition — release junction occupancy and proceed.
		TArray<FVector> TransitionTail = Owner->JnctState.TransitionPoints;
		const FVector TransitionEndPoint = Owner->JnctState.TransitionPoints.Last();
		const int32 ResumePointIndex = FMath::Clamp(
			Owner->JnctState.ExitLaneResumeIndex != INDEX_NONE ? Owner->JnctState.ExitLaneResumeIndex : (TransitionTail.Num() - 1),
			0,
			TransitionTail.Num() - 1);
		const FVector ResumeReferencePoint = TransitionTail.IsValidIndex(ResumePointIndex)
			? TransitionTail[ResumePointIndex]
			: TransitionEndPoint;
		Owner->JnctState.TransitionPoints.Empty();
		Owner->JnctState.TransitionIndex = 0;

		Owner->TurnSignalOffDelayRemaining = 1.5f;
		Owner->LaneChangeCoord_.ClearNav();

		UE_LOG(LogAAATraffic, Warning,
			TEXT("JNCT RELEASE-PATH2-CHECK: Pawn='%s' JnctState.JunctionId=%d "
				 "— junction curve complete, checking release"),
			Owner->GetPawn() ? *Owner->GetPawn()->GetName() : TEXT("NULL"),
			Owner->JnctState.JunctionId);

		const bool bCanReleasePhase =
			Owner->JnctState.Phase >= EJunctionPhase::Approaching &&
			Owner->JnctState.Phase <= EJunctionPhase::Traversing;
		const int32 ReleasedJunctionId = Owner->JnctState.JunctionId;
		// Save the canonical exit lane before Release() clears JnctState.
		const FTrafficLaneHandle ExitLane = Owner->JnctState.ToLane;

		if (bCanReleasePhase && ReleasedJunctionId > 0)
		{
			if (UWorld* World = Owner->GetWorld())
			{
				if (UTrafficSubsystem* TrafficSub = World->GetSubsystem<UTrafficSubsystem>())
				{
					// Skip contract check when deferred release is active — the
					// local token was intentionally cleared by PATH1 while the
					// subsystem record persists until the deferred release fires.
					if (!(Owner->bDeferredJunctionRelease && Owner->DeferredJunctionReleaseId == ReleasedJunctionId))
					{
						CheckJunctionOccupancyContract(TEXT("TickTraverse.PreRelease"), Owner->GetPawn(), Owner->JnctState.JunctionId, Owner->JnctState.Phase, Owner->JnctState.bOwnsJunctionOccupancy, TrafficSub, Owner);
					}
					if (Owner->JnctState.bOwnsJunctionOccupancy)
					{
						// Check if exit lane is a short dead-end. Same logic
						// as the early release path — only defer for short
						// dead-ends where vehicles stop inside the junction.
						// Also defer for proximity-conflict movements.
						const bool bExitLaneDeadEnd = ExitLane.IsValid()
							&& Owner->CachedProvider
							&& Owner->CachedProvider->GetConnectedLanes(ExitLane).Num() == 0
							&& Owner->CachedProvider->GetLaneLength(ExitLane) < 2000.0f;

						bool bProximityDefer = false;
						if (const FCanonicalMovementRecord* MvRec = TrafficSub->GetCanonicalMovement(Owner->JnctState.CanonicalMovementId))
						{
							bProximityDefer = MvRec->bHasProximityConflicts;
						}

						if (bExitLaneDeadEnd || bProximityDefer)
						{
							const float DeferDist = bProximityDefer ? 3000.0f : 1500.0f;
							UE_LOG(LogAAATraffic, Warning,
								TEXT("JNCT RELEASE-PATH2-DEFERRED: Pawn='%s' junction %d — "
									 "ExitLane=%d %s, deferring release (dist=%.0f)"),
								Owner->GetPawn() ? *Owner->GetPawn()->GetName() : TEXT("NULL"),
								ReleasedJunctionId, ExitLane.HandleId,
								bExitLaneDeadEnd ? TEXT("is dead-end") : TEXT("has proximity conflicts"),
								DeferDist);
							Owner->bDeferredJunctionRelease = true;
							Owner->DeferredJunctionReleaseId = ReleasedJunctionId;
							Owner->DeferredJunctionReleaseOrigin = VehicleLocation;
							Owner->DeferredJunctionReleaseDistCm = DeferDist;
						}
						else
						{
							UE_LOG(LogAAATraffic, Warning,
								TEXT("JNCT RELEASE-PATH2-FIRE: Pawn='%s' RELEASING junction %d (curve complete)"),
								Owner->GetPawn() ? *Owner->GetPawn()->GetName() : TEXT("NULL"),
								ReleasedJunctionId);
							TrafficSub->ReleaseJunction(ReleasedJunctionId, Owner);
						}
					}
					else
					{
						UE_LOG(LogAAATraffic, Warning,
							TEXT("JNCT RELEASE-PATH2-SKIP: Pawn='%s' junction %d curve complete with no occupancy token"),
							Owner->GetPawn() ? *Owner->GetPawn()->GetName() : TEXT("NULL"),
							ReleasedJunctionId);
					}
				}
			}

			Owner->JnctState.Release();
		}
		else
		{
			UE_LOG(LogAAATraffic, Warning,
				TEXT("JNCT RELEASE-PATH2-SKIP: Pawn='%s' invalid release state Phase=%d JunctionId=%d — resetting junction state"),
				Owner->GetPawn() ? *Owner->GetPawn()->GetName() : TEXT("NULL"),
				(int32)Owner->JnctState.Phase,
				ReleasedJunctionId);
			Owner->JnctState.Reset();
		}

		// ── Degenerate curve check ──
		// A self-referential provider curve (From==To) traces along the
		// approach lane itself.  The vehicle naturally "completes" it
		// while still far from the curve's endpoint.  Switching to the
		// exit lane would place CurrentLane on a distant road, causing
		// wrong-way detection before despawn can clean up.
		// Guard: if the vehicle is too far from the curve endpoint,
		// skip the exit-lane switch and despawn on the current lane.
		constexpr float MaxCurveEndGapCm = 2000.0f; // 20 m
		const float DistFromCurveEndSq = FVector::DistSquared2D(VehicleLocation, TransitionEndPoint);
		if (DistFromCurveEndSq > MaxCurveEndGapCm * MaxCurveEndGapCm)
		{
			UE_LOG(LogAAATraffic, Warning,
				TEXT("JNCT DEGENERATE-CURVE: Pawn='%s' Lane=%d dist=%.0f cm "
					 "from curve endpoint — skipping exit-lane switch, despawning."),
				Owner->GetPawn() ? *Owner->GetPawn()->GetName() : TEXT("null"),
				Owner->CurrentLane.HandleId,
				FMath::Sqrt(DistFromCurveEndSq));
			Owner->bPendingRecoveryDespawn = true;
			if (UWorld* W = Owner->GetWorld())
			{
				if (UTrafficSubsystem* Sub = W->GetSubsystem<UTrafficSubsystem>())
				{
					Sub->RequestDespawn(Owner,
						FString::Printf(TEXT("degenerate junction curve (%.0f cm from curve end)"),
							FMath::Sqrt(DistFromCurveEndSq)));
				}
			}
			OutTargetPoint = Owner->LanePoints.IsValidIndex(Owner->LastClosestIndex)
				? Owner->LanePoints[Owner->LastClosestIndex]
				: VehicleLocation;
			return true;
		}

		// ── Transition to exit lane ──
		// The junction curve took the vehicle from approach → canonical exit.
		// CurrentLane is still the junction interior lane whose physical
		// connection goes to a non-canonical exit.  Re-initialize on the
		// canonical exit lane so the vehicle follows the correct path.
		const bool bNeedsExitLaneSwitch = ExitLane.IsValid()
			&& ExitLane.HandleId != Owner->CurrentLane.HandleId
			&& Owner->CachedProvider;

		if (bNeedsExitLaneSwitch)
		{
			const int32 OldLane = Owner->CurrentLane.HandleId;
			Owner->CurrentLane = ExitLane;
			Owner->LanePoints.Reset();
			float NewLaneWidth = 0.0f;
			if (Owner->CachedProvider->GetLanePath(ExitLane, Owner->LanePoints, NewLaneWidth)
				&& Owner->LanePoints.Num() >= 2)
			{
				if (NewLaneWidth > 0.0f)
				{
					Owner->LaneWidth = NewLaneWidth;
				}
				Owner->bLaneDataReady = true;
				const int32 ResumeLaneIndex = Owner->FindClosestPointIndex(VehicleLocation, true);
				Owner->LastClosestIndex = FMath::Clamp(ResumeLaneIndex, 0, Owner->LanePoints.Num() - 1);
				Owner->SteeringComputer.Reset();
				Owner->NoRouteStuckTimer = 0.0f;
				Owner->PostExitCTERampRemaining = Owner->PostExitCTERampDuration;
				UE_LOG(LogAAATraffic, Log,
					TEXT("JNCT EXIT-LANE-SWITCH: Pawn='%s' JunctionLane=%d -> ExitLane=%d ResumeIdx=%d/%d VehiclePos=(%.1f,%.1f,%.1f) CurveEnd=(%.1f,%.1f,%.1f)"),
					Owner->GetPawn() ? *Owner->GetPawn()->GetName() : TEXT("NULL"),
					OldLane,
					ExitLane.HandleId,
					Owner->LastClosestIndex,
					Owner->LanePoints.Num(),
					VehicleLocation.X,
					VehicleLocation.Y,
					VehicleLocation.Z,
					ResumeReferencePoint.X,
					ResumeReferencePoint.Y,
					ResumeReferencePoint.Z);
			}
			else
			{
				// Failed to fetch exit lane — fall back to current lane resume.
				Owner->CurrentLane = FTrafficLaneHandle();
				Owner->CurrentLane.HandleId = OldLane;
				Owner->LanePoints.Reset();
				if (Owner->CachedProvider->GetLanePath(Owner->CurrentLane, Owner->LanePoints, Owner->LaneWidth))
				{
					Owner->bLaneDataReady = Owner->LanePoints.Num() >= 2;
				}
				const int32 ResumeLaneIndex = Owner->FindClosestPointIndex(VehicleLocation, true);
				Owner->LastClosestIndex = FMath::Clamp(ResumeLaneIndex, 0, FMath::Max(Owner->LanePoints.Num() - 1, 0));
				Owner->SteeringComputer.Reset();
				UE_LOG(LogAAATraffic, Warning,
					TEXT("JNCT EXIT-LANE-SWITCH-FAIL: Pawn='%s' could not fetch exit lane %d — resuming on junction lane %d"),
					Owner->GetPawn() ? *Owner->GetPawn()->GetName() : TEXT("NULL"),
					ExitLane.HandleId,
					OldLane);
			}
		}
		else
		{
			const int32 ResumeLaneIndex = Owner->FindClosestPointIndex(VehicleLocation, true);
			Owner->LastClosestIndex = FMath::Clamp(ResumeLaneIndex, 0, FMath::Max(Owner->LanePoints.Num() - 1, 0));
			Owner->SteeringComputer.Reset();
			Owner->PostExitCTERampRemaining = Owner->PostExitCTERampDuration;
			UE_LOG(LogAAATraffic, Log,
				TEXT("JNCT LANE-REACQUIRE: Pawn='%s' Lane=%d OldIdx=%d ResumeIdx=%d VehiclePos=(%.1f,%.1f,%.1f)"),
				Owner->GetPawn() ? *Owner->GetPawn()->GetName() : TEXT("NULL"),
				Owner->CurrentLane.HandleId,
				ClosestIndex,
				Owner->LastClosestIndex,
				VehicleLocation.X,
				VehicleLocation.Y,
				VehicleLocation.Z);
		}

		// ── Post-exit position snap ────────────────────────────────
		// If the vehicle ended the junction curve too far from the exit
		// lane (curve endpoint / exit-lane geometry mismatch, or physics
		// collision during traversal), ordinary CTE correction cannot
		// recover in time.  Snap the pawn to the nearest lane point so
		// the vehicle doesn't drive off-road indefinitely.
		FVector EffectiveVehicleLocation = VehicleLocation;
		if (Owner->LanePoints.IsValidIndex(Owner->LastClosestIndex))
		{
			const FVector& ClosestLanePt = Owner->LanePoints[Owner->LastClosestIndex];
			const float DistToLaneSq = FVector::DistSquared2D(VehicleLocation, ClosestLanePt);
			const float SnapThresholdCm = Owner->LaneWidth * 1.5f;
			const float MaxSnapCm = Owner->LaneWidth * 5.0f;

			// Proximity-conflict exits: if the vehicle lands far from the
			// exit lane center, either CTE correction swerves through the
			// overlapping approach lane or a TeleportPhysics snap desyncs
			// ChaosVehicle sub-bodies (doors) causing phantom collisions.
			// Despawn instead — cleanest recovery for both failure modes.
			const float ProxDespawnThreshCm = Owner->LaneWidth * 0.3f; // ~105 cm
			if (Owner->bDeferredJunctionRelease
				&& DistToLaneSq > ProxDespawnThreshCm * ProxDespawnThreshCm)
			{
				UE_LOG(LogAAATraffic, Warning,
					TEXT("JNCT EXIT-PROX-DESPAWN: Pawn='%s' Lane=%d dist=%.0f cm "
						 "from lane center on proximity-conflict exit — despawning."),
					Owner->GetPawn() ? *Owner->GetPawn()->GetName() : TEXT("null"),
					Owner->CurrentLane.HandleId,
					FMath::Sqrt(DistToLaneSq));
				Owner->bPendingRecoveryDespawn = true;
				if (UWorld* W = Owner->GetWorld())
				{
					if (UTrafficSubsystem* Sub = W->GetSubsystem<UTrafficSubsystem>())
					{
						Sub->RequestDespawn(Owner,
							FString::Printf(TEXT("proximity-exit CTE=%.0f cm"),
								FMath::Sqrt(DistToLaneSq)));
					}
				}
			}
			else if (DistToLaneSq > SnapThresholdCm * SnapThresholdCm)
			{
				// If the vehicle is catastrophically far from the lane,
				// snapping would teleport it to a mismatched position
				// (potentially facing the wrong direction). Skip the snap
				// and let the off-road despawn mechanism handle cleanup.
				if (DistToLaneSq > MaxSnapCm * MaxSnapCm)
				{
					UE_LOG(LogAAATraffic, Warning,
						TEXT("JNCT EXIT-SNAP-SKIP: Pawn='%s' Lane=%d dist=%.0f cm "
							 "exceeds max snap (%.0f cm) — requesting recovery despawn."),
						Owner->GetPawn() ? *Owner->GetPawn()->GetName() : TEXT("null"),
						Owner->CurrentLane.HandleId,
						FMath::Sqrt(DistToLaneSq),
						MaxSnapCm);
					Owner->bPendingRecoveryDespawn = true;
					if (UWorld* W = Owner->GetWorld())
					{
						if (UTrafficSubsystem* Sub = W->GetSubsystem<UTrafficSubsystem>())
						{
							Sub->RequestDespawn(Owner,
								FString::Printf(TEXT("EXIT-SNAP-SKIP dist=%.0f cm"),
									FMath::Sqrt(DistToLaneSq)));
						}
					}
				}
				else
				{
					APawn* Pawn = Owner->GetPawn();
					if (Pawn)
					{
						const FVector SnapTarget(ClosestLanePt.X, ClosestLanePt.Y, VehicleLocation.Z);
						Pawn->SetActorLocation(SnapTarget, /*bSweep=*/ false,
							/*OutSweepHitResult=*/ nullptr, ETeleportType::TeleportPhysics);
						EffectiveVehicleLocation = SnapTarget;
						UE_LOG(LogAAATraffic, Warning,
							TEXT("JNCT EXIT-SNAP: Pawn='%s' Lane=%d snapped %.0f cm to lane "
								 "(threshold=%.0f cm)"),
							*Pawn->GetName(),
							Owner->CurrentLane.HandleId,
							FMath::Sqrt(DistToLaneSq),
							SnapThresholdCm);
					}
				}
			}
		}

		OutTargetPoint = Owner->GetLookAheadPoint(EffectiveVehicleLocation, Owner->LastClosestIndex);
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
