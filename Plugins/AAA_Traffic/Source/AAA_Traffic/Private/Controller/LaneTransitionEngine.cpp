// LaneTransitionEngine.cpp — Implementation of lane-end transition and lane initialization.
#include "LaneTransitionEngine.h"
#include "TrafficVehicleController.h"
#include "TrafficSubsystem.h"
#include "TrafficLog.h"
#include "GameFramework/Pawn.h"

extern int32 GTrafficJunctionDiagnostics;
extern bool GTrafficVehicleTraceFlushOnSuccess;

// ---------------------------------------------------------------------------
// InitializeLane — sets up lane-following state for a new lane.
// ---------------------------------------------------------------------------

void FLaneTransitionEngine::InitializeLane(const FTrafficLaneHandle& InLane)
{
	// ── Phase-aware junction state handling ──
	if (Owner->JnctState.Phase == EJunctionPhase::Traversing)
	{
		UE_LOG(LogAAATraffic, Log,
			TEXT("JNCT INIT-KEEP: Pawn='%s' Phase=Traversing JunctionId=%d NewLane=%d — preserving junction state"),
			Owner->GetPawn() ? *Owner->GetPawn()->GetName() : TEXT("NULL"),
			Owner->JnctState.JunctionId, InLane.HandleId);
	}
	else if (Owner->JnctState.Phase == EJunctionPhase::Approaching)
	{
		UE_LOG(LogAAATraffic, Log,
			TEXT("JNCT INIT-KEEP: Pawn='%s' Phase=Approaching JunctionId=%d NewLane=%d ExitLane=%d — preserving approach state"),
			Owner->GetPawn() ? *Owner->GetPawn()->GetName() : TEXT("NULL"),
			Owner->JnctState.JunctionId, InLane.HandleId, Owner->JnctState.ToLane.HandleId);
	}
	else
	{
		if (Owner->JnctState.IsActive())
		{
			UE_LOG(LogAAATraffic, Warning,
				TEXT("JNCT INIT-RESET: Pawn='%s' Phase=%d JunctionId=%d bWaiting=%s TransPts=%d NewLane=%d — resetting"),
				Owner->GetPawn() ? *Owner->GetPawn()->GetName() : TEXT("NULL"),
				(int32)Owner->JnctState.Phase, Owner->JnctState.JunctionId,
				Owner->JnctState.bWaiting ? TEXT("YES") : TEXT("NO"),
				Owner->JnctState.TransitionPoints.Num(),
				InLane.HandleId);

			if (Owner->JnctState.Phase == EJunctionPhase::Waiting && Owner->JnctState.JunctionId != 0 && Owner->JnctState.bOwnsJunctionOccupancy)
			{
				if (UWorld* W = Owner->GetWorld())
				{
					if (UTrafficSubsystem* TrafficSub = W->GetSubsystem<UTrafficSubsystem>())
					{
						TrafficSub->ReleaseJunction(Owner->JnctState.JunctionId, Owner);
					}
				}
			}
		}
		Owner->JnctState.Reset();
	}

	Owner->CurrentLane = InLane;
	Owner->bLaneDataReady = false;
	Owner->bAtDeadEnd = false;
	Owner->DeadEndDespawnTimer = 0.0f;
	Owner->DistanceTraveledOnLane = 0.0f;
	Owner->PreviousVehicleLocation = FVector::ZeroVector;
	Owner->LastClosestIndex = 0;
	Owner->SteeringComputer.Reset();
	Owner->WaitLogThrottleCounter = 0;

	Owner->AccelModel.ClearDelayBuffer();

	Owner->CurrentTurnSignal = ETurnSignalState::Off;

	Owner->LaneChangeCoord_.ClearNav();
	Owner->LaneChangeCoord_.Reset();

	Owner->bFirstTickOnLane = true;

	UWorld* World = Owner->GetWorld();
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

	Owner->CachedProvider = Provider;

	Owner->LanePoints.Reset();
	ITrafficRoadProvider::FJunctionScanResult InitScan;
	if (Provider->GetLanePath(Owner->CurrentLane, Owner->LanePoints, Owner->LaneWidth) && Owner->LanePoints.Num() >= 2)
	{
		Owner->bLaneDataReady = true;

		// --- JUNCTION DIAGNOSTIC ---
		{
			const float TotalLen = Provider->GetLaneLength(Owner->CurrentLane);
			InitScan =
				Provider->GetDistanceToNextJunction(Owner->CurrentLane, TotalLen, 50000.0f, 10);
			const int32 DirectJunction = Provider->GetJunctionForLane(Owner->CurrentLane);
			TArray<FTrafficLaneHandle> InitConnected = Provider->GetConnectedLanes(Owner->CurrentLane);

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
				Owner->GetPawn() ? *Owner->GetPawn()->GetName() : TEXT("NULL"),
				Owner->CurrentLane.HandleId,
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
		for (int32 i = 0; i < Owner->LanePoints.Num() - 1; ++i)
		{
			TotalLaneLength += FVector::Dist(Owner->LanePoints[i], Owner->LanePoints[i + 1]);
		}

		UE_LOG(LogAAATraffic, Log,
			TEXT("TrafficVehicleController: Lane loaded — %d points, width %.1f cm, length %.1f cm (%.1f m)."),
			Owner->LanePoints.Num(), Owner->LaneWidth, TotalLaneLength, TotalLaneLength / 100.0f);

		const float LaneSpeedLimit = Provider->GetLaneSpeedLimit(Owner->CurrentLane);
		if (LaneSpeedLimit > 0.0f)
		{
			Owner->TargetSpeed = LaneSpeedLimit;
		}
		else if (Owner->DefaultSpeedLimit > 0.0f)
		{
			Owner->TargetSpeed = Owner->DefaultSpeedLimit;
		}
		if (Owner->BaseTargetSpeed <= 0.0f)
		{
			Owner->BaseTargetSpeed = Owner->TargetSpeed;
		}
	}
	else
	{
		UE_LOG(LogAAATraffic, Warning, TEXT("TrafficVehicleController: Failed to load lane path data."));
	}

	if (TrafficSub)
	{
		TrafficSub->UpdateVehicleLane(Owner, Owner->CurrentLane);

		if (Owner->JnctState.Phase == EJunctionPhase::Idle)
		{
			const TArray<int32>& InitCanonicalMovementIds =
				TrafficSub->GetCanonicalMovementsForApproachLane(Owner->CurrentLane.HandleId);
			if (!InitCanonicalMovementIds.IsEmpty() && !Owner->GetSelectedCanonicalMovement())
			{
				int32 InitialMovementId = 0;
				const TArray<FTrafficLaneHandle> NoFallbackExits;
				Owner->JnctState.BeginApproach(InitScan.JunctionId);
				Owner->JnctState.ApproachJunctionLane = InitScan.JunctionLane;
				Owner->JnctState.JunctionLane = InitScan.JunctionLane;
				Owner->JnctState.FromLane = Owner->CurrentLane;
				Owner->JnctState.ToLane = Owner->PickSurveyedExit(Owner->CurrentLane, NoFallbackExits);
				InitialMovementId = Owner->JnctState.CanonicalMovementId;

				if (const FCanonicalMovementRecord* InitialMovement = TrafficSub->GetCanonicalMovement(InitialMovementId))
				{
					Owner->JnctState.JunctionId = InitialMovement->JunctionId;
					Owner->JnctState.CanonicalMovementId = InitialMovementId;
					Owner->JnctState.FromLane = Owner->CurrentLane;
					Owner->JnctState.JunctionLane = InitialMovement->ApproachJunctionLane;
					Owner->JnctState.ApproachJunctionLane = InitialMovement->ApproachJunctionLane;
					Owner->JnctState.ToLane = InitialMovement->ToLane;
					if (InitScan.IsValid() && InitScan.JunctionId == InitialMovement->JunctionId)
					{
						Owner->JnctState.ApproachDistanceCm = InitScan.DistanceCm;
					}

					UE_LOG(LogAAATraffic, Log,
						TEXT("JNCT INIT-ARM: Pawn='%s' Movement=%d JunctionId=%d Approach=%d JunctionLane=%d Exit=%d Dist=%.0f"),
						Owner->GetPawn() ? *Owner->GetPawn()->GetName() : TEXT("NULL"),
						InitialMovement->MovementId,
						InitialMovement->JunctionId,
						Owner->CurrentLane.HandleId,
						InitialMovement->ApproachJunctionLane.HandleId,
						InitialMovement->ToLane.HandleId,
						Owner->JnctState.ApproachDistanceCm);
				}
				else
				{
					Owner->JnctState.Reset();
				}
			}
		}
	}
}

// ---------------------------------------------------------------------------
// CheckTransition — full lane-end transition.
// ---------------------------------------------------------------------------

void FLaneTransitionEngine::CheckTransition()
{
	if (Owner->bAtDeadEnd)
	{
		return;
	}

	Owner->AddLaneDecisionTrace(TEXT("TransitionCheck.Start"), 0,
		static_cast<float>(Owner->CurrentLane.HandleId), 0.0f,
		TEXT("Entering lane-end transition check"));

	if (!Owner->CachedProvider)
	{
		Owner->bAtDeadEnd = true;
		Owner->SetTurnSignal(ETurnSignalState::Hazard);
		Owner->AddLaneDecisionTrace(TEXT("TransitionCheck.NoProvider"), 0, 0.0f, 0.0f, TEXT("No cached provider"));
		Owner->FlushLaneDecisionTrace(TEXT("NoProviderDeadEnd"), true);
		UE_LOG(LogAAATraffic, Log,
			TEXT("TrafficVehicleController: No provider cached — dead end on lane %d."),
			Owner->CurrentLane.HandleId);
		return;
	}

	TArray<FTrafficLaneHandle> Connected = Owner->CachedProvider->GetConnectedLanes(Owner->CurrentLane);

	// ── One-way / turn-restriction filter ──
	if (Connected.Num() >= 1)
	{
		const float CurLaneLen = Owner->CachedProvider->GetLaneLength(Owner->CurrentLane);
		const FVector ApproachTangent = Owner->CachedProvider->GetLaneDirectionAtDistance(Owner->CurrentLane, CurLaneLen);
		TArray<FTrafficLaneHandle> Filtered;
		Filtered.Reserve(Connected.Num());
		for (const FTrafficLaneHandle& C : Connected)
		{
			const FVector ExitDir = Owner->CachedProvider->GetLaneDirection(C);
			if (FVector::DotProduct(ApproachTangent, ExitDir) >= 0.0f)
			{
				Filtered.Add(C);
			}
		}
		if (Filtered.Num() > 0)
		{
			Connected = MoveTemp(Filtered);
		}
		else
		{
			// All connected lanes are oncoming — treat as dead end.
			Connected.Empty();
		}
	}

	Owner->AddLaneDecisionTrace(TEXT("TransitionCheck.ConnectedFetched"), 0,
		static_cast<float>(Connected.Num()), 0.0f, TEXT("Fetched connected lanes"));

	if (Connected.IsEmpty())
	{
		Owner->bAtDeadEnd = true;
		Owner->DeadEndDespawnTimer = 0.0f;
		Owner->SetTurnSignal(ETurnSignalState::Hazard);
		Owner->AddLaneDecisionTrace(TEXT("TransitionCheck.DeadEnd"), 0, 0.0f, 0.0f,
			TEXT("No connected lanes — dead end, will despawn"));
		Owner->FlushLaneDecisionTrace(TEXT("DeadEndDespawn"), true);
		UE_LOG(LogAAATraffic, Warning,
			TEXT("TrafficVehicleController: Lane %d is a dead-end (no forward connections). "
				 "Vehicle will stop and despawn in %.1fs."),
			Owner->CurrentLane.HandleId, Owner->DeadEndDespawnDelaySec);
		return;
	}

	// --- Exit selection ---
	FTrafficLaneHandle NextLane;
	bool bUsedPreselected = false;
	UWorld* World = Owner->GetWorld();
	UTrafficSubsystem* TrafficSub = World ? World->GetSubsystem<UTrafficSubsystem>() : nullptr;
	const FCanonicalMovementRecord* SelectedCanonicalMovement = Owner->GetSelectedCanonicalMovement();
	const bool bCurrentLaneHasCanonicalApproach = TrafficSub
		&& !TrafficSub->GetCanonicalMovementsForApproachLane(Owner->CurrentLane.HandleId).IsEmpty();
	const bool bJunctionRouteAuthorityActive = Owner->JnctState.IsActive() || bCurrentLaneHasCanonicalApproach;

	if (SelectedCanonicalMovement && SelectedCanonicalMovement->IsValid())
	{
		if (Owner->JnctState.ToLane.IsValid()
			&& Owner->JnctState.ToLane.HandleId != SelectedCanonicalMovement->ToLane.HandleId)
		{
			UE_LOG(LogAAATraffic, Error,
				TEXT("JNCT ROUTE-CONTRACT: Pawn='%s' cached exit %d disagrees with canonical exit %d for movement %d -- realigning to canonical authority"),
				Owner->GetPawn() ? *Owner->GetPawn()->GetName() : TEXT("NULL"),
				Owner->JnctState.ToLane.HandleId,
				SelectedCanonicalMovement->ToLane.HandleId,
				SelectedCanonicalMovement->MovementId);
		}

		Owner->JnctState.ToLane = SelectedCanonicalMovement->ToLane;
	}
	else if (bJunctionRouteAuthorityActive && Owner->JnctState.ToLane.IsValid())
	{
		// Canonical authority exists but has no valid movement — keep the
		// cached exit so the vehicle can still navigate via the Rules table.
		UE_LOG(LogAAATraffic, Warning,
			TEXT("JNCT ROUTE-FALLBACK: Pawn='%s' cached exit %d exists without canonical movement authority — keeping for Rules-based fallback"),
			Owner->GetPawn() ? *Owner->GetPawn()->GetName() : TEXT("NULL"),
			Owner->JnctState.ToLane.HandleId);
	}

	if (SelectedCanonicalMovement
		&& SelectedCanonicalMovement->IsValid()
		&& SelectedCanonicalMovement->FromLane.HandleId == Owner->CurrentLane.HandleId)
	{
		for (const FTrafficLaneHandle& C : Connected)
		{
			if (C.HandleId == SelectedCanonicalMovement->ApproachJunctionLane.HandleId)
			{
				NextLane = C;
				bUsedPreselected = true;
				Owner->AddLaneDecisionTrace(TEXT("TransitionCheck.UsedCanonicalApproachLane"),
					NextLane.HandleId,
					static_cast<float>(SelectedCanonicalMovement->MovementId),
					0.0f,
					TEXT("Advancing onto canonical approach junction lane"));
				break;
			}
		}
	}

	// --- Feeder lane advancement ---
	// Vehicle has canonical authority for a downstream approach lane (FromLane != CurrentLane).
	// If FromLane is directly connected, advance to it so the vehicle reaches the canonical approach.
	if (!bUsedPreselected
		&& SelectedCanonicalMovement
		&& SelectedCanonicalMovement->IsValid()
		&& SelectedCanonicalMovement->FromLane.HandleId != Owner->CurrentLane.HandleId)
	{
		for (const FTrafficLaneHandle& C : Connected)
		{
			if (C.HandleId == SelectedCanonicalMovement->FromLane.HandleId)
			{
				NextLane = C;
				bUsedPreselected = true;
				Owner->AddLaneDecisionTrace(TEXT("TransitionCheck.FeederToApproach"),
					NextLane.HandleId,
					static_cast<float>(SelectedCanonicalMovement->MovementId),
					0.0f,
					TEXT("Advancing from feeder lane to canonical approach lane"));
				UE_LOG(LogAAATraffic, Log,
					TEXT("JNCT FEEDER-ADVANCE: Pawn='%s' Lane=%d -> Approach=%d Movement=%d"),
					Owner->GetPawn() ? *Owner->GetPawn()->GetName() : TEXT("NULL"),
					Owner->CurrentLane.HandleId,
					NextLane.HandleId,
					SelectedCanonicalMovement->MovementId);
				break;
			}
		}
	}

	if (Owner->JnctState.ToLane.HandleId != 0
		&& Owner->JnctState.ToLane.HandleId != Owner->CurrentLane.HandleId)
	{
		if (!Owner->IsUsableExitLane(Owner->JnctState.ToLane))
		{
			UE_LOG(LogAAATraffic, Warning,
				TEXT("JNCT PRESELECTED-EXIT-REJECTED: Pawn='%s' JunctionId=%d Exit=%d is terminal/invalid; clearing latched exit"),
				Owner->GetPawn() ? *Owner->GetPawn()->GetName() : TEXT("NULL"),
				Owner->JnctState.JunctionId,
				Owner->JnctState.ToLane.HandleId);
			Owner->JnctState.ToLane = FTrafficLaneHandle();
		}
	}

	if (Owner->JnctState.ToLane.HandleId != 0
		&& Owner->JnctState.ToLane.HandleId != Owner->CurrentLane.HandleId)
	{
		for (const FTrafficLaneHandle& C : Connected)
		{
			if (C.HandleId == Owner->JnctState.ToLane.HandleId)
			{
				NextLane = Owner->JnctState.ToLane;
				bUsedPreselected = true;
				Owner->AddLaneDecisionTrace(TEXT("TransitionCheck.UsedPreselectedExit"),
					NextLane.HandleId, 0.0f, 0.0f, TEXT("Reused junction pre-selected exit"));
				break;
			}
		}
	}

	// ── Turn bypass ──
	if (!bUsedPreselected && Owner->JnctState.ToLane.HandleId != 0
		&& Owner->JnctState.ToLane.HandleId != Owner->CurrentLane.HandleId
		&& Owner->CachedProvider)
	{
		bool bReachableViaDirectJunction = false;
		for (const FTrafficLaneHandle& C : Connected)
		{
			if (Owner->CachedProvider->GetJunctionForLane(C) != 0)
			{
				TArray<FTrafficLaneHandle> JExits = Owner->CachedProvider->GetConnectedLanes(C);
				for (const FTrafficLaneHandle& E : JExits)
				{
					if (E.HandleId == Owner->JnctState.ToLane.HandleId)
					{
						bReachableViaDirectJunction = true;
						break;
					}
				}
			}
			if (bReachableViaDirectJunction) { break; }
		}

		if (!bReachableViaDirectJunction && Owner->JnctState.bOwnsJunctionOccupancy)
		{
			// Validate that the bypass target lane actually exists.
			const float ExitLen = Owner->CachedProvider->GetLaneLength(Owner->JnctState.ToLane);
			if (ExitLen > 0.0f)
			{
				NextLane = Owner->JnctState.ToLane;
			bUsedPreselected = true;
			UE_LOG(LogAAATraffic, Warning,
				TEXT("JNCT TURN-BYPASS: Pawn='%s' Approach=%d -> Exit=%d "
					 "(skipping junction lane, will generate turn curve)"),
				Owner->GetPawn() ? *Owner->GetPawn()->GetName() : TEXT("NULL"),
				Owner->CurrentLane.HandleId, Owner->JnctState.ToLane.HandleId);
			Owner->AddLaneDecisionTrace(TEXT("TransitionCheck.TurnBypass"),
				NextLane.HandleId, 0.0f, 0.0f,
				TEXT("Cross-junction turn: bypassed junction lane"));
			}
			else
			{
				UE_LOG(LogAAATraffic, Warning,
					TEXT("JNCT TURN-BYPASS-INVALID: Pawn='%s' ToLane=%d has zero length — ignoring bypass"),
					Owner->GetPawn() ? *Owner->GetPawn()->GetName() : TEXT("NULL"),
					Owner->JnctState.ToLane.HandleId);
			}
		}
		else if (!bReachableViaDirectJunction)
		{
			UE_LOG(LogAAATraffic, Warning,
				TEXT("JNCT TURN-BYPASS-BLOCKED: Pawn='%s' JunctionId=%d Exit=%d requires occupancy before bypass transition"),
				Owner->GetPawn() ? *Owner->GetPawn()->GetName() : TEXT("NULL"),
				Owner->JnctState.JunctionId,
				Owner->JnctState.ToLane.HandleId);
		}
	}

	if (!bUsedPreselected)
	{
		if (bJunctionRouteAuthorityActive)
		{
			// Feeder lanes: vehicle has canonical authority but FromLane differs
			// from CurrentLane — still upstream and needs a normal transition.
			const bool bFeederLane = SelectedCanonicalMovement
				&& SelectedCanonicalMovement->IsValid()
				&& SelectedCanonicalMovement->FromLane.HandleId != Owner->CurrentLane.HandleId;
			if (bFeederLane)
			{
				UE_LOG(LogAAATraffic, Log,
					TEXT("JNCT FEEDER-PASSTHROUGH: Pawn='%s' Lane=%d FromLane=%d — allowing normal transition toward approach"),
					Owner->GetPawn() ? *Owner->GetPawn()->GetName() : TEXT("NULL"),
					Owner->CurrentLane.HandleId,
					SelectedCanonicalMovement->FromLane.HandleId);
			}
			else
			{
				// Canonical authority is active but no exit was preselected.
				// Fall through to PickSurveyedExit which will try the Rules
				// table as a fallback instead of blocking the transition.
				UE_LOG(LogAAATraffic, Warning,
					TEXT("JNCT ROUTE-FALLBACK: Pawn='%s' JunctionId=%d Lane=%d — no canonical route consumed, falling through to surveyor pick"),
					Owner->GetPawn() ? *Owner->GetPawn()->GetName() : TEXT("NULL"),
					Owner->JnctState.JunctionId,
					Owner->CurrentLane.HandleId);
			}
		}

		NextLane = Owner->PickSurveyedExit(Owner->CurrentLane, Connected);
		Owner->AddLaneDecisionTrace(TEXT("TransitionCheck.SurveyorPick"),
			NextLane.HandleId, 0.0f, 0.0f, TEXT("Fallback via surveyor"));

		// ── Exit-side lane matching ──
		if (Owner->CachedProvider)
		{
			const FVector ApproachRefDir = Owner->CachedProvider->GetLaneDirection(Owner->CurrentLane);
			int32 ApproachRightIdx = 0;
			{
				FTrafficLaneHandle Walk = Owner->CachedProvider->GetAdjacentLane(
					Owner->CurrentLane, ETrafficLaneSide::Right);
				for (int32 S = 0; S < 8 && Walk.IsValid(); ++S)
				{
					const FVector WalkDir = Owner->CachedProvider->GetLaneDirection(Walk);
					if (FVector::DotProduct(ApproachRefDir, WalkDir) < 0.5f) { break; }
					++ApproachRightIdx;
					Walk = Owner->CachedProvider->GetAdjacentLane(Walk, ETrafficLaneSide::Right);
				}
			}

			const FVector ExitRefDir = Owner->CachedProvider->GetLaneDirection(NextLane);
			FTrafficLaneHandle RightmostExit = NextLane;
			{
				FTrafficLaneHandle Walk = Owner->CachedProvider->GetAdjacentLane(
					NextLane, ETrafficLaneSide::Right);
				for (int32 S = 0; S < 8 && Walk.IsValid(); ++S)
				{
					const FVector WalkDir = Owner->CachedProvider->GetLaneDirection(Walk);
					if (FVector::DotProduct(ExitRefDir, WalkDir) < 0.5f) { break; }
					RightmostExit = Walk;
					Walk = Owner->CachedProvider->GetAdjacentLane(Walk, ETrafficLaneSide::Right);
				}
			}

			FTrafficLaneHandle MatchedLane = RightmostExit;
			{
				FTrafficLaneHandle Walk = RightmostExit;
				for (int32 i = 0; i < ApproachRightIdx; ++i)
				{
					const FTrafficLaneHandle Left = Owner->CachedProvider->GetAdjacentLane(
						Walk, ETrafficLaneSide::Left);
					if (!Left.IsValid()) { break; }
					const FVector LeftDir = Owner->CachedProvider->GetLaneDirection(Left);
					if (FVector::DotProduct(ExitRefDir, LeftDir) < 0.5f) { break; }
					Walk = Left;
					MatchedLane = Walk;
				}
			}

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
					Owner->AddLaneDecisionTrace(TEXT("TransitionCheck.ExitLaneMatched"),
						MatchedLane.HandleId,
						static_cast<float>(ApproachRightIdx), 0.0f,
						FString::Printf(TEXT("Overrode %d -> %d to match approach lateral pos"),
							NextLane.HandleId, MatchedLane.HandleId));
					NextLane = MatchedLane;
				}
			}
		}

		Owner->AddLaneDecisionTrace(
			TEXT("TransitionCheck.SelectedNextLane"),
			NextLane.HandleId, 0.0f, 0.0f,
			FString::Printf(TEXT("CandidateCount=%d"), Connected.Num()));
	} // end !bUsedPreselected

	UE_LOG(LogAAATraffic, Log,
		TEXT("TrafficVehicleController: Transitioning from lane %d to lane %d (%d candidates)."),
		Owner->CurrentLane.HandleId, NextLane.HandleId, Connected.Num());

	// Reset stuck timer — a viable transition was found.
	Owner->NoRouteStuckTimer = 0.0f;

	if (!NextLane.IsValid())
	{
		UE_LOG(LogAAATraffic, Error,
			TEXT("JNCT ROUTE-CONTRACT: Pawn='%s' lane %d produced no valid transition target"),
			Owner->GetPawn() ? *Owner->GetPawn()->GetName() : TEXT("NULL"),
			Owner->CurrentLane.HandleId);
		Owner->FlushLaneDecisionTrace(TEXT("InvalidTransitionTarget"), true);
		return;
	}

	const FCanonicalMovementRecord* CanonicalMovement = SelectedCanonicalMovement;
	if (TrafficSub && Owner->JnctState.JunctionId != 0)
	{
		const bool bRequiresCanonicalMovement = Owner->JnctState.bOwnsJunctionOccupancy || bCurrentLaneHasCanonicalApproach;
		if (bRequiresCanonicalMovement && (!CanonicalMovement || !CanonicalMovement->IsValid()))
		{
			UE_LOG(LogAAATraffic, Error,
				TEXT("JNCT CANONICAL-MISSING: Pawn='%s' JunctionId=%d transition to lane %d blocked because no canonical movement is selected"),
				Owner->GetPawn() ? *Owner->GetPawn()->GetName() : TEXT("NULL"),
				Owner->JnctState.JunctionId,
				NextLane.HandleId);
			Owner->FlushLaneDecisionTrace(TEXT("CanonicalMovementMissing"), true);
			return;
		}
	}

	// --- Junction smoothing ---
	FVector OldLaneEnd = FVector::ZeroVector;
	if (Owner->LanePoints.Num() >= 2)
	{
		OldLaneEnd = Owner->LanePoints.Last();
	}

	// ── Save old-lane tail for polyline continuity ──
	TArray<FVector> OldLaneTail;
	{
		const int32 OldIdx = FMath::Clamp(Owner->LastClosestIndex, 0,
			FMath::Max(Owner->LanePoints.Num() - 1, 0));
		OldLaneTail.Reserve(Owner->LanePoints.Num() - OldIdx);
		for (int32 k = OldIdx; k < Owner->LanePoints.Num(); ++k)
		{
			OldLaneTail.Add(Owner->LanePoints[k]);
		}
	}
	const float SavedHeadingCrossZ = Owner->SteeringComputer.PreviousHeadingCrossZ;

	if (GTrafficJunctionDiagnostics >= 1)
	{
		UE_LOG(LogAAATraffic, Log,
			TEXT("JNCT TRANSITION-PRE-INIT: Pawn='%s' OldLane=%d NextLane=%d "
				 "JnctState.JunctionId=%d Phase=%d bWaiting=%s JunctionTransPts=%d "
				 "— about to call InitializeLaneFollowing (Phase-aware)"),
			Owner->GetPawn() ? *Owner->GetPawn()->GetName() : TEXT("NULL"),
			Owner->CurrentLane.HandleId, NextLane.HandleId,
			Owner->JnctState.JunctionId,
			(int32)Owner->JnctState.Phase,
			Owner->JnctState.bWaiting ? TEXT("YES") : TEXT("NO"),
			Owner->JnctState.TransitionPoints.Num());
	}

	InitializeLane(NextLane);

	if (GTrafficJunctionDiagnostics >= 1)
	{
		UE_LOG(LogAAATraffic, Log,
			TEXT("JNCT TRANSITION-POST-INIT: Pawn='%s' NewLane=%d "
				 "JnctState.JunctionId=%d Phase=%d bWaiting=%s"),
			Owner->GetPawn() ? *Owner->GetPawn()->GetName() : TEXT("NULL"),
			NextLane.HandleId,
			Owner->JnctState.JunctionId,
			(int32)Owner->JnctState.Phase,
			Owner->JnctState.bWaiting ? TEXT("YES") : TEXT("NO"));
	}

	if (!Owner->bLaneDataReady)
	{
		Owner->bAtDeadEnd = true;
		Owner->SetTurnSignal(ETurnSignalState::Hazard);
		Owner->AddLaneDecisionTrace(TEXT("TransitionCheck.InitializeFailed"),
			NextLane.HandleId, 0.0f, 0.0f, TEXT("InitializeLaneFollowing failed"));
		Owner->FlushLaneDecisionTrace(TEXT("TransitionInitializeFailed"), true);
		UE_LOG(LogAAATraffic, Warning,
			TEXT("TrafficVehicleController: Failed to initialize lane following for lane %d from lane %d — treating as dead end."),
			NextLane.HandleId, Owner->CurrentLane.HandleId);
		return;
	}

	Owner->AddLaneDecisionTrace(TEXT("TransitionCheck.InitializeSucceeded"),
		NextLane.HandleId, static_cast<float>(Owner->LanePoints.Num()), 0.0f,
		TEXT("Lane initialized"));
	if (GTrafficVehicleTraceFlushOnSuccess)
	{
		Owner->FlushLaneDecisionTrace(TEXT("TransitionSuccess"), false);
	}

	// Generate junction transition points.
	Owner->JnctState.TransitionIndex = 0;
	Owner->JnctState.CurveStartIndex = 0;
	Owner->JnctState.TransitionReleaseIndex = INDEX_NONE;
	Owner->JnctState.ExitLaneResumeIndex = INDEX_NONE;
	Owner->JnctState.bTransitionPathFromProvider = false;
	if (CanonicalMovement && CanonicalMovement->IsValid())
	{
		Owner->JnctState.TransitionPoints = CanonicalMovement->CorridorPoints;
		Owner->JnctState.CurveStartIndex = FMath::Clamp(
			CanonicalMovement->EntryLaneAttachIndex,
			0,
			FMath::Max(CanonicalMovement->CorridorPoints.Num() - 1, 0));
		Owner->JnctState.TransitionReleaseIndex = FMath::Clamp(
			CanonicalMovement->TraversalReleaseIndex,
			0,
			FMath::Max(CanonicalMovement->CorridorPoints.Num() - 1, 0));
		Owner->JnctState.ExitLaneResumeIndex = FMath::Clamp(
			CanonicalMovement->ExitLaneResumeIndex,
			0,
			FMath::Max(CanonicalMovement->CorridorPoints.Num() - 1, 0));
		Owner->JnctState.bTransitionPathFromProvider = CanonicalMovement->SourceKind == ECanonicalMovementSourceKind::ProviderDerived;
		Owner->JnctState.TransitionIndex = Owner->JnctState.CurveStartIndex;

		// Snap TransitionIndex to the nearest curve point so that
		// TURN-BYPASS re-entries (where the vehicle is past the original
		// CurveStartIndex) don't stall the advancement loop.
		if (const APawn* P = Owner->GetPawn())
		{
			const FVector Loc = P->GetActorLocation();
			const int32 Num = Owner->JnctState.TransitionPoints.Num();
			int32 BestIdx = Owner->JnctState.CurveStartIndex;
			float BestDistSq = FVector::DistSquared(Loc, Owner->JnctState.TransitionPoints[BestIdx]);
			for (int32 Idx = BestIdx + 1; Idx < Num; ++Idx)
			{
				const float DSq = FVector::DistSquared(Loc, Owner->JnctState.TransitionPoints[Idx]);
				if (DSq < BestDistSq)
				{
					BestDistSq = DSq;
					BestIdx = Idx;
				}
			}
			Owner->JnctState.TransitionIndex = BestIdx;
		}
	}

	// ── Entry validation ─────────────────────────────────────
	// Reject junction curve entry if the vehicle is backward (heading
	// >90° from curve tangent) OR too far from the curve (snap
	// distance > 600 cm).  Either case means the vehicle cannot
	// follow the curve and would cause chaotic collisions.
	if (Owner->JnctState.TransitionPoints.Num() >= 2)
	{
		const int32 SnapIdx = Owner->JnctState.TransitionIndex;
		if (SnapIdx < Owner->JnctState.TransitionPoints.Num() - 1)
		{
			const FVector CurveTangent = (Owner->JnctState.TransitionPoints[SnapIdx + 1]
				- Owner->JnctState.TransitionPoints[SnapIdx]).GetSafeNormal2D();
			const FVector VehicleFwd = Owner->GetPawn()->GetActorForwardVector();
			const float HeadingDot = FVector::DotProduct(
				FVector(VehicleFwd.X, VehicleFwd.Y, 0.0f).GetSafeNormal(), CurveTangent);
			const float SnapDist = FVector::Dist(
				Owner->GetPawn()->GetActorLocation(),
				Owner->JnctState.TransitionPoints[SnapIdx]);

			constexpr float MaxSnapDistCm = 600.0f;
			if (HeadingDot < 0.0f || SnapDist > MaxSnapDistCm)
			{
				UE_LOG(LogAAATraffic, Warning,
					TEXT("Junction curve entry aborted — heading dot %.2f, snap dist %.0f cm. Despawning. Pawn=%s Jnct=%d"),
					HeadingDot, SnapDist,
					Owner->GetPawn() ? *Owner->GetPawn()->GetName() : TEXT("null"),
					Owner->JnctState.JunctionId);

				Owner->bPendingRecoveryDespawn = true;
				if (UWorld* W = Owner->GetWorld())
				{
					if (UTrafficSubsystem* Sub = W->GetSubsystem<UTrafficSubsystem>())
					{
						Sub->RequestDespawn(Owner,
							FString::Printf(TEXT("bad junction curve entry (dot=%.2f dist=%.0f)"),
								HeadingDot, SnapDist));
					}
				}
				return;
			}
		}
	}

	if (Owner->JnctState.TransitionPoints.Num() >= 3)
	{
		const int32 CurveStart = Owner->JnctState.CurveStartIndex;
		float TotalAngleDeg = 0.0f;
		float TotalArcLength = 0.0f;
		float MinTurnRadius = TNumericLimits<float>::Max();
		for (int32 i = CurveStart + 1; i < Owner->JnctState.TransitionPoints.Num() - 1; ++i)
		{
			const FVector Seg0 = Owner->JnctState.TransitionPoints[i] - Owner->JnctState.TransitionPoints[i - 1];
			const FVector Seg1 = Owner->JnctState.TransitionPoints[i + 1] - Owner->JnctState.TransitionPoints[i];
			TotalArcLength += Seg0.Size();
			const float CrossMag = FMath::Abs(FVector::CrossProduct(Seg0, Seg1).Z);
			if (CrossMag > KINDA_SMALL_NUMBER)
			{
				const float ChordLength = FVector::Dist(
					Owner->JnctState.TransitionPoints[i - 1],
					Owner->JnctState.TransitionPoints[i + 1]);
				const float Numerator = Seg0.Size() * Seg1.Size() * ChordLength;
				if (Numerator > KINDA_SMALL_NUMBER)
				{
					const float RadiusCm = Numerator / (2.0f * CrossMag);
					if (RadiusCm > 0.0f)
					{
						MinTurnRadius = FMath::Min(MinTurnRadius, RadiusCm);
					}
				}
			}
			const FVector Dir0 = Seg0.GetSafeNormal();
			const FVector Dir1 = Seg1.GetSafeNormal();
			if (!Dir0.IsNearlyZero() && !Dir1.IsNearlyZero())
			{
				const float Dot = FMath::Clamp(FVector::DotProduct(Dir0, Dir1), -1.0f, 1.0f);
				TotalAngleDeg += FMath::RadiansToDegrees(FMath::Acos(Dot));
			}
		}
		TotalArcLength += (Owner->JnctState.TransitionPoints.Last() - Owner->JnctState.TransitionPoints[Owner->JnctState.TransitionPoints.Num() - 2]).Size();
		const float TotalAngleRad = FMath::DegreesToRadians(FMath::Max(TotalAngleDeg, 1.0f));
		const float AvgTurnRadius = FMath::Max(TotalArcLength / TotalAngleRad, 50.0f);
		const float TurnRadius = MinTurnRadius == TNumericLimits<float>::Max()
			? AvgTurnRadius
			: MinTurnRadius;

		UE_LOG(LogAAATraffic, Warning,
			TEXT("JNCT PATH-PROFILE: Pawn='%s' JunctionId=%d Path=%s From=%d To=%d Pts=%d CurveStart=%d Arc=%.0f Angle=%.1f AvgR=%.0f MinR=%.0f"),
			Owner->GetPawn() ? *Owner->GetPawn()->GetName() : TEXT("NULL"),
			Owner->JnctState.JunctionId,
			Owner->JnctState.bTransitionPathFromProvider ? TEXT("PROVIDER") : TEXT("SYNTH"),
			Owner->CurrentLane.HandleId,
			NextLane.HandleId,
			Owner->JnctState.TransitionPoints.Num(),
			Owner->JnctState.CurveStartIndex,
			TotalArcLength,
			TotalAngleDeg,
			AvgTurnRadius,
			TurnRadius);
	}

	// Transition to Traversing when junction curves are committed.
	if (Owner->JnctState.TransitionPoints.Num() > 0
		&& (Owner->JnctState.Phase == EJunctionPhase::Approaching
			|| Owner->JnctState.Phase == EJunctionPhase::Waiting))
	{
		ensureAlwaysMsgf(
			Owner->JnctState.ToLane.IsValid(),
			TEXT("JNCT CONTRACT VIOLATION [TransitionCommit.ToLane]: Pawn='%s' JunctionId=%d committed transition path without a valid exit lane"),
			Owner->GetPawn() ? *Owner->GetPawn()->GetName() : TEXT("NULL"),
			Owner->JnctState.JunctionId);
		ensureAlwaysMsgf(
			Owner->JnctState.TransitionPoints.Num() >= 2,
			TEXT("JNCT CONTRACT VIOLATION [TransitionCommit.Path]: Pawn='%s' JunctionId=%d committed transition path with %d points"),
			Owner->GetPawn() ? *Owner->GetPawn()->GetName() : TEXT("NULL"),
			Owner->JnctState.JunctionId,
			Owner->JnctState.TransitionPoints.Num());

		// Degenerate corridor handling: if the junction curve is very short
		// (< 50cm arc) or has too few points, the curve may commit before
		// the negotiator grants occupancy. Skip the junction curve for these
		// degenerate straight-through paths and treat as a direct lane change.
		const float CurveArc = [&]()
		{
			float Arc = 0.0f;
			for (int32 Idx = 0; Idx + 1 < Owner->JnctState.TransitionPoints.Num(); ++Idx)
			{
				Arc += FVector::Dist(Owner->JnctState.TransitionPoints[Idx], Owner->JnctState.TransitionPoints[Idx + 1]);
			}
			return Arc;
		}();
		if (CurveArc < 50.0f && !Owner->JnctState.bOwnsJunctionOccupancy)
		{
			UE_LOG(LogAAATraffic, Log,
				TEXT("JNCT DEGENERATE-BYPASS: Pawn='%s' JunctionId=%d Arc=%.0f — "
					 "skipping degenerate junction curve, treating as direct lane transition"),
				Owner->GetPawn() ? *Owner->GetPawn()->GetName() : TEXT("NULL"),
				Owner->JnctState.JunctionId, CurveArc);
			Owner->JnctState.TransitionPoints.Empty();
			Owner->JnctState.TransitionIndex = 0;
			Owner->JnctState.CurveStartIndex = 0;
			// Don't block — let the vehicle proceed directly onto the exit lane
		}
		else if (Owner->JnctState.Phase == EJunctionPhase::Approaching && !Owner->JnctState.bOwnsJunctionOccupancy)
		{
			// Vehicle reached the junction curve entry without occupancy.
			// Attempt late-acquire before blocking.
			bool bLateAcquired = false;
			if (TrafficSub && Owner->JnctState.JunctionId > 0)
			{
				if (TrafficSub->TryOccupyJunction(Owner->JnctState.JunctionId, Owner, Owner->JnctState.CanonicalMovementId))
				{
					Owner->JnctState.bOwnsJunctionOccupancy = true;
					bLateAcquired = true;
					// Transition to Traversing immediately so occupancy token
					// and phase stay in sync — prevents contract violation
					// where subsystem evicts a stale Approaching-phase occupant.
					if (Owner->JnctState.TransitionPoints.Num() > 0)
					{
						Owner->JnctState.BeginTraversing();
					}
					UE_LOG(LogAAATraffic, Warning,
						TEXT("JNCT TRAVERSE-LATE-ACQUIRE: Pawn='%s' JunctionId=%d — late occupancy granted, proceeding"),
						Owner->GetPawn() ? *Owner->GetPawn()->GetName() : TEXT("NULL"),
						Owner->JnctState.JunctionId);
				}
			}
			if (!bLateAcquired)
			{
				UE_LOG(LogAAATraffic, Warning,
					TEXT("JNCT TRAVERSE-NO-OCCUPANCY: Pawn='%s' JunctionId=%d entering bypass curve without occupancy token"),
					Owner->GetPawn() ? *Owner->GetPawn()->GetName() : TEXT("NULL"),
					Owner->JnctState.JunctionId);
				Owner->JnctState.TransitionPoints.Empty();
				Owner->JnctState.TransitionIndex = 0;
				Owner->JnctState.CurveStartIndex = 0;
				Owner->FlushLaneDecisionTrace(TEXT("TransitionBlockedNoOccupancy"), true);
				return;
			}
		}
		else if (Owner->JnctState.TransitionPoints.Num() > 0)
		{
			Owner->JnctState.BeginTraversing();
		}
	}

	// ── Non-junction boundary: prepend old-lane tail ──
	if (Owner->JnctState.TransitionPoints.Num() == 0
		&& OldLaneTail.Num() > 1 && Owner->LanePoints.Num() > 0)
	{
		if (FVector::Dist2D(OldLaneTail.Last(), Owner->LanePoints[0]) < 150.0f)
		{
			OldLaneTail.Pop();
		}
		if (OldLaneTail.Num() > 0)
		{
			TArray<FVector> Combined;
			Combined.Reserve(OldLaneTail.Num() + Owner->LanePoints.Num());
			Combined.Append(OldLaneTail);
			Combined.Append(Owner->LanePoints);
			Owner->LanePoints = MoveTemp(Combined);
			Owner->LastClosestIndex = 0;
			Owner->SteeringComputer.PreviousHeadingCrossZ = SavedHeadingCrossZ;

			UE_LOG(LogAAATraffic, Log,
				TEXT("LANE-PREPEND: Pawn='%s' prepended %d old-tail points to "
					 "new lane %d (%d total points) — continuous polyline "
					 "from vehicle position through lane boundary."),
				Owner->GetPawn() ? *Owner->GetPawn()->GetName() : TEXT("NULL"),
				OldLaneTail.Num(),
				Owner->CurrentLane.HandleId,
				Owner->LanePoints.Num());
		}
	}
}
