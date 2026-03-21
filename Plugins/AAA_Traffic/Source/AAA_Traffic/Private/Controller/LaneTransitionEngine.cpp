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

			if (Owner->JnctState.Phase == EJunctionPhase::Waiting && Owner->JnctState.JunctionId != 0)
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
	if (Provider->GetLanePath(Owner->CurrentLane, Owner->LanePoints, Owner->LaneWidth) && Owner->LanePoints.Num() >= 2)
	{
		Owner->bLaneDataReady = true;

		// --- JUNCTION DIAGNOSTIC ---
		{
			const float TotalLen = Provider->GetLaneLength(Owner->CurrentLane);
			const ITrafficRoadProvider::FJunctionScanResult InitScan =
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
	if (Connected.Num() > 1)
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

		if (!bReachableViaDirectJunction)
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
	}

	if (!bUsedPreselected)
	{
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

	// --- Junction smoothing ---
	FVector OldLaneEnd = FVector::ZeroVector;
	FVector OldLaneTangent = FVector::ZeroVector;
	if (Owner->LanePoints.Num() >= 2)
	{
		OldLaneEnd = Owner->LanePoints.Last();
		const int32 NumAvgOld = FMath::Min(3, Owner->LanePoints.Num() - 1);
		FVector AvgOldDir = FVector::ZeroVector;
		for (int32 k = Owner->LanePoints.Num() - NumAvgOld; k < Owner->LanePoints.Num(); ++k)
		{
			AvgOldDir += (Owner->LanePoints[k] - Owner->LanePoints[k - 1]);
		}
		OldLaneTangent = AvgOldDir.GetSafeNormal();
	}

	TArray<FVector> ProviderJunctionPath;
	const bool bHasProviderPath = Owner->CachedProvider
		&& Owner->CachedProvider->GetJunctionPath(Owner->CurrentLane, NextLane, ProviderJunctionPath)
		&& ProviderJunctionPath.Num() >= 2;

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
	if (bHasProviderPath)
	{
		Owner->JnctState.TransitionPoints = MoveTemp(ProviderJunctionPath);
	}
	else if (!OldLaneTangent.IsNearlyZero() && Owner->LanePoints.Num() >= 2)
	{
		const FVector NewLaneStart = Owner->LanePoints[0];
		const int32 NumAvgNew = FMath::Min(3, Owner->LanePoints.Num() - 1);
		FVector AvgNewDir = FVector::ZeroVector;
		for (int32 k = 0; k < NumAvgNew; ++k)
		{
			AvgNewDir += (Owner->LanePoints[k + 1] - Owner->LanePoints[k]);
		}
		const FVector NewLaneTangent = AvgNewDir.GetSafeNormal();
		const float SpanDist = FVector::Dist(OldLaneEnd, NewLaneStart);

		if (SpanDist > 50.0f)
		{
			const float DotFactor = FVector::DotProduct(OldLaneTangent, NewLaneTangent);
			const float Alpha = FMath::Lerp(0.25f, 0.5f,
				FMath::Clamp((DotFactor + 1.0f) * 0.5f, 0.0f, 1.0f));
			const float TangentScale = SpanDist * Alpha;
			const FVector P0 = OldLaneEnd;
			const FVector P1 = NewLaneStart;
			const FVector M0 = OldLaneTangent * TangentScale;
			const FVector M1 = NewLaneTangent * TangentScale;

			const int32 NumSegments = FMath::Clamp(
				FMath::CeilToInt32(SpanDist / Owner->JunctionCurveResolutionCm), 6, 32);
			Owner->JnctState.TransitionPoints.Reserve(NumSegments + 1);
			for (int32 i = 0; i <= NumSegments; ++i)
			{
				const float T = static_cast<float>(i) / static_cast<float>(NumSegments);
				const float T2 = T * T;
				const float T3 = T2 * T;
				const float H00 = 2.0f * T3 - 3.0f * T2 + 1.0f;
				const float H10 = T3 - 2.0f * T2 + T;
				const float H01 = -2.0f * T3 + 3.0f * T2;
				const float H11 = T3 - T2;
				Owner->JnctState.TransitionPoints.Add(H00 * P0 + H10 * M0 + H01 * P1 + H11 * M1);
			}
		}
	}

	// ── Junction curve: prepend old-lane tail ──
	if (Owner->JnctState.TransitionPoints.Num() > 0 && OldLaneTail.Num() > 1)
	{
		if (FVector::Dist(OldLaneTail.Last(), Owner->JnctState.TransitionPoints[0]) < 150.0f)
		{
			OldLaneTail.Pop();
		}
		if (OldLaneTail.Num() > 0)
		{
			TArray<FVector> Combined;
			Combined.Reserve(OldLaneTail.Num() + Owner->JnctState.TransitionPoints.Num());
			Combined.Append(OldLaneTail);
			Combined.Append(Owner->JnctState.TransitionPoints);
			Owner->JnctState.CurveStartIndex = OldLaneTail.Num();
			Owner->JnctState.TransitionPoints = MoveTemp(Combined);

			UE_LOG(LogAAATraffic, Log,
				TEXT("JNCT CURVE-PREPEND: Pawn='%s' prepended %d old-tail points "
					 "to junction curve (%d total, CurveStartIdx=%d)."),
				Owner->GetPawn() ? *Owner->GetPawn()->GetName() : TEXT("NULL"),
				OldLaneTail.Num(),
				Owner->JnctState.TransitionPoints.Num(),
				Owner->JnctState.CurveStartIndex);
		}
	}

	// Transition to Traversing when junction curves are committed.
	if (Owner->JnctState.TransitionPoints.Num() > 0
		&& (Owner->JnctState.Phase == EJunctionPhase::Approaching
			|| Owner->JnctState.Phase == EJunctionPhase::Waiting))
	{
		Owner->JnctState.BeginTraversing();
	}

	// ── Non-junction boundary: prepend old-lane tail ──
	if (Owner->JnctState.TransitionPoints.Num() == 0
		&& OldLaneTail.Num() > 1 && Owner->LanePoints.Num() > 0)
	{
		if (FVector::Dist(OldLaneTail.Last(), Owner->LanePoints[0]) < 150.0f)
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
