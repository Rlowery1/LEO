// Copyright AAA_Traffic Contributors. All Rights Reserved.

#include "LaneChangeCoordinator.h"
#include "TrafficVehicleController.h"
#include "TrafficSubsystem.h"
#include "SteeringComputer.h"
#include "TrafficLog.h"
#include "ChaosWheeledVehicleMovementComponent.h"
#include "GameFramework/Pawn.h"

// Access the file-static CVar via extern — defined in TrafficVehicleController.cpp.
extern int32 GTrafficVehicleDecisionTrace;
extern bool  GTrafficVehicleTraceFlushOnSuccess;

namespace
{
	static bool IsTraceEnabled(int32 RequiredLevel)
	{
		return GTrafficVehicleDecisionTrace >= RequiredLevel;
	}
}

// ---------------------------------------------------------------------------
// Constructor
// ---------------------------------------------------------------------------

FLaneChangeCoordinator::FLaneChangeCoordinator()
	: State(ELaneChangeState::None)
{
}

// ---------------------------------------------------------------------------
// SetAggression
// ---------------------------------------------------------------------------

void FLaneChangeCoordinator::SetAggression(float Aggression)
{
	Aggression = FMath::Clamp(Aggression, 0.0f, 1.0f);

	Distance       = FMath::Lerp(2500.0f, 800.0f,  Aggression);
	CooldownTime   = FMath::Lerp(8.0f,    2.0f,    Aggression);
	SpeedThreshold = FMath::Lerp(0.5f,     0.85f,   Aggression);
	GapRequired    = FMath::Lerp(1200.0f,  500.0f,  Aggression);
}

// ---------------------------------------------------------------------------
// TickCooldown / ShouldEvaluate
// ---------------------------------------------------------------------------

void FLaneChangeCoordinator::TickCooldown(float DeltaSeconds)
{
	if (CooldownRemaining > 0.0f)
	{
		CooldownRemaining -= DeltaSeconds;
	}
}

bool FLaneChangeCoordinator::ShouldEvaluate() const
{
	return State == ELaneChangeState::None && CooldownRemaining <= 0.0f;
}

// ---------------------------------------------------------------------------
// Evaluate
// ---------------------------------------------------------------------------

FLaneChangeEvalResult FLaneChangeCoordinator::Evaluate(const FLaneChangeEvalContext& Ctx)
{
	FLaneChangeEvalResult Result;
	Result.bStarted    = false;
	Result.TurnSignal  = ETurnSignalState::Off;

	if (!Ctx.RoadProvider || !Ctx.bLaneDataReady) { return Result; }
	if (Owner)
	{
		Owner->AddLaneDecisionTrace(TEXT("LaneChange.EvaluateStart"), 0,
			static_cast<float>(Ctx.CurrentLane.HandleId), 0.0f, TEXT("Lane-change evaluation"));
	}

	const float EffectiveTarget = FMath::Max(Ctx.TargetSpeed, 1.0f);

	// Suppress non-navigational lane changes when approaching a junction.
	if (Ctx.bJunctionApproaching && !bNavigationalLaneChange)
	{
		if (IsTraceEnabled(2) && Owner)
		{
			Owner->AddLaneDecisionTrace(TEXT("LaneChange.RejectApproaching"), 0,
				0.0f, 0.0f, TEXT("Approaching intersection — suppressed"));
		}
		return Result;
	}

	// --- Navigational pre-positioning ---
	bool bNavigationalAttempt = false;
	ETrafficLaneSide NavigationalSide = ETrafficLaneSide::Left;
	if (bNavigationalLaneChange && NavigationalTargetLane.IsValid()
		&& NavigationalTargetLane != Ctx.CurrentLane)
	{
		bNavigationalAttempt = true;
		bool bFoundLeft = false;
		FTrafficLaneHandle Walk = Ctx.RoadProvider->GetAdjacentLane(
			Ctx.CurrentLane, ETrafficLaneSide::Left);
		for (int32 Safety = 0; Safety < 8 && Walk.IsValid(); ++Safety)
		{
			if (Walk == NavigationalTargetLane) { bFoundLeft = true; break; }
			Walk = Ctx.RoadProvider->GetAdjacentLane(Walk, ETrafficLaneSide::Left);
		}
		NavigationalSide = bFoundLeft ? ETrafficLaneSide::Left : ETrafficLaneSide::Right;
		if (Owner)
		{
			Owner->AddLaneDecisionTrace(TEXT("LaneChange.NavPrePos"),
				NavigationalTargetLane.HandleId,
				bFoundLeft ? -1.0f : 1.0f, 0.0f,
				TEXT("Navigational pre-positioning attempt"));
		}
	}
	else if (bNavigationalLaneChange)
	{
		bNavigationalLaneChange = false;
		NavigationalTargetLane  = FTrafficLaneHandle();
	}

	float LeaderDist = -1.0f;
	if (!bNavigationalAttempt)
	{
		if (Ctx.CurrentSpeed / EffectiveTarget >= SpeedThreshold)
		{
			if (IsTraceEnabled(2) && Owner)
			{
				Owner->AddLaneDecisionTrace(TEXT("LaneChange.RejectSpeedRatio"), 0,
					Ctx.CurrentSpeed / EffectiveTarget, SpeedThreshold,
					TEXT("Not slow enough"));
			}
			return Result;
		}

		LeaderDist = Ctx.LeaderDist;
		if (LeaderDist < 0.0f || LeaderDist > Ctx.DetectionDistance * 0.75f)
		{
			if (IsTraceEnabled(2) && Owner)
			{
				Owner->AddLaneDecisionTrace(TEXT("LaneChange.RejectLeaderDistance"), 0,
					LeaderDist, Ctx.DetectionDistance * 0.75f,
					TEXT("No close slow leader"));
			}
			return Result;
		}
	}

	// Suppress on tight curves (κ > 0.002 ≈ R < 5m).
	if (Ctx.LanePoints && Ctx.LanePoints->Num() >= 3)
	{
		const float CurCurvature = FMath::Abs(
			FSteeringComputer::ComputeLocalCurvature(*Ctx.LanePoints, Ctx.LastClosestIndex));
		if (CurCurvature > 0.002f)
		{
			if (IsTraceEnabled(2) && Owner)
			{
				Owner->AddLaneDecisionTrace(TEXT("LaneChange.RejectCurvature"), 0,
					CurCurvature, 0.002f, TEXT("On tight curve"));
			}
			return Result;
		}
	}

	const FVector MyDirection = Ctx.RoadProvider->GetLaneDirection(Ctx.CurrentLane);

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
		FTrafficLaneHandle CandidateLane = Ctx.RoadProvider->GetAdjacentLane(Ctx.CurrentLane, Side);
		if (!CandidateLane.IsValid())
		{
			if (IsTraceEnabled(2) && Owner)
			{
				Owner->AddLaneDecisionTrace(TEXT("LaneChange.RejectNoAdjacent"), 0,
					Side == ETrafficLaneSide::Left ? -1.0f : 1.0f, 0.0f,
					TEXT("No adjacent lane"));
			}
			continue;
		}

		// Direction safety: same-direction only.
		const FVector CandidateDir = Ctx.RoadProvider->GetLaneDirection(CandidateLane);
		const float DirDot = FVector::DotProduct(MyDirection, CandidateDir);
		if (DirDot <= 0.0f)
		{
			if (IsTraceEnabled(2) && Owner)
			{
				Owner->AddLaneDecisionTrace(TEXT("LaneChange.RejectDirection"),
					CandidateLane.HandleId, DirDot, 0.0f,
					TEXT("Opposite direction lane"));
			}
			continue;
		}

		// Lane type safety: don't change into non-drivable lanes.
		{
			const TArray<FTrafficLaneSection> CandidateSections =
				Ctx.RoadProvider->GetLaneSections(CandidateLane);
			if (!CandidateSections.IsEmpty())
			{
				const ETrafficLaneType CandidateType = CandidateSections[0].Type;
				if (CandidateType != ETrafficLaneType::Normal
					&& CandidateType != ETrafficLaneType::CenterTurn)
				{
					if (IsTraceEnabled(2) && Owner)
					{
						Owner->AddLaneDecisionTrace(TEXT("LaneChange.RejectLaneType"),
							CandidateLane.HandleId,
							static_cast<float>(CandidateType), 0.0f,
							TEXT("Non-drivable lane type"));
					}
					continue;
				}
			}
		}

		// Gap check via per-lane registry.
		if (!Ctx.TrafficSub) { continue; }

		const float EffectiveGapRequired = FMath::Max(GapRequired,
			Ctx.VehicleFrontExtent + Ctx.VehicleRearExtent + 100.0f);
		bool bGapClear = true;

		TArray<TWeakObjectPtr<ATrafficVehicleController>> Neighbors =
			Ctx.TrafficSub->GetVehiclesOnLane(CandidateLane);
		for (const TWeakObjectPtr<ATrafficVehicleController>& WeakNeighbor : Neighbors)
		{
			ATrafficVehicleController* Neighbor = WeakNeighbor.Get();
			if (!Neighbor || Neighbor == Owner) { continue; }

			const APawn* NeighborPawn = Neighbor->GetPawn();
			if (!NeighborPawn) { continue; }

			const float DistToNeighbor = FVector::Dist(
				Ctx.VehicleLocation, NeighborPawn->GetActorLocation());
			if (DistToNeighbor < EffectiveGapRequired)
			{
				bGapClear = false;
				if (IsTraceEnabled(2) && Owner)
				{
					Owner->AddLaneDecisionTrace(TEXT("LaneChange.RejectGap"),
						CandidateLane.HandleId, DistToNeighbor, GapRequired,
						TEXT("Neighbor too close"));
				}
				break;
			}
		}

		// Blind-spot check: vehicles mid-lane-change INTO candidate lane.
		if (bGapClear)
		{
			for (const TWeakObjectPtr<ATrafficVehicleController>& WeakOther
				: Ctx.TrafficSub->GetActiveVehicles())
			{
				ATrafficVehicleController* Other = WeakOther.Get();
				if (!Other || Other == Owner) { continue; }
				if (Other->LaneChangeCoord_.State != ELaneChangeState::Executing
					&& Other->LaneChangeCoord_.State != ELaneChangeState::Completing)
				{
					continue;
				}
				if (Other->LaneChangeCoord_.TargetLane != CandidateLane) { continue; }

				const APawn* OtherPawn = Other->GetPawn();
				if (!OtherPawn) { continue; }

				const float DistToOther = FVector::Dist(
					Ctx.VehicleLocation, OtherPawn->GetActorLocation());
				if (DistToOther < GapRequired)
				{
					bGapClear = false;
					if (IsTraceEnabled(2) && Owner)
					{
						Owner->AddLaneDecisionTrace(TEXT("LaneChange.RejectBlindSpot"),
							CandidateLane.HandleId, DistToOther, GapRequired,
							TEXT("Vehicle changing into target lane"));
					}
					break;
				}
			}
		}

		// Lateral closing-speed check.
		if (bGapClear)
		{
			for (const TWeakObjectPtr<ATrafficVehicleController>& WeakNeighbor : Neighbors)
			{
				ATrafficVehicleController* Neighbor = WeakNeighbor.Get();
				if (!Neighbor || Neighbor == Owner) { continue; }

				const APawn* NeighborPawn = Neighbor->GetPawn();
				if (!NeighborPawn) { continue; }

				const FVector ToNeighbor =
					NeighborPawn->GetActorLocation() - Ctx.VehicleLocation;
				const float DotAlong = FVector::DotProduct(ToNeighbor, Ctx.EgoForward);

				if (DotAlong >= 0.0f || FMath::Abs(DotAlong) > GapRequired * 2.0f)
				{
					continue;
				}

				const UChaosWheeledVehicleMovementComponent* NeighborMC =
					Cast<UChaosWheeledVehicleMovementComponent>(
						NeighborPawn->GetMovementComponent());
				if (!NeighborMC) { continue; }

				const float NeighborSpeed = FMath::Abs(NeighborMC->GetForwardSpeed());
				const float ClosingSpeed  = NeighborSpeed - Ctx.EgoForwardSpeed;

				if (ClosingSpeed > 500.0f)
				{
					bGapClear = false;
					if (IsTraceEnabled(2) && Owner)
					{
						Owner->AddLaneDecisionTrace(TEXT("LaneChange.RejectClosingSpeed"),
							CandidateLane.HandleId, ClosingSpeed, 500.0f,
							TEXT("Fast approaching vehicle behind"));
					}
					break;
				}
			}
		}

		if (!bGapClear) { continue; }

		// Candidate lane valid — get path and begin.
		TArray<FVector> CandidatePoints;
		float CandidateWidth;
		if (!Ctx.RoadProvider->GetLanePath(CandidateLane, CandidatePoints, CandidateWidth)
			|| CandidatePoints.Num() < 2)
		{
			if (IsTraceEnabled(2) && Owner)
			{
				Owner->AddLaneDecisionTrace(TEXT("LaneChange.RejectPath"),
					CandidateLane.HandleId,
					static_cast<float>(CandidatePoints.Num()), 2.0f,
					TEXT("Target lane path unavailable"));
			}
			continue;
		}

		State           = ELaneChangeState::Executing;
		TargetLane      = CandidateLane;
		TargetLanePoints = MoveTemp(CandidatePoints);
		TargetLaneWidth = CandidateWidth;
		Progress        = 0.0f;

		Result.bStarted   = true;
		Result.TurnSignal = (Side == ETrafficLaneSide::Left)
			? ETurnSignalState::Left : ETurnSignalState::Right;

		UE_LOG(LogAAATraffic, Log,
			TEXT("TrafficVehicleController: Beginning lane change from lane %d to lane %d (%s)."),
			Ctx.CurrentLane.HandleId, CandidateLane.HandleId,
			Side == ETrafficLaneSide::Left ? TEXT("left") : TEXT("right"));
		if (Owner)
		{
			Owner->AddLaneDecisionTrace(TEXT("LaneChange.Begin"),
				CandidateLane.HandleId, LeaderDist, Ctx.CurrentSpeed,
				Side == ETrafficLaneSide::Left ? TEXT("Left") : TEXT("Right"));
		}
		return Result;
	}

	if (IsTraceEnabled(2) && Owner)
	{
		Owner->AddLaneDecisionTrace(TEXT("LaneChange.NoCandidateAccepted"), 0,
			LeaderDist, Ctx.CurrentSpeed, TEXT("All adjacent lanes rejected"));
	}
	return Result;
}

// ---------------------------------------------------------------------------
// Blend
// ---------------------------------------------------------------------------

FLaneChangeBlendResult FLaneChangeCoordinator::Blend(const FLaneChangeBlendContext& Ctx)
{
	FLaneChangeBlendResult Result;

	// --- Continuous merge safety check ---
	if (State == ELaneChangeState::Executing && Progress < 0.8f && Ctx.TrafficSub)
	{
		const float AbortGapThreshold = GapRequired * 0.7f;
		TArray<TWeakObjectPtr<ATrafficVehicleController>> Neighbors =
			Ctx.TrafficSub->GetVehiclesOnLane(TargetLane);
		for (const TWeakObjectPtr<ATrafficVehicleController>& WeakNeighbor : Neighbors)
		{
			ATrafficVehicleController* Neighbor = WeakNeighbor.Get();
			if (!Neighbor || Neighbor == Owner) { continue; }
			const APawn* NeighborPawn = Neighbor->GetPawn();
			if (!NeighborPawn) { continue; }
			if (FVector::Dist(Ctx.VehicleLocation, NeighborPawn->GetActorLocation())
				< AbortGapThreshold)
			{
				if (Owner)
				{
					Owner->AddLaneDecisionTrace(TEXT("LaneChange.AbortUnsafeGap"),
						TargetLane.HandleId,
						FVector::Dist(Ctx.VehicleLocation, NeighborPawn->GetActorLocation()),
						AbortGapThreshold,
						TEXT("Continuous merge check failed"));
				}
				Abort();
				Result.Point    = Ctx.SourcePoint;
				Result.bAborted = true;
				return Result;
			}
		}
	}

	// Advance progress.
	{
		const float ProgressDelta = Ctx.DistanceThisTick / FMath::Max(Distance, 1.0f);
		Progress = FMath::Min(Progress + ProgressDelta, 1.0f);
	}

	const FVector SourcePoint = Ctx.SourcePoint;

	// Find closest index on target lane.
	int32 TargetClosestIndex = 0;
	float BestDistSq = MAX_flt;
	for (int32 i = 0; i < TargetLanePoints.Num(); ++i)
	{
		const float DSq = FVector::DistSquared(Ctx.VehicleLocation, TargetLanePoints[i]);
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
			const float SegDist = FVector::Dist2D(
				TargetLanePoints[Idx], TargetLanePoints[Idx + 1]);
			AccDist += SegDist;
			if (AccDist >= Ctx.LookAheadDistance)
			{
				const float Overshoot = AccDist - Ctx.LookAheadDistance;
				const float Alpha = 1.0f - (Overshoot / FMath::Max(SegDist, KINDA_SMALL_NUMBER));
				TargetPoint = FMath::Lerp(TargetLanePoints[Idx], TargetLanePoints[Idx + 1], Alpha);
				break;
			}
			++Idx;
		}
	}

	// Smooth blend with ease-in-out.
	const float BlendAlpha = FMath::SmoothStep(0.0f, 1.0f, Progress);
	Result.Point = FMath::Lerp(SourcePoint, TargetPoint, BlendAlpha);

	// Check for completion — enter settling phase.
	if (Progress >= 1.0f && State == ELaneChangeState::Executing)
	{
		State       = ELaneChangeState::Completing;
		SettleTimer = 0.5f;
		UE_LOG(LogAAATraffic, Verbose,
			TEXT("TrafficVehicleController: Lane change blend complete, entering settling phase on lane %d."),
			TargetLane.HandleId);
	}

	return Result;
}

// ---------------------------------------------------------------------------
// TickSettle
// ---------------------------------------------------------------------------

bool FLaneChangeCoordinator::TickSettle(float DeltaSeconds)
{
	SettleTimer -= DeltaSeconds;
	return SettleTimer <= 0.0f;
}

// ---------------------------------------------------------------------------
// Finalize
// ---------------------------------------------------------------------------

FLaneChangeFinalizeResult FLaneChangeCoordinator::Finalize(
	ITrafficRoadProvider* Provider,
	float DefaultSpeedLimit,
	float BaseTargetSpeed)
{
	if (Owner)
	{
		Owner->AddLaneDecisionTrace(TEXT("LaneChange.Finalize"),
			TargetLane.HandleId, Progress, CooldownTime,
			TEXT("Lane change finalized"));
	}
	UE_LOG(LogAAATraffic, Log,
		TEXT("TrafficVehicleController: Lane change complete — now on lane %d."),
		TargetLane.HandleId);

	FLaneChangeFinalizeResult Result;
	Result.NewLane       = TargetLane;
	Result.NewLanePoints = MoveTemp(TargetLanePoints);
	Result.NewLaneWidth  = TargetLaneWidth;

	// Query speed limit for the new lane.
	if (Provider)
	{
		const float LaneSpeedLimit = Provider->GetLaneSpeedLimit(TargetLane);
		if (LaneSpeedLimit > 0.0f)
		{
			Result.NewTargetSpeed = LaneSpeedLimit;
		}
		else if (DefaultSpeedLimit > 0.0f)
		{
			Result.NewTargetSpeed = DefaultSpeedLimit;
		}
		else
		{
			Result.NewTargetSpeed = BaseTargetSpeed;
		}
	}
	else
	{
		Result.NewTargetSpeed = BaseTargetSpeed;
	}

	// Preserve per-vehicle speed variation across lane changes.
	if (Owner)
	{
		Result.NewTargetSpeed *= Owner->SpeedVariationFactor;
	}

	// Reset internal state.
	State            = ELaneChangeState::None;
	TargetLane       = FTrafficLaneHandle();
	TargetLanePoints.Empty();
	Progress         = 0.0f;
	CooldownRemaining = CooldownTime;

	// Clear navigational lane change flag.
	bNavigationalLaneChange = false;
	NavigationalTargetLane  = FTrafficLaneHandle();

	if (GTrafficVehicleTraceFlushOnSuccess && Owner)
	{
		Owner->FlushLaneDecisionTrace(TEXT("LaneChangeSuccess"), false);
	}

	return Result;
}

// ---------------------------------------------------------------------------
// Abort
// ---------------------------------------------------------------------------

void FLaneChangeCoordinator::Abort()
{
	if (Owner)
	{
		Owner->AddLaneDecisionTrace(TEXT("LaneChange.Abort"),
			TargetLane.IsValid() ? TargetLane.HandleId : 0,
			Progress, CooldownTime, TEXT("AbortLaneChange called"));
	}
	UE_LOG(LogAAATraffic, Log,
		TEXT("TrafficVehicleController: Lane change ABORTED (unsafe gap) — staying on current lane."));

	State            = ELaneChangeState::None;
	TargetLane       = FTrafficLaneHandle();
	TargetLanePoints.Empty();
	Progress         = 0.0f;
	SettleTimer      = 0.0f;
	CooldownRemaining = CooldownTime;

	bNavigationalLaneChange = false;
	NavigationalTargetLane  = FTrafficLaneHandle();

	// If this lane change was part of a same-direction overtake, cancel the overtake too.
	if (bSameDirectionOvertake)
	{
		OvertakePhase = EOvertakePhase::None;
		bSameDirectionOvertake = false;
		OvertakeCooldownRemaining = OvertakeCooldownTimeSec;
	}

	if (Owner)
	{
		Owner->FlushLaneDecisionTrace(TEXT("LaneChangeAbort"), true);
	}
}

// ---------------------------------------------------------------------------
// Reset (quick, no cooldown/logging — used by junction/transition gates)
// ---------------------------------------------------------------------------

void FLaneChangeCoordinator::Reset()
{
	State    = ELaneChangeState::None;
	Progress = 0.0f;
	SettleTimer = 0.0f;
	TargetLanePoints.Empty();

	if (bSameDirectionOvertake)
	{
		OvertakePhase = EOvertakePhase::None;
		bSameDirectionOvertake = false;
	}
}

// ---------------------------------------------------------------------------
// ClearNav
// ---------------------------------------------------------------------------

void FLaneChangeCoordinator::ClearNav()
{
	bNavigationalLaneChange = false;
	NavigationalTargetLane  = FTrafficLaneHandle();
}

// ---------------------------------------------------------------------------
// Overtaking — lateral offset maneuver
// ---------------------------------------------------------------------------

void FLaneChangeCoordinator::TickOvertakeCooldown(float DeltaSeconds)
{
	if (OvertakeCooldownRemaining > 0.0f)
	{
		OvertakeCooldownRemaining -= DeltaSeconds;
	}
}

bool FLaneChangeCoordinator::IsOvertaking() const
{
	return OvertakePhase != EOvertakePhase::None;
}

float FLaneChangeCoordinator::GetOvertakeLateralOffset() const
{
	return OvertakeLateralOffset;
}

float FLaneChangeCoordinator::GetOvertakeSpeedBoostFactor() const
{
	if (OvertakePhase == EOvertakePhase::Passing
		|| OvertakePhase == EOvertakePhase::PullingOut)
	{
		return 1.0f + OvertakeSpeedBoostPct / 100.0f;
	}
	return 1.0f;
}

void FLaneChangeCoordinator::EvaluateOvertake(
	const FLaneChangeEvalContext& Ctx, float DeltaSeconds)
{
	// Guard: already overtaking, already changing lanes, or on cooldown.
	if (OvertakePhase != EOvertakePhase::None) { return; }
	if (State != ELaneChangeState::None) { return; }
	if (OvertakeCooldownRemaining > 0.0f) { return; }

	// Junction / curve / readiness guards (same as regular lane change).
	if (!Ctx.RoadProvider || !Ctx.bLaneDataReady) { return; }
	if (Ctx.bJunctionApproaching) { return; }
	if (Ctx.LanePoints && Ctx.LanePoints->Num() >= 3)
	{
		const float CurCurvature = FMath::Abs(
			FSteeringComputer::ComputeLocalCurvature(*Ctx.LanePoints, Ctx.LastClosestIndex));
		if (CurCurvature > 0.002f) { return; }
	}

	const float EffectiveTarget = FMath::Max(Ctx.TargetSpeed, 1.0f);

	// Stuck timer: vehicle must want to go faster than its leader.
	// Two conditions: either (a) conventional speed-ratio check against
	// SpeedThreshold, OR (b) the vehicle's target speed exceeds its
	// leader's current speed by at least 2%. Condition (b) catches small
	// differentials between adjacent platoon members that (a) misses.
	const bool bSlowRatio = Ctx.CurrentSpeed / EffectiveTarget < SpeedThreshold;
	const bool bFasterThanLeader = Ctx.LeaderSpeed > 1.0f
		&& EffectiveTarget > Ctx.LeaderSpeed * 1.02f;
	const bool bSlow = bSlowRatio || bFasterThanLeader;
	const bool bLeaderClose = Ctx.LeaderDist > 0.0f
		&& Ctx.LeaderDist < Ctx.DetectionDistance;

	if (bSlow && bLeaderClose)
	{
		OvertakeStuckTimer += DeltaSeconds;
	}
	else
	{
		OvertakeStuckTimer = 0.0f;
		return;
	}

	// Not stuck long enough yet — try same-direction first (shorter timer).
	if (OvertakeStuckTimer >= SameDirOvertakeStuckTimeSec)
	{

	// ── Attempt 1: Same-direction lane change (preferred, safe). ──
	const FVector MyDirection = Ctx.RoadProvider->GetLaneDirection(Ctx.CurrentLane);

	for (ETrafficLaneSide Side : { ETrafficLaneSide::Left, ETrafficLaneSide::Right })
	{
		FTrafficLaneHandle Candidate = Ctx.RoadProvider->GetAdjacentLane(Ctx.CurrentLane, Side);
		if (!Candidate.IsValid()) { continue; }
		const FVector CandDir = Ctx.RoadProvider->GetLaneDirection(Candidate);
		if (FVector::DotProduct(MyDirection, CandDir) < 0.0f) { continue; } // Oncoming, skip

		// Gap check on the candidate same-direction lane.
		if (!Ctx.TrafficSub) { continue; }
		bool bGapClear = true;
		const float EffectiveGap = FMath::Max(GapRequired,
			Ctx.VehicleFrontExtent + Ctx.VehicleRearExtent + 100.0f);

		const TArray<TWeakObjectPtr<ATrafficVehicleController>> Neighbors =
			Ctx.TrafficSub->GetVehiclesOnLane(Candidate);
		for (const TWeakObjectPtr<ATrafficVehicleController>& WeakNeighbor : Neighbors)
		{
			ATrafficVehicleController* Neighbor = WeakNeighbor.Get();
			if (!Neighbor || Neighbor == Owner) { continue; }
			const APawn* NeighborPawn = Neighbor->GetPawn();
			if (!NeighborPawn) { continue; }
			const float DistToNeighbor = FVector::Dist(
				Ctx.VehicleLocation, NeighborPawn->GetActorLocation());
			if (DistToNeighbor < EffectiveGap)
			{
				bGapClear = false;
				break;
			}
		}
		if (!bGapClear) { continue; }

		// Lane path availability.
		TArray<FVector> CandidatePoints;
		float CandidateWidth;
		if (!Ctx.RoadProvider->GetLanePath(Candidate, CandidatePoints, CandidateWidth)
			|| CandidatePoints.Num() < 2)
		{
			continue;
		}

		// Same-direction lane is clear — initiate lane change and tag as overtake.
		State            = ELaneChangeState::Executing;
		TargetLane       = Candidate;
		TargetLanePoints = MoveTemp(CandidatePoints);
		TargetLaneWidth  = CandidateWidth;
		Progress         = 0.0f;

		bSameDirectionOvertake = true;
		OvertakePhase          = EOvertakePhase::PullingOut;
		OvertakePassDistAccum  = 0.0f;
		OvertakeStuckTimer     = 0.0f;

		UE_LOG(LogAAATraffic, Log,
			TEXT("TrafficVehicleController: Beginning SAME-DIRECTION OVERTAKE from "
				 "lane %d to lane %d (%s)."),
			Ctx.CurrentLane.HandleId, Candidate.HandleId,
			Side == ETrafficLaneSide::Left ? TEXT("left") : TEXT("right"));
		if (Owner)
		{
			Owner->AddLaneDecisionTrace(TEXT("Overtake.SameDirBegin"),
				Candidate.HandleId, EffectiveTarget, Ctx.LeaderSpeed,
				Side == ETrafficLaneSide::Left ? TEXT("Left") : TEXT("Right"));
		}
		return;
	}

	} // end SameDirOvertakeStuckTimeSec gate

	// Wait for the longer timer before attempting oncoming-lane overtake.
	if (OvertakeStuckTimer < OvertakeStuckTimeSec) { return; }

	// ── Attempt 2: Oncoming-lane lateral offset (only on roads with
	//    both a same-direction neighbor and an oncoming lane). ──
	const FVector MyDirection2 = Ctx.RoadProvider->GetLaneDirection(Ctx.CurrentLane);
	FTrafficLaneHandle OncomingLane;
	ETrafficLaneSide OncomingSide = ETrafficLaneSide::Left;
	bool bHasSameDirectionNeighbor = false;

	for (ETrafficLaneSide Side : { ETrafficLaneSide::Left, ETrafficLaneSide::Right })
	{
		FTrafficLaneHandle Candidate = Ctx.RoadProvider->GetAdjacentLane(Ctx.CurrentLane, Side);
		if (!Candidate.IsValid()) { continue; }
		const FVector CandDir = Ctx.RoadProvider->GetLaneDirection(Candidate);
		const float DirDot = FVector::DotProduct(MyDirection2, CandDir);
		if (DirDot >= 0.0f)
		{
			bHasSameDirectionNeighbor = true;
		}
		else
		{
			OncomingLane = Candidate;
			OncomingSide = Side;
		}
	}

	if (!OncomingLane.IsValid())
	{
		// No oncoming lane at all (one-way / boulevard) — no target
		// lane available for overtaking.
		OvertakeStuckTimer = 0.0f;
		return;
	}

	// Check oncoming lane clearance via TrafficSubsystem registry.
	if (!Ctx.TrafficSub) { return; }

	const TArray<TWeakObjectPtr<ATrafficVehicleController>> OncomingVehicles =
		Ctx.TrafficSub->GetVehiclesOnLane(OncomingLane);

	for (const TWeakObjectPtr<ATrafficVehicleController>& WeakOncoming : OncomingVehicles)
	{
		ATrafficVehicleController* Oncoming = WeakOncoming.Get();
		if (!Oncoming || Oncoming == Owner) { continue; }
		const APawn* OncomingPawn = Oncoming->GetPawn();
		if (!OncomingPawn) { continue; }

		// Distance along our forward direction to the oncoming vehicle.
		const FVector ToOncoming = OncomingPawn->GetActorLocation() - Ctx.VehicleLocation;
		const float ForwardDist = FVector::DotProduct(ToOncoming, Ctx.EgoForward);

		// Only care about vehicles ahead of us.
		if (ForwardDist > 0.0f && ForwardDist < OvertakeMinClearanceCm)
		{
			// Not enough clearance.
			if (IsTraceEnabled(2) && Owner)
			{
				Owner->AddLaneDecisionTrace(TEXT("Overtake.RejectClearance"),
					OncomingLane.HandleId, ForwardDist, OvertakeMinClearanceCm,
					TEXT("Oncoming vehicle too close"));
			}
			return;
		}
	}

	// All clear — begin overtaking.
	OvertakePhase       = EOvertakePhase::PullingOut;
	OvertakeBlendProgress = 0.0f;
	OvertakePassDistAccum = 0.0f;
	OvertakeSide        = OncomingSide;
	OvertakeOncomingLane = OncomingLane;

	// Target offset ≈ lane width of own lane.
	TArray<FVector> TempPath;
	float TempWidth = 0.0f;
	if (Ctx.RoadProvider->GetLanePath(Ctx.CurrentLane, TempPath, TempWidth)
		&& TempWidth > 0.0f)
	{
		OvertakeTargetOffset = TempWidth;
	}
	else
	{
		OvertakeTargetOffset = 350.0f; // fallback 3.5m
	}

	OvertakeStuckTimer = 0.0f;

	UE_LOG(LogAAATraffic, Log,
		TEXT("TrafficVehicleController: Beginning OVERTAKE from lane %d via %s-side "
			 "(oncoming lane %d, offset %.0f cm)."),
		Ctx.CurrentLane.HandleId,
		OncomingSide == ETrafficLaneSide::Left ? TEXT("left") : TEXT("right"),
		OncomingLane.HandleId,
		OvertakeTargetOffset);
	if (Owner)
	{
		Owner->AddLaneDecisionTrace(TEXT("Overtake.Begin"),
			OncomingLane.HandleId, OvertakeTargetOffset, 0.0f,
			OncomingSide == ETrafficLaneSide::Left ? TEXT("Left") : TEXT("Right"));
	}
}

void FLaneChangeCoordinator::TickOvertake(
	float DeltaSeconds, float DistanceThisTick,
	const FLaneChangeEvalContext& Ctx)
{
	if (OvertakePhase == EOvertakePhase::None) { return; }

	// ── Same-direction overtake: lane change handles positioning, not lateral offset. ──
	if (bSameDirectionOvertake)
	{
		switch (OvertakePhase)
		{
		case EOvertakePhase::PullingOut:
			// Lane change is executing (State manages it).
			// When the lane change completes (State returns to None), advance to Passing.
			if (State == ELaneChangeState::None)
			{
				OvertakePhase = EOvertakePhase::Passing;
				OvertakePassDistAccum = 0.0f;
				UE_LOG(LogAAATraffic, Log,
					TEXT("TrafficVehicleController: Same-direction overtake — "
						 "lane change complete, now passing."));
			}
			break;

		case EOvertakePhase::Passing:
			OvertakePassDistAccum += DistanceThisTick;
			if (OvertakePassDistAccum >= OvertakeMinPassDistCm)
			{
				if (Ctx.LeaderDist < 0.0f
					|| Ctx.LeaderDist > Ctx.DetectionDistance * 0.5f
					|| OvertakePassDistAccum >= OvertakeMinPassDistCm * 3.0f)
				{
					OvertakePhase = EOvertakePhase::None;
					bSameDirectionOvertake = false;
					OvertakeCooldownRemaining = OvertakeCooldownTimeSec;
					UE_LOG(LogAAATraffic, Log,
						TEXT("TrafficVehicleController: Same-direction overtake COMPLETE "
							 "(%.0f cm passed). Cooldown %.1f s."),
						OvertakePassDistAccum, OvertakeCooldownTimeSec);
					if (Owner)
					{
						Owner->AddLaneDecisionTrace(TEXT("Overtake.SameDirComplete"),
							0, OvertakePassDistAccum, OvertakeCooldownTimeSec,
							TEXT("Same-direction overtake finished"));
					}
				}
			}
			break;

		default:
			// PullingIn not used for same-direction overtakes.
			OvertakePhase = EOvertakePhase::None;
			bSameDirectionOvertake = false;
			break;
		}
		return;
	}

	// ── Lateral-offset overtake (oncoming lane): continuous clearance check. ──
	if (OvertakePhase != EOvertakePhase::PullingIn
		&& Ctx.TrafficSub && OvertakeOncomingLane.IsValid())
	{
		const TArray<TWeakObjectPtr<ATrafficVehicleController>> OncomingVehicles =
			Ctx.TrafficSub->GetVehiclesOnLane(OvertakeOncomingLane);

		for (const TWeakObjectPtr<ATrafficVehicleController>& WeakOncoming : OncomingVehicles)
		{
			ATrafficVehicleController* Oncoming = WeakOncoming.Get();
			if (!Oncoming || Oncoming == Owner) { continue; }
			const APawn* OncomingPawn = Oncoming->GetPawn();
			if (!OncomingPawn) { continue; }

			const FVector ToOncoming = OncomingPawn->GetActorLocation() - Ctx.VehicleLocation;
			const float ForwardDist = FVector::DotProduct(ToOncoming, Ctx.EgoForward);

			if (ForwardDist > 0.0f && ForwardDist < OvertakeAbortClearanceCm)
			{
				UE_LOG(LogAAATraffic, Warning,
					TEXT("TrafficVehicleController: OVERTAKE ABORT — oncoming vehicle "
						 "at %.0f cm (threshold %.0f cm)."),
					ForwardDist, OvertakeAbortClearanceCm);
				if (Owner)
				{
					Owner->AddLaneDecisionTrace(TEXT("Overtake.AbortOncoming"),
						OvertakeOncomingLane.HandleId, ForwardDist,
						OvertakeAbortClearanceCm, TEXT("Oncoming too close — aborting"));
				}
				AbortOvertake();
				return;
			}
		}
	}

	switch (OvertakePhase)
	{
	case EOvertakePhase::PullingOut:
	{
		const float BlendDelta = DistanceThisTick / FMath::Max(OvertakeBlendDistCm, 1.0f);
		OvertakeBlendProgress = FMath::Min(OvertakeBlendProgress + BlendDelta, 1.0f);
		const float Alpha = FMath::SmoothStep(0.0f, 1.0f, OvertakeBlendProgress);
		OvertakeLateralOffset = OvertakeTargetOffset * Alpha;

		if (OvertakeBlendProgress >= 1.0f)
		{
			OvertakePhase = EOvertakePhase::Passing;
			OvertakePassDistAccum = 0.0f;
			UE_LOG(LogAAATraffic, Verbose,
				TEXT("TrafficVehicleController: Overtake pull-out complete, now passing."));
		}
		break;
	}

	case EOvertakePhase::Passing:
	{
		OvertakePassDistAccum += DistanceThisTick;
		OvertakeLateralOffset = OvertakeTargetOffset; // Hold full offset.

		if (OvertakePassDistAccum >= OvertakeMinPassDistCm)
		{
			// Check if we've actually passed the slow leader.
			// If leader is no longer detected ahead (LeaderDist < 0 or very far),
			// or we've traveled enough — begin pulling back in.
			if (Ctx.LeaderDist < 0.0f || Ctx.LeaderDist > Ctx.DetectionDistance * 0.5f
				|| OvertakePassDistAccum >= OvertakeMinPassDistCm * 3.0f)
			{
				OvertakePhase = EOvertakePhase::PullingIn;
				OvertakeBlendProgress = 0.0f;
				UE_LOG(LogAAATraffic, Log,
					TEXT("TrafficVehicleController: Overtake passing complete "
						 "(%.0f cm traveled), pulling back in."),
					OvertakePassDistAccum);
			}
		}
		break;
	}

	case EOvertakePhase::PullingIn:
	{
		const float BlendDelta = DistanceThisTick / FMath::Max(OvertakeBlendDistCm, 1.0f);
		OvertakeBlendProgress = FMath::Min(OvertakeBlendProgress + BlendDelta, 1.0f);
		const float Alpha = 1.0f - FMath::SmoothStep(0.0f, 1.0f, OvertakeBlendProgress);
		OvertakeLateralOffset = OvertakeTargetOffset * Alpha;

		if (OvertakeBlendProgress >= 1.0f)
		{
			// Overtake complete.
			OvertakePhase = EOvertakePhase::None;
			OvertakeLateralOffset = 0.0f;
			OvertakeCooldownRemaining = OvertakeCooldownTimeSec;
			OvertakeOncomingLane = FTrafficLaneHandle();
			UE_LOG(LogAAATraffic, Log,
				TEXT("TrafficVehicleController: Overtake COMPLETE — back on lane."));
			if (Owner)
			{
				Owner->AddLaneDecisionTrace(TEXT("Overtake.Complete"),
					0, OvertakePassDistAccum, OvertakeCooldownTimeSec,
					TEXT("Overtake finished"));
			}
		}
		break;
	}

	default:
		break;
	}
}

void FLaneChangeCoordinator::AbortOvertake()
{
	if (OvertakePhase == EOvertakePhase::None) { return; }

	// Same-direction overtake: just clear the tag — lane change abort is separate.
	if (bSameDirectionOvertake)
	{
		OvertakePhase = EOvertakePhase::None;
		bSameDirectionOvertake = false;
		OvertakeCooldownRemaining = OvertakeCooldownTimeSec;
		if (Owner)
		{
			Owner->AddLaneDecisionTrace(TEXT("Overtake.SameDirAbort"),
				0, 0.0f, 0.0f, TEXT("Same-direction overtake aborted"));
		}
		return;
	}

	// If already pulling in, let it finish.
	if (OvertakePhase == EOvertakePhase::PullingIn) { return; }

	// Transition to pulling in — smooth return.
	OvertakePhase = EOvertakePhase::PullingIn;
	OvertakeBlendProgress = 0.0f;

	if (Owner)
	{
		Owner->AddLaneDecisionTrace(TEXT("Overtake.Abort"),
			OvertakeOncomingLane.IsValid() ? OvertakeOncomingLane.HandleId : 0,
			OvertakeLateralOffset, 0.0f, TEXT("Overtake aborted — pulling in"));
	}
}
