// Copyright AAA_Traffic Contributors. All Rights Reserved.
// TrafficSubsystem_JunctionMgmt.cpp — Junction occupancy, conflict detection,
// stop-sign queues, and junction survey.

#include "TrafficSubsystem.h"
#include "TrafficRoadProvider.h"
#include "TrafficVehicleController.h"
#include "TrafficSignalController.h"
#include "TrafficLog.h"
#include "GameFramework/PawnMovementComponent.h"

extern int32 GTrafficJunctionDiagnostics;

namespace
{
	constexpr uint32 CanonicalFlagHasProviderPath = 1u << 0;
	constexpr uint32 CanonicalFlagHasEntryAttach = 1u << 1;
	constexpr uint32 CanonicalFlagHasExitAttach = 1u << 2;
	constexpr uint32 CanonicalFlagHasReleaseIndex = 1u << 3;
	constexpr uint32 CanonicalFlagHasResumeIndex = 1u << 4;
	constexpr uint32 CanonicalFlagHasConflictSet = 1u << 5;

	float ComputeLaneProgressCm(const TArray<FVector>& LanePoints, const FVector& WorldLocation)
	{
		if (LanePoints.Num() == 0)
		{
			return 0.0f;
		}

		int32 BestIndex = 0;
		float BestDistSq = FVector::DistSquared(WorldLocation, LanePoints[0]);
		for (int32 PointIndex = 1; PointIndex < LanePoints.Num(); ++PointIndex)
		{
			const float DistSq = FVector::DistSquared(WorldLocation, LanePoints[PointIndex]);
			if (DistSq < BestDistSq)
			{
				BestDistSq = DistSq;
				BestIndex = PointIndex;
			}
		}

		float ProgressCm = 0.0f;
		for (int32 PointIndex = 0; PointIndex < BestIndex && PointIndex < LanePoints.Num() - 1; ++PointIndex)
		{
			ProgressCm += FVector::Dist2D(LanePoints[PointIndex], LanePoints[PointIndex + 1]);
		}

		return ProgressCm;
	}

	int32 FindNearestPolylineIndex(const TArray<FVector>& Points, const FVector& WorldLocation)
	{
		if (Points.IsEmpty())
		{
			return INDEX_NONE;
		}

		int32 BestIndex = 0;
		float BestDistSq = FVector::DistSquared(Points[0], WorldLocation);
		for (int32 PointIndex = 1; PointIndex < Points.Num(); ++PointIndex)
		{
			const float DistSq = FVector::DistSquared(Points[PointIndex], WorldLocation);
			if (DistSq < BestDistSq)
			{
				BestDistSq = DistSq;
				BestIndex = PointIndex;
			}
		}

		return BestIndex;
	}

	float ComputePolylineArcLengthCm(const TArray<FVector>& Points)
	{
		float ArcLengthCm = 0.0f;
		for (int32 PointIndex = 0; PointIndex + 1 < Points.Num(); ++PointIndex)
		{
			ArcLengthCm += FVector::Dist(Points[PointIndex], Points[PointIndex + 1]);
		}
		return ArcLengthCm;
	}

	float ComputeMinTurnRadiusCm(const TArray<FVector>& Points)
	{
		float MinTurnRadiusCm = TNumericLimits<float>::Max();
		for (int32 PointIndex = 1; PointIndex + 1 < Points.Num(); ++PointIndex)
		{
			const FVector SegmentA = Points[PointIndex] - Points[PointIndex - 1];
			const FVector SegmentB = Points[PointIndex + 1] - Points[PointIndex];
			const float CrossMag = FMath::Abs(FVector::CrossProduct(SegmentA, SegmentB).Z);
			if (CrossMag <= KINDA_SMALL_NUMBER)
			{
				continue;
			}

			const float ChordLength = FVector::Dist(Points[PointIndex - 1], Points[PointIndex + 1]);
			const float Numerator = SegmentA.Size() * SegmentB.Size() * ChordLength;
			if (Numerator <= KINDA_SMALL_NUMBER)
			{
				continue;
			}

			const float RadiusCm = Numerator / (2.0f * CrossMag);
			if (RadiusCm > 0.0f)
			{
				MinTurnRadiusCm = FMath::Min(MinTurnRadiusCm, RadiusCm);
			}
		}

		return MinTurnRadiusCm == TNumericLimits<float>::Max() ? 0.0f : MinTurnRadiusCm;
	}

	ECanonicalMovementClass ClassifyCanonicalMovementClass(ETurnSignalState TurnDirection)
	{
		switch (TurnDirection)
		{
		case ETurnSignalState::Left:
			return ECanonicalMovementClass::Left;
		case ETurnSignalState::Right:
			return ECanonicalMovementClass::Right;
		case ETurnSignalState::Off:
			return ECanonicalMovementClass::Straight;
		default:
			return ECanonicalMovementClass::Unknown;
		}
	}

	void SortAndUniqueIds(TArray<int32>& Values)
	{
		Values.Sort();
		for (int32 Index = Values.Num() - 1; Index > 0; --Index)
		{
			if (Values[Index] == Values[Index - 1])
			{
				Values.RemoveAt(Index);
			}
		}
	}

	FTrafficLaneHandle ResolveApproachJunctionLane(
		ITrafficRoadProvider* Provider,
		const FTrafficLaneHandle& ApproachLane,
		int32 JunctionId)
	{
		if (!Provider || !ApproachLane.IsValid() || JunctionId == 0)
		{
			return FTrafficLaneHandle();
		}

		const ITrafficRoadProvider::FJunctionScanResult Scan = Provider->GetDistanceToNextJunction(
			ApproachLane,
			Provider->GetLaneLength(ApproachLane),
			50000.0f,
			10);
		if (Scan.JunctionId == JunctionId && Scan.JunctionLane.IsValid())
		{
			return Scan.JunctionLane;
		}

		const TArray<FTrafficLaneHandle> Connected = Provider->GetConnectedLanes(ApproachLane);
		for (const FTrafficLaneHandle& Candidate : Connected)
		{
			if (Provider->GetJunctionForLane(Candidate) == JunctionId)
			{
				return Candidate;
			}
		}

		return FTrafficLaneHandle();
	}

	bool BuildCanonicalMovementRecord(
		ITrafficRoadProvider* Provider,
		int32 JunctionId,
		const FTrafficLaneHandle& ApproachLane,
		const FTrafficLaneHandle& ApproachJunctionLane,
		const FJunctionExitRule& Rule,
		FCanonicalMovementRecord& OutRecord,
		FString& OutFailureReason)
	{
		if (!Provider)
		{
			OutFailureReason = TEXT("provider unavailable");
			return false;
		}

		if (!ApproachLane.IsValid() || !Rule.ExitLane.IsValid() || !ApproachJunctionLane.IsValid())
		{
			OutFailureReason = TEXT("invalid approach/exit/junction lane handles");
			return false;
		}

		if (!Rule.bPhysicallyFeasible)
		{
			OutFailureReason = TEXT("survey marked movement physically infeasible");
			return false;
		}

		TArray<FVector> ApproachPoints;
		TArray<FVector> ExitPoints;
		float ApproachWidthCm = 0.0f;
		float ExitWidthCm = 0.0f;
		if (!Provider->GetLanePath(ApproachLane, ApproachPoints, ApproachWidthCm) || ApproachPoints.Num() < 2)
		{
			OutFailureReason = TEXT("approach lane path unavailable");
			return false;
		}
		if (!Provider->GetLanePath(Rule.ExitLane, ExitPoints, ExitWidthCm) || ExitPoints.Num() < 2)
		{
			OutFailureReason = TEXT("exit lane path unavailable");
			return false;
		}

		OutRecord = FCanonicalMovementRecord();
		OutRecord.JunctionId = JunctionId;
		OutRecord.FromLane = ApproachLane;
		OutRecord.ToLane = Rule.ExitLane;
		OutRecord.ApproachJunctionLane = ApproachJunctionLane;
		OutRecord.TurnDirection = Rule.TurnDirection;
		OutRecord.MovementClass = ClassifyCanonicalMovementClass(Rule.TurnDirection);
		OutRecord.SelectionWeight = Rule.Weight;
		OutRecord.bLegallyAllowed = true;
		OutRecord.bPhysicallyFeasible = Rule.bPhysicallyFeasible;

		bool bHasProviderPath = Provider->GetJunctionPath(ApproachLane, Rule.ExitLane, OutRecord.CorridorPoints)
			&& OutRecord.CorridorPoints.Num() >= 2;
		if (!bHasProviderPath)
		{
			OutRecord.CorridorPoints.Reset();
			OutRecord.CorridorPoints.Add(ApproachPoints.Last());
			OutRecord.CorridorPoints.Add(ExitPoints[0]);
			OutRecord.SourceKind = ECanonicalMovementSourceKind::SynthesizedDuringCompile;
		}
		else
		{
			OutRecord.SourceKind = ECanonicalMovementSourceKind::ProviderDerived;
			OutRecord.ValidationFlags |= CanonicalFlagHasProviderPath;
		}

		if (OutRecord.CorridorPoints.Num() < 2)
		{
			OutFailureReason = TEXT("corridor path invalid");
			return false;
		}

		OutRecord.CorridorEntryPoint = OutRecord.CorridorPoints[0];
		OutRecord.CorridorExitPoint = OutRecord.CorridorPoints.Last();
		OutRecord.EntryLaneAttachIndex = FindNearestPolylineIndex(ApproachPoints, OutRecord.CorridorEntryPoint);
		OutRecord.ExitLaneAttachIndex = FindNearestPolylineIndex(ExitPoints, OutRecord.CorridorExitPoint);
		if (OutRecord.EntryLaneAttachIndex == INDEX_NONE)
		{
			OutFailureReason = TEXT("entry attach index unresolved");
			return false;
		}
		if (OutRecord.ExitLaneAttachIndex == INDEX_NONE)
		{
			OutFailureReason = TEXT("exit attach index unresolved");
			return false;
		}
		OutRecord.ValidationFlags |= CanonicalFlagHasEntryAttach;
		OutRecord.ValidationFlags |= CanonicalFlagHasExitAttach;

		OutRecord.CorridorArcLengthCm = ComputePolylineArcLengthCm(OutRecord.CorridorPoints);
		OutRecord.MinTurnRadiusCm = ComputeMinTurnRadiusCm(OutRecord.CorridorPoints);
		OutRecord.TraversalReleaseIndex = OutRecord.CorridorPoints.Num() - 1;
		OutRecord.ExitLaneResumeIndex = OutRecord.ExitLaneAttachIndex;
		OutRecord.ValidationFlags |= CanonicalFlagHasReleaseIndex;
		OutRecord.ValidationFlags |= CanonicalFlagHasResumeIndex;

#if !UE_BUILD_SHIPPING
		OutRecord.BuildNotes = FString::Printf(
			TEXT("Source=%s EntryAttach=%d ExitAttach=%d Arc=%.0f Radius=%.0f Weight=%.3f"),
			bHasProviderPath ? TEXT("provider") : TEXT("synth"),
			OutRecord.EntryLaneAttachIndex,
			OutRecord.ExitLaneAttachIndex,
			OutRecord.CorridorArcLengthCm,
			OutRecord.MinTurnRadiusCm,
			OutRecord.SelectionWeight);
#endif

		return true;
	}
}
// ---------------------------------------------------------------------------
// Junction Occupancy — multi-vehicle with geometric conflict detection
// ---------------------------------------------------------------------------

bool UTrafficSubsystem::TryOccupyJunction(int32 JunctionId,
	ATrafficVehicleController* Controller,
	int32 MovementId)
{
	if (JunctionId == 0 || !Controller)
	{
		UE_LOG(LogAAATraffic, Warning,
			TEXT("JNCT TryOccupy: JunctionId=%d Controller=%s — trivial pass (no junction or no controller)"),
			JunctionId, Controller ? TEXT("valid") : TEXT("NULL"));
		return true; // No junction — always OK.
	}

	const FString CallerName = (Controller && Controller->GetPawn())
		? Controller->GetPawn()->GetName() : TEXT("NULL");
	const FCanonicalMovementRecord* MovementRecord = GetCanonicalMovement(MovementId);
	if (!MovementRecord || !MovementRecord->IsValid() || MovementRecord->JunctionId != JunctionId)
	{
		UE_LOG(LogAAATraffic, Warning,
			TEXT("JNCT TryOccupy: JunctionId=%d Caller='%s' Movement=%d — DENIED (invalid canonical movement)"),
			JunctionId,
			*CallerName,
			MovementId);
		return false;
	}
	const FTrafficLaneHandle& FromLane = MovementRecord->FromLane;
	const FTrafficLaneHandle& ToLane = MovementRecord->ToLane;

	TArray<FJunctionOccupant>& Occupants = JunctionOccupancy.FindOrAdd(JunctionId);

	// Purge stale entries (GC'd controllers).
	Occupants.RemoveAll([](const FJunctionOccupant& O) { return !O.Controller.IsValid(); });

	// Evict occupants that have held the junction for too long.
	// A normal junction traversal takes a few seconds at most.
	// If an occupant is still here after MaxJunctionOccupancyTimeSec,
	// it is stuck (physics collision, broken path, etc.) and blocking
	// all traffic behind it.  Evicting cleans the occupancy so
	// waiting vehicles can proceed via normal TryOccupy.
	constexpr double MaxJunctionOccupancyTimeSec = 10.0;
	// Extended grace period for vehicles actively traversing at speed.
	constexpr double TraversingGraceSec = 20.0;
	const UWorld* World = GetWorld();
	const double NowSec = World ? World->GetTimeSeconds() : 0.0;
	Occupants.RemoveAll([NowSec, JunctionId](const FJunctionOccupant& O)
	{
		if (O.OccupiedAtTime > 0.0 && (NowSec - O.OccupiedAtTime) > MaxJunctionOccupancyTimeSec)
		{
			// Don't evict vehicles actively traversing at speed — large
			// junctions (R>10000) can legitimately take 15+ seconds.
			if (O.Controller.IsValid())
			{
				ATrafficVehicleController* Ctrl = O.Controller.Get();
				if (Ctrl->JnctState.Phase == EJunctionPhase::Traversing)
				{
					const float Speed = Ctrl->GetPawn()
						? Ctrl->GetPawn()->GetVelocity().Size() : 0.0f;
					if (Speed > 50.0f && (NowSec - O.OccupiedAtTime) <= TraversingGraceSec)
					{
						return false; // Still making progress.
					}
				}
			}

			const FString StuckName = (O.Controller.IsValid() && O.Controller.Get()->GetPawn())
				? O.Controller.Get()->GetPawn()->GetName() : TEXT("STALE");
			UE_LOG(LogAAATraffic, Warning,
				TEXT("JNCT EVICT-STALE: JunctionId=%d Occupant='%s' held for %.1fs — "
					 "evicting stuck occupant to unblock junction"),
				JunctionId, *StuckName, NowSec - O.OccupiedAtTime);
			// Sync controller token so it knows occupancy was revoked.
			if (O.Controller.IsValid())
			{
				O.Controller.Get()->JnctState.bOwnsJunctionOccupancy = false;
			}
			return true;
		}
		return false;
	});

	// Self-reentry: already occupying this junction — always OK.
	for (const FJunctionOccupant& Occ : Occupants)
	{
		if (Occ.Controller.Get() == Controller)
		{
			UE_LOG(LogAAATraffic, Warning,
				TEXT("JNCT TryOccupy: JunctionId=%d Caller='%s' — GRANTED (self-reentry)"),
				JunctionId, *CallerName);
			return true;
		}
	}

	// Conflict test: check against every existing canonical occupant.
	ITrafficRoadProvider* Provider = GetProvider();
	for (const FJunctionOccupant& Occ : Occupants)
	{
		if (!Occ.Controller.IsValid()) { continue; }

		const FCanonicalMovementRecord* OccupantMovement = GetCanonicalMovement(Occ.MovementId);
		const bool bConflict = !OccupantMovement
			|| !OccupantMovement->IsValid()
			|| MovementRecord->ConflictMovementIds.Contains(Occ.MovementId);

		if (bConflict)
		{
			const FString OccName = (Occ.Controller.Get() && Occ.Controller.Get()->GetPawn())
				? Occ.Controller.Get()->GetPawn()->GetName() : TEXT("STALE");
			UE_LOG(LogAAATraffic, Warning,
				TEXT("JNCT TryOccupy: JunctionId=%d Caller='%s' Movement=%d CONFLICTS with '%s' "
					 "(Caller: Mvmt=%d From=%d To=%d | Occupant: Mvmt=%d From=%d To=%d) — DENIED"),
				JunctionId, *CallerName, MovementRecord->MovementId, *OccName,
				MovementRecord->MovementId,
				FromLane.HandleId, ToLane.HandleId,
				Occ.MovementId,
				Occ.FromLane.HandleId, Occ.ToLane.HandleId);
			return false;
		}
	}

	// Queue spillback / "Don't Block the Box":
	// If the exit lane is already congested, deny entry to prevent the
	// vehicle from blocking the junction when it can't clear.
	if (ToLane.IsValid())
	{
		const TArray<TWeakObjectPtr<ATrafficVehicleController>>* ExitVehicles =
			VehiclesByLane.Find(ToLane.HandleId);
		if (ExitVehicles)
		{
			const float EnteringVehicleLengthCm = FMath::Max(
				Controller->VehicleFrontExtent + Controller->VehicleRearExtent,
				450.0f);
			const float RequiredExitClearanceCm = FMath::Max(500.0f, EnteringVehicleLengthCm + 250.0f);

			for (const TWeakObjectPtr<ATrafficVehicleController>& VC : *ExitVehicles)
			{
				ATrafficVehicleController* ExitController = VC.Get();
				if (!ExitController || ExitController == Controller)
				{
					continue;
				}

				const APawn* ExitPawn = ExitController->GetPawn();
				if (!ExitPawn || ExitController->LanePoints.Num() == 0)
				{
					continue;
				}

				const float ExitProgressCm = ComputeLaneProgressCm(
					ExitController->LanePoints,
					ExitPawn->GetActorLocation());
				const float BlockerRearExtentCm = FMath::Max(ExitController->VehicleRearExtent, 225.0f);
				const float AvailableClearanceCm = ExitProgressCm - BlockerRearExtentCm;

				if (AvailableClearanceCm < RequiredExitClearanceCm)
				{
					UE_LOG(LogAAATraffic, Warning,
						TEXT("JNCT TryOccupy: JunctionId=%d Caller='%s' — DENIED (exit lane %d blocked by '%s' clearance=%.0f<%.0f)"),
						JunctionId,
						*CallerName,
						ToLane.HandleId,
						*ExitPawn->GetName(),
						AvailableClearanceCm,
						RequiredExitClearanceCm);
					return false;
				}
			}

			// Count valid vehicles on the exit lane.
			int32 ExitCount = 0;
			for (const TWeakObjectPtr<ATrafficVehicleController>& VC : *ExitVehicles)
			{
				if (VC.IsValid()) { ++ExitCount; }
			}
			// Derive threshold from exit lane capacity instead of a fixed
			// constant.  Capacity = LaneLength / AverageVehicleSpacing.
			// AverageVehicleSpacing ≈ sedan length (450 cm) + min gap (200 cm).
			// Fall back to 3 if the provider can't report lane length.
			constexpr float AverageVehicleSpacingCm = 650.0f;
			int32 SpillbackThreshold = 3; // fallback
			if (Provider)
			{
				const float ExitLaneLen = Provider->GetLaneLength(ToLane);
				if (ExitLaneLen > 0.0f)
				{
					SpillbackThreshold = FMath::Max(2, FMath::FloorToInt32(ExitLaneLen / AverageVehicleSpacingCm));
				}
			}
			if (ExitCount >= SpillbackThreshold)
			{
				UE_LOG(LogAAATraffic, Warning,
					TEXT("JNCT TryOccupy: JunctionId=%d Caller='%s' — DENIED (exit lane %d has %d/%d vehicles — spillback)"),
					JunctionId, *CallerName, ToLane.HandleId, ExitCount, SpillbackThreshold);
				return false;
			}
		}
	}

	// No conflicts — grant entry.
	FJunctionOccupant NewOccupant;
	NewOccupant.Controller = Controller;
	NewOccupant.MovementId = MovementId;
	NewOccupant.FromLane = FromLane;
	NewOccupant.ToLane = ToLane;
	{
		const UWorld* W = GetWorld();
		NewOccupant.OccupiedAtTime = W ? W->GetTimeSeconds() : 0.0;
	}
	Occupants.Add(NewOccupant);

	UE_LOG(LogAAATraffic, Warning,
		TEXT("JNCT TryOccupy: JunctionId=%d Caller='%s' Movement=%d From=%d To=%d — GRANTED (%d occupants now)"),
		JunctionId, *CallerName, MovementId, FromLane.HandleId, ToLane.HandleId, Occupants.Num());
	return true;
}

void UTrafficSubsystem::ReleaseJunction(int32 JunctionId, ATrafficVehicleController* Controller)
{
	const FString CallerName = (Controller && Controller->GetPawn())
		? Controller->GetPawn()->GetName() : TEXT("NULL");

	if (TArray<FJunctionOccupant>* Occupants = JunctionOccupancy.Find(JunctionId))
	{
		const int32 Removed = Occupants->RemoveAll([Controller](const FJunctionOccupant& O)
		{
			return O.Controller.Get() == Controller;
		});

		if (Removed > 0)
		{
			UE_LOG(LogAAATraffic, Warning,
				TEXT("JNCT Release: JunctionId=%d Caller='%s' — RELEASED (%d remaining occupants)"),
				JunctionId, *CallerName, Occupants->Num());

			// Clean up empty junctions.
			if (Occupants->Num() == 0)
			{
				JunctionOccupancy.Remove(JunctionId);
			}
		}
		else
		{
			UE_LOG(LogAAATraffic, Warning,
				TEXT("JNCT Release: JunctionId=%d Caller='%s' — NOT FOUND in occupant list (already released?)"),
				JunctionId, *CallerName);
		}
	}
	else
	{
		UE_LOG(LogAAATraffic, Warning,
			TEXT("JNCT Release: JunctionId=%d Caller='%s' — junction NOT in occupancy map"),
			JunctionId, *CallerName);
	}
}

bool UTrafficSubsystem::HasConflictingApproach(
	int32 JunctionId,
	ATrafficVehicleController* Self,
	int32 MovementId) const
{
	const FCanonicalMovementRecord* SelfMovement = GetCanonicalMovement(MovementId);
	if (!SelfMovement || !SelfMovement->IsValid() || SelfMovement->JunctionId != JunctionId)
	{
		return false;
	}

	for (const TWeakObjectPtr<ATrafficVehicleController>& WeakOther : ActiveVehicles)
	{
		ATrafficVehicleController* Other = WeakOther.Get();
		if (!Other || Other == Self) { continue; }

		// Only consider vehicles interacting with the same junction.
		// JnctState.JunctionId is now set from the approach scan onward
		// (Phase >= Approaching), not just after the detection gate.
		if (Other->JnctState.JunctionId != JunctionId) { continue; }

		// Only yield to vehicles that are actually at the stop bar or
		// already in the junction (Phase >= Waiting). Vehicles still
		// approaching (Phase == Approaching) are too far away to
		// constitute an imminent conflict — yielding to them causes
		// 30+ second deadlocks.
		if (Other->JnctState.Phase < EJunctionPhase::Waiting) { continue; }

		const FCanonicalMovementRecord* OtherMovement = GetCanonicalMovement(Other->JnctState.CanonicalMovementId);
		if (!OtherMovement || !OtherMovement->IsValid() || OtherMovement->JunctionId != JunctionId)
		{
			continue;
		}
		if (OtherMovement->MovementClass != ECanonicalMovementClass::Straight)
		{
			continue;
		}

		if (SelfMovement->ConflictMovementIds.Contains(OtherMovement->MovementId))
		{
			UE_LOG(LogAAATraffic, Log,
				TEXT("JNCT LEFT-TURN-YIELD: JunctionId=%d Self='%s' yielding to "
					 "approaching straight-through '%s' (Movement=%d From=%d To=%d Phase=%d)"),
				JunctionId,
				Self && Self->GetPawn() ? *Self->GetPawn()->GetName() : TEXT("NULL"),
				Other->GetPawn() ? *Other->GetPawn()->GetName() : TEXT("NULL"),
				OtherMovement->MovementId,
				OtherMovement->FromLane.HandleId,
				OtherMovement->ToLane.HandleId,
				(int32)Other->JnctState.Phase);
			return true;
		}
	}
	return false;
}

// ---------------------------------------------------------------------------
// Cross-Traffic Gap Acceptance — yield-sign approach safety check
// ---------------------------------------------------------------------------

bool UTrafficSubsystem::HasApproachingCrossTraffic(int32 JunctionId,
	ATrafficVehicleController* Self,
	int32 MovementId,
	float GapThresholdSec) const
{
	const FCanonicalMovementRecord* SelfMovement = GetCanonicalMovement(MovementId);
	if (!SelfMovement || !SelfMovement->IsValid() || SelfMovement->JunctionId != JunctionId)
	{
		return false;
	}

	for (const TWeakObjectPtr<ATrafficVehicleController>& WeakOther : ActiveVehicles)
	{
		ATrafficVehicleController* Other = WeakOther.Get();
		if (!Other || Other == Self) { continue; }

		// Check vehicles interacting with the same junction (approach or engaged).
		if (Other->JnctState.JunctionId != JunctionId) { continue; }

		// Skip vehicles that are yielding/stopped (not a threat).
		const APawn* OtherPawn = Other->GetPawn();
		if (!OtherPawn) { continue; }
		const UPawnMovementComponent* OtherMC = OtherPawn->GetMovementComponent();
		if (!OtherMC) { continue; }
		const float OtherSpeed = static_cast<float>(OtherMC->Velocity.Size());
		if (OtherSpeed < 100.0f) { continue; } // < 1 m/s — not a moving threat.

		const FCanonicalMovementRecord* OtherMovement = GetCanonicalMovement(Other->JnctState.CanonicalMovementId);
		if (!OtherMovement || !OtherMovement->IsValid() || OtherMovement->JunctionId != JunctionId)
		{
			continue;
		}

		if (!SelfMovement->ConflictMovementIds.Contains(OtherMovement->MovementId))
		{
			continue; // Non-conflicting paths — safe.
		}

		// TTC check: compute time for the other vehicle to reach the junction.
		const float OtherDist = Other->JnctState.ApproachDistanceCm;
		if (OtherDist <= 0.0f) { return true; } // Already at junction and conflicting.
		const float TTC = OtherDist / OtherSpeed;
		if (TTC < GapThresholdSec)
		{
			return true; // Cross-traffic too close.
		}
	}
	return false;
}

// ---------------------------------------------------------------------------
// Stop-Sign FIFO Queue — first-arrived-first-served ordering
// ---------------------------------------------------------------------------

void UTrafficSubsystem::RecordStopSignArrival(int32 JunctionId, ATrafficVehicleController* Controller)
{
	if (JunctionId == 0 || !Controller) { return; }

	TArray<FStopSignArrival>& Queue = StopSignQueues.FindOrAdd(JunctionId);

	// Purge stale entries.
	Queue.RemoveAll([](const FStopSignArrival& A) { return !A.Controller.IsValid(); });

	// Don't double-register.
	for (const FStopSignArrival& A : Queue)
	{
		if (A.Controller.Get() == Controller) { return; }
	}

	FStopSignArrival Entry;
	Entry.Controller = Controller;
	{
		const UWorld* W = GetWorld();
		Entry.ArrivalTime = W ? W->GetTimeSeconds() : 0.0;
	}
	Queue.Add(Entry);
}

bool UTrafficSubsystem::IsStopSignTurnToGo(int32 JunctionId, ATrafficVehicleController* Controller) const
{
	if (JunctionId == 0 || !Controller) { return true; }

	const TArray<FStopSignArrival>* Queue = StopSignQueues.Find(JunctionId);
	if (!Queue || Queue->IsEmpty()) { return true; }

	// Find the first valid entry — that vehicle goes first.
	for (const FStopSignArrival& A : *Queue)
	{
		if (A.Controller.IsValid())
		{
			return A.Controller.Get() == Controller;
		}
	}
	return true; // All entries stale.
}

void UTrafficSubsystem::RemoveStopSignArrival(int32 JunctionId, ATrafficVehicleController* Controller)
{
	if (JunctionId == 0 || !Controller) { return; }

	TArray<FStopSignArrival>* Queue = StopSignQueues.Find(JunctionId);
	if (!Queue) { return; }

	Queue->RemoveAll([Controller](const FStopSignArrival& A)
	{
		return !A.Controller.IsValid() || A.Controller.Get() == Controller;
	});
}

// ---------------------------------------------------------------------------
// Junction Exit Survey — centralized turn legality engine
// ---------------------------------------------------------------------------

const TArray<FJunctionExitRule>& UTrafficSubsystem::GetLegalExits(int32 ApproachLaneId) const
{
	if (const TArray<FJunctionExitRule>* Found = JunctionExitRules.Find(ApproachLaneId))
	{
		return *Found;
	}
	return EmptyExitRules;
}

void UTrafficSubsystem::BuildJunctionSurvey()
{
	ITrafficRoadProvider* Provider = GetProvider();
	if (!Provider)
	{
		UE_LOG(LogAAATraffic, Warning, TEXT("SURVEY: No provider — skipping junction survey."));
		return;
	}

	JunctionExitRules.Empty();
	ResetCanonicalMovementTable();

	// Discover all junctions in the road network.
	// Walk every road → every lane → GetJunctionForLane to find junction IDs.
	TSet<int32> JunctionIds;
	TArray<FTrafficRoadHandle> AllRoads = Provider->GetAllRoads();
	for (const FTrafficRoadHandle& Road : AllRoads)
	{
		TArray<FTrafficLaneHandle> RoadLanes = Provider->GetLanesForRoad(Road);
		for (const FTrafficLaneHandle& Lane : RoadLanes)
		{
			const int32 JId = Provider->GetJunctionForLane(Lane);
			if (JId != 0)
			{
				JunctionIds.Add(JId);
			}
		}
	}

	UE_LOG(LogAAATraffic, Log, TEXT("SURVEY: Starting junction survey — %d junctions found."), JunctionIds.Num());

	// Build a one-time map: JunctionId → approach lanes.
	// Single pass over all roads/lanes instead of rescanning per junction.
	TMap<int32, TArray<FTrafficLaneHandle>> JunctionToApproachLanes;
	for (const FTrafficRoadHandle& Road : AllRoads)
	{
		TArray<FTrafficLaneHandle> RoadLanes = Provider->GetLanesForRoad(Road);
		for (const FTrafficLaneHandle& Lane : RoadLanes)
		{
			if (Provider->GetJunctionForLane(Lane) != 0) { continue; }
			TArray<FTrafficLaneHandle> LaneConnected = Provider->GetConnectedLanes(Lane);
			for (const FTrafficLaneHandle& LC : LaneConnected)
			{
				const int32 ConnJId = Provider->GetJunctionForLane(LC);
				if (ConnJId == 0) { continue; }
				TArray<FTrafficLaneHandle>& ForJunction = JunctionToApproachLanes.FindOrAdd(ConnJId);
				bool bDup = false;
				for (const FTrafficLaneHandle& A : ForJunction)
				{
					if (A.HandleId == Lane.HandleId) { bDup = true; break; }
				}
				if (!bDup) { ForJunction.Add(Lane); }
			}
		}
	}

	int32 TotalRules = 0;
	TArray<FCanonicalMovementRecord> PendingCanonicalRecords;
	TMap<uint64, int32> PendingPairToIndex;
	int32 SkippedMovementCount = 0;

	for (const int32 JunctionId : JunctionIds)
	{
		TArray<FTrafficLaneHandle> JunctionLanes = Provider->GetLanesForJunction(JunctionId);

		// Gather all non-junction exit lanes reachable from this junction.
		// Dead-end exits (lanes that don't reach another junction) are included
		// with normal weight.  Vehicles assigned to them will drive there,
		// reach the road end, and be recycled by the dead-end despawn system.
		// The reachability pass already skips dead-end exits gracefully
		// (no downstream junction → continue, not a network trap).
		TArray<FTrafficLaneHandle> AllExitLanes;
		int32 TerminalExitCount = 0;
		for (const FTrafficLaneHandle& JL : JunctionLanes)
		{
			TArray<FTrafficLaneHandle> Connected = Provider->GetConnectedLanes(JL);
			for (const FTrafficLaneHandle& C : Connected)
			{
				if (Provider->GetJunctionForLane(C) == 0)
				{
					bool bDup = false;
					for (const FTrafficLaneHandle& E : AllExitLanes)
					{
						if (E.HandleId == C.HandleId) { bDup = true; break; }
					}
					if (!bDup)
					{
						AllExitLanes.Add(C);

						// Track terminal exits for diagnostics only.
						const TArray<FTrafficLaneHandle> Downstream = Provider->GetConnectedLanes(C);
						if (Downstream.IsEmpty())
						{
							++TerminalExitCount;
						}
						else
						{
							const float LaneLengthCm = Provider->GetLaneLength(C);
							const ITrafficRoadProvider::FJunctionScanResult Scan =
								Provider->GetDistanceToNextJunction(C, LaneLengthCm, 50000.0f, 10);
							if (!Scan.IsValid()) { ++TerminalExitCount; }
						}
					}
				}
			}
		}

		if (TerminalExitCount > 0)
		{
			UE_LOG(LogAAATraffic, Log,
				TEXT("SURVEY: Junction %d — %d of %d exit lanes are downstream-terminal (included; dead-end despawn handles recycling)."),
				JunctionId, TerminalExitCount, AllExitLanes.Num());
		}

		// Approach lanes from the precomputed map.
		const TArray<FTrafficLaneHandle>* PrecomputedApproach = JunctionToApproachLanes.Find(JunctionId);
		TArray<FTrafficLaneHandle> ApproachLanes;
		if (PrecomputedApproach)
		{
			ApproachLanes = *PrecomputedApproach;
		}

		UE_LOG(LogAAATraffic, Log,
			TEXT("SURVEY: Junction %d — %d junction lanes, %d approach lanes, %d exit lanes"),
			JunctionId, JunctionLanes.Num(), ApproachLanes.Num(), AllExitLanes.Num());

		// For each approach lane, determine which exits are legal.
		for (const FTrafficLaneHandle& Approach : ApproachLanes)
		{
			const float ApproachLen = Provider->GetLaneLength(Approach);
			const FVector ApproachDir = Provider->GetLaneDirectionAtDistance(Approach, ApproachLen);
			const FTrafficRoadHandle ApproachRoad = Provider->GetRoadForLane(Approach);

			// --- Lane-position analysis (multi-lane turn rules) ---
			const FVector ApproachRefDir = Provider->GetLaneDirection(Approach);
			int32 RightCount = 0;
			{
				FTrafficLaneHandle Walk = Provider->GetAdjacentLane(Approach, ETrafficLaneSide::Right);
				for (int32 S = 0; S < 8 && Walk.IsValid(); ++S)
				{
					if (FVector::DotProduct(ApproachRefDir, Provider->GetLaneDirection(Walk)) < 0.5f) { break; }
					++RightCount;
					Walk = Provider->GetAdjacentLane(Walk, ETrafficLaneSide::Right);
				}
			}
			int32 LeftCount = 0;
			{
				FTrafficLaneHandle Walk = Provider->GetAdjacentLane(Approach, ETrafficLaneSide::Left);
				for (int32 S = 0; S < 8 && Walk.IsValid(); ++S)
				{
					if (FVector::DotProduct(ApproachRefDir, Provider->GetLaneDirection(Walk)) < 0.5f) { break; }
					++LeftCount;
					Walk = Provider->GetAdjacentLane(Walk, ETrafficLaneSide::Left);
				}
			}

			const int32 TotalSameDir = RightCount + 1 + LeftCount;
			const int32 PosFromRight = RightCount;

			bool bAllowLeft = true;
			bool bAllowStraight = true;
			bool bAllowRight = true;

			if (TotalSameDir > 1)
			{
				bAllowLeft = false;
				bAllowRight = false;
				const float HalfN = static_cast<float>(TotalSameDir) * 0.5f;
				const float HalfNm1 = static_cast<float>(TotalSameDir - 1) * 0.5f;

				if (static_cast<float>(PosFromRight) < HalfN) { bAllowRight = true; }
				if (static_cast<float>(PosFromRight) > HalfNm1) { bAllowLeft = true; }
			}

			TArray<FJunctionExitRule> Rules;

			// Track same-road-opposing rejects for orphan-prevention fallback
			struct FSameRoadReject { FTrafficLaneHandle Exit; float Dot; };
			TArray<FSameRoadReject> SameRoadRejects;

			for (const FTrafficLaneHandle& Exit : AllExitLanes)
			{
				const FVector ExitDir = Provider->GetLaneDirection(Exit);
				const float Dot = FVector::DotProduct(ApproachDir, ExitDir);

				// --- Filter 1: U-turn (>120° turn) ---
				if (Dot < -0.5f && AllExitLanes.Num() > 1)
				{
					UE_LOG(LogAAATraffic, Log,
						TEXT("SURVEY:   Approach=%d Exit=%d REJECTED (U-turn, Dot=%.3f)"),
						Approach.HandleId, Exit.HandleId, Dot);
					continue;
				}

				// --- Filter 2: Same-road opposing direction ---
				const FTrafficRoadHandle ExitRoad = Provider->GetRoadForLane(Exit);
				if (ApproachRoad.IsValid() && ExitRoad.IsValid()
					&& ApproachRoad.HandleId == ExitRoad.HandleId
					&& Dot < 0.0f)
				{
					SameRoadRejects.Add({Exit, Dot});
					UE_LOG(LogAAATraffic, Log,
						TEXT("SURVEY:   Approach=%d Exit=%d REJECTED (same-road opposing, Dot=%.3f)"),
						Approach.HandleId, Exit.HandleId, Dot);
					continue;
				}

				// --- Compute turn direction ---
				const float CrossZ = FVector::CrossProduct(ApproachDir, ExitDir).Z;
				constexpr float StraightThreshold = 0.26f; // sin(15°)
				ETurnSignalState TurnDir = ETurnSignalState::Off;
				if (FMath::Abs(CrossZ) >= StraightThreshold)
				{
					TurnDir = (CrossZ > 0.0f) ? ETurnSignalState::Left : ETurnSignalState::Right;
				}

				// --- Filter 3: Lane-position turn permission (multi-lane roads) ---
				if (TotalSameDir > 1)
				{
					const bool bPermitted =
						(TurnDir == ETurnSignalState::Off && bAllowStraight) ||
						(TurnDir == ETurnSignalState::Left && bAllowLeft) ||
						(TurnDir == ETurnSignalState::Right && bAllowRight);
					if (!bPermitted)
					{
						UE_LOG(LogAAATraffic, Log,
							TEXT("SURVEY:   Approach=%d Exit=%d REJECTED (lane-position: Turn=%s, Allow L=%s S=%s R=%s)"),
							Approach.HandleId, Exit.HandleId,
							TurnDir == ETurnSignalState::Left ? TEXT("L") : (TurnDir == ETurnSignalState::Right ? TEXT("R") : TEXT("S")),
							bAllowLeft ? TEXT("Y") : TEXT("N"),
							bAllowStraight ? TEXT("Y") : TEXT("N"),
							bAllowRight ? TEXT("Y") : TEXT("N"));
						continue;
					}
				}

				// --- Filter 4: Physical feasibility (junction path turn radius) ---
				// The curve generator already widens tight curves via tangent
				// scaling. This gate only rejects turns that are STILL too tight
				// after widening — i.e. genuinely impossible geometry.
				// Use a conservative threshold (200cm) well below the vehicle's
				// physical minimum (~400cm) to avoid false positives that would
				// leave vehicles with no routes at small intersections.
				bool bPhysicallyFeasible = true;
				{
					TArray<FVector> JunctionPath;
					if (Provider->GetJunctionPath(Approach, Exit, JunctionPath) && JunctionPath.Num() >= 3)
					{
						const float MeasuredMinRadius = ComputeMinTurnRadiusCm(JunctionPath);

						// 200cm is roughly half the vehicle's min turning radius.
						// Turns this tight are genuinely impossible even at a crawl.
						// Orphan-prevention below ensures that if this is the ONLY
						// exit for the approach, it stays alive to prevent a
						// network-wide reachability cascade.
						constexpr float HardInfeasibleRadiusCm = 200.0f;

						if (MeasuredMinRadius > 0.0f && MeasuredMinRadius < HardInfeasibleRadiusCm)
						{
							bPhysicallyFeasible = false;
							if (GTrafficJunctionDiagnostics >= 1)
							{
								UE_LOG(LogAAATraffic, Log,
									TEXT("SURVEY:   Approach=%d Exit=%d INFEASIBLE (MeasuredMinR=%.0f < HardLimit=%.0f)"),
									Approach.HandleId, Exit.HandleId, MeasuredMinRadius, HardInfeasibleRadiusCm);
							}
						}
					}
				}

				// --- Compute weight ---
				const float Weight = FMath::Max(FMath::Sqrt(FMath::Max(Dot + 1.0f, 0.0f)), 0.01f);

				FJunctionExitRule Rule;
				Rule.ExitLane = Exit;
				Rule.TurnDirection = TurnDir;
				Rule.Weight = bPhysicallyFeasible ? Weight : 0.0f;
				Rule.bPhysicallyFeasible = bPhysicallyFeasible;
				Rules.Add(Rule);

				if (GTrafficJunctionDiagnostics >= 1)
				{
					UE_LOG(LogAAATraffic, Log,
						TEXT("SURVEY:   Approach=%d Exit=%d Turn=%s Dot=%.3f W=%.3f Feasible=%s"),
						Approach.HandleId, Exit.HandleId,
						TurnDir == ETurnSignalState::Left ? TEXT("L") : (TurnDir == ETurnSignalState::Right ? TEXT("R") : TEXT("S")),
						Dot, Rule.Weight, bPhysicallyFeasible ? TEXT("Y") : TEXT("N"));
				}
			}

			// Safety: if all rules ended up with Weight=0 (typically because
			// every exit was marked physically infeasible), re-admit the
			// least-bad exit so the approach is not orphaned.  Without
			// this, the reachability pass cascades the dead-end through
			// the entire network.
			bool bAllZero = true;
			for (const FJunctionExitRule& R : Rules)
			{
				if (R.Weight > 0.0f) { bAllZero = false; break; }
			}
			if (bAllZero && Rules.Num() > 0)
			{
				// Find the rule with the largest measured radius (least
				// infeasible) and re-admit it as feasible with low weight.
				// This keeps the approach connected while still penalizing
				// the turn versus better alternatives (if any appear later).
				int32 BestIdx = 0;
				float BestRadius = -1.0f;
				for (int32 Ri = 0; Ri < Rules.Num(); ++Ri)
				{
					TArray<FVector> ProbeJunctionPath;
					if (Provider->GetJunctionPath(Approach, Rules[Ri].ExitLane, ProbeJunctionPath)
						&& ProbeJunctionPath.Num() >= 3)
					{
						const float R = ComputeMinTurnRadiusCm(ProbeJunctionPath);
						if (R > BestRadius) { BestRadius = R; BestIdx = Ri; }
					}
				}
				Rules[BestIdx].Weight = 0.05f;
				Rules[BestIdx].bPhysicallyFeasible = true;
				UE_LOG(LogAAATraffic, Warning,
					TEXT("SURVEY:   Approach=%d ORPHAN-PREVENTION — re-admitted Exit=%d (R=%.0f) as feasible fallback to prevent cascade"),
					Approach.HandleId, Rules[BestIdx].ExitLane.HandleId, BestRadius);
			}

			// Diagnostic: if all exit candidates were rejected by filters, this
			// approach lane is orphaned — it connects to a junction but has no
			// legal/feasible exits. Log prominently so road topology issues are
			// visible at startup.
			if (Rules.IsEmpty())
			{
				if (SameRoadRejects.Num() > 0)
				{
					// Orphan-prevention: re-admit same-road-opposing exits with minimal
					// weight when they are the ONLY option. This prevents network-wide
					// dead-end cascades while still penalizing U-turns when better
					// alternatives exist.
					for (const FSameRoadReject& Rej : SameRoadRejects)
					{
						FJunctionExitRule Rule;
						Rule.ExitLane = Rej.Exit;
						Rule.TurnDirection = ETurnSignalState::Off;
						Rule.Weight = 0.05f;
						Rule.bPhysicallyFeasible = true;
						Rules.Add(Rule);
					}
					UE_LOG(LogAAATraffic, Warning,
						TEXT("SURVEY:   Approach=%d ORPHAN-PREVENTION — re-admitted %d same-road-opposing exits as low-weight fallbacks"),
						Approach.HandleId, SameRoadRejects.Num());
				}
				else
				{
					UE_LOG(LogAAATraffic, Warning,
						TEXT("SURVEY:   Approach=%d ORPHANED — connected to junction but all %d exit candidates were rejected by filters"),
						Approach.HandleId, AllExitLanes.Num());
				}
			}

			TotalRules += Rules.Num();
			JunctionExitRules.Add(Approach.HandleId, MoveTemp(Rules));
		}
	}

	TArray<int32> SortedJunctionIds = JunctionIds.Array();
	SortedJunctionIds.Sort();
	for (const int32 JunctionId : SortedJunctionIds)
	{
		const TArray<FTrafficLaneHandle>* ApproachLanes = JunctionToApproachLanes.Find(JunctionId);
		if (!ApproachLanes)
		{
			continue;
		}

		TArray<FTrafficLaneHandle> SortedApproachLanes = *ApproachLanes;
		SortedApproachLanes.Sort([](const FTrafficLaneHandle& A, const FTrafficLaneHandle& B)
		{
			return A.HandleId < B.HandleId;
		});

		for (const FTrafficLaneHandle& ApproachLane : SortedApproachLanes)
		{
			const TArray<FJunctionExitRule>* Rules = JunctionExitRules.Find(ApproachLane.HandleId);
			if (!Rules)
			{
				continue;
			}

			FTrafficLaneHandle ApproachJunctionLane = ResolveApproachJunctionLane(Provider, ApproachLane, JunctionId);
			if (!ApproachJunctionLane.IsValid())
			{
				UE_LOG(LogAAATraffic, Error,
					TEXT("CANONICAL COMPILE: Junction=%d From=%d -- skipped because approach junction lane could not be resolved"),
					JunctionId,
					ApproachLane.HandleId);
				++SkippedMovementCount;
				continue;
			}

			TArray<FJunctionExitRule> SortedRules = *Rules;
			SortedRules.Sort([](const FJunctionExitRule& A, const FJunctionExitRule& B)
			{
				return A.ExitLane.HandleId < B.ExitLane.HandleId;
			});

			for (const FJunctionExitRule& Rule : SortedRules)
			{
				FCanonicalMovementRecord Record;
				FString FailureReason;
				if (!BuildCanonicalMovementRecord(Provider, JunctionId, ApproachLane, ApproachJunctionLane, Rule, Record, FailureReason))
				{
					UE_LOG(LogAAATraffic, Error,
						TEXT("CANONICAL COMPILE: Junction=%d From=%d To=%d -- skipped (%s)"),
						JunctionId,
						ApproachLane.HandleId,
						Rule.ExitLane.HandleId,
						*FailureReason);
					++SkippedMovementCount;
					continue;
				}

				const uint64 PairKey = (static_cast<uint64>(static_cast<uint32>(Record.FromLane.HandleId)) << 32)
					| static_cast<uint32>(Record.ToLane.HandleId);
				if (PendingPairToIndex.Contains(PairKey))
				{
					UE_LOG(LogAAATraffic, Error,
						TEXT("CANONICAL COMPILE: Junction=%d From=%d To=%d -- duplicate movement pair skipped"),
						JunctionId,
						Record.FromLane.HandleId,
						Record.ToLane.HandleId);
					++SkippedMovementCount;
					continue;
				}

				PendingPairToIndex.Add(PairKey, PendingCanonicalRecords.Num());
				PendingCanonicalRecords.Add(MoveTemp(Record));
			}
		}
	}

	PendingCanonicalRecords.Sort([](const FCanonicalMovementRecord& A, const FCanonicalMovementRecord& B)
	{
		if (A.JunctionId != B.JunctionId)
		{
			return A.JunctionId < B.JunctionId;
		}
		if (A.FromLane.HandleId != B.FromLane.HandleId)
		{
			return A.FromLane.HandleId < B.FromLane.HandleId;
		}
		return A.ToLane.HandleId < B.ToLane.HandleId;
	});

	for (int32 RecordIndex = 0; RecordIndex < PendingCanonicalRecords.Num(); ++RecordIndex)
	{
		FCanonicalMovementRecord& Record = PendingCanonicalRecords[RecordIndex];
		Record.MovementId = RecordIndex + 1;
		CanonicalMovementTable.Add(Record.MovementId, Record);
		JunctionToMovementIds.FindOrAdd(Record.JunctionId).Add(Record.MovementId);
		ApproachLaneToMovementIds.FindOrAdd(Record.FromLane.HandleId).Add(Record.MovementId);
	}

	for (auto& Pair : JunctionToMovementIds)
	{
		Pair.Value.Sort();
	}
	for (auto& Pair : ApproachLaneToMovementIds)
	{
		Pair.Value.Sort();
	}

	for (const int32 JunctionId : SortedJunctionIds)
	{
		TArray<int32>* MovementIds = JunctionToMovementIds.Find(JunctionId);
		if (!MovementIds || MovementIds->IsEmpty())
		{
			UE_LOG(LogAAATraffic, Error,
				TEXT("CANONICAL COMPILE: Junction=%d compiled to zero valid movements"),
				JunctionId);
			continue;
		}

		for (int32 IndexA = 0; IndexA < MovementIds->Num(); ++IndexA)
		{
			FCanonicalMovementRecord* RecordA = CanonicalMovementTable.Find((*MovementIds)[IndexA]);
			if (!RecordA)
			{
				continue;
			}

			for (int32 IndexB = IndexA + 1; IndexB < MovementIds->Num(); ++IndexB)
			{
				FCanonicalMovementRecord* RecordB = CanonicalMovementTable.Find((*MovementIds)[IndexB]);
				if (!RecordB)
				{
					continue;
				}

				if (!Provider->DoJunctionPathsConflict(RecordA->FromLane, RecordA->ToLane, RecordB->FromLane, RecordB->ToLane))
				{
					continue;
				}

				RecordA->ConflictMovementIds.Add(RecordB->MovementId);
				RecordB->ConflictMovementIds.Add(RecordA->MovementId);
			}
		}

		for (const int32 MovementId : *MovementIds)
		{
			if (FCanonicalMovementRecord* Record = CanonicalMovementTable.Find(MovementId))
			{
				SortAndUniqueIds(Record->ConflictMovementIds);
				Record->ValidationFlags |= CanonicalFlagHasConflictSet;
			}
		}
	}

	// ═══════════════════════════════════════════════════════════════
	// MULTI-HOP REACHABILITY PASS
	// For each canonical movement, verify that its ToLane has at least
	// one canonical movement at its downstream junction. If not, the
	// movement leads to a dead end in the network — zero its weight.
	// Iterate to a fixpoint so chains of dead-end exits are propagated.
	// ═══════════════════════════════════════════════════════════════
	{
		int32 ReachabilityPass = 0;
		int32 TotalCulled = 0;
		bool bChanged = true;
		while (bChanged)
		{
			bChanged = false;
			++ReachabilityPass;

			for (auto& MovementPair : CanonicalMovementTable)
			{
				FCanonicalMovementRecord& Record = MovementPair.Value;
				if (Record.SelectionWeight <= 0.0f)
				{
					continue; // Already culled
				}

				// Check if ToLane is an approach lane with surviving movements
				const TArray<int32>* DownstreamMovements = ApproachLaneToMovementIds.Find(Record.ToLane.HandleId);
				if (!DownstreamMovements || DownstreamMovements->IsEmpty())
				{
					// ToLane is not an approach lane — check if it has onward connectivity
					// that eventually reaches an approach lane (non-junction roads without
					// a junction ahead are valid terminal segments — don't cull those)
					const ITrafficRoadProvider::FJunctionScanResult Scan =
						Provider->GetDistanceToNextJunction(
							Record.ToLane,
							Provider->GetLaneLength(Record.ToLane),
							50000.0f, 10);
					if (!Scan.IsValid())
					{
						continue; // No downstream junction — not a network trap, just a terminal road
					}

					// There IS a downstream junction, but this lane has no canonical
					// movements there — it's a reachability trap
					Record.SelectionWeight = 0.0f;
					++TotalCulled;
					bChanged = true;

					UE_LOG(LogAAATraffic, Warning,
						TEXT("REACHABILITY: Movement=%d Junction=%d From=%d To=%d — "
							 "exit lane reaches Junction=%d but has no canonical movements there (pass %d)"),
						Record.MovementId, Record.JunctionId,
						Record.FromLane.HandleId, Record.ToLane.HandleId,
						Scan.JunctionId, ReachabilityPass);
					continue;
				}

				// Check if ANY downstream movement has non-zero weight
				bool bHasViableDownstream = false;
				for (const int32 DownId : *DownstreamMovements)
				{
					const FCanonicalMovementRecord* DownRecord = CanonicalMovementTable.Find(DownId);
					if (DownRecord && DownRecord->SelectionWeight > 0.0f)
					{
						bHasViableDownstream = true;
						break;
					}
				}

				if (!bHasViableDownstream)
				{
					// Before zeroing, check if this is the LAST viable movement
					// for its approach lane. If so, preserve it to prevent
					// orphaning the approach and cascading through the network.
					const TArray<int32>* ApproachMovements =
						ApproachLaneToMovementIds.Find(Record.FromLane.HandleId);
					if (ApproachMovements)
					{
						int32 ViableSiblings = 0;
						for (const int32 SibId : *ApproachMovements)
						{
							if (SibId == Record.MovementId) { continue; }
							const FCanonicalMovementRecord* Sib =
								CanonicalMovementTable.Find(SibId);
							if (Sib && Sib->SelectionWeight > 0.0f)
							{
								++ViableSiblings;
							}
						}
						if (ViableSiblings == 0)
						{
							UE_LOG(LogAAATraffic, Warning,
								TEXT("REACHABILITY: Movement=%d Junction=%d From=%d To=%d — "
									 "PRESERVED (last viable movement for approach, pass %d)"),
								Record.MovementId, Record.JunctionId,
								Record.FromLane.HandleId, Record.ToLane.HandleId,
								ReachabilityPass);
							continue; // Do NOT zero — would orphan this approach
						}
					}

					Record.SelectionWeight = 0.0f;
					++TotalCulled;
					bChanged = true;

					UE_LOG(LogAAATraffic, Warning,
						TEXT("REACHABILITY: Movement=%d Junction=%d From=%d To=%d — "
							 "all downstream movements from exit lane have zero weight (pass %d)"),
						Record.MovementId, Record.JunctionId,
						Record.FromLane.HandleId, Record.ToLane.HandleId,
						ReachabilityPass);
				}
			}

			// Safety: prevent infinite loops on degenerate networks
			if (ReachabilityPass > 100)
			{
				UE_LOG(LogAAATraffic, Error,
					TEXT("REACHABILITY: Fixpoint not reached after 100 passes — aborting. %d movements culled so far."),
					TotalCulled);
				break;
			}
		}

		if (TotalCulled > 0)
		{
			UE_LOG(LogAAATraffic, Warning,
				TEXT("REACHABILITY: Culled %d unreachable movements in %d passes."),
				TotalCulled, ReachabilityPass);
		}
		else
		{
			UE_LOG(LogAAATraffic, Log,
				TEXT("REACHABILITY: All movements have viable downstream paths (verified in %d pass)."),
				ReachabilityPass);
		}
	}

	// ═══════════════════════════════════════════════════════════════
	// POST-COMPILE INTEGRITY REPORT
	// Emit structured diagnostics so broken road setups are visible
	// at startup, not at runtime when a vehicle gets stuck.
	// ═══════════════════════════════════════════════════════════════
	{
		int32 OrphanedApproachCount = 0;
		int32 ZeroMovementJunctionCount = 0;
		int32 AllCulledJunctionCount = 0;
		int32 DegenerateCorridorCount = 0;
		int32 SynthesizedCorridorCount = 0;
		int32 IncompleteValidationCount = 0;

		constexpr uint32 FullValidationMask =
			CanonicalFlagHasProviderPath
			| CanonicalFlagHasEntryAttach
			| CanonicalFlagHasExitAttach
			| CanonicalFlagHasReleaseIndex
			| CanonicalFlagHasResumeIndex
			| CanonicalFlagHasConflictSet;

		// Check for orphaned approach lanes (approach has connected junction
		// lanes but ended up with zero exit rules due to filter rejections)
		for (const auto& ApproachPair : JunctionToApproachLanes)
		{
			for (const FTrafficLaneHandle& Approach : ApproachPair.Value)
			{
				const TArray<int32>* MovementIds = ApproachLaneToMovementIds.Find(Approach.HandleId);
				if (!MovementIds || MovementIds->IsEmpty())
				{
					++OrphanedApproachCount;
					UE_LOG(LogAAATraffic, Warning,
						TEXT("INTEGRITY: ORPHANED-APPROACH — Lane=%d at Junction=%d has no canonical movements "
							 "(all exits were rejected by survey filters)"),
						Approach.HandleId, ApproachPair.Key);
				}
			}
		}

		for (const int32 JunctionId : SortedJunctionIds)
		{
			const TArray<int32>* MovementIds = JunctionToMovementIds.Find(JunctionId);
			if (!MovementIds || MovementIds->IsEmpty())
			{
				++ZeroMovementJunctionCount;
				// Already logged as Error during conflict detection
				continue;
			}

			// Check if all surviving movements at this junction have zero weight
			bool bAllZeroWeight = true;
			for (const int32 MovementId : *MovementIds)
			{
				const FCanonicalMovementRecord* Record = CanonicalMovementTable.Find(MovementId);
				if (Record && Record->SelectionWeight > 0.0f)
				{
					bAllZeroWeight = false;
					break;
				}
			}
			if (bAllZeroWeight)
			{
				++AllCulledJunctionCount;
				UE_LOG(LogAAATraffic, Warning,
					TEXT("INTEGRITY: ALL-CULLED-JUNCTION — Junction=%d has %d movements but all have zero weight after reachability cull"),
					JunctionId, MovementIds->Num());
			}
		}

		// Per-movement diagnostics
		for (const auto& MovementPair : CanonicalMovementTable)
		{
			const FCanonicalMovementRecord& Record = MovementPair.Value;

			// Degenerate corridors (very short arc or too few points)
			if (Record.CorridorArcLengthCm < 50.0f || Record.CorridorPoints.Num() < 3)
			{
				++DegenerateCorridorCount;
				UE_LOG(LogAAATraffic, Warning,
					TEXT("INTEGRITY: DEGENERATE-CORRIDOR — Movement=%d Junction=%d From=%d To=%d Arc=%.0f Pts=%d "
						 "(may cause TRAVERSE-NO-OCCUPANCY if curve commits before occupancy grant)"),
					Record.MovementId, Record.JunctionId,
					Record.FromLane.HandleId, Record.ToLane.HandleId,
					Record.CorridorArcLengthCm, Record.CorridorPoints.Num());
			}

			if (Record.SourceKind == ECanonicalMovementSourceKind::SynthesizedDuringCompile)
			{
				++SynthesizedCorridorCount;
			}

			// Check provider-path bit separately since synthesized corridors won't have it
			const uint32 RequiredNonProviderFlags =
				CanonicalFlagHasEntryAttach
				| CanonicalFlagHasExitAttach
				| CanonicalFlagHasReleaseIndex
				| CanonicalFlagHasResumeIndex
				| CanonicalFlagHasConflictSet;
			if ((Record.ValidationFlags & RequiredNonProviderFlags) != RequiredNonProviderFlags)
			{
				++IncompleteValidationCount;
				UE_LOG(LogAAATraffic, Warning,
					TEXT("INTEGRITY: INCOMPLETE-VALIDATION — Movement=%d Junction=%d Flags=0x%08x (missing required bits from 0x%08x)"),
					Record.MovementId, Record.JunctionId,
					Record.ValidationFlags, RequiredNonProviderFlags);
			}
		}

		UE_LOG(LogAAATraffic, Log,
			TEXT("INTEGRITY REPORT: Junctions=%d Movements=%d | Orphaned=%d ZeroMovementJunctions=%d AllCulled=%d Degenerate=%d Synthesized=%d IncompleteFlags=%d"),
			SortedJunctionIds.Num(),
			CanonicalMovementTable.Num(),
			OrphanedApproachCount,
			ZeroMovementJunctionCount,
			AllCulledJunctionCount,
			DegenerateCorridorCount,
			SynthesizedCorridorCount,
			IncompleteValidationCount);
	}

	UE_LOG(LogAAATraffic, Log,
		TEXT("SURVEY: Junction survey complete — %d approach lanes, %d total exit rules."),
		JunctionExitRules.Num(), TotalRules);

	UE_LOG(LogAAATraffic, Log,
		TEXT("SURVEY: Canonical movement compile complete — %d movement records built, %d skipped."),
		CanonicalMovementTable.Num(),
		SkippedMovementCount);

	for (const int32 JunctionId : SortedJunctionIds)
	{
		const TArray<int32>& MovementIds = GetCanonicalMovementsForJunction(JunctionId);
		for (const int32 MovementId : MovementIds)
		{
			const FCanonicalMovementRecord* Record = GetCanonicalMovement(MovementId);
			if (!Record)
			{
				continue;
			}

			if (GTrafficJunctionDiagnostics >= 1)
			{
				UE_LOG(LogAAATraffic, Log,
					TEXT("CANONICAL TABLE: Junction=%d Movement=%d From=%d To=%d Turn=%d Class=%d Path=%d Arc=%.0f Radius=%.0f EntryAttach=%d ExitAttach=%d Release=%d Resume=%d Conflicts=[%s] Flags=0x%08x"),
					Record->JunctionId,
					Record->MovementId,
					Record->FromLane.HandleId,
					Record->ToLane.HandleId,
					(int32)Record->TurnDirection,
					(int32)Record->MovementClass,
					Record->CorridorPoints.Num(),
					Record->CorridorArcLengthCm,
					Record->MinTurnRadiusCm,
					Record->EntryLaneAttachIndex,
					Record->ExitLaneAttachIndex,
					Record->TraversalReleaseIndex,
					Record->ExitLaneResumeIndex,
					*FString::JoinBy(Record->ConflictMovementIds, TEXT(","), [](int32 Value) { return FString::FromInt(Value); }),
					Record->ValidationFlags);
			}
		}
	}
}
