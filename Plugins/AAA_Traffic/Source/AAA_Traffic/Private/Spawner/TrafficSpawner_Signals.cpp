// Copyright AAA_Traffic Contributors. All Rights Reserved.
// TrafficSpawner_Signals.cpp — Automatic traffic signal/stop-sign placement.

#include "TrafficSpawner.h"
#include "TrafficSubsystem.h"
#include "TrafficRoadProvider.h"
#include "TrafficSignalController.h"
#include "TrafficLog.h"
#include "Engine/World.h"

extern int32 GTrafficJunctionDiagnostics;

void ATrafficSpawner::PlaceAutoSignals(UWorld* World, ITrafficRoadProvider* Provider,
	const TArray<FTrafficLaneHandle>& AllLanes)
{
	if (!bAutoPlaceSignals) { return; }

	const bool bSignalDiag = (GTrafficJunctionDiagnostics >= 1);

	UTrafficSubsystem* TrafficSub = World ? World->GetSubsystem<UTrafficSubsystem>() : nullptr;

	// Discover unique junction IDs from lane data.
	TMap<int32, TArray<FTrafficLaneHandle>> JunctionLanes;
	for (const FTrafficLaneHandle& Lane : AllLanes)
	{
		const int32 JId = Provider->GetJunctionForLane(Lane);
		if (JId != 0)
		{
			JunctionLanes.FindOrAdd(JId).Add(Lane);
		}
	}

	if (JunctionLanes.IsEmpty()) { return; }

	// Sort junction IDs for deterministic processing (System.md §4.4).
	TArray<int32> JunctionIds;
	JunctionLanes.GetKeys(JunctionIds);
	JunctionIds.Sort();

	int32 SignalsPlaced = 0;
	for (const int32 JId : JunctionIds)
	{
		// Skip junctions that already have a manually-placed signal controller.
		if (TrafficSub && TrafficSub->GetSignalForJunction(JId))
		{
			continue;
		}

		const TArray<FTrafficLaneHandle>& Lanes = JunctionLanes[JId];

		// Group incoming lanes by road for phase groups.
		TMap<int32, TArray<FTrafficLaneHandle>> LanesByRoad;
		for (const FTrafficLaneHandle& Lane : Lanes)
		{
			const FTrafficRoadHandle Road = Provider->GetRoadForLane(Lane);
			LanesByRoad.FindOrAdd(Road.HandleId).Add(Lane);
		}

		// --- Use deferred spawn so JunctionId and PhaseGroups are set BEFORE
		// BeginPlay fires. Previously, SpawnActor triggered BeginPlay immediately,
		// which saw JunctionId==0, logged a warning, and disabled the signal's tick.
		FTransform SignalTransform = FTransform::Identity;
		ATrafficSignalController* Signal = World->SpawnActorDeferred<ATrafficSignalController>(
			ATrafficSignalController::StaticClass(), SignalTransform);
		if (!Signal) { continue; }

		Signal->JunctionId = JId;

		// Auto-classify by APPROACH ARM count (direction-clustered).
		// On a 2-lane bidirectional road both lanes share the same road
		// ID, so grouping by road under-counts.  Instead count unique
		// approach directions across all junction lanes.
		TArray<FVector> ApproachArmDirs;
		for (const FTrafficLaneHandle& JLane : Lanes)
		{
			const FVector Dir = Provider->GetLaneDirection(JLane);
			bool bMerged = false;
			for (const FVector& Existing : ApproachArmDirs)
			{
				if (FVector::DotProduct(Dir, Existing) > 0.7f) // ~45° merge threshold
				{
					bMerged = true;
					break;
				}
			}
			if (!bMerged)
			{
				ApproachArmDirs.Add(Dir);
			}
			if (bSignalDiag)
			{
				UE_LOG(LogAAATraffic, Warning,
					TEXT("SIGNAL-DIAG: JunctionId=%d ArmScan Lane=%d Dir=(%.3f, %.3f, %.3f) Merged=%s ArmsSoFar=%d"),
					JId, JLane.HandleId, Dir.X, Dir.Y, Dir.Z, bMerged ? TEXT("Y") : TEXT("N"), ApproachArmDirs.Num());
			}
		}
		const int32 ApproachArmCount = ApproachArmDirs.Num();

		// First, check for roundabout topology: if a junction's internal
		// lanes form a cycle (can follow connected lanes back to the start),
		// it's a roundabout. Roundabouts use Yield mode — entering traffic
		// yields to vehicles already in the circle.
		bool bIsRoundabout = false;
		if (ApproachArmCount >= 3)
		{
			// Detect cycle: for each junction lane, follow connected lanes
			// that stay within the same junction. If we can return to the
			// starting lane within MaxHops, it's circular.
			for (const FTrafficLaneHandle& StartLane : Lanes)
			{
				FTrafficLaneHandle Walk = StartLane;
				bool bCycle = false;
				for (int32 Hop = 0; Hop < 16; ++Hop)
				{
					TArray<FTrafficLaneHandle> WalkExits = Provider->GetConnectedLanes(Walk);
					FTrafficLaneHandle NextInJunction;
					for (const FTrafficLaneHandle& E : WalkExits)
					{
						if (Provider->GetJunctionForLane(E) == JId && E.HandleId != Walk.HandleId)
						{
							NextInJunction = E;
							break;
						}
					}
					if (!NextInJunction.IsValid()) { break; }
					Walk = NextInJunction;
					if (Walk.HandleId == StartLane.HandleId)
					{
						bCycle = true;
						break;
					}
				}
				if (bCycle) { bIsRoundabout = true; break; }
			}
		}

		if (bIsRoundabout)
		{
			Signal->ControlMode = EJunctionControlMode::Yield;
			UE_LOG(LogAAATraffic, Log,
				TEXT("PlaceAutoSignals: JunctionId=%d detected as ROUNDABOUT (%d approach arms, circular topology) → Yield"),
				JId, ApproachArmCount);
		}
		else if (ApproachArmCount >= 4)
		{
			Signal->ControlMode = EJunctionControlMode::Signal;
			if (bSignalDiag)
			{
				UE_LOG(LogAAATraffic, Warning,
					TEXT("SIGNAL-DIAG: JunctionId=%d ApproachArmCount=%d → Signal (4+ arms)"),
					JId, ApproachArmCount);
			}
		}
		else if (ApproachArmCount == 3)
		{
			Signal->ControlMode = EJunctionControlMode::StopSign;
			if (bSignalDiag)
			{
				UE_LOG(LogAAATraffic, Warning,
					TEXT("SIGNAL-DIAG: JunctionId=%d ApproachArmCount=%d → StopSign (3 arms)"),
					JId, ApproachArmCount);
			}
		}
		else
		{
			// For 2-arm junctions, examine actual turning angles between
			// approach and exit lanes to distinguish gentle curves from
			// genuine T-junctions that need stop control.
			float MaxTurnAngleDeg = 0.0f;
			for (const FTrafficLaneHandle& JLane : Lanes)
			{
				TArray<FTrafficLaneHandle> Exits = Provider->GetConnectedLanes(JLane);
				if (Exits.Num() == 0) { continue; }
				const float JLaneLen = Provider->GetLaneLength(JLane);
				const FVector ApproachDir = Provider->GetLaneDirectionAtDistance(JLane, JLaneLen);
				for (const FTrafficLaneHandle& Exit : Exits)
				{
					const FVector ExitDir = Provider->GetLaneDirection(Exit);
					const float Dot = FMath::Clamp(FVector::DotProduct(ApproachDir, ExitDir), -1.0f, 1.0f);
					const float AngleDeg = FMath::RadiansToDegrees(FMath::Acos(Dot));
					MaxTurnAngleDeg = FMath::Max(MaxTurnAngleDeg, AngleDeg);
				}
			}
			Signal->ControlMode = (MaxTurnAngleDeg > 45.0f)
				? EJunctionControlMode::StopSign
				: EJunctionControlMode::Yield;
			if (bSignalDiag)
			{
				UE_LOG(LogAAATraffic, Warning,
					TEXT("SIGNAL-DIAG: JunctionId=%d ApproachArmCount=%d MaxTurnAngle=%.1f → %s (2-arm)"),
					JId, ApproachArmCount, MaxTurnAngleDeg,
					MaxTurnAngleDeg > 45.0f ? TEXT("StopSign") : TEXT("Yield"));
			}
		}

		// Build phase groups: cluster opposing-direction roads into the
		// same phase so a standard 4-way gets 2 phases (N+S green
		// together, E+W green together) instead of 4.
		//
		// Algorithm: compute the average approach direction per road,
		// then greedily merge roads whose directions are nearly opposite
		// (dot < -0.3).
		TArray<int32> RoadKeys;
		LanesByRoad.GetKeys(RoadKeys);
		RoadKeys.Sort();

		// Compute average approach direction per road.
		TMap<int32, FVector> RoadApproachDir;
		for (const int32 RoadId : RoadKeys)
		{
			FVector DirSum = FVector::ZeroVector;
			for (const FTrafficLaneHandle& Lane : LanesByRoad[RoadId])
			{
				const float Len = Provider->GetLaneLength(Lane);
				const FVector LaneDir = Provider->GetLaneDirectionAtDistance(Lane, Len);
				DirSum += LaneDir;
				if (bSignalDiag)
				{
					UE_LOG(LogAAATraffic, Warning,
						TEXT("SIGNAL-DIAG: JunctionId=%d Road=%d Lane=%d EndDir=(%.3f, %.3f, %.3f)"),
						JId, RoadId, Lane.HandleId, LaneDir.X, LaneDir.Y, LaneDir.Z);
				}
			}
			const FVector Normalized = DirSum.GetSafeNormal();
			RoadApproachDir.Add(RoadId, Normalized);
			if (bSignalDiag)
			{
				UE_LOG(LogAAATraffic, Warning,
					TEXT("SIGNAL-DIAG: JunctionId=%d Road=%d DirSum=(%.3f, %.3f, %.3f) Normalized=(%.3f, %.3f, %.3f) LaneCount=%d"),
					JId, RoadId, DirSum.X, DirSum.Y, DirSum.Z, Normalized.X, Normalized.Y, Normalized.Z, LanesByRoad[RoadId].Num());
			}
		}

		// Greedy clustering: walk through roads and pair opposing ones.
		TArray<TArray<int32>> RoadClusters;
		TSet<int32> Assigned;
		for (int32 i = 0; i < RoadKeys.Num(); ++i)
		{
			if (Assigned.Contains(RoadKeys[i])) { continue; }
			TArray<int32> Cluster;
			Cluster.Add(RoadKeys[i]);
			Assigned.Add(RoadKeys[i]);
			const FVector& DirA = RoadApproachDir[RoadKeys[i]];
			// Find opposing roads (dot < -0.3 → facing each other).
			for (int32 j = i + 1; j < RoadKeys.Num(); ++j)
			{
				if (Assigned.Contains(RoadKeys[j])) { continue; }
				const FVector& DirB = RoadApproachDir[RoadKeys[j]];
				if (FVector::DotProduct(DirA, DirB) < -0.3f)
				{
					Cluster.Add(RoadKeys[j]);
					Assigned.Add(RoadKeys[j]);
				}
			}
			RoadClusters.Add(MoveTemp(Cluster));
		}

		if (bSignalDiag)
		{
			UE_LOG(LogAAATraffic, Warning,
				TEXT("SIGNAL-DIAG: JunctionId=%d Clustering complete — %d clusters from %d roads"),
				JId, RoadClusters.Num(), RoadKeys.Num());
		}

		// Build one phase group per cluster.
		Signal->PhaseGroups.Empty();
		for (const TArray<int32>& Cluster : RoadClusters)
		{
			FSignalPhaseGroup Group;
			TArray<FString> RoadNames;
			for (const int32 RoadId : Cluster)
			{
				RoadNames.Add(FString::Printf(TEXT("Road_%d"), RoadId));
				Group.GreenLanes.Append(LanesByRoad[RoadId]);
			}
			Group.GroupName = FString::Join(RoadNames, TEXT("+"));
			Signal->PhaseGroups.Add(MoveTemp(Group));
		}

		// ── Protected left-turn phases ──────────────────────────
		// For Signal-mode junctions, identify lanes that have left-turn
		// exits and create a dedicated arrow phase before each through-
		// phase. The arrow phase gets a shorter green (10s default).
		// This prevents left-turners from having to yield to through-
		// traffic — they get their own exclusive green.
		if (Signal->ControlMode == EJunctionControlMode::Signal && Signal->PhaseGroups.Num() > 1)
		{
			TArray<FSignalPhaseGroup> ExpandedGroups;
			for (const FSignalPhaseGroup& ThroughGroup : Signal->PhaseGroups)
			{
				// Find left-turn lanes in this group.
				TArray<FTrafficLaneHandle> LeftTurnLanes;
				for (const FTrafficLaneHandle& GLane : ThroughGroup.GreenLanes)
				{
					TArray<FTrafficLaneHandle> Exits = Provider->GetConnectedLanes(GLane);
					const float GLaneLen = Provider->GetLaneLength(GLane);
					const FVector ApproachDir = Provider->GetLaneDirectionAtDistance(GLane, GLaneLen);
					for (const FTrafficLaneHandle& Exit : Exits)
					{
						if (Provider->GetJunctionForLane(Exit) != 0) { continue; }
						const FVector ExitDir = Provider->GetLaneDirection(Exit);
						const float CrossZ = FVector::CrossProduct(ApproachDir, ExitDir).Z;
						if (CrossZ > 0.26f) // Left turn (sin(15°) threshold)
						{
							LeftTurnLanes.AddUnique(GLane);
							break;
						}
					}
				}

				// If left-turn lanes found, insert an arrow phase before through.
				if (LeftTurnLanes.Num() > 0)
				{
					FSignalPhaseGroup ArrowGroup;
					ArrowGroup.GroupName = ThroughGroup.GroupName + TEXT("_LeftArrow");
					ArrowGroup.GreenLanes = MoveTemp(LeftTurnLanes);
					ArrowGroup.GroupGreenDuration = 10.0f; // Shorter left-turn phase.
					ArrowGroup.bIsProtectedArrow = true;
					ExpandedGroups.Add(MoveTemp(ArrowGroup));
				}

				// Through-phase follows.
				ExpandedGroups.Add(ThroughGroup);
			}
			Signal->PhaseGroups = MoveTemp(ExpandedGroups);
		}

		// Derive all-red clearance from the longest junction lane — the
		// time a vehicle needs to cross the intersection at the intersection
		// speed limit.  Clamp to [2, 8] seconds for sanity.
		{
			float MaxJunctionLaneLen = 0.0f;
			for (const FTrafficLaneHandle& JLane : Lanes)
			{
				MaxJunctionLaneLen = FMath::Max(MaxJunctionLaneLen, Provider->GetLaneLength(JLane));
			}
			// crossing time = distance / speed. IntersectionSpeedLimitCmPerSec default = 2000 (20 m/s ≈ 45 mph).
			constexpr float TypicalIntersectionSpeedCmPerSec = 1500.0f; // ~15 m/s conservative turning speed
			const float CrossingTimeSec = (MaxJunctionLaneLen > 0.0f)
				? MaxJunctionLaneLen / TypicalIntersectionSpeedCmPerSec
				: 2.0f;
			Signal->AllRedClearanceSec = FMath::Clamp(CrossingTimeSec, 2.0f, 8.0f);
		}

		// Stagger signal phases so adjacent junctions don't all start green
		// simultaneously. Each junction offsets by one full phase period.
		Signal->PhaseOffset = static_cast<float>(SignalsPlaced)
			* (Signal->GreenDuration + Signal->YellowDuration);

		// Dump final phase group structure.
		if (bSignalDiag)
		{
			UE_LOG(LogAAATraffic, Warning,
				TEXT("SIGNAL-DIAG: JunctionId=%d FINAL — ControlMode=%d PhaseGroupCount=%d AllRedClearance=%.1fs Offset=%.1fs"),
				JId, static_cast<int32>(Signal->ControlMode), Signal->PhaseGroups.Num(),
				Signal->AllRedClearanceSec, Signal->PhaseOffset);
		}
		for (int32 gi = 0; gi < Signal->PhaseGroups.Num(); ++gi)
		{
			const FSignalPhaseGroup& G = Signal->PhaseGroups[gi];
			FString LaneList;
			for (const FTrafficLaneHandle& L : G.GreenLanes)
			{
				LaneList += FString::Printf(TEXT("%d "), L.HandleId);
			}
			if (bSignalDiag)
			{
				UE_LOG(LogAAATraffic, Warning,
					TEXT("SIGNAL-DIAG: JunctionId=%d Group[%d] '%s' GreenDur=%.1f Lanes=[%s]"),
					JId, gi, *G.GroupName, G.GroupGreenDuration, *LaneList);
			}
		}

		// Finish spawning — now BeginPlay sees the correct JunctionId and registers.
		Signal->FinishSpawning(SignalTransform);

		++SignalsPlaced;
	}

	if (SignalsPlaced > 0)
	{
		UE_LOG(LogAAATraffic, Log,
			TEXT("TrafficSpawner: Auto-placed %d junction controllers across %d junctions "
				 "(mode auto-classified by approach arm count: 4+=Signal, 3=StopSign, 2=angle-check)."),
			SignalsPlaced, JunctionIds.Num());
	}
}
