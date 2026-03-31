// TrafficVehicleController_DebugDraw.cpp -- In-world debug visualization.
// Split from TrafficVehicleController.cpp for maintainability.

#include "TrafficVehicleController.h"
#include "TrafficLog.h"
#include "DrawDebugHelpers.h"
#include "GameFramework/Pawn.h"
#include "Engine/World.h"

extern int32 GTrafficDebugDraw;
extern int32 GTrafficDebugDrawPaths;

void ATrafficVehicleController::DrawVehicleDebug()
{
#if defined(ENABLE_DRAW_DEBUG) && ENABLE_DRAW_DEBUG
	if (GTrafficDebugDrawPaths != 0 && GetPawn())
	{
		const UWorld* DbgWorld = GetWorld();
		const FVector VehicleLoc = GetPawn()->GetActorLocation();

		// --- PATH ONLY: Lane polyline (bright cyan, thick) ---
		for (int32 i = 0; i < LanePoints.Num() - 1; ++i)
		{
			DrawDebugLine(DbgWorld, LanePoints[i], LanePoints[i + 1],
				FColor::Cyan, false, -1.0f, SDPG_Foreground, 4.0f);
		}

		// --- PATH ONLY: Junction transition curve (yellow, thick) ---
		if (JnctState.TransitionPoints.Num() >= 2)
		{
			for (int32 i = 0; i < JnctState.TransitionPoints.Num() - 1; ++i)
			{
				DrawDebugLine(DbgWorld, JnctState.TransitionPoints[i],
					JnctState.TransitionPoints[i + 1],
					FColor::Yellow, false, -1.0f, SDPG_Foreground, 5.0f);
			}
			DrawDebugSphere(DbgWorld, JnctState.TransitionPoints[0],
				30.0f, 6, FColor::Green, false, -1.0f, SDPG_Foreground, 2.0f);
			DrawDebugSphere(DbgWorld, JnctState.TransitionPoints.Last(),
				30.0f, 6, FColor::Red, false, -1.0f, SDPG_Foreground, 2.0f);
		}

		// --- PATH ONLY: Vehicle position marker ---
		DrawDebugSphere(DbgWorld, VehicleLoc, 40.0f, 6, FColor::White,
			false, -1.0f, SDPG_Foreground, 2.0f);

		return; // Skip the full debug draw.
	}

	if ((bDebugDraw || GTrafficDebugDraw != 0) && GetPawn())
	{
		const UWorld* DbgWorld = GetWorld();
		const FVector VehicleLoc = GetPawn()->GetActorLocation();
		const FVector VehicleFwd = GetPawn()->GetActorForwardVector();
		const FVector TextBase = VehicleLoc + FVector(0, 0, 180.0f); // Above the car roof

		// --- Color based on state ---
		FColor StateColor = FColor::Green; // CRUISING
		if (DbgStateName == TEXT("WAITING"))          { StateColor = FColor::Red; }
		else if (DbgStateName == TEXT("BRAKING"))     { StateColor = FColor(255, 165, 0); } // Orange
		else if (DbgStateName == TEXT("FOLLOWING"))   { StateColor = FColor::Yellow; }
		else if (DbgStateName == TEXT("IN-JNCT"))     { StateColor = FColor(0, 200, 200); } // Teal
		else if (DbgStateName == TEXT("APPROACH"))    { StateColor = FColor(255, 200, 0); } // Amber

		// --- LINE 1: Vehicle name + state ---
		const FString VehicleName = GetPawn()->GetName();
		// Strip BP_Sedan_child_base_ prefix for readability.
		FString ShortName = VehicleName;
		ShortName.ReplaceInline(TEXT("BP_Sedan_child_base_"), TEXT(""));
		const FString Line1 = FString::Printf(TEXT("%s [%s]"), *ShortName, *DbgStateName);
		DrawDebugString(DbgWorld, TextBase, Line1, nullptr, StateColor, 0.0f, true, 1.2f);

		// --- LINE 2: Speed (mph) + Brake% ---
		const float SpeedMph = FMath::Abs(DbgCurrentSpeed) * 0.0223694f; // cm/s -> mph
		const FString Line2 = FString::Printf(TEXT("%.0f mph  Brake:%.0f%%  Throttle:%.0f%%"),
			SpeedMph, DbgBrake * 100.0f, DbgThrottle * 100.0f);
		DrawDebugString(DbgWorld, TextBase + FVector(0, 0, -20), Line2, nullptr, FColor::White, 0.0f, true, 1.0f);

		// --- LINE 3: Lane + RemainingDist + TransitionThreshold ---
		const FString Line3 = FString::Printf(TEXT("Lane:%d  Rem:%.0f  Thresh:%.0f  StopDist:%.0f"),
			CurrentLane.HandleId, DbgRemainingDist, DbgTransitionThreshold, DbgStoppingDist);
		DrawDebugString(DbgWorld, TextBase + FVector(0, 0, -40), Line3, nullptr, FColor::White, 0.0f, true, 0.9f);

		// --- LINE 4: Intersection-specific info (only when approaching/waiting) ---
		if (DbgDistToEntry >= 0.0f || JnctState.bWaiting)
		{
			const FString Line4 = FString::Printf(TEXT("DistToEntry:%.0f  DesiredStop:%.0f  Jnct:%d"),
				DbgDistToEntry, DbgDesiredStopSpeed, JnctState.JunctionId);
			DrawDebugString(DbgWorld, TextBase + FVector(0, 0, -58), Line4, nullptr,
				FColor::Red, 0.0f, true, 0.9f);
		}

		// --- LINE 4b: Junction Approach Scan info (when scan detected a junction ahead) ---
		if (DbgApproachJunctionDist >= 0.0f)
		{
			const bool bDbgApproaching = (JnctState.Phase == EJunctionPhase::Approaching);
			const float ApproachMph = DbgApproachSpeedLimit * 0.0223694f;
			const FString Line4b = FString::Printf(TEXT("SCAN: Jnct:%d  Dist:%.0f  VLimit:%.0fmph  %s"),
				DbgApproachJunctionId,
				DbgApproachJunctionDist,
				ApproachMph,
				bDbgApproaching ? TEXT("BRAKING") : TEXT("OK"));
			const FColor ScanColor = bDbgApproaching ? FColor(255, 165, 0) : FColor(0, 200, 100);
			DrawDebugString(DbgWorld, TextBase + FVector(0, 0, -94), Line4b, nullptr,
				ScanColor, 0.0f, true, 0.9f);
		}

		// --- LINE 5: Leader info (only when following) ---
		if (DbgLeaderDist >= 0.0f)
		{
			const float LeaderMph = FMath::Abs(DbgLeaderSpeed) * 0.0223694f;
			const FString Line5 = FString::Printf(TEXT("Leader:%.0fcm  LeaderSpeed:%.0fmph  FollowDist:%.0f"),
				DbgLeaderDist, LeaderMph, FollowingDistance);
			DrawDebugString(DbgWorld, TextBase + FVector(0, 0, -76), Line5, nullptr,
				FColor::Yellow, 0.0f, true, 0.9f);

			// Orange line from car to detected leader position.
			const FVector LeaderPos = VehicleLoc + VehicleFwd * DbgLeaderDist;
			DrawDebugLine(DbgWorld, VehicleLoc, LeaderPos, FColor::Orange, false, -1.0f, 0, 2.5f);
			DrawDebugSphere(DbgWorld, LeaderPos, 40.0f, 6, FColor::Orange, false, -1.0f, 0, 2.0f);
		}

		// --- INTERSECTION ENTRY POINT: Red sphere where the car thinks it must stop ---
		if (JnctState.bHasEntryPos)
		{
			DrawDebugSphere(DbgWorld, JnctState.EntryWorldPos, 60.0f, 8, FColor::Red, false, -1.0f, 0, 3.0f);
			// Yellow line from car to entry point -- shows actual stopping gap.
			DrawDebugLine(DbgWorld, VehicleLoc, JnctState.EntryWorldPos, FColor::Yellow, false, -1.0f, 0, 2.5f);

			// Blue sphere: where the decel curve predicts the car WILL stop
			// (v^2 / (2*300) cm from current position along forward vector).
			if (DbgStoppingDist > 10.0f)
			{
				const FVector PredictedStop = VehicleLoc + VehicleFwd * DbgStoppingDist;
				DrawDebugSphere(DbgWorld, PredictedStop, 40.0f, 6, FColor::Blue, false, -1.0f, 0, 2.5f);
				DrawDebugLine(DbgWorld, VehicleLoc, PredictedStop, FColor::Blue, false, -1.0f, 0, 1.5f);
			}
		}

		// --- STOPPING DISTANCE vs REMAINING DISTANCE comparison bar ---
		// Visualize whether the car CAN physically stop before lane end.
		// Green bar = can stop. Red bar = cannot stop (will overshoot).
		if (DbgRemainingDist > 0.0f && DbgStoppingDist > 0.0f)
		{
			const bool bCanStop = (DbgStoppingDist <= DbgRemainingDist);
			const FColor BarColor = bCanStop ? FColor::Green : FColor::Red;
			// Draw a line along forward vector showing stopping distance,
			// with the length clamped to remaining distance for comparison.
			const float BarLen = FMath::Min(DbgStoppingDist, DbgRemainingDist + 500.0f);
			const FVector BarEnd = VehicleLoc + VehicleFwd * BarLen;
			DrawDebugLine(DbgWorld, VehicleLoc + FVector(0,0,50), BarEnd + FVector(0,0,50),
				BarColor, false, -1.0f, 0, 4.0f);
		}

		// --- LANE POLYLINE (cyan) ---
		for (int32 i = 0; i < LanePoints.Num() - 1; ++i)
		{
			DrawDebugLine(DbgWorld, LanePoints[i], LanePoints[i + 1], FColor::Cyan, false, -1.0f, 0, 2.0f);
		}

		// --- LOOK-AHEAD TARGET (green sphere + line) ---
		if (!DbgTargetPoint.IsZero())
		{
			DrawDebugSphere(DbgWorld, DbgTargetPoint, 30.0f, 6, FColor::Green, false, -1.0f, 0, 2.0f);
			DrawDebugLine(DbgWorld, VehicleLoc, DbgTargetPoint, FColor::Green, false, -1.0f, 0, 1.5f);
		}

		// --- TRANSITION THRESHOLD RING (white circle at lane end minus threshold) ---
		if (DbgTransitionThreshold > 0.0f && LanePoints.Num() >= 2)
		{
			// Walk backward from lane end to find the threshold position.
			float DistFromEnd = 0.0f;
			FVector ThreshPt = LanePoints.Last();
			for (int32 i = LanePoints.Num() - 1; i > 0; --i)
			{
				const float Seg = FVector::Dist(LanePoints[i], LanePoints[i - 1]);
				if (DistFromEnd + Seg >= DbgTransitionThreshold)
				{
					const float Alpha = (DbgTransitionThreshold - DistFromEnd) / FMath::Max(Seg, 1.0f);
					ThreshPt = FMath::Lerp(LanePoints[i], LanePoints[i - 1], Alpha);
					break;
				}
				DistFromEnd += Seg;
			}
			DrawDebugSphere(DbgWorld, ThreshPt, 50.0f, 8, FColor::White, false, -1.0f, 0, 2.0f);
		}

		// --- APPROACH SCAN: Junction marker (orange diamond at scan distance ahead) ---
		if (DbgApproachJunctionDist > 0.0f)
		{
			const FVector ApproachPt = VehicleLoc + VehicleFwd * FMath::Min(DbgApproachJunctionDist, 10000.0f);
			const FColor ApproachColor = (JnctState.Phase == EJunctionPhase::Approaching) ? FColor(255, 100, 0) : FColor(100, 255, 100);
			DrawDebugSphere(DbgWorld, ApproachPt, 70.0f, 4, ApproachColor, false, -1.0f, 0, 3.0f);
			DrawDebugLine(DbgWorld, VehicleLoc + FVector(0, 0, 70), ApproachPt + FVector(0, 0, 70),
				ApproachColor, false, -1.0f, 0, 2.5f);
		}

		// --- LANE-CHANGE TARGET LANE (magenta) ---
		if (LaneChangeCoord_.State != ELaneChangeState::None)
		{
			for (int32 i = 0; i < LaneChangeCoord_.TargetLanePoints.Num() - 1; ++i)
			{
				DrawDebugLine(DbgWorld, LaneChangeCoord_.TargetLanePoints[i], LaneChangeCoord_.TargetLanePoints[i + 1], FColor::Magenta, false, -1.0f, 0, 2.0f);
			}
		}

		// --- JUNCTION TRANSITION CURVE (yellow) ---
		if (JnctState.TransitionPoints.Num() >= 2)
		{
			for (int32 i = 0; i < JnctState.TransitionPoints.Num() - 1; ++i)
			{
				DrawDebugLine(DbgWorld, JnctState.TransitionPoints[i],
					JnctState.TransitionPoints[i + 1],
					FColor::Yellow, false, -1.0f, 0, 3.0f);
			}
			DrawDebugSphere(DbgWorld, JnctState.TransitionPoints[0],
				25.0f, 4, FColor::Green, false, -1.0f, 0, 2.0f);
			DrawDebugSphere(DbgWorld, JnctState.TransitionPoints.Last(),
				25.0f, 4, FColor::Red, false, -1.0f, 0, 2.0f);
		}
	}
#endif // ENABLE_DRAW_DEBUG
}

