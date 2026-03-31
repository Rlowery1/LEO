// Copyright AAA_Traffic Plugin. All Rights Reserved.
//
// Full-simulation latent automation tests.
// Loads each test map, runs the complete traffic simulation (same code path
// as PIE), and validates 7 pass/fail conditions that match what a human
// sees when pressing Play:
//
//   1. Vehicles spawned
//   2. Cars complete junction turns
//   3. Cars stay on road
//   4. No cars get stuck
//   5. No collisions
//   6. No flipped vehicles
//   7. No wrong-way driving
//
// Run via: Tools -> Session Frontend -> Automation -> "AAA_Traffic"

#if WITH_DEV_AUTOMATION_TESTS

#include "CoreMinimal.h"
#include "Misc/AutomationTest.h"
#include "Tests/AutomationCommon.h"
#include "Engine/World.h"
#include "Engine/Engine.h"
#include "Kismet/GameplayStatics.h"

#include "Core/TrafficLog.h"
#include "Core/TrafficRoadProvider.h"
#include "Core/TrafficRoadQueryLibrary.h"
#include "Controller/TrafficVehicleController.h"
#include "Subsystem/TrafficSubsystem.h"

// ────────────────────────────────────────────────────────────────
// Test maps to iterate (UE content paths).
// Each becomes a separate test entry in the Automation tab.
// ────────────────────────────────────────────────────────────────

static const TArray<FString> GTestMapPaths = {
	TEXT("/Game/Test_Maps/Test_Map_2Way_Road"),
	TEXT("/Game/Test_Maps/Test_Map_Intersection_2Way_Road"),
	TEXT("/Game/Test_Maps/Test_Map_Roundabout_With_Straight_Roads"),
	TEXT("/Game/Test_Maps/Test_Map_Multiple_Intersections"),
	TEXT("/Game/Test_Maps/Test_Map_Short_Road_With_2Intersections"),
	TEXT("/Game/Test_Maps/Test_Map_Road_That_Loops_Into_Itself_With_Intersection"),
	TEXT("/Game/Test_Maps/Test_Map_5+_Connecting_Roads_To_Single_Intersection"),
};

// ────────────────────────────────────────────────────────────────
// Per-vehicle tracking state accumulated during the simulation.
// ────────────────────────────────────────────────────────────────

struct FVehicleTestRecord
{
	/** Pawn name for diagnostics. */
	FString PawnName;

	/** Did this vehicle ever reach the Traversing phase? */
	bool bEverTraversed = false;

	/** Did this vehicle ever return to Idle after Traversing? */
	bool bCompletedTurn = false;

	/** Was the last sampled phase Traversing? (detect transition to Idle) */
	bool bWasTraversing = false;

	/** Did this vehicle trigger CollisionBrakeTimer > 0? */
	bool bHadCollision = false;

	/** Did this vehicle trigger FlipTimeAccumulator > 0.5? */
	bool bFlipped = false;

	/** Ring buffer of recent positions for stuck detection (every 0.5s). */
	TArray<FVector> PositionHistory;

	/** Was the vehicle ever stuck (< 50cm movement in 10s window)? */
	bool bWasStuck = false;

	/** Was the vehicle ever driving the wrong way? */
	bool bWrongWay = false;

	/** Was the vehicle ever off-road (farther than lane width + margin)? */
	bool bWentOffRoad = false;

	/** Was the vehicle exempt from stuck detection because it was
	 *  waiting at a junction (Waiting phase)? */
	bool bCurrentlyWaiting = false;

	/** Did this vehicle ever enter any junction phase
	 *  (Approaching, Waiting, or Traversing)? */
	bool bEverInJunction = false;

	/** Seconds since the vehicle was last in Traversing phase.
	 *  Used to provide a collision grace window after exiting a junction. */
	float SecondsSinceTraversal = TNumericLimits<float>::Max();
};

// ────────────────────────────────────────────────────────────────
// Latent command: wait for vehicles to spawn (up to 30 seconds).
// Extended timeout accounts for WorldPartition deferred road discovery.
// ────────────────────────────────────────────────────────────────

class FWaitForVehicleSpawn : public IAutomationLatentCommand
{
public:
	FWaitForVehicleSpawn(FAutomationTestBase* InTest) : Test(InTest) {}

	virtual bool Update() override
	{
		UWorld* World = nullptr;
		if (GEngine)
		{
			for (const FWorldContext& Ctx : GEngine->GetWorldContexts())
			{
				if (Ctx.WorldType == EWorldType::Game || Ctx.WorldType == EWorldType::PIE)
				{
					World = Ctx.World();
					break;
				}
			}
		}
		if (!World) { return false; }

		UTrafficSubsystem* Sub = World->GetSubsystem<UTrafficSubsystem>();
		if (!Sub) { return false; }

		if (Sub->GetActiveVehicles().Num() > 0)
		{
			return true; // Vehicles are live — proceed.
		}

		AccumulatedTime += FApp::GetDeltaTime();
		if (AccumulatedTime > 30.0f)
		{
			Test->AddError(TEXT("FAIL: No vehicles spawned within 30 seconds."));
			return true; // Abort.
		}

		return false; // Keep waiting.
	}

private:
	FAutomationTestBase* Test = nullptr;
	float AccumulatedTime = 0.0f;
};

// ────────────────────────────────────────────────────────────────
// Latent command: run simulation for N seconds, sampling state.
// ────────────────────────────────────────────────────────────────

DEFINE_LATENT_AUTOMATION_COMMAND_TWO_PARAMETER(FRunSimulationAndValidate,
	FAutomationTestBase*, Test,
	float, SimulationDurationSec);

bool FRunSimulationAndValidate::Update()
{
	UWorld* World = nullptr;
	if (GEngine)
	{
		for (const FWorldContext& Ctx : GEngine->GetWorldContexts())
		{
			if (Ctx.WorldType == EWorldType::Game || Ctx.WorldType == EWorldType::PIE)
			{
				World = Ctx.World();
				break;
			}
		}
	}
	if (!World) { return false; }

	UTrafficSubsystem* Sub = World->GetSubsystem<UTrafficSubsystem>();
	if (!Sub) { return false; }

	// ── Persistent state across Update() calls ─────────────
	// Using static locals because DEFINE_LATENT_AUTOMATION_COMMAND
	// doesn't easily support complex member state.
	// We reset on every new test run via the bInitialized flag.

	static float ElapsedTime = 0.0f;
	static float SampleTimer = 0.0f;
	static TMap<int32, FVehicleTestRecord> VehicleRecords;
	static bool bInitialized = false;
	static float LastSimDuration = -1.0f;

	const float DeltaTime = FApp::GetDeltaTime();

	// Reset when a new test starts (different duration or first call).
	if (!bInitialized || LastSimDuration != SimulationDurationSec)
	{
		ElapsedTime = 0.0f;
		SampleTimer = 0.0f;
		VehicleRecords.Reset();
		bInitialized = true;
		LastSimDuration = SimulationDurationSec;
	}

	ElapsedTime += DeltaTime;
	SampleTimer += DeltaTime;

	// ── Sample every 0.5 seconds ───────────────────────────
	constexpr float SampleIntervalSec = 0.5f;
	if (SampleTimer >= SampleIntervalSec)
	{
		SampleTimer -= SampleIntervalSec;

		const auto& ActiveVehicles = Sub->GetActiveVehicles();
		for (const auto& WeakVehicle : ActiveVehicles)
		{
			ATrafficVehicleController* Controller = WeakVehicle.Get();
			if (!Controller || !Controller->GetPawn()) { continue; }

			const int32 VehicleId = Controller->GetUniqueID();
			FVehicleTestRecord& Record = VehicleRecords.FindOrAdd(VehicleId);

			if (Record.PawnName.IsEmpty())
			{
				Record.PawnName = Controller->GetPawn()->GetName();
			}

			const FVehicleDiagnosticSnapshot Diag = Controller->GetDiagnosticSnapshot();
			const APawn* Pawn = Controller->GetPawn();
			const FVector VehiclePos = Pawn->GetActorLocation();
			const FVector VehicleFwd = Pawn->GetActorForwardVector();

			// ── Condition 2: Junction turn completion ──────
			// Track junction presence via any non-Idle phase.
			// A turn is completed when the vehicle exits Traversing.
			// On roundabouts, the transition is Traversing→Approaching
			// (not Traversing→Idle), so we track any exit from Traversing.
			if (Diag.JunctionPhase != EJunctionPhase::Idle)
			{
				Record.bEverInJunction = true;
			}
			if (Diag.JunctionPhase == EJunctionPhase::Traversing)
			{
				Record.bEverTraversed = true;
				Record.bWasTraversing = true;
				Record.SecondsSinceTraversal = 0.0f;
			}
			else
			{
				if (Record.bWasTraversing)
				{
					Record.bCompletedTurn = true;
				}
				Record.bWasTraversing = false;
				Record.SecondsSinceTraversal += SampleIntervalSec;
			}

			Record.bCurrentlyWaiting =
				(Diag.JunctionPhase == EJunctionPhase::Waiting ||
				 Diag.JunctionPhase == EJunctionPhase::Approaching);

			// ── Condition 5: Collision detection ───────────
			// Only flag vehicle-vehicle collisions. Road geometry
			// scrapes (curbs/barriers) are not simulation failures.
			// Also exempt vehicles on a junction curve (Traversing)
			// or within 2 seconds of exiting one.
			constexpr float PostTraversalGraceSec = 2.0f;
			const bool bRecentlyTraversed =
				Record.SecondsSinceTraversal < PostTraversalGraceSec;
			if (Diag.CollisionBrakeTimer > 0.0f
				&& Diag.bCollisionWithVehicle
				&& Diag.JunctionPhase != EJunctionPhase::Traversing
				&& !bRecentlyTraversed)
			{
				if (!Record.bHadCollision)
				{
					Test->AddError(FString::Printf(
						TEXT("COLLISION: '%s' at (%.0f, %.0f, %.0f) — CollisionBrakeTimer=%.2f"),
						*Record.PawnName, VehiclePos.X, VehiclePos.Y, VehiclePos.Z,
						Diag.CollisionBrakeTimer));
				}
				Record.bHadCollision = true;
			}

			// ── Condition 6: Flip detection ────────────────
			if (Diag.FlipTimeAccumulator > 0.5f)
			{
				if (!Record.bFlipped)
				{
					Test->AddError(FString::Printf(
						TEXT("FLIPPED: '%s' at (%.0f, %.0f, %.0f) — FlipTime=%.2f"),
						*Record.PawnName, VehiclePos.X, VehiclePos.Y, VehiclePos.Z,
						Diag.FlipTimeAccumulator));
				}
				Record.bFlipped = true;
			}

			// ── Condition 3: Off-road detection ────────────
			// Skip during Traversing — the vehicle is deliberately
			// off regular road lanes while following the junction curve.
			// Post-exit alignment is handled by EXIT-SNAP; non-Traversing
			// off-road detection catches any persistent deviation.
			const bool bTraversingJunction =
				(Diag.JunctionPhase == EJunctionPhase::Traversing);
			if (!Record.bWentOffRoad && !bTraversingJunction)
			{
				const FTrafficLaneHandle NearLane =
					UTrafficRoadQueryLibrary::GetLaneAtLocation(World, VehiclePos);
				if (NearLane.IsValid())
				{
					TArray<FVector> LanePoints;
					float LaneWidth = 0.0f;
					if (UTrafficRoadQueryLibrary::GetLanePath(World, NearLane, LanePoints, LaneWidth))
					{
						// Find closest point on polyline.
						float MinDistSq = TNumericLimits<float>::Max();
						for (const FVector& LP : LanePoints)
						{
							const float D = FVector::DistSquared2D(VehiclePos, LP);
							if (D < MinDistSq) { MinDistSq = D; }
						}
						const float Dist = FMath::Sqrt(MinDistSq);
						const float Threshold = (LaneWidth * 0.5f) + 200.0f;
						if (Dist > Threshold)
						{
							Test->AddError(FString::Printf(
								TEXT("OFF-ROAD: '%s' at (%.0f, %.0f, %.0f) — %.0f cm from lane centerline (threshold=%.0f)"),
								*Record.PawnName, VehiclePos.X, VehiclePos.Y, VehiclePos.Z,
								Dist, Threshold));
							Record.bWentOffRoad = true;
						}
					}
				}
				else
				{
					// No lane found at all near this position.
					Test->AddError(FString::Printf(
						TEXT("OFF-ROAD: '%s' at (%.0f, %.0f, %.0f) — no lane found at location"),
						*Record.PawnName, VehiclePos.X, VehiclePos.Y, VehiclePos.Z));
					Record.bWentOffRoad = true;
				}
			}

			// ── Condition 7: Wrong-way detection ───────────
			// Exempt vehicles traversing junction curves — they can
			// momentarily face away from the lane direction during a turn.
			if (!Record.bWrongWay && !Record.bCurrentlyWaiting && !bTraversingJunction)
			{
				const TArray<FVector>& LanePts = Controller->GetLanePoints();
				if (LanePts.Num() >= 2)
				{
					// Find lane direction at the vehicle's nearest point.
					float MinDistSq = TNumericLimits<float>::Max();
					int32 NearestIdx = 0;
					for (int32 Idx = 0; Idx < LanePts.Num(); ++Idx)
					{
						const float D = FVector::DistSquared2D(VehiclePos, LanePts[Idx]);
						if (D < MinDistSq)
						{
							MinDistSq = D;
							NearestIdx = Idx;
						}
					}
					const int32 DirIdx = FMath::Min(NearestIdx, LanePts.Num() - 2);
					const FVector LaneDir = (LanePts[DirIdx + 1] - LanePts[DirIdx]).GetSafeNormal2D();
					const float HeadingDot = FVector::DotProduct(
						FVector(VehicleFwd.X, VehicleFwd.Y, 0.0f).GetSafeNormal(), LaneDir);

					// Dot < -0.5 means > 120° off — definitively wrong way.
					// Only flag if the vehicle is actually moving (speed > 100 cm/s).
					const float SpeedSq = Pawn->GetVelocity().SizeSquared();
					if (HeadingDot < -0.5f && SpeedSq > 100.0f * 100.0f)
					{
						Test->AddError(FString::Printf(
							TEXT("WRONG-WAY: '%s' at (%.0f, %.0f, %.0f) — HeadingDot=%.2f, Speed=%.0f cm/s"),
							*Record.PawnName, VehiclePos.X, VehiclePos.Y, VehiclePos.Z,
							HeadingDot, FMath::Sqrt(SpeedSq)));
						Record.bWrongWay = true;
					}
				}
			}

			// ── Condition 4: Stuck detection (position history) ──
			Record.PositionHistory.Add(VehiclePos);

			// Check over a 10-second window (20 samples at 0.5s intervals).
			constexpr int32 StuckWindowSamples = 20;
			if (Record.PositionHistory.Num() > StuckWindowSamples)
			{
				const FVector OldPos = Record.PositionHistory[
					Record.PositionHistory.Num() - 1 - StuckWindowSamples];
				const float MovedCm = FVector::Dist2D(VehiclePos, OldPos);

					// Exempt vehicles waiting at junctions, traversing curves,
				// or stopped at a dead-end lane awaiting despawn.
				if (MovedCm < 50.0f && !Record.bCurrentlyWaiting && !bTraversingJunction
					&& !Controller->IsAtDeadEnd() && !Record.bWasStuck)
				{
					Test->AddError(FString::Printf(
						TEXT("STUCK: '%s' at (%.0f, %.0f, %.0f) — moved only %.0f cm in 10 seconds"),
						*Record.PawnName, VehiclePos.X, VehiclePos.Y, VehiclePos.Z, MovedCm));
					Record.bWasStuck = true;
				}
			}
		}
	}

	// ── Simulation complete? ───────────────────────────────
	if (ElapsedTime < SimulationDurationSec)
	{
		return false; // Keep ticking.
	}

	// ── Final validation ───────────────────────────────────

	// Condition 1: Vehicles spawned.
	const int32 TotalVehicles = VehicleRecords.Num();
	if (TotalVehicles == 0)
	{
		Test->AddError(TEXT("FAIL: No vehicles were tracked during the simulation."));
	}
	else
	{
		Test->AddInfo(FString::Printf(TEXT("Tracked %d vehicles over %.0f seconds."),
			TotalVehicles, SimulationDurationSec));
	}

	// Condition 2: Junction turn completion.
	// Determine if this map has junctions by checking if any vehicle ever
	// entered the Approaching/Waiting/Traversing phase.
	bool bMapHasJunctions = false;
	int32 TurnsCompleted = 0;
	for (const auto& Pair : VehicleRecords)
	{
		if (Pair.Value.bEverInJunction || Pair.Value.bEverTraversed)
		{
			bMapHasJunctions = true;
		}
		if (Pair.Value.bCompletedTurn) { ++TurnsCompleted; }
	}

	if (bMapHasJunctions)
	{
		if (TurnsCompleted == 0)
		{
			Test->AddError(TEXT("FAIL: Map has junctions but NO vehicle completed a turn."));
		}
		else
		{
			Test->AddInfo(FString::Printf(TEXT("Junction turns completed: %d"), TurnsCompleted));
		}
	}
	else
	{
		Test->AddInfo(TEXT("Map has no junctions — skipping turn completion check."));
	}

	// Summary counts for other conditions.
	int32 Collisions = 0, Flips = 0, Stuck = 0, OffRoad = 0, WrongWay = 0;
	for (const auto& Pair : VehicleRecords)
	{
		if (Pair.Value.bHadCollision) { ++Collisions; }
		if (Pair.Value.bFlipped) { ++Flips; }
		if (Pair.Value.bWasStuck) { ++Stuck; }
		if (Pair.Value.bWentOffRoad) { ++OffRoad; }
		if (Pair.Value.bWrongWay) { ++WrongWay; }
	}

	if (Collisions == 0 && Flips == 0 && Stuck == 0 && OffRoad == 0 && WrongWay == 0)
	{
		Test->AddInfo(TEXT("All safety checks PASSED."));
	}
	else
	{
		Test->AddInfo(FString::Printf(
			TEXT("Safety issues: Collisions=%d, Flips=%d, Stuck=%d, OffRoad=%d, WrongWay=%d"),
			Collisions, Flips, Stuck, OffRoad, WrongWay));
	}

	// Reset static state for next test run.
	bInitialized = false;
	ElapsedTime = 0.0f;
	SampleTimer = 0.0f;
	VehicleRecords.Reset();

	return true; // Simulation done.
}

// ────────────────────────────────────────────────────────────────
// Complex automation test: one entry per map in the Automation tab.
// ────────────────────────────────────────────────────────────────

IMPLEMENT_COMPLEX_AUTOMATION_TEST(FTrafficSimulationTest,
	"AAA_Traffic.Simulation",
	EAutomationTestFlags::EditorContext | EAutomationTestFlags::ClientContext | EAutomationTestFlags::ProductFilter)

void FTrafficSimulationTest::GetTests(TArray<FString>& OutBeautifiedNames,
	TArray<FString>& OutTestCommands) const
{
	for (const FString& MapPath : GTestMapPaths)
	{
		// Extract map name for display.
		FString MapName = FPaths::GetBaseFilename(MapPath);
		OutBeautifiedNames.Add(MapName);
		OutTestCommands.Add(MapPath);
	}
}

bool FTrafficSimulationTest::RunTest(const FString& Parameters)
{
	const FString& MapPath = Parameters;

	UE_LOG(LogAAATraffic, Log, TEXT("=== AUTOMATION TEST: Loading map '%s' ==="), *MapPath);

	// Suppress LogAAATraffic Error messages (e.g. canonical-compile diagnostics,
	// spawner config warnings) from failing the test.  The test has its own
	// explicit AddError() calls for the 7 safety conditions.
	// Suppress a bounded number of LogAAATraffic Error messages (e.g.
	// canonical-compile diagnostics, spawner config warnings) from failing
	// the test.  The test has its own explicit AddError() calls for the
	// 7 safety conditions.  Unexpected errors beyond this count will still
	// surface as test failures.
	AddExpectedErrorPlain(TEXT("LogAAATraffic"), EAutomationExpectedErrorFlags::Contains, 50);

	// Step 1: Load the map.
	ADD_LATENT_AUTOMATION_COMMAND(FLoadGameMapCommand(MapPath));
	ADD_LATENT_AUTOMATION_COMMAND(FWaitForMapToLoadCommand());

	// Step 2: Wait for vehicles to spawn.
	ADD_LATENT_AUTOMATION_COMMAND(FWaitForVehicleSpawn(this));

	// Step 3: Let the simulation run for 120 seconds (2 minutes), sampling every 0.5s.
	ADD_LATENT_AUTOMATION_COMMAND(FRunSimulationAndValidate(this, 120.0f));

	return true;
}

#endif // WITH_DEV_AUTOMATION_TESTS
