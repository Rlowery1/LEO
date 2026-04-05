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
#include "Controller/LaneChangeCoordinator.h"
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
	// Phase 3 stress-test maps
	TEXT("/Game/Test_Maps/Test_Map_Overtaking_2Way_Straight"),
	TEXT("/Game/Test_Maps/Test_Maps_Overtaking_2Way_Curves"),
	TEXT("/Game/Test_Maps/Test_Map_Mixed_Lane_Types"),
	TEXT("/Game/Test_Maps/Test_Map_Road_With_ActiveSegments"),
	TEXT("/Game/Test_Maps/Test_Map_Personality_Mix_MultiLane"),
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

	// ── Phase 3 feature-exercise tracking ──────────────────
	/** Did this vehicle ever enter an overtake phase (PullingOut/Passing/PullingIn)? */
	bool bEverOvertook = false;

	/** Target speed assigned to this vehicle (cm/s), for variation detection. */
	float AssignedTargetSpeed = 0.0f;

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
				if (Ctx.World() && (Ctx.WorldType == EWorldType::Game || Ctx.WorldType == EWorldType::PIE))
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
// Maps where specific Phase 3 features MUST exercise to pass.
// ────────────────────────────────────────────────────────────────

/** Returns true if this map name requires at least one overtake to fire. */
static bool MapRequiresOvertaking(const FString& MapName)
{
	return MapName.Contains(TEXT("Overtaking"));
}

/** Returns true if this map name requires measurable speed variation. */
static bool MapRequiresSpeedVariation(const FString& MapName)
{
	return MapName.Contains(TEXT("Personality")) || MapName.Contains(TEXT("MultiLane"));
}

// ────────────────────────────────────────────────────────────────
// Latent command: run simulation for N seconds, sampling state.
// ────────────────────────────────────────────────────────────────

class FRunSimulationAndValidate : public IAutomationLatentCommand
{
public:
	FRunSimulationAndValidate(FAutomationTestBase* InTest, float InDuration, const FString& InMapName)
		: Test(InTest), SimulationDurationSec(InDuration), MapName(InMapName)
	{}

	virtual bool Update() override;

private:
	FAutomationTestBase* Test;
	float SimulationDurationSec;
	FString MapName;
};

bool FRunSimulationAndValidate::Update()
{
	UWorld* World = nullptr;
	if (GEngine)
	{
		for (const FWorldContext& Ctx : GEngine->GetWorldContexts())
		{
			if (Ctx.World() && (Ctx.WorldType == EWorldType::Game || Ctx.WorldType == EWorldType::PIE))
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

	// ── Warmup: skip sampling for the first 2 seconds ──────
	// After a map transition, remnant actors from the previous
	// world may survive for a few frames before being cleaned
	// up by the engine. Sampling during this window incorrectly
	// attributes stale actor state to the new test.
	constexpr float WarmupSec = 2.0f;
	if (ElapsedTime < WarmupSec)
	{
		return false;
	}

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
			}
			else
			{
				if (Record.bWasTraversing)
				{
					Record.bCompletedTurn = true;
				}
				Record.bWasTraversing = false;
			}

			Record.bCurrentlyWaiting =
				(Diag.JunctionPhase == EJunctionPhase::Waiting ||
				 Diag.JunctionPhase == EJunctionPhase::Approaching);

			const bool bTraversingJunction =
				(Diag.JunctionPhase == EJunctionPhase::Traversing);

			// ── Phase 3 feature-exercise sampling ──────────
			if (Diag.OvertakePhase != EOvertakePhase::None)
			{
				Record.bEverOvertook = true;
			}
			if (Record.AssignedTargetSpeed == 0.0f && Diag.TargetSpeedCmPerSec > 0.0f)
			{
				Record.AssignedTargetSpeed = Diag.TargetSpeedCmPerSec;
			}

			// ── Condition 5: Collision detection ───────────
			// Only flag vehicle-vehicle collisions. Road geometry
			// scrapes (curbs/barriers) are not simulation failures.
			if (Diag.CollisionBrakeTimer > 0.0f
				&& Diag.bCollisionWithVehicle)
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
			// Check distance to the active path:
			//   - During junction traversal → distance to junction curve
			//   - Otherwise → distance to nearest lane centerline
			if (!Record.bWentOffRoad)
			{
				const bool bOnJunctionCurve =
					(Diag.JunctionPhase == EJunctionPhase::Traversing
					 && Diag.JunctionCurveDistanceCm > 0.0f);

				if (bOnJunctionCurve)
				{
					// Junction curve off-road: LaneWidth/2 + 200cm margin.
					const float LaneW = Controller->GetLaneWidth();
					const float Threshold = (LaneW * 0.5f) + 200.0f;
					if (Diag.JunctionCurveDistanceCm > Threshold)
					{
						Test->AddError(FString::Printf(
							TEXT("OFF-ROAD: '%s' at (%.0f, %.0f, %.0f) — %.0f cm from junction curve (threshold=%.0f)"),
							*Record.PawnName, VehiclePos.X, VehiclePos.Y, VehiclePos.Z,
							Diag.JunctionCurveDistanceCm, Threshold));
						Record.bWentOffRoad = true;
					}
				}
				else
				{
					const FTrafficLaneHandle NearLane =
						UTrafficRoadQueryLibrary::GetLaneAtLocation(World, VehiclePos);
					if (NearLane.IsValid())
					{
						TArray<FVector> LanePoints;
						float LaneWidth = 0.0f;
						if (UTrafficRoadQueryLibrary::GetLanePath(World, NearLane, LanePoints, LaneWidth))
						{
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
						Test->AddError(FString::Printf(
							TEXT("OFF-ROAD: '%s' at (%.0f, %.0f, %.0f) — no lane found at location"),
							*Record.PawnName, VehiclePos.X, VehiclePos.Y, VehiclePos.Z));
						Record.bWentOffRoad = true;
					}
				}
			}

			// ── Condition 7: Wrong-way detection ───────────
			// Check heading against the active path direction:
			//   - During junction traversal → junction curve tangent
			//   - Otherwise → nearest lane segment direction
			if (!Record.bWrongWay && !Record.bCurrentlyWaiting)
			{
				FVector ReferenceDir = FVector::ZeroVector;

				// Use junction curve direction if available.
				if (!Diag.JunctionCurveDirection.IsNearlyZero())
				{
					ReferenceDir = Diag.JunctionCurveDirection;
				}
				else
				{
					const TArray<FVector>& LanePts = Controller->GetLanePoints();
					if (LanePts.Num() >= 2)
					{
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
						ReferenceDir = (LanePts[DirIdx + 1] - LanePts[DirIdx]).GetSafeNormal2D();
					}
				}

				if (!ReferenceDir.IsNearlyZero())
				{
					const float HeadingDot = FVector::DotProduct(
						FVector(VehicleFwd.X, VehicleFwd.Y, 0.0f).GetSafeNormal(), ReferenceDir);

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

	// ── Phase 3 feature-exercise validation ────────────────
	// These are map-aware: only fail if the map is specifically designed
	// to test a feature and that feature never fired.

	// Overtaking: at least 1 vehicle must have entered an overtake phase.
	int32 OvertakeCount = 0;
	for (const auto& Pair : VehicleRecords)
	{
		if (Pair.Value.bEverOvertook) { ++OvertakeCount; }
	}

	if (MapRequiresOvertaking(MapName))
	{
		if (OvertakeCount == 0)
		{
			Test->AddError(TEXT("FAIL: Overtaking map but NO vehicle ever initiated an overtake."));
		}
		else
		{
			Test->AddInfo(FString::Printf(
				TEXT("Overtaking exercised: %d vehicle(s) overtook during simulation."),
				OvertakeCount));
		}
	}
	else if (OvertakeCount > 0)
	{
		Test->AddInfo(FString::Printf(
			TEXT("Overtaking observed: %d vehicle(s) overtook (not required for this map)."),
			OvertakeCount));
	}

	// Speed variation: vehicles must have measurably different target speeds.
	TArray<float> UniqueTargetSpeeds;
	for (const auto& Pair : VehicleRecords)
	{
		if (Pair.Value.AssignedTargetSpeed > 0.0f)
		{
			bool bAlreadySeen = false;
			for (float S : UniqueTargetSpeeds)
			{
				if (FMath::IsNearlyEqual(S, Pair.Value.AssignedTargetSpeed, 1.0f))
				{
					bAlreadySeen = true;
					break;
				}
			}
			if (!bAlreadySeen)
			{
				UniqueTargetSpeeds.Add(Pair.Value.AssignedTargetSpeed);
			}
		}
	}

	if (MapRequiresSpeedVariation(MapName))
	{
		if (UniqueTargetSpeeds.Num() < 2)
		{
			Test->AddError(FString::Printf(
				TEXT("FAIL: Personality map but only %d distinct target speed(s) observed (need >=2)."),
				UniqueTargetSpeeds.Num()));
		}
		else
		{
			Test->AddInfo(FString::Printf(
				TEXT("Speed variation exercised: %d distinct target speeds observed across %d vehicles."),
				UniqueTargetSpeeds.Num(), TotalVehicles));
		}
	}
	else if (UniqueTargetSpeeds.Num() > 1)
	{
		Test->AddInfo(FString::Printf(
			TEXT("Speed variation observed: %d distinct speeds (not required for this map)."),
			UniqueTargetSpeeds.Num()));
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

	// Step 1: Load the map.
	ADD_LATENT_AUTOMATION_COMMAND(FLoadGameMapCommand(MapPath));
	ADD_LATENT_AUTOMATION_COMMAND(FWaitForMapToLoadCommand());

	// Step 2: Wait for vehicles to spawn.
	ADD_LATENT_AUTOMATION_COMMAND(FWaitForVehicleSpawn(this));

	// Step 3: Let the simulation run for 120 seconds (2 minutes), sampling every 0.5s.
	const FString MapName = FPaths::GetBaseFilename(MapPath);
	ADD_LATENT_AUTOMATION_COMMAND(FRunSimulationAndValidate(this, 120.0f, MapName));

	return true;
}

#endif // WITH_DEV_AUTOMATION_TESTS
