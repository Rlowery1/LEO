// Copyright AAA_Traffic Contributors. All Rights Reserved.

#include "TrafficSpawner.h"
#include "TrafficSubsystem.h"
#include "TrafficRoadProvider.h"
#include "RoadBLDReflectionProvider.h"
#include "TrafficVehicleController.h"
#include "TrafficSignalController.h"
#include "TrafficVehiclePool.h"
#include "TrafficLog.h"
#include "ChaosWheeledVehicleMovementComponent.h"
#include "ChaosVehicleWheel.h"
#include "WheeledVehiclePawn.h"
#include "AssetRegistry/IAssetRegistry.h"
#include "Engine/World.h"
#include "TimerManager.h"
#include "EngineUtils.h"
#include "GameFramework/PlayerController.h"
#include "GameFramework/Pawn.h"

// Global debug draw CVar — declared in TrafficVehicleController.cpp.
extern int32 GTrafficDebugDraw;

// Path-only debug draw CVar — declared in TrafficVehicleController.cpp.
extern int32 GTrafficDebugDrawPaths;

// Junction diagnostics CVar — declared in TrafficVehicleController.cpp.
extern int32 GTrafficJunctionDiagnostics;

ATrafficSpawner::ATrafficSpawner()
	: VehicleCount(1)
	, InitialSpawnBatchSize(4)
	, InitialSpawnBatchInterval(0.05f)
	, VehicleSpeed(2012.f)
	, SpawnSeed(42)
	, SpawnZOffset(50.f)
	, SpawnSpacing(1500.f)
	, SpeedVariation(15.f)
	, LaneChangeAggression(0.5f)
	, bEnableRespawn(true)
	, RespawnCheckInterval(2.0f)
	, MinRespawnDistance(10000.f)
	, DefaultSpeedLimit(2012.0f)
	, ResidentialSpeed(1118.0f)
	, UrbanSpeed(2012.0f)
	, HighwaySpeed(2906.0f)
{
#if defined(ENABLE_DRAW_DEBUG) && ENABLE_DRAW_DEBUG
	PrimaryActorTick.bCanEverTick = true;
	PrimaryActorTick.bStartWithTickEnabled = true; // Tick in editor for debug lane draw
#else
	PrimaryActorTick.bCanEverTick = false;
#endif
}

void ATrafficSpawner::BeginPlay()
{
	Super::BeginPlay();

	// Defer to next tick so adapter subsystems have finished OnWorldBeginPlay registration.
	GetWorldTimerManager().SetTimerForNextTick(this, &ATrafficSpawner::SpawnVehicles);

#if defined(ENABLE_DRAW_DEBUG) && ENABLE_DRAW_DEBUG
	if (bDebugDrawLanes || bDebugDrawIntersections)
	{
		SetActorTickEnabled(true);
	}
#endif
}

bool ATrafficSpawner::ShouldTickIfViewportsOnly() const
{
#if defined(ENABLE_DRAW_DEBUG) && ENABLE_DRAW_DEBUG
	return bDebugDrawLanes || bDebugDrawIntersections;
#else
	return false;
#endif
}

void ATrafficSpawner::Tick(float DeltaSeconds)
{
	Super::Tick(DeltaSeconds);

#if defined(ENABLE_DRAW_DEBUG) && ENABLE_DRAW_DEBUG
	// --- Mode 2 of traffic.DebugDrawPaths: draw all precomputed canonical corridors ---
	if (GTrafficDebugDrawPaths >= 2)
	{
		UWorld* World = GetWorld();
		if (World)
		{
			if (UTrafficSubsystem* TrafficSub = World->GetSubsystem<UTrafficSubsystem>())
			{
				static constexpr float ZLift = 60.0f;
				for (const auto& Pair : TrafficSub->GetAllCanonicalMovements())
				{
					const FCanonicalMovementRecord& Record = Pair.Value;
					if (Record.CorridorPoints.Num() < 2) { continue; }

					// Orange for provider-derived curves, red for synthesized (straight-line) fallbacks.
					const FColor CurveColor = (Record.SourceKind == ECanonicalMovementSourceKind::ProviderDerived)
						? FColor::Orange : FColor::Red;

					for (int32 i = 0; i < Record.CorridorPoints.Num() - 1; ++i)
					{
						const FVector A = Record.CorridorPoints[i] + FVector(0, 0, ZLift);
						const FVector B = Record.CorridorPoints[i + 1] + FVector(0, 0, ZLift);
						DrawDebugLine(World, A, B, CurveColor,
							false, -1.0f, SDPG_Foreground, 4.0f);
					}

					// Start/end markers.
					DrawDebugSphere(World,
						Record.CorridorPoints[0] + FVector(0, 0, ZLift),
						20.0f, 4, FColor::Green, false, -1.0f, SDPG_Foreground);
					DrawDebugSphere(World,
						Record.CorridorPoints.Last() + FVector(0, 0, ZLift),
						20.0f, 4, FColor::Red, false, -1.0f, SDPG_Foreground);
				}
			}
		}
	}

	if (!bDebugDrawLanes && !bDebugDrawIntersections)
	{
		return;
	}

	const bool bDrawLanes = bDebugDrawLanes || (GTrafficDebugDraw != 0);
	const bool bDrawIntersections = bDebugDrawIntersections || (GTrafficDebugDraw != 0);

	if (bDrawLanes && !bDebugCacheReady && !bDebugCacheAttempted)
	{
		CacheDebugLaneData();
	}

	if (bDrawIntersections && !bIntersectionCacheReady && !bIntersectionCacheAttempted)
	{
		CacheDebugIntersectionData();
	}

	if (bDrawLanes && bDebugCacheReady)
	{
		DrawDebugLanes();
	}

	if (bDrawIntersections && bIntersectionCacheReady)
	{
		DrawDebugIntersections();
	}
#endif
}

void ATrafficSpawner::EndPlay(const EEndPlayReason::Type EndPlayReason)
{
	// Unsubscribe from despawn delegate to prevent dangling callbacks.
	if (UWorld* World = GetWorld())
	{
		if (UTrafficSubsystem* TrafficSub = World->GetSubsystem<UTrafficSubsystem>())
		{
			TrafficSub->OnVehicleDespawned.RemoveAll(this);
			TrafficSub->OnProviderRegistered.RemoveAll(this);
		}
		World->GetTimerManager().ClearTimer(InitialSpawnBatchTimerHandle);
		World->GetTimerManager().ClearTimer(RespawnTimerHandle);
	}

	OwnedVehicles.Empty();

	Super::EndPlay(EndPlayReason);
}

void ATrafficSpawner::ProcessInitialSpawnBatch()
{
	UWorld* World = GetWorld();
	if (!World)
	{
		return;
	}

	UTrafficSubsystem* TrafficSub = World->GetSubsystem<UTrafficSubsystem>();
	ITrafficRoadProvider* Provider = TrafficSub ? TrafficSub->GetProvider() : nullptr;
	if (!Provider)
	{
		UE_LOG(LogAAATraffic, Warning,
			TEXT("TrafficSpawner: Initial spawn batch deferred because provider is unavailable."));
		return;
	}

	const int32 BatchSize = FMath::Max(1, InitialSpawnBatchSize);
	const int32 BatchEnd = FMath::Min(PendingInitialSpawnCursor + BatchSize, PendingInitialSpawns.Num());
	for (; PendingInitialSpawnCursor < BatchEnd; ++PendingInitialSpawnCursor)
	{
		const FPendingInitialSpawn& Request = PendingInitialSpawns[PendingInitialSpawnCursor];
		SpawnSingleVehicle(World, Provider, Request.Lane, Request.SlotIndex, Request.VehicleIndex);
	}

	if (PendingInitialSpawnCursor >= PendingInitialSpawns.Num())
	{
		World->GetTimerManager().ClearTimer(InitialSpawnBatchTimerHandle);
		PendingInitialSpawns.Empty();
		PendingInitialSpawnCursor = 0;
		FinalizeSpawnSetup(World, TrafficSub);
	}
}

void ATrafficSpawner::FinalizeSpawnSetup(UWorld* World, UTrafficSubsystem* TrafficSub)
{
	bSpawnComplete = true;

	// Unbind delegate now that spawning is complete (review feedback: keep delegate list clean).
	if (TrafficSub)
	{
		TrafficSub->OnProviderRegistered.RemoveAll(this);
	}

	// Subscribe to despawn events for respawning.
	if (bEnableRespawn && TrafficSub)
	{
		TrafficSub->OnVehicleDespawned.AddUObject(this, &ATrafficSpawner::OnVehicleDespawned);

		World->GetTimerManager().SetTimer(
			RespawnTimerHandle,
			FTimerDelegate::CreateUObject(this, &ATrafficSpawner::CheckRespawn),
			RespawnCheckInterval,
			/*bLoop=*/ true);

		UE_LOG(LogAAATraffic, Log,
			TEXT("TrafficSpawner: Respawn enabled — checking every %.1f seconds."),
			RespawnCheckInterval);
	}
}


void ATrafficSpawner::SpawnSingleVehicle(UWorld* World, ITrafficRoadProvider* Provider,
	const FTrafficLaneHandle& Lane, int32 SlotIndex, int32 VehicleIndex)
{
	TArray<FVector> LanePoints;
	float LaneWidth;
	if (!Provider->GetLanePath(Lane, LanePoints, LaneWidth) || LanePoints.Num() < 2)
	{
		UE_LOG(LogAAATraffic, Warning,
			TEXT("TrafficSpawner: Could not get path for lane %d — skipping."),
			Lane.HandleId);
		return;
	}

	const float TargetOffset = SlotIndex * SpawnSpacing;

	// Walk along lane points to find the staggered spawn position.
	FVector SpawnPos = LanePoints[0];
	FVector SpawnDir = (LanePoints[1] - LanePoints[0]).GetSafeNormal();
	float AccumulatedDist = 0.0f;
	for (int32 p = 0; p < LanePoints.Num() - 1; ++p)
	{
		const float SegLen = FVector::Dist(LanePoints[p], LanePoints[p + 1]);
		if (AccumulatedDist + SegLen >= TargetOffset)
		{
			const float Alpha = (TargetOffset - AccumulatedDist) / FMath::Max(SegLen, KINDA_SMALL_NUMBER);
			SpawnPos = FMath::Lerp(LanePoints[p], LanePoints[p + 1], Alpha);
			SpawnDir = (LanePoints[p + 1] - LanePoints[p]).GetSafeNormal();
			break;
		}
		AccumulatedDist += SegLen;
	}

	// If the requested offset exceeds the lane length, clamp to the lane end.
	if (TargetOffset > AccumulatedDist && LanePoints.Num() >= 2)
	{
		const int32 LastIdx = LanePoints.Num() - 1;
		SpawnPos = LanePoints[LastIdx];
		SpawnDir = (LanePoints[LastIdx] - LanePoints[LastIdx - 1]).GetSafeNormal();
	}

	// Curvature-aware spawn: avoid placing vehicles on tight curves where
	// they'd immediately need emergency braking. If the spawn point is on
	// a curve with radius < 25m (κ > 0.0004), skip this spawn slot.
	// This prevents freshly spawned vehicles from immediately departing
	// their lane on sharp curves.
	{
		// Find the polyline index closest to the spawn position.
		int32 SpawnIdx = 0;
		float BestDSq = TNumericLimits<float>::Max();
		for (int32 i = 0; i < LanePoints.Num(); ++i)
		{
			const float DSq = FVector::DistSquared(SpawnPos, LanePoints[i]);
			if (DSq < BestDSq)
			{
				BestDSq = DSq;
				SpawnIdx = i;
			}
		}

		// Compute Menger curvature at the spawn point.
		if (LanePoints.Num() >= 3 && SpawnIdx > 0 && SpawnIdx < LanePoints.Num() - 1)
		{
			const FVector& A = LanePoints[FMath::Max(0, SpawnIdx - 2)];
			const FVector& B = LanePoints[SpawnIdx];
			const FVector& C = LanePoints[FMath::Min(LanePoints.Num() - 1, SpawnIdx + 2)];
			const float AB = FVector::Dist2D(A, B);
			const float BC = FVector::Dist2D(B, C);
			const float CA = FVector::Dist2D(C, A);
			const float Denom = AB * BC * CA;
			if (Denom > KINDA_SMALL_NUMBER)
			{
				const float SignedArea2 = (B.X - A.X) * (C.Y - A.Y) - (B.Y - A.Y) * (C.X - A.X);
				const float Curvature = FMath::Abs(2.0f * SignedArea2 / Denom);
				if (Curvature > 0.0004f) // radius < 25m
				{
					UE_LOG(LogAAATraffic, Log,
						TEXT("TrafficSpawner: Skipping vehicle %d on lane %d — "
							 "spawn on tight curve (κ=%.5f, R≈%.0fm)."),
						VehicleIndex, Lane.HandleId, Curvature,
						Curvature > KINDA_SMALL_NUMBER ? 1.0f / Curvature / 100.0f : 999999.0f);
					return;
				}
			}
		}
	}

	const FVector SpawnLocation = SpawnPos + FVector(0.0, 0.0, SpawnZOffset);
	const FRotator SpawnRotation = SpawnDir.Rotation();
	const FTransform SpawnTransform(SpawnRotation, SpawnLocation);

	// FIX (Phase 3): Pre-spawn overlap check — skip this slot if an existing
	// traffic vehicle is already within SpawnSpacing. Without this, the
	// AdjustIfPossibleButAlwaysSpawn policy can stack vehicles on top of each
	// other, causing immediate physics explosions on spawn.
	{
		UTrafficSubsystem* OverlapSub = World->GetSubsystem<UTrafficSubsystem>();
		if (OverlapSub)
		{
			TArray<ATrafficVehicleController*> Nearby = OverlapSub->GetNearbyVehicles(SpawnLocation, SpawnSpacing);
			if (Nearby.Num() > 0)
			{
				UE_LOG(LogAAATraffic, Log,
					TEXT("TrafficSpawner: Skipping vehicle %d on lane %d — "
						 "existing vehicle too close (within %.0f cm)."),
					VehicleIndex, Lane.HandleId, SpawnSpacing);
				return;
			}
		}
	}

	// Select vehicle class via weighted random (C1: vehicle variety).
	const TSubclassOf<APawn> ChosenClass = SelectVehicleClass(VehicleIndex);
	if (!ChosenClass)
	{
		UE_LOG(LogAAATraffic, Warning,
			TEXT("TrafficSpawner: No valid vehicle class for vehicle %d."), VehicleIndex);
		return;
	}

	// Try to acquire from the vehicle pool first (I1: object pooling).
	APawn* Vehicle = nullptr;
	UTrafficSubsystem* TrafficSub = World->GetSubsystem<UTrafficSubsystem>();
	UTrafficVehiclePool* Pool = TrafficSub ? TrafficSub->GetVehiclePool() : nullptr;
	if (Pool)
	{
		Vehicle = Pool->AcquireVehicle(World, ChosenClass, SpawnTransform);
	}

	if (!Vehicle)
	{
		FActorSpawnParameters SpawnParams;
		SpawnParams.SpawnCollisionHandlingOverride =
			ESpawnActorCollisionHandlingMethod::AdjustIfPossibleButAlwaysSpawn;
		Vehicle = World->SpawnActor<APawn>(ChosenClass, SpawnTransform, SpawnParams);
	}
	if (!Vehicle)
	{
		UE_LOG(LogAAATraffic, Warning,
			TEXT("TrafficSpawner: Failed to spawn vehicle %d."), VehicleIndex);
		return;
	}

	ATrafficVehicleController* Controller =
		World->SpawnActor<ATrafficVehicleController>();
	if (Controller)
	{
		Controller->SetRandomSeed(SpawnSeed + VehicleIndex);

		// Apply deterministic per-vehicle speed variation.
		float FinalSpeed = VehicleSpeed;
		if (SpeedVariation > KINDA_SMALL_NUMBER)
		{
			constexpr int32 SpeedVariationSeedOffset = 10000;
			FRandomStream SpeedRng(SpawnSeed + VehicleIndex + SpeedVariationSeedOffset);
			const float VariationFraction = SpeedVariation / 100.0f;
			const float Offset = SpeedRng.FRandRange(-VariationFraction, VariationFraction);
			FinalSpeed = VehicleSpeed * (1.0f + Offset);
			FinalSpeed = FMath::Max(FinalSpeed, 0.0f);
		}
		Controller->SetTargetSpeed(FinalSpeed);
		Controller->SetLaneChangeAggression(LaneChangeAggression);

		// Per-road speed limit: check the override map via the subsystem,
		// falling back to the spawner's DefaultSpeedLimit (UrbanSpeed by default).
		float EffectiveSpeedLimit = DefaultSpeedLimit;
		{
			UTrafficSubsystem* SpawnSub = World->GetSubsystem<UTrafficSubsystem>();
			ITrafficRoadProvider* SpawnProv = SpawnSub ? SpawnSub->GetProvider() : nullptr;
			if (SpawnProv && SpawnSub)
			{
				const FTrafficRoadHandle Road = SpawnProv->GetRoadForLane(Lane);
				const float RoadLimit = SpawnSub->GetRoadSpeedLimit(Road.HandleId);
				if (RoadLimit > 0.0f)
				{
					EffectiveSpeedLimit = RoadLimit;
				}
			}
		}
		Controller->SetDefaultSpeedLimit(EffectiveSpeedLimit);

		Controller->Possess(Vehicle);
		Controller->InitializeLaneFollowing(Lane);

		// Immediately register in the spatial grid so subsequent same-frame
		// spawns detect this vehicle in the overlap check. Without this,
		// UpdateVehiclePosition only runs on the next Tick, leaving a one-frame
		// window where GetNearbyVehicles returns empty and multiple vehicles
		// can stack on the same lane start.
		if (TrafficSub)
		{
			TrafficSub->UpdateVehiclePosition(Controller, SpawnLocation);
		}

		OwnedVehicles.Add(Controller);

		// --- Spawner diagnostic: confirm possession and movement readiness ---
		{
			UPawnMovementComponent* MC = Vehicle->GetMovementComponent();
			UChaosWheeledVehicleMovementComponent* ChaosMC =
				Cast<UChaosWheeledVehicleMovementComponent>(MC);
			UE_LOG(LogAAATraffic, Log,
				TEXT("TrafficSpawner: Vehicle %d spawned on lane %d. "
					 "Class='%s' MovementComp='%s' ChaosWheeledCast=%s Possessed=%s"),
				VehicleIndex, Lane.HandleId,
				*Vehicle->GetClass()->GetName(),
				MC ? *MC->GetClass()->GetName() : TEXT("NULL"),
				ChaosMC ? TEXT("OK") : TEXT("FAILED"),
				Controller->GetPawn() ? TEXT("Yes") : TEXT("No"));
		}
	}
}

void ATrafficSpawner::OnVehicleDespawned(ATrafficVehicleController* Controller, const FTrafficLaneHandle& Lane)
{
	if (!bEnableRespawn || !bSpawnComplete) { return; }

	// Only respond to vehicles owned by this spawner.
	if (!Controller || !OwnedVehicles.Remove(Controller))
	{
		return;
	}

	// Queue a respawn-safe lane (non-dead-end). If the vehicle was on a
	// dead-end lane, redirect to a random respawn-safe lane instead.
	if (CachedRespawnLanes.Num() == 0) { return; }

	if (Lane.IsValid() && CachedRespawnLanes.Contains(Lane))
	{
		RespawnQueue.Add(Lane);
	}
	else
	{
		// Lane is invalid or a dead-end — pick a respawn-safe lane deterministically.
		FRandomStream RespawnRng(SpawnSeed + TotalVehiclesSpawned + RespawnCounter);
		const int32 Idx = RespawnRng.RandRange(0, CachedRespawnLanes.Num() - 1);
		RespawnQueue.Add(CachedRespawnLanes[Idx]);
	}
}

void ATrafficSpawner::CheckRespawn()
{
	if (RespawnQueue.IsEmpty()) { return; }

	UWorld* World = GetWorld();
	if (!World) { return; }

	UTrafficSubsystem* TrafficSub = World->GetSubsystem<UTrafficSubsystem>();
	ITrafficRoadProvider* Provider = TrafficSub ? TrafficSub->GetProvider() : nullptr;
	if (!Provider) { return; }

	// Gather player positions for distance validation.
	TArray<FVector> PlayerPositions;
	for (FConstPlayerControllerIterator It = World->GetPlayerControllerIterator(); It; ++It)
	{
		if (const APlayerController* PC = It->Get())
		{
			if (const APawn* PlayerPawn = PC->GetPawn())
			{
				PlayerPositions.Add(PlayerPawn->GetActorLocation());
			}
		}
	}

	const float MinDistSq = MinRespawnDistance * MinRespawnDistance;

	// Process queued respawns.
	TArray<FTrafficLaneHandle> Remaining;
	for (const FTrafficLaneHandle& Lane : RespawnQueue)
	{
		// Get lane start position to check against player distances.
		TArray<FVector> LanePoints;
		float LaneWidth;
		if (!Provider->GetLanePath(Lane, LanePoints, LaneWidth) || LanePoints.Num() < 2)
		{
			continue; // Lane no longer valid — drop.
		}

		const FVector SpawnCandidate = LanePoints[0] + FVector(0, 0, SpawnZOffset);

		// Ensure spawn location is far enough from all players.
		bool bTooClose = false;
		for (const FVector& PlayerPos : PlayerPositions)
		{
			if (FVector::DistSquared(SpawnCandidate, PlayerPos) < MinDistSq)
			{
				bTooClose = true;
				break;
			}
		}

		if (bTooClose)
		{
			Remaining.Add(Lane); // Retry next interval.
			continue;
		}

		++RespawnCounter;
		const int32 VehicleIdx = TotalVehiclesSpawned++;
		// Always spawn at lane start (SlotIndex=0). The pre-spawn overlap
		// check inside SpawnSingleVehicle prevents stacking.
		SpawnSingleVehicle(World, Provider, Lane, 0, VehicleIdx);

		UE_LOG(LogAAATraffic, Log,
			TEXT("TrafficSpawner: Respawned vehicle on lane %d (respawn #%d)."),
			Lane.HandleId, RespawnCounter);
	}

	RespawnQueue = MoveTemp(Remaining);
}

void ATrafficSpawner::OnProviderRegistered(ITrafficRoadProvider* Provider)
{
	if (bSpawnComplete) { return; }

#if defined(ENABLE_DRAW_DEBUG) && ENABLE_DRAW_DEBUG
	// Reset debug cache so it picks up provider data on next tick.
	bDebugCacheReady = false;
	bDebugCacheAttempted = false;
	DebugLanes.Empty();
#endif

	UE_LOG(LogAAATraffic, Log,
		TEXT("TrafficSpawner: Provider registered (deferred) — attempting vehicle spawn."));
	SpawnVehicles();
}

void ATrafficSpawner::SpawnVehicles()
{
	if (bSpawnComplete) { return; }

	if (VehicleClasses.Num() == 0 && !VehicleClass)
	{
		// Auto-discover: find a Blueprint Pawn inheriting AWheeledVehiclePawn.
		// Prefer smaller vehicles (names containing "Sedan") over trucks.
		auto IsValidCandidate = [](const UClass* C) -> bool
		{
			return C
				&& C->IsChildOf(AWheeledVehiclePawn::StaticClass())
				&& !C->HasAnyClassFlags(CLASS_Abstract | CLASS_Deprecated | CLASS_NewerVersionExists)
				&& C != AWheeledVehiclePawn::StaticClass();
		};

		UClass* Fallback = nullptr;

		// Pass 1: loaded classes (fast — no disk I/O).
		for (TObjectIterator<UClass> It; It; ++It)
		{
			if (IsValidCandidate(*It))
			{
				if (It->GetName().Contains(TEXT("Sedan")))
				{
					VehicleClass = *It;
					break;
				}
				if (!Fallback) { Fallback = *It; }
			}
		}

		// Pass 2: Asset Registry (catches GC'd / never-loaded BPs).
		if (!VehicleClass && !Fallback)
		{
			if (IAssetRegistry* Registry = IAssetRegistry::Get())
			{
				TArray<FTopLevelAssetPath> BasePaths;
				BasePaths.Add(AWheeledVehiclePawn::StaticClass()->GetClassPathName());
				TSet<FTopLevelAssetPath> DerivedPaths;
				Registry->GetDerivedClassNames(BasePaths, TSet<FTopLevelAssetPath>(), DerivedPaths);
				for (const FTopLevelAssetPath& Path : DerivedPaths)
				{
					UClass* Candidate = FindObject<UClass>(nullptr, *Path.ToString());
					if (!Candidate)
					{
						Candidate = LoadObject<UClass>(nullptr, *Path.ToString());
					}
					if (IsValidCandidate(Candidate))
					{
						if (Candidate->GetName().Contains(TEXT("Sedan")))
						{
							VehicleClass = Candidate;
							break;
						}
						if (!Fallback) { Fallback = Candidate; }
					}
				}
			}
		}

		if (!VehicleClass) { VehicleClass = Fallback; }

		if (VehicleClass)
		{
			UE_LOG(LogAAATraffic, Warning,
				TEXT("TrafficSpawner: No VehicleClass assigned — auto-discovered '%s'"),
				*VehicleClass->GetName());
		}
		else
		{
			UE_LOG(LogAAATraffic, Error, TEXT("TrafficSpawner: No VehicleClass or VehicleClasses assigned and auto-discovery found nothing."));
			return;
		}
	}

	UWorld* World = GetWorld();
	if (!World)
	{
		UE_LOG(LogAAATraffic, Error, TEXT("TrafficSpawner: World is null in SpawnVehicles."));
		return;
	}

	UTrafficSubsystem* TrafficSub = World->GetSubsystem<UTrafficSubsystem>();
	ITrafficRoadProvider* Provider = TrafficSub ? TrafficSub->GetProvider() : nullptr;
	if (!Provider)
	{
		// No provider yet — subscribe for deferred retry.
		if (TrafficSub)
		{
			TrafficSub->OnProviderRegistered.AddUObject(this, &ATrafficSpawner::OnProviderRegistered);
			UE_LOG(LogAAATraffic, Log,
				TEXT("TrafficSpawner: No provider yet — subscribed for deferred retry."));
		}
		else
		{
			UE_LOG(LogAAATraffic, Error,
				TEXT("TrafficSpawner: No TrafficSubsystem found."));
		}
		return;
	}

	// Push spawner speed tiers into the provider so road classification
	// uses user-configured values instead of hardcoded defaults.
	if (auto* RoadBLDProvider = World->GetSubsystem<URoadBLDReflectionProvider>())
	{
		RoadBLDProvider->SetSpeedTiers(ResidentialSpeed, UrbanSpeed, HighwaySpeed);
	}

	// ── Fleet vehicle pre-computation ——————————————————————
	// Scan all vehicle class CDOs BEFORE junction paths are generated
	// (PlaceAutoSignals triggers canonical movement compilation).
	// Extract worst-case min turn radius and max half-width so the
	// provider can generate junction curves safe for every vehicle.
	{
		float WorstMinTurnRadius = TNumericLimits<float>::Max();
		float WorstMaxHalfWidth = 100.0f; // conservative default

		auto ScanVehicleClass = [&](TSubclassOf<APawn> PawnClass)
		{
			if (!PawnClass) { return; }
			const APawn* PawnCDO = PawnClass->GetDefaultObject<APawn>();
			if (!PawnCDO) { return; }

			const UChaosWheeledVehicleMovementComponent* MovComp =
				PawnCDO->FindComponentByClass<UChaosWheeledVehicleMovementComponent>();
			if (!MovComp) { return; }

			float MaxSteerDeg = 0.0f;
			for (const FChaosWheelSetup& WS : MovComp->WheelSetups)
			{
				if (!WS.WheelClass) { continue; }
				const UChaosVehicleWheel* WheelCDO =
					WS.WheelClass->GetDefaultObject<UChaosVehicleWheel>();
				if (WheelCDO && WheelCDO->AxleType == EAxleType::Front)
				{
					MaxSteerDeg = FMath::Max(MaxSteerDeg, WheelCDO->MaxSteerAngle);
				}
			}

			if (MaxSteerDeg > 1.0f)
			{
				// Conservative wheelbase: 280cm default (typical sedan).
				// Actual wheelbase requires bone positions from a spawned mesh.
				constexpr float EstimatedWheelbaseCm = 280.0f;
				const float SteerRad = FMath::DegreesToRadians(
					FMath::Clamp(MaxSteerDeg, 15.0f, 55.0f));
				const float MinRadius = EstimatedWheelbaseCm / FMath::Tan(SteerRad);
				WorstMinTurnRadius = FMath::Min(WorstMinTurnRadius, MinRadius);
			}

			// Try to get vehicle bounds from the CDO for half-width.
			const FBox Bounds = PawnCDO->GetComponentsBoundingBox(false);
			if (Bounds.IsValid)
			{
				const FVector ActorOrigin = PawnCDO->GetActorLocation();
				const FVector Right = PawnCDO->GetActorRightVector();

				FVector RightCorner;
				RightCorner.X = (Right.X >= 0.0f) ? Bounds.Max.X : Bounds.Min.X;
				RightCorner.Y = (Right.Y >= 0.0f) ? Bounds.Max.Y : Bounds.Min.Y;
				RightCorner.Z = (Right.Z >= 0.0f) ? Bounds.Max.Z : Bounds.Min.Z;

				const float HalfWidth = FMath::Clamp(
					FMath::Abs(FVector::DotProduct(RightCorner - ActorOrigin, Right)),
					50.0f,
					300.0f);
				if (HalfWidth > 50.0f)
				{
					WorstMaxHalfWidth = FMath::Max(WorstMaxHalfWidth, HalfWidth);
				}
			}

			UE_LOG(LogAAATraffic, Log,
				TEXT("TrafficSpawner: Fleet scan — '%s' MaxSteer=%.1f° MinTurnR=%.0f cm HalfWidth=%.0f cm"),
				*PawnClass->GetName(), MaxSteerDeg,
				(MaxSteerDeg > 1.0f) ? (280.0f / FMath::Tan(FMath::DegreesToRadians(
					FMath::Clamp(MaxSteerDeg, 15.0f, 55.0f)))) : 0.0f,
				WorstMaxHalfWidth);
		};

		if (VehicleClasses.Num() > 0)
		{
			for (const FVehicleClassEntry& Entry : VehicleClasses)
			{
				ScanVehicleClass(Entry.VehicleClass);
			}
		}
		else
		{
			ScanVehicleClass(VehicleClass);
		}

		if (WorstMinTurnRadius < TNumericLimits<float>::Max())
		{
			// Add 15% safety margin — real steering performance varies with speed.
			WorstMinTurnRadius *= 1.15f;
			Provider->SetFleetVehicleConstraints(WorstMinTurnRadius, WorstMaxHalfWidth);
			if (TrafficSub)
			{
				TrafficSub->RebuildJunctionSurvey();
			}
		}
	}

	// Gather all lanes across all roads.
	TArray<FTrafficRoadHandle> Roads = Provider->GetAllRoads();
	if (Roads.IsEmpty())
	{
		UE_LOG(LogAAATraffic, Warning, TEXT("TrafficSpawner: No roads found by the provider."));
		return;
	}

	// Push per-road speed overrides into the subsystem so controllers can
	// query the correct speed limit when transitioning between roads.
	for (const auto& OverrideEntry : RoadSpeedOverrides)
	{
		TrafficSub->SetRoadSpeedLimit(OverrideEntry.Key, OverrideEntry.Value);
	}

	TArray<FTrafficLaneHandle> AllLanes;
	for (const FTrafficRoadHandle& Road : Roads)
	{
		AllLanes.Append(Provider->GetLanesForRoad(Road));
	}

	if (AllLanes.IsEmpty())
	{
		UE_LOG(LogAAATraffic, Warning, TEXT("TrafficSpawner: No lanes found on any road."));
		return;
	}

	// Auto-place traffic signals at discovered junctions (C2).
	// Must happen before filtering so PlaceAutoSignals sees all lanes.
	PlaceAutoSignals(World, Provider, AllLanes);

	// Filter out junction lanes — vehicles must not spawn inside intersections.
	// Junction lanes are short virtual segments (12-14m) and spawning on them
	// causes vehicles to immediately enter junction logic on their first tick.
	TArray<FTrafficLaneHandle> AllLanesPreJunctionFilter = AllLanes;
	{
		const int32 PreFilterCount = AllLanes.Num();
		AllLanes.RemoveAll([Provider](const FTrafficLaneHandle& Lane)
		{
			return Provider->GetJunctionForLane(Lane) != 0;
		});
		UE_LOG(LogAAATraffic, Log,
			TEXT("TrafficSpawner: Filtered %d junction lanes from spawn pool (%d -> %d)."),
			PreFilterCount - AllLanes.Num(), PreFilterCount, AllLanes.Num());
	}

	// Filter out stub lanes that are too short for a vehicle to spawn on.
	// These occur when a road's intersection mask starts at (or very near)
	// the road endpoint, producing a pre-mask segment of only a few cm.
	{
		constexpr float MinSpawnLaneLengthCm = 500.0f;
		const int32 PreStubCount = AllLanes.Num();
		AllLanes.RemoveAll([Provider, MinSpawnLaneLengthCm](const FTrafficLaneHandle& Lane)
		{
			const float LaneLengthCm = Provider->GetLaneLength(Lane);
			// Treat length <= 0 as "unknown" (provider has no data) — keep the
			// lane rather than wrongly filtering it as a stub.
			return LaneLengthCm > 0.0f && LaneLengthCm < MinSpawnLaneLengthCm;
		});
		const int32 StubsRemoved = PreStubCount - AllLanes.Num();
		if (StubsRemoved > 0)
		{
			UE_LOG(LogAAATraffic, Log,
				TEXT("TrafficSpawner: Filtered %d stub lanes (< %.0f cm) from spawn pool (%d -> %d)."),
				StubsRemoved, MinSpawnLaneLengthCm, PreStubCount, AllLanes.Num());
		}
	}

	if (AllLanes.IsEmpty())
	{
		// All lanes touch junctions (e.g. short road between 2 intersections).
		// Fall back to the longest non-stub junction lanes so vehicles can
		// still spawn. Prefer lanes >= 800cm to give enough room to start.
		constexpr float FallbackMinLengthCm = 800.0f;
		for (const FTrafficLaneHandle& Lane : AllLanesPreJunctionFilter)
		{
			const float Len = Provider->GetLaneLength(Lane);
			if (Len >= FallbackMinLengthCm)
			{
				AllLanes.Add(Lane);
			}
		}
		// If still empty, take any lane that isn't a micro-stub (> 300cm).
		if (AllLanes.IsEmpty())
		{
			for (const FTrafficLaneHandle& Lane : AllLanesPreJunctionFilter)
			{
				const float Len = Provider->GetLaneLength(Lane);
				if (Len > 300.0f || Len <= 0.0f)
				{
					AllLanes.Add(Lane);
				}
			}
		}
		if (AllLanes.IsEmpty())
		{
			UE_LOG(LogAAATraffic, Warning, TEXT("TrafficSpawner: All lanes are junction lanes — no spawn candidates!"));
			return;
		}
		UE_LOG(LogAAATraffic, Warning,
			TEXT("TrafficSpawner: All non-junction lanes filtered — using %d junction-adjacent lanes as fallback."),
			AllLanes.Num());
	}

	// Cache filtered lanes for respawn use.
	CachedAllLanes = AllLanes;

	// Build a respawn-safe subset: lanes that have a viable downstream route.
	// A single immediate connection is not sufficient here because a lane can
	// feed directly into a terminal corridor (for example 25 -> 15 -> dead-end).
	// Prefer lanes whose precomputed forward scan reaches a junction; if none do,
	// fall back to the broader non-dead-end set so pure non-junction maps still work.
	CachedRespawnLanes = AllLanes;
	CachedRespawnLanes.RemoveAll([Provider](const FTrafficLaneHandle& L)
	{
		const TArray<FTrafficLaneHandle> Connected = Provider->GetConnectedLanes(L);
		if (Connected.IsEmpty())
		{
			return true;
		}

		const float LaneLengthCm = Provider->GetLaneLength(L);
		const ITrafficRoadProvider::FJunctionScanResult Scan =
			Provider->GetDistanceToNextJunction(L, LaneLengthCm, 50000.0f, 10);
		return !Scan.IsValid();
	});

	// Filter out lanes whose downstream junction has no canonical movements
	// for them. A lane may reach a junction geometrically but the survey may
	// have rejected all exits (e.g. same-road opposing with a single exit),
	// making it a network trap. Only apply if the subsystem is available and
	// has canonical data compiled.
	if (TrafficSub)
	{
		const int32 PreCanonicalCount = CachedRespawnLanes.Num();
		CachedRespawnLanes.RemoveAll([Provider, TrafficSub](const FTrafficLaneHandle& L)
		{
			// Check if this lane is an approach lane with canonical movements
			const TArray<int32>& DirectMovements =
				TrafficSub->GetCanonicalMovementsForApproachLane(L.HandleId);
			if (!DirectMovements.IsEmpty())
			{
				// Has direct canonical movements — check if any have non-zero weight
				for (const int32 MoveId : DirectMovements)
				{
					const FCanonicalMovementRecord* Record = TrafficSub->GetCanonicalMovement(MoveId);
					if (Record && Record->SelectionWeight > 0.0f)
					{
						return false; // Keep — has viable canonical authority
					}
				}
				return true; // Remove — all canonical movements have zero weight
			}

			// Not a direct approach lane — check if the downstream junction
			// approach lane has canonical movements
			const float LaneLengthCm = Provider->GetLaneLength(L);
			const ITrafficRoadProvider::FJunctionScanResult Scan =
				Provider->GetDistanceToNextJunction(L, LaneLengthCm, 50000.0f, 10);
			if (!Scan.IsValid())
			{
				return false; // No junction downstream — keep (non-junction road)
			}
			if (Scan.ApproachLane.IsValid())
			{
				const TArray<int32>& ApproachMovements =
					TrafficSub->GetCanonicalMovementsForApproachLane(Scan.ApproachLane.HandleId);
				if (ApproachMovements.IsEmpty())
				{
					return true; // Remove — downstream approach has no canonical movements
				}
				for (const int32 MoveId : ApproachMovements)
				{
					const FCanonicalMovementRecord* Record = TrafficSub->GetCanonicalMovement(MoveId);
					if (Record && Record->SelectionWeight > 0.0f)
					{
						return false; // Keep
					}
				}
				return true; // Remove — all downstream movements zero weight
			}

			return false; // Keep by default
		});

		const int32 CanonicalFiltered = PreCanonicalCount - CachedRespawnLanes.Num();
		if (CanonicalFiltered > 0)
		{
			UE_LOG(LogAAATraffic, Log,
				TEXT("TrafficSpawner: Filtered %d lanes with no viable canonical authority at downstream junction (%d -> %d)."),
				CanonicalFiltered, PreCanonicalCount, CachedRespawnLanes.Num());
		}
	}

	if (CachedRespawnLanes.IsEmpty())
	{
		// Fallback: no lane reaches a junction ahead. This is valid on simple
		// straight-road maps, so fall back to immediate non-dead-end lanes.
		CachedRespawnLanes = AllLanes;
		CachedRespawnLanes.RemoveAll([Provider](const FTrafficLaneHandle& L)
		{
			return Provider->GetConnectedLanes(L).IsEmpty();
		});

		if (CachedRespawnLanes.IsEmpty())
		{
			CachedRespawnLanes = AllLanes;
		}
	}

	// Allow VehicleCount to exceed lane count — vehicles share lanes with
	// staggered positioning via SpawnSpacing.
	const int32 SpawnCount = VehicleCount;

	// Use respawn-safe lanes (no dead-ends) for initial spawn so vehicles
	// can immediately participate in the traffic network.
	const TArray<FTrafficLaneHandle>& SpawnLanes = CachedRespawnLanes.Num() > 0 ? CachedRespawnLanes : AllLanes;

	UE_LOG(LogAAATraffic, Log,
		TEXT("TrafficSpawner: Spawning %d vehicles across %d available lanes (%d downstream-terminal lanes excluded)."),
		SpawnCount, SpawnLanes.Num(), AllLanes.Num() - SpawnLanes.Num());

	// Track how many vehicles have been placed on each lane for staggered positioning.
	TMap<int32, int32> LaneOccupancy;
	PendingInitialSpawns.Empty();
	PendingInitialSpawnCursor = 0;
	PendingInitialSpawns.Reserve(SpawnCount);

	for (int32 i = 0; i < SpawnCount; ++i)
	{
		const FTrafficLaneHandle& Lane = SpawnLanes[i % SpawnLanes.Num()];
		const int32 SlotIndex = LaneOccupancy.FindOrAdd(Lane.HandleId, 0);
		LaneOccupancy[Lane.HandleId] = SlotIndex + 1;

		FPendingInitialSpawn& Request = PendingInitialSpawns.Emplace_GetRef();
		Request.Lane = Lane;
		Request.SlotIndex = SlotIndex;
		Request.VehicleIndex = i;
	}

	TotalVehiclesSpawned = SpawnCount;
	if (PendingInitialSpawns.IsEmpty())
	{
		FinalizeSpawnSetup(World, TrafficSub);
		return;
	}

	UE_LOG(LogAAATraffic, Log,
		TEXT("TrafficSpawner: Initial population will be spawned in batches of %d every %.2f seconds."),
		FMath::Max(1, InitialSpawnBatchSize),
		FMath::Max(0.0f, InitialSpawnBatchInterval));

	ProcessInitialSpawnBatch();

	if (PendingInitialSpawns.Num() > 0)
	{
		World->GetTimerManager().SetTimer(
			InitialSpawnBatchTimerHandle,
			FTimerDelegate::CreateUObject(this, &ATrafficSpawner::ProcessInitialSpawnBatch),
			FMath::Max(0.0f, InitialSpawnBatchInterval),
			/*bLoop=*/ true);
	}
}

TSubclassOf<APawn> ATrafficSpawner::SelectVehicleClass(int32 VehicleIndex) const
{
	if (VehicleClasses.Num() == 0)
	{
		return VehicleClass;
	}

	// Build CDF from valid entries.
	float TotalWeight = 0.0f;
	for (const FVehicleClassEntry& Entry : VehicleClasses)
	{
		if (Entry.VehicleClass)
		{
			TotalWeight += Entry.Weight;
		}
	}

	if (TotalWeight <= KINDA_SMALL_NUMBER)
	{
		return VehicleClass; // Fallback to single class.
	}

	constexpr int32 ClassSelectionSeedOffset = 20000;
	FRandomStream ClassRng(SpawnSeed + VehicleIndex + ClassSelectionSeedOffset);
	const float Roll = ClassRng.FRandRange(0.0f, TotalWeight);

	float Cumulative = 0.0f;
	for (const FVehicleClassEntry& Entry : VehicleClasses)
	{
		if (!Entry.VehicleClass) { continue; }
		Cumulative += Entry.Weight;
		if (Roll <= Cumulative)
		{
			return Entry.VehicleClass;
		}
	}

	// Fallback (floating-point edge case).
	for (int32 i = VehicleClasses.Num() - 1; i >= 0; --i)
	{
		if (VehicleClasses[i].VehicleClass)
		{
			return VehicleClasses[i].VehicleClass;
		}
	}
	return VehicleClass;
}

