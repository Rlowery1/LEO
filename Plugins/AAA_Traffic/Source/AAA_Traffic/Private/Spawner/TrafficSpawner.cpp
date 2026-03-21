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
#include "Engine/World.h"
#include "TimerManager.h"
#include "EngineUtils.h"
#include "GameFramework/PlayerController.h"
#include "GameFramework/Pawn.h"

// Global debug draw CVar — declared in TrafficVehicleController.cpp.
extern int32 GTrafficDebugDraw;

// Junction diagnostics CVar — declared in TrafficVehicleController.cpp.
extern int32 GTrafficJunctionDiagnostics;

ATrafficSpawner::ATrafficSpawner()
	: VehicleCount(1)
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
		World->GetTimerManager().ClearTimer(RespawnTimerHandle);
	}

	OwnedVehicles.Empty();

	Super::EndPlay(EndPlayReason);
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

	// Queue the lane for respawn.
	if (Lane.IsValid())
	{
		RespawnQueue.Add(Lane);
	}
	else if (CachedAllLanes.Num() > 0)
	{
		// If lane is unknown, pick one deterministically.
		FRandomStream RespawnRng(SpawnSeed + TotalVehiclesSpawned + RespawnCounter);
		const int32 Idx = RespawnRng.RandRange(0, CachedAllLanes.Num() - 1);
		RespawnQueue.Add(CachedAllLanes[Idx]);
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
		// Use RespawnCounter as slot offset to stagger respawn positions
		// and avoid clumping when multiple respawns target the same lane.
		SpawnSingleVehicle(World, Provider, Lane, RespawnCounter, VehicleIdx);

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
		UE_LOG(LogAAATraffic, Error, TEXT("TrafficSpawner: No VehicleClass or VehicleClasses assigned."));
		return;
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
		UE_LOG(LogAAATraffic, Warning, TEXT("TrafficSpawner: All lanes are junction lanes — no spawn candidates!"));
		return;
	}

	// Cache filtered lanes for respawn use.
	CachedAllLanes = AllLanes;

	// Allow VehicleCount to exceed lane count — vehicles share lanes with
	// staggered positioning via SpawnSpacing.
	const int32 SpawnCount = VehicleCount;

	UE_LOG(LogAAATraffic, Log,
		TEXT("TrafficSpawner: Spawning %d vehicles across %d available lanes (junction lanes excluded)."),
		SpawnCount, AllLanes.Num());

	// Track how many vehicles have been placed on each lane for staggered positioning.
	TMap<int32, int32> LaneOccupancy;

	for (int32 i = 0; i < SpawnCount; ++i)
	{
		const FTrafficLaneHandle& Lane = AllLanes[i % AllLanes.Num()];
		const int32 SlotIndex = LaneOccupancy.FindOrAdd(Lane.HandleId, 0);
		LaneOccupancy[Lane.HandleId] = SlotIndex + 1;

		SpawnSingleVehicle(World, Provider, Lane, SlotIndex, i);
	}

	TotalVehiclesSpawned = SpawnCount;
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

