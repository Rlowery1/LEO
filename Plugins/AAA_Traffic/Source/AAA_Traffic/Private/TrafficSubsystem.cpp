// Copyright AAA_Traffic Contributors. All Rights Reserved.

#include "TrafficSubsystem.h"
#include "TrafficRoadProvider.h"
#include "TrafficVehicleController.h"
#include "TrafficSignalController.h"
#include "TrafficVehiclePool.h"
#include "TrafficLog.h"
#include "Engine/World.h"
#include "TimerManager.h"
#include "GameFramework/PlayerController.h"
#include "GameFramework/Pawn.h"
#include "GameFramework/PawnMovementComponent.h"

DEFINE_LOG_CATEGORY(LogAAATraffic);

const TArray<FJunctionExitRule> UTrafficSubsystem::EmptyExitRules;

UTrafficSubsystem::UTrafficSubsystem()
	: DespawnDistance(50000.f)		// 500m
	, DespawnCheckInterval(1.0f)		// once per second
	, bDespawnDeadEndVehicles(true)
{
}

void UTrafficSubsystem::Initialize(FSubsystemCollectionBase& Collection)
{
	Super::Initialize(Collection);
	VehiclePool = NewObject<UTrafficVehiclePool>(this, TEXT("VehiclePool"));
}

void UTrafficSubsystem::Deinitialize()
{
	if (UWorld* World = GetWorld())
	{
		World->GetTimerManager().ClearTimer(DespawnTimerHandle);
	}
	ActiveVehicles.Empty();
	VehiclesByLane.Empty();
	SpatialGrid.Empty();
	VehicleCellMap.Empty();
	VehicleLODMap.Empty();
	JunctionOccupancy.Empty();
	SignalControllerMap.Empty();
	if (VehiclePool)
	{
		VehiclePool->DrainPool();
	}
	Super::Deinitialize();
}

bool UTrafficSubsystem::DoesSupportWorldType(const EWorldType::Type WorldType) const
{
	return WorldType == EWorldType::Game || WorldType == EWorldType::PIE;
}

void UTrafficSubsystem::RegisterProvider(UObject* InProvider)
{
	if (!InProvider)
	{
		return;
	}

	if (!InProvider->GetClass()->ImplementsInterface(UTrafficRoadProvider::StaticClass()))
	{
		UE_LOG(LogAAATraffic, Warning,
			TEXT("TrafficSubsystem: Object '%s' does not implement ITrafficRoadProvider."),
			*InProvider->GetName());
		return;
	}

	if (ActiveProviderObject && ActiveProviderObject != InProvider)
	{
		UE_LOG(LogAAATraffic, Warning,
			TEXT("TrafficSubsystem: Replacing existing provider '%s' with '%s'."),
			*ActiveProviderObject->GetName(), *InProvider->GetName());
	}

	ActiveProviderObject = InProvider;
	UE_LOG(LogAAATraffic, Log, TEXT("TrafficSubsystem: Provider registered — %s"), *InProvider->GetName());

	// Build the junction exit survey before notifying listeners.
	BuildJunctionSurvey();

	// Notify listeners (e.g. TrafficSpawner deferred retry).
	OnProviderRegistered.Broadcast(Cast<ITrafficRoadProvider>(InProvider));
}

void UTrafficSubsystem::UnregisterProvider(UObject* InProvider)
{
	if (ActiveProviderObject == InProvider)
	{
		ActiveProviderObject = nullptr;
		UE_LOG(LogAAATraffic, Log, TEXT("TrafficSubsystem: Provider unregistered."));
	}
}

ITrafficRoadProvider* UTrafficSubsystem::GetProvider() const
{
	if (!ActiveProviderObject)
	{
		return nullptr;
	}

	return Cast<ITrafficRoadProvider>(ActiveProviderObject);
}

bool UTrafficSubsystem::HasProvider() const
{
	return ActiveProviderObject != nullptr;
}

void UTrafficSubsystem::RegisterVehicle(ATrafficVehicleController* InController)
{
	if (!InController) return;
	InController->DeterministicSpawnIndex = NextSpawnIndex++;
	ActiveVehicles.Add(InController);

	// Start the despawn timer on first vehicle registration.
	if (ActiveVehicles.Num() == 1)
	{
		if (UWorld* World = GetWorld())
		{
			World->GetTimerManager().SetTimer(
				DespawnTimerHandle,
				FTimerDelegate::CreateUObject(this, &UTrafficSubsystem::PerformDespawnSweep),
				DespawnCheckInterval,
				/*bLoop=*/ true);
		}
	}
}

void UTrafficSubsystem::UnregisterVehicle(ATrafficVehicleController* InController)
{
	if (!InController) return;
	ActiveVehicles.RemoveSingle(InController);

	// Remove from per-lane registry.
	for (auto& Pair : VehiclesByLane)
	{
		Pair.Value.Remove(InController);
	}

	// Remove from spatial grid.
	if (const int64* CellKey = VehicleCellMap.Find(InController))
	{
		if (TArray<TWeakObjectPtr<ATrafficVehicleController>>* Cell = SpatialGrid.Find(*CellKey))
		{
			Cell->Remove(InController);
		}
		VehicleCellMap.Remove(InController);
	}
	VehicleLODMap.Remove(InController);

	// Release any junction this vehicle may be occupying.
	for (auto It = JunctionOccupancy.CreateIterator(); It; ++It)
	{
		It.Value().RemoveAll([InController](const FJunctionOccupant& Occ)
		{
			return Occ.Controller.Get() == InController;
		});
		if (It.Value().Num() == 0)
		{
			It.RemoveCurrent();
		}
	}

	// Stop the timer when no vehicles remain.
	if (ActiveVehicles.Num() == 0)
	{
		if (UWorld* World = GetWorld())
		{
			World->GetTimerManager().ClearTimer(DespawnTimerHandle);
		}
	}
}

void UTrafficSubsystem::UpdateVehicleLane(ATrafficVehicleController* InController, const FTrafficLaneHandle& NewLane)
{
	if (!InController) return;

	// Remove from any previous lane.
	for (auto& Pair : VehiclesByLane)
	{
		Pair.Value.Remove(InController);
	}

	// Add to new lane.
	if (NewLane.IsValid())
	{
		VehiclesByLane.FindOrAdd(NewLane.HandleId).Add(InController);
	}
}

TArray<TWeakObjectPtr<ATrafficVehicleController>> UTrafficSubsystem::GetVehiclesOnLane(const FTrafficLaneHandle& Lane) const
{
	if (const TArray<TWeakObjectPtr<ATrafficVehicleController>>* Vehicles = VehiclesByLane.Find(Lane.HandleId))
	{
		return *Vehicles;
	}
	return TArray<TWeakObjectPtr<ATrafficVehicleController>>();
}

void UTrafficSubsystem::PerformDespawnSweep()
{
	UWorld* World = GetWorld();
	if (!World) return;

	// Gather player positions for distance checks.
	TArray<FVector> PlayerPositions;
	for (FConstPlayerControllerIterator It = World->GetPlayerControllerIterator(); It; ++It)
	{
		if (const APlayerController* PC = It->Get())
		{
			if (const APawn* PlayerPawn = PC->GetPawn())
			{
				PlayerPositions.Add(PlayerPawn->GetActorLocation());
			}
			else
			{
				// Fallback to camera location if no pawn.
				FVector CamLoc;
				FRotator CamRot;
				PC->GetPlayerViewPoint(CamLoc, CamRot);
				PlayerPositions.Add(CamLoc);
			}
		}
	}

	// Update LOD tiers while we have player positions handy.
	UpdateLODTiers(PlayerPositions);

	const float DespawnDistSq = DespawnDistance * DespawnDistance;
	const bool bHasPlayers = !PlayerPositions.IsEmpty();

	// Purge stale weak pointers first (backwards to preserve indices).
	for (int32 i = ActiveVehicles.Num() - 1; i >= 0; --i)
	{
		if (!ActiveVehicles[i].IsValid())
		{
			ActiveVehicles.RemoveAt(i);
		}
	}

	// Collect vehicles to despawn.
	TArray<ATrafficVehicleController*> ToDespawn;
	TArray<FString> DespawnReasons;

	for (const TWeakObjectPtr<ATrafficVehicleController>& WeakVC : ActiveVehicles)
	{
		ATrafficVehicleController* Controller = WeakVC.Get();
		if (!Controller) { continue; }

		const APawn* VehiclePawn = Controller->GetPawn();
		if (!VehiclePawn)
		{
			continue;
		}

		// Check 1: dead-end vehicle that has stopped.
		if (bDespawnDeadEndVehicles && Controller->IsAtDeadEnd())
		{
			const float Speed = VehiclePawn->GetVelocity().Size();
			if (Speed < 10.0f) // Nearly stopped
			{
				ToDespawn.Add(Controller);
				DespawnReasons.Add(TEXT("dead-end stopped"));
				continue;
			}
		}

		// Check 2: distance from nearest player (skip if no players present).
		if (!bHasPlayers) continue;

		const FVector VehicleLoc = VehiclePawn->GetActorLocation();
		float NearestDistSq = MAX_flt;
		for (const FVector& PlayerLoc : PlayerPositions)
		{
			NearestDistSq = FMath::Min(NearestDistSq, FVector::DistSquared(VehicleLoc, PlayerLoc));
		}

		if (NearestDistSq >= DespawnDistSq)
		{
			ToDespawn.Add(Controller);
			DespawnReasons.Add(FString::Printf(TEXT("%.0f cm from nearest player"), FMath::Sqrt(NearestDistSq)));
		}
	}

	// Destroy collected vehicles (or pool them if pool is available).
	for (int32 Idx = 0; Idx < ToDespawn.Num(); ++Idx)
	{
		ATrafficVehicleController* Controller = ToDespawn[Idx];
		APawn* VehiclePawn = Controller->GetPawn();
		const FString VehicleName = VehiclePawn ? VehiclePawn->GetName() : TEXT("unknown");

		// Broadcast before destroy so listeners (e.g. spawner respawn) can react.
		OnVehicleDespawned.Broadcast(Controller, Controller->GetCurrentLane());

		Controller->UnPossess();

		if (VehiclePawn && VehiclePool)
		{
			// Pool the pawn instead of destroying it (I1: object pooling).
			VehiclePool->ReleaseVehicle(VehiclePawn);
		}
		else if (VehiclePawn)
		{
			VehiclePawn->Destroy();
		}
		Controller->Destroy();

		UE_LOG(LogAAATraffic, Log,
			TEXT("TrafficSubsystem: Despawned vehicle '%s' — reason: %s."),
			*VehicleName, *DespawnReasons[Idx]);
	}
}

// ---------------------------------------------------------------------------
// Safety Despawn
// ---------------------------------------------------------------------------

void UTrafficSubsystem::RequestDespawn(ATrafficVehicleController* Controller, const FString& Reason)
{
	if (!Controller) return;

	// Capture everything we need before deferring. The controller may be
	// garbage-collected between now and next tick, so use a weak pointer.
	TWeakObjectPtr<ATrafficVehicleController> WeakController(Controller);
	const FString CapturedReason = Reason;

	// Defer actual destruction to next tick to avoid invalidating iterators
	// or destroying objects during another object's Tick.
	GetWorld()->GetTimerManager().SetTimerForNextTick(
		FTimerDelegate::CreateWeakLambda(this, [this, WeakController, CapturedReason]()
		{
			ATrafficVehicleController* Ctrl = WeakController.Get();
			if (!Ctrl) return;

			APawn* VehiclePawn = Ctrl->GetPawn();
			const FString VehicleName = VehiclePawn
				? VehiclePawn->GetName() : TEXT("unknown");

			// Broadcast before destroy so listeners (e.g. spawner) can react.
			OnVehicleDespawned.Broadcast(Ctrl, Ctrl->GetCurrentLane());

			Ctrl->UnPossess();

			if (VehiclePawn && VehiclePool)
			{
				VehiclePool->ReleaseVehicle(VehiclePawn);
			}
			else if (VehiclePawn)
			{
				VehiclePawn->Destroy();
			}
			Ctrl->Destroy();

			UE_LOG(LogAAATraffic, Warning,
				TEXT("TrafficSubsystem: SAFETY despawned '%s' — %s"),
				*VehicleName, *CapturedReason);
		}));
}

// ---------------------------------------------------------------------------
// Spatial Grid
// ---------------------------------------------------------------------------

int64 UTrafficSubsystem::ComputeCellKey(const FVector& Position)
{
	const int32 CX = FMath::FloorToInt32(Position.X / SpatialCellSize);
	const int32 CY = FMath::FloorToInt32(Position.Y / SpatialCellSize);
	return (static_cast<int64>(CX) << 32) | (static_cast<int64>(CY) & 0xFFFFFFFF);
}

void UTrafficSubsystem::UpdateVehiclePosition(ATrafficVehicleController* Controller, const FVector& Position)
{
	if (!Controller) return;

	const int64 NewKey = ComputeCellKey(Position);

	// Remove from old cell if moved.
	if (const int64* OldKey = VehicleCellMap.Find(Controller))
	{
		if (*OldKey == NewKey) return; // Same cell, nothing to do.
		if (TArray<TWeakObjectPtr<ATrafficVehicleController>>* OldCell = SpatialGrid.Find(*OldKey))
		{
			OldCell->Remove(Controller);
		}
	}

	// Insert into new cell.
	SpatialGrid.FindOrAdd(NewKey).Add(Controller);
	VehicleCellMap.Add(Controller, NewKey);
}

TArray<ATrafficVehicleController*> UTrafficSubsystem::GetNearbyVehicles(const FVector& Position, float Radius) const
{
	TArray<ATrafficVehicleController*> Result;
	const float RadiusSq = Radius * Radius;

	// Determine the range of cells to check.
	const int32 CellRange = FMath::CeilToInt32(Radius / SpatialCellSize);
	const int32 CenterX = FMath::FloorToInt32(Position.X / SpatialCellSize);
	const int32 CenterY = FMath::FloorToInt32(Position.Y / SpatialCellSize);

	for (int32 DX = -CellRange; DX <= CellRange; ++DX)
	{
		for (int32 DY = -CellRange; DY <= CellRange; ++DY)
		{
			const int64 Key = (static_cast<int64>(CenterX + DX) << 32) | static_cast<int64>(static_cast<uint32>(CenterY + DY));
			if (const TArray<TWeakObjectPtr<ATrafficVehicleController>>* Cell = SpatialGrid.Find(Key))
			{
				for (const TWeakObjectPtr<ATrafficVehicleController>& Weak : *Cell)
				{
					ATrafficVehicleController* V = Weak.Get();
					if (!V) continue;
					const APawn* P = V->GetPawn();
					if (!P) continue;
					if (FVector::DistSquared(Position, P->GetActorLocation()) <= RadiusSq)
					{
						Result.Add(V);
					}
				}
			}
		}
	}
	return Result;
}

// ---------------------------------------------------------------------------
// LOD
// ---------------------------------------------------------------------------

ETrafficLOD UTrafficSubsystem::GetVehicleLOD(const ATrafficVehicleController* Controller) const
{
	if (const ETrafficLOD* LOD = VehicleLODMap.Find(Controller))
	{
		return *LOD;
	}
	return ETrafficLOD::Full;
}

void UTrafficSubsystem::UpdateLODTiers(const TArray<FVector>& PlayerPositions)
{
	// LOD thresholds (cm) with hysteresis bands to prevent flickering
	// at boundary distances. Promotion (closer) uses inner threshold,
	// demotion (farther) uses outer threshold.
	constexpr float LOD1PromoteSq = 10000.0f * 10000.0f;  // 100m — promote to Full
	constexpr float LOD1DemoteSq  = 11000.0f * 11000.0f;  // 110m — demote from Full
	constexpr float LOD2PromoteSq = 30000.0f * 30000.0f;  // 300m — promote to Reduced
	constexpr float LOD2DemoteSq  = 33000.0f * 33000.0f;  // 330m — demote from Reduced

	for (const TWeakObjectPtr<ATrafficVehicleController>& WeakVehicle : ActiveVehicles)
	{
		ATrafficVehicleController* Controller = WeakVehicle.Get();
		if (!Controller) continue;
		const APawn* VehiclePawn = Controller->GetPawn();
		if (!VehiclePawn) continue;

		float NearestDistSq = MAX_flt;
		const FVector VehicleLoc = VehiclePawn->GetActorLocation();
		for (const FVector& PlayerLoc : PlayerPositions)
		{
			NearestDistSq = FMath::Min(NearestDistSq, FVector::DistSquared(VehicleLoc, PlayerLoc));
		}

		const ETrafficLOD CurrentLOD = GetVehicleLOD(Controller);
		ETrafficLOD Tier = CurrentLOD;

		// Apply hysteresis: use different thresholds depending on direction.
		if (CurrentLOD == ETrafficLOD::Full)
		{
			// Currently Full — only demote if past the outer LOD1 boundary.
			if (NearestDistSq >= LOD2DemoteSq)      { Tier = ETrafficLOD::Minimal; }
			else if (NearestDistSq >= LOD1DemoteSq)  { Tier = ETrafficLOD::Reduced; }
		}
		else if (CurrentLOD == ETrafficLOD::Reduced)
		{
			if (NearestDistSq < LOD1PromoteSq)       { Tier = ETrafficLOD::Full; }
			else if (NearestDistSq >= LOD2DemoteSq)   { Tier = ETrafficLOD::Minimal; }
		}
		else // Minimal
		{
			if (NearestDistSq < LOD1PromoteSq)       { Tier = ETrafficLOD::Full; }
			else if (NearestDistSq < LOD2PromoteSq)   { Tier = ETrafficLOD::Reduced; }
		}

		VehicleLODMap.Add(Controller, Tier);
	}
}

// ---------------------------------------------------------------------------
// Signal Controllers
// ---------------------------------------------------------------------------

void UTrafficSubsystem::RegisterSignalController(int32 JunctionId, ATrafficSignalController* Controller)
{
	if (Controller && JunctionId != 0)
	{
		SignalControllerMap.Add(JunctionId, Controller);
		UE_LOG(LogAAATraffic, Log, TEXT("TrafficSubsystem: Signal controller registered for junction %d."), JunctionId);
	}
}

void UTrafficSubsystem::UnregisterSignalController(int32 JunctionId)
{
	SignalControllerMap.Remove(JunctionId);
}

ATrafficSignalController* UTrafficSubsystem::GetSignalForJunction(int32 JunctionId) const
{
	if (const TWeakObjectPtr<ATrafficSignalController>* Weak = SignalControllerMap.Find(JunctionId))
	{
		return Weak->Get();
	}
	return nullptr;
}

// ---------------------------------------------------------------------------
// Road Speed Limits
// ---------------------------------------------------------------------------

void UTrafficSubsystem::SetRoadSpeedLimit(int32 RoadHandleId, float SpeedLimit)
{
	if (RoadHandleId != 0 && SpeedLimit > 0.0f)
	{
		RoadSpeedLimits.Add(RoadHandleId, SpeedLimit);
	}
}

float UTrafficSubsystem::GetRoadSpeedLimit(int32 RoadHandleId) const
{
	if (const float* Found = RoadSpeedLimits.Find(RoadHandleId))
	{
		return *Found;
	}
	return -1.0f;
}

// ---------------------------------------------------------------------------
// Junction Occupancy — multi-vehicle with geometric conflict detection
// ---------------------------------------------------------------------------

bool UTrafficSubsystem::TryOccupyJunction(int32 JunctionId,
	ATrafficVehicleController* Controller,
	const FTrafficLaneHandle& FromLane, const FTrafficLaneHandle& ToLane)
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

	TArray<FJunctionOccupant>& Occupants = JunctionOccupancy.FindOrAdd(JunctionId);

	// Purge stale entries (GC'd controllers).
	Occupants.RemoveAll([](const FJunctionOccupant& O) { return !O.Controller.IsValid(); });

	// Evict occupants that have held the junction for too long.
	// A normal junction traversal takes a few seconds at most.
	// If an occupant is still here after MaxJunctionOccupancyTimeSec,
	// it is stuck (physics collision, broken path, etc.) and blocking
	// all traffic behind it.  Evicting cleans the occupancy so
	// waiting vehicles can proceed via normal TryOccupy.
	constexpr double MaxJunctionOccupancyTimeSec = 20.0;
	const double NowSec = FPlatformTime::Seconds();
	Occupants.RemoveAll([NowSec, JunctionId](const FJunctionOccupant& O)
	{
		if (O.OccupiedAtTime > 0.0 && (NowSec - O.OccupiedAtTime) > MaxJunctionOccupancyTimeSec)
		{
			const FString StuckName = (O.Controller.IsValid() && O.Controller.Get()->GetPawn())
				? O.Controller.Get()->GetPawn()->GetName() : TEXT("STALE");
			UE_LOG(LogAAATraffic, Warning,
				TEXT("JNCT EVICT-STALE: JunctionId=%d Occupant='%s' held for %.1fs — "
					 "evicting stuck occupant to unblock junction"),
				JunctionId, *StuckName, NowSec - O.OccupiedAtTime);
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

	// Conflict test: check against every existing occupant.
	ITrafficRoadProvider* Provider = GetProvider();
	for (const FJunctionOccupant& Occ : Occupants)
	{
		if (!Occ.Controller.IsValid()) { continue; }

		// If no provider or either occupant has invalid lane info, be conservative.
		bool bConflict = true;
		if (Provider && FromLane.IsValid() && ToLane.IsValid()
			&& Occ.FromLane.IsValid() && Occ.ToLane.IsValid())
		{
			bConflict = Provider->DoJunctionPathsConflict(
				FromLane, ToLane, Occ.FromLane, Occ.ToLane);
		}

		if (bConflict)
		{
			const FString OccName = (Occ.Controller.Get() && Occ.Controller.Get()->GetPawn())
				? Occ.Controller.Get()->GetPawn()->GetName() : TEXT("STALE");
			UE_LOG(LogAAATraffic, Warning,
				TEXT("JNCT TryOccupy: JunctionId=%d Caller='%s' CONFLICTS with '%s' "
					 "(From=%d→To=%d vs From=%d→To=%d) — DENIED"),
				JunctionId, *CallerName, *OccName,
				FromLane.HandleId, ToLane.HandleId,
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
	NewOccupant.FromLane = FromLane;
	NewOccupant.ToLane = ToLane;
	NewOccupant.OccupiedAtTime = FPlatformTime::Seconds();
	Occupants.Add(NewOccupant);

	UE_LOG(LogAAATraffic, Warning,
		TEXT("JNCT TryOccupy: JunctionId=%d Caller='%s' From=%d To=%d — GRANTED (%d occupants now)"),
		JunctionId, *CallerName, FromLane.HandleId, ToLane.HandleId, Occupants.Num());
	return true;
}

void UTrafficSubsystem::ForceOccupyJunction(int32 JunctionId,
	ATrafficVehicleController* Controller,
	const FTrafficLaneHandle& FromLane, const FTrafficLaneHandle& ToLane)
{
	if (JunctionId == 0 || !Controller) { return; }

	const FString CallerName = (Controller && Controller->GetPawn())
		? Controller->GetPawn()->GetName() : TEXT("NULL");

	TArray<FJunctionOccupant>& Occupants = JunctionOccupancy.FindOrAdd(JunctionId);

	// Purge stale entries.
	Occupants.RemoveAll([](const FJunctionOccupant& O) { return !O.Controller.IsValid(); });

	// Self-reentry check — already present, nothing to do.
	for (const FJunctionOccupant& Occ : Occupants)
	{
		if (Occ.Controller.Get() == Controller) { return; }
	}

	FJunctionOccupant NewOccupant;
	NewOccupant.Controller = Controller;
	NewOccupant.FromLane = FromLane;
	NewOccupant.ToLane = ToLane;
	NewOccupant.OccupiedAtTime = FPlatformTime::Seconds();
	Occupants.Add(NewOccupant);

	UE_LOG(LogAAATraffic, Warning,
		TEXT("JNCT ForceOccupy: JunctionId=%d Caller='%s' From=%d To=%d — "
			 "FORCED entry (deadlock break, %d occupants now)"),
		JunctionId, *CallerName, FromLane.HandleId, ToLane.HandleId, Occupants.Num());
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
	const FTrafficLaneHandle& FromLane,
	const FTrafficLaneHandle& ToLane) const
{
	ITrafficRoadProvider* Provider = GetProvider();
	if (!Provider) { return false; }

	for (const TWeakObjectPtr<ATrafficVehicleController>& WeakOther : ActiveVehicles)
	{
		ATrafficVehicleController* Other = WeakOther.Get();
		if (!Other || Other == Self) { continue; }

		// Only consider vehicles interacting with the same junction.
		// JnctState.JunctionId is now set from the approach scan onward
		// (Phase >= Approaching), not just after the detection gate.
		if (Other->JnctState.JunctionId != JunctionId) { continue; }

		// Resolve the other vehicle's path (FromLane/ToLane).
		// Primary: use engaged lanes if available.
		// Fallback: estimate from approach junction lane (fixes blind spot —
		// approach-phase vehicles were invisible in the old code).
		FTrafficLaneHandle OtherFromLane = Other->JnctState.FromLane;
		FTrafficLaneHandle OtherToLane = Other->JnctState.ToLane;
		if (!OtherFromLane.IsValid() || !OtherToLane.IsValid())
		{
			if (Other->JnctState.ApproachJunctionLane.IsValid())
			{
				OtherFromLane = Other->CurrentLane;
				TArray<FTrafficLaneHandle> Exits = Provider->GetConnectedLanes(Other->JnctState.ApproachJunctionLane);
				if (Exits.Num() > 0) { OtherToLane = Exits[0]; }
			}
			if (!OtherFromLane.IsValid() || !OtherToLane.IsValid()) { continue; }
		}

		// Only yield to straight-through (non-turning) vehicles — they have higher priority.
		const FVector OtherApproachDir = Provider->GetLaneDirectionAtDistance(
			OtherFromLane, Provider->GetLaneLength(OtherFromLane));
		const FVector OtherExitDir = Provider->GetLaneDirection(OtherToLane);
		const float OtherCrossZ = FVector::CrossProduct(OtherApproachDir, OtherExitDir).Z;
		if (FMath::Abs(OtherCrossZ) >= 0.26f) { continue; } // Other is also turning — no priority advantage.

		// Check if our intended path would geometrically conflict with theirs.
		if (Provider->DoJunctionPathsConflict(FromLane, ToLane,
				OtherFromLane, OtherToLane))
		{
			UE_LOG(LogAAATraffic, Log,
				TEXT("JNCT LEFT-TURN-YIELD: JunctionId=%d Self='%s' yielding to "
					 "approaching straight-through '%s' (From=%d To=%d Phase=%d)"),
				JunctionId,
				Self && Self->GetPawn() ? *Self->GetPawn()->GetName() : TEXT("NULL"),
				Other->GetPawn() ? *Other->GetPawn()->GetName() : TEXT("NULL"),
				OtherFromLane.HandleId,
				OtherToLane.HandleId,
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
	const FTrafficLaneHandle& FromLane, const FTrafficLaneHandle& ToLane,
	float GapThresholdSec) const
{
	ITrafficRoadProvider* Provider = GetProvider();
	if (!Provider || JunctionId == 0) { return false; }

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

		// Resolve the other vehicle's path.
		FTrafficLaneHandle OtherFromLane = Other->JnctState.FromLane;
		FTrafficLaneHandle OtherToLane = Other->JnctState.ToLane;
		if (!OtherFromLane.IsValid() || !OtherToLane.IsValid())
		{
			if (Other->JnctState.ApproachJunctionLane.IsValid())
			{
				OtherFromLane = Other->CurrentLane;
				TArray<FTrafficLaneHandle> OtherExits = Provider->GetConnectedLanes(Other->JnctState.ApproachJunctionLane);
				if (OtherExits.Num() > 0)
				{
					OtherToLane = OtherExits[0];
				}
				else
				{
					continue;
				}
			}
			else
			{
				continue;
			}
		}

		if (!OtherFromLane.IsValid() || !OtherToLane.IsValid()) { continue; }

		// Check geometric conflict.
		if (!Provider->DoJunctionPathsConflict(FromLane, ToLane, OtherFromLane, OtherToLane))
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
	Entry.ArrivalTime = FPlatformTime::Seconds();
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

	int32 TotalRules = 0;

	for (const int32 JunctionId : JunctionIds)
	{
		TArray<FTrafficLaneHandle> JunctionLanes = Provider->GetLanesForJunction(JunctionId);

		// Gather all non-junction exit lanes reachable from this junction.
		TArray<FTrafficLaneHandle> AllExitLanes;
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
					if (!bDup) { AllExitLanes.Add(C); }
				}
			}
		}

		// Find approach lanes: non-junction lanes that connect INTO this junction.
		// An approach lane is any lane whose GetConnectedLanes output includes a
		// junction lane belonging to this junction.
		TArray<FTrafficLaneHandle> ApproachLanes;
		for (const FTrafficRoadHandle& Road : AllRoads)
		{
			TArray<FTrafficLaneHandle> RoadLanes = Provider->GetLanesForRoad(Road);
			for (const FTrafficLaneHandle& Lane : RoadLanes)
			{
				if (Provider->GetJunctionForLane(Lane) != 0) { continue; } // Skip junction lanes.
				TArray<FTrafficLaneHandle> LaneConnected = Provider->GetConnectedLanes(Lane);
				for (const FTrafficLaneHandle& LC : LaneConnected)
				{
					// Does this connected lane belong to our junction?
					bool bIsJunctionLane = false;
					for (const FTrafficLaneHandle& JL : JunctionLanes)
					{
						if (JL.HandleId == LC.HandleId) { bIsJunctionLane = true; break; }
					}
					if (bIsJunctionLane)
					{
						bool bDup = false;
						for (const FTrafficLaneHandle& A : ApproachLanes)
						{
							if (A.HandleId == Lane.HandleId) { bDup = true; break; }
						}
						if (!bDup) { ApproachLanes.Add(Lane); }
						break;
					}
				}
			}
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

				// --- Filter 4: Physical feasibility (junction path swept width) ---
				bool bPhysicallyFeasible = true;
				{
					TArray<FVector> JunctionPath;
					if (Provider->GetJunctionPath(Approach, Exit, JunctionPath) && JunctionPath.Num() >= 3)
					{
						// Estimate turn radius from the junction path.
						// Sample 3 points: start, mid, end — compute radius of circumscribed circle.
						const FVector& P0 = JunctionPath[0];
						const FVector& P1 = JunctionPath[JunctionPath.Num() / 2];
						const FVector& P2 = JunctionPath.Last();

						const FVector A = P1 - P0;
						const FVector B = P2 - P1;
						const float CrossMag = FMath::Abs(FVector::CrossProduct(A, B).Z);
						if (CrossMag > 1.0f)
						{
							const float ChordLen = FVector::Dist(P0, P2);
							const float TurnRadius = (A.Size() * B.Size() * ChordLen) / (2.0f * CrossMag);

							// Vehicle swept half-width needs clearance.
							// Typical sedan: ~200cm half-width. Use lane width as clearance bound.
							constexpr float VehicleHalfWidth = 200.0f;
							const float LaneWidth = Provider->GetLaneWidthAtDistance(Exit, 0.0f);
							const float MinRadius = VehicleHalfWidth * 2.0f; // Tight turn limit

							if (TurnRadius < MinRadius && LaneWidth > 0.0f && TurnRadius < LaneWidth * 0.5f)
							{
								bPhysicallyFeasible = false;
								UE_LOG(LogAAATraffic, Log,
									TEXT("SURVEY:   Approach=%d Exit=%d INFEASIBLE (TurnRadius=%.0f < MinRadius=%.0f, LaneWidth=%.0f)"),
									Approach.HandleId, Exit.HandleId, TurnRadius, MinRadius, LaneWidth);
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

				UE_LOG(LogAAATraffic, Log,
					TEXT("SURVEY:   Approach=%d Exit=%d Turn=%s Dot=%.3f W=%.3f Feasible=%s"),
					Approach.HandleId, Exit.HandleId,
					TurnDir == ETurnSignalState::Left ? TEXT("L") : (TurnDir == ETurnSignalState::Right ? TEXT("R") : TEXT("S")),
					Dot, Rule.Weight, bPhysicallyFeasible ? TEXT("Y") : TEXT("N"));
			}

			// Safety: if all rules ended up with Weight=0, give them minimal weight
			// so the vehicle has at least some option.
			bool bAllZero = true;
			for (const FJunctionExitRule& R : Rules)
			{
				if (R.Weight > 0.0f) { bAllZero = false; break; }
			}
			if (bAllZero && Rules.Num() > 0)
			{
				for (FJunctionExitRule& R : Rules)
				{
					R.Weight = 0.01f;
				}
				UE_LOG(LogAAATraffic, Warning,
					TEXT("SURVEY:   Approach=%d — all exits had W=0, applied safety minimum."),
					Approach.HandleId);
			}

			TotalRules += Rules.Num();
			JunctionExitRules.Add(Approach.HandleId, MoveTemp(Rules));
		}
	}

	UE_LOG(LogAAATraffic, Log,
		TEXT("SURVEY: Junction survey complete — %d approach lanes, %d total exit rules."),
		JunctionExitRules.Num(), TotalRules);
}
