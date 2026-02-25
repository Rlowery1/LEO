// Copyright AAA_Traffic Contributors. All Rights Reserved.

#include "TrafficSubsystem.h"
#include "TrafficRoadProvider.h"
#include "TrafficVehicleController.h"
#include "TrafficSignalController.h"
#include "TrafficLog.h"
#include "Engine/World.h"
#include "TimerManager.h"
#include "GameFramework/PlayerController.h"
#include "GameFramework/Pawn.h"

DEFINE_LOG_CATEGORY(LogAAATraffic);

UTrafficSubsystem::UTrafficSubsystem()
	: DespawnDistance(50000.f)		// 500m
	, DespawnCheckInterval(1.0f)		// once per second
	, bDespawnDeadEndVehicles(true)
{
}

void UTrafficSubsystem::Initialize(FSubsystemCollectionBase& Collection)
{
	Super::Initialize(Collection);
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
	ActiveVehicles.Remove(InController);

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
		if (It.Value().Get() == InController)
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

	// Collect vehicles to despawn (can't modify set during iteration).
	TArray<ATrafficVehicleController*> ToDespawn;
	TArray<FString> DespawnReasons;

	for (auto It = ActiveVehicles.CreateIterator(); It; ++It)
	{
		ATrafficVehicleController* Controller = It->Get();
		if (!Controller)
		{
			It.RemoveCurrent();
			continue;
		}

		const APawn* VehiclePawn = Controller->GetPawn();
		if (!VehiclePawn)
		{
			It.RemoveCurrent();
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

	// Destroy collected vehicles.
	for (int32 Idx = 0; Idx < ToDespawn.Num(); ++Idx)
	{
		ATrafficVehicleController* Controller = ToDespawn[Idx];
		APawn* VehiclePawn = Controller->GetPawn();
		const FString VehicleName = VehiclePawn ? VehiclePawn->GetName() : TEXT("unknown");

		// Broadcast before destroy so listeners (e.g. spawner respawn) can react.
		OnVehicleDespawned.Broadcast(Controller, Controller->GetCurrentLane());

		Controller->UnPossess();

		if (VehiclePawn)
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
	// LOD thresholds (cm).
	constexpr float LOD1DistSq = 10000.0f * 10000.0f;  // 100m
	constexpr float LOD2DistSq = 30000.0f * 30000.0f;  // 300m

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

		ETrafficLOD Tier;
		if (NearestDistSq < LOD1DistSq)
		{
			Tier = ETrafficLOD::Full;
		}
		else if (NearestDistSq < LOD2DistSq)
		{
			Tier = ETrafficLOD::Reduced;
		}
		else
		{
			Tier = ETrafficLOD::Minimal;
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
// Junction Occupancy
// ---------------------------------------------------------------------------

bool UTrafficSubsystem::TryOccupyJunction(int32 JunctionId, ATrafficVehicleController* Controller)
{
	if (JunctionId == 0 || !Controller) return true; // No junction — always OK.

	TWeakObjectPtr<ATrafficVehicleController>& Occupant = JunctionOccupancy.FindOrAdd(JunctionId);
	ATrafficVehicleController* Current = Occupant.Get();

	if (!Current || Current == Controller)
	{
		Occupant = Controller;
		return true;
	}

	// Junction is occupied by another vehicle — do not override existing occupancy.
	// Once granted, occupancy remains exclusive until the holder releases it.
	return false;
}

void UTrafficSubsystem::ReleaseJunction(int32 JunctionId, ATrafficVehicleController* Controller)
{
	if (TWeakObjectPtr<ATrafficVehicleController>* Occupant = JunctionOccupancy.Find(JunctionId))
	{
		if (Occupant->Get() == Controller)
		{
			JunctionOccupancy.Remove(JunctionId);
		}
	}
}
