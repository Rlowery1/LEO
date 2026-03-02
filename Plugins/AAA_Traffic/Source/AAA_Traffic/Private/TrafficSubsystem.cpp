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

	// No conflicts — grant entry.
	FJunctionOccupant NewOccupant;
	NewOccupant.Controller = Controller;
	NewOccupant.FromLane = FromLane;
	NewOccupant.ToLane = ToLane;
	Occupants.Add(NewOccupant);

	UE_LOG(LogAAATraffic, Warning,
		TEXT("JNCT TryOccupy: JunctionId=%d Caller='%s' From=%d To=%d — GRANTED (%d occupants now)"),
		JunctionId, *CallerName, FromLane.HandleId, ToLane.HandleId, Occupants.Num());
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
