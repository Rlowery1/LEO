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

extern int32 GTrafficJunctionDiagnostics;

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

	UWorld* World = GetWorld();
	if (!World) return;

	// Capture everything we need before deferring. The controller may be
	// garbage-collected between now and next tick, so use a weak pointer.
	TWeakObjectPtr<ATrafficVehicleController> WeakController(Controller);
	const FString CapturedReason = Reason;

	// Defer actual destruction to next tick to avoid invalidating iterators
	// or destroying objects during another object's Tick.
	World->GetTimerManager().SetTimerForNextTick(
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

	// §4.4 Determinism: sort by stable key so callers never depend on
	// spatial-grid traversal order.  DeterministicSpawnIndex is unique
	// per vehicle; fall back to pointer value as a final tie-break.
	Result.Sort([](ATrafficVehicleController& A, ATrafficVehicleController& B)
	{
		if (A.DeterministicSpawnIndex != B.DeterministicSpawnIndex)
		{
			return A.DeterministicSpawnIndex < B.DeterministicSpawnIndex;
		}
		return &A < &B;
	});

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

