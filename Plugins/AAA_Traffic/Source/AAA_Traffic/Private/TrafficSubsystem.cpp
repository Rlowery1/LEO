// Copyright AAA_Traffic Contributors. All Rights Reserved.

#include "TrafficSubsystem.h"
#include "TrafficRoadProvider.h"
#include "TrafficVehicleController.h"
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
