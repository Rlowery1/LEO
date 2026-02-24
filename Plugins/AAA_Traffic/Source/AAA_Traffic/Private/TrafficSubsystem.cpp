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

void UTrafficSubsystem::Initialize(FSubsystemCollectionBase& Collection)
{
	Super::Initialize(Collection);

	// Set defaults.
	DespawnDistance = 50000.f;		// 500m
	DespawnCheckInterval = 1.0f;		// once per second
	bDespawnDeadEndVehicles = true;
}

void UTrafficSubsystem::Deinitialize()
{
	if (UWorld* World = GetWorld())
	{
		World->GetTimerManager().ClearTimer(DespawnTimerHandle);
	}
	ActiveVehicles.Empty();
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

	// Stop the timer when no vehicles remain.
	if (ActiveVehicles.Num() == 0)
	{
		if (UWorld* World = GetWorld())
		{
			World->GetTimerManager().ClearTimer(DespawnTimerHandle);
		}
	}
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

	// If no players, skip — don't despawn everything.
	if (PlayerPositions.IsEmpty()) return;

	const float DespawnDistSq = DespawnDistance * DespawnDistance;

	// Collect vehicles to despawn (can't modify set during iteration).
	TArray<ATrafficVehicleController*> ToDespawn;

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
				continue;
			}
		}

		// Check 2: distance from nearest player.
		const FVector VehicleLoc = VehiclePawn->GetActorLocation();
		bool bInRange = false;
		for (const FVector& PlayerLoc : PlayerPositions)
		{
			if (FVector::DistSquared(VehicleLoc, PlayerLoc) < DespawnDistSq)
			{
				bInRange = true;
				break;
			}
		}

		if (!bInRange)
		{
			ToDespawn.Add(Controller);
		}
	}

	// Destroy collected vehicles.
	for (ATrafficVehicleController* Controller : ToDespawn)
	{
		APawn* VehiclePawn = Controller->GetPawn();
		Controller->UnPossess();

		if (VehiclePawn)
		{
			VehiclePawn->Destroy();
		}
		Controller->Destroy();

		UE_LOG(LogAAATraffic, Log, TEXT("TrafficSubsystem: Despawned vehicle."));
	}
}
