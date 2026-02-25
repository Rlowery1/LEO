// Copyright AAA_Traffic Contributors. All Rights Reserved.

#include "TrafficVehiclePool.h"
#include "TrafficLog.h"
#include "GameFramework/Pawn.h"
#include "Components/PrimitiveComponent.h"
#include "Engine/World.h"

APawn* UTrafficVehiclePool::AcquireVehicle(UWorld* World, TSubclassOf<APawn> VehicleClass, const FTransform& SpawnTransform)
{
	if (!VehicleClass || !World) { return nullptr; }

	TArray<TWeakObjectPtr<APawn>>* ClassPool = Pool.Find(VehicleClass.Get());
	if (!ClassPool || ClassPool->Num() == 0) { return nullptr; }

	// Pop from back (LIFO — last released, first reused).
	APawn* Pawn = nullptr;
	while (ClassPool->Num() > 0)
	{
		TWeakObjectPtr<APawn> Candidate = ClassPool->Pop(EAllowShrinking::No);
		if (Candidate.IsValid())
		{
			Pawn = Candidate.Get();
			break;
		}
		// Stale pointer — skip.
	}

	if (!Pawn) { return nullptr; }

	// Reactivate the pawn.
	Pawn->SetActorTransform(SpawnTransform, /*bSweep=*/ false, /*OutSweepHitResult=*/ nullptr, ETeleportType::TeleportPhysics);
	Pawn->SetActorHiddenInGame(false);
	Pawn->SetActorEnableCollision(true);
	Pawn->SetActorTickEnabled(true);

	// Re-enable component ticking.
	TArray<UActorComponent*> Components;
	Pawn->GetComponents(Components);
	for (UActorComponent* Comp : Components)
	{
		Comp->SetComponentTickEnabled(true);
		if (UPrimitiveComponent* Prim = Cast<UPrimitiveComponent>(Comp))
		{
			Prim->SetSimulatePhysics(true);
		}
	}

	UE_LOG(LogAAATraffic, Verbose,
		TEXT("TrafficVehiclePool: Acquired pooled %s."), *VehicleClass->GetName());

	return Pawn;
}

void UTrafficVehiclePool::ReleaseVehicle(APawn* Vehicle)
{
	if (!Vehicle || !IsValid(Vehicle)) { return; }

	// Detach from any controller.
	if (AController* Controller = Vehicle->GetController())
	{
		Controller->UnPossess();
	}

	// Deactivate the pawn.
	Vehicle->SetActorHiddenInGame(true);
	Vehicle->SetActorEnableCollision(false);
	Vehicle->SetActorTickEnabled(false);

	// Disable component ticking and physics.
	TArray<UActorComponent*> Components;
	Vehicle->GetComponents(Components);
	for (UActorComponent* Comp : Components)
	{
		Comp->SetComponentTickEnabled(false);
		if (UPrimitiveComponent* Prim = Cast<UPrimitiveComponent>(Comp))
		{
			Prim->SetSimulatePhysics(false);
		}
	}

	// Move to a far-away location to prevent any residual interactions.
	Vehicle->SetActorLocation(FVector(0.0, 0.0, -100000.0), /*bSweep=*/ false, /*OutSweepHitResult=*/ nullptr, ETeleportType::TeleportPhysics);

	// Add to pool.
	Pool.FindOrAdd(Vehicle->GetClass()).Add(Vehicle);

	UE_LOG(LogAAATraffic, Verbose,
		TEXT("TrafficVehiclePool: Released %s to pool."), *Vehicle->GetClass()->GetName());
}

int32 UTrafficVehiclePool::GetPooledCount() const
{
	int32 Count = 0;
	for (const auto& Pair : Pool)
	{
		for (const TWeakObjectPtr<APawn>& Weak : Pair.Value)
		{
			if (Weak.IsValid()) { ++Count; }
		}
	}
	return Count;
}

void UTrafficVehiclePool::DrainPool()
{
	for (auto& Pair : Pool)
	{
		for (const TWeakObjectPtr<APawn>& Weak : Pair.Value)
		{
			if (APawn* Pawn = Weak.Get())
			{
				Pawn->Destroy();
			}
		}
	}
	Pool.Empty();

	UE_LOG(LogAAATraffic, Log, TEXT("TrafficVehiclePool: Pool drained."));
}
