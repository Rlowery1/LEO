// Copyright AAA_Traffic Contributors. All Rights Reserved.

#pragma once

#include "CoreMinimal.h"
#include "Subsystems/WorldSubsystem.h"
#include "TrafficSubsystem.generated.h"

class ITrafficRoadProvider;
class ATrafficVehicleController;

/**
 * World subsystem that holds a reference to the active road provider.
 * Road-kit adapters register themselves here; the traffic system queries through here.
 */
UCLASS()
class AAA_TRAFFIC_API UTrafficSubsystem : public UWorldSubsystem
{
	GENERATED_BODY()

public:
	/** Register a road provider (adapter). Only one active provider is supported at a time. */
	void RegisterProvider(UObject* InProvider);

	/** Unregister the current provider. */
	void UnregisterProvider(UObject* InProvider);

	/** Get the currently active road provider, or nullptr if none is registered. */
	ITrafficRoadProvider* GetProvider() const;

	/** Register an active traffic vehicle controller. */
	void RegisterVehicle(ATrafficVehicleController* InController);

	/** Unregister a traffic vehicle controller. */
	void UnregisterVehicle(ATrafficVehicleController* InController);

	/** Get the set of all currently active vehicle controllers. */
	const TSet<TWeakObjectPtr<ATrafficVehicleController>>& GetActiveVehicles() const { return ActiveVehicles; }

private:
	/** The UObject that implements ITrafficRoadProvider. Kept as UObject* to prevent GC. */
	UPROPERTY()
	TObjectPtr<UObject> ActiveProviderObject;

	/**
	 * Registry of all active traffic vehicle controllers in the world.
	 *
	 * NOTE:
	 * - The proximity detection system currently relies on direct physics sweeps and does not
	 *   query this registry for neighbor detection.
	 * - This registry exists to support future despawn / lifecycle management and potential
	 *   debugging / analytics features.
	 *
	 * Per System.md Section 8 (minimalism), this is treated as bounded technical debt:
	 * - Keep registration / unregistration overhead minimal and free of heavy per-tick work.
	 * - If this set becomes a performance hotspot or remains unused by concrete features,
	 *   either wire it into those features or remove it instead of growing its responsibilities.
	 *
	 * Not marked UPROPERTY: TSet<TWeakObjectPtr<T>> is not supported by UHT reflection.
	 * GC safety is already guaranteed by TWeakObjectPtr, which nullifies automatically
	 * when the referenced UObject is collected.
	 */
	TSet<TWeakObjectPtr<ATrafficVehicleController>> ActiveVehicles;
};
