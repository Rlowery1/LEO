// Copyright AAA_Traffic Contributors. All Rights Reserved.

#pragma once

#include "CoreMinimal.h"
#include "Subsystems/WorldSubsystem.h"
#include "TrafficSubsystem.generated.h"

class ITrafficRoadProvider;

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

private:
	/** The UObject that implements ITrafficRoadProvider. Kept as UObject* to prevent GC. */
	UPROPERTY()
	TObjectPtr<UObject> ActiveProviderObject;
};
