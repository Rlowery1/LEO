// Copyright AAA_Traffic Contributors. All Rights Reserved.

#include "TrafficSubsystem.h"
#include "TrafficRoadProvider.h"

void UTrafficSubsystem::RegisterProvider(UObject* InProvider)
{
	if (!InProvider)
	{
		return;
	}

	if (!InProvider->GetClass()->ImplementsInterface(UTrafficRoadProvider::StaticClass()))
	{
		UE_LOG(LogTemp, Warning,
			TEXT("TrafficSubsystem: Object '%s' does not implement ITrafficRoadProvider."),
			*InProvider->GetName());
		return;
	}

	if (ActiveProviderObject && ActiveProviderObject != InProvider)
	{
		UE_LOG(LogTemp, Warning,
			TEXT("TrafficSubsystem: Replacing existing provider '%s' with '%s'."),
			*ActiveProviderObject->GetName(), *InProvider->GetName());
	}

	ActiveProviderObject = InProvider;
	UE_LOG(LogTemp, Log, TEXT("TrafficSubsystem: Provider registered — %s"), *InProvider->GetName());
}

void UTrafficSubsystem::UnregisterProvider(UObject* InProvider)
{
	if (ActiveProviderObject == InProvider)
	{
		ActiveProviderObject = nullptr;
		UE_LOG(LogTemp, Log, TEXT("TrafficSubsystem: Provider unregistered."));
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
