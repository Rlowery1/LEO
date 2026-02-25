// Copyright AAA_Traffic Contributors. All Rights Reserved.

#include "TrafficSignalController.h"
#include "TrafficSubsystem.h"
#include "TrafficLog.h"
#include "Engine/World.h"

ATrafficSignalController::ATrafficSignalController()
	: JunctionId(0)
	, GreenDuration(15.0f)
	, YellowDuration(3.0f)
	, RedDuration(15.0f)
	, PhaseOffset(0.0f)
	, CurrentPhase(ETrafficSignalPhase::Red)
	, PhaseTimer(0.0f)
{
	PrimaryActorTick.bCanEverTick = true;
	PrimaryActorTick.TickInterval = 0.0f; // Every frame for responsive signal changes.
}

void ATrafficSignalController::BeginPlay()
{
	Super::BeginPlay();

	if (JunctionId == 0)
	{
		UE_LOG(LogAAATraffic, Warning,
			TEXT("TrafficSignalController '%s': JunctionId is 0 — signal will not control any junction."),
			*GetName());
		SetActorTickEnabled(false);
		return;
	}

	// Register with the traffic subsystem.
	if (UWorld* World = GetWorld())
	{
		if (UTrafficSubsystem* TrafficSub = World->GetSubsystem<UTrafficSubsystem>())
		{
			TrafficSub->RegisterSignalController(JunctionId, this);
		}
	}

	// Apply phase offset: advance through complete cycles to determine starting phase.
	// This allows staggering signals at nearby junctions.
	float RemainingOffset = FMath::Max(PhaseOffset, 0.0f);
	CurrentPhase = ETrafficSignalPhase::Green;
	PhaseTimer = GreenDuration;

	while (RemainingOffset > 0.0f)
	{
		if (RemainingOffset >= PhaseTimer)
		{
			RemainingOffset -= PhaseTimer;
			PhaseTimer = 0.0f; // Reset before advancing so additive AdvancePhase works correctly.
			AdvancePhase();
		}
		else
		{
			PhaseTimer -= RemainingOffset;
			RemainingOffset = 0.0f;
		}
	}

	UE_LOG(LogAAATraffic, Log,
		TEXT("TrafficSignalController '%s': Registered for junction %d — starting phase: %s (%.1fs remaining)."),
		*GetName(), JunctionId,
		CurrentPhase == ETrafficSignalPhase::Green ? TEXT("Green") :
		CurrentPhase == ETrafficSignalPhase::Yellow ? TEXT("Yellow") : TEXT("Red"),
		PhaseTimer);
}

void ATrafficSignalController::EndPlay(const EEndPlayReason::Type EndPlayReason)
{
	if (JunctionId != 0)
	{
		if (UWorld* World = GetWorld())
		{
			if (UTrafficSubsystem* TrafficSub = World->GetSubsystem<UTrafficSubsystem>())
			{
				TrafficSub->UnregisterSignalController(JunctionId);
			}
		}
	}

	Super::EndPlay(EndPlayReason);
}

void ATrafficSignalController::Tick(float DeltaSeconds)
{
	Super::Tick(DeltaSeconds);

	PhaseTimer -= DeltaSeconds;

	while (PhaseTimer <= 0.0f)
	{
		AdvancePhase();
	}
}

void ATrafficSignalController::AdvancePhase()
{
	// Additive timer: preserves negative overflow from PhaseTimer so that
	// large DeltaSeconds values don't cause the signal to drift.
	switch (CurrentPhase)
	{
	case ETrafficSignalPhase::Green:
		CurrentPhase = ETrafficSignalPhase::Yellow;
		PhaseTimer += YellowDuration;
		break;

	case ETrafficSignalPhase::Yellow:
		CurrentPhase = ETrafficSignalPhase::Red;
		PhaseTimer += RedDuration;
		break;

	case ETrafficSignalPhase::Red:
		CurrentPhase = ETrafficSignalPhase::Green;
		PhaseTimer += GreenDuration;
		break;
	}
}
