// Copyright AAA_Traffic Contributors. All Rights Reserved.

#include "TrafficSignalController.h"
#include "TrafficSubsystem.h"
#include "TrafficLog.h"
#include "Engine/World.h"
#include "DrawDebugHelpers.h"

// Global debug draw CVar — declared in TrafficVehicleController.cpp.
extern int32 GTrafficDebugDraw;

ATrafficSignalController::ATrafficSignalController()
	: JunctionId(0)
	, GreenDuration(15.0f)
	, YellowDuration(3.0f)
	, RedDuration(15.0f)
	, PhaseOffset(0.0f)
	, CurrentPhase(ETrafficSignalPhase::Red)
	, PhaseTimer(0.0f)
	, CurrentGroupIndex(0)
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

#if ENABLE_DRAW_DEBUG
	if (bDebugDrawSignal || GTrafficDebugDraw != 0)
	{
		FColor PhaseColor = FColor::Red;
		if (CurrentPhase == ETrafficSignalPhase::Green) { PhaseColor = FColor::Green; }
		else if (CurrentPhase == ETrafficSignalPhase::Yellow) { PhaseColor = FColor::Yellow; }

		DrawDebugSphere(GetWorld(), GetActorLocation() + FVector(0, 0, 200), 80.0f, 8, PhaseColor, false, -1.0f, 0, 3.0f);

		// Draw the junction ID as text above the sphere.
		DrawDebugString(GetWorld(), GetActorLocation() + FVector(0, 0, 350),
			FString::Printf(TEXT("J%d %s %.1fs"), JunctionId,
				CurrentPhase == ETrafficSignalPhase::Green ? TEXT("G") :
				CurrentPhase == ETrafficSignalPhase::Yellow ? TEXT("Y") : TEXT("R"),
				PhaseTimer),
			nullptr, PhaseColor, 0.0f, false);
	}
#endif
}

void ATrafficSignalController::AdvancePhase()
{
	// Additive timer: preserves negative overflow from PhaseTimer so that
	// large DeltaSeconds values don't cause the signal to drift.
	//
	// Multi-phase mode: when PhaseGroups has entries, the cycle is:
	//   Group[0] Green → Yellow → Red(gap) → Group[1] Green → Yellow → Red(gap) → ...
	// Legacy mode (empty PhaseGroups): simple Green → Yellow → Red.

	switch (CurrentPhase)
	{
	case ETrafficSignalPhase::Green:
		CurrentPhase = ETrafficSignalPhase::Yellow;
		PhaseTimer += YellowDuration;
		break;

	case ETrafficSignalPhase::Yellow:
		CurrentPhase = ETrafficSignalPhase::Red;
		// Multi-group signals: use a brief all-red clearance interval (2s)
		// instead of the full RedDuration. The "red" for one group IS the
		// green for the next group. Legacy (single-group) keeps full RedDuration.
		PhaseTimer += (PhaseGroups.Num() > 1)
			? FMath::Min(RedDuration, 2.0f)
			: RedDuration;
		break;

	case ETrafficSignalPhase::Red:
		// Advance to next group (wraps around).
		if (PhaseGroups.Num() > 0)
		{
			CurrentGroupIndex = (CurrentGroupIndex + 1) % PhaseGroups.Num();
		}
		CurrentPhase = ETrafficSignalPhase::Green;
		PhaseTimer += GreenDuration;
		break;
	}

	// Log every phase change.
	UE_LOG(LogAAATraffic, Warning,
		TEXT("SIGNAL PHASE-CHANGE: Signal='%s' JunctionId=%d NewPhase=%s "
			 "GroupIndex=%d/%d Timer=%.2f"),
		*GetName(), JunctionId,
		CurrentPhase == ETrafficSignalPhase::Green ? TEXT("GREEN") :
		CurrentPhase == ETrafficSignalPhase::Yellow ? TEXT("YELLOW") : TEXT("RED"),
		CurrentGroupIndex, PhaseGroups.Num(), PhaseTimer);
}

bool ATrafficSignalController::IsLaneGreen(const FTrafficLaneHandle& Lane) const
{
	// Throttled logging: log every 120th call to avoid log spam.
	static int32 IsLaneGreenLogCounter = 0;
	const bool bShouldLog = ((++IsLaneGreenLogCounter) % 120) == 0;

	// Legacy mode: no phase groups configured — all lanes share the same phase.
	if (PhaseGroups.Num() == 0)
	{
		const bool bResult = (CurrentPhase == ETrafficSignalPhase::Green);
		if (bShouldLog)
		{
			UE_LOG(LogAAATraffic, Warning,
				TEXT("SIGNAL IsLaneGreen: Signal='%s' Lane=%d Phase=%s Result=%s (legacy mode, no groups)"),
				*GetName(), Lane.HandleId,
				CurrentPhase == ETrafficSignalPhase::Green ? TEXT("GREEN") :
				CurrentPhase == ETrafficSignalPhase::Yellow ? TEXT("YELLOW") : TEXT("RED"),
				bResult ? TEXT("GREEN") : TEXT("RED"));
		}
		return bResult;
	}

	// Multi-phase: only lanes in the currently-green group get green.
	if (CurrentPhase != ETrafficSignalPhase::Green)
	{
		if (bShouldLog)
		{
			UE_LOG(LogAAATraffic, Warning,
				TEXT("SIGNAL IsLaneGreen: Signal='%s' Lane=%d Phase=%s — NOT green (phase not Green)"),
				*GetName(), Lane.HandleId,
				CurrentPhase == ETrafficSignalPhase::Yellow ? TEXT("YELLOW") : TEXT("RED"));
		}
		return false;
	}

	if (!PhaseGroups.IsValidIndex(CurrentGroupIndex))
	{
		if (bShouldLog)
		{
			UE_LOG(LogAAATraffic, Warning,
				TEXT("SIGNAL IsLaneGreen: Signal='%s' Lane=%d GroupIndex=%d INVALID — returning false"),
				*GetName(), Lane.HandleId, CurrentGroupIndex);
		}
		return false;
	}

	const FSignalPhaseGroup& ActiveGroup = PhaseGroups[CurrentGroupIndex];
	for (const FTrafficLaneHandle& GreenLane : ActiveGroup.GreenLanes)
	{
		if (GreenLane == Lane)
		{
			if (bShouldLog)
			{
				UE_LOG(LogAAATraffic, Warning,
					TEXT("SIGNAL IsLaneGreen: Signal='%s' Lane=%d GroupIndex=%d — MATCH found, returning GREEN"),
					*GetName(), Lane.HandleId, CurrentGroupIndex);
			}
			return true;
		}
	}

	if (bShouldLog)
	{
		UE_LOG(LogAAATraffic, Warning,
			TEXT("SIGNAL IsLaneGreen: Signal='%s' Lane=%d GroupIndex=%d GreenLaneCount=%d — NOT in green group"),
			*GetName(), Lane.HandleId, CurrentGroupIndex, ActiveGroup.GreenLanes.Num());
	}
	return false;
}
