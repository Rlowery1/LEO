// Copyright AAA_Traffic Contributors. All Rights Reserved.

#pragma once

#include "CoreMinimal.h"
#include "GameFramework/Actor.h"
#include "TrafficSignalController.generated.h"

class UTrafficSubsystem;

/**
 * Phase of a traffic signal (traffic light).
 */
UENUM(BlueprintType)
enum class ETrafficSignalPhase : uint8
{
	/** Vehicles may proceed. */
	Green,
	/** Signal about to turn red — vehicles should stop if safe. */
	Yellow,
	/** Vehicles must stop. */
	Red
};

/**
 * Actor that controls a traffic signal at a junction.
 *
 * Place in the world, assign a JunctionId that matches the junction
 * topology from the road provider, and the signal will automatically
 * cycle through Green → Yellow → Red phases.
 *
 * Traffic vehicles approaching this junction query the signal phase
 * via the subsystem and stop on Red/Yellow.
 *
 * Determinism: phase progression is time-based using DeltaSeconds
 * (no randomness). All vehicles querying the same signal at the same
 * sim-time will see the same phase.
 */
UCLASS(Blueprintable, ClassGroup = "Traffic")
class AAA_TRAFFIC_API ATrafficSignalController : public AActor
{
	GENERATED_BODY()

public:
	ATrafficSignalController();

	virtual void BeginPlay() override;
	virtual void EndPlay(const EEndPlayReason::Type EndPlayReason) override;
	virtual void Tick(float DeltaSeconds) override;

	/** Get the current signal phase. */
	ETrafficSignalPhase GetCurrentPhase() const { return CurrentPhase; }

	/** Get the junction ID this signal controls. */
	int32 GetJunctionId() const { return JunctionId; }

	// --- Configuration ---

	/** The junction identifier this signal controls (must match the road provider's junction IDs). */
	UPROPERTY(EditAnywhere, BlueprintReadWrite, Category = "Traffic|Signal")
	int32 JunctionId;

	/** Duration of the Green phase (seconds). */
	UPROPERTY(EditAnywhere, BlueprintReadWrite, Category = "Traffic|Signal", meta = (ClampMin = "1"))
	float GreenDuration;

	/** Duration of the Yellow phase (seconds). */
	UPROPERTY(EditAnywhere, BlueprintReadWrite, Category = "Traffic|Signal", meta = (ClampMin = "0.5"))
	float YellowDuration;

	/** Duration of the Red phase (seconds). */
	UPROPERTY(EditAnywhere, BlueprintReadWrite, Category = "Traffic|Signal", meta = (ClampMin = "1"))
	float RedDuration;

	/** Initial phase offset (seconds) — stagger signals at nearby junctions. */
	UPROPERTY(EditAnywhere, BlueprintReadWrite, Category = "Traffic|Signal", meta = (ClampMin = "0"))
	float PhaseOffset;

private:
	/** Current phase of the signal. */
	ETrafficSignalPhase CurrentPhase;

	/** Time remaining in the current phase (seconds). */
	float PhaseTimer;

	/** Advance to the next phase in the cycle. */
	void AdvancePhase();
};
