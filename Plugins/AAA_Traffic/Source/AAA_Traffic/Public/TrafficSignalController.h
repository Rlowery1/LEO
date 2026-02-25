// Copyright AAA_Traffic Contributors. All Rights Reserved.

#pragma once

#include "CoreMinimal.h"
#include "GameFramework/Actor.h"
#include "TrafficRoadProvider.h"
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
 * A group of lanes that share a green phase in a multi-phase signal.
 * For a simple 4-way intersection, you might have 2 groups: one for N/S lanes
 * and one for E/W lanes.
 *
 * When PhaseGroups is empty on the signal controller, the signal falls back to
 * the legacy single-phase mode where all lanes get green simultaneously.
 */
USTRUCT(BlueprintType)
struct FSignalPhaseGroup
{
	GENERATED_BODY()

	/** Human-readable label for this phase group (e.g. "North-South"). */
	UPROPERTY(EditAnywhere, BlueprintReadWrite, Category = "Traffic|Signal")
	FString GroupName;

	/** Lanes that receive green during this group's phase. */
	UPROPERTY(EditAnywhere, BlueprintReadWrite, Category = "Traffic|Signal")
	TArray<FTrafficLaneHandle> GreenLanes;
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
	UFUNCTION(BlueprintCallable, Category = "Traffic|Signal")
	ETrafficSignalPhase GetCurrentPhase() const { return CurrentPhase; }

	/** Get the junction ID this signal controls. */
	UFUNCTION(BlueprintPure, Category = "Traffic|Signal")
	int32 GetJunctionId() const { return JunctionId; }

	/**
	 * Check if a given lane has a green signal right now.
	 * When PhaseGroups are configured, checks if the lane is in the current green group.
	 * When PhaseGroups is empty (legacy mode), returns true if the whole signal is green.
	 */
	UFUNCTION(BlueprintCallable, Category = "Traffic|Signal")
	bool IsLaneGreen(const FTrafficLaneHandle& Lane) const;

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

	/**
	 * Optional multi-phase groups. Each group defines a set of lanes that get green
	 * simultaneously. The signal cycles through groups in order:
	 *   Group0 Green → Yellow → Red(all-red gap) → Group1 Green → Yellow → ...
	 *
	 * When empty, the signal uses the legacy single-phase mode (all lanes green/red together).
	 */
	UPROPERTY(EditAnywhere, BlueprintReadWrite, Category = "Traffic|Signal")
	TArray<FSignalPhaseGroup> PhaseGroups;

	/** When true (non-Shipping builds), draws a sphere at this signal's location colored by the current phase. Has no effect in Shipping. */
	UPROPERTY(EditAnywhere, BlueprintReadWrite, Category = "Traffic|Debug")
	bool bDebugDrawSignal = false;

private:
	/** Current phase of the signal. */
	ETrafficSignalPhase CurrentPhase;

	/** Time remaining in the current phase (seconds). */
	float PhaseTimer;

	/** Index into PhaseGroups for the currently-green group (multi-phase mode). */
	int32 CurrentGroupIndex;

	/** Advance to the next phase in the cycle. */
	void AdvancePhase();
};
