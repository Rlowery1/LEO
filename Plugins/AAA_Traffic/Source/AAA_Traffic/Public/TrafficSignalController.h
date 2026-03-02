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
 * How a junction is controlled.
 * Selected per-controller via the ControlMode dropdown in the Details panel.
 * Auto-placed signals get a heuristic default based on approach road count
 * (4+ = Signal, 3 = StopSign, 2 = Yield).
 */
UENUM(BlueprintType)
enum class EJunctionControlMode : uint8
{
	/** Full traffic light cycling (Green → Yellow → Red per phase group). */
	Signal,
	/** All-way stop: vehicles must come to a full stop, wait, then proceed when clear. */
	StopSign,
	/** Yield: vehicles slow down and proceed without full stop if the junction is clear. */
	Yield,
	/** Flashing red on all approaches — treated as an all-way stop. */
	FlashingRed
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
	 * When ControlMode is Signal: checks phase groups or legacy phase.
	 * When ControlMode is StopSign/FlashingRed: always returns false (vehicles must stop).
	 * When ControlMode is Yield: always returns false (vehicles must slow/check occupancy).
	 */
	UFUNCTION(BlueprintCallable, Category = "Traffic|Signal")
	bool IsLaneGreen(const FTrafficLaneHandle& Lane) const;

	/**
	 * Get the control mode for this junction controller.
	 * Vehicles use this to decide stop-sign vs yield vs signal behavior.
	 */
	UFUNCTION(BlueprintPure, Category = "Traffic|Signal")
	EJunctionControlMode GetControlMode() const { return ControlMode; }

	// --- Configuration ---

	/**
	 * How this junction is controlled. Determines vehicle behavior on approach:
	 * - Signal: full green/yellow/red cycling
	 * - StopSign: full stop required, wait StopSignWaitTimeSec, then proceed when clear
	 * - Yield: slow approach, proceed without stopping if clear
	 * - FlashingRed: treated as all-way stop
	 *
	 * Auto-placed controllers get a heuristic default. Change in the Details panel
	 * to override any junction.
	 */
	UPROPERTY(EditAnywhere, BlueprintReadWrite, Category = "Traffic|Signal")
	EJunctionControlMode ControlMode;

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

	/**
	 * Time (seconds) vehicles must wait at a full stop before proceeding
	 * (StopSign and FlashingRed control modes only). Default 2.0s.
	 */
	UPROPERTY(EditAnywhere, BlueprintReadWrite, Category = "Traffic|Signal", meta = (ClampMin = "0.5", EditCondition = "ControlMode == EJunctionControlMode::StopSign || ControlMode == EJunctionControlMode::FlashingRed"))
	float StopSignWaitTimeSec;

	/** When true (non-Shipping builds), draws a sphere at this signal's location colored by the current phase. Has no effect in Shipping. */
	UPROPERTY(EditAnywhere, Category = "Traffic|Debug")
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
