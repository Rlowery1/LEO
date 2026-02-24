// Copyright AAA_Traffic Contributors. All Rights Reserved.

#pragma once

#include "CoreMinimal.h"
#include "AIController.h"
#include "TrafficRoadProvider.h"
#include "TrafficVehicleController.generated.h"

/**
 * AI controller that drives a Chaos vehicle along a lane
 * obtained from an ITrafficRoadProvider adapter.
 *
 * Uses a pure-pursuit steering model: picks a look-ahead point on the
 * lane centerline and feeds throttle / steering / brake to the vehicle
 * movement component.
 */
UCLASS()
class AAA_TRAFFIC_API ATrafficVehicleController : public AAIController
{
	GENERATED_BODY()

public:
	ATrafficVehicleController();

	/**
	 * Tell this controller which lane to follow.
	 * Fetches the lane path from the active road provider and begins driving.
	 */
	void InitializeLaneFollowing(const FTrafficLaneHandle& InLane);

	/** Set the target speed for this controller (cm/s). */
	void SetTargetSpeed(float InSpeed);

	/** Set the random seed before possession (determines RandomStream). */
	void SetRandomSeed(int32 InSeed);

protected:
	virtual void OnPossess(APawn* InPawn) override;
	virtual void Tick(float DeltaSeconds) override;

private:
	/** Feed throttle / steering / brake into the vehicle movement component. */
	void UpdateVehicleInput(float DeltaSeconds);

	/** Find the index of the closest lane point to the given world position. */
	int32 FindClosestPointIndex(const FVector& VehicleLocation) const;

	/** Walk forward along lane points by LookAheadDistance and return the target point. */
	FVector GetLookAheadPoint(const FVector& VehicleLocation, int32 ClosestIndex) const;

	// ----- State -----

	/** Handle to the lane currently being followed. */
	FTrafficLaneHandle CurrentLane;

	/** Cached world-space centerline points of the current lane. */
	TArray<FVector> LanePoints;

	/** Cached lane width in cm. */
	float LaneWidth;

	/** Set to true once lane data has been fetched from the provider. */
	bool bLaneDataReady;

	// ----- Tuning -----

	/** Target speed in cm/s (default ~54 km/h). */
	UPROPERTY(EditAnywhere, Category = "Traffic", meta = (ClampMin = "0"))
	float TargetSpeed;

	/** Distance ahead on the lane path (cm) used as the steering target. */
	UPROPERTY(EditAnywhere, Category = "Traffic", meta = (ClampMin = "100"))
	float LookAheadDistance;

	/**
	 * Seed for deterministic random decisions.
	 * Reserved for future use (e.g. lane choice at intersections).
	 */
	UPROPERTY(EditAnywhere, Category = "Traffic")
	int32 RandomSeed;

	/**
	 * Deterministic random stream (seeded from RandomSeed).
	 * Reserved for future lane-choice / gap-acceptance randomization.
	 */
	FRandomStream RandomStream;
};
