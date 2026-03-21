// Copyright AAA_Traffic Contributors. All Rights Reserved.

#pragma once

#include "CoreMinimal.h"

class ATrafficVehicleController;

/**
 * Junction negotiation logic extracted from ATrafficVehicleController.
 *
 * Handles:
 *  - Approach scan (detect junctions ahead, compute braking envelope)
 *  - Lane-end detection + right-of-way + occupy attempt
 *  - Post-transition release (unlock junction after lane switch)
 *  - Traversal steering (follow synthesized junction curve)
 *
 * Accesses all state through Owner pointer; not a UObject.
 */
struct AAA_TRAFFIC_API FJunctionNegotiator
{
	/** Owning controller — set once in the controller's constructor. */
	ATrafficVehicleController* Owner = nullptr;

	/**
	 * Junction approach scan.
	 * Queries provider for distance-to-next-junction, computes approach speed
	 * envelope, and initiates lane pre-positioning for upcoming turns.
	 *
	 * @return True if approach braking is active (vehicle should decelerate).
	 */
	bool TickApproach(int32 ClosestIndex, float AbsSpeed, float CurrentSpeed);

	/**
	 * Lane-end detection + junction right-of-way + occupy attempt.
	 * Called inside the detection gate when near lane-end.
	 * Finds junctions via three look-up paths, resolves entry point,
	 * and attempts right-of-way acquisition.
	 */
	void TickDetectAndOccupy(float RemainingDist, float TransitionThreshold,
		float AbsSpeed, float CurrentSpeed, const FVector& VehicleLocation);

	/**
	 * Post-transition junction release check.
	 * After CheckLaneTransition places the vehicle on a new lane, checks
	 * whether junction occupancy should be released or deferred.
	 */
	void TickPostTransitionRelease();

	/**
	 * Junction curve traversal.
	 * Walks the vehicle along synthesized Hermite transition points,
	 * releasing junction occupancy when the curve is complete.
	 *
	 * @param OutTargetPoint  Receives the steering target for this tick.
	 * @return True if traversal was active and OutTargetPoint was set.
	 */
	bool TickTraverse(const FVector& VehicleLocation, int32 ClosestIndex,
		FVector& OutTargetPoint);
};
