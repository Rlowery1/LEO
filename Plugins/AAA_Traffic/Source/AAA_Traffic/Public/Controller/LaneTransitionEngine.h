// LaneTransitionEngine.h — Lane-end transition detection, exit selection, and lane initialization.
#pragma once

#include "CoreMinimal.h"
#include "TrafficRoadProvider.h"

class ATrafficVehicleController;

/**
 * Encapsulates lane-end transition logic: detecting when a vehicle reaches
 * the end of its current lane, selecting the next lane (exit), generating
 * junction smoothing curves, and initializing the new lane-following state.
 *
 * Operates on the owning controller via a friend pointer — no persistent
 * inter-tick state of its own.
 */
struct AAA_TRAFFIC_API FLaneTransitionEngine
{
	ATrafficVehicleController* Owner = nullptr;

	/**
	 * Full lane-end transition: select exit lane, initialize it, generate
	 * junction smoothing curves.  Called when the vehicle's remaining
	 * distance on the current lane drops below the transition threshold.
	 */
	void CheckTransition();

	/**
	 * Initialize lane-following state for a given lane.
	 * Resets steering, acceleration, lane-change state, fetches polyline
	 * and speed limit from the provider.
	 */
	void InitializeLane(const FTrafficLaneHandle& InLane);
};
