// LeaderDetector.h — Leader vehicle detection via polyline sweep + spatial grid.
// Handles full-LOD (physics sweep) and reduced-LOD (spatial grid) paths,
// bumper-to-bumper correction, and brake-light perception.

#pragma once

#include "CoreMinimal.h"
#include "TrafficRoadProvider.h"

class UTrafficSubsystem;
class ATrafficVehicleController;
enum class ETrafficLOD : uint8;
enum class ELaneChangeState : uint8;

struct AAA_TRAFFIC_API FLeaderDetectorInput
{
	// Vehicle state
	FVector VehicleLocation   = FVector::ZeroVector;
	FVector VehicleForward    = FVector::ForwardVector;
	float   VehicleFrontExtent = 0.0f;
	float   VehicleRearExtent  = 0.0f;
	float   DetectionDistance  = 0.0f;
	float   LaneWidth         = 0.0f;

	// Lane data
	TArrayView<const FVector> LanePoints;
	int32 ClosestIndex         = 0;
	ELaneChangeState LaneChangeState;
	FTrafficLaneHandle CurrentLane;
	ITrafficRoadProvider* RoadProvider = nullptr;
	bool bLaneDataReady        = false;

	// Junction data
	int32 JunctionId           = 0;
	TArrayView<const FVector> JunctionTransitionPoints;
	int32 JunctionTransitionIndex = 0;

	// LOD + subsystem
	ETrafficLOD TickLOD;
	UTrafficSubsystem* TrafficSubsystem = nullptr;

	// For physics sweep (full LOD)
	const UWorld* World        = nullptr;
	const APawn* ControlledPawn = nullptr;

	// Brake-light perception
	float IDMReactionDelaySec  = 0.0f;
};

struct AAA_TRAFFIC_API FLeaderDetectorOutput
{
	float LeaderDist                 = -1.0f;  // <0 = no leader
	float LeaderSpeed                = 0.0f;
	bool  bLeaderBrakeLightsVisible  = false;
};

/** Stateless leader detector. */
struct AAA_TRAFFIC_API FLeaderDetector
{
	FLeaderDetectorOutput Detect(const FLeaderDetectorInput& In) const;

private:
	float SweepPolyline(const FLeaderDetectorInput& In, float& OutSpeed) const;
	float SweepStraight(const FLeaderDetectorInput& In, float& OutSpeed) const;
	void  DetectBrakeLights(const FLeaderDetectorInput& In, float LeaderDist,
	                        FLeaderDetectorOutput& Out) const;
};
