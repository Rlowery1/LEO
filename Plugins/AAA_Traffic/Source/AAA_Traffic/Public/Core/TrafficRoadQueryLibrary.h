// Copyright AAA_Traffic Contributors. All Rights Reserved.

#pragma once

#include "CoreMinimal.h"
#include "Kismet/BlueprintFunctionLibrary.h"
#include "TrafficRoadProvider.h"
#include "TrafficRoadQueryLibrary.generated.h"

/**
 * Static Blueprint function library that wraps the ITrafficRoadProvider
 * interface so Blueprint users can query road/lane data without C++.
 *
 * All functions pull the active provider from the world's UTrafficSubsystem.
 * If no provider is registered the functions return default/invalid values.
 */
UCLASS()
class AAA_TRAFFIC_API UTrafficRoadQueryLibrary : public UBlueprintFunctionLibrary
{
	GENERATED_BODY()

public:
	/** Get handles to all roads in the world. */
	UFUNCTION(BlueprintCallable, Category = "Traffic|Road", meta = (WorldContext = "WorldContextObject"))
	static TArray<FTrafficRoadHandle> GetAllRoads(const UObject* WorldContextObject);

	/** Get handles to all lanes belonging to a road. */
	UFUNCTION(BlueprintCallable, Category = "Traffic|Road", meta = (WorldContext = "WorldContextObject"))
	static TArray<FTrafficLaneHandle> GetLanesForRoad(const UObject* WorldContextObject, const FTrafficRoadHandle& Road);

	/** Get the centerline points and width for a lane. Returns false if lane is invalid. */
	UFUNCTION(BlueprintCallable, Category = "Traffic|Road", meta = (WorldContext = "WorldContextObject"))
	static bool GetLanePath(const UObject* WorldContextObject, const FTrafficLaneHandle& Lane,
		TArray<FVector>& OutPoints, float& OutWidth);

	/** Get the travel direction for a lane (unit vector). */
	UFUNCTION(BlueprintPure, Category = "Traffic|Road", meta = (WorldContext = "WorldContextObject"))
	static FVector GetLaneDirection(const UObject* WorldContextObject, const FTrafficLaneHandle& Lane);

	/** Get lanes connected at the end of the given lane. */
	UFUNCTION(BlueprintCallable, Category = "Traffic|Road", meta = (WorldContext = "WorldContextObject"))
	static TArray<FTrafficLaneHandle> GetConnectedLanes(const UObject* WorldContextObject, const FTrafficLaneHandle& Lane);

	/** Find the closest lane to a world location. Returns an invalid handle if no lane is nearby. */
	UFUNCTION(BlueprintCallable, Category = "Traffic|Road", meta = (WorldContext = "WorldContextObject"))
	static FTrafficLaneHandle GetLaneAtLocation(const UObject* WorldContextObject, const FVector& Location);

	/** Get the adjacent lane on a given side. Returns an invalid handle if none exists. */
	UFUNCTION(BlueprintCallable, Category = "Traffic|Road", meta = (WorldContext = "WorldContextObject"))
	static FTrafficLaneHandle GetAdjacentLane(const UObject* WorldContextObject,
		const FTrafficLaneHandle& Lane, ETrafficLaneSide Side);

	/** Get the road that owns a lane. Returns an invalid handle if the lane is unknown. */
	UFUNCTION(BlueprintPure, Category = "Traffic|Road", meta = (WorldContext = "WorldContextObject"))
	static FTrafficRoadHandle GetRoadForLane(const UObject* WorldContextObject, const FTrafficLaneHandle& Lane);

	/** Get the speed limit for a lane (cm/s). Returns -1 if unavailable. */
	UFUNCTION(BlueprintPure, Category = "Traffic|Road", meta = (WorldContext = "WorldContextObject"))
	static float GetLaneSpeedLimit(const UObject* WorldContextObject, const FTrafficLaneHandle& Lane);

	/** Get the junction ID at the end of a lane. Returns 0 if no junction. */
	UFUNCTION(BlueprintPure, Category = "Traffic|Road", meta = (WorldContext = "WorldContextObject"))
	static int32 GetJunctionForLane(const UObject* WorldContextObject, const FTrafficLaneHandle& Lane);
};
