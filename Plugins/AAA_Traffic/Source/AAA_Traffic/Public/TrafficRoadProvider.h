// Copyright AAA_Traffic Contributors. All Rights Reserved.

#pragma once

#include "CoreMinimal.h"
#include "UObject/Interface.h"
#include "TrafficRoadProvider.generated.h"

/**
 * Opaque handle to a road managed by a traffic road provider.
 * The handle ID is meaningful only to the adapter that created it.
 */
USTRUCT(BlueprintType)
struct AAA_TRAFFIC_API FTrafficRoadHandle
{
	GENERATED_BODY()

	FTrafficRoadHandle() : HandleId(0) {}
	explicit FTrafficRoadHandle(int32 InId) : HandleId(InId) {}

	bool IsValid() const { return HandleId != 0; }

	bool operator==(const FTrafficRoadHandle& Other) const { return HandleId == Other.HandleId; }
	bool operator!=(const FTrafficRoadHandle& Other) const { return HandleId != Other.HandleId; }

	friend uint32 GetTypeHash(const FTrafficRoadHandle& Handle) { return ::GetTypeHash(Handle.HandleId); }

	UPROPERTY()
	int32 HandleId;
};

/**
 * Opaque handle to a lane managed by a traffic road provider.
 * The handle ID is meaningful only to the adapter that created it.
 */
USTRUCT(BlueprintType)
struct AAA_TRAFFIC_API FTrafficLaneHandle
{
	GENERATED_BODY()

	FTrafficLaneHandle() : HandleId(0) {}
	explicit FTrafficLaneHandle(int32 InId) : HandleId(InId) {}

	bool IsValid() const { return HandleId != 0; }

	bool operator==(const FTrafficLaneHandle& Other) const { return HandleId == Other.HandleId; }
	bool operator!=(const FTrafficLaneHandle& Other) const { return HandleId != Other.HandleId; }

	friend uint32 GetTypeHash(const FTrafficLaneHandle& Handle) { return ::GetTypeHash(Handle.HandleId); }

	UPROPERTY()
	int32 HandleId;
};

/**
 * Interface that all road-kit adapters must implement.
 *
 * Provides a uniform set of queries the traffic system uses
 * to interact with any road kit's native data.
 * Each adapter answers these questions using its road kit's own API — no
 * data conversion or duplication is required.
 */
UINTERFACE(MinimalAPI)
class UTrafficRoadProvider : public UInterface
{
	GENERATED_BODY()
};

class AAA_TRAFFIC_API ITrafficRoadProvider
{
	GENERATED_BODY()

public:
	/** Get handles to all roads currently in the world. */
	virtual TArray<FTrafficRoadHandle> GetAllRoads() = 0;

	/** Get handles to all lanes belonging to the given road. */
	virtual TArray<FTrafficLaneHandle> GetLanesForRoad(const FTrafficRoadHandle& Road) = 0;

	/**
	 * Get the world-space centerline points and width for a lane.
	 * @param Lane          Handle to the lane.
	 * @param OutPoints     Filled with ordered world-space centerline positions.
	 * @param OutWidth      Filled with lane width in cm.
	 * @return true if the lane was resolved and data was filled.
	 */
	virtual bool GetLanePath(const FTrafficLaneHandle& Lane, TArray<FVector>& OutPoints, float& OutWidth) = 0;

	/**
	 * Get the traffic flow direction for a lane (unit vector along the lane's travel direction).
	 */
	virtual FVector GetLaneDirection(const FTrafficLaneHandle& Lane) = 0;

	/**
	 * Get the lanes reachable from the end of the given lane
	 * (intersection / continuation connectivity).
	 * Returns empty if the lane is a dead end or connectivity is not yet available.
	 */
	virtual TArray<FTrafficLaneHandle> GetConnectedLanes(const FTrafficLaneHandle& Lane) = 0;

	/**
	 * Find the lane closest to the given world location.
	 * Returns an invalid handle if no lane is near the location.
	 */
	virtual FTrafficLaneHandle GetLaneAtLocation(const FVector& Location) = 0;
};
