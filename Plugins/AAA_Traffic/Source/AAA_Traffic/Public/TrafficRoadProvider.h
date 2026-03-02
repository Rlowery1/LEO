// Copyright AAA_Traffic Contributors. All Rights Reserved.

#pragma once

#include "CoreMinimal.h"
#include "UObject/Interface.h"
#include "TrafficRoadProvider.generated.h"

/**
 * Which side of the current lane to query for an adjacent lane.
 */
UENUM(BlueprintType)
enum class ETrafficLaneSide : uint8
{
	Left,
	Right
};

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

	/**
	 * Get the adjacent (neighbor) lane on the given side.
	 * Returns an invalid handle if no same-direction neighbor exists on that side.
	 * Only returns neighbors on the same road segment (no cross-road adjacency).
	 */
	virtual FTrafficLaneHandle GetAdjacentLane(const FTrafficLaneHandle& Lane, ETrafficLaneSide Side) = 0;

	/**
	 * Get the road that owns the given lane.
	 * Returns an invalid handle if the lane is unknown.
	 */
	virtual FTrafficRoadHandle GetRoadForLane(const FTrafficLaneHandle& Lane) = 0;

	/**
	 * Get the speed limit for a lane (cm/s).
	 * Returns a negative value (e.g. -1.0) if the road kit has no speed-limit data,
	 * in which case the caller should fall back to its own default speed.
	 */
	virtual float GetLaneSpeedLimit(const FTrafficLaneHandle& Lane) = 0;

	/**
	 * Get the junction identifier at the end of a lane.
	 * Returns a non-zero ID if the lane terminates at an intersection / multi-road junction.
	 * Returns 0 if the lane continues within the same road or is a dead end.
	 */
	virtual int32 GetJunctionForLane(const FTrafficLaneHandle& Lane) { return 0; }

	/**
	 * Get the world-space centroid for a junction.
	 * Returns true and fills OutCentroid if the provider has authoritative
	 * centroid data for the given junction.  Returns false if not available,
	 * in which case the caller should compute its own centroid.
	 */
	virtual bool GetJunctionCentroid(int32 JunctionId, FVector& OutCentroid) { return false; }

	/**
	 * Get a smooth path through a junction between two connected lanes.
	 * Fills OutPath with a series of world-space points representing the junction trajectory.
	 * Returns true if a path was generated, false otherwise (caller should fall back to straight-line).
	 */
	virtual bool GetJunctionPath(const FTrafficLaneHandle& FromLane, const FTrafficLaneHandle& ToLane, TArray<FVector>& OutPath) { return false; }

	/**
	 * Query whether a lane's travel direction is reversed relative to its
	 * road's reference direction.  When true, the provider returns lane
	 * path points already oriented in the correct travel direction.
	 */
	virtual bool IsLaneReversed(const FTrafficLaneHandle& Lane) { return false; }

	/**
	 * Get the exact 3D world position where an intersection begins for a
	 * given approach lane. Derived from intersection mask geometry (not
	 * quantized polyline indices), so it adapts to any intersection shape.
	 *
	 * @param ApproachLane  Handle to a free-flow lane that precedes an intersection.
	 * @param OutPoint      Filled with the precise intersection boundary position.
	 * @return true if the lane approaches an intersection and the point was computed.
	 */
	virtual bool GetIntersectionEntryPoint(const FTrafficLaneHandle& ApproachLane, FVector& OutPoint) { return false; }

	/**
	 * Test whether two junction-crossing movements conflict (i.e. their paths
	 * through the junction would geometrically intersect).
	 * Used by multi-vehicle junction occupancy to allow non-conflicting
	 * movements to proceed simultaneously.
	 *
	 * @param FromA  Approach lane of vehicle A (pre-junction free-flow lane).
	 * @param ToA    Exit lane of vehicle A (post-junction free-flow lane).
	 * @param FromB  Approach lane of vehicle B.
	 * @param ToB    Exit lane of vehicle B.
	 * @return true if the movements conflict (default: conservative, always true).
	 */
	virtual bool DoJunctionPathsConflict(
		const FTrafficLaneHandle& FromA, const FTrafficLaneHandle& ToA,
		const FTrafficLaneHandle& FromB, const FTrafficLaneHandle& ToB) { return true; }

	/**
	 * Get the total polyline length of a lane (cm).
	 * Used by the multi-hop junction scan to accumulate distance through
	 * the lane graph without re-fetching full polyline data.
	 * Returns 0.0 if the lane is unknown.
	 */
	virtual float GetLaneLength(const FTrafficLaneHandle& Lane) { return 0.0f; }

	/**
	 * Result of a multi-hop junction scan ahead of a given lane.
	 */
	struct FJunctionScanResult
	{
		/** Junction ID found (0 = none within search range). */
		int32 JunctionId = 0;

		/** Total distance (cm) from the current position to the junction entry. */
		float DistanceCm = 0.0f;

		/** The lane handle that IS the junction lane (for signal queries). */
		FTrafficLaneHandle JunctionLane;

		/** The free-flow lane immediately before the junction (approach lane). */
		FTrafficLaneHandle ApproachLane;

		bool IsValid() const { return JunctionId != 0; }
	};

	/**
	 * Walk the lane graph forward from the given lane to find the nearest
	 * upcoming junction within MaxSearchDistCm.
	 *
	 * Starts at RemainingDistOnCurrentLane from the end of StartLane, then
	 * follows GetConnectedLanes hop-by-hop, accumulating polyline lengths,
	 * until a lane with GetJunctionForLane() != 0 is found or
	 * MaxSearchDistCm is exceeded.
	 *
	 * Default implementation uses GetConnectedLanes / GetJunctionForLane /
	 * GetLaneLength — adapters can override for optimized lookups.
	 *
	 * @param StartLane                Handle to the vehicle's current lane.
	 * @param RemainingDistOnCurrentLane  Distance remaining on StartLane (cm).
	 * @param MaxSearchDistCm          Maximum forward search distance (cm).
	 * @param MaxHops                  Maximum number of lane transitions to follow.
	 * @return Scan result with junction info, or invalid if none found.
	 */
	virtual FJunctionScanResult GetDistanceToNextJunction(
		const FTrafficLaneHandle& StartLane,
		float RemainingDistOnCurrentLane,
		float MaxSearchDistCm = 50000.0f,
		int32 MaxHops = 10);
};
