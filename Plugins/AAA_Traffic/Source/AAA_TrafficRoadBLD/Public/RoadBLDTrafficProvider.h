// Copyright AAA_Traffic Contributors. All Rights Reserved.

#pragma once

#include "CoreMinimal.h"
#include "Subsystems/WorldSubsystem.h"
#include "TrafficRoadProvider.h"
#include "RoadBLDTrafficProvider.generated.h"

#if WITH_ROADBLD
class ADynamicRoad;
class ADynamicRoadNetwork;
class UDynamicRoadLane;
#endif

/**
 * RoadBLD adapter — implements ITrafficRoadProvider by querying
 * RoadBLD's native road, lane, and corner data directly.
 *
 * Automatically registers with UTrafficSubsystem when the world begins play.
 * No data is duplicated: every query delegates to RoadBLD's own objects.
 *
 * When WITH_ROADBLD == 0 (RoadBLD not installed), this subsystem exists
 * but does nothing — ShouldCreateSubsystem returns false.
 */
UCLASS()
class AAA_TRAFFICROADBLD_API URoadBLDTrafficProvider : public UWorldSubsystem, public ITrafficRoadProvider
{
	GENERATED_BODY()

public:
	// --- USubsystem ---
	virtual bool ShouldCreateSubsystem(UObject* Outer) const override;
	virtual void Initialize(FSubsystemCollectionBase& Collection) override;
	virtual void Deinitialize() override;
	virtual void OnWorldBeginPlay(UWorld& InWorld) override;

	// --- ITrafficRoadProvider ---
	virtual TArray<FTrafficRoadHandle> GetAllRoads() override;
	virtual TArray<FTrafficLaneHandle> GetLanesForRoad(const FTrafficRoadHandle& Road) override;
	virtual bool GetLanePath(const FTrafficLaneHandle& Lane, TArray<FVector>& OutPoints, float& OutWidth) override;
	virtual FVector GetLaneDirection(const FTrafficLaneHandle& Lane) override;
	virtual TArray<FTrafficLaneHandle> GetConnectedLanes(const FTrafficLaneHandle& Lane) override;
	virtual FTrafficLaneHandle GetLaneAtLocation(const FVector& Location) override;
	virtual FTrafficLaneHandle GetAdjacentLane(const FTrafficLaneHandle& Lane, ETrafficLaneSide Side) override;
	virtual FTrafficRoadHandle GetRoadForLane(const FTrafficLaneHandle& Lane) override;
	virtual float GetLaneSpeedLimit(const FTrafficLaneHandle& Lane) override;
	virtual int32 GetJunctionForLane(const FTrafficLaneHandle& Lane) override;
	virtual bool GetJunctionPath(const FTrafficLaneHandle& FromLane, const FTrafficLaneHandle& ToLane, TArray<FVector>& OutPath) override;
	virtual bool IsLaneReversed(const FTrafficLaneHandle& Lane) override;

#if WITH_ROADBLD
private:
	/** Scan the world for RoadBLD actors and build internal handle maps. */
	void CacheRoadData();

	/** Build the lane connectivity table from RoadBLD corner data. */
	void BuildLaneConnectivity();

	/** Detect left/right neighbor lanes on the same road via shared edge curves. */
	void BuildLaneAdjacency();

	/** Detect lanes whose travel direction is reversed (left-of-center on 2-way roads). */
	void DetectReversedLanes();

	/** Resolve a road handle back to the live RoadBLD actor. */
	ADynamicRoad* ResolveRoad(const FTrafficRoadHandle& Handle) const;

	/** Resolve a lane handle back to the live RoadBLD lane object. */
	UDynamicRoadLane* ResolveLane(const FTrafficLaneHandle& Handle) const;

	/** Find the parent ADynamicRoad for a given lane. */
	ADynamicRoad* GetRoadForLane(UDynamicRoadLane* Lane) const;

	// ----- Internal state -----

	/** Monotonically increasing ID shared across roads and lanes. */
	int32 NextHandleId;

	/** Road handle ID → RoadBLD road actor. */
	TMap<int32, TWeakObjectPtr<ADynamicRoad>> RoadHandleMap;

	/** Road handle ID → ordered list of lane handle IDs for that road. */
	TMap<int32, TArray<int32>> RoadToLaneHandles;

	/** Lane handle ID → RoadBLD lane object. */
	TMap<int32, TWeakObjectPtr<UDynamicRoadLane>> LaneHandleMap;

	/** Reverse lookup: lane object → handle ID. */
	TMap<TWeakObjectPtr<UDynamicRoadLane>, int32> LaneToHandleMap;

	/** Cached road network actor (used for corner traversal). */
	TWeakObjectPtr<ADynamicRoadNetwork> CachedRoadNetwork;

	/** Pre-built lane connectivity: lane handle ID → list of connected lane handles. */
	TMap<int32, TArray<FTrafficLaneHandle>> LaneConnectionMap;

	/** Left neighbor: lane handle ID → left-adjacent lane handle ID (0 = none). */
	TMap<int32, int32> LeftNeighborMap;

	/** Right neighbor: lane handle ID → right-adjacent lane handle ID (0 = none). */
	TMap<int32, int32> RightNeighborMap;

	/** Reverse lookup: lane handle ID → owning road handle ID. */
	TMap<int32, int32> LaneToRoadHandleMap;

	/** Lane handle ID → junction ID (non-zero if lane terminates at a junction). */
	TMap<int32, int32> LaneToJunctionMap;

	/** Junction ID → centroid world position (for junction path generation). */
	TMap<int32, FVector> JunctionCentroids;

	/** Lane handles whose travel direction is reversed relative to reference line. */
	TSet<int32> ReversedLaneSet;

	/** True once CacheRoadData() has run. */
	bool bCached;
#endif // WITH_ROADBLD
};
