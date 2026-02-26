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

	/** Build corner-based lane connectivity from RoadBLD corner data (fallback). */
	void BuildLaneConnectivity();

	/** Cache start/end positions and directions for every lane. Called after CacheRoadData. */
	void CacheLaneEndpoints();

	/** Detect through-roads and create virtual lane segments at intersections. */
	void DetectAndSplitThroughRoads();

	/** Build proximity-based connections between lane endpoints on different roads. */
	void BuildProximityConnections();

	/** Group connected lane endpoints into junction clusters and assign junction IDs. */
	void BuildJunctionGrouping();

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

	// ----- Structs -----

	/** Cached lane endpoint geometry — computed once after CacheRoadData. */
	struct FLaneEndpointCache
	{
		FVector StartPos = FVector::ZeroVector;
		FVector EndPos = FVector::ZeroVector;
		FVector StartDir = FVector::ForwardVector;
		FVector EndDir = FVector::ForwardVector;
		TArray<FVector> Polyline;
		float Width = 0.0f;
	};

	/** Virtual lane segment info — for through-road splitting. */
	struct FVirtualLaneInfo
	{
		int32 OriginalLaneHandle = 0;
		int32 StartPointIndex = 0;
		int32 EndPointIndex = 0;
	};

	/** Record of a proximity-based connection for junction grouping. */
	struct FProximityConnection
	{
		int32 FromLane = 0;
		int32 ToLane = 0;
		FVector Midpoint = FVector::ZeroVector;
	};

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

	// ── Proximity connectivity state ────────────────────────

	/** Lane handle → cached endpoint geometry. */
	TMap<int32, FLaneEndpointCache> LaneEndpointMap;

	/** Virtual lane handle → info about what portion of the original lane it represents. */
	TMap<int32, FVirtualLaneInfo> VirtualLaneMap;

	/** Original lane handle → ordered list of virtual lane handles that replace it. */
	TMap<int32, TArray<int32>> OriginalToVirtualMap;

	/** Original lane handles that have been split into virtual segments. */
	TSet<int32> ReplacedLaneHandles;

	/** Proximity connections detected — used for junction grouping. */
	TArray<FProximityConnection> ProximityConnectionList;
#endif // WITH_ROADBLD
};
