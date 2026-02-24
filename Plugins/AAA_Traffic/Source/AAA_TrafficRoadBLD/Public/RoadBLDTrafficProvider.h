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

#if WITH_ROADBLD
private:
	/** Scan the world for RoadBLD actors and build internal handle maps. */
	void CacheRoadData();

	/** Build the lane connectivity table from RoadBLD corner data. */
	void BuildLaneConnectivity();

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

	/** Lane handle ID → RoadBLD lane object. */
	TMap<int32, TWeakObjectPtr<UDynamicRoadLane>> LaneHandleMap;

	/** Reverse lookup: lane object → handle ID. */
	TMap<TWeakObjectPtr<UDynamicRoadLane>, int32> LaneToHandleMap;

	/** Cached road network actor (used for corner traversal). */
	TWeakObjectPtr<ADynamicRoadNetwork> CachedRoadNetwork;

	/** Pre-built lane connectivity: lane handle ID → list of connected lane handles. */
	TMap<int32, TArray<FTrafficLaneHandle>> LaneConnectionMap;

	/** True once CacheRoadData() has run. */
	bool bCached;
#endif // WITH_ROADBLD
};
