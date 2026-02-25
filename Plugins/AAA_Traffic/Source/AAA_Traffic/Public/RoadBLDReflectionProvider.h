// Copyright AAA_Traffic Contributors. All Rights Reserved.

#pragma once

#include "CoreMinimal.h"
#include "Subsystems/WorldSubsystem.h"
#include "TrafficRoadProvider.h"
#include "RoadBLDReflectionProvider.generated.h"

/**
 * Reflection-based RoadBLD adapter — implements ITrafficRoadProvider
 * entirely through UE's UFUNCTION / UPROPERTY reflection system.
 *
 * Unlike URoadBLDTrafficProvider (in AAA_TrafficRoadBLD module), this class
 * requires NO compile-time RoadBLD headers.  It discovers RoadBLD classes
 * at runtime via FindObject<UClass>("/Script/RoadBLDRuntime.ClassName") and
 * invokes all methods through ProcessEvent.
 *
 * Lifecycle:
 *   1. ShouldCreateSubsystem — returns true only if the DynamicRoad UClass
 *      exists in any loaded module (i.e. RoadBLD plugin is present).
 *   2. OnWorldBeginPlay — runs API contract validation, caches road/lane data,
 *      builds lane connectivity, then registers with UTrafficSubsystem.
 *   3. Yields to any already-registered compiled provider so that
 *      URoadBLDTrafficProvider (WITH_ROADBLD=1) always wins when available.
 *
 * Determinism: all container iterations are sorted by name/handle, lane
 * selection is done through sorted arrays, never random map traversal.
 */
UCLASS()
class AAA_TRAFFIC_API URoadBLDReflectionProvider : public UWorldSubsystem, public ITrafficRoadProvider
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

private:
	// ── Data caching ────────────────────────────────────────

	/** Discover roads, lanes, and cache handle maps via reflection. */
	void CacheRoadData(UWorld* World);

	/** Build lane connectivity from RoadNetworkCorners via reflection. */
	void BuildLaneConnectivity(UWorld* World);

	/** Detect left/right neighbor lanes on the same road via shared edge curves. */
	void BuildLaneAdjacency();

	// ── Reflection helpers ──────────────────────────────────

	/**
	 * Call a UFUNCTION on a UObject via ProcessEvent, with typed parameters.
	 * Caller supplies a pre-zeroed param struct sized to Func->ParmsSize.
	 */
	static void CallReflection(UObject* Target, UFunction* Func, void* Params);

	/**
	 * Get the road length (double) from a DynamicRoad actor.
	 * Returns 0.0 on failure.
	 */
	double GetRoadLength(UObject* RoadActor) const;

	/**
	 * Get all lane UObjects from a DynamicRoad actor.
	 * Returns an empty array on failure.
	 */
	TArray<UObject*> GetAllLanesForRoad(UObject* RoadActor) const;

	/**
	 * Get the reference-line UObject from a DynamicRoad actor.
	 * Returns nullptr on failure.
	 */
	UObject* GetReferenceLine(UObject* RoadActor) const;

	/**
	 * Invoke ConvertDistanceBetweenCurves(From, To, Distance) → double
	 * on a DynamicRoad actor via reflection.
	 */
	double ConvertDistanceBetweenCurves(UObject* RoadActor, UObject* From, UObject* To, double Distance) const;

	/**
	 * Invoke Get3DPositionAtDistance(ReferenceLine, Distance) → FVector
	 * on a curve object via reflection.
	 */
	FVector Get3DPositionAtDistance(UObject* CurveObj, UObject* RefLine, double Distance) const;

	/**
	 * Invoke GetWorldPositionAtDistance(Distance) → FVector2D
	 * on a road actor via reflection.
	 */
	FVector2D GetWorldPositionAtDistance(UObject* RoadActor, double Distance) const;

	// ── Per-lane data ───────────────────────────────────────

	/** Stored lane metadata keyed by handle ID. */
	struct FReflectionLaneData
	{
		/** The lane UObject (UDynamicRoadLane). */
		TWeakObjectPtr<UObject> LaneObject;

		/** The owning road actor (ADynamicRoad). */
		TWeakObjectPtr<UObject> RoadActor;

		/** The lane width in cm. */
		float LaneWidth = 0.0f;

		/** The left edge curve UObject (UEdgeCurve). */
		TWeakObjectPtr<UObject> LeftEdge;

		/** The right edge curve UObject (UEdgeCurve). */
		TWeakObjectPtr<UObject> RightEdge;
	};

	// ── Internal state ──────────────────────────────────────

	/** Monotonically increasing handle ID — shared across roads and lanes. */
	int32 NextHandleId = 1;

	/** Road handle ID → road actor UObject. */
	TMap<int32, TWeakObjectPtr<UObject>> RoadHandleMap;

	/** Road handle ID → ordered list of lane handle IDs for that road. */
	TMap<int32, TArray<int32>> RoadToLaneHandles;

	/** Lane handle ID → lane metadata. */
	TMap<int32, FReflectionLaneData> LaneHandleMap;

	/** Reverse lookup: lane UObject → handle ID. */
	TMap<TWeakObjectPtr<UObject>, int32> LaneToHandleMap;

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

	/** True once CacheRoadData() has successfully run. */
	bool bCached = false;

	// ── Cached reflection pointers (resolved once in CacheRoadData) ─

	/** DynamicRoad UClass. */
	UClass* DynRoadClass = nullptr;

	/** DynamicRoadNetwork UClass. */
	UClass* DynNetworkClass = nullptr;

	/** DynamicRoad::GetLength() */
	UFunction* GetLengthFunc = nullptr;

	/** DynamicRoad::GetAllLanes() */
	UFunction* GetAllLanesFunc = nullptr;

	/** DynamicRoad::ConvertDistanceBetweenCurves() */
	UFunction* ConvertDistFunc = nullptr;

	/** DynamicRoad::GetWorldPositionAtDistance() */
	UFunction* GetWorldPosFunc = nullptr;

	/** DynamicRoad::ReferenceLine property */
	FProperty* RefLineProp = nullptr;

	/** Curve class Get3DPositionAtDistance — resolved lazily from first edge. */
	UFunction* Get3DPosFunc = nullptr;

	/** Lane class LeftEdgeCurve property. */
	FProperty* LeftEdgeProp = nullptr;

	/** Lane class RightEdgeCurve property. */
	FProperty* RightEdgeProp = nullptr;

	/** Lane class LaneWidth property. */
	FProperty* LaneWidthProp = nullptr;

	/** FRoadNetworkCorner::IntersectionPoint property. */
	FProperty* IntersectionPointProp = nullptr;
};
