// Copyright AAA_Traffic Contributors. All Rights Reserved.

#include "RoadBLDTrafficProvider.h"
#include "TrafficSubsystem.h"
#include "TrafficLog.h"

#if WITH_ROADBLD
#include "DynamicRoad.h"
#include "DynamicRoadNetwork.h"
#include "DynamicRoadLane.h"
#include "EdgeCurve.h"
#include "ClothoidCurve.h"       // UCurveObject (base of UEdgeCurve, UReferenceLine)
#include "RoadUtilityLibrary.h"
#include "EngineUtils.h"         // TActorIterator
#endif

// ---------------------------------------------------------------------------
// USubsystem overrides
// ---------------------------------------------------------------------------

bool URoadBLDTrafficProvider::ShouldCreateSubsystem(UObject* Outer) const
{
#if WITH_ROADBLD
	return true;
#else
	return false; // No RoadBLD — don't instantiate this subsystem.
#endif
}

void URoadBLDTrafficProvider::Initialize(FSubsystemCollectionBase& Collection)
{
	Super::Initialize(Collection);
#if WITH_ROADBLD
	NextHandleId = 1;
	bCached = false;
#endif
}

void URoadBLDTrafficProvider::Deinitialize()
{
#if WITH_ROADBLD
	if (UWorld* World = GetWorld())
	{
		if (UTrafficSubsystem* TrafficSub = World->GetSubsystem<UTrafficSubsystem>())
		{
			TrafficSub->UnregisterProvider(this);
		}
	}

	RoadHandleMap.Empty();
	LaneHandleMap.Empty();
	LaneToHandleMap.Empty();
	LaneConnectionMap.Empty();
	CachedRoadNetwork = nullptr;
	bCached = false;
#endif
	Super::Deinitialize();
}

void URoadBLDTrafficProvider::OnWorldBeginPlay(UWorld& InWorld)
{
	Super::OnWorldBeginPlay(InWorld);
#if WITH_ROADBLD
	CacheRoadData();

	if (UTrafficSubsystem* TrafficSub = InWorld.GetSubsystem<UTrafficSubsystem>())
	{
		TrafficSub->RegisterProvider(this);
		UE_LOG(LogAAATraffic, Log,
			TEXT("RoadBLDTrafficProvider: Registered — %d roads, %d lanes cached."),
			RoadHandleMap.Num(), LaneHandleMap.Num());
	}
#endif
}

// ---------------------------------------------------------------------------
// ITrafficRoadProvider — stubs (WITH_ROADBLD == 0) or full impl
// ---------------------------------------------------------------------------

#if WITH_ROADBLD

void URoadBLDTrafficProvider::CacheRoadData()
{
	if (bCached) return;

	UWorld* World = GetWorld();
	if (!World) return;

	for (TActorIterator<ADynamicRoadNetwork> It(World); It; ++It)
	{
		CachedRoadNetwork = *It;
		break;
	}

	// Collect all roads and sort by name for deterministic handle assignment.
	// TActorIterator order is non-deterministic — sorting ensures stable IDs
	// across runs with the same world state (System.md §4.4).
	TArray<ADynamicRoad*> SortedRoads;
	for (TActorIterator<ADynamicRoad> It(World); It; ++It)
	{
		SortedRoads.Add(*It);
	}
	SortedRoads.Sort([](const ADynamicRoad* A, const ADynamicRoad* B)
	{
		return A->GetName() < B->GetName();
	});

	for (ADynamicRoad* Road : SortedRoads)
	{
		const int32 RoadId = NextHandleId++;
		RoadHandleMap.Add(RoadId, Road);

		TArray<UDynamicRoadLane*> Lanes = Road->GetAllLanes();
		// Sort lanes by name for deterministic ordering.
		Lanes.Sort([](const UDynamicRoadLane* A, const UDynamicRoadLane* B)
		{
			return A->GetName() < B->GetName();
		});
		for (UDynamicRoadLane* Lane : Lanes)
		{
			if (!Lane) continue;
			const int32 LaneId = NextHandleId++;
			LaneHandleMap.Add(LaneId, Lane);
			LaneToHandleMap.Add(Lane, LaneId);
		}
	}

	bCached = true;

	BuildLaneConnectivity();
}

void URoadBLDTrafficProvider::BuildLaneConnectivity()
{
	LaneConnectionMap.Empty();

	ADynamicRoadNetwork* Network = CachedRoadNetwork.Get();
	if (!Network)
	{
		UE_LOG(LogAAATraffic, Warning,
			TEXT("RoadBLDTrafficProvider: No road network found — lane connectivity unavailable."));
		return;
	}

	const TArray<FRoadNetworkCorner>& Corners = Network->RoadNetworkCorners;

	// Helper: collect lane handles adjacent to an edge curve.
	auto CollectLaneHandles = [this](UEdgeCurve* Edge, TArray<int32>& OutHandles)
	{
		if (!Edge) return;
		if (UDynamicRoadLane* Left = Edge->LeftLane)
		{
			if (const int32* Handle = LaneToHandleMap.Find(Left))
			{
				OutHandles.AddUnique(*Handle);
			}
		}
		if (UDynamicRoadLane* Right = Edge->RightLane)
		{
			if (const int32* Handle = LaneToHandleMap.Find(Right))
			{
				OutHandles.AddUnique(*Handle);
			}
		}
	};

	for (const FRoadNetworkCorner& Corner : Corners)
	{
		if (Corner.bStale) continue;
		if (!Corner.StartEdge || !Corner.EndEdge) continue;

		TArray<int32> StartLaneHandles;
		TArray<int32> EndLaneHandles;
		CollectLaneHandles(Corner.StartEdge, StartLaneHandles);
		CollectLaneHandles(Corner.EndEdge, EndLaneHandles);

		// Connect lanes on the StartEdge side to lanes on the EndEdge side (bidirectional).
		for (const int32 SrcHandle : StartLaneHandles)
		{
			TArray<FTrafficLaneHandle>& SrcConnections = LaneConnectionMap.FindOrAdd(SrcHandle);
			for (const int32 DstHandle : EndLaneHandles)
			{
				if (SrcHandle == DstHandle) continue;
				SrcConnections.AddUnique(FTrafficLaneHandle(DstHandle));
			}
		}
		for (const int32 DstHandle : EndLaneHandles)
		{
			TArray<FTrafficLaneHandle>& DstConnections = LaneConnectionMap.FindOrAdd(DstHandle);
			for (const int32 SrcHandle : StartLaneHandles)
			{
				if (SrcHandle == DstHandle) continue;
				DstConnections.AddUnique(FTrafficLaneHandle(SrcHandle));
			}
		}
	}

	// Sort each connection list by HandleId for deterministic selection (System.md §4.4).
	for (auto& Pair : LaneConnectionMap)
	{
		Pair.Value.Sort([](const FTrafficLaneHandle& A, const FTrafficLaneHandle& B)
		{
			return A.HandleId < B.HandleId;
		});
	}

	int32 ConnectedLaneCount = 0;
	for (const auto& Pair : LaneConnectionMap)
	{
		ConnectedLaneCount += Pair.Value.Num();
	}
	UE_LOG(LogAAATraffic, Log,
		TEXT("RoadBLDTrafficProvider: Built lane connectivity — %d lanes with connections, %d total links."),
		LaneConnectionMap.Num(), ConnectedLaneCount);
}

TArray<FTrafficRoadHandle> URoadBLDTrafficProvider::GetAllRoads()
{
	TArray<FTrafficRoadHandle> Result;
	Result.Reserve(RoadHandleMap.Num());

	TArray<int32> Keys;
	RoadHandleMap.GetKeys(Keys);
	Keys.Sort();

	for (const int32 Key : Keys)
	{
		if (RoadHandleMap[Key].IsValid())
		{
			Result.Emplace(Key);
		}
	}
	return Result;
}

TArray<FTrafficLaneHandle> URoadBLDTrafficProvider::GetLanesForRoad(const FTrafficRoadHandle& Road)
{
	TArray<FTrafficLaneHandle> Result;

	ADynamicRoad* RoadActor = ResolveRoad(Road);
	if (!RoadActor) return Result;

	TArray<UDynamicRoadLane*> Lanes = RoadActor->GetAllLanes();
	for (UDynamicRoadLane* Lane : Lanes)
	{
		if (!Lane) continue;
		if (const int32* HandleId = LaneToHandleMap.Find(Lane))
		{
			Result.Emplace(*HandleId);
		}
	}

	// Sort by HandleId for deterministic output order (System.md §4.4).
	Result.Sort([](const FTrafficLaneHandle& A, const FTrafficLaneHandle& B)
	{
		return A.HandleId < B.HandleId;
	});
	return Result;
}

bool URoadBLDTrafficProvider::GetLanePath(
	const FTrafficLaneHandle& Lane,
	TArray<FVector>& OutPoints,
	float& OutWidth)
{
	UDynamicRoadLane* RoadLane = ResolveLane(Lane);
	if (!RoadLane) return false;

	OutWidth = static_cast<float>(RoadLane->LaneWidth);

	// Build centerline from the lane's own left/right edge curves.
	// The midpoint of the two edges at each sample gives the true lane center,
	// avoiding the P1 bug where all lanes shared the road reference line.
	UEdgeCurve* LeftEdge = RoadLane->LeftEdgeCurve;
	UEdgeCurve* RightEdge = RoadLane->RightEdgeCurve;

	ADynamicRoad* Road = GetRoadForLane(RoadLane);
	if (!Road) return false;

	const double RoadLength = Road->GetLength();
	if (RoadLength <= 0.0) return false;

	const double SampleInterval = 100.0; // 1 m in UE units (cm)
	const int32 NumSamples = FMath::Max(2, FMath::CeilToInt(RoadLength / SampleInterval) + 1);

	OutPoints.Reset(NumSamples);
	for (int32 i = 0; i < NumSamples; ++i)
	{
		const double Distance = FMath::Min(
			RoadLength,
			(RoadLength * static_cast<double>(i)) / static_cast<double>(NumSamples - 1));

		FVector Pos;

		if (LeftEdge && RightEdge && Road->ReferenceLine)
		{
			// Convert distance from reference-line space to each edge curve's space.
			const double LeftDist = Road->ConvertDistanceBetweenCurves(
				Road->ReferenceLine, LeftEdge, Distance);
			const double RightDist = Road->ConvertDistanceBetweenCurves(
				Road->ReferenceLine, RightEdge, Distance);

			// UCurveObject::Get3DPositionAtDistance returns FVector (3D world pos).
			const FVector LeftPos = LeftEdge->Get3DPositionAtDistance(
				Road->ReferenceLine, LeftDist);
			const FVector RightPos = RightEdge->Get3DPositionAtDistance(
				Road->ReferenceLine, RightDist);

			Pos = (LeftPos + RightPos) * 0.5;
		}
		else if (Road->ReferenceLine)
		{
			// Fallback: use road reference line if edge curves unavailable.
			Pos = Road->ReferenceLine->Get3DPositionAtDistance(
				Road->ReferenceLine, Distance);
		}
		else
		{
			// Last-resort fallback: 2D position from road actor.
			const FVector2D Pos2D = Road->GetWorldPositionAtDistance(Distance);
			Pos = FVector(Pos2D.X, Pos2D.Y, Road->GetActorLocation().Z);
		}

		OutPoints.Add(Pos);
	}

	return true;
}

FVector URoadBLDTrafficProvider::GetLaneDirection(const FTrafficLaneHandle& Lane)
{
	TArray<FVector> Points;
	float Width;
	if (GetLanePath(Lane, Points, Width) && Points.Num() >= 2)
	{
		return (Points.Last() - Points[0]).GetSafeNormal();
	}
	return FVector::ForwardVector;
}

TArray<FTrafficLaneHandle> URoadBLDTrafficProvider::GetConnectedLanes(const FTrafficLaneHandle& Lane)
{
	if (const TArray<FTrafficLaneHandle>* Connected = LaneConnectionMap.Find(Lane.HandleId))
	{
		return *Connected;
	}
	return TArray<FTrafficLaneHandle>();
}

FTrafficLaneHandle URoadBLDTrafficProvider::GetLaneAtLocation(const FVector& Location)
{
	// Sort road handle keys for deterministic iteration order (System.md §4.4).
	TArray<int32> Keys;
	RoadHandleMap.GetKeys(Keys);
	Keys.Sort();

	for (const int32 Key : Keys)
	{
		ADynamicRoad* Road = RoadHandleMap[Key].Get();
		if (!Road) continue;

		UDynamicRoadLane* Lane = URoadUtilityLibrary::GetLaneAtLocation(Road, Location);
		if (Lane)
		{
			if (const int32* HandleId = LaneToHandleMap.Find(Lane))
			{
				return FTrafficLaneHandle(*HandleId);
			}
		}
	}

	return FTrafficLaneHandle();
}

ADynamicRoad* URoadBLDTrafficProvider::ResolveRoad(const FTrafficRoadHandle& Handle) const
{
	if (const TWeakObjectPtr<ADynamicRoad>* Found = RoadHandleMap.Find(Handle.HandleId))
	{
		return Found->Get();
	}
	return nullptr;
}

UDynamicRoadLane* URoadBLDTrafficProvider::ResolveLane(const FTrafficLaneHandle& Handle) const
{
	if (const TWeakObjectPtr<UDynamicRoadLane>* Found = LaneHandleMap.Find(Handle.HandleId))
	{
		return Found->Get();
	}
	return nullptr;
}

ADynamicRoad* URoadBLDTrafficProvider::GetRoadForLane(UDynamicRoadLane* Lane) const
{
	if (!Lane) return nullptr;
	return Lane->GetTypedOuter<ADynamicRoad>();
}

#else // !WITH_ROADBLD — empty stubs so the module compiles without RoadBLD

TArray<FTrafficRoadHandle> URoadBLDTrafficProvider::GetAllRoads() { return {}; }
TArray<FTrafficLaneHandle> URoadBLDTrafficProvider::GetLanesForRoad(const FTrafficRoadHandle&) { return {}; }
bool URoadBLDTrafficProvider::GetLanePath(const FTrafficLaneHandle&, TArray<FVector>&, float&) { return false; }
FVector URoadBLDTrafficProvider::GetLaneDirection(const FTrafficLaneHandle&) { return FVector::ForwardVector; }
TArray<FTrafficLaneHandle> URoadBLDTrafficProvider::GetConnectedLanes(const FTrafficLaneHandle&) { return {}; }
FTrafficLaneHandle URoadBLDTrafficProvider::GetLaneAtLocation(const FVector&) { return {}; }

#endif // WITH_ROADBLD
