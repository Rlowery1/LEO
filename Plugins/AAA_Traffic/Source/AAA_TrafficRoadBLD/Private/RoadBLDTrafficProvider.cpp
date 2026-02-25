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
	RoadToLaneHandles.Empty();
	LaneHandleMap.Empty();
	LaneToHandleMap.Empty();
	LaneConnectionMap.Empty();
	LeftNeighborMap.Empty();
	RightNeighborMap.Empty();
	LaneToRoadHandleMap.Empty();
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

		TArray<int32>& RoadLaneIds = RoadToLaneHandles.Add(RoadId);

		for (UDynamicRoadLane* Lane : Lanes)
		{
			if (!Lane) continue;
			const int32 LaneId = NextHandleId++;
			LaneHandleMap.Add(LaneId, Lane);
			LaneToHandleMap.Add(Lane, LaneId);
			RoadLaneIds.Add(LaneId);
			LaneToRoadHandleMap.Add(LaneId, RoadId);
		}
	}

	bCached = true;

	BuildLaneConnectivity();
	BuildLaneAdjacency();
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
				if (OutHandles.Contains(*Handle))
				{
					UE_LOG(LogAAATraffic, Warning,
						TEXT("RoadBLDTrafficProvider: Lane '%s' (handle %d) appears multiple times on edge '%s'."),
						*Left->GetName(), *Handle, *GetNameSafe(Edge));
				}
				OutHandles.AddUnique(*Handle);
			}
		}
		if (UDynamicRoadLane* Right = Edge->RightLane)
		{
			if (const int32* Handle = LaneToHandleMap.Find(Right))
			{
				if (OutHandles.Contains(*Handle))
				{
					UE_LOG(LogAAATraffic, Warning,
						TEXT("RoadBLDTrafficProvider: Lane '%s' (handle %d) appears multiple times on edge '%s'."),
						*Right->GetName(), *Handle, *GetNameSafe(Edge));
				}
				OutHandles.AddUnique(*Handle);
			}
		}
	};

	// Junction data collection for grouping pass.
	struct FCornerJunctionEntry
	{
		FVector IntersectionPoint;
		TArray<int32> StartLaneHandles;
	};
	TArray<FCornerJunctionEntry> CornerJunctionData;

	for (const FRoadNetworkCorner& Corner : Corners)
	{
		if (Corner.bStale) continue;
		if (!Corner.StartEdge || !Corner.EndEdge) continue;

		TArray<int32> StartLaneHandles;
		TArray<int32> EndLaneHandles;
		CollectLaneHandles(Corner.StartEdge, StartLaneHandles);
		CollectLaneHandles(Corner.EndEdge, EndLaneHandles);

		// Connect lanes on StartEdge to lanes on EndEdge (forward only).
		// FRoadNetworkCorner is directional: StartEdge flows into EndEdge.
		// Reverse links are intentionally omitted to prevent wrong-way traversal.
		for (const int32 SrcHandle : StartLaneHandles)
		{
			TArray<FTrafficLaneHandle>& SrcConnections = LaneConnectionMap.FindOrAdd(SrcHandle);
			for (const int32 DstHandle : EndLaneHandles)
			{
				if (SrcHandle == DstHandle) continue;
				SrcConnections.AddUnique(FTrafficLaneHandle(DstHandle));
			}
		}

		// Collect junction data for grouping pass.
		if (StartLaneHandles.Num() > 0)
		{
			CornerJunctionData.Add({Corner.IntersectionPoint, MoveTemp(StartLaneHandles)});
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

	// ── Junction grouping pass ───────────────────────────────────
	LaneToJunctionMap.Empty();
	JunctionCentroids.Empty();

	if (CornerJunctionData.Num() > 0)
	{
		constexpr float JunctionGroupThreshold = 200.0f; // cm
		const float ThresholdSq = JunctionGroupThreshold * JunctionGroupThreshold;

		// Union-find for clustering corners by proximity.
		TArray<int32> Parent;
		Parent.SetNumUninitialized(CornerJunctionData.Num());
		for (int32 k = 0; k < Parent.Num(); ++k) { Parent[k] = k; }

		TFunction<int32(int32)> Find = [&Parent, &Find](int32 X) -> int32
		{
			if (Parent[X] != X) { Parent[X] = Find(Parent[X]); }
			return Parent[X];
		};

		auto Union = [&Parent, &Find](int32 A, int32 B)
		{
			int32 RA = Find(A);
			int32 RB = Find(B);
			if (RA != RB) { Parent[RA] = RB; }
		};

		for (int32 a = 0; a < CornerJunctionData.Num(); ++a)
		{
			for (int32 b = a + 1; b < CornerJunctionData.Num(); ++b)
			{
				if (FVector::DistSquared(CornerJunctionData[a].IntersectionPoint,
					CornerJunctionData[b].IntersectionPoint) <= ThresholdSq)
				{
					Union(a, b);
				}
			}
		}

		TMap<int32, TArray<int32>> Clusters;
		for (int32 k = 0; k < CornerJunctionData.Num(); ++k)
		{
			Clusters.FindOrAdd(Find(k)).Add(k);
		}

		struct FClusterEntry
		{
			FVector Centroid;
			TArray<int32> CornerIndices;
		};
		TArray<FClusterEntry> SortedClusters;
		for (auto& ClusterPair : Clusters)
		{
			FClusterEntry Entry;
			FVector Sum = FVector::ZeroVector;
			for (int32 Idx : ClusterPair.Value)
			{
				Sum += CornerJunctionData[Idx].IntersectionPoint;
			}
			Entry.Centroid = Sum / static_cast<float>(ClusterPair.Value.Num());
			Entry.CornerIndices = MoveTemp(ClusterPair.Value);
			SortedClusters.Add(MoveTemp(Entry));
		}
		SortedClusters.Sort([](const FClusterEntry& A, const FClusterEntry& B)
		{
			if (!FMath::IsNearlyEqual(A.Centroid.X, B.Centroid.X, 1.0f)) return A.Centroid.X < B.Centroid.X;
			if (!FMath::IsNearlyEqual(A.Centroid.Y, B.Centroid.Y, 1.0f)) return A.Centroid.Y < B.Centroid.Y;
			return A.Centroid.Z < B.Centroid.Z;
		});

		for (int32 JIdx = 0; JIdx < SortedClusters.Num(); ++JIdx)
		{
			const int32 JunctionId = JIdx + 1;
			JunctionCentroids.Add(JunctionId, SortedClusters[JIdx].Centroid);

			for (int32 CornerIdx : SortedClusters[JIdx].CornerIndices)
			{
				for (int32 LaneHandle : CornerJunctionData[CornerIdx].StartLaneHandles)
				{
					LaneToJunctionMap.FindOrAdd(LaneHandle) = JunctionId;
				}
			}
		}

		UE_LOG(LogAAATraffic, Log,
			TEXT("RoadBLDTrafficProvider: Grouped %d corners into %d junctions, %d lanes mapped."),
			CornerJunctionData.Num(), SortedClusters.Num(), LaneToJunctionMap.Num());
	}
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

// ---------------------------------------------------------------------------
// BuildLaneAdjacency — shared-edge detection for left/right neighbors
// ---------------------------------------------------------------------------

void URoadBLDTrafficProvider::BuildLaneAdjacency()
{
	LeftNeighborMap.Empty();
	RightNeighborMap.Empty();

	for (const auto& RoadPair : RoadToLaneHandles)
	{
		const TArray<int32>& LaneIds = RoadPair.Value;
		if (LaneIds.Num() < 2) { continue; }

		for (int32 i = 0; i < LaneIds.Num(); ++i)
		{
			UDynamicRoadLane* LaneI = ResolveLane(FTrafficLaneHandle(LaneIds[i]));
			if (!LaneI) { continue; }

			for (int32 j = i + 1; j < LaneIds.Num(); ++j)
			{
				UDynamicRoadLane* LaneJ = ResolveLane(FTrafficLaneHandle(LaneIds[j]));
				if (!LaneJ) { continue; }

				// I's right edge == J's left edge ⇒ J is to the right of I.
				if (LaneI->RightEdgeCurve && LaneJ->LeftEdgeCurve
					&& LaneI->RightEdgeCurve == LaneJ->LeftEdgeCurve)
				{
					RightNeighborMap.Add(LaneIds[i], LaneIds[j]);
					LeftNeighborMap.Add(LaneIds[j], LaneIds[i]);
				}
				// I's left edge == J's right edge ⇒ J is to the left of I.
				else if (LaneI->LeftEdgeCurve && LaneJ->RightEdgeCurve
					&& LaneI->LeftEdgeCurve == LaneJ->RightEdgeCurve)
				{
					LeftNeighborMap.Add(LaneIds[i], LaneIds[j]);
					RightNeighborMap.Add(LaneIds[j], LaneIds[i]);
				}
			}
		}
	}

	UE_LOG(LogAAATraffic, Log,
		TEXT("RoadBLDTrafficProvider: Built lane adjacency — %d left links, %d right links."),
		LeftNeighborMap.Num(), RightNeighborMap.Num());
}

// ---------------------------------------------------------------------------
// ITrafficRoadProvider — adjacency & speed limit
// ---------------------------------------------------------------------------

FTrafficLaneHandle URoadBLDTrafficProvider::GetAdjacentLane(
	const FTrafficLaneHandle& Lane, ETrafficLaneSide Side)
{
	const TMap<int32, int32>& Map = (Side == ETrafficLaneSide::Left) ? LeftNeighborMap : RightNeighborMap;
	if (const int32* NeighborId = Map.Find(Lane.HandleId))
	{
		return FTrafficLaneHandle(*NeighborId);
	}
	return FTrafficLaneHandle();
}

FTrafficRoadHandle URoadBLDTrafficProvider::GetRoadForLane(const FTrafficLaneHandle& Lane)
{
	if (const int32* RoadId = LaneToRoadHandleMap.Find(Lane.HandleId))
	{
		return FTrafficRoadHandle(*RoadId);
	}
	return FTrafficRoadHandle();
}

float URoadBLDTrafficProvider::GetLaneSpeedLimit(const FTrafficLaneHandle& /*Lane*/)
{
	// RoadBLD does not expose per-lane speed limits.
	return -1.0f;
}

int32 URoadBLDTrafficProvider::GetJunctionForLane(const FTrafficLaneHandle& Lane)
{
	if (const int32* JId = LaneToJunctionMap.Find(Lane.HandleId))
	{
		return *JId;
	}
	return 0;
}

bool URoadBLDTrafficProvider::GetJunctionPath(
	const FTrafficLaneHandle& FromLane,
	const FTrafficLaneHandle& ToLane,
	TArray<FVector>& OutPath)
{
	const int32* FromJunction = LaneToJunctionMap.Find(FromLane.HandleId);
	if (!FromJunction || *FromJunction == 0) { return false; }

	const FVector* Centroid = JunctionCentroids.Find(*FromJunction);
	if (!Centroid) { return false; }

	TArray<FVector> FromPoints;
	float FromWidth;
	TArray<FVector> ToPoints;
	float ToWidth;

	if (!GetLanePath(FromLane, FromPoints, FromWidth) || FromPoints.Num() == 0) { return false; }
	if (!GetLanePath(ToLane, ToPoints, ToWidth) || ToPoints.Num() == 0) { return false; }

	OutPath.Reset(3);
	OutPath.Add(FromPoints.Last());
	OutPath.Add(*Centroid);
	OutPath.Add(ToPoints[0]);
	return true;
}

#else // !WITH_ROADBLD — empty stubs so the module compiles without RoadBLD

TArray<FTrafficRoadHandle> URoadBLDTrafficProvider::GetAllRoads() { return {}; }
TArray<FTrafficLaneHandle> URoadBLDTrafficProvider::GetLanesForRoad(const FTrafficRoadHandle&) { return {}; }
bool URoadBLDTrafficProvider::GetLanePath(const FTrafficLaneHandle&, TArray<FVector>&, float&) { return false; }
FVector URoadBLDTrafficProvider::GetLaneDirection(const FTrafficLaneHandle&) { return FVector::ForwardVector; }
TArray<FTrafficLaneHandle> URoadBLDTrafficProvider::GetConnectedLanes(const FTrafficLaneHandle&) { return {}; }
FTrafficLaneHandle URoadBLDTrafficProvider::GetLaneAtLocation(const FVector&) { return {}; }
FTrafficLaneHandle URoadBLDTrafficProvider::GetAdjacentLane(const FTrafficLaneHandle&, ETrafficLaneSide) { return {}; }
FTrafficRoadHandle URoadBLDTrafficProvider::GetRoadForLane(const FTrafficLaneHandle&) { return {}; }
float URoadBLDTrafficProvider::GetLaneSpeedLimit(const FTrafficLaneHandle&) { return -1.0f; }
int32 URoadBLDTrafficProvider::GetJunctionForLane(const FTrafficLaneHandle&) { return 0; }
bool URoadBLDTrafficProvider::GetJunctionPath(const FTrafficLaneHandle&, const FTrafficLaneHandle&, TArray<FVector>&) { return false; }

#endif // WITH_ROADBLD
