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
#include "HAL/IConsoleManager.h" // IConsoleManager for CVar lookups
#include "HAL/PlatformTime.h"
#endif

#if WITH_ROADBLD
namespace
{
	static int32 GetTrafficDiagnosticsLevel()
	{
		if (IConsoleVariable* CVar = IConsoleManager::Get().FindConsoleVariable(TEXT("traffic.DiagnosticsLevel")))
		{
			return CVar->GetInt();
		}
		return 0;
	}

	static int32 GetTrafficDiagnosticsSampleLimit()
	{
		if (IConsoleVariable* CVar = IConsoleManager::Get().FindConsoleVariable(TEXT("traffic.DiagnosticsSampleLimit")))
		{
			return FMath::Max(1, CVar->GetInt());
		}
		return 24;
	}

	static bool GetTrafficDiagnosticsValidateGraph()
	{
		if (IConsoleVariable* CVar = IConsoleManager::Get().FindConsoleVariable(TEXT("traffic.DiagnosticsValidateGraph")))
		{
			return CVar->GetBool();
		}
		return false;
	}

	static bool ShouldLogDiagnosticsLevel(const int32 Level)
	{
		return GetTrafficDiagnosticsLevel() >= Level;
	}
}
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
	LaneToJunctionMap.Empty();
	JunctionCentroids.Empty();
	LaneEndpointMap.Empty();
	VirtualLaneMap.Empty();
	OriginalToVirtualMap.Empty();
	ReplacedLaneHandles.Empty();
	ProximityConnectionList.Empty();
	RoadTotalWidthMap.Empty();
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
	const bool bDiagPhaseSummary = ShouldLogDiagnosticsLevel(1);

	auto RunDiagPhase = [this, bDiagPhaseSummary](const TCHAR* PhaseName, TFunctionRef<void()> Fn)
	{
		const double StartSeconds = FPlatformTime::Seconds();
		Fn();
		if (bDiagPhaseSummary)
		{
			UE_LOG(LogAAATraffic, Log,
				TEXT("RoadBLDTrafficProvider: Phase=%s Time=%.2f ms Roads=%d Lanes=%d LaneConnKeys=%d ProxConns=%d JunctionLanes=%d Junctions=%d VirtualLanes=%d ReplacedLanes=%d"),
				PhaseName,
				(FPlatformTime::Seconds() - StartSeconds) * 1000.0,
				RoadHandleMap.Num(),
				LaneHandleMap.Num(),
				LaneConnectionMap.Num(),
				ProximityConnectionList.Num(),
				LaneToJunctionMap.Num(),
				JunctionCentroids.Num(),
				VirtualLaneMap.Num(),
				ReplacedLaneHandles.Num());
		}
	};

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

	// Phase ordering: detect reversed lanes and adjacency first (uses original lanes),
	// then cache endpoints, split through-roads, build connectivity, group junctions.
	RunDiagPhase(TEXT("DetectReversedLanes"), [&]() { DetectReversedLanes(); });
	RunDiagPhase(TEXT("BuildLaneAdjacency"), [&]() { BuildLaneAdjacency(); });
	RunDiagPhase(TEXT("CacheLaneEndpoints"), [&]() { CacheLaneEndpoints(); });
	RunDiagPhase(TEXT("DetectAndSplitThroughRoads"), [&]() { DetectAndSplitThroughRoads(); });
	RunDiagPhase(TEXT("BuildLaneConnectivity"), [&]() { BuildLaneConnectivity(); });
	RunDiagPhase(TEXT("BuildProximityConnections"), [&]() { BuildProximityConnections(); });
	RunDiagPhase(TEXT("BuildJunctionGrouping"), [&]() { BuildJunctionGrouping(); });

	if (GetTrafficDiagnosticsValidateGraph() || ShouldLogDiagnosticsLevel(2))
	{
		RunConnectivityDiagnostics(TEXT("CacheRoadData"));
	}
}

void URoadBLDTrafficProvider::BuildLaneConnectivity()
{
	// Note: LaneConnectionMap is NOT cleared here — proximity connections
	// may already be present. Corner-based connections are additive.

	ADynamicRoadNetwork* Network = CachedRoadNetwork.Get();
	if (!Network)
	{
		UE_LOG(LogAAATraffic, Log,
			TEXT("RoadBLDTrafficProvider: No road network found — skipping corner-based connectivity."));
		return;
	}

	const TArray<FRoadNetworkCorner>& Corners = Network->RoadNetworkCorners;
	if (Corners.Num() == 0)
	{
		UE_LOG(LogAAATraffic, Log,
			TEXT("RoadBLDTrafficProvider: RoadNetworkCorners is empty — no corner-based connectivity."));
		return;
	}

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

	int32 CornerConnections = 0;
	for (const FRoadNetworkCorner& Corner : Corners)
	{
		if (Corner.bStale) continue;
		if (!Corner.StartEdge || !Corner.EndEdge) continue;

		TArray<int32> StartHandles, EndHandles;
		CollectLaneHandles(Corner.StartEdge, StartHandles);
		CollectLaneHandles(Corner.EndEdge, EndHandles);

		for (const int32 Src : StartHandles)
		{
			TArray<FTrafficLaneHandle>& Conns = LaneConnectionMap.FindOrAdd(Src);
			for (const int32 Dst : EndHandles)
			{
				if (Src != Dst)
				{
					Conns.AddUnique(FTrafficLaneHandle(Dst));
					++CornerConnections;
				}
			}
		}
	}

	UE_LOG(LogAAATraffic, Log,
		TEXT("RoadBLDTrafficProvider: Corner-based connectivity added %d links from %d corners."),
		CornerConnections, Corners.Num());
}

// ---------------------------------------------------------------------------
// CacheLaneEndpoints — pre-compute geometry for every lane
// ---------------------------------------------------------------------------

void URoadBLDTrafficProvider::CacheLaneEndpoints()
{
	LaneEndpointMap.Empty();

	TArray<int32> SortedHandles;
	LaneHandleMap.GetKeys(SortedHandles);
	SortedHandles.Sort();

	for (const int32 HandleId : SortedHandles)
	{
		TArray<FVector> Points;
		float Width = 0.0f;

		if (!GetLanePath(FTrafficLaneHandle(HandleId), Points, Width) || Points.Num() < 2)
		{
			continue;
		}

		FLaneEndpointCache Cache;
		Cache.Polyline = Points;
		Cache.Width = Width;
		Cache.StartPos = Points[0];
		Cache.EndPos = Points.Last();
		Cache.StartDir = (Points[1] - Points[0]).GetSafeNormal();
		Cache.EndDir = (Points.Last() - Points[Points.Num() - 2]).GetSafeNormal();

		LaneEndpointMap.Add(HandleId, MoveTemp(Cache));
	}

	// Build RoadTotalWidthMap: sum of all lane widths per road.
	RoadTotalWidthMap.Empty();
	for (const int32 HandleId : SortedHandles)
	{
		const FLaneEndpointCache* EP = LaneEndpointMap.Find(HandleId);
		if (!EP) { continue; }
		const int32* Road = LaneToRoadHandleMap.Find(HandleId);
		if (!Road) { continue; }
		float& Total = RoadTotalWidthMap.FindOrAdd(*Road);
		Total += EP->Width;
	}

	UE_LOG(LogAAATraffic, Log,
		TEXT("RoadBLDTrafficProvider: Cached endpoints for %d lanes, road widths for %d roads."),
		LaneEndpointMap.Num(), RoadTotalWidthMap.Num());
}

// ---------------------------------------------------------------------------
// DetectAndSplitThroughRoads — create virtual lane segments
// ---------------------------------------------------------------------------

void URoadBLDTrafficProvider::DetectAndSplitThroughRoads()
{
	VirtualLaneMap.Empty();
	OriginalToVirtualMap.Empty();
	ReplacedLaneHandles.Empty();

	// Read CVars by name (registered in the AAA_Traffic module).
	float SplitRadius = 500.0f;
	if (IConsoleVariable* CVar = IConsoleManager::Get().FindConsoleVariable(TEXT("traffic.ThroughRoadRadius")))
	{
		SplitRadius = CVar->GetFloat();
	}
	const float SplitRadiusSq = SplitRadius * SplitRadius;

	struct FEndpointInfo
	{
		int32 LaneHandle;
		int32 RoadHandle;
		FVector Position;
	};
	TArray<FEndpointInfo> AllEndpoints;

	TArray<int32> SortedRoads;
	RoadToLaneHandles.GetKeys(SortedRoads);
	SortedRoads.Sort();

	for (const int32 RoadId : SortedRoads)
	{
		const TArray<int32>& Lanes = RoadToLaneHandles[RoadId];
		for (const int32 LaneId : Lanes)
		{
			const FLaneEndpointCache* EP = LaneEndpointMap.Find(LaneId);
			if (!EP) { continue; }
			AllEndpoints.Add({ LaneId, RoadId, EP->StartPos });
			AllEndpoints.Add({ LaneId, RoadId, EP->EndPos });
		}
	}

	// Pre-compute AABB per candidate lane for early rejection.
	struct FLaneAABB
	{
		int32 LaneHandle;
		int32 RoadHandle;
		FVector Min;
		FVector Max;
	};
	TArray<FLaneAABB> CandidateBoxes;
	for (const auto& Pair : LaneEndpointMap)
	{
		const int32* CandRoad = LaneToRoadHandleMap.Find(Pair.Key);
		if (!CandRoad) { continue; }
		const TArray<FVector>& Poly = Pair.Value.Polyline;
		if (Poly.Num() < 3) { continue; }

		FVector BoxMin = Poly[0];
		FVector BoxMax = Poly[0];
		for (int32 p = 1; p < Poly.Num(); ++p)
		{
			BoxMin = BoxMin.ComponentMin(Poly[p]);
			BoxMax = BoxMax.ComponentMax(Poly[p]);
		}
		BoxMin -= FVector(SplitRadius);
		BoxMax += FVector(SplitRadius);
		CandidateBoxes.Add({ Pair.Key, *CandRoad, BoxMin, BoxMax });
	}

	TMap<int32, TArray<float>> SplitCandidates;

	for (const FEndpointInfo& EP : AllEndpoints)
	{
		for (const FLaneAABB& Cand : CandidateBoxes)
		{
			const int32 CandidateLane = Cand.LaneHandle;
			if (Cand.RoadHandle == EP.RoadHandle) { continue; }

			// AABB early rejection.
			if (EP.Position.X < Cand.Min.X || EP.Position.X > Cand.Max.X ||
				EP.Position.Y < Cand.Min.Y || EP.Position.Y > Cand.Max.Y ||
				EP.Position.Z < Cand.Min.Z || EP.Position.Z > Cand.Max.Z)
			{
				continue;
			}

			const FLaneEndpointCache& CandCache = LaneEndpointMap[CandidateLane];
			const TArray<FVector>& Poly = CandCache.Polyline;

			float BestDistSq = SplitRadiusSq;
			int32 BestSegIndex = -1;
			float BestSegT = 0.0f;

			for (int32 s = 0; s < Poly.Num() - 1; ++s)
			{
				const FVector& A = Poly[s];
				const FVector& B = Poly[s + 1];
				const FVector AB = B - A;
				const float SegLenSq = AB.SizeSquared();
				if (SegLenSq < 1.0f) { continue; }

				float T = FVector::DotProduct(EP.Position - A, AB) / SegLenSq;
				T = FMath::Clamp(T, 0.0f, 1.0f);
				const FVector ClosestPt = A + AB * T;
				const float DistSq = FVector::DistSquared(EP.Position, ClosestPt);

				if (DistSq < BestDistSq)
				{
					BestDistSq = DistSq;
					BestSegIndex = s;
					BestSegT = T;
				}
			}

			if (BestSegIndex < 0) { continue; }

			const float ParamPos = static_cast<float>(BestSegIndex) + BestSegT;
			const float TotalSegs = static_cast<float>(Poly.Num() - 1);
			const float NormParam = ParamPos / TotalSegs;

			if (NormParam < 0.10f || NormParam > 0.90f) { continue; }

			SplitCandidates.FindOrAdd(CandidateLane).Add(NormParam);
		}
	}

	int32 TotalVirtuals = 0;
	TArray<int32> SortedCandidates;
	SplitCandidates.GetKeys(SortedCandidates);
	SortedCandidates.Sort();

	for (const int32 OriginalLane : SortedCandidates)
	{
		TArray<float>& Params = SplitCandidates[OriginalLane];
		Params.Sort();

		TArray<float> MergedParams;
		for (const float P : Params)
		{
			if (MergedParams.Num() == 0 || (P - MergedParams.Last()) > 0.05f)
			{
				MergedParams.Add(P);
			}
		}

		const FLaneEndpointCache* OrigCache = LaneEndpointMap.Find(OriginalLane);
		if (!OrigCache || OrigCache->Polyline.Num() < 3) { continue; }

		const TArray<FVector>& Poly = OrigCache->Polyline;
		const int32 TotalPoints = Poly.Num();

		TArray<int32> SplitIndices;
		for (const float NP : MergedParams)
		{
			int32 Idx = FMath::RoundToInt32(NP * static_cast<float>(TotalPoints - 1));
			Idx = FMath::Clamp(Idx, 1, TotalPoints - 2);
			if (SplitIndices.Num() == 0 || SplitIndices.Last() != Idx)
			{
				SplitIndices.Add(Idx);
			}
		}

		if (SplitIndices.Num() == 0) { continue; }

		TArray<int32> VirtualHandles;
		int32 PrevStart = 0;

		for (int32 s = 0; s <= SplitIndices.Num(); ++s)
		{
			const int32 SegEnd = (s < SplitIndices.Num()) ? SplitIndices[s] : (TotalPoints - 1);
			if (SegEnd <= PrevStart) { continue; }

			const int32 VirtualId = NextHandleId++;
			FVirtualLaneInfo VInfo;
			VInfo.OriginalLaneHandle = OriginalLane;
			VInfo.StartPointIndex = PrevStart;
			VInfo.EndPointIndex = SegEnd;
			VirtualLaneMap.Add(VirtualId, VInfo);
			VirtualHandles.Add(VirtualId);

			FLaneEndpointCache VCache;
			VCache.Width = OrigCache->Width;
			for (int32 p = PrevStart; p <= SegEnd; ++p)
			{
				VCache.Polyline.Add(Poly[p]);
			}
			VCache.StartPos = VCache.Polyline[0];
			VCache.EndPos = VCache.Polyline.Last();
			if (VCache.Polyline.Num() >= 2)
			{
				VCache.StartDir = (VCache.Polyline[1] - VCache.Polyline[0]).GetSafeNormal();
				VCache.EndDir = (VCache.Polyline.Last() - VCache.Polyline[VCache.Polyline.Num() - 2]).GetSafeNormal();
			}
			LaneEndpointMap.Add(VirtualId, MoveTemp(VCache));

			PrevStart = SegEnd;
		}

		if (VirtualHandles.Num() > 1)
		{
			ReplacedLaneHandles.Add(OriginalLane);
			OriginalToVirtualMap.Add(OriginalLane, MoveTemp(VirtualHandles));
			TotalVirtuals += OriginalToVirtualMap[OriginalLane].Num();

			const TArray<int32>& Virtuals = OriginalToVirtualMap[OriginalLane];
			for (int32 v = 0; v < Virtuals.Num() - 1; ++v)
			{
				TArray<FTrafficLaneHandle>& Conns = LaneConnectionMap.FindOrAdd(Virtuals[v]);
				Conns.AddUnique(FTrafficLaneHandle(Virtuals[v + 1]));
			}
		}
		else
		{
			for (const int32 VH : VirtualHandles)
			{
				VirtualLaneMap.Remove(VH);
				LaneEndpointMap.Remove(VH);
			}
		}
	}

	UE_LOG(LogAAATraffic, Log,
		TEXT("RoadBLDTrafficProvider: Through-road splitting created %d virtual segments from %d source lanes."),
		TotalVirtuals, ReplacedLaneHandles.Num());
}

// ---------------------------------------------------------------------------
// BuildProximityConnections — endpoint matching + U-turn gating
// ---------------------------------------------------------------------------

void URoadBLDTrafficProvider::BuildProximityConnections()
{
	ProximityConnectionList.Empty();

	// Read CVars by name (registered in the AAA_Traffic module).
	float ProxThreshold = 500.0f;
	float DirectionDotMin = -0.5f;
	float MinUTurnWidth = 1100.0f;

	if (IConsoleVariable* CVar = IConsoleManager::Get().FindConsoleVariable(TEXT("traffic.ProximityThreshold")))
	{
		ProxThreshold = CVar->GetFloat();
	}
	if (IConsoleVariable* CVar = IConsoleManager::Get().FindConsoleVariable(TEXT("traffic.DirectionDotMin")))
	{
		DirectionDotMin = CVar->GetFloat();
	}
	if (IConsoleVariable* CVar = IConsoleManager::Get().FindConsoleVariable(TEXT("traffic.MinUTurnWidth")))
	{
		MinUTurnWidth = CVar->GetFloat();
	}

	const float ProxThresholdSq = ProxThreshold * ProxThreshold;

	TArray<int32> WorkingSet;
	for (const auto& Pair : LaneEndpointMap)
	{
		if (!ReplacedLaneHandles.Contains(Pair.Key))
		{
			WorkingSet.Add(Pair.Key);
		}
	}
	WorkingSet.Sort();

	// Precompute effective road handle for each lane in the working set.
	TMap<int32, int32> EffectiveRoadMap;
	EffectiveRoadMap.Reserve(WorkingSet.Num());
	for (const int32 Handle : WorkingSet)
	{
		int32 Road = 0;
		if (const FVirtualLaneInfo* V = VirtualLaneMap.Find(Handle))
		{
			const int32* R = LaneToRoadHandleMap.Find(V->OriginalLaneHandle);
			Road = R ? *R : 0;
		}
		else
		{
			const int32* R = LaneToRoadHandleMap.Find(Handle);
			Road = R ? *R : 0;
		}
		EffectiveRoadMap.Add(Handle, Road);
	}

	int32 ProximityLinks = 0;
	int32 PairComparisons = 0;
	int32 SkipSameRoad = 0;
	int32 SkipDistance = 0;
	int32 ForwardAccepted = 0;
	int32 UTurnCandidates = 0;
	int32 UTurnAccepted = 0;
	int32 UTurnRejectedWidth = 0;
	int32 DuplicateConnectionAttempts = 0;

	TArray<FString> SampledRejects;
	TArray<FString> SampledAccepts;
	const bool bSampleDiagnostics = ShouldLogDiagnosticsLevel(3);
	const int32 MaxSamples = GetTrafficDiagnosticsSampleLimit();

	for (int32 i = 0; i < WorkingSet.Num(); ++i)
	{
		const int32 HandleA = WorkingSet[i];
		const FLaneEndpointCache* CacheA = LaneEndpointMap.Find(HandleA);
		if (!CacheA) { continue; }

		const int32 RoadA = EffectiveRoadMap[HandleA];

		for (int32 j = 0; j < WorkingSet.Num(); ++j)
		{
			if (i == j) { continue; }
			++PairComparisons;

			const int32 HandleB = WorkingSet[j];
			const FLaneEndpointCache* CacheB = LaneEndpointMap.Find(HandleB);
			if (!CacheB) { continue; }

			const int32 RoadB = EffectiveRoadMap[HandleB];

			if (RoadA == RoadB && RoadA != 0)
			{
				++SkipSameRoad;
				continue;
			}

			const float DistSq = FVector::DistSquared(CacheA->EndPos, CacheB->StartPos);
			if (DistSq > ProxThresholdSq)
			{
				++SkipDistance;
				if (bSampleDiagnostics && SampledRejects.Num() < MaxSamples)
				{
					const float Dist = FMath::Sqrt(DistSq);
					if (FMath::Abs(Dist - ProxThreshold) <= 150.0f)
					{
						SampledRejects.Add(FString::Printf(
							TEXT("RejectDistance A=%d B=%d Dist=%.2f Threshold=%.2f RoadA=%d RoadB=%d"),
							HandleA, HandleB, Dist, ProxThreshold, RoadA, RoadB));
					}
				}
				continue;
			}

			const float Dot = FVector::DotProduct(CacheA->EndDir, CacheB->StartDir);

			if (Dot >= DirectionDotMin)
			{
				TArray<FTrafficLaneHandle>& Conns = LaneConnectionMap.FindOrAdd(HandleA);
				const int32 PrevCount = Conns.Num();
				Conns.AddUnique(FTrafficLaneHandle(HandleB));
				if (Conns.Num() == PrevCount)
				{
					++DuplicateConnectionAttempts;
				}
				else
				{
					++ProximityLinks;
					++ForwardAccepted;

					FProximityConnection PC;
					PC.FromLane = HandleA;
					PC.ToLane = HandleB;
					PC.Midpoint = (CacheA->EndPos + CacheB->StartPos) * 0.5f;
					ProximityConnectionList.Add(MoveTemp(PC));

					if (bSampleDiagnostics && SampledAccepts.Num() < MaxSamples)
					{
						SampledAccepts.Add(FString::Printf(
							TEXT("AcceptForward A=%d B=%d Dist=%.2f Dot=%.3f RoadA=%d RoadB=%d"),
							HandleA, HandleB, FMath::Sqrt(DistSq), Dot, RoadA, RoadB));
					}
				}
			}
			else
			{
				++UTurnCandidates;
				// U-turn candidate — gate by total road width (not individual lane width).
				const float* RoadWidthA = RoadTotalWidthMap.Find(RoadA);
				const float* RoadWidthB = RoadTotalWidthMap.Find(RoadB);
				const float WidthA = RoadWidthA ? *RoadWidthA : CacheA->Width;
				const float WidthB = RoadWidthB ? *RoadWidthB : CacheB->Width;
				const bool bWidthOk = (WidthA >= MinUTurnWidth) && (WidthB >= MinUTurnWidth);
				if (bWidthOk)
				{
					TArray<FTrafficLaneHandle>& Conns = LaneConnectionMap.FindOrAdd(HandleA);
					const int32 PrevCount = Conns.Num();
					Conns.AddUnique(FTrafficLaneHandle(HandleB));
					if (Conns.Num() == PrevCount)
					{
						++DuplicateConnectionAttempts;
					}
					else
					{
						++ProximityLinks;
						++UTurnAccepted;

						FProximityConnection PC;
						PC.FromLane = HandleA;
						PC.ToLane = HandleB;
						PC.Midpoint = (CacheA->EndPos + CacheB->StartPos) * 0.5f;
						ProximityConnectionList.Add(MoveTemp(PC));

						if (bSampleDiagnostics && SampledAccepts.Num() < MaxSamples)
						{
							SampledAccepts.Add(FString::Printf(
								TEXT("AcceptUTurn A=%d B=%d Dist=%.2f Dot=%.3f WidthA=%.1f WidthB=%.1f"),
								HandleA, HandleB, FMath::Sqrt(DistSq), Dot, WidthA, WidthB));
						}
					}
				}
				else
				{
					++UTurnRejectedWidth;
					if (bSampleDiagnostics && SampledRejects.Num() < MaxSamples)
					{
						SampledRejects.Add(FString::Printf(
							TEXT("RejectUTurnWidth A=%d B=%d Dist=%.2f Dot=%.3f WidthA=%.1f WidthB=%.1f MinWidth=%.1f"),
							HandleA, HandleB, FMath::Sqrt(DistSq), Dot, WidthA, WidthB, MinUTurnWidth));
					}
				}
			}
		}
	}

	for (auto& Pair : LaneConnectionMap)
	{
		Pair.Value.Sort([](const FTrafficLaneHandle& A, const FTrafficLaneHandle& B)
		{
			return A.HandleId < B.HandleId;
		});
	}

	UE_LOG(LogAAATraffic, Log,
		TEXT("RoadBLDTrafficProvider: Proximity connections added %d links (U-turn gating at %.0f cm)."),
		ProximityLinks, MinUTurnWidth);

	if (ShouldLogDiagnosticsLevel(2))
	{
		UE_LOG(LogAAATraffic, Log,
			TEXT("RoadBLDTrafficProvider: ProximityDiag PairComparisons=%d WorkingSet=%d SkipSameRoad=%d SkipDistance=%d ForwardAccepted=%d UTurnCandidates=%d UTurnAccepted=%d UTurnRejectedWidth=%d DuplicateAttempts=%d"),
			PairComparisons,
			WorkingSet.Num(),
			SkipSameRoad,
			SkipDistance,
			ForwardAccepted,
			UTurnCandidates,
			UTurnAccepted,
			UTurnRejectedWidth,
			DuplicateConnectionAttempts);
	}

	if (bSampleDiagnostics)
	{
		for (const FString& Line : SampledAccepts)
		{
			UE_LOG(LogAAATraffic, Log, TEXT("RoadBLDTrafficProvider: ProximitySample %s"), *Line);
		}
		for (const FString& Line : SampledRejects)
		{
			UE_LOG(LogAAATraffic, Log, TEXT("RoadBLDTrafficProvider: ProximitySample %s"), *Line);
		}
	}
}

void URoadBLDTrafficProvider::RunConnectivityDiagnostics(const TCHAR* PhaseTag) const
{
	if (!(GetTrafficDiagnosticsValidateGraph() || ShouldLogDiagnosticsLevel(2)))
	{
		return;
	}

	int32 MissingSourceLane = 0;
	int32 MissingTargetLane = 0;
	int32 DuplicateEdges = 0;
	int32 SelfEdges = 0;
	int32 AsymmetricAdjacency = 0;
	int32 JunctionWithoutCentroid = 0;

	for (const auto& Pair : LaneConnectionMap)
	{
		const int32 Source = Pair.Key;
		if (!LaneEndpointMap.Contains(Source) && !LaneHandleMap.Contains(Source))
		{
			++MissingSourceLane;
		}

		TSet<int32> SeenTargets;
		for (const FTrafficLaneHandle& TargetHandle : Pair.Value)
		{
			const int32 Target = TargetHandle.HandleId;
			if (Source == Target)
			{
				++SelfEdges;
			}
			if (SeenTargets.Contains(Target))
			{
				++DuplicateEdges;
			}
			else
			{
				SeenTargets.Add(Target);
			}
			if (!LaneEndpointMap.Contains(Target) && !LaneHandleMap.Contains(Target))
			{
				++MissingTargetLane;
			}
		}
	}

	for (const auto& Pair : RightNeighborMap)
	{
		const int32 RightOf = Pair.Key;
		const int32 RightLane = Pair.Value;
		const int32* LeftBack = LeftNeighborMap.Find(RightLane);
		if (!LeftBack || *LeftBack != RightOf)
		{
			++AsymmetricAdjacency;
		}
	}

	for (const auto& Pair : LeftNeighborMap)
	{
		const int32 LeftOf = Pair.Key;
		const int32 LeftLane = Pair.Value;
		const int32* RightBack = RightNeighborMap.Find(LeftLane);
		if (!RightBack || *RightBack != LeftOf)
		{
			++AsymmetricAdjacency;
		}
	}

	for (const auto& Pair : LaneToJunctionMap)
	{
		if (Pair.Value > 0 && !JunctionCentroids.Contains(Pair.Value))
		{
			++JunctionWithoutCentroid;
		}
	}

	UE_LOG(LogAAATraffic, Log,
		TEXT("RoadBLDTrafficProvider: GraphDiag Phase=%s LaneConnKeys=%d ProxConns=%d MissingSource=%d MissingTarget=%d DuplicateEdges=%d SelfEdges=%d AsymmetricAdjacency=%d JunctionWithoutCentroid=%d"),
		PhaseTag,
		LaneConnectionMap.Num(),
		ProximityConnectionList.Num(),
		MissingSourceLane,
		MissingTargetLane,
		DuplicateEdges,
		SelfEdges,
		AsymmetricAdjacency,
		JunctionWithoutCentroid);

	if (GetTrafficDiagnosticsValidateGraph())
	{
		ensureMsgf(MissingSourceLane == 0, TEXT("RoadBLDTrafficProvider: connectivity references missing source lanes."));
		ensureMsgf(MissingTargetLane == 0, TEXT("RoadBLDTrafficProvider: connectivity references missing target lanes."));
		ensureMsgf(SelfEdges == 0, TEXT("RoadBLDTrafficProvider: self-loop connectivity edges detected."));
		ensureMsgf(AsymmetricAdjacency == 0, TEXT("RoadBLDTrafficProvider: left/right adjacency symmetry violation detected."));
		ensureMsgf(JunctionWithoutCentroid == 0, TEXT("RoadBLDTrafficProvider: lane mapped to junction without centroid."));
	}
}

// ---------------------------------------------------------------------------
// BuildJunctionGrouping — cluster proximity connections into junctions
// ---------------------------------------------------------------------------

void URoadBLDTrafficProvider::BuildJunctionGrouping()
{
	if (ProximityConnectionList.Num() == 0)
	{
		// No proximity connections — preserve any corner-based junction IDs that may
		// already exist (from BuildLaneConnectivity). Do NOT clear LaneToJunctionMap.
		UE_LOG(LogAAATraffic, Log,
			TEXT("RoadBLDTrafficProvider: No proximity connections — junction grouping skipped (corner IDs preserved)."));
		return;
	}

	// Rebuild from proximity data — replaces any pre-existing corner-based IDs.
	LaneToJunctionMap.Empty();
	JunctionCentroids.Empty();

	constexpr float JunctionGroupThreshold = 200.0f;
	const float ThresholdSq = JunctionGroupThreshold * JunctionGroupThreshold;

	const int32 N = ProximityConnectionList.Num();

	TArray<int32> Parent;
	Parent.SetNumUninitialized(N);
	for (int32 k = 0; k < N; ++k) { Parent[k] = k; }

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

	for (int32 a = 0; a < N; ++a)
	{
		for (int32 b = a + 1; b < N; ++b)
		{
			if (FVector::DistSquared(ProximityConnectionList[a].Midpoint,
				ProximityConnectionList[b].Midpoint) <= ThresholdSq)
			{
				Union(a, b);
			}
		}
	}

	TMap<int32, TArray<int32>> Clusters;
	for (int32 k = 0; k < N; ++k)
	{
		Clusters.FindOrAdd(Find(k)).Add(k);
	}

	struct FClusterEntry
	{
		FVector Centroid;
		TArray<int32> ConnIndices;
	};
	TArray<FClusterEntry> SortedClusters;

	for (auto& ClusterPair : Clusters)
	{
		FClusterEntry Entry;
		FVector Sum = FVector::ZeroVector;
		for (int32 Idx : ClusterPair.Value)
		{
			Sum += ProximityConnectionList[Idx].Midpoint;
		}
		Entry.Centroid = Sum / static_cast<float>(ClusterPair.Value.Num());
		Entry.ConnIndices = MoveTemp(ClusterPair.Value);
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

		for (int32 ConnIdx : SortedClusters[JIdx].ConnIndices)
		{
			const FProximityConnection& PC = ProximityConnectionList[ConnIdx];
			LaneToJunctionMap.FindOrAdd(PC.FromLane) = JunctionId;
			LaneToJunctionMap.FindOrAdd(PC.ToLane) = JunctionId;
		}
	}

	UE_LOG(LogAAATraffic, Log,
		TEXT("RoadBLDTrafficProvider: Grouped %d connections into %d junctions, %d lanes mapped."),
		N, SortedClusters.Num(), LaneToJunctionMap.Num());
}

void URoadBLDTrafficProvider::DetectReversedLanes()
{
	ReversedLaneSet.Empty();

	for (const auto& RoadPair : RoadToLaneHandles)
	{
		if (RoadPair.Value.Num() < 2) { continue; } // Single-lane — cannot be 2-way.

		ADynamicRoad* Road = ResolveRoad(FTrafficRoadHandle(RoadPair.Key));
		if (!Road || !Road->ReferenceLine) { continue; }

		const double RoadLength = Road->GetLength();
		if (RoadLength < 200.0) { continue; }

		// Sample reference line at midpoint for direction and position.
		const double MidDist = RoadLength * 0.5;
		const double NearDist = FMath::Max(MidDist - 100.0, 0.0);
		const FVector RefMid = Road->ReferenceLine->Get3DPositionAtDistance(Road->ReferenceLine, MidDist);
		const FVector RefNear = Road->ReferenceLine->Get3DPositionAtDistance(Road->ReferenceLine, NearDist);
		const FVector RefDir = (RefMid - RefNear).GetSafeNormal();
		if (RefDir.IsNearlyZero()) { continue; }

		// Classify each lane as left-of-center or right-of-center.
		struct FSideData { int32 LaneId; float Cross; };
		TArray<FSideData> SideInfo;
		int32 LeftCount = 0, RightCount = 0;

		for (int32 LaneId : RoadPair.Value)
		{
			TArray<FVector> Points;
			float Width;
			if (!GetLanePath(FTrafficLaneHandle(LaneId), Points, Width) || Points.Num() < 2) { continue; }
			const FVector LaneMid = Points[Points.Num() / 2];
			const float Cross = FVector::CrossProduct(RefDir, (LaneMid - RefMid)).Z;
			SideInfo.Add({ LaneId, Cross });
			if (Cross > 50.0f) { ++LeftCount; }
			else if (Cross < -50.0f) { ++RightCount; }
		}

		// Only mark reversed on 2-way roads (lanes on both sides).
		if (LeftCount == 0 || RightCount == 0) { continue; }

		for (const FSideData& S : SideInfo)
		{
			if (S.Cross > 50.0f) { ReversedLaneSet.Add(S.LaneId); }
		}
	}

	UE_LOG(LogAAATraffic, Log,
		TEXT("RoadBLDTrafficProvider: Detected %d reversed lanes on 2-way roads."),
		ReversedLaneSet.Num());
}

bool URoadBLDTrafficProvider::IsLaneReversed(const FTrafficLaneHandle& Lane)
{
	int32 EffectiveId = Lane.HandleId;
	if (const FVirtualLaneInfo* VInfo = VirtualLaneMap.Find(Lane.HandleId))
	{
		EffectiveId = VInfo->OriginalLaneHandle;
	}
	return ReversedLaneSet.Contains(EffectiveId);
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

	const TArray<int32>* LaneIds = RoadToLaneHandles.Find(Road.HandleId);
	if (!LaneIds) return Result;

	for (const int32 LaneId : *LaneIds)
	{
		// If the lane was split, expose virtual segments instead.
		if (const TArray<int32>* Virtuals = OriginalToVirtualMap.Find(LaneId))
		{
			for (const int32 VId : *Virtuals)
			{
				Result.Emplace(VId);
			}
		}
		else
		{
			Result.Emplace(LaneId);
		}
	}

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
	// Virtual lane — return the cached polyline subset.
	if (const FVirtualLaneInfo* VInfo = VirtualLaneMap.Find(Lane.HandleId))
	{
		if (const FLaneEndpointCache* VCache = LaneEndpointMap.Find(Lane.HandleId))
		{
			OutPoints = VCache->Polyline;
			OutWidth = VCache->Width;
			return OutPoints.Num() >= 2;
		}
		return false;
	}

	// Cached endpoint data available — return it directly.
	if (const FLaneEndpointCache* Cache = LaneEndpointMap.Find(Lane.HandleId))
	{
		OutPoints = Cache->Polyline;
		OutWidth = Cache->Width;
		return OutPoints.Num() >= 2;
	}

	// Fallback: compute from RoadBLD API (only used before CacheLaneEndpoints runs).
	UDynamicRoadLane* RoadLane = ResolveLane(Lane);
	if (!RoadLane) return false;

	OutWidth = static_cast<float>(RoadLane->LaneWidth);

	UEdgeCurve* LeftEdge = RoadLane->LeftEdgeCurve;
	UEdgeCurve* RightEdge = RoadLane->RightEdgeCurve;

	ADynamicRoad* Road = GetRoadForLane(RoadLane);
	if (!Road) return false;

	const double RoadLength = Road->GetLength();
	if (RoadLength <= 0.0) return false;

	const double SampleInterval = 100.0;
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
			const double LeftDist = Road->ConvertDistanceBetweenCurves(
				Road->ReferenceLine, LeftEdge, Distance);
			const double RightDist = Road->ConvertDistanceBetweenCurves(
				Road->ReferenceLine, RightEdge, Distance);

			const FVector LeftPos = LeftEdge->Get3DPositionAtDistance(
				Road->ReferenceLine, LeftDist);
			const FVector RightPos = RightEdge->Get3DPositionAtDistance(
				Road->ReferenceLine, RightDist);

			Pos = (LeftPos + RightPos) * 0.5;
		}
		else if (Road->ReferenceLine)
		{
			Pos = Road->ReferenceLine->Get3DPositionAtDistance(
				Road->ReferenceLine, Distance);
		}
		else
		{
			const FVector2D Pos2D = Road->GetWorldPositionAtDistance(Distance);
			Pos = FVector(Pos2D.X, Pos2D.Y, Road->GetActorLocation().Z);
		}

		OutPoints.Add(Pos);
	}

	if (ReversedLaneSet.Contains(Lane.HandleId))
	{
		Algo::Reverse(OutPoints);
	}

	return true;
}

FVector URoadBLDTrafficProvider::GetLaneDirection(const FTrafficLaneHandle& Lane)
{
	if (const FLaneEndpointCache* Cache = LaneEndpointMap.Find(Lane.HandleId))
	{
		return (Cache->EndPos - Cache->StartPos).GetSafeNormal();
	}
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
	// Point-to-polyline distance for accuracy on curved/long lanes.
	int32 BestHandle = 0;
	float BestDistSq = TNumericLimits<float>::Max();

	// Sort keys for deterministic iteration (System.md §4.4).
	TArray<int32> EndpointKeys;
	LaneEndpointMap.GetKeys(EndpointKeys);
	EndpointKeys.Sort();

	for (const int32 LaneId : EndpointKeys)
	{
		if (ReplacedLaneHandles.Contains(LaneId)) { continue; }

		const FLaneEndpointCache& Cache = LaneEndpointMap[LaneId];
		// Walk the cached polyline to find closest point on any segment.
		float LaneBestDistSq = TNumericLimits<float>::Max();
		for (int32 s = 0; s < Cache.Polyline.Num() - 1; ++s)
		{
			const FVector& A = Cache.Polyline[s];
			const FVector& B = Cache.Polyline[s + 1];
			const FVector AB = B - A;
			const float SegLenSq = AB.SizeSquared();

			float DistSq;
			if (SegLenSq <= KINDA_SMALL_NUMBER)
			{
				DistSq = FVector::DistSquared(Location, A);
			}
			else
			{
				const float T = FMath::Clamp(FVector::DotProduct(Location - A, AB) / SegLenSq, 0.0f, 1.0f);
				DistSq = FVector::DistSquared(Location, A + AB * T);
			}

			if (DistSq < LaneBestDistSq) { LaneBestDistSq = DistSq; }
		}

		if (LaneBestDistSq < BestDistSq)
		{
			BestDistSq = LaneBestDistSq;
			BestHandle = LaneId;
		}
	}

	return FTrafficLaneHandle(BestHandle);
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
	int32 EffectiveId = Lane.HandleId;
	int32 SegmentIndex = -1;
	if (const FVirtualLaneInfo* VInfo = VirtualLaneMap.Find(Lane.HandleId))
	{
		EffectiveId = VInfo->OriginalLaneHandle;
		if (const TArray<int32>* Virtuals = OriginalToVirtualMap.Find(EffectiveId))
		{
			for (int32 i = 0; i < Virtuals->Num(); ++i)
			{
				if ((*Virtuals)[i] == Lane.HandleId) { SegmentIndex = i; break; }
			}
		}
	}

	const TMap<int32, int32>& Map = (Side == ETrafficLaneSide::Left) ? LeftNeighborMap : RightNeighborMap;
	if (const int32* NeighborId = Map.Find(EffectiveId))
	{
		if (SegmentIndex >= 0)
		{
			if (const TArray<int32>* NVirtuals = OriginalToVirtualMap.Find(*NeighborId))
			{
				if (SegmentIndex < NVirtuals->Num())
				{
					return FTrafficLaneHandle((*NVirtuals)[SegmentIndex]);
				}
			}
		}
		return FTrafficLaneHandle(*NeighborId);
	}
	return FTrafficLaneHandle();
}

FTrafficRoadHandle URoadBLDTrafficProvider::GetRoadForLane(const FTrafficLaneHandle& Lane)
{
	int32 EffectiveId = Lane.HandleId;
	if (const FVirtualLaneInfo* VInfo = VirtualLaneMap.Find(Lane.HandleId))
	{
		EffectiveId = VInfo->OriginalLaneHandle;
	}
	if (const int32* RoadId = LaneToRoadHandleMap.Find(EffectiveId))
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
	if (const FVirtualLaneInfo* VInfo = VirtualLaneMap.Find(Lane.HandleId))
	{
		if (const int32* JId = LaneToJunctionMap.Find(VInfo->OriginalLaneHandle))
		{
			return *JId;
		}
	}
	return 0;
}

bool URoadBLDTrafficProvider::GetJunctionPath(
	const FTrafficLaneHandle& FromLane,
	const FTrafficLaneHandle& ToLane,
	TArray<FVector>& OutPath)
{
	const int32 FromJunctionId = GetJunctionForLane(FromLane);
	if (FromJunctionId == 0) { return false; }

	const int32 ToJunctionId = GetJunctionForLane(ToLane);
	if (ToJunctionId != FromJunctionId) { return false; }

	const FVector* Centroid = JunctionCentroids.Find(FromJunctionId);
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
bool URoadBLDTrafficProvider::IsLaneReversed(const FTrafficLaneHandle&) { return false; }

#endif // WITH_ROADBLD
