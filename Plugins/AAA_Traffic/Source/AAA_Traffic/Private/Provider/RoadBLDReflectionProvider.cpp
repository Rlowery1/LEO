// Copyright AAA_Traffic Contributors. All Rights Reserved.

#include "RoadBLDReflectionProvider.h"
#include "RoadBLDAPIContract.h"
#include "TrafficSubsystem.h"
#include "TrafficLog.h"
#include "Engine/World.h"
#include "EngineUtils.h"
#include "HAL/PlatformTime.h"

// ---------------------------------------------------------------------------
// CVars — tunable proximity connectivity thresholds
// ---------------------------------------------------------------------------

float GProximityThreshold = 500.0f;
float GThroughRoadRadius = 500.0f;
float GMinUTurnWidth = 1100.0f;
float GDirectionDotMin = -0.5f;

static FAutoConsoleVariableRef CVarProximityThreshold(
	TEXT("traffic.ProximityThreshold"),
	GProximityThreshold,
	TEXT("Max distance (cm) between two lane endpoints to consider them connected. Default 500."),
	ECVF_Default);

static FAutoConsoleVariableRef CVarThroughRoadRadius(
	TEXT("traffic.ThroughRoadRadius"),
	GThroughRoadRadius,
	TEXT("Max distance (cm) from a side-road endpoint to a through-road polyline midpoint for split detection. Default 500."),
	ECVF_Default);

static FAutoConsoleVariableRef CVarMinUTurnWidth(
	TEXT("traffic.MinUTurnWidth"),
	GMinUTurnWidth,
	TEXT("Minimum total road width (cm) at junction to allow U-turn connections. Default 1100 (AASHTO standard)."),
	ECVF_Default);

static FAutoConsoleVariableRef CVarDirectionDotMin(
	TEXT("traffic.DirectionDotMin"),
	GDirectionDotMin,
	TEXT("Minimum dot product between lane directions for non-U-turn connections. Default -0.5."),
	ECVF_Default);

bool GEnableDiagnosticDumps = false;
static FAutoConsoleVariableRef CVarEnableDiagnosticDumps(
	TEXT("traffic.EnableDiagnosticDumps"),
	GEnableDiagnosticDumps,
	TEXT("Enable verbose RoadBLD array dumps and corner diagnostics during road discovery. Default false."),
	ECVF_Default);

int32 GTrafficDiagnosticsLevel = 0;
static FAutoConsoleVariableRef CVarTrafficDiagnosticsLevel(
	TEXT("traffic.DiagnosticsLevel"),
	GTrafficDiagnosticsLevel,
	TEXT("Connectivity diagnostics verbosity: 0=off, 1=phase summary, 2=reason counters + invariants, 3=sample traces."),
	ECVF_Default);

int32 GTrafficDiagnosticsSampleLimit = 24;
static FAutoConsoleVariableRef CVarTrafficDiagnosticsSampleLimit(
	TEXT("traffic.DiagnosticsSampleLimit"),
	GTrafficDiagnosticsSampleLimit,
	TEXT("Max sampled diagnostic records per phase when traffic.DiagnosticsLevel >= 3."),
	ECVF_Default);

bool GTrafficDiagnosticsValidateGraph = false;
static FAutoConsoleVariableRef CVarTrafficDiagnosticsValidateGraph(
	TEXT("traffic.DiagnosticsValidateGraph"),
	GTrafficDiagnosticsValidateGraph,
	TEXT("Run post-build graph invariant validation (adjacency symmetry, dangling handles, duplicate edges)."),
	ECVF_Default);

// Shared diagnostic helpers — defined here, extern-declared by sibling
// RoadBLDReflectionProvider_*.cpp files.  Must NOT be in an anonymous
// namespace so that the linker resolves the single definition in unity
// and non-unity builds alike.
bool ShouldLogDiagnostics(const int32 Level)
{
	return GEnableDiagnosticDumps || GTrafficDiagnosticsLevel >= Level;
}

int32 GetDiagnosticsSampleLimit()
{
	return FMath::Max(1, GTrafficDiagnosticsSampleLimit);
}

// ---------------------------------------------------------------------------
// USubsystem overrides
// ---------------------------------------------------------------------------

bool URoadBLDReflectionProvider::ShouldCreateSubsystem(UObject* Outer) const
{
	// Only spin up if RoadBLD classes are actually loaded.
	return FindObject<UClass>(nullptr, TEXT("/Script/RoadBLDRuntime.DynamicRoad")) != nullptr;
}

void URoadBLDReflectionProvider::Initialize(FSubsystemCollectionBase& Collection)
{
	Super::Initialize(Collection);
	NextHandleId = 1;
	bCached = false;
}

void URoadBLDReflectionProvider::Deinitialize()
{
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
	LaneEndpointMap.Empty();
	VirtualLaneMap.Empty();
	OriginalToVirtualMap.Empty();
	ReplacedLaneHandles.Empty();
	ProximityConnectionList.Empty();
	CachedIntersectionMasks.Empty();
	IntersectionGroupCentroids.Empty();
	RoadTotalWidthMap.Empty();
	bCached = false;

	DynRoadClass = nullptr;
	DynNetworkClass = nullptr;
	GetLengthFunc = nullptr;
	GetAllLanesFunc = nullptr;
	ConvertDistFunc = nullptr;
	GetWorldPosFunc = nullptr;
	RefLineProp = nullptr;
	Get3DPosFunc = nullptr;
	LeftEdgeProp = nullptr;
	RightEdgeProp = nullptr;
	LaneWidthProp = nullptr;

	Super::Deinitialize();
}

void URoadBLDReflectionProvider::OnWorldBeginPlay(UWorld& InWorld)
{
	Super::OnWorldBeginPlay(InWorld);
	const bool bDiagPhaseSummary = ShouldLogDiagnostics(1);

	auto RunDiagPhase = [this, bDiagPhaseSummary](const TCHAR* PhaseName, TFunctionRef<void()> Fn)
	{
		const double StartSeconds = FPlatformTime::Seconds();
		Fn();
		if (bDiagPhaseSummary)
		{
			UE_LOG(LogAAATraffic, Log,
				TEXT("RoadBLDReflectionProvider: Phase=%s Time=%.2f ms Roads=%d Lanes=%d LaneConnKeys=%d ProxConns=%d JunctionLanes=%d Junctions=%d VirtualLanes=%d ReplacedLanes=%d"),
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

	UTrafficSubsystem* TrafficSub = InWorld.GetSubsystem<UTrafficSubsystem>();

	// ── API Contract Validation ──────────────────────────────────
	FRoadBLDAPIContractResult ContractResult = FRoadBLDAPIContract::Validate();
	if (!ContractResult.bAllPassed)
	{
		UE_LOG(LogAAATraffic, Error,
			TEXT("RoadBLDReflectionProvider: API contract validation FAILED — %d issue(s):"),
			ContractResult.Failures.Num());
		for (const FRoadBLDContractFailure& Fail : ContractResult.Failures)
		{
			UE_LOG(LogAAATraffic, Error, TEXT("  %s"), *Fail.ToString());
		}
		UE_LOG(LogAAATraffic, Error,
			TEXT("RoadBLDReflectionProvider: Provider will NOT register. The installed RoadBLD version may be incompatible."));
		return;
	}
	UE_LOG(LogAAATraffic, Log, TEXT("RoadBLDReflectionProvider: API contract validation passed."));

	// ── Cache road/lane data ─────────────────────────────────────
	RunDiagPhase(TEXT("CacheRoadData"), [&]() { CacheRoadData(&InWorld); });

	if (!bCached || RoadHandleMap.IsEmpty())
	{
		UE_LOG(LogAAATraffic, Warning,
			TEXT("RoadBLDReflectionProvider: No roads found — provider will NOT register."));
		return;
	}

	// ── Detect reversed lanes on 2-way roads ─────────────────────
	RunDiagPhase(TEXT("DetectReversedLanes"), [&]() { DetectReversedLanes(); });

	// ── Build same-road lane adjacency (left/right neighbors) ────
	RunDiagPhase(TEXT("BuildLaneAdjacency"), [&]() { BuildLaneAdjacency(); });

	// ── Cache lane endpoint geometry ────────────────────────────
	RunDiagPhase(TEXT("CacheLaneEndpoints"), [&]() { CacheLaneEndpoints(); });

	// ── Corner-based connectivity (RoadNetworkCorners fallback) ──
	RunDiagPhase(TEXT("BuildLaneConnectivity"), [&]() { BuildLaneConnectivity(&InWorld); });

	// ── Read IntersectionMasks and group into physical intersections ─
	// (Must run BEFORE through-road splitting so mask data is available.)
	RunDiagPhase(TEXT("BuildMaskBasedIntersections"), [&]() { BuildMaskBasedIntersections(&InWorld); });

	// ── Detect through-roads and create virtual lane splits ──────
	// Roads with 2+ masks are through-roads; split lanes at mask boundaries.
	RunDiagPhase(TEXT("DetectAndSplitThroughRoads"), [&]() { DetectAndSplitThroughRoads(); });

	// ── Proximity-based connectivity ────────────────────────────
	RunDiagPhase(TEXT("BuildProximityConnections"), [&]() { BuildProximityConnections(); });

	// ── Group connections into junctions ────────────────────────
	RunDiagPhase(TEXT("BuildJunctionGrouping"), [&]() { BuildJunctionGrouping(); });

	// ── Precompute full junction map for every lane in the network ──
	// Gives every vehicle O(1) knowledge of the entire road layout.
	RunDiagPhase(TEXT("PrecomputeJunctionMap"), [&]() { PrecomputeJunctionMap(); });

	if (GTrafficDiagnosticsValidateGraph || ShouldLogDiagnostics(2))
	{
		RunConnectivityDiagnostics(TEXT("OnWorldBeginPlay"));
	}

	// ── Register ─────────────────────────────────────────────────
	if (TrafficSub)
	{
		TrafficSub->RegisterProvider(this);
		UE_LOG(LogAAATraffic, Log,
			TEXT("RoadBLDReflectionProvider: Registered — %d roads, %d lanes cached."),
			RoadHandleMap.Num(), LaneHandleMap.Num());
	}
}
TArray<FTrafficRoadHandle> URoadBLDReflectionProvider::GetAllRoads()
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

TArray<FTrafficLaneHandle> URoadBLDReflectionProvider::GetLanesForRoad(const FTrafficRoadHandle& Road)
{
	TArray<FTrafficLaneHandle> Result;

	const TArray<int32>* LaneIds = RoadToLaneHandles.Find(Road.HandleId);
	if (!LaneIds) { return Result; }

	for (const int32 LaneId : *LaneIds)
	{
		// If this lane was split into virtual segments, return those instead.
		if (const TArray<int32>* Virtuals = OriginalToVirtualMap.Find(LaneId))
		{
			for (int32 VId : *Virtuals) { Result.Emplace(VId); }
		}
		else
		{
			Result.Emplace(LaneId);
		}
	}

	// Ensure deterministic ordering consistent with other providers.
	Result.Sort([](const FTrafficLaneHandle& A, const FTrafficLaneHandle& B)
	{
		return A.HandleId < B.HandleId;
	});
	return Result;
}

bool URoadBLDReflectionProvider::GetLanePath(
	const FTrafficLaneHandle& Lane,
	TArray<FVector>& OutPoints,
	float& OutWidth)
{
	// Virtual lane — return the pre-cached polyline for this virtual segment.
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

	// Check cached endpoint polyline first (avoids recomputation).
	if (const FLaneEndpointCache* Cached = LaneEndpointMap.Find(Lane.HandleId))
	{
		if (Cached->Polyline.Num() >= 2)
		{
			OutPoints = Cached->Polyline;
			OutWidth = Cached->Width;
			return true;
		}
	}

	const FReflectionLaneData* Data = LaneHandleMap.Find(Lane.HandleId);
	if (!Data) { return false; }

	UObject* RoadActor = Data->RoadActor.Get();
	if (!RoadActor) { return false; }

	OutWidth = Data->LaneWidth;

	const double RoadLength = GetRoadLength(RoadActor);
	if (RoadLength <= 0.0) { return false; }

	UObject* RefLine = GetReferenceLine(RoadActor);
	UObject* LeftEdge = Data->LeftEdge.Get();
	UObject* RightEdge = Data->RightEdge.Get();

	const double SampleInterval = 100.0; // 1 m
	const int32 NumSamples = FMath::Max(2, FMath::CeilToInt(RoadLength / SampleInterval) + 1);

	OutPoints.Reset(NumSamples);

	const bool bCanDoEdgeSampling = LeftEdge && RightEdge && RefLine
		&& Get3DPosFunc && ConvertDistFunc;

	// ── DIAG: Log once per lane when edge sampling is unavailable ─
	if (!bCanDoEdgeSampling)
	{
		static TSet<int32> LoggedLanes;
		static TWeakObjectPtr<UWorld> LoggedLanesWorld;
		// Clear on new PIE session so diagnostics re-fire for fresh play.
		UWorld* CurrentWorld = GetWorld();
		if (!LoggedLanesWorld.IsValid() || LoggedLanesWorld.Get() != CurrentWorld)
		{
			LoggedLanes.Empty();
			LoggedLanesWorld = CurrentWorld;
		}
		if (!LoggedLanes.Contains(Lane.HandleId))
		{
			LoggedLanes.Add(Lane.HandleId);
			UE_LOG(LogAAATraffic, Warning,
				TEXT("RoadBLDReflectionProvider: GetLanePath FALLBACK for lane %d — "
					 "using road centerline instead of lane edges. "
					 "LeftEdge=%s  RightEdge=%s  RefLine=%s  Get3DPosFunc=%s  ConvertDistFunc=%s"),
				Lane.HandleId,
				LeftEdge        ? TEXT("OK") : TEXT("NULL"),
				RightEdge       ? TEXT("OK") : TEXT("NULL"),
				RefLine         ? TEXT("OK") : TEXT("NULL"),
				Get3DPosFunc    ? TEXT("OK") : TEXT("NULL"),
				ConvertDistFunc ? TEXT("OK") : TEXT("NULL"));
		}
	}

	for (int32 i = 0; i < NumSamples; ++i)
	{
		const double Dist = FMath::Min(RoadLength,
			(RoadLength * static_cast<double>(i)) / static_cast<double>(NumSamples - 1));

		FVector Pos;

		if (bCanDoEdgeSampling)
		{
			const double LeftDist = ConvertDistanceBetweenCurves(RoadActor, RefLine, LeftEdge, Dist);
			const double RightDist = ConvertDistanceBetweenCurves(RoadActor, RefLine, RightEdge, Dist);

			const FVector LeftPos = Get3DPositionAtDistance(LeftEdge, RefLine, LeftDist);
			const FVector RightPos = Get3DPositionAtDistance(RightEdge, RefLine, RightDist);
			Pos = (LeftPos + RightPos) * 0.5;
		}
		else
		{
			// Fallback: 2D centerline from road actor.
			const FVector2D Pos2D = GetWorldPositionAtDistance(RoadActor, Dist);
			Pos = FVector(Pos2D.X, Pos2D.Y, RoadActor->IsA<AActor>()
				? CastChecked<AActor>(RoadActor)->GetActorLocation().Z : 0.0);
		}

		OutPoints.Add(Pos);
	}

	// Reverse points for lanes whose travel direction opposes the reference line (C3).
	if (ReversedLaneSet.Contains(Lane.HandleId))
	{
		Algo::Reverse(OutPoints);
	}

	return OutPoints.Num() >= 2;
}

FVector URoadBLDReflectionProvider::GetLaneDirection(const FTrafficLaneHandle& Lane)
{
	// Return the tangent at the START of the lane (first segment direction),
	// not the start-to-end chord.  The chord loses all curvature information
	// on curved roads — a 180° loop road would return a near-zero vector.
	if (const FLaneEndpointCache* Cached = LaneEndpointMap.Find(Lane.HandleId))
	{
		return Cached->StartDir;
	}
	TArray<FVector> Points;
	float Width;
	if (GetLanePath(Lane, Points, Width) && Points.Num() >= 2)
	{
		return (Points[1] - Points[0]).GetSafeNormal();
	}
	return FVector::ForwardVector;
}

TArray<FTrafficLaneHandle> URoadBLDReflectionProvider::GetConnectedLanes(const FTrafficLaneHandle& Lane)
{
	if (const TArray<FTrafficLaneHandle>* Connected = LaneConnectionMap.Find(Lane.HandleId))
	{
		return *Connected;
	}

	// Virtual-lane fallback: if this is a virtual segment and it has no
	// connections of its own, check whether the original (pre-split) lane
	// still has connections that were not remapped. This is a safety net
	// for edge cases missed by the post-split remapping pass.
	// TODO(P1): Audit DetectAndSplitThroughRoads + BuildJunctionGrouping to
	//           find and fix the root cause so this fallback can be removed.
	if (const FVirtualLaneInfo* VInfo = VirtualLaneMap.Find(Lane.HandleId))
	{
		// Only fall back for the LAST virtual segment of the original lane,
		// since that is the one whose endpoint corresponds to the original
		// lane's endpoint (where cross-road connections originate).
		const TArray<int32>* Siblings = OriginalToVirtualMap.Find(VInfo->OriginalLaneHandle);
		if (Siblings && Siblings->Num() > 0 && (*Siblings).Last() == Lane.HandleId)
		{
			if (const TArray<FTrafficLaneHandle>* OrigConns = LaneConnectionMap.Find(VInfo->OriginalLaneHandle))
			{
				UE_LOG(LogAAATraffic, Warning,
					TEXT("GetConnectedLanes: Virtual lane %d falling back to original %d connections (%d entries). "
						 "This indicates the post-split remapping missed this lane."),
					Lane.HandleId, VInfo->OriginalLaneHandle, OrigConns->Num());
				return *OrigConns;
			}
		}
	}

	return TArray<FTrafficLaneHandle>();
}

FTrafficLaneHandle URoadBLDReflectionProvider::GetLaneAtLocation(const FVector& Location)
{
	// Point-to-polyline distance for accuracy on curved/long lanes.
	float BestDistSq = MAX_flt;
	int32 BestLaneHandle = 0;

	TArray<int32> EndpointKeys;
	LaneEndpointMap.GetKeys(EndpointKeys);
	EndpointKeys.Sort(); // Deterministic iteration (System.md §4.4).

	for (const int32 LaneId : EndpointKeys)
	{
		if (ReplacedLaneHandles.Contains(LaneId)) { continue; }

		const FLaneEndpointCache& Cache = LaneEndpointMap[LaneId];
		// Walk the cached polyline to find closest point on any segment.
		float LaneBestDistSq = MAX_flt;
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
			BestLaneHandle = LaneId;
		}
	}

	return FTrafficLaneHandle(BestLaneHandle);
}
bool URoadBLDReflectionProvider::IsLaneReversed(const FTrafficLaneHandle& Lane)
{
	// Virtual lane — check the original.
	int32 EffectiveId = Lane.HandleId;
	if (const FVirtualLaneInfo* VInfo = VirtualLaneMap.Find(Lane.HandleId))
	{
		EffectiveId = VInfo->OriginalLaneHandle;
	}
	return ReversedLaneSet.Contains(EffectiveId);
}

bool URoadBLDReflectionProvider::GetIntersectionEntryPoint(const FTrafficLaneHandle& ApproachLane, FVector& OutPoint)
{
	if (const FVector* Pt = IntersectionEntryPointMap.Find(ApproachLane.HandleId))
	{
		OutPoint = *Pt;
		return true;
	}
	return false;
}

// ---------------------------------------------------------------------------
// ITrafficRoadProvider — junction queries
// ---------------------------------------------------------------------------
FTrafficLaneHandle URoadBLDReflectionProvider::GetAdjacentLane(
	const FTrafficLaneHandle& Lane, ETrafficLaneSide Side)
{
	// Virtual lane — find the corresponding virtual segment of the neighbor.
	int32 EffectiveId = Lane.HandleId;
	int32 SegmentIndex = -1;
	if (const FVirtualLaneInfo* VInfo = VirtualLaneMap.Find(Lane.HandleId))
	{
		EffectiveId = VInfo->OriginalLaneHandle;
		// Determine which segment index this virtual lane is.
		if (const TArray<int32>* Virtuals = OriginalToVirtualMap.Find(EffectiveId))
		{
			for (int32 i = 0; i < Virtuals->Num(); ++i)
			{
				if ((*Virtuals)[i] == Lane.HandleId) { SegmentIndex = i; break; }
			}
		}
	}

	// FIX: For reversed lanes, the driver faces opposite to the reference
	// line. Reference-line "left" is the driver's physical "right" and
	// vice versa. Swap the lookup so callers get driver-relative adjacency.
	const bool bIsReversedAdj = ReversedLaneSet.Contains(EffectiveId);
	const ETrafficLaneSide EffectiveSide = bIsReversedAdj
		? (Side == ETrafficLaneSide::Left ? ETrafficLaneSide::Right : ETrafficLaneSide::Left)
		: Side;
	const TMap<int32, int32>& Map = (EffectiveSide == ETrafficLaneSide::Left) ? LeftNeighborMap : RightNeighborMap;
	if (const int32* NeighborId = Map.Find(EffectiveId))
	{
		// If the neighbor was also split, return the same-indexed virtual segment.
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
		// Neighbor not split — return as-is.
		return FTrafficLaneHandle(*NeighborId);
	}
	return FTrafficLaneHandle();
}

FTrafficRoadHandle URoadBLDReflectionProvider::GetRoadForLane(const FTrafficLaneHandle& Lane)
{
	// Virtual lane — resolve to original lane's road.
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

float URoadBLDReflectionProvider::GetLaneSpeedLimit(const FTrafficLaneHandle& Lane)
{
	// Look up the lane's road, then return the classified speed limit.
	// Virtual lanes map to their original lane's road via EffectiveId.
	int32 EffectiveId = Lane.HandleId;
	if (const FVirtualLaneInfo* VInfo = VirtualLaneMap.Find(Lane.HandleId))
	{
		EffectiveId = VInfo->OriginalLaneHandle;
	}

	if (const int32* RoadId = LaneToRoadHandleMap.Find(EffectiveId))
	{
		if (const float* Speed = RoadClassifiedSpeedLimits.Find(*RoadId))
		{
			return *Speed;
		}
	}

	// No classification data — caller falls back to its own default.
	return -1.0f;
}
double URoadBLDReflectionProvider::GetRoadLength(UObject* RoadActor) const
{
	if (!RoadActor || !GetLengthFunc) { return 0.0; }

	struct { double ReturnValue; } Params;
	Params.ReturnValue = 0.0;
	RoadActor->ProcessEvent(GetLengthFunc, &Params);
	return Params.ReturnValue;
}

TArray<UObject*> URoadBLDReflectionProvider::GetAllLanesForRoad(UObject* RoadActor) const
{
	if (!RoadActor || !GetAllLanesFunc) { return {}; }

	struct { TArray<UObject*> ReturnValue; } Params;
	RoadActor->ProcessEvent(GetAllLanesFunc, &Params);
	return MoveTemp(Params.ReturnValue);
}

UObject* URoadBLDReflectionProvider::GetReferenceLine(UObject* RoadActor) const
{
	if (!RoadActor || !RefLineProp) { return nullptr; }

	FObjectPropertyBase* ObjProp = CastField<FObjectPropertyBase>(RefLineProp);
	if (!ObjProp) { return nullptr; }

	return ObjProp->GetObjectPropertyValue_InContainer(RoadActor);
}

double URoadBLDReflectionProvider::ConvertDistanceBetweenCurves(
	UObject* RoadActor, UObject* From, UObject* To, double Distance) const
{
	if (!RoadActor || !ConvertDistFunc) { return Distance; }

	uint8* Params = static_cast<uint8*>(FMemory_Alloca(ConvertDistFunc->ParmsSize));
	FMemory::Memzero(Params, ConvertDistFunc->ParmsSize);

	int32 ParamIdx = 0;
	for (TFieldIterator<FProperty> PIt(ConvertDistFunc); PIt && (PIt->PropertyFlags & CPF_Parm); ++PIt)
	{
		if (PIt->HasAnyPropertyFlags(CPF_ReturnParm)) { continue; }
		if (ParamIdx == 0)      { *PIt->ContainerPtrToValuePtr<UObject*>(Params) = From; }
		else if (ParamIdx == 1) { *PIt->ContainerPtrToValuePtr<UObject*>(Params) = To; }
		else if (ParamIdx == 2) { *PIt->ContainerPtrToValuePtr<double>(Params) = Distance; }
		++ParamIdx;
	}

	RoadActor->ProcessEvent(ConvertDistFunc, Params);

	if (FProperty* RetProp = ConvertDistFunc->GetReturnProperty())
	{
		return *RetProp->ContainerPtrToValuePtr<double>(Params);
	}
	return Distance;
}

FVector URoadBLDReflectionProvider::Get3DPositionAtDistance(
	UObject* CurveObj, UObject* RefLine, double Distance) const
{
	if (!CurveObj || !Get3DPosFunc) { return FVector::ZeroVector; }

	uint8* Params = static_cast<uint8*>(FMemory_Alloca(Get3DPosFunc->ParmsSize));
	FMemory::Memzero(Params, Get3DPosFunc->ParmsSize);

	int32 PIdx = 0;
	for (TFieldIterator<FProperty> PIt(Get3DPosFunc); PIt && (PIt->PropertyFlags & CPF_Parm); ++PIt)
	{
		if (PIt->HasAnyPropertyFlags(CPF_ReturnParm)) { continue; }
		if (PIdx == 0)      { *PIt->ContainerPtrToValuePtr<UObject*>(Params) = RefLine; }
		else if (PIdx == 1) { *PIt->ContainerPtrToValuePtr<double>(Params) = Distance; }
		++PIdx;
	}

	CurveObj->ProcessEvent(Get3DPosFunc, Params);

	if (FProperty* RetProp = Get3DPosFunc->GetReturnProperty())
	{
		return *RetProp->ContainerPtrToValuePtr<FVector>(Params);
	}
	return FVector::ZeroVector;
}

FVector2D URoadBLDReflectionProvider::GetWorldPositionAtDistance(
	UObject* RoadActor, double Distance) const
{
	if (!RoadActor || !GetWorldPosFunc) { return FVector2D::ZeroVector; }

	struct { double Distance; FVector2D ReturnValue; } Params;
	Params.Distance = Distance;
	Params.ReturnValue = FVector2D::ZeroVector;
	RoadActor->ProcessEvent(GetWorldPosFunc, &Params);
	return Params.ReturnValue;
}
// ---------------------------------------------------------------------------
// GetLaneLength — O(1) lookup from precomputed table.
// ---------------------------------------------------------------------------
float URoadBLDReflectionProvider::GetLaneLength(const FTrafficLaneHandle& Lane)
{
	const float* Len = PrecomputedLaneLengths.Find(Lane.HandleId);
	if (Len)
	{
		return *Len;
	}

	// Fallback: compute from polyline (shouldn't happen after init).
	const FLaneEndpointCache* Cache = LaneEndpointMap.Find(Lane.HandleId);
	if (!Cache || Cache->Polyline.Num() < 2)
	{
		return 0.0f;
	}
	float Length = 0.0f;
	for (int32 i = 0; i < Cache->Polyline.Num() - 1; ++i)
	{
		Length += FVector::Dist(Cache->Polyline[i], Cache->Polyline[i + 1]);
	}
	return Length;
}

// ---------------------------------------------------------------------------
// GetDistanceToNextJunction — O(1) lookup from precomputed full-network map.
// Vehicles have instant access to the entire road/junction layout.
//
// The precomputed map stores the distance from each lane's END to the next
// junction. The caller passes RemainingDistOnCurrentLane, which is added
// as an offset to give the distance from the vehicle's current position.
//
// MaxSearchDistCm and MaxHops are ignored — the map is already computed
// for the entire network. They remain as parameters for interface compat.
// ---------------------------------------------------------------------------
ITrafficRoadProvider::FJunctionScanResult URoadBLDReflectionProvider::GetDistanceToNextJunction(
	const FTrafficLaneHandle& StartLane,
	float RemainingDistOnCurrentLane,
	float MaxSearchDistCm,
	int32 MaxHops)
{
	if (!StartLane.IsValid())
	{
		return FJunctionScanResult();
	}

	const FJunctionScanResult* Precomputed = PrecomputedJunctionMap.Find(StartLane.HandleId);
	if (!Precomputed)
	{
		return FJunctionScanResult();
	}

	if (!Precomputed->IsValid())
	{
		return *Precomputed; // No junction downstream — returns JunctionId=0
	}

	// The precomputed distance is from the lane's END. The vehicle is at
	// RemainingDistOnCurrentLane from the end, so we need to check:
	// - If the current lane IS the junction (distance = 0), return 0.
	// - Otherwise, the distance from the vehicle = RemainingDist + precomputed.
	FJunctionScanResult Result = *Precomputed;

	if (Result.DistanceCm == 0.0f && Result.JunctionLane == StartLane)
	{
		// Vehicle is ON a junction lane — distance is 0.
		return Result;
	}

	// Precomputed distance is from lane END to junction.
	// Vehicle is RemainingDistOnCurrentLane from lane END.
	// Total distance from vehicle to junction:
	//   = remaining on this lane + accumulated distance through successor lanes
	// BUT: the precomputed AccDist already counts from lane END (0 for the
	// current lane), so we just add the remaining distance.
	Result.DistanceCm = RemainingDistOnCurrentLane + Result.DistanceCm;

	return Result;
}

// ---------------------------------------------------------------------------
// ITrafficRoadProvider — positional direction query
// ---------------------------------------------------------------------------

FVector URoadBLDReflectionProvider::GetLaneDirectionAtDistance(
	const FTrafficLaneHandle& Lane, float Distance)
{
	// Walk the cached polyline to find the segment at the requested distance.
	const FLaneEndpointCache* Cached = LaneEndpointMap.Find(Lane.HandleId);
	if (!Cached || Cached->Polyline.Num() < 2)
	{
		return GetLaneDirection(Lane);
	}

	const TArray<FVector>& Pts = Cached->Polyline;
	float Accumulated = 0.0f;
	for (int32 i = 0; i < Pts.Num() - 1; ++i)
	{
		const float SegLen = FVector::Dist(Pts[i], Pts[i + 1]);
		if (Accumulated + SegLen >= Distance || i == Pts.Num() - 2)
		{
			return (Pts[i + 1] - Pts[i]).GetSafeNormal();
		}
		Accumulated += SegLen;
	}
	return Cached->EndDir;
}

// ---------------------------------------------------------------------------
// ITrafficRoadProvider — positional width query
// ---------------------------------------------------------------------------

float URoadBLDReflectionProvider::GetLaneWidthAtDistance(
	const FTrafficLaneHandle& Lane, float Distance)
{
	// RoadBLD exposes a single width per lane via reflection.
	// Variable-width support would require additional reflection calls
	// to the edge curves.  For now, return the uniform cached width.
	const FLaneEndpointCache* Cached = LaneEndpointMap.Find(Lane.HandleId);
	if (Cached)
	{
		return Cached->Width;
	}
	// Fallback: fetch from lane metadata.
	const FReflectionLaneData* Data = LaneHandleMap.Find(Lane.HandleId);
	return Data ? Data->LaneWidth : 350.0f;
}

// ---------------------------------------------------------------------------
// ITrafficRoadProvider — positional curvature query (Menger curvature)
// ---------------------------------------------------------------------------

float URoadBLDReflectionProvider::GetLaneCurvatureAtDistance(
	const FTrafficLaneHandle& Lane, float Distance)
{
	// Compute Menger curvature κ from 3 polyline points surrounding the
	// requested distance.  κ = 4·Area(triangle) / (|AB|·|BC|·|CA|).
	// Sign: positive = left turn, negative = right turn (Z-up right-hand).
	const FLaneEndpointCache* Cached = LaneEndpointMap.Find(Lane.HandleId);
	if (!Cached || Cached->Polyline.Num() < 3)
	{
		return 0.0f;
	}

	const TArray<FVector>& Pts = Cached->Polyline;

	// Find the index closest to the requested distance.
	float Accumulated = 0.0f;
	int32 MidIdx = 0;
	for (int32 i = 0; i < Pts.Num() - 1; ++i)
	{
		const float SegLen = FVector::Dist(Pts[i], Pts[i + 1]);
		if (Accumulated + SegLen >= Distance)
		{
			MidIdx = i;
			break;
		}
		Accumulated += SegLen;
		MidIdx = i + 1;
	}

	// Widen the point spacing to suppress 100cm sampling noise.
	// Use half CurveScanWindowSize equivalent (≈2-3 points spread).
	constexpr int32 Spread = 2;
	const int32 IdxA = FMath::Max(0, MidIdx - Spread);
	const int32 IdxC = FMath::Min(Pts.Num() - 1, MidIdx + Spread);
	const int32 IdxB = (IdxA + IdxC) / 2;

	if (IdxA == IdxB || IdxB == IdxC)
	{
		return 0.0f;
	}

	const FVector& A = Pts[IdxA];
	const FVector& B = Pts[IdxB];
	const FVector& C = Pts[IdxC];

	const float AB = FVector::Dist2D(A, B);
	const float BC = FVector::Dist2D(B, C);
	const float CA = FVector::Dist2D(C, A);
	const float Denom = AB * BC * CA;

	if (Denom < KINDA_SMALL_NUMBER)
	{
		return 0.0f;
	}

	// Signed area of triangle (2D cross product / 2).
	const float SignedArea2 = (B.X - A.X) * (C.Y - A.Y) - (B.Y - A.Y) * (C.X - A.X);
	return (2.0f * SignedArea2) / Denom;
}

// ---------------------------------------------------------------------------
// ITrafficRoadProvider — arc length from polyline
// ---------------------------------------------------------------------------

float URoadBLDReflectionProvider::GetLaneArcLength(const FTrafficLaneHandle& Lane)
{
	// Polyline arc length — same as GetLaneLength since our polylines are
	// the highest-resolution data we cache (100cm segments).  Native RoadBLD
	// GetCurveLength via reflection could yield slightly different values
	// but would require additional reflection setup.
	return GetLaneLength(Lane);
}