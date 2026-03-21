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

static float GProximityThreshold = 500.0f;
static float GThroughRoadRadius = 500.0f;
static float GMinUTurnWidth = 1100.0f;
static float GDirectionDotMin = -0.5f;

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

static bool GEnableDiagnosticDumps = false;
static FAutoConsoleVariableRef CVarEnableDiagnosticDumps(
	TEXT("traffic.EnableDiagnosticDumps"),
	GEnableDiagnosticDumps,
	TEXT("Enable verbose RoadBLD array dumps and corner diagnostics during road discovery. Default false."),
	ECVF_Default);

static int32 GTrafficDiagnosticsLevel = 0;
static FAutoConsoleVariableRef CVarTrafficDiagnosticsLevel(
	TEXT("traffic.DiagnosticsLevel"),
	GTrafficDiagnosticsLevel,
	TEXT("Connectivity diagnostics verbosity: 0=off, 1=phase summary, 2=reason counters + invariants, 3=sample traces."),
	ECVF_Default);

static int32 GTrafficDiagnosticsSampleLimit = 24;
static FAutoConsoleVariableRef CVarTrafficDiagnosticsSampleLimit(
	TEXT("traffic.DiagnosticsSampleLimit"),
	GTrafficDiagnosticsSampleLimit,
	TEXT("Max sampled diagnostic records per phase when traffic.DiagnosticsLevel >= 3."),
	ECVF_Default);

static bool GTrafficDiagnosticsValidateGraph = false;
static FAutoConsoleVariableRef CVarTrafficDiagnosticsValidateGraph(
	TEXT("traffic.DiagnosticsValidateGraph"),
	GTrafficDiagnosticsValidateGraph,
	TEXT("Run post-build graph invariant validation (adjacency symmetry, dangling handles, duplicate edges)."),
	ECVF_Default);

namespace
{
	static bool ShouldLogDiagnostics(const int32 Level)
	{
		return GEnableDiagnosticDumps || GTrafficDiagnosticsLevel >= Level;
	}

	static int32 GetDiagnosticsSampleLimit()
	{
		return FMath::Max(1, GTrafficDiagnosticsSampleLimit);
	}
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

// ---------------------------------------------------------------------------
// ITrafficRoadProvider
// ---------------------------------------------------------------------------

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

// ---------------------------------------------------------------------------
// CacheRoadData — discover roads and lanes via reflection
// ---------------------------------------------------------------------------

void URoadBLDReflectionProvider::CacheRoadData(UWorld* World)
{
	if (bCached) { return; }

	// ── Resolve class pointers ───────────────────────────────────
	DynRoadClass = FindObject<UClass>(nullptr, TEXT("/Script/RoadBLDRuntime.DynamicRoad"));
	DynNetworkClass = FindObject<UClass>(nullptr, TEXT("/Script/RoadBLDRuntime.DynamicRoadNetwork"));

	if (!DynRoadClass)
	{
		UE_LOG(LogAAATraffic, Warning,
			TEXT("RoadBLDReflectionProvider: DynamicRoad class not found."));
		return;
	}

	// ── Resolve function/property pointers ───────────────────────
	GetLengthFunc   = DynRoadClass->FindFunctionByName(TEXT("GetLength"));
	GetAllLanesFunc = DynRoadClass->FindFunctionByName(TEXT("GetAllLanes"));
	ConvertDistFunc = DynRoadClass->FindFunctionByName(TEXT("ConvertDistanceBetweenCurves"));
	GetWorldPosFunc = DynRoadClass->FindFunctionByName(TEXT("GetWorldPositionAtDistance"));
	RefLineProp     = DynRoadClass->FindPropertyByName(TEXT("ReferenceLine"));

	if (!GetLengthFunc || !GetAllLanesFunc)
	{
		UE_LOG(LogAAATraffic, Warning,
			TEXT("RoadBLDReflectionProvider: Missing GetLength or GetAllLanes on DynamicRoad."));
		return;
	}

	// ── Iterate all DynamicRoad actors ───────────────────────────
	TArray<AActor*> RoadActors;
	for (FActorIterator It(World); It; ++It)
	{
		if (It->IsA(DynRoadClass))
		{
			RoadActors.Add(*It);
		}
	}
	// Deterministic ordering (System.md §4.4).
	RoadActors.Sort([](const AActor& A, const AActor& B) { return A.GetName() < B.GetName(); });

	UE_LOG(LogAAATraffic, Log,
		TEXT("RoadBLDReflectionProvider: Found %d DynamicRoad actors in world."),
		RoadActors.Num());

	for (AActor* RoadActor : RoadActors)
	{
		const double RoadLength = GetRoadLength(RoadActor);
		if (RoadLength <= 0.0)
		{
			UE_LOG(LogAAATraffic, Log,
				TEXT("RoadBLDReflectionProvider: Skipping road '%s' — Length=%.1f <= 0."),
				*RoadActor->GetName(), RoadLength);
			continue;
		}

		const int32 RoadId = NextHandleId++;
		RoadHandleMap.Add(RoadId, RoadActor);

		TArray<UObject*> Lanes = GetAllLanesForRoad(RoadActor);
		// Filter null entries before sorting — GetAllLanes() can return nulls.
		Lanes.RemoveAll([](const UObject* Obj) { return Obj == nullptr; });
		// Sort lanes by name for deterministic handle ordering.
		Lanes.Sort([](const UObject& A, const UObject& B)
		{
			return A.GetName() < B.GetName();
		});

		// ── Filter lanes by ELaneType — only keep drivable types ─
		{
			// Resolve the ELaneType UEnum once for name-based comparison.
			UEnum* LaneTypeEnum = FindObject<UEnum>(nullptr, TEXT("/Script/RoadBLDRuntime.ELaneType"));

			const int32 PreFilterCount = Lanes.Num();
			Lanes.RemoveAll([LaneTypeEnum](const UObject* LaneObj) -> bool
			{
				if (!LaneObj) { return true; }

				// Read LaneSections array from the lane object.
				FArrayProperty* SectionsProp = CastField<FArrayProperty>(
					LaneObj->GetClass()->FindPropertyByName(TEXT("LaneSections")));
				if (!SectionsProp) { return false; } // Can't determine — keep it.

				FScriptArrayHelper Sections(SectionsProp,
					SectionsProp->ContainerPtrToValuePtr<void>(LaneObj));
				if (Sections.Num() == 0) { return false; } // No sections — keep.

				// Read LaneType enum from the first section.
				UScriptStruct* SectionStruct = CastField<FStructProperty>(SectionsProp->Inner)
					? CastField<FStructProperty>(SectionsProp->Inner)->Struct : nullptr;
				if (!SectionStruct) { return false; }

				FProperty* TypeProp = SectionStruct->FindPropertyByName(TEXT("LaneType"));
				if (!TypeProp) { return false; }

				// Read the raw enum value.
				const uint8* ElemPtr = Sections.GetRawPtr(0);
				int64 EnumVal = 0;
				if (FEnumProperty* EP = CastField<FEnumProperty>(TypeProp))
				{
					FNumericProperty* Under = EP->GetUnderlyingProperty();
					EnumVal = Under->GetSignedIntPropertyValue(
						TypeProp->ContainerPtrToValuePtr<void>(ElemPtr));
				}
				else if (FByteProperty* BP = CastField<FByteProperty>(TypeProp))
				{
					EnumVal = BP->GetPropertyValue(
						TypeProp->ContainerPtrToValuePtr<void>(ElemPtr));
				}

				// Resolve the enum value name for robust comparison.
				FString TypeName;
				if (LaneTypeEnum)
				{
					TypeName = LaneTypeEnum->GetNameStringByValue(EnumVal);
				}

				UE_LOG(LogAAATraffic, Verbose,
					TEXT("RoadBLDReflectionProvider: Lane '%s' LaneType=%s (%lld)"),
					*LaneObj->GetName(), *TypeName, EnumVal);

				// Keep only types a vehicle can drive on.
				const bool bDrivable =
					TypeName == TEXT("Normal") ||
					TypeName == TEXT("CenterTurn") ||
					TypeName == TEXT("Restricted");
				return !bDrivable;
			});

			UE_LOG(LogAAATraffic, Log,
				TEXT("RoadBLDReflectionProvider: Road '%s' — %d total lanes, %d drivable after ELaneType filter."),
				*RoadActor->GetName(), PreFilterCount, Lanes.Num());
		}

		// ── Resolve lane-class properties once (first valid lane) ─
		if (Lanes.Num() > 0 && !LeftEdgeProp && Lanes[0])
		{
			UClass* LaneClass = Lanes[0]->GetClass();

			// Validate lane contract.
			FRoadBLDAPIContractResult LaneResult = FRoadBLDAPIContract::ValidateLaneClass(LaneClass);
			if (!LaneResult.bAllPassed)
			{
				UE_LOG(LogAAATraffic, Warning,
					TEXT("RoadBLDReflectionProvider: Lane class contract issues:"));
				for (const FRoadBLDContractFailure& F : LaneResult.Failures)
				{
					UE_LOG(LogAAATraffic, Warning, TEXT("  %s"), *F.ToString());
				}
			}

			LeftEdgeProp  = LaneClass->FindPropertyByName(TEXT("LeftEdgeCurve"));
			RightEdgeProp = LaneClass->FindPropertyByName(TEXT("RightEdgeCurve"));
			LaneWidthProp = LaneClass->FindPropertyByName(TEXT("LaneWidth"));

			// Resolve Get3DPositionAtDistance from the first non-null edge curve.
			if (LeftEdgeProp)
			{
				FObjectPropertyBase* ObjProp = CastField<FObjectPropertyBase>(LeftEdgeProp);
				if (ObjProp)
				{
					for (UObject* L : Lanes)
					{
						if (!L) { continue; }
						UObject* Edge = ObjProp->GetObjectPropertyValue_InContainer(L);
						if (Edge)
						{
							// Validate curve contract.
							FRoadBLDAPIContractResult CurveResult = FRoadBLDAPIContract::ValidateCurveClass(Edge->GetClass());
							if (!CurveResult.bAllPassed)
							{
								UE_LOG(LogAAATraffic, Warning,
									TEXT("RoadBLDReflectionProvider: Curve class contract issues:"));
								for (const FRoadBLDContractFailure& F : CurveResult.Failures)
								{
									UE_LOG(LogAAATraffic, Warning, TEXT("  %s"), *F.ToString());
								}
							}

							Get3DPosFunc = Edge->GetClass()->FindFunctionByName(TEXT("Get3DPositionAtDistance"));
							break;
						}
					}
				}
			}

			// ── DIAG: Log class-level function resolution result ─────
			UE_LOG(LogAAATraffic, Log,
				TEXT("RoadBLDReflectionProvider: Function resolution — "
					 "Get3DPosFunc=%s  ConvertDistFunc=%s  LeftEdgeProp=%s  RightEdgeProp=%s"),
				Get3DPosFunc   ? TEXT("OK") : TEXT("NULL"),
				ConvertDistFunc ? TEXT("OK") : TEXT("NULL"),
				LeftEdgeProp   ? TEXT("OK") : TEXT("NULL"),
				RightEdgeProp  ? TEXT("OK") : TEXT("NULL"));
		}

		// ── BUG-5 FIX: Retry Get3DPosFunc on subsequent roads ────
		// The property resolution block above is one-shot (guarded by !LeftEdgeProp).
		// If the first road's lanes all had null edge curves, Get3DPosFunc stays null.
		// This block retries on every road until the function is found.
		if (!Get3DPosFunc && LeftEdgeProp && Lanes.Num() > 0)
		{
			FObjectPropertyBase* ObjProp = CastField<FObjectPropertyBase>(LeftEdgeProp);
			if (ObjProp)
			{
				for (UObject* L : Lanes)
				{
					if (!L) { continue; }
					UObject* Edge = ObjProp->GetObjectPropertyValue_InContainer(L);
					if (Edge)
					{
						Get3DPosFunc = Edge->GetClass()->FindFunctionByName(TEXT("Get3DPositionAtDistance"));
						if (Get3DPosFunc)
						{
							UE_LOG(LogAAATraffic, Log,
								TEXT("RoadBLDReflectionProvider: Get3DPosFunc resolved on road '%s' (deferred resolution)."),
								*RoadActor->GetName());
							break;
						}
					}
				}
			}
		}

		// ── Create lane handles ──────────────────────────────────
		TArray<int32>& RoadLaneIds = RoadToLaneHandles.Add(RoadId);

		for (UObject* LaneObj : Lanes)
		{
			if (!LaneObj) { continue; }

			const int32 LaneId = NextHandleId++;
			RoadLaneIds.Add(LaneId);
			LaneToRoadHandleMap.Add(LaneId, RoadId);

			FReflectionLaneData& LData = LaneHandleMap.Add(LaneId);
			LData.LaneObject = LaneObj;
			LData.RoadActor = RoadActor;

			// Read LaneWidth.
			if (LaneWidthProp)
			{
				if (FDoubleProperty* DblProp = CastField<FDoubleProperty>(LaneWidthProp))
				{
					LData.LaneWidth = static_cast<float>(DblProp->GetPropertyValue_InContainer(LaneObj));
				}
				else if (FFloatProperty* FltProp = CastField<FFloatProperty>(LaneWidthProp))
				{
					LData.LaneWidth = FltProp->GetPropertyValue_InContainer(LaneObj);
				}
			}

			// Read edge curves.
			if (LeftEdgeProp)
			{
				FObjectPropertyBase* ObjP = CastField<FObjectPropertyBase>(LeftEdgeProp);
				if (ObjP) { LData.LeftEdge = ObjP->GetObjectPropertyValue_InContainer(LaneObj); }
			}
			if (RightEdgeProp)
			{
				FObjectPropertyBase* ObjP = CastField<FObjectPropertyBase>(RightEdgeProp);
				if (ObjP) { LData.RightEdge = ObjP->GetObjectPropertyValue_InContainer(LaneObj); }
			}

			// ── DIAG: Warn when a drivable lane has null edge curves ─
			if (!LData.LeftEdge.Get() || !LData.RightEdge.Get())
			{
				UE_LOG(LogAAATraffic, Warning,
					TEXT("RoadBLDReflectionProvider: Lane %d ('%s') on road '%s' "
						 "has NULL edge curves — LeftEdge=%s  RightEdge=%s. "
						 "Lane will fall back to road-center polyline."),
					LaneId,
					*LaneObj->GetName(),
					*RoadActor->GetName(),
					LData.LeftEdge.Get()  ? TEXT("OK") : TEXT("NULL"),
					LData.RightEdge.Get() ? TEXT("OK") : TEXT("NULL"));
			}

			LaneToHandleMap.Add(LaneObj, LaneId);
		}
	}

	bCached = true;

	// ── DIAG: Post-cache edge sampling summary ──────────────────
	{
		int32 TotalLanes = 0;
		int32 LanesWithBothEdges = 0;
		int32 LanesWithMissingEdges = 0;
		for (const auto& Pair : LaneHandleMap)
		{
			++TotalLanes;
			if (Pair.Value.LeftEdge.Get() && Pair.Value.RightEdge.Get())
			{
				++LanesWithBothEdges;
			}
			else
			{
				++LanesWithMissingEdges;
			}
		}
		UE_LOG(LogAAATraffic, Log,
			TEXT("RoadBLDReflectionProvider: Cached %d roads, %d lanes. "
				 "EdgeSampling: %d/%d lanes have both edges, %d missing. "
				 "Get3DPosFunc=%s  ConvertDistFunc=%s"),
			RoadHandleMap.Num(), TotalLanes,
			LanesWithBothEdges, TotalLanes, LanesWithMissingEdges,
			Get3DPosFunc    ? TEXT("OK") : TEXT("NULL"),
			ConvertDistFunc ? TEXT("OK") : TEXT("NULL"));

		if (LanesWithMissingEdges > 0 || !Get3DPosFunc || !ConvertDistFunc)
		{
			UE_LOG(LogAAATraffic, Warning,
				TEXT("RoadBLDReflectionProvider: *** Edge sampling will FAIL for %d lane(s). "
					 "Vehicles on those lanes will drive on the road centerline. ***"),
				(!Get3DPosFunc || !ConvertDistFunc) ? TotalLanes : LanesWithMissingEdges);
		}
	}
}

// ---------------------------------------------------------------------------
// TriggerRoadBLDRebuildAndDiagnostics — optionally dump all RoadBLD arrays
// before and after an incremental rebuild. Separated from BuildLaneConnectivity
// to keep connectivity logic focused on corner processing.
// ---------------------------------------------------------------------------

void URoadBLDReflectionProvider::TriggerRoadBLDRebuildAndDiagnostics(UWorld* World, AActor* NetworkActor)
{
	check(NetworkActor);

	// ── Dump all RoadBLD intersection arrays BEFORE rebuild ─────
	// Gated behind traffic.EnableDiagnosticDumps CVar since this performs
	// extensive reflection queries and string formatting for every array.
	if (GEnableDiagnosticDumps)
	{
		UE_LOG(LogAAATraffic, Log, TEXT("===== PRE-REBUILD ROADBLD INTERSECTION DATA DUMP ====="));

		// 1) RoadNetworkCorners on the network actor
		{
			FArrayProperty* CornersArr = CastField<FArrayProperty>(DynNetworkClass->FindPropertyByName(TEXT("RoadNetworkCorners")));
			int32 N = 0;
			if (CornersArr) { FScriptArrayHelper H(CornersArr, CornersArr->ContainerPtrToValuePtr<void>(NetworkActor)); N = H.Num(); }
			UE_LOG(LogAAATraffic, Log, TEXT("  RoadNetworkCorners: %d entries"), N);
		}

		// 2) CornerEditData on the network actor
		{
			FArrayProperty* EditArr = CastField<FArrayProperty>(DynNetworkClass->FindPropertyByName(TEXT("CornerEditData")));
			int32 N = 0;
			if (EditArr)
			{
				FScriptArrayHelper H(EditArr, EditArr->ContainerPtrToValuePtr<void>(NetworkActor));
				N = H.Num();
				FStructProperty* SP = CastField<FStructProperty>(EditArr->Inner);
				if (SP && N > 0)
				{
					UScriptStruct* S = SP->Struct;
					FProperty* SDProp = S->FindPropertyByName(TEXT("StartDistance"));
					FProperty* EDProp = S->FindPropertyByName(TEXT("EndDistance"));
					FProperty* SEProp = S->FindPropertyByName(TEXT("StartEdge"));
					FProperty* EEProp = S->FindPropertyByName(TEXT("EndEdge"));
					for (int32 i = 0; i < N; ++i)
					{
						const uint8* Ptr = H.GetRawPtr(i);
						double SD = 0, ED = 0;
						if (FDoubleProperty* DP = CastField<FDoubleProperty>(SDProp)) { SD = DP->GetPropertyValue_InContainer(Ptr); }
						if (FDoubleProperty* DP = CastField<FDoubleProperty>(EDProp)) { ED = DP->GetPropertyValue_InContainer(Ptr); }
						FString SE = TEXT("?"), EE = TEXT("?");
						if (SEProp) { SEProp->ExportTextItem_Direct(SE, Ptr, nullptr, nullptr, PPF_None); }
						if (EEProp) { EEProp->ExportTextItem_Direct(EE, Ptr, nullptr, nullptr, PPF_None); }
						UE_LOG(LogAAATraffic, Log, TEXT("    CornerEditData[%d]: StartDist=%.1f EndDist=%.1f StartEdge=%s EndEdge=%s"), i, SD, ED, *SE, *EE);
					}
				}
			}
			UE_LOG(LogAAATraffic, Log, TEXT("  CornerEditData: %d entries"), N);
		}

		// 3) IntersectionMasks on the network actor
		{
			FArrayProperty* MasksArr = CastField<FArrayProperty>(DynNetworkClass->FindPropertyByName(TEXT("IntersectionMasks")));
			int32 N = 0;
			if (MasksArr)
			{
				FScriptArrayHelper H(MasksArr, MasksArr->ContainerPtrToValuePtr<void>(NetworkActor));
				N = H.Num();
				FStructProperty* SP = CastField<FStructProperty>(MasksArr->Inner);
				if (SP && N > 0)
				{
					UScriptStruct* S = SP->Struct;
					FProperty* SDProp = S->FindPropertyByName(TEXT("StartDistance"));
					FProperty* EDProp = S->FindPropertyByName(TEXT("EndDistance"));
					FProperty* PRProp = S->FindPropertyByName(TEXT("ParentRoad"));
					FObjectPropertyBase* PRObj = CastField<FObjectPropertyBase>(PRProp);
					FProperty* SCIProp = S->FindPropertyByName(TEXT("StartCutIndex"));
					FProperty* ECIProp = S->FindPropertyByName(TEXT("EndCutIndex"));
					for (int32 i = 0; i < N; ++i)
					{
						const uint8* Ptr = H.GetRawPtr(i);
						double SD = 0, ED = 0;
						if (FDoubleProperty* DP = CastField<FDoubleProperty>(SDProp)) { SD = DP->GetPropertyValue_InContainer(Ptr); }
						if (FDoubleProperty* DP = CastField<FDoubleProperty>(EDProp)) { ED = DP->GetPropertyValue_InContainer(Ptr); }
						UObject* PR = PRObj ? PRObj->GetObjectPropertyValue_InContainer(Ptr) : nullptr;
						int32 SCI = -1, ECI = -1;
						if (FIntProperty* IP = CastField<FIntProperty>(SCIProp)) { SCI = IP->GetPropertyValue_InContainer(Ptr); }
						if (FIntProperty* IP = CastField<FIntProperty>(ECIProp)) { ECI = IP->GetPropertyValue_InContainer(Ptr); }
						UE_LOG(LogAAATraffic, Log, TEXT("    IntersectionMask[%d]: StartDist=%.1f EndDist=%.1f ParentRoad=%s StartCutIdx=%d EndCutIdx=%d"), i, SD, ED, PR ? *PR->GetName() : TEXT("NULL"), SCI, ECI);
					}
				}
			}
			UE_LOG(LogAAATraffic, Log, TEXT("  IntersectionMasks: %d entries"), N);
		}

		// 4) RoadNetworkPerimeterCuts on the network actor — full field dump
		{
			FArrayProperty* CutsArr = CastField<FArrayProperty>(DynNetworkClass->FindPropertyByName(TEXT("RoadNetworkPerimeterCuts")));
			int32 N = 0;
			if (CutsArr)
			{
				FScriptArrayHelper H(CutsArr, CutsArr->ContainerPtrToValuePtr<void>(NetworkActor));
				N = H.Num();
				FStructProperty* SP2 = CastField<FStructProperty>(CutsArr->Inner);
				if (SP2 && N > 0)
				{
					UScriptStruct* CS = SP2->Struct;
					FProperty* DistProp  = CS->FindPropertyByName(TEXT("Distance"));
					FProperty* LCIProp   = CS->FindPropertyByName(TEXT("LeftCornerIndex"));
					FProperty* RCIProp   = CS->FindPropertyByName(TEXT("RightCornerIndex"));
					FProperty* DirProp   = CS->FindPropertyByName(TEXT("Direction"));
					FProperty* PRoadProp = CS->FindPropertyByName(TEXT("ParentRoad"));
					FObjectPropertyBase* PRoadObj = CastField<FObjectPropertyBase>(PRoadProp);
					for (int32 i = 0; i < N; ++i)
					{
						const uint8* Ptr = H.GetRawPtr(i);
						double Dist = 0;
						if (FDoubleProperty* DP = CastField<FDoubleProperty>(DistProp)) { Dist = DP->GetPropertyValue_InContainer(Ptr); }
						int32 LCI = -1, RCI = -1, Dir = 0;
						if (FIntProperty* IP = CastField<FIntProperty>(LCIProp)) { LCI = IP->GetPropertyValue_InContainer(Ptr); }
						if (FIntProperty* IP = CastField<FIntProperty>(RCIProp)) { RCI = IP->GetPropertyValue_InContainer(Ptr); }
						if (FIntProperty* IP = CastField<FIntProperty>(DirProp)) { Dir = IP->GetPropertyValue_InContainer(Ptr); }
						UObject* PR = PRoadObj ? PRoadObj->GetObjectPropertyValue_InContainer(Ptr) : nullptr;
						UE_LOG(LogAAATraffic, Log, TEXT("    PerimeterCut[%d]: Dist=%.1f LeftCorner=%d RightCorner=%d Dir=%d Road=%s"),
							i, Dist, LCI, RCI, Dir, PR ? *PR->GetName() : TEXT("NULL"));
					}
				}
			}
			UE_LOG(LogAAATraffic, Log, TEXT("  RoadNetworkPerimeterCuts: %d entries"), N);
		}

		// 5) IntersectionCutDistances on each DynamicRoad actor
		if (DynRoadClass)
		{
			FProperty* ICDProp = DynRoadClass->FindPropertyByName(TEXT("IntersectionCutDistances"));
			FArrayProperty* ICDArr = CastField<FArrayProperty>(ICDProp);
			for (auto& Pair : RoadHandleMap)
			{
				UObject* RoadObj = Pair.Value.Get();
				if (!RoadObj || !ICDArr) { continue; }
				FScriptArrayHelper H(ICDArr, ICDArr->ContainerPtrToValuePtr<void>(RoadObj));
				const int32 RN = H.Num();
				UE_LOG(LogAAATraffic, Log, TEXT("  Road '%s' IntersectionCutDistances: %d entries"), *RoadObj->GetName(), RN);
				FStructProperty* V2SP = CastField<FStructProperty>(ICDArr->Inner);
				if (V2SP && RN > 0)
				{
					for (int32 i = 0; i < RN; ++i)
					{
						const uint8* Ptr = H.GetRawPtr(i);
						const FVector2D* V = reinterpret_cast<const FVector2D*>(Ptr);
						UE_LOG(LogAAATraffic, Log, TEXT("    [%d]: X=%.1f Y=%.1f"), i, V->X, V->Y);
					}
				}
			}
		}

		UE_LOG(LogAAATraffic, Log, TEXT("===== END PRE-REBUILD DUMP ====="));
	} // GEnableDiagnosticDumps

	// ── Trigger RoadBLD incremental rebuild with FastPreview ────
	// WHY this is needed at runtime:
	//   RoadBLD's intersection detection (RoadNetworkCorners, IntersectionMasks,
	//   PerimeterCuts) is computed lazily inside the editor during road editing.
	//   At runtime/PIE, these arrays may be stale or unpopulated if the user
	//   modified roads after the last editor rebuild. Calling
	//   RebuildRoadNetworkIncremental with bFastPreview=true triggers only the
	//   intersection detection pass (skips expensive mesh generation) so the
	//   corner and mask arrays are up-to-date when we read them.
	//
	// PERFORMANCE: bFastPreview=true avoids road-mesh generation, so the cost
	//   is bounded to intersection detection (~ms range for typical networks).
	//
	// SHIPPING: RebuildRoadNetworkIncremental is a UFunction on the RoadBLD
	//   runtime module (not editor-only), so it is available in packaged builds.
	//   If RoadBLD ships pre-baked data, this call is a no-op for already
	//   up-to-date networks.
	//
	// TODO: Consider adding a CVar gate (traffic.SkipRoadBLDRebuild) for users
	//   who have confirmed their data is pre-baked and want to skip this step.
	{
		UFunction* RebuildFunc = NetworkActor->FindFunction(TEXT("RebuildRoadNetworkIncremental"));
		if (RebuildFunc)
		{
			// Build the ModifiedRoads array from our cached road actors.
			TArray<UObject*> AllRoads;
			for (auto& Pair : RoadHandleMap)
			{
				if (UObject* R = Pair.Value.Get()) { AllRoads.Add(R); }
			}

			// Params layout must match:
			// TArray<ADynamicRoad*> ModifiedRoads, TArray<ADynamicRoad*> RoadsToIgnore, bool bFastPreview
			struct
			{
				TArray<UObject*> ModifiedRoads;
				TArray<UObject*> RoadsToIgnore;
				bool bFastPreview;
			} Parms;
			Parms.ModifiedRoads = AllRoads;
			Parms.RoadsToIgnore = TArray<UObject*>();
			Parms.bFastPreview  = true;

			UE_LOG(LogAAATraffic, Log,
				TEXT("RoadBLDReflectionProvider: Calling RebuildRoadNetworkIncremental(bFastPreview=true) with %d roads..."),
				AllRoads.Num());

			NetworkActor->ProcessEvent(RebuildFunc, &Parms);

			UE_LOG(LogAAATraffic, Log,
				TEXT("RoadBLDReflectionProvider: RebuildRoadNetworkIncremental completed."));
		}
		else
		{
			UE_LOG(LogAAATraffic, Warning,
				TEXT("RoadBLDReflectionProvider: RebuildRoadNetworkIncremental function not found."));
		}
	}

	// ── UNCONDITIONAL post-rebuild array inventory ─────────────
	// Always log the count of every corner-related array so we know
	// exactly where CornerBuilder stored its data.  This is the single
	// source of truth for diagnosing "0 cross-road connections."
	{
		UE_LOG(LogAAATraffic, Log, TEXT("===== POST-REBUILD CORNER ARRAY INVENTORY ====="));

		// 1. RoadNetworkCorners
		FArrayProperty* CornersArr = CastField<FArrayProperty>(
			DynNetworkClass->FindPropertyByName(TEXT("RoadNetworkCorners")));
		int32 NC = 0;
		if (CornersArr)
		{
			FScriptArrayHelper H(CornersArr, CornersArr->ContainerPtrToValuePtr<void>(NetworkActor));
			NC = H.Num();
		}
		UE_LOG(LogAAATraffic, Log, TEXT("  RoadNetworkCorners: %s (%d entries)"),
			CornersArr ? TEXT("found") : TEXT("NOT FOUND"), NC);

		// 2. CornerEditData
		FArrayProperty* EditArr = CastField<FArrayProperty>(
			DynNetworkClass->FindPropertyByName(TEXT("CornerEditData")));
		int32 NE = 0;
		if (EditArr)
		{
			FScriptArrayHelper H(EditArr, EditArr->ContainerPtrToValuePtr<void>(NetworkActor));
			NE = H.Num();
		}
		UE_LOG(LogAAATraffic, Log, TEXT("  CornerEditData:     %s (%d entries)"),
			EditArr ? TEXT("found") : TEXT("NOT FOUND"), NE);

		// 3. IntersectionMasks
		FArrayProperty* MasksArr = CastField<FArrayProperty>(
			DynNetworkClass->FindPropertyByName(TEXT("IntersectionMasks")));
		int32 NM = 0;
		if (MasksArr)
		{
			FScriptArrayHelper H(MasksArr, MasksArr->ContainerPtrToValuePtr<void>(NetworkActor));
			NM = H.Num();
		}
		UE_LOG(LogAAATraffic, Log, TEXT("  IntersectionMasks:  %s (%d entries)"),
			MasksArr ? TEXT("found") : TEXT("NOT FOUND"), NM);

		// 4. RoadNetworkPerimeterCuts
		FArrayProperty* CutsArr = CastField<FArrayProperty>(
			DynNetworkClass->FindPropertyByName(TEXT("RoadNetworkPerimeterCuts")));
		int32 NP = 0;
		if (CutsArr)
		{
			FScriptArrayHelper H(CutsArr, CutsArr->ContainerPtrToValuePtr<void>(NetworkActor));
			NP = H.Num();
		}
		UE_LOG(LogAAATraffic, Log, TEXT("  PerimeterCuts:      %s (%d entries)"),
			CutsArr ? TEXT("found") : TEXT("NOT FOUND"), NP);

		// 5. Enumerate ALL TArray properties on DynNetworkClass that
		//    contain "Corner" or "Edge" in their name — catches any
		//    array we haven't thought of yet.
		UE_LOG(LogAAATraffic, Log, TEXT("  -- All array properties on %s containing 'Corner' or 'Edge':"),
			*DynNetworkClass->GetName());
		for (TFieldIterator<FArrayProperty> It(DynNetworkClass); It; ++It)
		{
			FString PropName = It->GetName();
			if (PropName.Contains(TEXT("Corner")) || PropName.Contains(TEXT("Edge")))
			{
				FScriptArrayHelper H(*It, It->ContainerPtrToValuePtr<void>(NetworkActor));
				UE_LOG(LogAAATraffic, Log, TEXT("     %s: %d entries"), *PropName, H.Num());
			}
		}

		// 6. Enumerate ALL UFunctions on DynNetworkClass that contain
		//    "Corner" — shows what API is available.
		UE_LOG(LogAAATraffic, Log, TEXT("  -- All UFunctions containing 'Corner':"));
		for (TFieldIterator<UFunction> It(DynNetworkClass); It; ++It)
		{
			FString FuncName = It->GetName();
			if (FuncName.Contains(TEXT("Corner")))
			{
				UE_LOG(LogAAATraffic, Log, TEXT("     %s"), *FuncName);
			}
		}

		// 7. PerimeterCuts corner-index dump: if cuts reference corners
		//    by index, show which indices they use.
		if (CutsArr && NP > 0)
		{
			FStructProperty* SP = CastField<FStructProperty>(CutsArr->Inner);
			if (SP)
			{
				UScriptStruct* CS = SP->Struct;
				FIntProperty* LCIProp = CastField<FIntProperty>(CS->FindPropertyByName(TEXT("LeftCornerIndex")));
				FIntProperty* RCIProp = CastField<FIntProperty>(CS->FindPropertyByName(TEXT("RightCornerIndex")));
				FProperty* PRoadProp = CS->FindPropertyByName(TEXT("ParentRoad"));
				FObjectPropertyBase* PRoadObj = CastField<FObjectPropertyBase>(PRoadProp);
				FProperty* DistProp = CS->FindPropertyByName(TEXT("Distance"));
				FScriptArrayHelper CutsHelper(CutsArr, CutsArr->ContainerPtrToValuePtr<void>(NetworkActor));
				for (int32 i = 0; i < CutsHelper.Num(); ++i)
				{
					const uint8* Ptr = CutsHelper.GetRawPtr(i);
					int32 LCI = LCIProp ? LCIProp->GetPropertyValue_InContainer(Ptr) : -999;
					int32 RCI = RCIProp ? RCIProp->GetPropertyValue_InContainer(Ptr) : -999;
					UObject* PR = PRoadObj ? PRoadObj->GetObjectPropertyValue_InContainer(Ptr) : nullptr;
					double Dist = 0;
					if (FDoubleProperty* DP = CastField<FDoubleProperty>(DistProp)) { Dist = DP->GetPropertyValue_InContainer(Ptr); }
					UE_LOG(LogAAATraffic, Log,
						TEXT("  PerimeterCut[%d]: LeftCorner=%d RightCorner=%d Dist=%.1f Road=%s"),
						i, LCI, RCI, Dist, PR ? *PR->GetName() : TEXT("NULL"));
				}
			}
		}

		// 8. If RoadNetworkCorners has entries, dump their StartEdge/EndEdge
		if (CornersArr && NC > 0)
		{
			FStructProperty* SP = CastField<FStructProperty>(CornersArr->Inner);
			if (SP)
			{
				UScriptStruct* CS = SP->Struct;
				FObjectPropertyBase* SEProp = CastField<FObjectPropertyBase>(CS->FindPropertyByName(TEXT("StartEdge")));
				FObjectPropertyBase* EEProp = CastField<FObjectPropertyBase>(CS->FindPropertyByName(TEXT("EndEdge")));
				FBoolProperty* StaleProp = CastField<FBoolProperty>(CS->FindPropertyByName(TEXT("bStale")));
				FScriptArrayHelper CornersHelper(CornersArr, CornersArr->ContainerPtrToValuePtr<void>(NetworkActor));
				for (int32 i = 0; i < CornersHelper.Num(); ++i)
				{
					const uint8* Ptr = CornersHelper.GetRawPtr(i);
					UObject* SE = SEProp ? SEProp->GetObjectPropertyValue_InContainer(Ptr) : nullptr;
					UObject* EE = EEProp ? EEProp->GetObjectPropertyValue_InContainer(Ptr) : nullptr;
					bool bStale = StaleProp ? StaleProp->GetPropertyValue_InContainer(Ptr) : false;
					UE_LOG(LogAAATraffic, Log,
						TEXT("  Corner[%d]: StartEdge=%s EndEdge=%s Stale=%s"),
						i,
						SE ? *SE->GetName() : TEXT("NULL"),
						EE ? *EE->GetName() : TEXT("NULL"),
						bStale ? TEXT("YES") : TEXT("no"));
				}
			}
		}

		// 9. Edge-walking probe: call GetNextCornerConnection on the
		//    first edge with detailed return-value logging.
		UFunction* ProbeFunc = DynNetworkClass->FindFunctionByName(TEXT("GetNextCornerConnection"));
		if (ProbeFunc)
		{
			// Find the first valid edge from the first road.
			UObject* ProbeEdge = nullptr;
			for (auto& Pair : RoadHandleMap)
			{
				UObject* RoadObj = Pair.Value.Get();
				if (!RoadObj) { continue; }
				FArrayProperty* ECProp = CastField<FArrayProperty>(
					RoadObj->GetClass()->FindPropertyByName(TEXT("EdgeCurves")));
				if (!ECProp) { continue; }
				FScriptArrayHelper EH(ECProp, ECProp->ContainerPtrToValuePtr<void>(RoadObj));
				FObjectPropertyBase* InnerObj = CastField<FObjectPropertyBase>(ECProp->Inner);
				if (!InnerObj) { continue; }
				for (int32 e = 0; e < EH.Num(); ++e)
				{
					UObject* E = InnerObj->GetObjectPropertyValue(EH.GetRawPtr(e));
					if (E) { ProbeEdge = E; break; }
				}
				if (ProbeEdge) { break; }
			}

			if (ProbeEdge)
			{
				UE_LOG(LogAAATraffic, Log,
					TEXT("  Edge-walk probe: calling GetNextCornerConnection(Edge=%s, Dist=-1.0)"),
					*ProbeEdge->GetName());

				// Dump the function's parameter layout.
				for (TFieldIterator<FProperty> It(ProbeFunc); It; ++It)
				{
					UE_LOG(LogAAATraffic, Log,
						TEXT("    Param: %s  Type=%s  Offset=%d  Size=%d  Flags=0x%llX"),
						*It->GetName(),
						*It->GetCPPType(),
						It->GetOffset_ForInternal(),
						It->GetSize(),
						It->GetPropertyFlags());
				}
				UE_LOG(LogAAATraffic, Log,
					TEXT("    ParmsSize=%d  ReturnValueOffset=%d"),
					ProbeFunc->ParmsSize,
					ProbeFunc->ReturnValueOffset);
			}
		}

		UE_LOG(LogAAATraffic, Log, TEXT("===== END POST-REBUILD CORNER ARRAY INVENTORY ====="));
	}

	// ── Extended post-rebuild dump (gated behind CVar) ──────────
	if (GEnableDiagnosticDumps)
	{
		UE_LOG(LogAAATraffic, Log, TEXT("===== POST-REBUILD ROADBLD EXTENDED DUMP ====="));

		if (DynRoadClass)
		{
			FArrayProperty* ICDArr = CastField<FArrayProperty>(DynRoadClass->FindPropertyByName(TEXT("IntersectionCutDistances")));
			for (auto& Pair : RoadHandleMap)
			{
				UObject* RoadObj = Pair.Value.Get();
				if (!RoadObj || !ICDArr) { continue; }
				FScriptArrayHelper H(ICDArr, ICDArr->ContainerPtrToValuePtr<void>(RoadObj));
				UE_LOG(LogAAATraffic, Log, TEXT("  Road '%s' IntersectionCutDistances: %d entries"), *RoadObj->GetName(), H.Num());
			}
		}

		UE_LOG(LogAAATraffic, Log, TEXT("===== END POST-REBUILD EXTENDED DUMP ====="));
	} // GEnableDiagnosticDumps
}

// ---------------------------------------------------------------------------
// BuildLaneConnectivity — edge-walking corner discovery
// ---------------------------------------------------------------------------
// Attempt edge-walking via GetNextCornerConnection() first. If that returns
// zero corners (common — the function may require editor-only state), fall
// back to reading the RoadNetworkCorners TArray which CornerBuilder
// populates during RebuildRoadNetworkIncremental.

void URoadBLDReflectionProvider::BuildLaneConnectivity(UWorld* World)
{
	// Note: LaneConnectionMap is NOT cleared here — proximity connections
	// may already be present. Corner-based connections are additive.

	if (!DynNetworkClass)
	{
		UE_LOG(LogAAATraffic, Log,
			TEXT("RoadBLDReflectionProvider: DynamicRoadNetwork class not found — skipping corner-based connectivity."));
		return;
	}

	// Find the road network actor deterministically.
	AActor* NetworkActor = nullptr;
	{
		TArray<AActor*> NetworkCandidates;
		for (FActorIterator It(World); It; ++It)
		{
			if (It->IsA(DynNetworkClass))
			{
				NetworkCandidates.Add(*It);
			}
		}
		if (NetworkCandidates.Num() > 0)
		{
			NetworkCandidates.Sort([](const AActor& A, const AActor& B) { return A.GetName() < B.GetName(); });
			NetworkActor = NetworkCandidates[0];
		}
	}
	if (!NetworkActor)
	{
		UE_LOG(LogAAATraffic, Log,
			TEXT("RoadBLDReflectionProvider: No DynamicRoadNetwork actor — skipping corner-based connectivity."));
		return;
	}

	// Trigger rebuild and optional diagnostic dumps in a separate method
	// to keep this function focused on corner-based connectivity logic.
	TriggerRoadBLDRebuildAndDiagnostics(World, NetworkActor);

	// ── Resolve GetNextCornerConnection UFunction on the network class ──
	UFunction* GetNextCornerFunc = DynNetworkClass->FindFunctionByName(TEXT("GetNextCornerConnection"));
	if (!GetNextCornerFunc)
	{
		UE_LOG(LogAAATraffic, Warning,
			TEXT("RoadBLDReflectionProvider: GetNextCornerConnection not found on DynamicRoadNetwork — "
				 "falling back to RoadNetworkCorners TArray (may be empty)."));
		BuildLaneConnectivityFromCornersArray(World, NetworkActor);
		return;
	}

	// ── Resolve function parameter properties by name ──
	FObjectPropertyBase* ParamEdgeObjProp = nullptr;
	FProperty* ParamDistProp = nullptr;  // May be FDoubleProperty or FFloatProperty
	FBoolProperty*       ParamFoundBoolProp = nullptr;
	FStructProperty*     ReturnProp = nullptr;

	for (TFieldIterator<FProperty> It(GetNextCornerFunc); It; ++It)
	{
		FProperty* Prop = *It;
		const FName PropName = Prop->GetFName();

		if (PropName == TEXT("Edge"))
		{
			ParamEdgeObjProp = CastField<FObjectPropertyBase>(Prop);
		}
		else if (PropName == TEXT("CurrentDistance"))
		{
			ParamDistProp = Prop;
		}
		else if (PropName == TEXT("bFound"))
		{
			ParamFoundBoolProp = CastField<FBoolProperty>(Prop);
		}
		else if (Prop->HasAnyPropertyFlags(CPF_ReturnParm))
		{
			ReturnProp = CastField<FStructProperty>(Prop);
		}
	}

	if (!ParamEdgeObjProp || !ParamDistProp || !ParamFoundBoolProp || !ReturnProp)
	{
		UE_LOG(LogAAATraffic, Warning,
			TEXT("RoadBLDReflectionProvider: GetNextCornerConnection has unexpected parameter layout — "
				 "Edge=%s Dist=%s Found=%s Return=%s. Falling back to TArray."),
			ParamEdgeObjProp ? TEXT("OK") : TEXT("MISSING"),
			ParamDistProp ? TEXT("OK") : TEXT("MISSING"),
			ParamFoundBoolProp ? TEXT("OK") : TEXT("MISSING"),
			ReturnProp ? TEXT("OK") : TEXT("MISSING"));
		BuildLaneConnectivityFromCornersArray(World, NetworkActor);
		return;
	}

	// ── Resolve FRoadNetworkCorner sub-properties from the return struct ──
	UScriptStruct* CornerStruct = ReturnProp->Struct;
	if (!CornerStruct)
	{
		UE_LOG(LogAAATraffic, Warning,
			TEXT("RoadBLDReflectionProvider: Return value struct is null — cannot parse corner data."));
		BuildLaneConnectivityFromCornersArray(World, NetworkActor);
		return;
	}

	FObjectPropertyBase* CornerStartEdgeObj = CastField<FObjectPropertyBase>(
		CornerStruct->FindPropertyByName(TEXT("StartEdge")));
	FObjectPropertyBase* CornerEndEdgeObj = CastField<FObjectPropertyBase>(
		CornerStruct->FindPropertyByName(TEXT("EndEdge")));
	FProperty* CornerStartDistProp = CornerStruct->FindPropertyByName(TEXT("StartDistance"));
	FProperty* CornerEndDistProp = CornerStruct->FindPropertyByName(TEXT("EndDistance"));
	FBoolProperty* CornerStaleBool = CastField<FBoolProperty>(
		CornerStruct->FindPropertyByName(TEXT("bStale")));
	FStructProperty* CornerIDStructProp = CastField<FStructProperty>(
		CornerStruct->FindPropertyByName(TEXT("CornerID")));
	FProperty* CornerIntersectionPtProp = CornerStruct->FindPropertyByName(TEXT("IntersectionPoint"));

	if (!CornerStartEdgeObj || !CornerEndEdgeObj)
	{
		UE_LOG(LogAAATraffic, Warning,
			TEXT("RoadBLDReflectionProvider: FRoadNetworkCorner missing StartEdge/EndEdge — "
				 "cannot build connections. Falling back to TArray."));
		BuildLaneConnectivityFromCornersArray(World, NetworkActor);
		return;
	}

	// ── Resolve edge-curve LeftLane/RightLane properties ──
	FObjectPropertyBase* EdgeLeftLaneObj = nullptr;
	FObjectPropertyBase* EdgeRightLaneObj = nullptr;

	for (auto& Pair : RoadHandleMap)
	{
		UObject* RoadObj = Pair.Value.Get();
		if (!RoadObj) { continue; }

		FArrayProperty* EdgeCurvesProp = CastField<FArrayProperty>(
			RoadObj->GetClass()->FindPropertyByName(TEXT("EdgeCurves")));
		if (!EdgeCurvesProp) { continue; }

		FScriptArrayHelper EdgesHelper(EdgeCurvesProp,
			EdgeCurvesProp->ContainerPtrToValuePtr<void>(RoadObj));
		FObjectPropertyBase* InnerObjProp = CastField<FObjectPropertyBase>(EdgeCurvesProp->Inner);
		if (!InnerObjProp) { continue; }

		for (int32 e = 0; e < EdgesHelper.Num(); ++e)
		{
			UObject* EdgeObj = InnerObjProp->GetObjectPropertyValue(EdgesHelper.GetRawPtr(e));
			if (!EdgeObj) { continue; }

			FProperty* LeftP = EdgeObj->GetClass()->FindPropertyByName(TEXT("LeftLane"));
			FProperty* RightP = EdgeObj->GetClass()->FindPropertyByName(TEXT("RightLane"));
			EdgeLeftLaneObj = CastField<FObjectPropertyBase>(LeftP);
			EdgeRightLaneObj = CastField<FObjectPropertyBase>(RightP);
			if (EdgeLeftLaneObj && EdgeRightLaneObj) { break; }
		}
		if (EdgeLeftLaneObj && EdgeRightLaneObj) { break; }
	}

	if (!EdgeLeftLaneObj || !EdgeRightLaneObj)
	{
		UE_LOG(LogAAATraffic, Warning,
			TEXT("RoadBLDReflectionProvider: Could not resolve LeftLane/RightLane on edge curves — "
				 "no corner-based connections."));
		return;
	}

	// ── Lambda: collect lane handles from an edge object ──
	auto CollectLaneHandles = [&](UObject* Edge, TArray<int32>& OutHandles)
	{
		if (!Edge) { return; }
		UObject* LeftLane = EdgeLeftLaneObj->GetObjectPropertyValue_InContainer(Edge);
		UObject* RightLane = EdgeRightLaneObj->GetObjectPropertyValue_InContainer(Edge);
		if (LeftLane)
		{
			if (const int32* H = LaneToHandleMap.Find(LeftLane)) { OutHandles.AddUnique(*H); }
		}
		if (RightLane)
		{
			if (const int32* H = LaneToHandleMap.Find(RightLane)) { OutHandles.AddUnique(*H); }
		}
	};

	// ── Helper: read a double value from a property that may be double or float ──
	auto ReadDistanceValue = [](FProperty* Prop, const void* Container) -> double
	{
		if (FDoubleProperty* DblProp = CastField<FDoubleProperty>(Prop))
		{
			return DblProp->GetPropertyValue_InContainer(Container);
		}
		if (FFloatProperty* FltProp = CastField<FFloatProperty>(Prop))
		{
			return static_cast<double>(FltProp->GetPropertyValue_InContainer(Container));
		}
		return 0.0;
	};

	auto WriteDistanceValue = [](FProperty* Prop, void* Container, double Value)
	{
		if (FDoubleProperty* DblProp = CastField<FDoubleProperty>(Prop))
		{
			DblProp->SetPropertyValue_InContainer(Container, Value);
		}
		else if (FFloatProperty* FltProp = CastField<FFloatProperty>(Prop))
		{
			FltProp->SetPropertyValue_InContainer(Container, static_cast<float>(Value));
		}
	};

	// ── Walk all edge curves to discover corners ──────────────────
	TSet<FGuid> SeenCornerIDs;
	int32 CornerConnections = 0;
	int32 TotalCornersFound = 0;
	int32 TotalEdgesWalked = 0;

	// Sort road handles for deterministic iteration.
	TArray<int32> SortedRoadHandles;
	RoadHandleMap.GetKeys(SortedRoadHandles);
	SortedRoadHandles.Sort();

	for (const int32 RoadHandle : SortedRoadHandles)
	{
		UObject* RoadObj = RoadHandleMap[RoadHandle].Get();
		if (!RoadObj) { continue; }

		// Access EdgeCurves TArray property on this road.
		FArrayProperty* EdgeCurvesProp = CastField<FArrayProperty>(
			RoadObj->GetClass()->FindPropertyByName(TEXT("EdgeCurves")));
		if (!EdgeCurvesProp) { continue; }

		FScriptArrayHelper EdgesHelper(EdgeCurvesProp,
			EdgeCurvesProp->ContainerPtrToValuePtr<void>(RoadObj));
		FObjectPropertyBase* InnerObjProp = CastField<FObjectPropertyBase>(EdgeCurvesProp->Inner);
		if (!InnerObjProp) { continue; }

		for (int32 e = 0; e < EdgesHelper.Num(); ++e)
		{
			UObject* EdgeObj = InnerObjProp->GetObjectPropertyValue(EdgesHelper.GetRawPtr(e));
			if (!EdgeObj) { continue; }

			++TotalEdgesWalked;

			// Walk corners along this edge using GetNextCornerConnection.
			// Start at distance -1.0 to catch any corner at distance 0.0.
			double WalkDistance = -1.0;
			constexpr int32 MaxCornersPerEdge = 100; // safety limit

			for (int32 Step = 0; Step < MaxCornersPerEdge; ++Step)
			{
				// Allocate params buffer on the stack.
				TArray<uint8> ParamsStorage;
				ParamsStorage.AddZeroed(GetNextCornerFunc->ParmsSize);
				uint8* ParamsBuffer = ParamsStorage.GetData();

				// Set input parameters.
				ParamEdgeObjProp->SetObjectPropertyValue_InContainer(ParamsBuffer, EdgeObj);
				WriteDistanceValue(ParamDistProp, ParamsBuffer, WalkDistance);

				// Call GetNextCornerConnection on the network actor.
				NetworkActor->ProcessEvent(GetNextCornerFunc, ParamsBuffer);

				// Read bFound output.
				const bool bFound = ParamFoundBoolProp->GetPropertyValue_InContainer(ParamsBuffer);
				if (!bFound) { break; } // No more corners on this edge.

				// Read the returned FRoadNetworkCorner from the return value.
				const uint8* CornerPtr = ReturnProp->ContainerPtrToValuePtr<uint8>(ParamsBuffer);

				// ── Advance walk position past this corner ──
				UObject* CornerStartEdge = CornerStartEdgeObj->GetObjectPropertyValue_InContainer(CornerPtr);
				UObject* CornerEndEdge = CornerEndEdgeObj->GetObjectPropertyValue_InContainer(CornerPtr);

				double CornerDistOnThisEdge = WalkDistance + 100.0; // fallback
				if (CornerStartEdge == EdgeObj && CornerStartDistProp)
				{
					CornerDistOnThisEdge = ReadDistanceValue(CornerStartDistProp, CornerPtr);
				}
				else if (CornerEndEdge == EdgeObj && CornerEndDistProp)
				{
					CornerDistOnThisEdge = ReadDistanceValue(CornerEndDistProp, CornerPtr);
				}
				WalkDistance = CornerDistOnThisEdge + 0.1;

				// ── Skip stale corners ──
				if (CornerStaleBool && CornerStaleBool->GetPropertyValue_InContainer(CornerPtr))
				{
					continue;
				}

				// ── Deduplicate by CornerID (FGuid) ──
				// Each corner is seen from both participating edges; only process once.
				if (CornerIDStructProp)
				{
					FGuid ThisCornerID;
					const void* IDPtr = CornerIDStructProp->ContainerPtrToValuePtr<void>(CornerPtr);
					FMemory::Memcpy(&ThisCornerID, IDPtr, sizeof(FGuid));

					if (ThisCornerID.IsValid())
					{
						bool bAlreadySeen = false;
						SeenCornerIDs.Add(ThisCornerID, &bAlreadySeen);
						if (bAlreadySeen) { continue; }
					}
				}

				++TotalCornersFound;

				// ── Build cross-road connections from this corner ──
				if (!CornerStartEdge || !CornerEndEdge) { continue; }

				TArray<int32> StartHandles, EndHandles;
				CollectLaneHandles(CornerStartEdge, StartHandles);
				CollectLaneHandles(CornerEndEdge, EndHandles);

				// Connect StartEdge lanes ↔ EndEdge lanes (bidirectional).
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
				for (const int32 Src : EndHandles)
				{
					TArray<FTrafficLaneHandle>& Conns = LaneConnectionMap.FindOrAdd(Src);
					for (const int32 Dst : StartHandles)
					{
						if (Src != Dst)
						{
							Conns.AddUnique(FTrafficLaneHandle(Dst));
							++CornerConnections;
						}
					}
				}

				if (ShouldLogDiagnostics(2))
				{
					UE_LOG(LogAAATraffic, Log,
						TEXT("  Corner: StartEdge=%s EndEdge=%s StartLanes=[%s] EndLanes=[%s] Dist=%.1f"),
						CornerStartEdge ? *CornerStartEdge->GetName() : TEXT("null"),
						CornerEndEdge ? *CornerEndEdge->GetName() : TEXT("null"),
						*FString::JoinBy(StartHandles, TEXT(","),
							[](int32 H) { return FString::FromInt(H); }),
						*FString::JoinBy(EndHandles, TEXT(","),
							[](int32 H) { return FString::FromInt(H); }),
						CornerDistOnThisEdge);
				}
			}
		}
	}

	UE_LOG(LogAAATraffic, Log,
		TEXT("RoadBLDReflectionProvider: Edge-walking corner discovery — "
			 "walked %d edges across %d roads, found %d unique corners, "
			 "added %d cross-road connections."),
		TotalEdgesWalked, SortedRoadHandles.Num(),
		TotalCornersFound, CornerConnections);

	// ── Fallback: if edge-walking found nothing, try the TArray directly ──
	// After RebuildRoadNetworkIncremental, CornerBuilder may populate
	// RoadNetworkCorners even when GetNextCornerConnection returns empty.
	if (TotalCornersFound == 0)
	{
		UE_LOG(LogAAATraffic, Log,
			TEXT("RoadBLDReflectionProvider: Edge-walking found 0 corners — "
				 "falling back to RoadNetworkCorners TArray."));
		BuildLaneConnectivityFromCornersArray(World, NetworkActor);
	}
}

// ---------------------------------------------------------------------------
// BuildLaneConnectivityFromCornersArray — reads RoadNetworkCorners TArray
// ---------------------------------------------------------------------------
// Primary fallback when GetNextCornerConnection edge-walking returns empty.
// After RebuildRoadNetworkIncremental, CornerBuilder populates this TArray
// with the computed intersection corners.

void URoadBLDReflectionProvider::BuildLaneConnectivityFromCornersArray(
	UWorld* World, AActor* NetworkActor)
{
	if (!NetworkActor || !DynNetworkClass) { return; }

	// Try RoadNetworkCorners first, then CornerEditData as fallback.
	// CornerBuilder may populate either or both arrays depending on
	// RoadBLD version and configuration.
	FArrayProperty* CornerArrayProp = nullptr;
	FName UsedArrayName = NAME_None;

	{
		FProperty* P = DynNetworkClass->FindPropertyByName(TEXT("RoadNetworkCorners"));
		FArrayProperty* AP = P ? CastField<FArrayProperty>(P) : nullptr;
		if (AP)
		{
			FScriptArrayHelper H(AP, AP->ContainerPtrToValuePtr<void>(NetworkActor));
			if (H.Num() > 0)
			{
				CornerArrayProp = AP;
				UsedArrayName = TEXT("RoadNetworkCorners");
			}
			else
			{
				UE_LOG(LogAAATraffic, Log,
					TEXT("RoadBLDReflectionProvider: RoadNetworkCorners has 0 entries — trying CornerEditData."));
			}
		}
	}

	if (!CornerArrayProp)
	{
		FProperty* P = DynNetworkClass->FindPropertyByName(TEXT("CornerEditData"));
		FArrayProperty* AP = P ? CastField<FArrayProperty>(P) : nullptr;
		if (AP)
		{
			FScriptArrayHelper H(AP, AP->ContainerPtrToValuePtr<void>(NetworkActor));
			if (H.Num() > 0)
			{
				CornerArrayProp = AP;
				UsedArrayName = TEXT("CornerEditData");
			}
			else
			{
				UE_LOG(LogAAATraffic, Log,
					TEXT("RoadBLDReflectionProvider: CornerEditData also has 0 entries — no corner-based connectivity."));
			}
		}
	}

	if (!CornerArrayProp)
	{
		UE_LOG(LogAAATraffic, Log,
			TEXT("RoadBLDReflectionProvider: No populated corner array found — skipping corner-based connectivity."));
		return;
	}

	FScriptArrayHelper ArrayHelper(CornerArrayProp, CornerArrayProp->ContainerPtrToValuePtr<void>(NetworkActor));
	const int32 NumCorners = ArrayHelper.Num();

	UE_LOG(LogAAATraffic, Log,
		TEXT("RoadBLDReflectionProvider: Using %s with %d entries for corner connectivity."),
		*UsedArrayName.ToString(), NumCorners);

	FStructProperty* StructProp = CastField<FStructProperty>(CornerArrayProp->Inner);
	if (!StructProp) { return; }

	UScriptStruct* CornerStruct = StructProp->Struct;
	FObjectPropertyBase* StartEdgeObjProp = CastField<FObjectPropertyBase>(
		CornerStruct->FindPropertyByName(TEXT("StartEdge")));
	FObjectPropertyBase* EndEdgeObjProp = CastField<FObjectPropertyBase>(
		CornerStruct->FindPropertyByName(TEXT("EndEdge")));
	FBoolProperty* bStaleNative = CastField<FBoolProperty>(
		CornerStruct->FindPropertyByName(TEXT("bStale")));
	FProperty* IntPtProp = CornerStruct->FindPropertyByName(TEXT("IntersectionPoint"));

	if (!StartEdgeObjProp || !EndEdgeObjProp)
	{
		UE_LOG(LogAAATraffic, Warning,
			TEXT("RoadBLDReflectionProvider: %s entries lack StartEdge/EndEdge — "
				 "cannot build corner connections."),
			*UsedArrayName.ToString());
		return;
	}

	// Resolve edge-curve lane properties from first valid edge.
	FObjectPropertyBase* EdgeLeftLaneObj = nullptr;
	FObjectPropertyBase* EdgeRightLaneObj = nullptr;
	for (int32 i = 0; i < NumCorners; ++i)
	{
		UObject* Edge = StartEdgeObjProp->GetObjectPropertyValue_InContainer(ArrayHelper.GetRawPtr(i));
		if (Edge)
		{
			EdgeLeftLaneObj = CastField<FObjectPropertyBase>(
				Edge->GetClass()->FindPropertyByName(TEXT("LeftLane")));
			EdgeRightLaneObj = CastField<FObjectPropertyBase>(
				Edge->GetClass()->FindPropertyByName(TEXT("RightLane")));
			break;
		}
	}
	if (!EdgeLeftLaneObj || !EdgeRightLaneObj) { return; }

	auto CollectLaneHandles = [&](UObject* Edge, TArray<int32>& OutHandles)
	{
		if (!Edge) { return; }
		UObject* LeftLane  = EdgeLeftLaneObj->GetObjectPropertyValue_InContainer(Edge);
		UObject* RightLane = EdgeRightLaneObj->GetObjectPropertyValue_InContainer(Edge);

		UE_LOG(LogAAATraffic, Log,
			TEXT("    CollectLaneHandles: Edge=%s LeftLane=%s(%p) RightLane=%s(%p) LaneToHandleMap.Num=%d"),
			*Edge->GetName(),
			LeftLane ? *LeftLane->GetName() : TEXT("NULL"), static_cast<void*>(LeftLane),
			RightLane ? *RightLane->GetName() : TEXT("NULL"), static_cast<void*>(RightLane),
			LaneToHandleMap.Num());

		if (LeftLane)
		{
			if (const int32* H = LaneToHandleMap.Find(LeftLane))
			{
				OutHandles.AddUnique(*H);
			}
			else
			{
				// Edge->LeftLane points to a lane UObject not in our map.
				// Try by-name fallback: find any cached lane with the same name.
				for (const auto& MapPair : LaneToHandleMap)
				{
					if (MapPair.Key.IsValid() && MapPair.Key->GetName() == LeftLane->GetName())
					{
						OutHandles.AddUnique(MapPair.Value);
						UE_LOG(LogAAATraffic, Log,
							TEXT("      LeftLane '%s' matched by name to handle %d (cached ptr=%p vs edge ptr=%p)"),
							*LeftLane->GetName(), MapPair.Value, static_cast<void*>(MapPair.Key.Get()), static_cast<void*>(LeftLane));
						break;
					}
				}
			}
		}
		if (RightLane)
		{
			if (const int32* H = LaneToHandleMap.Find(RightLane))
			{
				OutHandles.AddUnique(*H);
			}
			else
			{
				for (const auto& MapPair : LaneToHandleMap)
				{
					if (MapPair.Key.IsValid() && MapPair.Key->GetName() == RightLane->GetName())
					{
						OutHandles.AddUnique(MapPair.Value);
						UE_LOG(LogAAATraffic, Log,
							TEXT("      RightLane '%s' matched by name to handle %d (cached ptr=%p vs edge ptr=%p)"),
							*RightLane->GetName(), MapPair.Value, static_cast<void*>(MapPair.Key.Get()), static_cast<void*>(RightLane));
						break;
					}
				}
			}
		}
	};

	int32 CornerConnections = 0;
	for (int32 i = 0; i < NumCorners; ++i)
	{
		const uint8* ElemPtr = ArrayHelper.GetRawPtr(i);
		if (bStaleNative && bStaleNative->GetPropertyValue_InContainer(ElemPtr)) { continue; }

		UObject* StartEdge = StartEdgeObjProp->GetObjectPropertyValue_InContainer(ElemPtr);
		UObject* EndEdge   = EndEdgeObjProp->GetObjectPropertyValue_InContainer(ElemPtr);
		if (!StartEdge || !EndEdge) { continue; }

		TArray<int32> StartHandles, EndHandles;
		CollectLaneHandles(StartEdge, StartHandles);
		CollectLaneHandles(EndEdge, EndHandles);

		// Connect StartEdge lanes ↔ EndEdge lanes (bidirectional).
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
		for (const int32 Src : EndHandles)
		{
			TArray<FTrafficLaneHandle>& Conns = LaneConnectionMap.FindOrAdd(Src);
			for (const int32 Dst : StartHandles)
			{
				if (Src != Dst)
				{
					Conns.AddUnique(FTrafficLaneHandle(Dst));
					++CornerConnections;
				}
			}
		}

		UE_LOG(LogAAATraffic, Log,
			TEXT("  TArray Corner[%d]: StartEdge=%s EndEdge=%s StartLanes=[%s] EndLanes=[%s]"),
			i,
			*StartEdge->GetName(), *EndEdge->GetName(),
			*FString::JoinBy(StartHandles, TEXT(","),
				[](int32 H) { return FString::FromInt(H); }),
			*FString::JoinBy(EndHandles, TEXT(","),
				[](int32 H) { return FString::FromInt(H); }));
	}

	UE_LOG(LogAAATraffic, Log,
		TEXT("RoadBLDReflectionProvider: TArray corner connectivity added %d links from %d corners."),
		CornerConnections, NumCorners);
}

// ---------------------------------------------------------------------------
// CacheLaneEndpoints — pre-compute geometry for every lane
// ---------------------------------------------------------------------------

void URoadBLDReflectionProvider::CacheLaneEndpoints()
{
	LaneEndpointMap.Empty();

	// Sort lane handles deterministically.
	TArray<int32> SortedHandles;
	LaneHandleMap.GetKeys(SortedHandles);
	SortedHandles.Sort();

	for (const int32 HandleId : SortedHandles)
	{
		TArray<FVector> Points;
		float Width = 0.0f;

		// Call the base GetLanePath which resolves polyline from reflection.
		// At this point VirtualLaneMap is empty so it will use original path logic.
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
		TEXT("RoadBLDReflectionProvider: Cached endpoints for %d lanes, road widths for %d roads."),
		LaneEndpointMap.Num(), RoadTotalWidthMap.Num());

	// ── DIAG: Dump per-lane endpoint positions to verify opposing lanes
	// have distinct center lines (not sharing road centerline). ──
	for (const int32 HandleId : SortedHandles)
	{
		const FLaneEndpointCache* EP = LaneEndpointMap.Find(HandleId);
		if (!EP) { continue; }
		const int32* Road = LaneToRoadHandleMap.Find(HandleId);
		UE_LOG(LogAAATraffic, Warning,
			TEXT("JDIAG LANE-ENDPOINTS: Lane=%d Road=%d Width=%.1f "
				 "Start=(%.1f,%.1f,%.1f) End=(%.1f,%.1f,%.1f) "
				 "Dir=(%.3f,%.3f,%.3f)"),
			HandleId, Road ? *Road : -1, EP->Width,
			EP->StartPos.X, EP->StartPos.Y, EP->StartPos.Z,
			EP->EndPos.X, EP->EndPos.Y, EP->EndPos.Z,
			EP->StartDir.X, EP->StartDir.Y, EP->StartDir.Z);
	}

	RebuildRoadSpeedClassification();
}

// ---------------------------------------------------------------------------
// SetSpeedTiers — update classified speed tiers and rebuild the road speed map.
// Called by TrafficSpawner after the provider is ready.
// ---------------------------------------------------------------------------

void URoadBLDReflectionProvider::SetSpeedTiers(float InResidentialSpeed, float InUrbanSpeed, float InHighwaySpeed)
{
	ConfiguredResidentialSpeed = InResidentialSpeed;
	ConfiguredUrbanSpeed = InUrbanSpeed;
	ConfiguredHighwaySpeed = InHighwaySpeed;

	RebuildRoadSpeedClassification();

	UE_LOG(LogAAATraffic, Log,
		TEXT("RoadBLDReflectionProvider: SetSpeedTiers — Residential=%.0f  Urban=%.0f  Highway=%.0f — reclassified %d roads."),
		ConfiguredResidentialSpeed, ConfiguredUrbanSpeed, ConfiguredHighwaySpeed,
		RoadClassifiedSpeedLimits.Num());
}

// ---------------------------------------------------------------------------
// RebuildRoadSpeedClassification — classify speed by lane count, then cap
// each road at the physics-safe speed for its tightest curve.
//
// Step 1: Assign tier speed based on lane count (residential / urban / highway).
// Step 2: For each road, walk every lane's cached polyline and measure
//         curvature in a sliding window.  Compute the minimum safe speed
//         from v = sqrt(0.3g × R) and cap the road's classified speed.
//
// This means a 2-lane residential road with a hairpin gets a LOWER speed
// than a straight 2-lane road, matching how real speed advisory signs work.
// ---------------------------------------------------------------------------

void URoadBLDReflectionProvider::RebuildRoadSpeedClassification()
{
	RoadClassifiedSpeedLimits.Empty();

	// Step 1: Classify by lane count.
	for (const auto& RoadEntry : RoadToLaneHandles)
	{
		const int32 RoadId = RoadEntry.Key;
		const int32 LaneCount = RoadEntry.Value.Num();
		float ClassifiedSpeed;
		if (LaneCount <= 2)
		{
			ClassifiedSpeed = ConfiguredResidentialSpeed;
		}
		else if (LaneCount <= 4)
		{
			ClassifiedSpeed = ConfiguredUrbanSpeed;
		}
		else
		{
			ClassifiedSpeed = ConfiguredHighwaySpeed;
		}
		RoadClassifiedSpeedLimits.Add(RoadId, ClassifiedSpeed);
	}

	// Step 2: Cap each road's speed at the geometry-safe limit.
	// For every lane on this road, walk the cached polyline with a
	// sliding window, find the tightest turning radius, and compute
	// the maximum safe speed: v = sqrt(lateralAccelBudget × R).
	constexpr float LatAccelBudget = 294.0f; // 0.3g in cm/s²
	constexpr int32 WindowSize = 10;          // ~10m at 100cm samples
	constexpr float MinRadius = 50.0f;        // 0.5m clamp
	constexpr float MinAngleDeg = 3.0f;       // ignore nearly-straight windows

	int32 CappedCount = 0;
	for (auto& SpeedEntry : RoadClassifiedSpeedLimits)
	{
		const int32 RoadId = SpeedEntry.Key;
		float& RoadSpeed = SpeedEntry.Value;

		const TArray<int32>* LaneHandles = RoadToLaneHandles.Find(RoadId);
		if (!LaneHandles) { continue; }

		float TightestRadius = TNumericLimits<float>::Max();

		for (const int32 LaneId : *LaneHandles)
		{
			const FLaneEndpointCache* Cache = LaneEndpointMap.Find(LaneId);
			if (!Cache || Cache->Polyline.Num() < 3) { continue; }

			const TArray<FVector>& Poly = Cache->Polyline;

			// Sliding window: measure curvature in each window of WindowSize segments.
			for (int32 WinStart = 0; WinStart < Poly.Num() - 2; ++WinStart)
			{
				float WinAngleDeg = 0.0f;
				float WinArc = 0.0f;
				const int32 WinEnd = FMath::Min(WinStart + WindowSize, Poly.Num() - 2);

				for (int32 i = WinStart; i < WinEnd; ++i)
				{
					const FVector Seg0 = Poly[i + 1] - Poly[i];
					const FVector Seg1 = (i + 2 < Poly.Num())
						? (Poly[i + 2] - Poly[i + 1])
						: Seg0;
					WinArc += Seg0.Size();
					const FVector Dir0 = Seg0.GetSafeNormal();
					const FVector Dir1 = Seg1.GetSafeNormal();
					if (!Dir0.IsNearlyZero() && !Dir1.IsNearlyZero())
					{
						const float Dot = FMath::Clamp(FVector::DotProduct(Dir0, Dir1), -1.0f, 1.0f);
						WinAngleDeg += FMath::RadiansToDegrees(FMath::Acos(Dot));
					}
				}

				if (WinAngleDeg > MinAngleDeg && WinArc > KINDA_SMALL_NUMBER)
				{
					const float WinAngleRad = FMath::DegreesToRadians(WinAngleDeg);
					const float Radius = FMath::Max(WinArc / WinAngleRad, MinRadius);
					TightestRadius = FMath::Min(TightestRadius, Radius);
				}
			}
		}

		if (TightestRadius < TNumericLimits<float>::Max())
		{
			const float GeometrySpeedLimit = FMath::Sqrt(LatAccelBudget * TightestRadius);
			if (GeometrySpeedLimit < RoadSpeed)
			{
				RoadSpeed = GeometrySpeedLimit;
				++CappedCount;
			}
		}
	}

	UE_LOG(LogAAATraffic, Log,
		TEXT("RoadBLDReflectionProvider: Classified speed limits for %d roads (%d capped by curve geometry)."),
		RoadClassifiedSpeedLimits.Num(), CappedCount);
}

// ---------------------------------------------------------------------------
// DetectAndSplitThroughRoads — create virtual lane segments at intersection
// mask boundaries.  A road with 2+ masks is a through-road; its lanes are
// split so each intersection segment gets its own junction mapping.
// Requires: CachedIntersectionMasks populated (BuildMaskBasedIntersections).
// ---------------------------------------------------------------------------

void URoadBLDReflectionProvider::DetectAndSplitThroughRoads()
{
	VirtualLaneMap.Empty();
	OriginalToVirtualMap.Empty();
	ReplacedLaneHandles.Empty();
	VirtualLaneToGroupId.Empty();
	IntersectionEntryPointMap.Empty();

	if (CachedIntersectionMasks.Num() == 0)
	{
		UE_LOG(LogAAATraffic, Log,
			TEXT("RoadBLDReflectionProvider: No intersection masks — through-road splitting skipped."));
		return;
	}

	// ── Step 1: Group masks by ParentRoad UObject ───────────────
	// Build: RoadActor → list of mask indices (sorted by StartDistance).
	TMap<UObject*, TArray<int32>> RoadToMaskIndices;
	for (int32 i = 0; i < CachedIntersectionMasks.Num(); ++i)
	{
		UObject* Road = CachedIntersectionMasks[i].ParentRoad.Get();
		if (Road)
		{
			RoadToMaskIndices.FindOrAdd(Road).Add(i);
		}
	}

	// Sort each road's mask list by StartDistance for deterministic ordering.
	for (auto& Pair : RoadToMaskIndices)
	{
		Pair.Value.Sort([this](const int32 A, const int32 B)
		{
			return CachedIntersectionMasks[A].StartDistance < CachedIntersectionMasks[B].StartDistance;
		});
	}

	// ── Step 2: Identify roads that need splitting (1+ masks) ────
	// Any road with at least one intersection mask must be split into
	// virtual segments so that only the mask-covered portion is tagged
	// as a junction lane. Without splitting, BuildJunctionGrouping maps
	// ALL lanes on the road to the junction, preventing vehicle spawning
	// on the free-flow portions.
	// Also need: road handle for each road actor (reverse lookup).
	TMap<UObject*, int32> RoadActorToHandle;
	for (const auto& Pair : RoadHandleMap)
	{
		UObject* Actor = Pair.Value.Get();
		if (Actor)
		{
			RoadActorToHandle.Add(Actor, Pair.Key);
		}
	}

	int32 TotalVirtuals = 0;
	int32 ThroughRoadCount = 0;

	for (const auto& RMPair : RoadToMaskIndices)
	{
		UObject* RoadActor = RMPair.Key;
		const TArray<int32>& MaskIndices = RMPair.Value;

		// Split any road that has at least one intersection mask.
		if (MaskIndices.IsEmpty()) { continue; }

		const int32* RoadHandlePtr = RoadActorToHandle.Find(RoadActor);
		if (!RoadHandlePtr) { continue; }
		const int32 RoadHandle = *RoadHandlePtr;

		const TArray<int32>* LaneHandles = RoadToLaneHandles.Find(RoadHandle);
		if (!LaneHandles || LaneHandles->Num() == 0) { continue; }

		const double RoadLength = GetRoadLength(RoadActor);
		if (RoadLength <= 0.0) { continue; }

		++ThroughRoadCount;

		UE_LOG(LogAAATraffic, Log,
			TEXT("  Splitting road %s (handle=%d, length=%.1f): %d mask(s)"),
			*RoadActor->GetName(), RoadHandle, RoadLength, MaskIndices.Num());

		// ── Step 3: For each lane on this road, compute split indices ──
		for (const int32 LaneHandle : *LaneHandles)
		{
			const FLaneEndpointCache* OrigCache = LaneEndpointMap.Find(LaneHandle);
			if (!OrigCache || OrigCache->Polyline.Num() < 3) { continue; }

			const TArray<FVector>& Poly = OrigCache->Polyline;
			const int32 TotalPoints = Poly.Num();

			// Build split-point list from mask StartDistance/EndDistance.
			// Each mask produces TWO split points: one at its start, one at its end.
			// This creates segments: [road-start..mask1-start], [mask1-start..mask1-end],
			// [mask1-end..mask2-start], [mask2-start..mask2-end], ... [maskN-end..road-end]
			struct FSplitPoint
			{
				int32 PolyIndex;
				int32 GroupId; // >0 = start of intersection segment, 0 = start of free segment
				double MaskDistance; // exact reference-line distance for this boundary (cm)
			};
			TArray<FSplitPoint> SplitPoints;

			// FIX: Detect if this lane's polyline is reversed relative to
			// the reference line. GetLanePath() reverses polylines for
			// opposite-direction lanes so vehicles drive correctly, but
			// mask distances are always in reference-line order. Without
			// mirroring, split indices land at the wrong polyline position
			// — placing the intersection segment on the wrong portion of
			// the road for reversed lanes.
			const bool bIsReversedLane = ReversedLaneSet.Contains(LaneHandle);

			for (const int32 MaskIdx : MaskIndices)
			{
				const FIntersectionMaskInfo& Mask = CachedIntersectionMasks[MaskIdx];

				// Convert reference-line distance to polyline index.
				// Polyline is sampled uniformly: point[i] = distance (RoadLength * i) / (NumSamples - 1)
				// So: index = round(Distance / RoadLength * (TotalPoints - 1))
				const float StartNorm = static_cast<float>(Mask.StartDistance / RoadLength);
				const float EndNorm = static_cast<float>(Mask.EndDistance / RoadLength);

				int32 StartIdx, EndIdx;
				double SplitMaskDistStart, SplitMaskDistEnd;

				if (bIsReversedLane)
				{
					// Reversed lane: Poly[0] = ref-END, Poly[N-1] = ref-START.
					// Mirror the mapping: refDist D → idx = (N-1) - round(D/L*(N-1)).
					// StartDistance (lower ref-line value) maps to a HIGHER poly index,
					// EndDistance (higher ref-line value) maps to a LOWER poly index.
					const int32 MaxIdx = TotalPoints - 1;
					const int32 RawStart = FMath::RoundToInt32(StartNorm * static_cast<float>(MaxIdx));
					const int32 RawEnd = FMath::RoundToInt32(EndNorm * static_cast<float>(MaxIdx));
					StartIdx = MaxIdx - RawEnd;     // EndDist → lower poly index (earlier in travel)
					EndIdx   = MaxIdx - RawStart;   // StartDist → higher poly index (later in travel)

					// Boundary distances: for the reversed vehicle approaching from
					// ref-END, the first intersection boundary it encounters is at
					// Mask.EndDistance on the reference line (StartIdx position).
					// The far boundary (where intersection ends) is at Mask.StartDistance.
					SplitMaskDistStart = Mask.EndDistance;
					SplitMaskDistEnd = Mask.StartDistance;
				}
				else
				{
					// Non-reversed: polyline matches reference-line order.
					StartIdx = FMath::RoundToInt32(StartNorm * static_cast<float>(TotalPoints - 1));
					EndIdx = FMath::RoundToInt32(EndNorm * static_cast<float>(TotalPoints - 1));
					SplitMaskDistStart = Mask.StartDistance;
					SplitMaskDistEnd = Mask.EndDistance;
				}

				// Clamp to valid range (never at very first or last point).
				StartIdx = FMath::Clamp(StartIdx, 1, TotalPoints - 2);
				EndIdx = FMath::Clamp(EndIdx, StartIdx + 1, TotalPoints - 1);

				// Mark start of intersection segment and start of free segment after it.
				SplitPoints.Add({ StartIdx, Mask.GroupId, SplitMaskDistStart });
				if (EndIdx < TotalPoints - 1)
				{
					SplitPoints.Add({ EndIdx, 0, SplitMaskDistEnd }); // Free segment starts after intersection
				}
				// If EndIdx == TotalPoints - 1, the mask extends to the very end
				// of the road. No free segment split is added because there is no
				// road beyond this point. The final virtual segment (from StartIdx
				// to TotalPoints-1) will carry the mask's GroupId, which is the
				// correct behavior — the road ends inside the intersection.

				UE_LOG(LogAAATraffic, Log,
					TEXT("    Lane %d: Mask group=%d StartDist=%.1f EndDist=%.1f → idx=[%d..%d] reversed=%s"),
					LaneHandle, Mask.GroupId, Mask.StartDistance, Mask.EndDistance,
					StartIdx, EndIdx, bIsReversedLane ? TEXT("YES") : TEXT("NO"));
			}

			// Sort split points by poly index, then by GroupId descending
			// so intersection starts (GroupId > 0) are preferred over free
			// starts (GroupId == 0) when two split points collide at the
			// same quantized poly index.
			SplitPoints.Sort([](const FSplitPoint& A, const FSplitPoint& B)
			{
				if (A.PolyIndex != B.PolyIndex) { return A.PolyIndex < B.PolyIndex; }
				// Prefer intersection (GroupId > 0) over free (GroupId == 0).
				return A.GroupId > B.GroupId;
			});

			// Remove duplicates at the same poly index, keeping the first
			// entry (which, after sorting, is the intersection-start if any).
			for (int32 i = SplitPoints.Num() - 1; i > 0; --i)
			{
				if (SplitPoints[i].PolyIndex == SplitPoints[i - 1].PolyIndex)
				{
					SplitPoints.RemoveAt(i);
				}
			}

			if (SplitPoints.Num() == 0) { continue; }

			// ── Step 4: Create virtual segments ──────────────
			// Resolve lane edge curves + refline for exact entry-point computation.
			const FReflectionLaneData* LData = LaneHandleMap.Find(LaneHandle);
			UObject* LaneRoadActor = LData ? LData->RoadActor.Get() : nullptr;
			UObject* LaneRefLine = LaneRoadActor ? GetReferenceLine(LaneRoadActor) : nullptr;
			UObject* LaneLeftEdge = LData ? LData->LeftEdge.Get() : nullptr;
			UObject* LaneRightEdge = LData ? LData->RightEdge.Get() : nullptr;
			const bool bCanComputeEntryPoint = LaneLeftEdge && LaneRightEdge && LaneRefLine
				&& LaneRoadActor && Get3DPosFunc && ConvertDistFunc;

			TArray<int32> VirtualHandles;
			TArray<int32> VirtualGroupIds; // parallel array: group ID per virtual segment
			TArray<double> VirtualBoundaryDistances; // parallel: mask distance at end of each segment (0 if N/A)
			int32 PrevStart = 0;
			int32 PrevGroupId = 0; // First segment before any intersection = free

			for (int32 s = 0; s <= SplitPoints.Num(); ++s)
			{
				const int32 SegEnd = (s < SplitPoints.Num())
					? SplitPoints[s].PolyIndex
					: (TotalPoints - 1);

				if (SegEnd <= PrevStart) { PrevStart = SegEnd; continue; }

				const int32 VirtualId = NextHandleId++;
				FVirtualLaneInfo VInfo;
				VInfo.OriginalLaneHandle = LaneHandle;
				VInfo.StartPointIndex = PrevStart;
				VInfo.EndPointIndex = SegEnd;
				VirtualLaneMap.Add(VirtualId, VInfo);
				VirtualHandles.Add(VirtualId);
				VirtualGroupIds.Add(PrevGroupId);

				// Store the exact mask distance at this segment's end boundary.
				// For segments ending at a split point, this is the split point's MaskDistance.
				// For the final segment, there's no boundary.
				const double BoundaryDist = (s < SplitPoints.Num())
					? SplitPoints[s].MaskDistance : 0.0;
				VirtualBoundaryDistances.Add(BoundaryDist);

				// Tag this virtual lane with its group (0 = free, >0 = intersection).
				VirtualLaneToGroupId.Add(VirtualId, PrevGroupId);

				// Cache endpoint data for the virtual segment.
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

				UE_LOG(LogAAATraffic, Log,
					TEXT("      Virtual %d: points [%d..%d], group=%d (%s)"),
					VirtualId, PrevStart, SegEnd, PrevGroupId,
					PrevGroupId > 0 ? TEXT("intersection") : TEXT("free-flow"));

				PrevStart = SegEnd;
				// Next segment's group: if this was the split point, use its GroupId.
				PrevGroupId = (s < SplitPoints.Num()) ? SplitPoints[s].GroupId : 0;
			}

			// ── Step 5: Compute exact intersection entry points ──
			// For each free-flow virtual segment whose next segment is an
			// intersection, derive the precise 3D boundary position from
			// MaskDistance via continuous edge-curve evaluation (no polyline
			// quantization error).
			for (int32 v = 0; v < VirtualHandles.Num() - 1; ++v)
			{
				const int32 ThisGroup = VirtualGroupIds[v];
				const int32 NextGroup = VirtualGroupIds[v + 1];

				if (ThisGroup == 0 && NextGroup > 0 && bCanComputeEntryPoint)
				{
					// This free-flow segment precedes an intersection.
					// The boundary distance is stored in VirtualBoundaryDistances[v].
					const double BoundaryDist = VirtualBoundaryDistances[v];
					if (BoundaryDist > 0.0)
					{
						// Convert reference-line distance to edge-curve distances.
						const double LeftDist = ConvertDistanceBetweenCurves(
							LaneRoadActor, LaneRefLine, LaneLeftEdge, BoundaryDist);
						const double RightDist = ConvertDistanceBetweenCurves(
							LaneRoadActor, LaneRefLine, LaneRightEdge, BoundaryDist);

						// Sample exact 3D positions on both lane edges.
						const FVector LeftPos = Get3DPositionAtDistance(LaneLeftEdge, LaneRefLine, LeftDist);
						const FVector RightPos = Get3DPositionAtDistance(LaneRightEdge, LaneRefLine, RightDist);

						// Lane centerline at the intersection boundary.
						const FVector EntryPoint = (LeftPos + RightPos) * 0.5f;

						// Store on the VirtualLaneInfo.
						FVirtualLaneInfo* VInfo = VirtualLaneMap.Find(VirtualHandles[v]);
						if (VInfo)
						{
							VInfo->IntersectionEntryPoint = EntryPoint;
							VInfo->bHasIntersectionEntryPoint = true;
						}

						// Store in the lookup map for the provider interface.
						IntersectionEntryPointMap.Add(VirtualHandles[v], EntryPoint);

						UE_LOG(LogAAATraffic, Log,
							TEXT("      Virtual %d: intersection entry point computed at MaskDist=%.1f → (%.1f, %.1f, %.1f)"),
							VirtualHandles[v], BoundaryDist, EntryPoint.X, EntryPoint.Y, EntryPoint.Z);
					}
				}
			}

			if (VirtualHandles.Num() > 1)
			{
				ReplacedLaneHandles.Add(LaneHandle);
				OriginalToVirtualMap.Add(LaneHandle, MoveTemp(VirtualHandles));
				TotalVirtuals += OriginalToVirtualMap[LaneHandle].Num();

				// Connect consecutive virtual segments internally.
				const TArray<int32>& Virtuals = OriginalToVirtualMap[LaneHandle];
				for (int32 v = 0; v < Virtuals.Num() - 1; ++v)
				{
					TArray<FTrafficLaneHandle>& Conns = LaneConnectionMap.FindOrAdd(Virtuals[v]);
					Conns.AddUnique(FTrafficLaneHandle(Virtuals[v + 1]));
				}
			}
			else
			{
				// Only one segment — splitting didn't help. Clean up.
				// NOTE: NextHandleId is NOT rolled back. Handle IDs are opaque
				// monotonic identifiers; gaps in the sequence are acceptable and
				// have no functional impact. Rolling back would risk collisions
				// if other code had already observed the incremented value.
				for (const int32 VH : VirtualHandles)
				{
					VirtualLaneMap.Remove(VH);
					LaneEndpointMap.Remove(VH);
					VirtualLaneToGroupId.Remove(VH);
				}
			}
		}
	}

	// ── Remap pre-existing corner-based connections to virtual handles ──
	// BuildLaneConnectivity ran BEFORE splitting and wrote turn connections
	// keyed by original lane handles. Those originals are now in
	// ReplacedLaneHandles — no vehicle will ever drive on them. We must
	// FIX: Use positional proximity instead of blind first/last assignment.
	// Corner connections are at specific physical locations — a corner at
	// the ref-line END should route to the virtual segment nearest that
	// position, not always to the last/first virtual. This matters for
	// reversed lanes where polyline order is opposite to ref-line order,
	// and for roads with junctions at both ends.
	int32 RemappedOutgoing = 0;
	int32 RemappedIncoming = 0;

	for (const int32 OrigHandle : ReplacedLaneHandles)
	{
		const TArray<int32>* Virtuals = OriginalToVirtualMap.Find(OrigHandle);
		if (!Virtuals || Virtuals->Num() == 0) { continue; }

		const int32 FirstVirtual = (*Virtuals)[0];
		const int32 LastVirtual  = (*Virtuals).Last();

		// --- Outgoing: assign each connection to the virtual whose EndPos
		//     is physically closest to the destination lane's StartPos. ---
		if (const TArray<FTrafficLaneHandle>* OldConns = LaneConnectionMap.Find(OrigHandle))
		{
			// Copy because we remove the key below.
			TArray<FTrafficLaneHandle> OldConnsCopy = *OldConns;
			LaneConnectionMap.Remove(OrigHandle);

			for (const FTrafficLaneHandle& Dst : OldConnsCopy)
			{
				// Skip connections to other virtuals of the same original
				// (internal chain is already set up above).
				if (Virtuals->Contains(Dst.HandleId)) { continue; }

				// Find which virtual's EndPos is closest to the destination's StartPos.
				const FLaneEndpointCache* DstCache = LaneEndpointMap.Find(Dst.HandleId);
				int32 BestVirtual = LastVirtual; // fallback
				if (DstCache)
				{
					float BestDistSq = MAX_FLT;
					for (const int32 VH : *Virtuals)
					{
						const FLaneEndpointCache* VCache = LaneEndpointMap.Find(VH);
						if (!VCache) { continue; }
						const float DSq = FVector::DistSquared(VCache->EndPos, DstCache->StartPos);
						if (DSq < BestDistSq)
						{
							BestDistSq = DSq;
							BestVirtual = VH;
						}
					}
				}

				LaneConnectionMap.FindOrAdd(BestVirtual).AddUnique(Dst);
				++RemappedOutgoing;
			}
		}

		// --- Incoming: rewrite any connection that targets the original ---
		// Find which virtual's StartPos is closest to the source lane's EndPos.
		for (auto& ConnPair : LaneConnectionMap)
		{
			TArray<FTrafficLaneHandle>& DstList = ConnPair.Value;
			for (FTrafficLaneHandle& Dst : DstList)
			{
				if (Dst.HandleId == OrigHandle)
				{
					const FLaneEndpointCache* SrcCache = LaneEndpointMap.Find(ConnPair.Key);
					int32 BestVirtual = FirstVirtual; // fallback
					if (SrcCache)
					{
						float BestDistSq = MAX_FLT;
						for (const int32 VH : *Virtuals)
						{
							const FLaneEndpointCache* VCache = LaneEndpointMap.Find(VH);
							if (!VCache) { continue; }
							const float DSq = FVector::DistSquared(VCache->StartPos, SrcCache->EndPos);
							if (DSq < BestDistSq)
							{
								BestDistSq = DSq;
								BestVirtual = VH;
							}
						}
					}
					Dst = FTrafficLaneHandle(BestVirtual);
					++RemappedIncoming;
				}
			}
		}
	}

	// Sort all connection lists for determinism after remapping.
	for (auto& Pair : LaneConnectionMap)
	{
		Pair.Value.Sort([](const FTrafficLaneHandle& A, const FTrafficLaneHandle& B)
		{
			return A.HandleId < B.HandleId;
		});
	}

	UE_LOG(LogAAATraffic, Log,
		TEXT("RoadBLDReflectionProvider: Road splitting — %d roads split, %d virtual segments from %d source lanes. "
			 "Remapped %d outgoing + %d incoming corner connections to virtual handles."),
		ThroughRoadCount, TotalVirtuals, ReplacedLaneHandles.Num(),
		RemappedOutgoing, RemappedIncoming);
}

// ---------------------------------------------------------------------------
// BuildMaskBasedIntersections — read IntersectionMasks from RoadBLD and
// group them into physical intersection clusters for junction assignment
// ---------------------------------------------------------------------------

void URoadBLDReflectionProvider::BuildMaskBasedIntersections(UWorld* World)
{
	CachedIntersectionMasks.Empty();
	IntersectionGroupCentroids.Empty();

	if (!DynNetworkClass || !World)
	{
		return;
	}

	// Find the DynamicRoadNetwork actor deterministically.
	AActor* NetworkActor = nullptr;
	{
		TArray<AActor*> Candidates;
		for (FActorIterator It(World); It; ++It)
		{
			if (It->IsA(DynNetworkClass))
			{
				Candidates.Add(*It);
			}
		}
		if (Candidates.Num() > 0)
		{
			Candidates.Sort([](const AActor& A, const AActor& B) { return A.GetName() < B.GetName(); });
			NetworkActor = Candidates[0];
		}
	}
	if (!NetworkActor)
	{
		UE_LOG(LogAAATraffic, Log,
			TEXT("RoadBLDReflectionProvider: No DynamicRoadNetwork actor — mask-based intersections skipped."));
		return;
	}

	// ── Step 1: Read IntersectionMasks ──────────────────────────
	FArrayProperty* MasksArr = CastField<FArrayProperty>(
		DynNetworkClass->FindPropertyByName(TEXT("IntersectionMasks")));
	if (!MasksArr)
	{
		UE_LOG(LogAAATraffic, Log,
			TEXT("RoadBLDReflectionProvider: IntersectionMasks property not found — mask-based intersections skipped."));
		return;
	}

	FScriptArrayHelper MaskHelper(MasksArr, MasksArr->ContainerPtrToValuePtr<void>(NetworkActor));
	const int32 NumMasks = MaskHelper.Num();
	if (NumMasks == 0)
	{
		UE_LOG(LogAAATraffic, Log,
			TEXT("RoadBLDReflectionProvider: IntersectionMasks is empty — no intersections to group."));
		return;
	}

	FStructProperty* MaskStructProp = CastField<FStructProperty>(MasksArr->Inner);
	if (!MaskStructProp) { return; }

	UScriptStruct* MaskStruct = MaskStructProp->Struct;
	FDoubleProperty* MaskSDProp = CastField<FDoubleProperty>(MaskStruct->FindPropertyByName(TEXT("StartDistance")));
	FDoubleProperty* MaskEDProp = CastField<FDoubleProperty>(MaskStruct->FindPropertyByName(TEXT("EndDistance")));
	FObjectPropertyBase* MaskPRObj = CastField<FObjectPropertyBase>(MaskStruct->FindPropertyByName(TEXT("ParentRoad")));
	FIntProperty* MaskSCIProp = CastField<FIntProperty>(MaskStruct->FindPropertyByName(TEXT("StartCutIndex")));
	FIntProperty* MaskECIProp = CastField<FIntProperty>(MaskStruct->FindPropertyByName(TEXT("EndCutIndex")));

	if (!MaskSDProp || !MaskEDProp || !MaskPRObj || !MaskSCIProp || !MaskECIProp)
	{
		UE_LOG(LogAAATraffic, Warning,
			TEXT("RoadBLDReflectionProvider: IntersectionMask struct missing expected properties (SD=%d ED=%d PR=%d SCI=%d ECI=%d)."),
			!!MaskSDProp, !!MaskEDProp, !!MaskPRObj, !!MaskSCIProp, !!MaskECIProp);
		return;
	}

	// ── Step 2: Read RoadNetworkPerimeterCuts (optional) ────────
	// PerimeterCuts are used for Signal A (corner-index grouping) but are
	// NOT required for mask-based intersection detection to work. If cuts
	// are unavailable, we skip Signal A and group by world-position only.
	FArrayProperty* CutsArr = CastField<FArrayProperty>(
		DynNetworkClass->FindPropertyByName(TEXT("RoadNetworkPerimeterCuts")));

	int32 NumCuts = 0;
	FScriptArrayHelper* CutHelperPtr = nullptr;
	FScriptArrayHelper CutHelperStorage(CutsArr, CutsArr ? CutsArr->ContainerPtrToValuePtr<void>(NetworkActor) : nullptr);
	FIntProperty* CutLCIProp = nullptr;
	FIntProperty* CutRCIProp = nullptr;

	if (CutsArr)
	{
		CutHelperPtr = &CutHelperStorage;
		NumCuts = CutHelperStorage.Num();

		FStructProperty* CutStructProp = CastField<FStructProperty>(CutsArr->Inner);
		if (CutStructProp)
		{
			UScriptStruct* CutStruct = CutStructProp->Struct;
			CutLCIProp = CastField<FIntProperty>(CutStruct->FindPropertyByName(TEXT("LeftCornerIndex")));
			CutRCIProp = CastField<FIntProperty>(CutStruct->FindPropertyByName(TEXT("RightCornerIndex")));
		}
	}

	if (NumCuts == 0)
	{
		UE_LOG(LogAAATraffic, Log,
			TEXT("RoadBLDReflectionProvider: RoadNetworkPerimeterCuts unavailable or empty — Signal A (corner grouping) will be skipped; grouping by world-position only."));
	}

	// ── Step 3: Parse masks, collect corner indices, compute world positions ─
	// We compute the world-space centroid for each mask here (needed for
	// grouping in Step 4). Corner indices are also collected as a supplementary
	// signal but may be all -1 in practice (confirmed from FastPreview rebuilds).
	TArray<FVector> MaskWorldPositions;
	MaskWorldPositions.SetNum(NumMasks);

	for (int32 i = 0; i < NumMasks; ++i)
	{
		const uint8* MaskPtr = MaskHelper.GetRawPtr(i);

		FIntersectionMaskInfo MaskInfo;
		MaskInfo.StartDistance = MaskSDProp->GetPropertyValue_InContainer(MaskPtr);
		MaskInfo.EndDistance   = MaskEDProp->GetPropertyValue_InContainer(MaskPtr);
		MaskInfo.ParentRoad    = MaskPRObj->GetObjectPropertyValue_InContainer(MaskPtr);
		MaskInfo.StartCutIndex = MaskSCIProp->GetPropertyValue_InContainer(MaskPtr);
		MaskInfo.EndCutIndex   = MaskECIProp->GetPropertyValue_InContainer(MaskPtr);

		// Collect all unique corner indices from this mask's cuts.
		// Handle inverted ranges (StartCutIndex > EndCutIndex).
		// Only available when perimeter cuts were successfully read.
		if (NumCuts > 0 && CutLCIProp && CutRCIProp)
		{
			const int32 CutLo = FMath::Min(MaskInfo.StartCutIndex, MaskInfo.EndCutIndex);
			const int32 CutHi = FMath::Max(MaskInfo.StartCutIndex, MaskInfo.EndCutIndex);
			for (int32 CutIdx = CutLo; CutIdx <= CutHi && CutIdx < NumCuts; ++CutIdx)
			{
				if (CutIdx < 0) { continue; }
				const uint8* CutPtr = CutHelperPtr->GetRawPtr(CutIdx);
				const int32 LCI = CutLCIProp->GetPropertyValue_InContainer(CutPtr);
				const int32 RCI = CutRCIProp->GetPropertyValue_InContainer(CutPtr);
				if (LCI >= 0) { MaskInfo.CornerIndices.AddUnique(LCI); }
				if (RCI >= 0) { MaskInfo.CornerIndices.AddUnique(RCI); }
			}
		}

		// Compute world-space position at mask midpoint for grouping.
		FVector WorldPos = FVector::ZeroVector;
		UObject* RoadActor = MaskInfo.ParentRoad.Get();
		if (RoadActor)
		{
			const double MidDist = (MaskInfo.StartDistance + MaskInfo.EndDistance) * 0.5;
			UObject* RefLine = GetReferenceLine(RoadActor);
			if (RefLine && Get3DPosFunc)
			{
				WorldPos = Get3DPositionAtDistance(RefLine, RefLine, MidDist);
			}
		}
		MaskWorldPositions[i] = WorldPos;

		UE_LOG(LogAAATraffic, Log,
			TEXT("  Mask[%d]: Road=%s Dist=[%.1f, %.1f] Cuts=[%d, %d] Corners=%d WorldPos=(%.1f, %.1f, %.1f)"),
			i,
			MaskInfo.ParentRoad.IsValid() ? *MaskInfo.ParentRoad->GetName() : TEXT("NULL"),
			MaskInfo.StartDistance, MaskInfo.EndDistance,
			MaskInfo.StartCutIndex, MaskInfo.EndCutIndex,
			MaskInfo.CornerIndices.Num(),
			WorldPos.X, WorldPos.Y, WorldPos.Z);

		CachedIntersectionMasks.Add(MoveTemp(MaskInfo));
	}

	// ── Step 4: Group masks using two complementary signals ─────
	//
	// Signal A — Corner indices (structural):
	//   Two masks sharing a corner index belong to the same intersection.
	//   Currently all PerimeterCut corner indices return -1 at runtime,
	//   so this signal may produce 0 links. Kept for forward compatibility.
	//
	// Signal B — World-position proximity:
	//   Masks at the same physical intersection have nearby X,Y world
	//   positions. For straight crossroads the midpoints are nearly identical,
	//   but for angled or roundabout junctions, road reference-line offsets
	//   can separate them by up to ~500 cm. Uses 800 cm proximity threshold.
	//
	// Both signals feed into the same union-find.

	// Filter: exclude masks with no valid ParentRoad (phantom/orphan masks).
	TArray<bool> MaskValid;
	MaskValid.SetNumUninitialized(NumMasks);
	int32 ValidCount = 0;
	for (int32 k = 0; k < NumMasks; ++k)
	{
		MaskValid[k] = CachedIntersectionMasks[k].ParentRoad.IsValid();
		if (!MaskValid[k])
		{
			UE_LOG(LogAAATraffic, Warning,
				TEXT("  Mask[%d] has NULL ParentRoad — excluded from grouping"), k);
		}
		else
		{
			++ValidCount;
		}
	}
	UE_LOG(LogAAATraffic, Log, TEXT("  %d/%d masks valid for grouping"), ValidCount, NumMasks);

	TArray<int32> Parent;
	Parent.SetNumUninitialized(NumMasks);
	for (int32 k = 0; k < NumMasks; ++k) { Parent[k] = k; }

	// Iterative find with path compression — avoids unbounded recursion
	// if the parent graph were to contain cycles due to a bug.
	auto Find = [&Parent](int32 X) -> int32
	{
		int32 Root = X;
		while (Parent[Root] != Root) { Root = Parent[Root]; }

		// Path compression pass.
		int32 Current = X;
		while (Parent[Current] != Current)
		{
			const int32 Next = Parent[Current];
			Parent[Current] = Root;
			Current = Next;
		}
		return Root;
	};

	auto Union = [&Parent, &Find](int32 A, int32 B)
	{
		const int32 RA = Find(A);
		const int32 RB = Find(B);
		if (RA != RB) { Parent[RA] = RB; }
	};

	// Signal A: corner-index grouping.
	int32 CornerLinkCount = 0;
	{
		TMap<int32, TArray<int32>> CornerToMasks;
		for (int32 MaskIdx = 0; MaskIdx < NumMasks; ++MaskIdx)
		{
			if (!MaskValid[MaskIdx]) { continue; }
			for (const int32 CornerIdx : CachedIntersectionMasks[MaskIdx].CornerIndices)
			{
				CornerToMasks.FindOrAdd(CornerIdx).Add(MaskIdx);
			}
		}
		for (const auto& Pair : CornerToMasks)
		{
			const TArray<int32>& MaskIndices = Pair.Value;
			for (int32 m = 1; m < MaskIndices.Num(); ++m)
			{
				Union(MaskIndices[0], MaskIndices[m]);
				++CornerLinkCount;
			}
		}
	}

	// Signal B: world-position proximity.
	// Masks whose XY world positions are within a spatial tolerance belong
	// to the same physical intersection. Uses grid-based spatial bucketing
	// to avoid O(N²) pairwise comparisons for large mask counts.
	constexpr float PositionEpsilon = 800.0f; // cm radius
	constexpr float PositionEpsilonSq = PositionEpsilon * PositionEpsilon;
	int32 PositionLinkCount = 0;

	// Grid bucketing: cell size = PositionEpsilon so each mask only needs
	// to check its own cell and 8 neighbours.
	TMap<int64, TArray<int32>> SpatialGrid;
	auto CellKey = [](float X, float Y, float CellSize) -> int64
	{
		const int32 CX = FMath::FloorToInt32(X / CellSize);
		const int32 CY = FMath::FloorToInt32(Y / CellSize);
		return (static_cast<int64>(CX) << 32) | static_cast<int64>(static_cast<uint32>(CY));
	};

	for (int32 i = 0; i < NumMasks; ++i)
	{
		if (!MaskValid[i]) { continue; }
		const int64 Key = CellKey(MaskWorldPositions[i].X, MaskWorldPositions[i].Y, PositionEpsilon);
		SpatialGrid.FindOrAdd(Key).Add(i);
	}

	for (int32 i = 0; i < NumMasks; ++i)
	{
		if (!MaskValid[i]) { continue; }
		const int32 CX = FMath::FloorToInt32(MaskWorldPositions[i].X / PositionEpsilon);
		const int32 CY = FMath::FloorToInt32(MaskWorldPositions[i].Y / PositionEpsilon);

		// Check own cell and 8 neighbours.
		for (int32 DX = -1; DX <= 1; ++DX)
		{
			for (int32 DY = -1; DY <= 1; ++DY)
			{
				const int64 NKey = (static_cast<int64>(CX + DX) << 32) | static_cast<int64>(static_cast<uint32>(CY + DY));
				const TArray<int32>* Cell = SpatialGrid.Find(NKey);
				if (!Cell) { continue; }

				for (const int32 j : *Cell)
				{
					if (j <= i) { continue; } // Only process pairs once (j > i).
					if (!MaskValid[j]) { continue; }
					const float DistX = MaskWorldPositions[i].X - MaskWorldPositions[j].X;
					const float DistY = MaskWorldPositions[i].Y - MaskWorldPositions[j].Y;
					const float XYDistSq = DistX * DistX + DistY * DistY;
					if (XYDistSq <= PositionEpsilonSq)
					{
						Union(i, j);
						++PositionLinkCount;

						UE_LOG(LogAAATraffic, Log,
							TEXT("    Masks [%d] and [%d] grouped by world-position proximity (XY dist=%.1f cm)"),
							i, j, FMath::Sqrt(XYDistSq));
					}
				}
			}
		}
	}

	UE_LOG(LogAAATraffic, Log,
		TEXT("  Grouping signals: %d corner-index links, %d world-position links"),
		CornerLinkCount, PositionLinkCount);

	// ── Step 5: Collect groups ─────────────────────────────────
	TMap<int32, TArray<int32>> Groups;
	for (int32 k = 0; k < NumMasks; ++k)
	{
		if (!MaskValid[k]) { continue; }
		Groups.FindOrAdd(Find(k)).Add(k);
	}

	// Sort groups deterministically.
	// Use the minimum (StartDistance, ParentRoad name) in each group as sort key
	// so junction IDs are stable across rebuilds.
	struct FGroupEntry
	{
		FString SortKey;
		TArray<int32> MaskIndices;
	};
	TArray<FGroupEntry> SortedGroups;

	for (auto& GroupPair : Groups)
	{
		FGroupEntry Entry;
		Entry.MaskIndices = MoveTemp(GroupPair.Value);

		// Build sort key from first mask (by road name + start distance).
		Entry.MaskIndices.Sort();
		const FIntersectionMaskInfo& FirstMask = CachedIntersectionMasks[Entry.MaskIndices[0]];
		const FString RoadName = FirstMask.ParentRoad.IsValid() ? FirstMask.ParentRoad->GetName() : TEXT("_");
		Entry.SortKey = FString::Printf(TEXT("%s_%.1f"), *RoadName, FirstMask.StartDistance);

		SortedGroups.Add(MoveTemp(Entry));
	}

	SortedGroups.Sort([](const FGroupEntry& A, const FGroupEntry& B)
	{
		return A.SortKey < B.SortKey;
	});

	// ── Step 6: Assign group IDs and compute centroids ─────────
	// Centroids are averaged from mask world positions computed in Step 3.
	for (int32 GIdx = 0; GIdx < SortedGroups.Num(); ++GIdx)
	{
		const int32 GroupId = GIdx + 1;

		FVector CentroidSum = FVector::ZeroVector;
		int32 CentroidCount = 0;

		for (const int32 MaskIdx : SortedGroups[GIdx].MaskIndices)
		{
			CachedIntersectionMasks[MaskIdx].GroupId = GroupId;

			const FIntersectionMaskInfo& Mask = CachedIntersectionMasks[MaskIdx];
			const FVector& WorldPos = MaskWorldPositions[MaskIdx];

			CentroidSum += WorldPos;
			++CentroidCount;

			UE_LOG(LogAAATraffic, Log,
				TEXT("    Group %d Mask[%d]: Road=%s Dist=[%.1f, %.1f] WorldPos=(%.1f, %.1f, %.1f)"),
				GroupId, MaskIdx,
				Mask.ParentRoad.IsValid() ? *Mask.ParentRoad->GetName() : TEXT("NULL"),
				Mask.StartDistance, Mask.EndDistance,
				WorldPos.X, WorldPos.Y, WorldPos.Z);
		}

		if (CentroidCount > 0)
		{
			const FVector Centroid = CentroidSum / static_cast<float>(CentroidCount);
			IntersectionGroupCentroids.Add(GroupId, Centroid);

			UE_LOG(LogAAATraffic, Log,
				TEXT("    Group %d centroid: (%.1f, %.1f, %.1f) from %d masks"),
				GroupId, Centroid.X, Centroid.Y, Centroid.Z, CentroidCount);
		}
	}

	UE_LOG(LogAAATraffic, Log,
		TEXT("RoadBLDReflectionProvider: Mask-based intersections — %d masks, %d corner links, %d position links, %d intersection groups."),
		NumMasks, CornerLinkCount, PositionLinkCount, SortedGroups.Num());
}

// ---------------------------------------------------------------------------
// BuildProximityConnections — endpoint matching + U-turn gating
// ---------------------------------------------------------------------------

void URoadBLDReflectionProvider::BuildProximityConnections()
{
	ProximityConnectionList.Empty();

	const float ProxThreshold = GProximityThreshold;
	const float ProxThresholdSq = ProxThreshold * ProxThreshold;
	const float DirectionDotMin = GDirectionDotMin;
	const float MinUTurnWidth = GMinUTurnWidth;

	// Build working set: originals that were NOT split + all virtuals.
	TArray<int32> WorkingSet;
	for (const auto& Pair : LaneEndpointMap)
	{
		if (!ReplacedLaneHandles.Contains(Pair.Key))
		{
			WorkingSet.Add(Pair.Key);
		}
	}
	WorkingSet.Sort(); // Deterministic order.

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
	const bool bSampleDiagnostics = ShouldLogDiagnostics(3);
	const int32 MaxSamples = GetDiagnosticsSampleLimit();

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

			// Only connect lanes on DIFFERENT roads.
			if (RoadA == RoadB && RoadA != 0)
			{
				++SkipSameRoad;
				continue;
			}

			// Check EndPos(A) → StartPos(B) proximity.
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

			// Direction compatibility: EndDir(A) · StartDir(B).
			const float Dot = FVector::DotProduct(CacheA->EndDir, CacheB->StartDir);

			if (Dot >= DirectionDotMin)
			{
				// Forward or angled connection — always allowed.
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
				}

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
					}

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

	// Sort all connection lists deterministically.
	for (auto& Pair : LaneConnectionMap)
	{
		Pair.Value.Sort([](const FTrafficLaneHandle& A, const FTrafficLaneHandle& B)
		{
			return A.HandleId < B.HandleId;
		});
	}

	UE_LOG(LogAAATraffic, Log,
		TEXT("RoadBLDReflectionProvider: Proximity connections added %d links (U-turn gating at %.0f cm)."),
		ProximityLinks, MinUTurnWidth);

	if (ShouldLogDiagnostics(2))
	{
		UE_LOG(LogAAATraffic, Log,
			TEXT("RoadBLDReflectionProvider: ProximityDiag PairComparisons=%d WorkingSet=%d SkipSameRoad=%d SkipDistance=%d ForwardAccepted=%d UTurnCandidates=%d UTurnAccepted=%d UTurnRejectedWidth=%d DuplicateAttempts=%d"),
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
			UE_LOG(LogAAATraffic, Log, TEXT("RoadBLDReflectionProvider: ProximitySample %s"), *Line);
		}
		for (const FString& Line : SampledRejects)
		{
			UE_LOG(LogAAATraffic, Log, TEXT("RoadBLDReflectionProvider: ProximitySample %s"), *Line);
		}
	}
}

void URoadBLDReflectionProvider::RunConnectivityDiagnostics(const TCHAR* PhaseTag) const
{
	if (!(GTrafficDiagnosticsValidateGraph || ShouldLogDiagnostics(2)))
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
		TEXT("RoadBLDReflectionProvider: GraphDiag Phase=%s LaneConnKeys=%d LaneEdges=%d MissingSource=%d MissingTarget=%d DuplicateEdges=%d SelfEdges=%d AsymmetricAdjacency=%d JunctionWithoutCentroid=%d"),
		PhaseTag,
		LaneConnectionMap.Num(),
		ProximityConnectionList.Num(),
		MissingSourceLane,
		MissingTargetLane,
		DuplicateEdges,
		SelfEdges,
		AsymmetricAdjacency,
		JunctionWithoutCentroid);

	if (GTrafficDiagnosticsValidateGraph)
	{
		ensureMsgf(MissingSourceLane == 0, TEXT("RoadBLDReflectionProvider: connectivity references missing source lanes."));
		ensureMsgf(MissingTargetLane == 0, TEXT("RoadBLDReflectionProvider: connectivity references missing target lanes."));
		ensureMsgf(SelfEdges == 0, TEXT("RoadBLDReflectionProvider: self-loop connectivity edges detected."));
		ensureMsgf(AsymmetricAdjacency == 0, TEXT("RoadBLDReflectionProvider: left/right adjacency symmetry violation detected."));
		ensureMsgf(JunctionWithoutCentroid == 0, TEXT("RoadBLDReflectionProvider: lane mapped to junction without centroid."));
	}
}

// ---------------------------------------------------------------------------
// BuildJunctionGrouping — register mask-based junctions, then assign
// proximity connections
// ---------------------------------------------------------------------------

void URoadBLDReflectionProvider::BuildJunctionGrouping()
{
	LaneToJunctionMap.Empty();
	JunctionCentroids.Empty();

	// ── Mask-based junctions (primary path) ─────────────────────
	// When IntersectionGroupCentroids is populated, mask groups define the
	// authoritative set of junctions. Junctions are registered FIRST from
	// mask data, then ProximityConnections are assigned to them.
	if (IntersectionGroupCentroids.Num() > 0)
	{
		// Step 1: Register all mask group centroids as junctions.
		for (const auto& Pair : IntersectionGroupCentroids)
		{
			JunctionCentroids.Add(Pair.Key, Pair.Value);
		}

		UE_LOG(LogAAATraffic, Log,
			TEXT("  Registered %d mask-based junctions with centroids."),
			JunctionCentroids.Num());

		// Step 2: Build GroupId → set of road handles from mask data.
		// Also build reverse map: road handle → list of GroupIds (for
		// through-roads that participate in multiple intersections).
		TMap<int32, TSet<int32>> GroupToRoadHandles;
		TMap<int32, TArray<int32>> RoadHandleToGroups;
		TMap<int32, TSet<UObject*>> GroupToRoads; // For proximity connection matching.

		// Pre-compute reverse map: RoadActor* → road handle.
		// Avoids O(N) scan of RoadHandleMap for every mask.
		TMap<UObject*, int32> RoadActorToHandleCache;
		for (const auto& RHPair : RoadHandleMap)
		{
			if (UObject* R = RHPair.Value.Get())
			{
				RoadActorToHandleCache.Add(R, RHPair.Key);
			}
		}

		for (const FIntersectionMaskInfo& Mask : CachedIntersectionMasks)
		{
			UObject* RoadActor = Mask.ParentRoad.Get();
			if (!RoadActor || Mask.GroupId <= 0) { continue; }

			GroupToRoads.FindOrAdd(Mask.GroupId).Add(RoadActor);

			// Resolve road handle for this road actor.
			const int32* CachedHandle = RoadActorToHandleCache.Find(RoadActor);
			if (CachedHandle)
			{
				GroupToRoadHandles.FindOrAdd(Mask.GroupId).Add(*CachedHandle);
				RoadHandleToGroups.FindOrAdd(*CachedHandle).AddUnique(Mask.GroupId);
			}
		}

		// Step 3: Map lanes to junctions using road participation.
		// For through-roads that were split by DetectAndSplitThroughRoads,
		// each virtual segment already has its group ID in VirtualLaneToGroupId.
		// For non-split lanes, map directly by road participation.
		int32 LanesMapped = 0;
		for (const auto& GroupPair : GroupToRoadHandles)
		{
			const int32 JunctionId = GroupPair.Key;

			for (const int32 RoadHandle : GroupPair.Value)
			{
				const TArray<int32>* Lanes = RoadToLaneHandles.Find(RoadHandle);
				if (!Lanes) { continue; }

				for (const int32 LaneHandle : *Lanes)
				{
					// Check if this lane was split into virtual segments.
					if (ReplacedLaneHandles.Contains(LaneHandle))
					{
						// Through-road lane was split. Map each virtual segment
						// using its pre-computed group ID from splitting.
						const TArray<int32>* Virtuals = OriginalToVirtualMap.Find(LaneHandle);
						if (Virtuals)
						{
							for (const int32 VH : *Virtuals)
							{
								const int32* VGroupId = VirtualLaneToGroupId.Find(VH);
								if (VGroupId && *VGroupId > 0)
								{
									LaneToJunctionMap.FindOrAdd(VH) = *VGroupId;
									++LanesMapped;
								}
								// VGroupId == 0 → free-flow segment, no junction.
							}
						}
						continue; // Don't map the original lane.
					}

					LaneToJunctionMap.Add(LaneHandle, JunctionId);
					++LanesMapped;
				}
			}
		}

		UE_LOG(LogAAATraffic, Log,
			TEXT("  Mapped %d lanes to mask-based junctions."), LanesMapped);

		// Step 4: Process ProximityConnections.
		// These represent end-to-end road joins found by endpoint matching.
		// Assign each to an existing mask junction or create a new one.

		// Pre-compute lane→road UObject map to avoid repeated lookups
		// inside the proximity connection loop.
		TMap<int32, UObject*> LaneToRoadActorCache;
		for (const FProximityConnection& PC : ProximityConnectionList)
		{
			for (const int32 LH : { PC.FromLane, PC.ToLane })
			{
				if (LaneToRoadActorCache.Contains(LH)) { continue; }
				int32 EffectiveLane = LH;
				if (const FVirtualLaneInfo* V = VirtualLaneMap.Find(LH))
				{
					EffectiveLane = V->OriginalLaneHandle;
				}
				const int32* RoadHandlePtr = LaneToRoadHandleMap.Find(EffectiveLane);
				UObject* RoadObj = RoadHandlePtr ? RoadHandleMap.FindRef(*RoadHandlePtr).Get() : nullptr;
				LaneToRoadActorCache.Add(LH, RoadObj);
			}
		}

		// Build reverse lookup: (RoadA, RoadB) pair → list of GroupIds.
		// Reduces proximity connection assignment from O(P×G) to O(P).
		// Key: sorted pair of road actor pointers encoded as int64.
		auto MakeRoadPairKey = [](UObject* A, UObject* B) -> int64
		{
			const uintptr_t PA = reinterpret_cast<uintptr_t>(A);
			const uintptr_t PB = reinterpret_cast<uintptr_t>(B);
			const uintptr_t Lo = FMath::Min(PA, PB);
			const uintptr_t Hi = FMath::Max(PA, PB);
			// Combine into a single 64-bit key via hash.
			return static_cast<int64>(HashCombine(GetTypeHash(Lo), GetTypeHash(Hi)));
		};

		TMap<int64, TArray<int32>> RoadPairToGroups;
		for (const auto& GPair : GroupToRoads)
		{
			TArray<UObject*> Roads = GPair.Value.Array();
			for (int32 RIdx = 0; RIdx < Roads.Num(); ++RIdx)
			{
				for (int32 RJdx = RIdx + 1; RJdx < Roads.Num(); ++RJdx)
				{
					const int64 PKey = MakeRoadPairKey(Roads[RIdx], Roads[RJdx]);
					RoadPairToGroups.FindOrAdd(PKey).AddUnique(GPair.Key);
				}
			}
		}

		int32 PCMatched = 0;
		int32 PCSkipped = 0;

		for (const FProximityConnection& PC : ProximityConnectionList)
		{
			UObject* RoadA = LaneToRoadActorCache.FindRef(PC.FromLane);
			UObject* RoadB = LaneToRoadActorCache.FindRef(PC.ToLane);

			// Try to find an intersection group that contains both roads.
			int32 GroupId = 0;
			const TArray<int32>* Candidates = nullptr;
			if (RoadA && RoadB)
			{
				const int64 PKey = MakeRoadPairKey(RoadA, RoadB);
				Candidates = RoadPairToGroups.Find(PKey);
			}

			if (Candidates && Candidates->Num() == 1)
			{
				GroupId = (*Candidates)[0];
			}
			else if (Candidates && Candidates->Num() > 1)
			{
				// Tiebreak: nearest centroid.
				float BestDistSq = TNumericLimits<float>::Max();
				for (const int32 CandId : *Candidates)
				{
					const FVector* C = JunctionCentroids.Find(CandId);
					if (C)
					{
						const float DSq = FVector::DistSquared(PC.Midpoint, *C);
						if (DSq < BestDistSq) { BestDistSq = DSq; GroupId = CandId; }
					}
				}
			}
			else
			{
				// Neither road shares a mask group with the other.
				// This is a simple end-to-end road join (e.g. user extended a
				// road by drawing a new piece), NOT an intersection.
				// Do NOT assign a junction — the lane connectivity (next-lane)
				// already handles the hand-off; adding a junction here would
				// cause vehicles to stop at a non-intersection join point.
				UE_LOG(LogAAATraffic, Log,
					TEXT("  Connection FromLane=%d -> ToLane=%d (Road=%s -> Road=%s): road continuation (no shared mask group) — skipping junction assignment."),
					PC.FromLane, PC.ToLane,
					RoadA ? *RoadA->GetName() : TEXT("NULL"),
					RoadB ? *RoadB->GetName() : TEXT("NULL"));
				++PCSkipped;
				continue;
			}

			if (GroupId > 0)
			{
				++PCMatched;
				UE_LOG(LogAAATraffic, Log,
					TEXT("  Connection FromLane=%d -> ToLane=%d -> matched junction %d"),
					PC.FromLane, PC.ToLane, GroupId);
			}

			// DO NOT tag FromLane/ToLane as junction here.
			// Mask-based splitting (Step 3) already set the correct IDs:
			// intersection (mask) segments have GroupId > 0, free-flow
			// (pre/post-mask) segments are deliberately JunctionId=0.
			// Tagging proximity-connection participants would pollute
			// pre-mask approach lanes and post-mask departure lanes with
			// junction status, causing vehicles to:
			//   - be filtered from spawn pool (too few spawn candidates)
			//   - re-detect the junction after exiting (stuck mid-intersection)
			//   - get assigned to wrong signal groups
			// The connectivity (which lane leads where) is already stored
			// in LaneConnectionMap by BuildProximityConnections.
		}

		UE_LOG(LogAAATraffic, Log,
			TEXT("RoadBLDReflectionProvider: Junction grouping — %d mask junctions, %d proximity connections (%d matched to junctions, %d skipped as road continuations), %d total junctions, %d lanes mapped."),
			IntersectionGroupCentroids.Num(), ProximityConnectionList.Num(),
			PCMatched, PCSkipped, JunctionCentroids.Num(), LaneToJunctionMap.Num());

		// Build reverse map: junction ID → list of lane handles.
		JunctionToLanesMap.Empty();
		for (const auto& Pair : LaneToJunctionMap)
		{
			JunctionToLanesMap.FindOrAdd(Pair.Value).AddUnique(Pair.Key);
		}
		// Sort each list for deterministic iteration.
		for (auto& Pair : JunctionToLanesMap)
		{
			Pair.Value.Sort();
		}
		UE_LOG(LogAAATraffic, Log,
			TEXT("RoadBLDReflectionProvider: Built JunctionToLanesMap — %d junctions."),
			JunctionToLanesMap.Num());

		return;
	}

	// ── No mask data + no connections ──────────────────────────
	if (ProximityConnectionList.Num() == 0)
	{
		UE_LOG(LogAAATraffic, Log,
			TEXT("RoadBLDReflectionProvider: No mask data and no proximity connections — junction grouping skipped."));
		return;
	}

	// ── No mask data: all proximity connections are road continuations ──
	// RoadBLD is the authoritative source for intersection data.
	// Zero masks means RoadBLD determined there are no intersections in
	// this road network (e.g. an overpass, or isolated road segments).
	// Proximity connections still provide lane-to-lane handoffs, but no
	// junctions should be created — creating them would spawn false
	// junction markers at simple road joins.
	UE_LOG(LogAAATraffic, Log,
		TEXT("RoadBLDReflectionProvider: No mask data — %d proximity connections treated as road continuations, 0 junctions created."),
		ProximityConnectionList.Num());
}

// ---------------------------------------------------------------------------
// BuildLaneAdjacency — detect left/right neighbor lanes per road
// ---------------------------------------------------------------------------

void URoadBLDReflectionProvider::BuildLaneAdjacency()
{
	LeftNeighborMap.Empty();
	RightNeighborMap.Empty();

	// For each road, examine pairs of lanes and detect adjacency via shared edge curves.
	// Lane A's RightEdge == Lane B's LeftEdge ⇒ B is to the right of A (and A is to the left of B).
	for (const auto& RoadPair : RoadToLaneHandles)
	{
		const TArray<int32>& LaneIds = RoadPair.Value;
		if (LaneIds.Num() < 2) { continue; }

		for (int32 i = 0; i < LaneIds.Num(); ++i)
		{
			const FReflectionLaneData* DataI = LaneHandleMap.Find(LaneIds[i]);
			if (!DataI) { continue; }

			for (int32 j = i + 1; j < LaneIds.Num(); ++j)
			{
				const FReflectionLaneData* DataJ = LaneHandleMap.Find(LaneIds[j]);
				if (!DataJ) { continue; }

				UObject* iRight = DataI->RightEdge.Get();
				UObject* jLeft  = DataJ->LeftEdge.Get();
				UObject* iLeft  = DataI->LeftEdge.Get();
				UObject* jRight = DataJ->RightEdge.Get();

				// I's right == J's left ⇒ J is to the right of I.
				if (iRight && jLeft && iRight == jLeft)
				{
					RightNeighborMap.Add(LaneIds[i], LaneIds[j]);
					LeftNeighborMap.Add(LaneIds[j], LaneIds[i]);
				}
				// I's left == J's right ⇒ J is to the left of I.
				else if (iLeft && jRight && iLeft == jRight)
				{
					LeftNeighborMap.Add(LaneIds[i], LaneIds[j]);
					RightNeighborMap.Add(LaneIds[j], LaneIds[i]);
				}
			}
		}
	}

	UE_LOG(LogAAATraffic, Log,
		TEXT("RoadBLDReflectionProvider: Built lane adjacency — %d left links, %d right links."),
		LeftNeighborMap.Num(), RightNeighborMap.Num());
}

// ---------------------------------------------------------------------------
// DetectReversedLanes — identify lanes on the left side of 2-way roads
// ---------------------------------------------------------------------------

void URoadBLDReflectionProvider::DetectReversedLanes()
{
	ReversedLaneSet.Empty();

	for (const auto& RoadPair : RoadToLaneHandles)
	{
		if (RoadPair.Value.Num() < 2) { continue; } // Single-lane — cannot be 2-way.

		UObject* RoadActor = RoadHandleMap.FindRef(RoadPair.Key).Get();
		if (!RoadActor) { continue; }

		const double RoadLength = GetRoadLength(RoadActor);
		if (RoadLength < 200.0) { continue; }

		UObject* RefLine = GetReferenceLine(RoadActor);
		if (!RefLine || !Get3DPosFunc) { continue; }

		// Sample reference line at midpoint for direction and position.
		const double MidDist = RoadLength * 0.5;
		const double NearDist = FMath::Max(MidDist - 100.0, 0.0);
		const FVector RefMid = Get3DPositionAtDistance(RefLine, RefLine, MidDist);
		const FVector RefNear = Get3DPositionAtDistance(RefLine, RefLine, NearDist);
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
		TEXT("RoadBLDReflectionProvider: Detected %d reversed lanes on 2-way roads."),
		ReversedLaneSet.Num());
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

int32 URoadBLDReflectionProvider::GetJunctionForLane(const FTrafficLaneHandle& Lane)
{
	// Check the handle directly first (works for both virtual and original).
	if (const int32* JId = LaneToJunctionMap.Find(Lane.HandleId))
	{
		UE_LOG(LogAAATraffic, Verbose,
			TEXT("JNCT GetJunctionForLane: Lane=%d → JunctionId=%d (direct map hit)"),
			Lane.HandleId, *JId);
		return *JId;
	}
	// Virtual lane — also check via original handle.
	if (const FVirtualLaneInfo* VInfo = VirtualLaneMap.Find(Lane.HandleId))
	{
		if (const int32* JId = LaneToJunctionMap.Find(VInfo->OriginalLaneHandle))
		{
			UE_LOG(LogAAATraffic, Verbose,
				TEXT("JNCT GetJunctionForLane: Lane=%d (virtual, orig=%d) → JunctionId=%d (fallback hit)"),
				Lane.HandleId, VInfo->OriginalLaneHandle, *JId);
			return *JId;
		}
	}
	// No junction for this lane — this fires very frequently for free-flow lanes,
	// so we use Verbose severity (not Log/Warning).
	UE_LOG(LogAAATraffic, Verbose,
		TEXT("JNCT GetJunctionForLane: Lane=%d → 0 (no junction found)"),
		Lane.HandleId);
	return 0;
}

bool URoadBLDReflectionProvider::GetJunctionCentroid(int32 JunctionId, FVector& OutCentroid)
{
	const FVector* C = JunctionCentroids.Find(JunctionId);
	if (C)
	{
		OutCentroid = *C;
		return true;
	}
	return false;
}

TArray<FTrafficLaneHandle> URoadBLDReflectionProvider::GetLanesForJunction(int32 JunctionId)
{
	TArray<FTrafficLaneHandle> Result;
	if (const TArray<int32>* LaneIds = JunctionToLanesMap.Find(JunctionId))
	{
		Result.Reserve(LaneIds->Num());
		for (const int32 Id : *LaneIds)
		{
			Result.Emplace(FTrafficLaneHandle(Id));
		}
	}
	return Result;
}

bool URoadBLDReflectionProvider::GetJunctionPath(
	const FTrafficLaneHandle& FromLane,
	const FTrafficLaneHandle& ToLane,
	TArray<FVector>& OutPath)
{
	// Generate a cubic Hermite curve from the end of FromLane to the start of
	// ToLane using cached endpoint tangents.  This replaces the old BUG-4 fix
	// that returned false (forcing the controller to synthesize its own curve).
	// Generating it here ensures DoJunctionPathsConflict can also use curved
	// paths for accurate conflict detection.
	const FLaneEndpointCache* FromCache = LaneEndpointMap.Find(FromLane.HandleId);
	const FLaneEndpointCache* ToCache = LaneEndpointMap.Find(ToLane.HandleId);
	if (!FromCache || !ToCache)
	{
		return false;
	}

	const FVector P0 = FromCache->EndPos;
	const FVector P1 = ToCache->StartPos;
	const float SpanDist = FVector::Dist(P0, P1);

	if (SpanDist < 50.0f)
	{
		// Negligible gap — no junction curve needed.
		return false;
	}

	// Angle-aware tangent scaling: sharp turns need shorter tangents to
	// avoid overshooting into the oncoming lane.  Straight-through keeps
	// the original 0.5 × SpanDist; a 90° turn uses ~0.35; a U-turn ~0.25.
	const float DotFactor = FVector::DotProduct(FromCache->EndDir, ToCache->StartDir);
	// For actual turns (dot < 0.3) raise the floor from 0.25 to 0.35 to
	// pull the curve outward through the mid-arc — wider clearance at apex.
	const float AlphaFloor = (DotFactor < 0.3f) ? 0.35f : 0.25f;
	const float Alpha = FMath::Lerp(AlphaFloor, 0.5f,
		FMath::Clamp((DotFactor + 1.0f) * 0.5f, 0.0f, 1.0f));
	const float TangentScale = SpanDist * Alpha;

	// ── Inner-corner offset for turns ──────────────────────────────
	// Shift the control points outward (away from the inner curb) so the
	// Hermite curve clears the corner.  Cross-product Z of approach×exit
	// gives turn direction: positive = left turn, negative = right turn.
	const float CrossZ = FVector::CrossProduct(FromCache->EndDir, ToCache->StartDir).Z;
	FVector AdjP0 = P0;
	FVector AdjP1 = P1;
	constexpr float InnerCornerOffset = 150.0f; // cm — roughly half a lane width
	if (FMath::Abs(CrossZ) > 0.26f) // Only for actual turns (>15°)
	{
		// Perpendicular in XY: rotate the tangent 90° about Z.
		// For a right turn (CrossZ < 0): shift LEFT (positive perp).
		// For a left turn (CrossZ > 0): shift RIGHT (negative perp).
		const float Sign = (CrossZ < 0.0f) ? 1.0f : -1.0f;
		const FVector PerpFrom = FVector(-FromCache->EndDir.Y, FromCache->EndDir.X, 0.0f) * Sign;
		const FVector PerpTo = FVector(-ToCache->StartDir.Y, ToCache->StartDir.X, 0.0f) * Sign;
		// Scale offset by turn sharpness: 90° turns get full offset, gentle turns less.
		const float SharpnessFactor = FMath::Clamp(FMath::Abs(CrossZ), 0.0f, 1.0f);
		AdjP0 += PerpFrom * (InnerCornerOffset * SharpnessFactor);
		AdjP1 += PerpTo * (InnerCornerOffset * SharpnessFactor);
	}

	const FVector M0 = FromCache->EndDir * TangentScale;
	const FVector M1 = ToCache->StartDir * TangentScale;

	// Segment count scales with span distance for consistent resolution.
	// 200 cm per segment, clamped to [6, 32].
	const int32 NumSegments = FMath::Clamp(FMath::CeilToInt32(SpanDist / 200.0f), 6, 32);
	OutPath.Reserve(NumSegments + 1);
	for (int32 i = 0; i <= NumSegments; ++i)
	{
		const float T = static_cast<float>(i) / static_cast<float>(NumSegments);
		const float T2 = T * T;
		const float T3 = T2 * T;
		const float H00 = 2.0f * T3 - 3.0f * T2 + 1.0f;
		const float H10 = T3 - 2.0f * T2 + T;
		const float H01 = -2.0f * T3 + 3.0f * T2;
		const float H11 = T3 - T2;
		OutPath.Add(H00 * AdjP0 + H10 * M0 + H01 * AdjP1 + H11 * M1);
	}
	return true;
}

// ---------------------------------------------------------------------------
// ITrafficRoadProvider — junction path conflict detection
// ---------------------------------------------------------------------------

// 2D line-segment intersection test (ignores Z). Returns true if AB crosses CD.
static bool Segments2DIntersect(const FVector& A, const FVector& B,
	const FVector& C, const FVector& D)
{
	// Parametric intersection of AB and CD in XY plane.
	const float Ax = B.X - A.X, Ay = B.Y - A.Y;
	const float Bx = D.X - C.X, By = D.Y - C.Y;
	const float Denom = Ax * By - Ay * Bx;

	if (FMath::Abs(Denom) < KINDA_SMALL_NUMBER)
	{
		return false; // Parallel or degenerate.
	}

	const float Cx = C.X - A.X, Cy = C.Y - A.Y;
	const float T = (Cx * By - Cy * Bx) / Denom;
	const float U = (Cx * Ay - Cy * Ax) / Denom;

	// Strict interior intersection (excluding shared endpoints).
	constexpr float Eps = 0.01f;
	return (T > Eps && T < (1.0f - Eps)) && (U > Eps && U < (1.0f - Eps));
}

bool URoadBLDReflectionProvider::DoJunctionPathsConflict(
	const FTrafficLaneHandle& FromA, const FTrafficLaneHandle& ToA,
	const FTrafficLaneHandle& FromB, const FTrafficLaneHandle& ToB)
{
	// Same approach or same exit → conflict (vehicles would merge).
	if (FromA == FromB || ToA == ToB) { return true; }

	// Get actual curved junction paths (Hermite curves from GetJunctionPath).
	TArray<FVector> PathA, PathB;
	const bool bHasPathA = GetJunctionPath(FromA, ToA, PathA);
	const bool bHasPathB = GetJunctionPath(FromB, ToB, PathB);

	// Fallback to straight-line if curves unavailable.
	if (!bHasPathA || PathA.Num() < 2)
	{
		PathA.Reset();
		const FLaneEndpointCache* FromACache = LaneEndpointMap.Find(FromA.HandleId);
		const FLaneEndpointCache* ToACache = LaneEndpointMap.Find(ToA.HandleId);
		if (!FromACache || !ToACache) { return true; }
		PathA.Add(FromACache->EndPos);
		PathA.Add(ToACache->StartPos);
	}
	if (!bHasPathB || PathB.Num() < 2)
	{
		PathB.Reset();
		const FLaneEndpointCache* FromBCache = LaneEndpointMap.Find(FromB.HandleId);
		const FLaneEndpointCache* ToBCache = LaneEndpointMap.Find(ToB.HandleId);
		if (!FromBCache || !ToBCache) { return true; }
		PathB.Add(FromBCache->EndPos);
		PathB.Add(ToBCache->StartPos);
	}

	// Test all pairs of segments between PathA and PathB for 2D intersection.
	for (int32 i = 0; i < PathA.Num() - 1; ++i)
	{
		for (int32 j = 0; j < PathB.Num() - 1; ++j)
		{
			if (Segments2DIntersect(PathA[i], PathA[i + 1], PathB[j], PathB[j + 1]))
			{
				return true;
			}
		}
	}
	return false;
}

// ---------------------------------------------------------------------------
// ITrafficRoadProvider — adjacency & speed limit
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

// ---------------------------------------------------------------------------
// Reflection helpers
// ---------------------------------------------------------------------------

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
// PrecomputeJunctionMap — ONE-TIME pass over the entire lane graph at world
// startup. For every lane in the network, computes:
//   1. Total polyline length (stored in PrecomputedLaneLengths)
//   2. Distance to the nearest downstream junction via graph walk
//      (stored in PrecomputedJunctionMap)
//
// After this runs, GetLaneLength and GetDistanceToNextJunction are O(1)
// lookups. Every vehicle has instant access to the ENTIRE road/junction
// layout — no per-tick scanning needed.
// ---------------------------------------------------------------------------
void URoadBLDReflectionProvider::PrecomputeJunctionMap()
{
	PrecomputedLaneLengths.Empty();
	PrecomputedJunctionMap.Empty();

	// ── Phase 1: Compute and cache every lane's polyline length ──
	for (const auto& Pair : LaneEndpointMap)
	{
		const int32 LaneId = Pair.Key;
		const FLaneEndpointCache& Cache = Pair.Value;

		float Length = 0.0f;
		for (int32 i = 0; i < Cache.Polyline.Num() - 1; ++i)
		{
			Length += FVector::Dist(Cache.Polyline[i], Cache.Polyline[i + 1]);
		}
		PrecomputedLaneLengths.Add(LaneId, Length);
	}

	// ── Phase 2: For every lane, walk forward to find the next junction ──
	// We iterate ALL lanes (including virtual segments) and for each one,
	// walk the LaneConnectionMap graph until we hit a junction lane or
	// exhaust the search (no junction downstream).
	//
	// The result is keyed by each lane's handle ID and stores the cumulative
	// distance from that lane's END to the junction's START.

	constexpr int32 MaxWalkHops = 50; // generous — real networks are unlikely to exceed this

	// Collect all lane IDs to iterate deterministically.
	TArray<int32> AllLaneIds;
	LaneEndpointMap.GetKeys(AllLaneIds);
	AllLaneIds.Sort(); // deterministic order

	int32 JunctionLaneCount = 0;
	int32 FreeFlowWithJunctionAhead = 0;
	int32 DeadEndLanes = 0;

	for (const int32 StartLaneId : AllLaneIds)
	{
		// If this lane IS a junction lane, distance = 0.
		const int32* StartJunction = LaneToJunctionMap.Find(StartLaneId);
		if (StartJunction && *StartJunction != 0)
		{
			FJunctionScanResult Result;
			Result.JunctionId = *StartJunction;
			Result.DistanceCm = 0.0f;
			Result.JunctionLane = FTrafficLaneHandle(StartLaneId);
			Result.ApproachLane = FTrafficLaneHandle(StartLaneId);
			PrecomputedJunctionMap.Add(StartLaneId, Result);
			++JunctionLaneCount;
			continue;
		}

		// BFS walk forward from this lane through ALL successor branches
		// to find the nearest downstream junction. Previous code followed
		// only the first unvisited path at each fork, missing junctions
		// reachable via other branches.
		struct FBFSEntry
		{
			int32 LaneId;
			int32 PreviousLaneId;
			float AccumulatedDist;
		};

		TArray<FBFSEntry> Queue;
		TSet<int32> Visited;
		Visited.Add(StartLaneId);
		bool bFound = false;

		// Seed the queue with all direct successors of the start lane.
		{
			const TArray<FTrafficLaneHandle>* NextLanes = LaneConnectionMap.Find(StartLaneId);
			if (NextLanes)
			{
				for (const FTrafficLaneHandle& Next : *NextLanes)
				{
					if (!Visited.Contains(Next.HandleId))
					{
						Visited.Add(Next.HandleId);
						FBFSEntry Entry;
						Entry.LaneId = Next.HandleId;
						Entry.PreviousLaneId = StartLaneId;
						Entry.AccumulatedDist = 0.0f;
						Queue.Add(Entry);
					}
				}
			}
		}

		int32 QueueIdx = 0;
		while (QueueIdx < Queue.Num() && QueueIdx < MaxWalkHops)
		{
			const FBFSEntry Current = Queue[QueueIdx++];

			// Check if this lane is a junction lane.
			const int32* CurJunction = LaneToJunctionMap.Find(Current.LaneId);
			if (CurJunction && *CurJunction != 0)
			{
				FJunctionScanResult Result;
				Result.JunctionId = *CurJunction;
				Result.DistanceCm = Current.AccumulatedDist;
				Result.JunctionLane = FTrafficLaneHandle(Current.LaneId);
				Result.ApproachLane = FTrafficLaneHandle(Current.PreviousLaneId);
				PrecomputedJunctionMap.Add(StartLaneId, Result);
				bFound = true;
				++FreeFlowWithJunctionAhead;
				break; // found nearest junction — done for this start lane
			}

			// Accumulate this lane's length and enqueue its successors.
			float DistAfterThis = Current.AccumulatedDist;
			const float* CurLen = PrecomputedLaneLengths.Find(Current.LaneId);
			if (CurLen)
			{
				DistAfterThis += *CurLen;
			}

			const TArray<FTrafficLaneHandle>* NextLanes = LaneConnectionMap.Find(Current.LaneId);
			if (NextLanes)
			{
				for (const FTrafficLaneHandle& Next : *NextLanes)
				{
					if (!Visited.Contains(Next.HandleId))
					{
						Visited.Add(Next.HandleId);
						FBFSEntry Entry;
						Entry.LaneId = Next.HandleId;
						Entry.PreviousLaneId = Current.LaneId;
						Entry.AccumulatedDist = DistAfterThis;
						Queue.Add(Entry);
					}
				}
			}
		}

		if (!bFound)
		{
			++DeadEndLanes;
			// No junction downstream — store an invalid result so lookup
			// still returns instantly (no junction = JunctionId 0).
			FJunctionScanResult NoJunction;
			PrecomputedJunctionMap.Add(StartLaneId, NoJunction);
		}
	}

	UE_LOG(LogAAATraffic, Log,
		TEXT("PrecomputeJunctionMap: %d total lanes mapped. "
			 "%d junction lanes, %d free-flow with junction ahead, %d dead-end/no-junction."),
		AllLaneIds.Num(), JunctionLaneCount, FreeFlowWithJunctionAhead, DeadEndLanes);

	// ── DETAILED PER-LANE DUMP ──
	// Log every lane's junction map result so we can see exactly which lanes
	// "see" junctions and which are blind. This is the DEFINITIVE record
	// of the precomputed map. If a lane shows "NO-JUNCTION" here, vehicles
	// on that lane will NEVER detect an intersection via the approach scan.
	UE_LOG(LogAAATraffic, Log, TEXT("PrecomputeJunctionMap — PER-LANE DUMP START (%d lanes):"), AllLaneIds.Num());
	for (const int32 LaneId : AllLaneIds)
	{
		const FJunctionScanResult* R = PrecomputedJunctionMap.Find(LaneId);
		const float* LLen = PrecomputedLaneLengths.Find(LaneId);
		const float Len = LLen ? *LLen : 0.0f;
		const int32* JID = LaneToJunctionMap.Find(LaneId);
		const bool bIsJunctionLane = JID && *JID != 0;
		const TArray<FTrafficLaneHandle>* Connections = LaneConnectionMap.Find(LaneId);
		const int32 NumConnections = Connections ? Connections->Num() : 0;

		if (R && R->IsValid())
		{
			UE_LOG(LogAAATraffic, Verbose,
				TEXT("  Lane=%d Length=%.0f IsJunction=%s Connections=%d => JunctionId=%d Dist=%.0f JnctLane=%d ApproachLane=%d"),
				LaneId, Len,
				bIsJunctionLane ? TEXT("YES") : TEXT("no"),
				NumConnections,
				R->JunctionId, R->DistanceCm, R->JunctionLane.HandleId, R->ApproachLane.HandleId);
		}
		else
		{
			UE_LOG(LogAAATraffic, Verbose,
				TEXT("  Lane=%d Length=%.0f IsJunction=%s Connections=%d => NO-JUNCTION-DOWNSTREAM (vehicles here are blind)"),
				LaneId, Len,
				bIsJunctionLane ? TEXT("YES") : TEXT("no"),
				NumConnections);
		}
	}
	UE_LOG(LogAAATraffic, Verbose, TEXT("PrecomputeJunctionMap — PER-LANE DUMP END"));
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
