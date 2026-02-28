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

	// Yield to an already-registered compiled provider (URoadBLDTrafficProvider).
	UTrafficSubsystem* TrafficSub = InWorld.GetSubsystem<UTrafficSubsystem>();
	if (TrafficSub && TrafficSub->GetProvider() != nullptr)
	{
		UE_LOG(LogAAATraffic, Log,
			TEXT("RoadBLDReflectionProvider: A compiled provider is already registered — yielding."));
		return;
	}

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
	// Virtual or cached endpoint — use cached endpoint data.
	if (const FLaneEndpointCache* Cached = LaneEndpointMap.Find(Lane.HandleId))
	{
		return (Cached->EndPos - Cached->StartPos).GetSafeNormal();
	}
	TArray<FVector> Points;
	float Width;
	if (GetLanePath(Lane, Points, Width) && Points.Num() >= 2)
	{
		return (Points.Last() - Points[0]).GetSafeNormal();
	}
	return FVector::ForwardVector;
}

TArray<FTrafficLaneHandle> URoadBLDReflectionProvider::GetConnectedLanes(const FTrafficLaneHandle& Lane)
{
	if (const TArray<FTrafficLaneHandle>* Connected = LaneConnectionMap.Find(Lane.HandleId))
	{
		return *Connected;
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

			LaneToHandleMap.Add(LaneObj, LaneId);
		}
	}

	bCached = true;

	UE_LOG(LogAAATraffic, Log,
		TEXT("RoadBLDReflectionProvider: Cached %d roads, %d lanes via reflection."),
		RoadHandleMap.Num(), LaneHandleMap.Num());
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

	// ── Post-rebuild dump: check all arrays again ───────────────
	if (GEnableDiagnosticDumps)
	{
		UE_LOG(LogAAATraffic, Log, TEXT("===== POST-REBUILD ROADBLD INTERSECTION DATA DUMP ====="));

		FArrayProperty* CornersArr = CastField<FArrayProperty>(DynNetworkClass->FindPropertyByName(TEXT("RoadNetworkCorners")));
		int32 NC = 0;
		if (CornersArr) { FScriptArrayHelper H(CornersArr, CornersArr->ContainerPtrToValuePtr<void>(NetworkActor)); NC = H.Num(); }
		UE_LOG(LogAAATraffic, Log, TEXT("  RoadNetworkCorners: %d entries"), NC);

		FArrayProperty* EditArr = CastField<FArrayProperty>(DynNetworkClass->FindPropertyByName(TEXT("CornerEditData")));
		int32 NE = 0;
		if (EditArr) { FScriptArrayHelper H(EditArr, EditArr->ContainerPtrToValuePtr<void>(NetworkActor)); NE = H.Num(); }
		UE_LOG(LogAAATraffic, Log, TEXT("  CornerEditData: %d entries"), NE);

		FArrayProperty* MasksArr = CastField<FArrayProperty>(DynNetworkClass->FindPropertyByName(TEXT("IntersectionMasks")));
		int32 NM = 0;
		if (MasksArr) { FScriptArrayHelper H(MasksArr, MasksArr->ContainerPtrToValuePtr<void>(NetworkActor)); NM = H.Num(); }
		UE_LOG(LogAAATraffic, Log, TEXT("  IntersectionMasks: %d entries"), NM);

		FArrayProperty* CutsArr = CastField<FArrayProperty>(DynNetworkClass->FindPropertyByName(TEXT("RoadNetworkPerimeterCuts")));
		int32 NP = 0;
		if (CutsArr) { FScriptArrayHelper H(CutsArr, CutsArr->ContainerPtrToValuePtr<void>(NetworkActor)); NP = H.Num(); }
		UE_LOG(LogAAATraffic, Log, TEXT("  RoadNetworkPerimeterCuts: %d entries"), NP);

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

		UE_LOG(LogAAATraffic, Log, TEXT("===== END POST-REBUILD DUMP ====="));
	} // GEnableDiagnosticDumps
}

// ---------------------------------------------------------------------------
// BuildLaneConnectivity — corner-based fallback from RoadNetworkCorners
// ---------------------------------------------------------------------------

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

	// Access RoadNetworkCorners TArray via reflection.
	FProperty* CornersProp = DynNetworkClass->FindPropertyByName(TEXT("RoadNetworkCorners"));
	FArrayProperty* CornerArrayProp = CornersProp ? CastField<FArrayProperty>(CornersProp) : nullptr;
	if (!CornerArrayProp)
	{
		UE_LOG(LogAAATraffic, Log,
			TEXT("RoadBLDReflectionProvider: RoadNetworkCorners property not found or not array — skipping."));
		return;
	}

	FScriptArrayHelper ArrayHelper(CornerArrayProp, CornerArrayProp->ContainerPtrToValuePtr<void>(NetworkActor));
	const int32 NumCorners = ArrayHelper.Num();

	if (NumCorners == 0)
	{
		UE_LOG(LogAAATraffic, Log,
			TEXT("RoadBLDReflectionProvider: RoadNetworkCorners is empty — no corner-based connectivity."));
		return;
	}

	// Resolve FRoadNetworkCorner struct properties.
	FStructProperty* StructProp = CastField<FStructProperty>(CornerArrayProp->Inner);
	if (!StructProp) { return; }

	UScriptStruct* CornerStruct = StructProp->Struct;
	FProperty* StartEdgeProp = CornerStruct->FindPropertyByName(TEXT("StartEdge"));
	FProperty* EndEdgeProp   = CornerStruct->FindPropertyByName(TEXT("EndEdge"));
	FProperty* bStaleProp    = CornerStruct->FindPropertyByName(TEXT("bStale"));
	IntersectionPointProp    = CornerStruct->FindPropertyByName(TEXT("IntersectionPoint"));

	FObjectPropertyBase* StartEdgeObjProp = CastField<FObjectPropertyBase>(StartEdgeProp);
	FObjectPropertyBase* EndEdgeObjProp   = CastField<FObjectPropertyBase>(EndEdgeProp);
	FBoolProperty*       bStaleNative     = CastField<FBoolProperty>(bStaleProp);

	if (!StartEdgeObjProp || !EndEdgeObjProp) { return; }

	// Resolve edge-curve lane properties.
	FProperty* EdgeLeftLaneProp  = nullptr;
	FProperty* EdgeRightLaneProp = nullptr;
	for (int32 i = 0; i < NumCorners; ++i)
	{
		const uint8* ElemPtr = ArrayHelper.GetRawPtr(i);
		UObject* Edge = StartEdgeObjProp->GetObjectPropertyValue_InContainer(ElemPtr);
		if (Edge)
		{
			EdgeLeftLaneProp  = Edge->GetClass()->FindPropertyByName(TEXT("LeftLane"));
			EdgeRightLaneProp = Edge->GetClass()->FindPropertyByName(TEXT("RightLane"));
			break;
		}
	}
	if (!EdgeLeftLaneProp || !EdgeRightLaneProp) { return; }

	FObjectPropertyBase* EdgeLeftLaneObj  = CastField<FObjectPropertyBase>(EdgeLeftLaneProp);
	FObjectPropertyBase* EdgeRightLaneObj = CastField<FObjectPropertyBase>(EdgeRightLaneProp);
	if (!EdgeLeftLaneObj || !EdgeRightLaneObj) { return; }

	auto CollectLaneHandles = [&](UObject* Edge, TArray<int32>& OutHandles)
	{
		if (!Edge) { return; }
		UObject* LeftLane  = EdgeLeftLaneObj->GetObjectPropertyValue_InContainer(Edge);
		UObject* RightLane = EdgeRightLaneObj->GetObjectPropertyValue_InContainer(Edge);
		if (LeftLane)  { if (const int32* H = LaneToHandleMap.Find(LeftLane))  { OutHandles.AddUnique(*H); } }
		if (RightLane) { if (const int32* H = LaneToHandleMap.Find(RightLane)) { OutHandles.AddUnique(*H); } }
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
		TEXT("RoadBLDReflectionProvider: Corner-based connectivity added %d links from %d corners."),
		CornerConnections, NumCorners);
}

// ---------------------------------------------------------------------------
// DumpCornerDiagnostics — print all RoadNetworkCorner data to the log
// ---------------------------------------------------------------------------

void URoadBLDReflectionProvider::DumpCornerDiagnostics(UWorld* World)
{
	// Gated behind CVar — this function performs extensive reflection queries
	// and string formatting for every corner, which is expensive at scale.
	if (!GEnableDiagnosticDumps)
	{
		return;
	}

	if (!World || !DynNetworkClass)
	{
		UE_LOG(LogAAATraffic, Log, TEXT("CornerDump: No world or network class — skipping."));
		return;
	}

	// Find the DynamicRoadNetwork actor deterministically.
	AActor* NetworkActor = nullptr;
	{
		TArray<AActor*> CandidateNetworks;
		for (TActorIterator<AActor> It(World); It; ++It)
		{
			if (It->GetClass()->IsChildOf(DynNetworkClass))
			{
				CandidateNetworks.Add(*It);
			}
		}
		if (CandidateNetworks.Num() == 0)
		{
			UE_LOG(LogAAATraffic, Log, TEXT("CornerDump: No DynamicRoadNetwork actor found."));
			return;
		}

		// Deterministically pick the first actor sorted by name.
		CandidateNetworks.Sort([](const AActor& A, const AActor& B)
		{
			return A.GetName() < B.GetName();
		});

		NetworkActor = CandidateNetworks[0];
	}

	// Access RoadNetworkCorners array.
	FProperty* CornersProp = DynNetworkClass->FindPropertyByName(TEXT("RoadNetworkCorners"));
	FArrayProperty* CornerArrayProp = CornersProp ? CastField<FArrayProperty>(CornersProp) : nullptr;
	if (!CornerArrayProp)
	{
		UE_LOG(LogAAATraffic, Log, TEXT("CornerDump: RoadNetworkCorners property not found."));
		return;
	}

	FScriptArrayHelper ArrayHelper(CornerArrayProp, CornerArrayProp->ContainerPtrToValuePtr<void>(NetworkActor));
	const int32 NumCorners = ArrayHelper.Num();

	UE_LOG(LogAAATraffic, Log, TEXT("===== CORNER DIAGNOSTIC DUMP: %d corners total ====="), NumCorners);

	if (NumCorners == 0) { return; }

	// Resolve struct.
	FStructProperty* StructProp = CastField<FStructProperty>(CornerArrayProp->Inner);
	if (!StructProp) { return; }
	UScriptStruct* CS = StructProp->Struct;

	// Resolve all corner properties.
	FProperty* StartEdgeProp    = CS->FindPropertyByName(TEXT("StartEdge"));
	FProperty* EndEdgeProp      = CS->FindPropertyByName(TEXT("EndEdge"));
	FProperty* bStaleProp       = CS->FindPropertyByName(TEXT("bStale"));
	FProperty* IntPointProp     = CS->FindPropertyByName(TEXT("IntersectionPoint"));
	FProperty* StartDistProp    = CS->FindPropertyByName(TEXT("StartDistance"));
	FProperty* EndDistProp      = CS->FindPropertyByName(TEXT("EndDistance"));
	FProperty* StartOffsetProp  = CS->FindPropertyByName(TEXT("StartOffset"));
	FProperty* EndOffsetProp    = CS->FindPropertyByName(TEXT("EndOffset"));
	FProperty* DirAProp         = CS->FindPropertyByName(TEXT("DirectionA"));
	FProperty* DirBProp         = CS->FindPropertyByName(TEXT("DirectionB"));
	FProperty* RadiusProp       = CS->FindPropertyByName(TEXT("CornerRadius"));
	FProperty* CornerIDProp     = CS->FindPropertyByName(TEXT("CornerID"));

	FObjectPropertyBase* StartEdgeObj = CastField<FObjectPropertyBase>(StartEdgeProp);
	FObjectPropertyBase* EndEdgeObj   = CastField<FObjectPropertyBase>(EndEdgeProp);
	FBoolProperty*       bStaleNat    = CastField<FBoolProperty>(bStaleProp);

	// Resolve edge→lane properties from first valid edge.
	FObjectPropertyBase* EdgeLeftLaneObj  = nullptr;
	FObjectPropertyBase* EdgeRightLaneObj = nullptr;
	for (int32 i = 0; i < NumCorners && !EdgeLeftLaneObj; ++i)
	{
		const uint8* Ptr = ArrayHelper.GetRawPtr(i);
		UObject* Edge = StartEdgeObj ? StartEdgeObj->GetObjectPropertyValue_InContainer(Ptr) : nullptr;
		if (Edge)
		{
			EdgeLeftLaneObj  = CastField<FObjectPropertyBase>(Edge->GetClass()->FindPropertyByName(TEXT("LeftLane")));
			EdgeRightLaneObj = CastField<FObjectPropertyBase>(Edge->GetClass()->FindPropertyByName(TEXT("RightLane")));
		}
	}

	// Helper: read double/float property value.
	auto ReadNum = [](FProperty* Prop, const void* Container) -> double
	{
		if (!Prop) { return 0.0; }
		if (FDoubleProperty* D = CastField<FDoubleProperty>(Prop)) { return D->GetPropertyValue_InContainer(Container); }
		if (FFloatProperty* F = CastField<FFloatProperty>(Prop)) { return static_cast<double>(F->GetPropertyValue_InContainer(Container)); }
		return 0.0;
	};

	// Helper: read int32 property value.
	auto ReadInt = [](FProperty* Prop, const void* Container) -> int32
	{
		if (!Prop) { return 0; }
		if (FIntProperty* I = CastField<FIntProperty>(Prop)) { return I->GetPropertyValue_InContainer(Container); }
		return 0;
	};

	// Helper: describe an edge and its lanes.
	auto DescribeEdge = [&](UObject* Edge, const TCHAR* Label) -> FString
	{
		if (!Edge) { return FString::Printf(TEXT("%s=NULL"), Label); }

		FString Result = FString::Printf(TEXT("%s=%s"), Label, *Edge->GetName());

		// Resolve lanes on this edge.
		UObject* LeftLane  = EdgeLeftLaneObj  ? EdgeLeftLaneObj->GetObjectPropertyValue_InContainer(Edge)  : nullptr;
		UObject* RightLane = EdgeRightLaneObj ? EdgeRightLaneObj->GetObjectPropertyValue_InContainer(Edge) : nullptr;

		auto DescribeLane = [&](UObject* LaneObj, const TCHAR* Side) -> FString
		{
			if (!LaneObj) { return FString::Printf(TEXT("%s=NULL"), Side); }
			const int32* HandlePtr = LaneToHandleMap.Find(LaneObj);
			const int32 Handle = HandlePtr ? *HandlePtr : -1;
			// Find owning road.
			FString RoadName = TEXT("?");
			if (Handle >= 0)
			{
				if (const int32* RoadH = LaneToRoadHandleMap.Find(Handle))
				{
					if (const TWeakObjectPtr<UObject>* RoadPtr = RoadHandleMap.Find(*RoadH))
					{
						if (UObject* RoadObj = RoadPtr->Get()) { RoadName = RoadObj->GetName(); }
					}
				}
			}
			return FString::Printf(TEXT("%s=%s H#%d Road=%s"), Side, *LaneObj->GetName(), Handle, *RoadName);
		};

		Result += TEXT(" [") + DescribeLane(LeftLane, TEXT("L")) + TEXT(", ") + DescribeLane(RightLane, TEXT("R")) + TEXT("]");
		return Result;
	};

	// Dump each corner.
	for (int32 i = 0; i < NumCorners; ++i)
	{
		const uint8* Ptr = ArrayHelper.GetRawPtr(i);

		const bool bStale = bStaleNat ? bStaleNat->GetPropertyValue_InContainer(Ptr) : false;

		// IntersectionPoint
		FVector IntPt = FVector::ZeroVector;
		if (IntPointProp)
		{
			const FVector* ValPtr = IntPointProp->ContainerPtrToValuePtr<FVector>(Ptr);
			if (ValPtr) { IntPt = *ValPtr; }
		}

		const double StartDist   = ReadNum(StartDistProp, Ptr);
		const double EndDist     = ReadNum(EndDistProp, Ptr);
		const double StartOffset = ReadNum(StartOffsetProp, Ptr);
		const double EndOffset   = ReadNum(EndOffsetProp, Ptr);
		const double Radius      = ReadNum(RadiusProp, Ptr);
		const int32 DirA         = ReadInt(DirAProp, Ptr);
		const int32 DirB         = ReadInt(DirBProp, Ptr);

		UObject* StartEdge = StartEdgeObj ? StartEdgeObj->GetObjectPropertyValue_InContainer(Ptr) : nullptr;
		UObject* EndEdge   = EndEdgeObj   ? EndEdgeObj->GetObjectPropertyValue_InContainer(Ptr)   : nullptr;

		FString StartDesc = DescribeEdge(StartEdge, TEXT("StartEdge"));
		FString EndDesc   = DescribeEdge(EndEdge,   TEXT("EndEdge"));

		UE_LOG(LogAAATraffic, Log,
			TEXT("Corner[%d]%s: IntersectionPoint=(%.1f, %.1f, %.1f)"),
			i, bStale ? TEXT(" [STALE]") : TEXT(""),
			IntPt.X, IntPt.Y, IntPt.Z);

		UE_LOG(LogAAATraffic, Log,
			TEXT("  %s"),
			*StartDesc);

		UE_LOG(LogAAATraffic, Log,
			TEXT("  %s"),
			*EndDesc);

		UE_LOG(LogAAATraffic, Log,
			TEXT("  StartDist=%.1f EndDist=%.1f StartOffset=%.1f EndOffset=%.1f Radius=%.1f DirA=%d DirB=%d"),
			StartDist, EndDist, StartOffset, EndOffset, Radius, DirA, DirB);
	}

	UE_LOG(LogAAATraffic, Log, TEXT("===== END CORNER DIAGNOSTIC DUMP ====="));
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

	// ── Step 2: Identify through-roads (roads with 2+ masks) ────
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

		// Only roads with 2+ masks need splitting.
		if (MaskIndices.Num() < 2) { continue; }

		const int32* RoadHandlePtr = RoadActorToHandle.Find(RoadActor);
		if (!RoadHandlePtr) { continue; }
		const int32 RoadHandle = *RoadHandlePtr;

		const TArray<int32>* LaneHandles = RoadToLaneHandles.Find(RoadHandle);
		if (!LaneHandles || LaneHandles->Num() == 0) { continue; }

		const double RoadLength = GetRoadLength(RoadActor);
		if (RoadLength <= 0.0) { continue; }

		++ThroughRoadCount;

		UE_LOG(LogAAATraffic, Log,
			TEXT("  Through-road %s (handle=%d, length=%.1f): %d masks"),
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
			};
			TArray<FSplitPoint> SplitPoints;

			for (const int32 MaskIdx : MaskIndices)
			{
				const FIntersectionMaskInfo& Mask = CachedIntersectionMasks[MaskIdx];

				// Convert reference-line distance to polyline index.
				// Polyline is sampled uniformly: point[i] = distance (RoadLength * i) / (NumSamples - 1)
				// So: index = round(Distance / RoadLength * (TotalPoints - 1))
				const float StartNorm = static_cast<float>(Mask.StartDistance / RoadLength);
				const float EndNorm = static_cast<float>(Mask.EndDistance / RoadLength);

				int32 StartIdx = FMath::RoundToInt32(StartNorm * static_cast<float>(TotalPoints - 1));
				int32 EndIdx = FMath::RoundToInt32(EndNorm * static_cast<float>(TotalPoints - 1));

				// Clamp to valid range (never at very first or last point).
				StartIdx = FMath::Clamp(StartIdx, 1, TotalPoints - 2);
				EndIdx = FMath::Clamp(EndIdx, StartIdx + 1, TotalPoints - 1);

				// Mark start of intersection segment and start of free segment after it.
				SplitPoints.Add({ StartIdx, Mask.GroupId });
				if (EndIdx < TotalPoints - 1)
				{
					SplitPoints.Add({ EndIdx, 0 }); // Free segment starts after intersection
				}
				// If EndIdx == TotalPoints - 1, the mask extends to the very end
				// of the road. No free segment split is added because there is no
				// road beyond this point. The final virtual segment (from StartIdx
				// to TotalPoints-1) will carry the mask's GroupId, which is the
				// correct behavior — the road ends inside the intersection.

				UE_LOG(LogAAATraffic, Log,
					TEXT("    Lane %d: Mask group=%d → StartDist=%.1f (idx=%d), EndDist=%.1f (idx=%d)"),
					LaneHandle, Mask.GroupId, Mask.StartDistance, StartIdx, Mask.EndDistance, EndIdx);
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
			TArray<int32> VirtualHandles;
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

	UE_LOG(LogAAATraffic, Log,
		TEXT("RoadBLDReflectionProvider: Through-road splitting — %d through-roads, %d virtual segments from %d source lanes."),
		ThroughRoadCount, TotalVirtuals, ReplacedLaneHandles.Num());
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

			// Map these lanes to the junction.
			LaneToJunctionMap.FindOrAdd(PC.FromLane) = GroupId;
			LaneToJunctionMap.FindOrAdd(PC.ToLane) = GroupId;
		}

		UE_LOG(LogAAATraffic, Log,
			TEXT("RoadBLDReflectionProvider: Junction grouping — %d mask junctions, %d proximity connections (%d matched to junctions, %d skipped as road continuations), %d total junctions, %d lanes mapped."),
			IntersectionGroupCentroids.Num(), ProximityConnectionList.Num(),
			PCMatched, PCSkipped, JunctionCentroids.Num(), LaneToJunctionMap.Num());
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

// ---------------------------------------------------------------------------
// ITrafficRoadProvider — junction queries
// ---------------------------------------------------------------------------

int32 URoadBLDReflectionProvider::GetJunctionForLane(const FTrafficLaneHandle& Lane)
{
	// Check the handle directly first (works for both virtual and original).
	if (const int32* JId = LaneToJunctionMap.Find(Lane.HandleId))
	{
		return *JId;
	}
	// Virtual lane — also check via original handle.
	if (const FVirtualLaneInfo* VInfo = VirtualLaneMap.Find(Lane.HandleId))
	{
		if (const int32* JId = LaneToJunctionMap.Find(VInfo->OriginalLaneHandle))
		{
			return *JId;
		}
	}
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

bool URoadBLDReflectionProvider::GetJunctionPath(
	const FTrafficLaneHandle& FromLane,
	const FTrafficLaneHandle& ToLane,
	TArray<FVector>& OutPath)
{
	// Resolve junction for FromLane (virtual-aware).
	const int32 FromJunctionId = GetJunctionForLane(FromLane);
	if (FromJunctionId == 0)
	{
		return false;
	}

	// Validate that ToLane maps to the same junction.
	const int32 ToJunctionId = GetJunctionForLane(ToLane);
	if (ToJunctionId != FromJunctionId)
	{
		return false;
	}

	const FVector* Centroid = JunctionCentroids.Find(FromJunctionId);
	if (!Centroid)
	{
		return false;
	}

	// Build a 3-point path: FromLane endpoint → junction centroid → ToLane startpoint.
	TArray<FVector> FromPoints;
	float FromWidth;
	TArray<FVector> ToPoints;
	float ToWidth;

	if (!GetLanePath(FromLane, FromPoints, FromWidth) || FromPoints.Num() == 0)
	{
		return false;
	}
	if (!GetLanePath(ToLane, ToPoints, ToWidth) || ToPoints.Num() == 0)
	{
		return false;
	}

	OutPath.Reset(3);
	OutPath.Add(FromPoints.Last());
	OutPath.Add(*Centroid);
	OutPath.Add(ToPoints[0]);
	return true;
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

	const TMap<int32, int32>& Map = (Side == ETrafficLaneSide::Left) ? LeftNeighborMap : RightNeighborMap;
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

float URoadBLDReflectionProvider::GetLaneSpeedLimit(const FTrafficLaneHandle& /*Lane*/)
{
	// RoadBLD does not expose per-lane speed limits. Return -1 so the caller
	// knows to fall back to its own default speed.
	return -1.0f;
}

// ---------------------------------------------------------------------------
// Reflection helpers
// ---------------------------------------------------------------------------

void URoadBLDReflectionProvider::CallReflection(UObject* Target, UFunction* Func, void* Params)
{
	check(Target && Func);
	Target->ProcessEvent(Func, Params);
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
