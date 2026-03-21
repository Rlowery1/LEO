// Copyright AAA_Traffic Contributors. All Rights Reserved.

#include "RoadBLDReflectionProvider.h"
#include "TrafficLog.h"
#include "Engine/World.h"
#include "EngineUtils.h"

// CVars defined in RoadBLDReflectionProvider.cpp
extern bool GEnableDiagnosticDumps;
extern int32 GTrafficDiagnosticsLevel;
extern bool GTrafficDiagnosticsValidateGraph;

namespace
{
	static bool ShouldLogDiagnostics(const int32 Level)
	{
		return GEnableDiagnosticDumps || GTrafficDiagnosticsLevel >= Level;
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

	// ── Post-rebuild array inventory (gated behind CVar) ─────────
	// Log the count of every corner-related array so we know
	// exactly where CornerBuilder stored its data.
	if (GEnableDiagnosticDumps)
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