// Copyright AAA_Traffic Contributors. All Rights Reserved.

#pragma once

#include "CoreMinimal.h"

/**
 * Runtime API-contract validation for RoadBLD reflection access.
 *
 * The reflection-based provider depends on specific UClass names, UFUNCTION
 * signatures, and UPROPERTY names in the RoadBLDRuntime module.  If a future
 * WorldBLD update renames or removes any of these symbols, the provider must
 * detect the breakage *before* it tries to call a stale function pointer.
 *
 * Usage:
 *   FRoadBLDAPIContractResult Result = FRoadBLDAPIContract::Validate();
 *   if (!Result.bAllPassed) { /* log Result.Failures and bail * / }
 *
 * The baseline was captured from RoadBLD 1.2.1 (shipped with WorldBLD).
 */

// ---------------------------------------------------------------------------
// Contract-check result
// ---------------------------------------------------------------------------

struct FRoadBLDContractFailure
{
	FString Category;   // e.g. "UClass", "UFunction", "FProperty"
	FString Symbol;     // e.g. "/Script/RoadBLDRuntime.DynamicRoad"
	FString Detail;     // human-readable reason

	FString ToString() const
	{
		return FString::Printf(TEXT("[%s] %s — %s"), *Category, *Symbol, *Detail);
	}
};

struct FRoadBLDAPIContractResult
{
	bool bAllPassed = true;
	TArray<FRoadBLDContractFailure> Failures;

	void AddFailure(const FString& Category, const FString& Symbol, const FString& Detail)
	{
		bAllPassed = false;
		Failures.Add({ Category, Symbol, Detail });
	}
};

// ---------------------------------------------------------------------------
// Contract validator
// ---------------------------------------------------------------------------

struct AAA_TRAFFIC_API FRoadBLDAPIContract
{
	/**
	 * Validate that every class, function, and property the reflection provider
	 * depends on is present in the loaded RoadBLDRuntime module.
	 *
	 * This is called once during provider initialization.  If it returns failures,
	 * the reflection provider refuses to register and logs diagnostics.
	 */
	static FRoadBLDAPIContractResult Validate()
	{
		FRoadBLDAPIContractResult Result;

		// ── UClasses ───────────────────────────────────────────────
		struct FClassEntry
		{
			const TCHAR* Path;
			TArray<const TCHAR*> RequiredFunctions;
			TArray<const TCHAR*> RequiredProperties;
		};

		// DynamicRoad
		FClassEntry DynamicRoad;
		DynamicRoad.Path = TEXT("/Script/RoadBLDRuntime.DynamicRoad");
		DynamicRoad.RequiredFunctions = {
			TEXT("GetLength"),
			TEXT("GetAllLanes"),
			TEXT("ConvertDistanceBetweenCurves"),
			TEXT("GetWorldPositionAtDistance")
		};
		DynamicRoad.RequiredProperties = {
			TEXT("ReferenceLine")
		};

		// DynamicRoadNetwork
		FClassEntry DynamicRoadNetwork;
		DynamicRoadNetwork.Path = TEXT("/Script/RoadBLDRuntime.DynamicRoadNetwork");
		DynamicRoadNetwork.RequiredFunctions = {};
		DynamicRoadNetwork.RequiredProperties = {
			TEXT("RoadNetworkCorners")
		};

		// We discover lane / curve classes at runtime from instances, but we
		// can at least verify the lane property names we depend on.
		// These are checked separately during CacheRoadData (lazy resolve).

		TArray<FClassEntry> Classes = { DynamicRoad, DynamicRoadNetwork };

		for (const FClassEntry& Entry : Classes)
		{
			UClass* Class = FindObject<UClass>(nullptr, Entry.Path);
			if (!Class)
			{
				Result.AddFailure(TEXT("UClass"), Entry.Path, TEXT("Class not found in loaded modules."));
				continue;  // Can't check members without the class.
			}

			for (const TCHAR* FuncName : Entry.RequiredFunctions)
			{
				if (!Class->FindFunctionByName(FuncName))
				{
					Result.AddFailure(TEXT("UFunction"), 
						FString::Printf(TEXT("%s::%s"), Entry.Path, FuncName),
						TEXT("Function not found on class."));
				}
			}

			for (const TCHAR* PropName : Entry.RequiredProperties)
			{
				if (!Class->FindPropertyByName(FName(PropName)))
				{
					Result.AddFailure(TEXT("FProperty"),
						FString::Printf(TEXT("%s::%s"), Entry.Path, PropName),
						TEXT("Property not found on class."));
				}
			}
		}

		// ── Lane properties (checked if DynamicRoad exists) ─────────
		// These are on UDynamicRoadLane, which we discover from instances.
		// We store the names here as documentation of the contract; actual
		// validation happens lazily when the first lane is resolved.

		return Result;
	}

	/**
	 * Validate lane-class properties against the expected contract.
	 * Called once when the first lane UObject is discovered.
	 *
	 * @param LaneClass  The UClass of the first lane object.
	 * @return           Contract result for lane-specific symbols.
	 */
	static FRoadBLDAPIContractResult ValidateLaneClass(UClass* LaneClass)
	{
		FRoadBLDAPIContractResult Result;
		if (!LaneClass)
		{
			Result.AddFailure(TEXT("UClass"), TEXT("LaneClass"), TEXT("Null lane class."));
			return Result;
		}

		static const TCHAR* RequiredLaneProperties[] = {
			TEXT("LeftEdgeCurve"),
			TEXT("RightEdgeCurve"),
			TEXT("LaneWidth")
		};

		for (const TCHAR* PropName : RequiredLaneProperties)
		{
			if (!LaneClass->FindPropertyByName(FName(PropName)))
			{
				Result.AddFailure(TEXT("FProperty"),
					FString::Printf(TEXT("%s::%s"), *LaneClass->GetPathName(), PropName),
					TEXT("Property not found on lane class."));
			}
		}

		return Result;
	}

	/**
	 * Validate curve-class function (Get3DPositionAtDistance).
	 * Called once when the first edge curve UObject is discovered.
	 */
	static FRoadBLDAPIContractResult ValidateCurveClass(UClass* CurveClass)
	{
		FRoadBLDAPIContractResult Result;
		if (!CurveClass)
		{
			Result.AddFailure(TEXT("UClass"), TEXT("CurveClass"), TEXT("Null curve class."));
			return Result;
		}

		if (!CurveClass->FindFunctionByName(TEXT("Get3DPositionAtDistance")))
		{
			Result.AddFailure(TEXT("UFunction"),
				FString::Printf(TEXT("%s::Get3DPositionAtDistance"), *CurveClass->GetPathName()),
				TEXT("Function not found on curve class."));
		}

		return Result;
	}
};
