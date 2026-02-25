// Copyright AAA_Traffic Contributors. All Rights Reserved.

#include "TrafficRoadQueryLibrary.h"
#include "TrafficSubsystem.h"
#include "Engine/Engine.h"
#include "Engine/World.h"

// ---------------------------------------------------------------------------
// Helper: resolve the active provider from a world context object.
// ---------------------------------------------------------------------------
static ITrafficRoadProvider* ResolveProvider(const UObject* WorldContextObject)
{
	if (!WorldContextObject) { return nullptr; }
	const UWorld* World = GEngine->GetWorldFromContextObject(WorldContextObject, EGetWorldErrorMode::ReturnNull);
	if (!World) { return nullptr; }
	UTrafficSubsystem* Sub = World->GetSubsystem<UTrafficSubsystem>();
	return Sub ? Sub->GetProvider() : nullptr;
}

// ---------------------------------------------------------------------------
// Blueprint-callable wrappers
// ---------------------------------------------------------------------------

TArray<FTrafficRoadHandle> UTrafficRoadQueryLibrary::GetAllRoads(const UObject* WorldContextObject)
{
	ITrafficRoadProvider* P = ResolveProvider(WorldContextObject);
	return P ? P->GetAllRoads() : TArray<FTrafficRoadHandle>();
}

TArray<FTrafficLaneHandle> UTrafficRoadQueryLibrary::GetLanesForRoad(const UObject* WorldContextObject, const FTrafficRoadHandle& Road)
{
	ITrafficRoadProvider* P = ResolveProvider(WorldContextObject);
	return P ? P->GetLanesForRoad(Road) : TArray<FTrafficLaneHandle>();
}

bool UTrafficRoadQueryLibrary::GetLanePath(const UObject* WorldContextObject, const FTrafficLaneHandle& Lane,
	TArray<FVector>& OutPoints, float& OutWidth)
{
	ITrafficRoadProvider* P = ResolveProvider(WorldContextObject);
	if (!P) { OutPoints.Empty(); OutWidth = 0.0f; return false; }
	return P->GetLanePath(Lane, OutPoints, OutWidth);
}

FVector UTrafficRoadQueryLibrary::GetLaneDirection(const UObject* WorldContextObject, const FTrafficLaneHandle& Lane)
{
	ITrafficRoadProvider* P = ResolveProvider(WorldContextObject);
	return P ? P->GetLaneDirection(Lane) : FVector::ZeroVector;
}

TArray<FTrafficLaneHandle> UTrafficRoadQueryLibrary::GetConnectedLanes(const UObject* WorldContextObject, const FTrafficLaneHandle& Lane)
{
	ITrafficRoadProvider* P = ResolveProvider(WorldContextObject);
	return P ? P->GetConnectedLanes(Lane) : TArray<FTrafficLaneHandle>();
}

FTrafficLaneHandle UTrafficRoadQueryLibrary::GetLaneAtLocation(const UObject* WorldContextObject, const FVector& Location)
{
	ITrafficRoadProvider* P = ResolveProvider(WorldContextObject);
	return P ? P->GetLaneAtLocation(Location) : FTrafficLaneHandle();
}

FTrafficLaneHandle UTrafficRoadQueryLibrary::GetAdjacentLane(const UObject* WorldContextObject,
	const FTrafficLaneHandle& Lane, ETrafficLaneSide Side)
{
	ITrafficRoadProvider* P = ResolveProvider(WorldContextObject);
	return P ? P->GetAdjacentLane(Lane, Side) : FTrafficLaneHandle();
}

FTrafficRoadHandle UTrafficRoadQueryLibrary::GetRoadForLane(const UObject* WorldContextObject, const FTrafficLaneHandle& Lane)
{
	ITrafficRoadProvider* P = ResolveProvider(WorldContextObject);
	return P ? P->GetRoadForLane(Lane) : FTrafficRoadHandle();
}

float UTrafficRoadQueryLibrary::GetLaneSpeedLimit(const UObject* WorldContextObject, const FTrafficLaneHandle& Lane)
{
	ITrafficRoadProvider* P = ResolveProvider(WorldContextObject);
	return P ? P->GetLaneSpeedLimit(Lane) : -1.0f;
}

int32 UTrafficRoadQueryLibrary::GetJunctionForLane(const UObject* WorldContextObject, const FTrafficLaneHandle& Lane)
{
	ITrafficRoadProvider* P = ResolveProvider(WorldContextObject);
	return P ? P->GetJunctionForLane(Lane) : 0;
}
