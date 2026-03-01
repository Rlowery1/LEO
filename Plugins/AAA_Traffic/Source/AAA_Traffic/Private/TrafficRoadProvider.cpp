// Copyright AAA_Traffic Contributors. All Rights Reserved.

#include "TrafficRoadProvider.h"

ITrafficRoadProvider::FJunctionScanResult ITrafficRoadProvider::GetDistanceToNextJunction(
	const FTrafficLaneHandle& StartLane,
	float RemainingDistOnCurrentLane,
	float MaxSearchDistCm,
	int32 MaxHops)
{
	FJunctionScanResult Result;

	if (!StartLane.IsValid() || MaxSearchDistCm <= 0.0f || MaxHops <= 0)
	{
		return Result;
	}

	// Distance accumulated so far = remaining distance on the current lane.
	float AccumulatedDist = RemainingDistOnCurrentLane;

	// Check if current lane IS already a junction.
	const int32 CurrentJunction = GetJunctionForLane(StartLane);
	if (CurrentJunction != 0)
	{
		Result.JunctionId = CurrentJunction;
		Result.DistanceCm = 0.0f; // Already on a junction lane.
		Result.JunctionLane = StartLane;
		Result.ApproachLane = StartLane;
		return Result;
	}

	// Visited set to prevent infinite cycles in the lane graph.
	TSet<int32> Visited;
	Visited.Add(StartLane.HandleId);

	FTrafficLaneHandle CurrentLane = StartLane;

	for (int32 Hop = 0; Hop < MaxHops; ++Hop)
	{
		// Get connected (successor) lanes from the current lane.
		TArray<FTrafficLaneHandle> NextLanes = GetConnectedLanes(CurrentLane);
		if (NextLanes.Num() == 0)
		{
			break; // Dead end — no junction ahead.
		}

		// Sort by HandleId for deterministic traversal regardless of provider
		// container ordering.  O(N log N) on a typically tiny array (1-3 lanes).
		NextLanes.Sort([](const FTrafficLaneHandle& A, const FTrafficLaneHandle& B)
		{
			return A.HandleId < B.HandleId;
		});

		// Follow the first unvisited successor.
		bool bFoundNext = false;
		for (const FTrafficLaneHandle& NextLane : NextLanes)
		{
			if (Visited.Contains(NextLane.HandleId))
			{
				continue;
			}

			Visited.Add(NextLane.HandleId);

			// Check if this next lane is a junction.
			const int32 NextJunction = GetJunctionForLane(NextLane);
			if (NextJunction != 0)
			{
				Result.JunctionId = NextJunction;
				Result.DistanceCm = AccumulatedDist;
				Result.JunctionLane = NextLane;
				Result.ApproachLane = CurrentLane;
				return Result;
			}

			// Accumulate the full length of this next lane.
			const float NextLength = GetLaneLength(NextLane);
			AccumulatedDist += NextLength;

			if (AccumulatedDist > MaxSearchDistCm)
			{
				return Result; // Exceeded search range — no junction found.
			}

			CurrentLane = NextLane;
			bFoundNext = true;
			break; // Follow only one path (avoid exponential branching).
		}

		if (!bFoundNext)
		{
			break; // All successors already visited — cycle detected.
		}
	}

	return Result;
}
