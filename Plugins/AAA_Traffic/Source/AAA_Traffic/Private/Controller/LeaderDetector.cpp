// LeaderDetector.cpp — Leader vehicle detection implementation.

#include "LeaderDetector.h"
#include "TrafficSubsystem.h"
#include "TrafficVehicleController.h"
#include "TrafficLog.h"
#include "GameFramework/PawnMovementComponent.h"
#include "Engine/World.h"

// ─────────────────────────────────────────────────────────────
// Public entry point
// ─────────────────────────────────────────────────────────────

FLeaderDetectorOutput FLeaderDetector::Detect(const FLeaderDetectorInput& In) const
{
	FLeaderDetectorOutput Out;

	if (In.TickLOD == ETrafficLOD::Full)
	{
		// Full-LOD: polyline-following physics sweep.
		float Speed = 0.0f;
		float Dist = -1.0f;

		// Choose sweep strategy.
		if (In.LaneChangeState != ELaneChangeState::None)
		{
			Dist = SweepStraight(In, Speed);
		}
		else if (In.JunctionId != 0 && In.JunctionTransitionPoints.Num() >= 2)
		{
			Dist = SweepPolyline(In, Speed);
		}
		else if (In.LanePoints.Num() >= 2)
		{
			Dist = SweepPolyline(In, Speed);
		}
		else
		{
			Dist = SweepStraight(In, Speed);
		}

		// Spatial fallback: if physics sweep found nothing, query the
		// spatial grid for nearby vehicles on the same or adjacent lanes.
		// This catches cases where the sphere sweep is blocked by road
		// geometry and also detects vehicles on adjacent lanes that
		// physically overlap near junctions.
		if (Dist < 0.0f && In.TrafficSubsystem)
		{
			TArray<ATrafficVehicleController*> Nearby =
				In.TrafficSubsystem->GetNearbyVehicles(In.VehicleLocation, In.DetectionDistance);

			float BestDist = MAX_FLT;
			float BestSpeed = 0.0f;

			for (const ATrafficVehicleController* Other : Nearby)
			{
				if (!Other || Other->GetPawn() == In.ControlledPawn) { continue; }
				const APawn* OP = Other->GetPawn();
				if (!OP) { continue; }

				// Heading alignment — reject any vehicle facing the
				// opposite direction (oncoming traffic). A leader is
				// always travelling roughly the same way we are.
				if (FVector::DotProduct(OP->GetActorForwardVector(),
						In.VehicleForward) < 0.0f)
				{
					continue;
				}

				const FVector Delta = OP->GetActorLocation() - In.VehicleLocation;
				const float D = Delta.Size();
				if (D < KINDA_SMALL_NUMBER) { continue; }
				if (FVector::DotProduct(Delta / D, In.VehicleForward) < 0.5f) { continue; }
				if (D > In.DetectionDistance) { continue; }
				if (D < BestDist)
				{
					BestDist = D;
					if (const UPawnMovementComponent* OtherMC = OP->GetMovementComponent())
					{
						BestSpeed = FVector::DotProduct(OtherMC->Velocity, In.VehicleForward);
					}
					else
					{
						BestSpeed = 0.0f;
					}
				}
			}

			if (BestDist < MAX_FLT)
			{
				// Subtract leader's rear extent to convert center-to-center
				// distance to center-to-rear-bumper.  The shared bumper
				// correction below subtracts VehicleFrontExtent, yielding
				// true front-bumper-to-rear-bumper gap.
				Dist = BestDist - In.VehicleRearExtent;
				Speed = BestSpeed;
			}
		}

		// Bumper-to-bumper correction.
		if (Dist >= 0.0f)
		{
			Dist = FMath::Max(Dist - In.VehicleFrontExtent, 1.0f);
		}
		Out.LeaderDist = Dist;
		Out.LeaderSpeed = Speed;
	}
	else if (In.TickLOD == ETrafficLOD::Reduced)
	{
		// Reduced-LOD: analytical detection via spatial grid.
		if (In.TrafficSubsystem)
		{
			TArray<ATrafficVehicleController*> Nearby =
				In.TrafficSubsystem->GetNearbyVehicles(In.VehicleLocation, In.DetectionDistance);

			float BestDist = MAX_FLT;
			float BestSpeed = 0.0f;

			for (const ATrafficVehicleController* Other : Nearby)
			{
				if (!Other || Other->GetPawn() == In.ControlledPawn) { continue; }
				const APawn* OP = Other->GetPawn();
				if (!OP) { continue; }

				// Heading alignment — reject oncoming traffic.
				if (FVector::DotProduct(OP->GetActorForwardVector(),
						In.VehicleForward) < 0.0f)
				{
					continue;
				}

				const FVector Delta = OP->GetActorLocation() - In.VehicleLocation;
				const float Dist = Delta.Size();
				if (Dist < KINDA_SMALL_NUMBER
					|| FVector::DotProduct(Delta / Dist, In.VehicleForward) < 0.5f)
				{
					continue;
				}
				if (Dist < BestDist)
				{
					BestDist = Dist;
					if (const UPawnMovementComponent* OtherMC = OP->GetMovementComponent())
					{
						BestSpeed = FVector::DotProduct(OtherMC->Velocity, In.VehicleForward);
					}
					else
					{
						BestSpeed = 0.0f;
					}
				}
			}

			if (BestDist < In.DetectionDistance)
			{
				// Subtract both ego front extent and leader rear extent
				// to convert center-to-center to true bumper-to-bumper gap.
				Out.LeaderDist = FMath::Max(BestDist - In.VehicleFrontExtent - In.VehicleRearExtent, 1.0f);
				Out.LeaderSpeed = BestSpeed;
			}
		}
	}
	else if (In.TickLOD == ETrafficLOD::Minimal)
	{
		// Minimal LOD: no leader detection to preserve frame budget.
		Out.LeaderDist = -1.0f;
		Out.LeaderSpeed = 0.0f;
	}

	// Brake-light perception.
	DetectBrakeLights(In, Out.LeaderDist, Out);

	return Out;
}

// ─────────────────────────────────────────────────────────────
// Polyline-following sweep (full LOD)
// ─────────────────────────────────────────────────────────────

float FLeaderDetector::SweepPolyline(const FLeaderDetectorInput& In, float& OutSpeed) const
{
	OutSpeed = 0.0f;
	if (!In.World || !In.ControlledPawn) { return -1.0f; }

	const float SweepRadius = FMath::Max(In.LaneWidth * 0.4f, 50.0f);
	const float SelfBuffer = 10.0f;

	FCollisionQueryParams QP;
	QP.AddIgnoredActor(In.ControlledPawn);
	QP.bTraceComplex = false;

	// Select polyline.
	TArrayView<const FVector> Pts;
	int32 StartIdx = 0;
	if (In.JunctionId != 0 && In.JunctionTransitionPoints.Num() >= 2)
	{
		Pts = In.JunctionTransitionPoints;
		StartIdx = FMath::Clamp(In.JunctionTransitionIndex, 0, Pts.Num() - 1);
	}
	else
	{
		Pts = In.LanePoints;
		StartIdx = FMath::Clamp(In.ClosestIndex, 0, Pts.Num() - 1);
	}
	if (Pts.Num() < 2) { return -1.0f; }

	float AccumDist = 0.0f;
	FVector SegStart = In.VehicleLocation;
	bool bFirst = true;

	for (int32 i = StartIdx; i < Pts.Num() - 1; ++i)
	{
		const FVector& SegEnd = Pts[i + 1];
		FVector SegDir = (SegEnd - SegStart).GetSafeNormal();
		if (SegDir.IsNearlyZero()) { SegStart = SegEnd; continue; }

		const float SegLen = FVector::Dist(SegStart, SegEnd);
		const float Budget = In.DetectionDistance - AccumDist;
		if (Budget <= 0.0f) { break; }

		const FVector SwStart = bFirst
			? (SegStart + SegDir * (SweepRadius + SelfBuffer))
			: SegStart;
		const float SweepLen = FMath::Min(SegLen, Budget);
		const FVector SwEnd = SwStart + SegDir * SweepLen;

		FHitResult Hit;
		if (In.World->SweepSingleByChannel(
				Hit, SwStart, SwEnd, FQuat::Identity,
				ECC_Pawn, FCollisionShape::MakeSphere(SweepRadius), QP)
			&& Hit.GetActor())
		{
			const APawn* HP = Cast<APawn>(Hit.GetActor());
			if (HP && FVector::DotProduct(HP->GetActorForwardVector(), SegDir) >= 0.0f)
			{
				if (const UPawnMovementComponent* LM = HP->GetMovementComponent())
				{
					OutSpeed = FVector::DotProduct(LM->Velocity, SegDir);
				}
				// Correct distance: add sphere contact radius + first-seg
				// start offset to convert from sweep-relative to
				// VehicleLocation-relative distance.
				const float StartCorrection = bFirst
					? (SweepRadius + SelfBuffer) : 0.0f;
				return AccumDist + Hit.Distance
					+ SweepRadius + StartCorrection;
			}
		}

		AccumDist += SweepLen;
		SegStart = SegEnd;
		bFirst = false;
	}

	// Extension past polyline end.
	const float ExtBudget = In.DetectionDistance - AccumDist;
	if (ExtBudget > 0.0f && Pts.Num() >= 2)
	{
		const FVector LastDir = (Pts[Pts.Num() - 1] - Pts[Pts.Num() - 2]).GetSafeNormal();
		if (!LastDir.IsNearlyZero())
		{
			const FVector ExtEnd = SegStart + LastDir * ExtBudget;
			FHitResult Hit;
			if (In.World->SweepSingleByChannel(
					Hit, SegStart, ExtEnd, FQuat::Identity,
					ECC_Pawn, FCollisionShape::MakeSphere(SweepRadius), QP)
				&& Hit.GetActor())
			{
				const APawn* HP = Cast<APawn>(Hit.GetActor());
				if (HP && FVector::DotProduct(HP->GetActorForwardVector(), LastDir) >= 0.0f)
				{
					if (const UPawnMovementComponent* LM = HP->GetMovementComponent())
					{
						OutSpeed = FVector::DotProduct(LM->Velocity, LastDir);
					}
					return AccumDist + Hit.Distance + SweepRadius;
				}
			}
		}
	}

	return -1.0f;
}

// ─────────────────────────────────────────────────────────────
// Straight-line sweep fallback
// ─────────────────────────────────────────────────────────────

float FLeaderDetector::SweepStraight(const FLeaderDetectorInput& In, float& OutSpeed) const
{
	OutSpeed = 0.0f;
	if (!In.World || !In.ControlledPawn) { return -1.0f; }

	const float SweepRadius = FMath::Max(In.LaneWidth * 0.4f, 50.0f);
	const float SelfBuffer = 10.0f;

	FCollisionQueryParams QP;
	QP.AddIgnoredActor(In.ControlledPawn);
	QP.bTraceComplex = false;

	const FVector Dir   = In.VehicleForward;
	const FVector Start = In.VehicleLocation + Dir * (SweepRadius + SelfBuffer);
	const FVector End   = Start + Dir * In.DetectionDistance;

	FHitResult Hit;
	if (!In.World->SweepSingleByChannel(
			Hit, Start, End, FQuat::Identity,
			ECC_Pawn, FCollisionShape::MakeSphere(SweepRadius), QP)
		|| !Hit.GetActor())
	{
		return -1.0f;
	}

	const APawn* HP = Cast<APawn>(Hit.GetActor());
	if (!HP) { return -1.0f; }
	if (FVector::DotProduct(HP->GetActorForwardVector(), Dir) < 0.0f) { return -1.0f; }

	if (const UPawnMovementComponent* LM = HP->GetMovementComponent())
	{
		OutSpeed = FVector::DotProduct(LM->Velocity, Dir);
	}
	return Hit.Distance + 2.0f * SweepRadius + SelfBuffer;
}

// ─────────────────────────────────────────────────────────────
// Brake-light perception
// ─────────────────────────────────────────────────────────────

void FLeaderDetector::DetectBrakeLights(
	const FLeaderDetectorInput& In, float LeaderDist,
	FLeaderDetectorOutput& Out) const
{
	Out.bLeaderBrakeLightsVisible = false;

	if (LeaderDist < 0.0f || In.IDMReactionDelaySec <= KINDA_SMALL_NUMBER)
	{
		return;
	}
	if (!In.TrafficSubsystem || !In.CurrentLane.IsValid()) { return; }

	TArray<TWeakObjectPtr<ATrafficVehicleController>> LaneVehicles =
		In.TrafficSubsystem->GetVehiclesOnLane(In.CurrentLane);

	float NearestAheadDist = MAX_FLT;
	const ATrafficVehicleController* NearestLeader = nullptr;

	for (const TWeakObjectPtr<ATrafficVehicleController>& WeakOther : LaneVehicles)
	{
		const ATrafficVehicleController* Other = WeakOther.Get();
		if (!Other || Other->GetPawn() == In.ControlledPawn) { continue; }
		const APawn* OP = Other->GetPawn();
		if (!OP) { continue; }

		const FVector Delta = OP->GetActorLocation() - In.VehicleLocation;
		const float Dist = Delta.Size();
		if (Dist < KINDA_SMALL_NUMBER) { continue; }
		if (FVector::DotProduct(Delta / Dist, In.VehicleForward) < 0.5f) { continue; }
		if (Dist < NearestAheadDist)
		{
			NearestAheadDist = Dist;
			NearestLeader = Other;
		}
	}

	if (NearestLeader && NearestLeader->AreBrakeLightsOn())
	{
		Out.bLeaderBrakeLightsVisible = true;
	}
}
