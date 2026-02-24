// Copyright AAA_Traffic Contributors. All Rights Reserved.

#include "TrafficVehicleController.h"
#include "TrafficSubsystem.h"
#include "TrafficLog.h"
#include "ChaosWheeledVehicleMovementComponent.h"
#include "GameFramework/Pawn.h"

ATrafficVehicleController::ATrafficVehicleController()
	: LaneWidth(0.f)
	, bLaneDataReady(false)
	, CachedProvider(nullptr)
	, bAtDeadEnd(false)
	, DistanceTraveledOnLane(0.0f)
	, PreviousVehicleLocation(FVector::ZeroVector)
	, TargetSpeed(1500.f)
	, LookAheadDistance(500.f)
	, RandomSeed(0)
{
	PrimaryActorTick.bCanEverTick = true;
}

void ATrafficVehicleController::SetTargetSpeed(float InSpeed)
{
	TargetSpeed = FMath::Max(0.0f, InSpeed);
}

void ATrafficVehicleController::SetRandomSeed(int32 InSeed)
{
	RandomSeed = InSeed;
}

void ATrafficVehicleController::InitializeLaneFollowing(const FTrafficLaneHandle& InLane)
{
	CurrentLane = InLane;
	bLaneDataReady = false;
	bAtDeadEnd = false;
	DistanceTraveledOnLane = 0.0f;
	PreviousVehicleLocation = FVector::ZeroVector;

	UWorld* World = GetWorld();
	if (!World)
	{
		UE_LOG(LogAAATraffic, Warning, TEXT("TrafficVehicleController: World is null, cannot initialize lane following."));
		return;
	}
	UTrafficSubsystem* TrafficSub = World->GetSubsystem<UTrafficSubsystem>();
	ITrafficRoadProvider* Provider = TrafficSub ? TrafficSub->GetProvider() : nullptr;
	if (!Provider)
	{
		UE_LOG(LogAAATraffic, Warning, TEXT("TrafficVehicleController: No road provider available."));
		return;
	}

	CachedProvider = Provider;

	LanePoints.Reset();
	if (Provider->GetLanePath(CurrentLane, LanePoints, LaneWidth) && LanePoints.Num() >= 2)
	{
		bLaneDataReady = true;
		UE_LOG(LogAAATraffic, Log,
			TEXT("TrafficVehicleController: Lane loaded — %d points, width %.1f cm."),
			LanePoints.Num(), LaneWidth);
	}
	else
	{
		UE_LOG(LogAAATraffic, Warning, TEXT("TrafficVehicleController: Failed to load lane path data."));
	}
}

void ATrafficVehicleController::OnPossess(APawn* InPawn)
{
	Super::OnPossess(InPawn);
	RandomStream.Initialize(RandomSeed);
}

void ATrafficVehicleController::Tick(float DeltaSeconds)
{
	Super::Tick(DeltaSeconds);

	if (!bLaneDataReady || !GetPawn())
	{
		return;
	}

	UpdateVehicleInput(DeltaSeconds);
}

void ATrafficVehicleController::UpdateVehicleInput(float DeltaSeconds)
{
	APawn* ControlledPawn = GetPawn();
	if (!ControlledPawn)
	{
		return;
	}

	UChaosWheeledVehicleMovementComponent* VehicleMovement =
		Cast<UChaosWheeledVehicleMovementComponent>(ControlledPawn->GetMovementComponent());
	if (!VehicleMovement)
	{
		return;
	}

	const FVector VehicleLocation = ControlledPawn->GetActorLocation();
	const FVector VehicleForward = ControlledPawn->GetActorForwardVector();
	const float CurrentSpeed = VehicleMovement->GetForwardSpeed();

	// Track cumulative distance traveled on this lane to prevent short-lane transition loops.
	if (!PreviousVehicleLocation.IsZero())
	{
		DistanceTraveledOnLane += FVector::Dist(PreviousVehicleLocation, VehicleLocation);
	}
	PreviousVehicleLocation = VehicleLocation;

	// Find where we are on the lane and where to aim
	const int32 ClosestIndex = FindClosestPointIndex(VehicleLocation);

	// --- Lane-end detection ---
	if (!bAtDeadEnd)
	{
		const float RemainingDist = GetRemainingDistance(ClosestIndex);
		// Use speed-based look-ahead so fast vehicles get more advance notice.
		const float ReactionTime = 1.0f; // seconds
		const float TransitionThreshold = FMath::Max(LookAheadDistance, FMath::Abs(CurrentSpeed) * ReactionTime);
		if (RemainingDist < TransitionThreshold && DistanceTraveledOnLane > TransitionThreshold)
		{
			CheckLaneTransition();
			if (!bLaneDataReady || !GetPawn())
			{
				// InitializeLaneFollowing may have failed — bail.
				return;
			}
			if (!bAtDeadEnd)
			{
				// Successfully transitioned — recalculate on new lane data.
				return;
			}
		}
	}

	// --- Dead-end braking ---
	if (bAtDeadEnd)
	{
		VehicleMovement->SetThrottleInput(0.0f);
		VehicleMovement->SetSteeringInput(0.0f);
		VehicleMovement->SetBrakeInput(1.0f);
		return;
	}

	const FVector TargetPoint = GetLookAheadPoint(VehicleLocation, ClosestIndex);

	// --- Steering ---
	const FVector ToTarget = (TargetPoint - VehicleLocation).GetSafeNormal2D();
	const FVector Forward2D = VehicleForward.GetSafeNormal2D();
	const float CrossZ = FVector::CrossProduct(Forward2D, ToTarget).Z;

	// CrossZ is positive when target is to the right, negative when left.
	// Scale for reasonable responsiveness.
	const float SteeringInput = FMath::Clamp(CrossZ * 2.0f, -1.0f, 1.0f);

	// --- Throttle / Brake ---
	// Guard against TargetSpeed == 0 to prevent division by zero.
	if (TargetSpeed <= KINDA_SMALL_NUMBER)
	{
		VehicleMovement->SetThrottleInput(0.0f);
		VehicleMovement->SetSteeringInput(SteeringInput);
		VehicleMovement->SetBrakeInput(1.0f);
		return;
	}

	const float SpeedError = TargetSpeed - FMath::Abs(CurrentSpeed);
	float ThrottleInput = FMath::Clamp(SpeedError / TargetSpeed, 0.0f, 1.0f);
	float BrakeInput = 0.0f;

	// Brake if significantly over target speed
	if (CurrentSpeed > TargetSpeed * 1.1f)
	{
		ThrottleInput = 0.0f;
		BrakeInput = FMath::Clamp((CurrentSpeed - TargetSpeed) / TargetSpeed, 0.0f, 1.0f);
	}

	// Back off throttle in sharp turns
	if (FMath::Abs(SteeringInput) > 0.5f)
	{
		ThrottleInput *= 0.5f;
	}

	VehicleMovement->SetThrottleInput(ThrottleInput);
	VehicleMovement->SetSteeringInput(SteeringInput);
	VehicleMovement->SetBrakeInput(BrakeInput);
}

int32 ATrafficVehicleController::FindClosestPointIndex(const FVector& VehicleLocation) const
{
	int32 BestIndex = 0;
	float BestDistSq = MAX_flt;

	for (int32 i = 0; i < LanePoints.Num(); ++i)
	{
		const float DistSq = FVector::DistSquared(VehicleLocation, LanePoints[i]);
		if (DistSq < BestDistSq)
		{
			BestDistSq = DistSq;
			BestIndex = i;
		}
	}

	return BestIndex;
}

FVector ATrafficVehicleController::GetLookAheadPoint(
	const FVector& VehicleLocation, int32 ClosestIndex) const
{
	// Walk forward along lane points until LookAheadDistance is reached.
	float AccumulatedDist = 0.0f;
	int32 Index = ClosestIndex;

	while (Index < LanePoints.Num() - 1)
	{
		const float SegmentDist = FVector::Dist(LanePoints[Index], LanePoints[Index + 1]);
		AccumulatedDist += SegmentDist;

		if (AccumulatedDist >= LookAheadDistance)
		{
			const float Overshoot = AccumulatedDist - LookAheadDistance;
			const float Alpha = 1.0f - (Overshoot / FMath::Max(SegmentDist, KINDA_SMALL_NUMBER));
			return FMath::Lerp(LanePoints[Index], LanePoints[Index + 1], Alpha);
		}

		++Index;
	}

	// If the look-ahead extends beyond the lane, return the last point.
	return LanePoints.Last();
}

float ATrafficVehicleController::GetRemainingDistance(int32 FromIndex) const
{
	// Need at least two lane points to measure any distance.
	if (LanePoints.Num() < 2)
	{
		return 0.0f;
	}

	// Clamp to valid range so FromIndex + 1 is a valid index.
	if (FromIndex < 0)
	{
		FromIndex = 0;
	}
	if (FromIndex >= LanePoints.Num() - 1)
	{
		return 0.0f; // Already at or past the last point.
	}

	float Distance = 0.0f;

	// Use the vehicle's actual location for the first partial segment so
	// the measurement reflects how far the vehicle truly is from the next point.
	const APawn* ControlledPawn = GetPawn();
	const FVector StartPos = (ControlledPawn != nullptr)
		? ControlledPawn->GetActorLocation()
		: LanePoints[FromIndex];

	Distance += FVector::Dist(StartPos, LanePoints[FromIndex + 1]);

	// Sum remaining full segments.
	for (int32 i = FromIndex + 1; i < LanePoints.Num() - 1; ++i)
	{
		Distance += FVector::Dist(LanePoints[i], LanePoints[i + 1]);
	}
	return Distance;
}

void ATrafficVehicleController::CheckLaneTransition()
{
	if (!CachedProvider)
	{
		bAtDeadEnd = true;
		UE_LOG(LogAAATraffic, Log,
			TEXT("TrafficVehicleController: No provider cached — dead end on lane %d."),
			CurrentLane.HandleId);
		return;
	}

	TArray<FTrafficLaneHandle> Connected = CachedProvider->GetConnectedLanes(CurrentLane);
	if (Connected.IsEmpty())
	{
		bAtDeadEnd = true;
		UE_LOG(LogAAATraffic, Log,
			TEXT("TrafficVehicleController: No connected lanes from lane %d — dead end."),
			CurrentLane.HandleId);
		return;
	}

	// Deterministic lane selection using seeded RandomStream.
	const int32 LaneIndex = RandomStream.RandRange(0, Connected.Num() - 1);

	const FTrafficLaneHandle NextLane = Connected[LaneIndex];

	UE_LOG(LogAAATraffic, Log,
		TEXT("TrafficVehicleController: Transitioning from lane %d to lane %d (%d candidates)."),
		CurrentLane.HandleId, NextLane.HandleId, Connected.Num());

	InitializeLaneFollowing(NextLane);

	// If lane following could not be initialized for the selected connected lane,
	// treat this as a dead end so braking logic engages.
	if (!bLaneDataReady)
	{
		bAtDeadEnd = true;
		UE_LOG(LogAAATraffic, Warning,
			TEXT("TrafficVehicleController: Failed to initialize lane following for lane %d from lane %d — treating as dead end."),
			NextLane.HandleId, CurrentLane.HandleId);
	}
}
