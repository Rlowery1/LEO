// TrafficVehicleController_OnPossess.cpp — Vehicle initialization and physics setup.
// Split from TrafficVehicleController.cpp for maintainability.

#include "TrafficVehicleController.h"
#include "TrafficSubsystem.h"
#include "TrafficLog.h"
#include "ChaosWheeledVehicleMovementComponent.h"
#include "ChaosVehicleWheel.h"
#include "GameFramework/Pawn.h"
#include "Engine/World.h"
#include "PhysicsEngine/BodyInstance.h"
#include "Components/PrimitiveComponent.h"
#include "PhysicsProxy/SingleParticlePhysicsProxy.h"
#include "UObject/UnrealType.h"
void ATrafficVehicleController::OnPossess(APawn* InPawn)
{
	Super::OnPossess(InPawn);
	RandomStream.Initialize(RandomSeed);

	// --- Diagnostic: log pawn class and movement component details ---
	if (InPawn)
	{
		const FString PawnClass = InPawn->GetClass()->GetName();
		UPawnMovementComponent* GenericMC = InPawn->GetMovementComponent();
		const FString MCClass = GenericMC ? GenericMC->GetClass()->GetName() : TEXT("NULL");

		UChaosWheeledVehicleMovementComponent* ChaosMC =
			Cast<UChaosWheeledVehicleMovementComponent>(GenericMC);

		UE_LOG(LogAAATraffic, Log,
			TEXT("VehicleController::OnPossess: Pawn='%s' Class='%s' MovementComponent='%s' ChaosWheeledCast=%s"),
			*InPawn->GetName(), *PawnClass, *MCClass,
			ChaosMC ? TEXT("OK") : TEXT("FAILED â€” vehicles will NOT move"));

		if (!ChaosMC)
		{
			UE_LOG(LogAAATraffic, Error,
				TEXT("VehicleController::OnPossess: CRITICAL â€” Pawn '%s' (class '%s') does not have a UChaosWheeledVehicleMovementComponent. "
					 "GetMovementComponent() returned '%s'. The vehicle controller requires ChaosWheeledVehicleMovementComponent to set throttle/steering/brake. "
					 "Ensure the vehicle Blueprint inherits from AWheeledVehiclePawn or has this component added."),
				*InPawn->GetName(), *PawnClass, *MCClass);

			// List all components on the pawn for further diagnosis.
			TArray<UActorComponent*> AllComps;
			InPawn->GetComponents(AllComps);
			for (const UActorComponent* Comp : AllComps)
			{
				UE_LOG(LogAAATraffic, Log,
					TEXT("  Component: '%s' Class='%s'"),
					*Comp->GetName(), *Comp->GetClass()->GetName());
			}
		}
		else
		{
			// --- Ensure Chaos vehicle is in a drivable state ---
			// Marketplace Blueprints often default to parked/sleeping/handbrake-on.
			// Explicitly wake and unpark the vehicle so throttle input takes effect.
			ChaosMC->SetParked(false);
			ChaosMC->SetSleeping(false);
			ChaosMC->SetHandbrakeInput(false);
			ChaosMC->SetTargetGear(1, /*bImmediate=*/ true);
			ChaosMC->SetUseAutomaticGears(true);

			// --- Prevent ProcessSleeping from re-sleeping the physics body ---
			// PROVEN by diagnostic: BodyAwake=NO at T=2s despite per-tick
			// SetSleeping(false). The engine's ProcessSleeping uses a speed-
			// based sleep threshold (default 10 cm/s) and an interpolated
			// ThrottleInput (gated by bRequiresControllerForInputs). Two
			// belt-and-suspenders fixes:
			//
			// 1) SetRequiresControllerForInputs(false): ensures ThrottleInput
			//    interpolation always runs in UpdateState regardless of
			//    controller type, so ProcessSleeping sees throttle > wake
			//    tolerance and keeps the body awake.
			//
			// 2) SleepThreshold = 0: the speed-based sleep path requires
			//    speedÂ² < thresholdÂ². With threshold=0 this is never true,
			//    so ProcessSleeping's sleep counter never increments even if
			//    the vehicle is momentarily stationary.
			ChaosMC->SetRequiresControllerForInputs(false);
			ChaosMC->SleepThreshold = 0.0f;

			// --- Prevent Chaos solver from putting the physics body to sleep ---
			// PROVEN through 3 test runs: GT-level sleep prevention (SleepThreshold,
			// RequiresControllerForInputs, per-tick SetSleeping) all failed because
			// the Chaos physics solver independently sleeps rigid bodies based on
			// kinetic energy via the Island Manager. The Island Manager only skips
			// sleep for particles where SleepType == NeverSleep.
			if (UPrimitiveComponent* Prim = ChaosMC->UpdatedPrimitive)
			{
				if (FBodyInstance* BI = Prim->GetBodyInstance())
				{
					FPhysicsActorHandle PhysHandle = BI->GetPhysicsActorHandle();
					if (PhysHandle)
					{
						PhysHandle->GetGameThreadAPI().SetSleepType(Chaos::ESleepType::NeverSleep);
						UE_LOG(LogAAATraffic, Log,
							TEXT("VehicleController::OnPossess: Set Chaos ESleepType::NeverSleep on '%s'"),
							*InPawn->GetName());
					}
				}
			}

			// --- Neutralize the handbrake/parking brake torque per wheel ---
			// The Chaos physics sim applies HandbrakeTorque on any rear wheel
			// with HandbrakeEnabled whenever EITHER the handbrake input is
			// nonzero OR ParkingEnabled is true (SetParked). Marketplace
			// Blueprints can re-assert parking/handbrake state via their own
			// Tick or EventGraph, overriding our SetParked(false) call above.
			//
			// Rather than fighting per-frame to clear that state, we zero out
			// the HandbrakeTorque magnitude on every wheel. This is a one-time
			// config change: 0 torque * any input = 0 force. Our AI never
			// uses the handbrake; we use SetBrakeInput() for all deceleration.
			const int32 NumWheels = ChaosMC->WheelSetups.Num();
			for (int32 WIdx = 0; WIdx < NumWheels; ++WIdx)
			{
				ChaosMC->SetWheelHandbrakeTorque(WIdx, 0.0f);
			}

			UE_LOG(LogAAATraffic, Log,
				TEXT("VehicleController::OnPossess: Vehicle '%s' initialized â€” "
					 "Parked=false, Sleeping=false, Handbrake=false, Gear=1, AutoGears=true, "
					 "HandbrakeTorque zeroed on %d wheels, "
					 "RequiresControllerForInputs=false, SleepThreshold=0"),
				*InPawn->GetName(), NumWheels);

			// I3 FIX: Compute vehicle-specific max braking deceleration from
			// the Chaos wheel setup so emergency braking maps to reality.
			// Formula: TotalBrakeTorque / (avgWheelRadius * vehicleMass) â†’ cm/sÂ².
			{
				float TotalBrakeTorque = 0.0f;  // Nm across all wheels
				float RadiusSum = 0.0f;         // accumulate wheel radii (cm)
				int32 RadiusCount = 0;
				for (int32 WI = 0; WI < NumWheels; ++WI)
				{
					if (WI < ChaosMC->WheelSetups.Num())
					{
						const auto& WS = ChaosMC->WheelSetups[WI];
						if (WS.WheelClass)
						{
							const UChaosVehicleWheel* WheelCDO = WS.WheelClass->GetDefaultObject<UChaosVehicleWheel>();
							if (WheelCDO)
							{
								TotalBrakeTorque += WheelCDO->MaxBrakeTorque;
								RadiusSum += WheelCDO->WheelRadius;
								++RadiusCount;
							}
						}
					}
				}
				// Average wheel radius: use measured data, else 30 cm fallback.
				const float AvgWheelRadius = (RadiusCount > 0)
					? (RadiusSum / static_cast<float>(RadiusCount))
					: 30.0f;
				// Vehicle mass from Chaos physics body.
				float VehicleMassKg = 1500.0f; // fallback
				if (UPrimitiveComponent* Prim = ChaosMC->UpdatedPrimitive)
				{
					if (FBodyInstance* BI = Prim->GetBodyInstance())
					{
						VehicleMassKg = FMath::Max(BI->GetBodyMass(), 100.0f);
					}
				}
				// Torque(Nm) / Radius(m) = Force(N), Force/Mass = accel(m/sÂ²) â†’ *100 for cm/sÂ²
				const float RadiusM = FMath::Max(AvgWheelRadius * 0.01f, 0.1f);
				MaxBrakeDecelCmPerSec2 = FMath::Clamp(
					(TotalBrakeTorque / RadiusM / VehicleMassKg) * 100.0f,
					300.0f, 2000.0f); // clamp to sane range
				UE_LOG(LogAAATraffic, Log,
					TEXT("VehicleController::OnPossess: Vehicle '%s' MaxBrakeDecel=%.0f cm/sÂ² "
						 "(TotalBrakeTorque=%.0f Nm, AvgWheelRadius=%.1f cm, Mass=%.0f kg)"),
					*InPawn->GetName(), MaxBrakeDecelCmPerSec2,
					TotalBrakeTorque, AvgWheelRadius, VehicleMassKg);
			}

			// --- Compute wheelbase and max steer angle from Chaos wheel setup ---
			// Walk the wheel CDOs: group by AxleType (Front vs Rear), get bone
			// positions from the skeletal mesh to compute axle separation.
			// Also extract MaxSteerAngle from the first steered (Front) wheel.
			{
				float FrontAxleX = 0.0f, RearAxleX = 0.0f;
				int32 FrontCount = 0, RearCount = 0;
				float MaxSteerDeg = 0.0f;

				// Get the skeletal mesh for bone position lookups.
				USkeletalMeshComponent* MeshComp = nullptr;
				if (InPawn)
				{
					MeshComp = InPawn->FindComponentByClass<USkeletalMeshComponent>();
				}

				for (int32 WI = 0; WI < ChaosMC->WheelSetups.Num(); ++WI)
				{
					const auto& WS = ChaosMC->WheelSetups[WI];
					if (!WS.WheelClass) continue;
					const UChaosVehicleWheel* WheelCDO = WS.WheelClass->GetDefaultObject<UChaosVehicleWheel>();
					if (!WheelCDO) continue;

					// Get wheel position from the bone name on the skeletal mesh.
					float WheelLocalX = WS.AdditionalOffset.X; // fallback
					if (MeshComp && !WS.BoneName.IsNone())
					{
						const int32 BoneIdx = MeshComp->GetBoneIndex(WS.BoneName);
						if (BoneIdx != INDEX_NONE)
						{
							// GetBoneTransform returns world space; convert to local.
							const FTransform BoneWorld = MeshComp->GetBoneTransform(BoneIdx);
							const FTransform ActorInv = InPawn->GetActorTransform().Inverse();
							const FVector LocalPos = ActorInv.TransformPosition(BoneWorld.GetLocation());
							WheelLocalX = LocalPos.X + WS.AdditionalOffset.X;
						}
					}

					if (WheelCDO->AxleType == EAxleType::Front)
					{
						FrontAxleX += WheelLocalX;
						++FrontCount;
						MaxSteerDeg = FMath::Max(MaxSteerDeg, WheelCDO->MaxSteerAngle);
					}
					else if (WheelCDO->AxleType == EAxleType::Rear)
					{
						RearAxleX += WheelLocalX;
						++RearCount;
					}
				}
				if (FrontCount > 0 && RearCount > 0)
				{
					FrontAxleX /= static_cast<float>(FrontCount);
					RearAxleX /= static_cast<float>(RearCount);
					VehicleWheelbaseCm = FMath::Clamp(FMath::Abs(FrontAxleX - RearAxleX), 150.0f, 600.0f);
				}
				if (MaxSteerDeg > 1.0f)
				{
					VehicleMaxSteerAngleRad = FMath::DegreesToRadians(FMath::Clamp(MaxSteerDeg, 15.0f, 55.0f));
				}
				UE_LOG(LogAAATraffic, Log,
					TEXT("VehicleController::OnPossess: Vehicle '%s' Wheelbase=%.1f cm, MaxSteerAngle=%.1fÂ°"),
					*InPawn->GetName(), VehicleWheelbaseCm,
					FMath::RadiansToDegrees(VehicleMaxSteerAngleRad));
			}
		}

		// --- Compute front-bumper extent from actor bounds ---
		// Distance from actor origin to the front-most point of the
		// bounding box along the forward axis.  Uses the AABB support
		// function: for each world axis pick Max or Min depending on
		// the sign of the forward vector, giving the corner that
		// projects furthest forward.  This is orientation-correct
		// regardless of which world axis the car faces at spawn.
		{
			const FBox ActorBounds = InPawn->GetComponentsBoundingBox(/*bNonColliding=*/ false);
			if (ActorBounds.IsValid)
			{
				const FVector ActorOrigin = InPawn->GetActorLocation();
				const FVector Fwd = InPawn->GetActorForwardVector();
				const FVector Right = InPawn->GetActorRightVector();

				// Support point: the AABB corner furthest along Fwd.
				FVector FrontCorner;
				FrontCorner.X = (Fwd.X >= 0.0f) ? ActorBounds.Max.X : ActorBounds.Min.X;
				FrontCorner.Y = (Fwd.Y >= 0.0f) ? ActorBounds.Max.Y : ActorBounds.Min.Y;
				FrontCorner.Z = (Fwd.Z >= 0.0f) ? ActorBounds.Max.Z : ActorBounds.Min.Z;

				VehicleFrontExtent = FVector::DotProduct(FrontCorner - ActorOrigin, Fwd);
				// Clamp to reasonable range to handle edge cases.
				VehicleFrontExtent = FMath::Clamp(VehicleFrontExtent, 50.0f, 1000.0f);

				// Rear extent: AABB corner furthest along -Fwd.
				FVector RearCorner;
				RearCorner.X = (Fwd.X < 0.0f) ? ActorBounds.Max.X : ActorBounds.Min.X;
				RearCorner.Y = (Fwd.Y < 0.0f) ? ActorBounds.Max.Y : ActorBounds.Min.Y;
				RearCorner.Z = (Fwd.Z < 0.0f) ? ActorBounds.Max.Z : ActorBounds.Min.Z;
				VehicleRearExtent = FMath::Abs(FVector::DotProduct(RearCorner - ActorOrigin, Fwd));
				VehicleRearExtent = FMath::Clamp(VehicleRearExtent, 50.0f, 1000.0f);

				// Lateral half-width: AABB corner furthest along Right.
				FVector RightCorner;
				RightCorner.X = (Right.X >= 0.0f) ? ActorBounds.Max.X : ActorBounds.Min.X;
				RightCorner.Y = (Right.Y >= 0.0f) ? ActorBounds.Max.Y : ActorBounds.Min.Y;
				RightCorner.Z = (Right.Z >= 0.0f) ? ActorBounds.Max.Z : ActorBounds.Min.Z;
				VehicleLateralHalfWidth = FVector::DotProduct(RightCorner - ActorOrigin, Right);
				VehicleLateralHalfWidth = FMath::Clamp(VehicleLateralHalfWidth, 50.0f, 300.0f);
			}
			else
			{
				// Fallback: typical sedan half-length.
				VehicleFrontExtent = 250.0f;
				VehicleRearExtent = 250.0f;
				VehicleLateralHalfWidth = 100.0f;
			}
			UE_LOG(LogAAATraffic, Log,
				TEXT("VehicleController::OnPossess: Vehicle '%s' front=%.1f rear=%.1f halfWidth=%.1f cm"),
				*InPawn->GetName(), VehicleFrontExtent, VehicleRearExtent, VehicleLateralHalfWidth);
		}

		// --- Neutralize marketplace BP physics-optimization parking ---
		// PROVEN ROOT CAUSE: The parent vehicle Blueprint (DD_Vehicles_Advanced)
		// runs "Physics optimization" every tick via EventTick â†’ Sequence Then 0.
		// Entry gate: (Base Vehicle Initialized AND Optimized AND GameTime > BeginPlayMargin)
		// Once this gate opens (~2s), the Optimization Unpossessed block detects
		// that the vehicle has no PlayerController (only our AIController), treats
		// it as "unpossessed", and calls CE_StopCar â†’ CE Set Parked (Parked=true),
		// killing the drivetrain every frame.
		//
		// Fix: use UE reflection to set the BP's "Optimized" variable to false.
		// This disables the entry gate, preventing the entire parking chain from
		// ever firing. All other parent EventTick behavior (engine sound, wheel
		// effects, smoke VFX, crash impacts, lights) continues unaffected because
		// those are on Sequence Then 1-4, which do not check 'Optimized'.
		//
		// Also set "Can Sleep" to false as a secondary safety net â€” this is the
		// inner gate checked by CE_CheckVehicleSleep before evaluating wake/sleep.
		{
			UClass* VehicleBPClass = InPawn->GetClass();
			int32 PropertiesNeutralized = 0;

			// Disable the 'Optimized' gate (prevents Physics optimization from running).
			if (FBoolProperty* OptimizedProp = FindFProperty<FBoolProperty>(VehicleBPClass, TEXT("Optimized")))
			{
				OptimizedProp->SetPropertyValue_InContainer(InPawn, false);
				++PropertiesNeutralized;
			}

			// Disable the 'Can Sleep' inner gate (prevents CE_CheckVehicleSleep from
			// evaluating wake/sleep even if the Optimized gate is bypassed).
			// BP variable names with spaces use the space-free internal FName â€” search
			// by both common internal names and DisplayName metadata for robustness.
			FBoolProperty* CanSleepProp = FindFProperty<FBoolProperty>(VehicleBPClass, TEXT("CanSleep"));
			if (!CanSleepProp)
			{
				CanSleepProp = FindFProperty<FBoolProperty>(VehicleBPClass, TEXT("bCanSleep"));
			}
			if (!CanSleepProp)
			{
				CanSleepProp = FindFProperty<FBoolProperty>(VehicleBPClass, TEXT("Can Sleep"));
			}
#if WITH_EDITORONLY_DATA
			if (!CanSleepProp)
			{
				// Last resort: iterate all bool properties and match by DisplayName metadata.
				// HasMetaData/GetMetaData are editor-only APIs (metadata stripped in Game builds).
				for (TFieldIterator<FBoolProperty> It(VehicleBPClass); It; ++It)
				{
					if (It->HasMetaData(TEXT("DisplayName")))
					{
						const FString& DisplayName = It->GetMetaData(TEXT("DisplayName"));
						if (DisplayName.Equals(TEXT("Can Sleep"), ESearchCase::IgnoreCase))
						{
							CanSleepProp = *It;
							break;
						}
					}
				}
			}
#endif // WITH_EDITORONLY_DATA
			if (CanSleepProp)
			{
				CanSleepProp->SetPropertyValue_InContainer(InPawn, false);
				++PropertiesNeutralized;
			}

			if (PropertiesNeutralized > 0)
			{
				UE_LOG(LogAAATraffic, Log,
					TEXT("VehicleController::OnPossess: Neutralized %d BP optimization properties on '%s' "
						 "(Optimized=false, CanSleep=false) â€” parent BP parking chain disabled."),
					PropertiesNeutralized, *InPawn->GetName());
			}
			else
			{
				// Property names may differ across marketplace pack versions. Log all
				// boolean properties so the developer can identify the correct FName.
				UE_LOG(LogAAATraffic, Warning,
					TEXT("VehicleController::OnPossess: Could not find 'Optimized' or 'Can Sleep' "
						 "boolean properties on '%s' (class '%s'). The parent BP may still park this vehicle. "
						 "Listing all boolean properties for diagnosis:"),
					*InPawn->GetName(), *VehicleBPClass->GetName());

				for (TFieldIterator<FBoolProperty> It(VehicleBPClass); It; ++It)
				{
					UE_LOG(LogAAATraffic, Warning, TEXT("  Bool property: '%s'"), *It->GetName());
				}
			}
		}

		// --- Bind collision response ---
		// When the pawn physically hits another actor, start a brief
		// full-braking period so the vehicle doesn't just keep pushing.
		InPawn->OnActorHit.AddDynamic(this, &ATrafficVehicleController::HandleActorHit);
	}
	else
	{
		UE_LOG(LogAAATraffic, Error, TEXT("VehicleController::OnPossess: InPawn is NULL."));
	}

	if (UWorld* World = GetWorld())
	{
		if (UTrafficSubsystem* TrafficSub = World->GetSubsystem<UTrafficSubsystem>())
		{
			TrafficSub->RegisterVehicle(this);
		}
	}
}
