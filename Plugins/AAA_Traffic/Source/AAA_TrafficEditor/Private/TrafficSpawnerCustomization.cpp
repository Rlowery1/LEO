// Copyright AAA_Traffic Contributors. All Rights Reserved.

#include "TrafficSpawnerCustomization.h"
#include "TrafficSpawner.h"

#include "DetailLayoutBuilder.h"
#include "DetailCategoryBuilder.h"
#include "DetailWidgetRow.h"
#include "Widgets/Input/SButton.h"
#include "Widgets/Text/STextBlock.h"

#define LOCTEXT_NAMESPACE "TrafficSpawnerCustomization"

TSharedRef<IDetailCustomization> FTrafficSpawnerCustomization::MakeInstance()
{
	return MakeShareable(new FTrafficSpawnerCustomization);
}

void FTrafficSpawnerCustomization::CustomizeDetails(IDetailLayoutBuilder& DetailBuilder)
{
	// Collect the selected spawner objects.
	TArray<TWeakObjectPtr<UObject>> SelectedObjects;
	DetailBuilder.GetObjectsBeingCustomized(SelectedObjects);

	IDetailCategoryBuilder& QuickSetup = DetailBuilder.EditCategory(
		TEXT("Quick Setup"),
		LOCTEXT("QuickSetupCategory", "Quick Setup"),
		ECategoryPriority::Important);

	QuickSetup.AddCustomRow(LOCTEXT("SetupRow", "Setup"))
		.NameContent()
		[
			SNew(STextBlock)
				.Text(LOCTEXT("QuickSetupLabel", "One-Click Setup"))
				.Font(IDetailLayoutBuilder::GetDetailFont())
		]
		.ValueContent()
		.MaxDesiredWidth(200.0f)
		[
			SNew(SButton)
				.Text(LOCTEXT("SetupButton", "Setup Traffic Defaults"))
				.ToolTipText(LOCTEXT("SetupTooltip",
					"Configure this spawner with sensible defaults:\n"
					"10 vehicles, 1500 cm/s, seed 42, respawn on, auto-signals on."))
				.OnClicked_Lambda([SelectedObjects]() -> FReply
				{
					for (const TWeakObjectPtr<UObject>& Obj : SelectedObjects)
					{
						ATrafficSpawner* Spawner = Cast<ATrafficSpawner>(Obj.Get());
						if (!Spawner) { continue; }

						// Only set values that haven't been explicitly configured.
						Spawner->Modify();
						Spawner->VehicleCount     = 10;
						Spawner->VehicleSpeed     = 1500.0f;
						Spawner->SpawnSeed        = 42;
						Spawner->SpawnSpacing     = 1500.0f;
						Spawner->SpeedVariation   = 15.0f;
						Spawner->bEnableRespawn   = true;
						Spawner->bAutoPlaceSignals = true;
						Spawner->SpawnZOffset     = 50.0f;
						Spawner->LaneChangeAggression = 0.5f;
						Spawner->DefaultSpeedLimit = 0.0f;
						Spawner->RespawnCheckInterval = 3.0f;
						Spawner->MinRespawnDistance = 10000.0f;
					}
					return FReply::Handled();
				})
		];
}

#undef LOCTEXT_NAMESPACE
