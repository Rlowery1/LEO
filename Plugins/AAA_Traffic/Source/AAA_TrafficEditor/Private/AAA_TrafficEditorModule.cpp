// Copyright AAA_Traffic Contributors. All Rights Reserved.

#include "AAA_TrafficEditorModule.h"
#include "Modules/ModuleManager.h"
#include "PropertyEditorModule.h"
#include "TrafficSpawnerCustomization.h"

DEFINE_LOG_CATEGORY_STATIC(LogAAATrafficEditor, Log, All);

#define LOCTEXT_NAMESPACE "AAA_TrafficEditor"

void FAAA_TrafficEditorModule::StartupModule()
{
	UE_LOG(LogAAATrafficEditor, Log, TEXT("AAA_TrafficEditor module started."));

	// Register the Details customization for ATrafficSpawner.
	FPropertyEditorModule& PropertyModule = FModuleManager::LoadModuleChecked<FPropertyEditorModule>("PropertyEditor");
	PropertyModule.RegisterCustomClassLayout(
		TEXT("TrafficSpawner"),
		FOnGetDetailCustomizationInstance::CreateStatic(&FTrafficSpawnerCustomization::MakeInstance));
}

void FAAA_TrafficEditorModule::ShutdownModule()
{
	UE_LOG(LogAAATrafficEditor, Log, TEXT("AAA_TrafficEditor module shut down."));

	if (FModuleManager::Get().IsModuleLoaded("PropertyEditor"))
	{
		FPropertyEditorModule& PropertyModule = FModuleManager::GetModuleChecked<FPropertyEditorModule>("PropertyEditor");
		PropertyModule.UnregisterCustomClassLayout(TEXT("TrafficSpawner"));
	}
}

#undef LOCTEXT_NAMESPACE

IMPLEMENT_MODULE(FAAA_TrafficEditorModule, AAA_TrafficEditor);
