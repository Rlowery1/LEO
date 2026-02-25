// Copyright AAA_Traffic Contributors. All Rights Reserved.

#include "AAA_TrafficEditorModule.h"
#include "Modules/ModuleManager.h"
#include "PropertyEditorModule.h"

#include "TrafficSpawner.h"
#include "TrafficSignalController.h"

#define LOCTEXT_NAMESPACE "AAA_TrafficEditor"

void FAAA_TrafficEditorModule::StartupModule()
{
	UE_LOG(LogTemp, Log, TEXT("AAA_TrafficEditor module started."));
}

void FAAA_TrafficEditorModule::ShutdownModule()
{
	UE_LOG(LogTemp, Log, TEXT("AAA_TrafficEditor module shut down."));
}

#undef LOCTEXT_NAMESPACE

IMPLEMENT_MODULE(FAAA_TrafficEditorModule, AAA_TrafficEditor);
