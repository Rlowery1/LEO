// Copyright AAA_Traffic Contributors. All Rights Reserved.

#pragma once

#include "Modules/ModuleInterface.h"

class FAAA_TrafficEditorModule : public IModuleInterface
{
public:
	virtual void StartupModule() override;
	virtual void ShutdownModule() override;
};
