// Copyright AAA_Traffic Contributors. All Rights Reserved.

#pragma once

#include "IDetailCustomization.h"

/**
 * Details panel customization for ATrafficSpawner.
 *
 * Adds a "Quick Setup" category with a convenience button that configures
 * sensible defaults for one-click traffic spawning:
 * - VehicleCount      = 10
 * - VehicleSpeed      = 1500 cm/s
 * - SpawnSeed         = 42
 * - SpawnSpacing      = 1500 cm
 * - SpeedVariation    = 15%
 * - bEnableRespawn    = true
 * - bAutoPlaceSignals = true
 *
 * The button only appears in the Editor — no runtime cost.
 */
class FTrafficSpawnerCustomization : public IDetailCustomization
{
public:
	static TSharedRef<IDetailCustomization> MakeInstance();

	virtual void CustomizeDetails(IDetailLayoutBuilder& DetailBuilder) override;
};
