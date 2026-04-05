// Copyright AAA_Traffic Contributors. All Rights Reserved.

#pragma once

#include "Logging/LogMacros.h"
#include "Stats/Stats.h"

/** Shared log category for the AAA_Traffic plugin. */
DECLARE_LOG_CATEGORY_EXTERN(LogAAATraffic, Log, All);

/** Stat group for Unreal Insights profiling (stat AAA_Traffic). */
DECLARE_STATS_GROUP(TEXT("AAA_Traffic"), STATGROUP_AAATraffic, STATCAT_Advanced);
DECLARE_CYCLE_STAT(TEXT("DespawnSweep"), STAT_AAATraffic_DespawnSweep, STATGROUP_AAATraffic);
DECLARE_CYCLE_STAT(TEXT("UpdateLODTiers"), STAT_AAATraffic_UpdateLODTiers, STATGROUP_AAATraffic);
DECLARE_CYCLE_STAT(TEXT("TryOccupyJunction"), STAT_AAATraffic_TryOccupyJunction, STATGROUP_AAATraffic);
DECLARE_CYCLE_STAT(TEXT("UpdateVehicleInput"), STAT_AAATraffic_UpdateVehicleInput, STATGROUP_AAATraffic);
