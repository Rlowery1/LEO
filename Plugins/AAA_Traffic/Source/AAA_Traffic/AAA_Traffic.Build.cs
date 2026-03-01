// Copyright Epic Games, Inc. All Rights Reserved.

using UnrealBuildTool;

public class AAA_Traffic : ModuleRules
{
	public AAA_Traffic(ReadOnlyTargetRules Target) : base(Target)
	{
		PCHUsage = PCHUsageMode.UseExplicitOrSharedPCHs;

		PublicDependencyModuleNames.AddRange(
			new string[]
			{
				"Core",
				"CoreUObject",
				"Engine",
				"InputCore",
				"EnhancedInput",
				"AIModule",
				"ChaosVehicles",
				"Chaos"
			}
		);

		PrivateDependencyModuleNames.AddRange(new string[] { });
	}
}
