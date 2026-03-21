// Copyright Epic Games, Inc. All Rights Reserved.

using UnrealBuildTool;

public class AAA_Traffic : ModuleRules
{
	public AAA_Traffic(ReadOnlyTargetRules Target) : base(Target)
	{
		PCHUsage = PCHUsageMode.UseExplicitOrSharedPCHs;

		// Register subdirectory include paths so #include "Foo.h" resolves
		// regardless of which subdirectory the header lives in.
		string[] Subdirs = { "Core", "Controller", "Provider", "Subsystem", "Spawner" };
		foreach (string Sub in Subdirs)
		{
			PublicIncludePaths.Add(System.IO.Path.Combine(ModuleDirectory, "Public", Sub));
			PrivateIncludePaths.Add(System.IO.Path.Combine(ModuleDirectory, "Private", Sub));
		}

		PublicDependencyModuleNames.AddRange(
			new string[]
			{
				"Core",
				"CoreUObject",
				"Engine",
				"AIModule",
				"ChaosVehicles"
			}
		);

		PrivateDependencyModuleNames.AddRange(new string[] { "Chaos" });
	}
}
