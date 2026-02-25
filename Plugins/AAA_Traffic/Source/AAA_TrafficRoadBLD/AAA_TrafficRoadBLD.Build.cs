// Copyright AAA_Traffic Contributors. All Rights Reserved.

using UnrealBuildTool;
using System.IO;

public class AAA_TrafficRoadBLD : ModuleRules
{
	public AAA_TrafficRoadBLD(ReadOnlyTargetRules Target) : base(Target)
	{
		PCHUsage = PCHUsageMode.UseExplicitOrSharedPCHs;

		PublicDependencyModuleNames.AddRange(
			new string[]
			{
				"Core",
				"CoreUObject",
				"Engine",
				"AAA_Traffic"
			}
		);

		// RoadBLD is an optional sibling plugin.
		// When present WITH compilable headers, this module compiles the full adapter.
		// When absent (CI), or present but precompiled-only (no source headers), it compiles as an empty stub.
		string RoadBLDDir = Path.GetFullPath(Path.Combine(PluginDirectory, "..", "RoadBLD"));
		string RoadBLDPublicDir = Path.Combine(RoadBLDDir, "Source", "RoadBLDRuntime", "Public");
		bool bHasRoadBLD = Directory.Exists(RoadBLDDir) && Directory.Exists(RoadBLDPublicDir);

		if (bHasRoadBLD)
		{
			PrivateDependencyModuleNames.Add("RoadBLDRuntime");
			PublicDefinitions.Add("WITH_ROADBLD=1");
		}
		else
		{
			PublicDefinitions.Add("WITH_ROADBLD=0");
		}
	}
}
