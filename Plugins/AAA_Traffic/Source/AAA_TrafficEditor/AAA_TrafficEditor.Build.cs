// Copyright AAA_Traffic Contributors. All Rights Reserved.

using UnrealBuildTool;

public class AAA_TrafficEditor : ModuleRules
{
	public AAA_TrafficEditor(ReadOnlyTargetRules Target) : base(Target)
	{
		PCHUsage = PCHUsageMode.UseExplicitOrSharedPCHs;

		PublicDependencyModuleNames.AddRange(
			new string[]
			{
				"Core",
				"CoreUObject",
				"Engine"
			}
		);

		PrivateDependencyModuleNames.AddRange(
			new string[]
			{
				"AAA_Traffic",
				"UnrealEd",
				"Slate",
				"SlateCore",
				"PropertyEditor",
				"EditorStyle"
			}
		);
	}
}
