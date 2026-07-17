// Copyright Epic Games, Inc. All Rights Reserved.

using UnrealBuildTool;

public class UnrealJoltEditor : ModuleRules
{
	public UnrealJoltEditor(ReadOnlyTargetRules Target) : base(Target)
	{
		PCHUsage = ModuleRules.PCHUsageMode.UseExplicitOrSharedPCHs;

		PublicDependencyModuleNames.AddRange(
			new string[]
			{
				"Core",
				"CoreUObject",
				"Engine",
				"UnrealJolt",
			}
		);

		PrivateDependencyModuleNames.AddRange(
			new string[]
			{
				"Slate",
				"SlateCore",
				"InputCore",
				"EditorStyle",
				"UnrealEd",
				"PropertyEditor",
				"DetailCustomizations",
				"EditorSubsystem",
				"ToolMenus", 
				"BlueprintGraph",
				"Kismet"
			}
		);
	}
}
