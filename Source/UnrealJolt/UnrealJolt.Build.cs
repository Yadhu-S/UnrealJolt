// Copyright Epic Games, Inc. All Rights Reserved.

using UnrealBuildTool;

public class UnrealJolt : ModuleRules
{
	public UnrealJolt(ReadOnlyTargetRules Target) : base(Target)
	{
		PCHUsage = ModuleRules.PCHUsageMode.UseExplicitOrSharedPCHs;

		/* Uncomment below if you have modified Landscapesplinesegment.h and recompiled the engine
		 * Add LANDSCAPE_API to avoid link errors
		 * more about issue here https://forums.unrealengine.com/t/linker-error-for-ulandscapesplinesegment-getlocalmeshcomponents-while-other-functions-from-exact-there-work/1609583
		 */
		//PublicDefinitions.Add("JOLT_PLUGIN_LANDSCAPE_API_MODIFIED");

		PublicIncludePaths.AddRange(
				new string[] {
				// ... add public include paths required here ...
				}
				);


		PrivateIncludePaths.AddRange(
				new string[] {
				// ... add other private include paths required here ...
				}
				);


		PublicDependencyModuleNames.AddRange(
				new string[]
				{
				"Core",
				"CoreUObject",
				"Engine",
				"InputCore",
				"UnrealJoltLibrary",
				"Projects",
				"Landscape",
				"PhysicsCore"
				// ... add other public dependencies that you statically link with here ...
				}
				);
		if (Target.bBuildEditor) // Only add in editor builds
		{
			PublicDependencyModuleNames.AddRange(new string[] { "UnrealEd", "EditorSubsystem" });
		}


		PrivateDependencyModuleNames.AddRange(
				new string[]
				{
				// ... add private dependencies that you statically link with here ...	
				}
				);


		DynamicallyLoadedModuleNames.AddRange(
				new string[]
				{
				// ... add any modules that your module loads dynamically here ...
				}
				);


	}
}
