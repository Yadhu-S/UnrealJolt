// Copyright Epic Games, Inc. All Rights Reserved.

#include "JoltPhysicsPlugin.h"
#include "JoltSettings.h"
#include "Modules/ModuleManager.h"

#if WITH_EDITOR
#include "ISettingsModule.h"
#endif

#define LOCTEXT_NAMESPACE "FUnrealJoltModule "

void FUnrealJoltModule ::StartupModule()
{
	#if WITH_EDITOR
	if (ISettingsModule* SettingModule = FModuleManager::GetModulePtr<ISettingsModule>("Settings"))
	{
		SettingModule->RegisterSettings("Project", "Plugins", "Jolt",
			LOCTEXT("RuntimeSettingsName", "Jolt"),
			LOCTEXT("RuntimeSettingsDescription", "Jolt physics plugin configuration"),
			GetMutableDefault<UJoltSettings>());
	}
	#endif
}

void FUnrealJoltModule ::ShutdownModule()
{
	#if WITH_EDITOR
	if (ISettingsModule* SettingsModule = FModuleManager::GetModulePtr<ISettingsModule>("Settings"))
	{
		SettingsModule->UnregisterSettings("Project", "Plugins", "MySetting");
	}
	#endif
}

#undef LOCTEXT_NAMESPACE

IMPLEMENT_MODULE(FUnrealJoltModule, UnrealJolt)
