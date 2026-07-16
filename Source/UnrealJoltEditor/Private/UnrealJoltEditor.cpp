#include "UnrealJoltEditor.h"

#include "JoltMobilityCustomization.h"
#include "JoltPhysicsComponent.h"
#include "JoltSettings.h"
#include "JoltSettingsDetails.h"
#include "Modules/ModuleManager.h"
#include "PropertyEditorModule.h"

#define LOCTEXT_NAMESPACE "FUnrealJoltEditorModule"

void FUnrealJoltEditorModule::StartupModule()
{
	FPropertyEditorModule& PropertyModule = FModuleManager::LoadModuleChecked<FPropertyEditorModule>("PropertyEditor");

	PropertyModule.RegisterCustomClassLayout(
	   UJoltSettings::StaticClass()->GetFName(),
	   FOnGetDetailCustomizationInstance::CreateStatic(&FJoltSettingsDetails::MakeInstance));

	PropertyModule.RegisterCustomClassLayout(
	   UJoltPhysicsComponent::StaticClass()->GetFName(),
	   FOnGetDetailCustomizationInstance::CreateStatic(&FJoltPhysicsComponentDetails::MakeInstance));

	PropertyModule.NotifyCustomizationModuleChanged();
}

void FUnrealJoltEditorModule::ShutdownModule()
{
	if (FModuleManager::Get().IsModuleLoaded("PropertyEditor"))
	{
		FPropertyEditorModule& PropertyModule = FModuleManager::GetModuleChecked<FPropertyEditorModule>("PropertyEditor");
		PropertyModule.UnregisterCustomClassLayout(UJoltSettings::StaticClass()->GetFName());
		PropertyModule.UnregisterCustomClassLayout(UJoltPhysicsComponent::StaticClass()->GetFName());
		PropertyModule.NotifyCustomizationModuleChanged();
	}
}

#undef LOCTEXT_NAMESPACE

IMPLEMENT_MODULE(FUnrealJoltEditorModule, UnrealJoltEditor)
