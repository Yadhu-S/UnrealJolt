#include "UnrealJoltEditor.h"
#include "JoltMotionTypeCustomization.h"
#include "JoltPhysicsDetailsCustomization.h"
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

	PropertyModule.RegisterCustomPropertyTypeLayout(
		TEXT("EJoltMotionType"),
		FOnGetPropertyTypeCustomizationInstance::CreateStatic(&FJoltMotionTypeCustomization::MakeInstance));

	PropertyModule.RegisterCustomClassLayout(
		AActor::StaticClass()->GetFName(),
		FOnGetDetailCustomizationInstance::CreateStatic(&FJoltPhysicsDetailsCustomization::MakeInstance));
	
	TSharedRef<FPropertySection> PhysicsSection = PropertyModule.FindOrCreateSection(
		AActor::StaticClass()->GetFName(), "Physics", LOCTEXT("Physics", "Physics"));
	PhysicsSection->AddCategory("Jolt Physics");
	
	PropertyModule.NotifyCustomizationModuleChanged();
}

void FUnrealJoltEditorModule::ShutdownModule()
{
	if (FModuleManager::Get().IsModuleLoaded("PropertyEditor"))
	{
		FPropertyEditorModule& PropertyModule = FModuleManager::GetModuleChecked<FPropertyEditorModule>("PropertyEditor");
		PropertyModule.UnregisterCustomClassLayout(UJoltSettings::StaticClass()->GetFName());
		PropertyModule.UnregisterCustomPropertyTypeLayout(TEXT("EJoltMotionType"));
		PropertyModule.UnregisterCustomClassLayout(AActor::StaticClass()->GetFName());

		TSharedRef<FPropertySection> PhysicsSection = PropertyModule.FindOrCreateSection(
			UStaticMeshComponent::StaticClass()->GetFName(), "Physics", LOCTEXT("Physics", "Physics"));
		PhysicsSection->RemoveCategory("Jolt Physics");

		PropertyModule.NotifyCustomizationModuleChanged();
	}
}

#undef LOCTEXT_NAMESPACE

IMPLEMENT_MODULE(FUnrealJoltEditorModule, UnrealJoltEditor)
