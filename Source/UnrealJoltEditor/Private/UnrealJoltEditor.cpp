#include "UnrealJoltEditor.h"

#include "BlueprintCompilationManager.h"
#include "JoltMotionTypeCustomization.h"
#include "JoltPhysicsBlueprintCompilerExtension.h"
#include "JoltPhysicsComponent.h"
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
		UStaticMeshComponent::StaticClass()->GetFName(),
		FOnGetDetailCustomizationInstance::CreateStatic(&FJoltPhysicsDetailsCustomization::MakeInstance));
	
	TSharedRef<FPropertySection> PhysicsSection = PropertyModule.FindOrCreateSection(
		UStaticMeshComponent::StaticClass()->GetFName(), "Physics", LOCTEXT("Physics", "Physics"));
	PhysicsSection->AddCategory("Jolt Physics");
	
	PropertyModule.NotifyCustomizationModuleChanged();
	
	CompilerExtension = NewObject<UJoltPhysicsBlueprintCompilerExtension>(GetTransientPackage(), NAME_None, RF_Standalone);
	CompilerExtension->AddToRoot();
	FBlueprintCompilationManager::RegisterCompilerExtension(UBlueprint::StaticClass(), CompilerExtension);
}

void FUnrealJoltEditorModule::ShutdownModule()
{
	if (FModuleManager::Get().IsModuleLoaded("PropertyEditor"))
	{
		FPropertyEditorModule& PropertyModule = FModuleManager::GetModuleChecked<FPropertyEditorModule>("PropertyEditor");
		PropertyModule.UnregisterCustomClassLayout(UJoltSettings::StaticClass()->GetFName());
		PropertyModule.UnregisterCustomPropertyTypeLayout(TEXT("EJoltMotionType"));
		PropertyModule.UnregisterCustomClassLayout(UStaticMeshComponent::StaticClass()->GetFName());
		
		PropertyModule.NotifyCustomizationModuleChanged();
	}
	
	if (CompilerExtension && UObjectInitialized())
	{
		CompilerExtension->RemoveFromRoot();
	}

	CompilerExtension = nullptr;
}

#undef LOCTEXT_NAMESPACE

IMPLEMENT_MODULE(FUnrealJoltEditorModule, UnrealJoltEditor)
