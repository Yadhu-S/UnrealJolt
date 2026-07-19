#include "JoltPhysicsDetailsCustomization.h"
#include "JoltPhysicsComponent.h"
#include "Components/PrimitiveComponent.h"
#include "DetailLayoutBuilder.h"
#include "DetailCategoryBuilder.h"
#include "IDetailGroup.h"

TSharedRef<IDetailCustomization> FJoltPhysicsDetailsCustomization::MakeInstance()
{
	return MakeShared<FJoltPhysicsDetailsCustomization>();
}

void FJoltPhysicsDetailsCustomization::CustomizeDetails(IDetailLayoutBuilder& DetailBuilder)
{
	TArray<TWeakObjectPtr<UObject>> Objects;
	DetailBuilder.GetObjectsBeingCustomized(Objects);
	
	if (Objects.IsEmpty()) return;
	
	const AActor* Owner = Cast<AActor>(Objects[0].Get());
	if (!Owner) return;
	
	UJoltPhysicsComponent* JoltComponent = Owner->FindComponentByClass<UJoltPhysicsComponent>();
	if (!JoltComponent) return;
	
	DetailBuilder.HideCategory(FName("Physics"));

    // If a UPROPERTY on the class points at this component, the engine already flattens and nests its properties correctly, so nothing further is needed
    bool bIsNativeProperty = false;
    for (TFieldIterator<FObjectProperty> PropertyIt(Owner->GetClass()); PropertyIt; ++PropertyIt)
    {
       FObjectProperty* ObjectProperty = *PropertyIt;
       if (!ObjectProperty->PropertyClass->IsChildOf(UJoltPhysicsComponent::StaticClass())) continue;
       if (ObjectProperty->GetObjectPropertyValue(ObjectProperty->ContainerPtrToValuePtr<void>(Owner)) != JoltComponent) continue;
       bIsNativeProperty = true;
       break;
    }

    if (!bIsNativeProperty)
    {
    	// Instance added component, no UPROPERTY backs it, so nothing auto flattens and we have to build the rows ourselves
    	TArray<UObject*> JoltObjects = { JoltComponent };
    	TMap<FName, IDetailGroup*> Groups;
    	
    	for (TFieldIterator<FProperty> PropertyIt(UJoltPhysicsComponent::StaticClass()); PropertyIt; ++PropertyIt)
    	{
    		FProperty* Property = *PropertyIt;
    		if (!Property->HasAnyPropertyFlags(CPF_Edit)) continue;
    		
    		const FString FullCategory = Property->GetMetaData(TEXT("Category"));
    		if (FullCategory.IsEmpty()) continue;
    		
    		// We have to handle subgroups (e.g. Jolt Physics|Motion) ourselves
    		FString TopCategory = FullCategory;
    		FString SubGroup;
    		FullCategory.Split(TEXT("|"), &TopCategory, &SubGroup);
    		
    		TSharedPtr<IPropertyHandle> Handle = DetailBuilder.AddObjectPropertyData(JoltObjects, Property->GetFName());
    		if (!Handle.IsValid()) continue;
    		
    		IDetailCategoryBuilder& CategoryBuilder = DetailBuilder.EditCategory(FName(*TopCategory));
    		
    		if (SubGroup.IsEmpty())
    		{
    			CategoryBuilder.AddProperty(Handle.ToSharedRef());
    			continue;
    		}
    		
    		const FName GroupName(*SubGroup);
    		IDetailGroup*& Group = Groups.FindOrAdd(GroupName);
    		if (!Group) Group = &CategoryBuilder.AddGroup(GroupName, FText::FromName(GroupName));
    		Group->AddPropertyRow(Handle.ToSharedRef());
    	}
    }
	
    // Put it under materials, just like the original physics category
    const int32 MaterialsSortOrder = DetailBuilder.EditCategory("Materials").GetSortOrder();
    DetailBuilder.EditCategory("Jolt Physics").SetSortOrder(MaterialsSortOrder + 1);
}