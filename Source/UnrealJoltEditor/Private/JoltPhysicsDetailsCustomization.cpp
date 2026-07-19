#include "JoltPhysicsDetailsCustomization.h"
#include "JoltPhysicsComponent.h"
#include "DetailLayoutBuilder.h"
#include "DetailCategoryBuilder.h"
#include "IDetailChildrenBuilder.h"
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
		IDetailCategoryBuilder& CategoryBuilder = DetailBuilder.EditCategory(FName("Jolt Physics"));
		CategoryBuilder.AddCustomBuilder(MakeShared<FJoltPhysicsNodeBuilder>(&DetailBuilder, JoltComponent));
	}
	
	// Put it under materials, just like the original physics category
	const int32 MaterialsSortOrder = DetailBuilder.EditCategory("Materials").GetSortOrder();
	DetailBuilder.EditCategory("Jolt Physics").SetSortOrder(MaterialsSortOrder + 1);
}

void FJoltPhysicsNodeBuilder::GenerateChildContent(IDetailChildrenBuilder& ChildrenBuilder)
{
	UJoltPhysicsComponent* Component = JoltComponent.Get();
	if (!Component || !DetailBuilder) return;

	TArray<UObject*> JoltObjects = { Component };
	TMap<FName, IDetailGroup*> Groups;

	for (TFieldIterator<FProperty> PropertyIt(UJoltPhysicsComponent::StaticClass()); PropertyIt; ++PropertyIt)
	{
		FProperty* Property = *PropertyIt;
		if (!Property->HasAnyPropertyFlags(CPF_Edit)) continue;

		// "Jolt Physics" will be the main category, and then we'll group by whatever comes after
		// We don't currently nest any further than the first subgroup, so be careful with categories on the component
		const FString FullCategory = Property->GetMetaData(TEXT("Category"));
		if (FullCategory.IsEmpty()) continue;

		FString TopCategory = FullCategory;
		FString SubGroup;
		FullCategory.Split(TEXT("|"), &TopCategory, &SubGroup);

		TSharedPtr<IPropertyHandle> Handle = DetailBuilder->AddObjectPropertyData(JoltObjects, Property->GetFName());
		if (!Handle.IsValid()) continue;

		// We rebuild, since for some reason, switching from static to dynamic doesn't update the values on EditCondition properties
		if (Property->GetFName() == GET_MEMBER_NAME_CHECKED(UJoltPhysicsComponent, MotionType))
		{
			Handle->SetOnPropertyValueChanged(FSimpleDelegate::CreateRaw(this, &FJoltPhysicsNodeBuilder::OnMotionTypeChanged));
		}

		if (SubGroup.IsEmpty())
		{
			ChildrenBuilder.AddProperty(Handle.ToSharedRef());
		}
		else
		{
			const FName GroupName(*SubGroup);
			IDetailGroup*& Group = Groups.FindOrAdd(GroupName);
			if (!Group) Group = &ChildrenBuilder.AddGroup(GroupName, FText::FromName(GroupName));
			Group->AddPropertyRow(Handle.ToSharedRef());
		}
	}
}

void FJoltPhysicsNodeBuilder::OnMotionTypeChanged()
{
    OnRebuildChildren.ExecuteIfBound();
}