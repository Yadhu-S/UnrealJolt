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

	if (Objects.Num() != 1) return;

	const UPrimitiveComponent* PrimitiveComponent = Cast<UPrimitiveComponent>(Objects[0].Get());
	if (!PrimitiveComponent) return;

	const AActor* Owner = PrimitiveComponent->GetOwner();
	if (!Owner) return;

	UJoltPhysicsComponent* JoltComponent = Owner->FindComponentByClass<UJoltPhysicsComponent>();
	if (!JoltComponent) return;

	DetailBuilder.HideCategory(FName("Physics"));

	IDetailCategoryBuilder& JoltCategory = DetailBuilder.EditCategory("Jolt Physics");

	// Put it under materials, just like the original physics category
	const int32 MaterialsSortOrder = DetailBuilder.EditCategory("Materials").GetSortOrder();
	JoltCategory.SetSortOrder(MaterialsSortOrder + 1);

	TArray<UObject*> JoltObjects;
	JoltObjects.Add(JoltComponent);

	// Adds each named property as a row inside a named subgroup of the Jolt Physics category
	auto AddGroup = [&](const FName& GroupName, const TArray<FName>& PropertyNames)
	{
		IDetailGroup& Group = JoltCategory.AddGroup(GroupName, FText::FromName(GroupName));

		for (const FName& PropertyName : PropertyNames)
		{
			TSharedPtr<IPropertyHandle> Handle = DetailBuilder.AddObjectPropertyData(JoltObjects, PropertyName);
			if (Handle.IsValid())
			{
				Group.AddPropertyRow(Handle.ToSharedRef());
			}
		}
	};

	// I don't foresee new properties to be added constantly, so - even though it's a little
	// unwieldy - this should be fine.
	AddGroup(FName("Motion"), { 
		GET_MEMBER_NAME_CHECKED(UJoltPhysicsComponent, MotionType), 
		GET_MEMBER_NAME_CHECKED(UJoltPhysicsComponent, AllowedDOFs), 
		GET_MEMBER_NAME_CHECKED(UJoltPhysicsComponent, Layer), 
		GET_MEMBER_NAME_CHECKED(UJoltPhysicsComponent, Mass) 
		});
	
	AddGroup(FName("Forces"), { 
		GET_MEMBER_NAME_CHECKED(UJoltPhysicsComponent, GravityFactor), 
		GET_MEMBER_NAME_CHECKED(UJoltPhysicsComponent, bApplyGyroscopicForce), 
		GET_MEMBER_NAME_CHECKED(UJoltPhysicsComponent, MaxLinearVelocity), 
		GET_MEMBER_NAME_CHECKED(UJoltPhysicsComponent, MaxAngularVelocity) 
	});
	
	AddGroup(FName("Surface"), { 
		GET_MEMBER_NAME_CHECKED(UJoltPhysicsComponent, Friction), 
		GET_MEMBER_NAME_CHECKED(UJoltPhysicsComponent, Restitution) 
	});
	
	AddGroup(FName("Damping"), { 
		GET_MEMBER_NAME_CHECKED(UJoltPhysicsComponent, LinearDamping), 
		GET_MEMBER_NAME_CHECKED(UJoltPhysicsComponent, AngularDamping) 
	});
	
	AddGroup(FName("Advanced"), { 
		GET_MEMBER_NAME_CHECKED(UJoltPhysicsComponent, bAllowSleeping), 
		GET_MEMBER_NAME_CHECKED(UJoltPhysicsComponent, NumVelocityStepsOverride), 
		GET_MEMBER_NAME_CHECKED(UJoltPhysicsComponent, NumPositionStepsOverride), 
		GET_MEMBER_NAME_CHECKED(UJoltPhysicsComponent, bEnhancedInternalEdgeRemoval) 
	});
}