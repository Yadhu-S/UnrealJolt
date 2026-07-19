#pragma once

#include "CoreMinimal.h"
#include "IDetailCustomization.h"
#include "IDetailCustomNodeBuilder.h"

class UJoltPhysicsComponent;

/** Hides the default Physics category and mirrors Jolt Physics properties in its place on actors with a Jolt Physics component */
class FJoltPhysicsDetailsCustomization : public IDetailCustomization
{
public:
	static TSharedRef<IDetailCustomization> MakeInstance();
	virtual void CustomizeDetails(IDetailLayoutBuilder& DetailBuilder) override;
};

class FJoltPhysicsNodeBuilder : public IDetailCustomNodeBuilder
{
public:
	FJoltPhysicsNodeBuilder(IDetailLayoutBuilder* InDetailBuilder, UJoltPhysicsComponent* InJoltComponent)
	: DetailBuilder(InDetailBuilder), JoltComponent(InJoltComponent) {}
	
	virtual void GenerateHeaderRowContent(FDetailWidgetRow& NodeRow) override {};
	
	virtual void SetOnRebuildChildren(const FSimpleDelegate NewRegenerateChildren) override { OnRebuildChildren = NewRegenerateChildren; };

	virtual void GenerateChildContent(IDetailChildrenBuilder& ChildrenBuilder) override;
	virtual FName GetName() const override { return FName("JoltPhysicsProperties"); };

private:
	IDetailLayoutBuilder* DetailBuilder = nullptr;
	TWeakObjectPtr<UJoltPhysicsComponent> JoltComponent;
	FSimpleDelegate OnRebuildChildren;
	void OnMotionTypeChanged();
};