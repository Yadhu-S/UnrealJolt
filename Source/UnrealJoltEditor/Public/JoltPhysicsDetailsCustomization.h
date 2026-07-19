#pragma once

#include "CoreMinimal.h"
#include "IDetailCustomization.h"

/** Hides the default Physics category and mirrors Jolt Physics properties in its place on actors with a Jolt Physics component */
class FJoltPhysicsDetailsCustomization : public IDetailCustomization
{
public:
	static TSharedRef<IDetailCustomization> MakeInstance();
	virtual void CustomizeDetails(IDetailLayoutBuilder& DetailBuilder) override;
};