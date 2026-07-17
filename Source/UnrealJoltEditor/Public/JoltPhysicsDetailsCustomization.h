#pragma once

#include "CoreMinimal.h"
#include "IDetailCustomization.h"

/** Hide the default physics category, and replaces them with Jolt Physics' properties to declutter static mesh components */
class FJoltPhysicsDetailsCustomization : public IDetailCustomization
{
public:
	static TSharedRef<IDetailCustomization> MakeInstance();
	virtual void CustomizeDetails(IDetailLayoutBuilder& DetailBuilder) override;
};