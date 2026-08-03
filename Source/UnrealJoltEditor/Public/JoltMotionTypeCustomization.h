#pragma once

#include "CoreMinimal.h"
#include "HAL/Platform.h"
#include "IPropertyTypeCustomization.h"
#include "Internationalization/Text.h"
#include "Styling/SlateColor.h"
#include "Templates/SharedPointer.h"
#include "UObject/NameTypes.h"

class IDetailCategoryBuilder;
class IDetailLayoutBuilder;
class IPropertyHandle;

enum class EJoltMotionType : uint8;

/** A switcher for motion type in the style of Unreal's physics mobility enum. */
class FJoltMotionTypeCustomization : public IPropertyTypeCustomization
{
public:
	static TSharedRef<IPropertyTypeCustomization> MakeInstance();

	virtual void CustomizeHeader(TSharedRef<IPropertyHandle> PropertyHandle, FDetailWidgetRow& HeaderRow, IPropertyTypeCustomizationUtils& CustomizationUtils) override;
	virtual void CustomizeChildren(TSharedRef<IPropertyHandle> PropertyHandle, IDetailChildrenBuilder& ChildBuilder, IPropertyTypeCustomizationUtils& CustomizationUtils) override {}

private:
	EJoltMotionType GetActiveMotionType() const;
	FSlateColor GetMotionTypeTextColor(EJoltMotionType InMotionType) const;
	void OnMotionTypeChanged(EJoltMotionType InMotionType);
	FText GetMotionTypeToolTip() const;

	TSharedPtr<IPropertyHandle> MotionTypeHandle;
};