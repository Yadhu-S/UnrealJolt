#pragma once

#include "CoreMinimal.h"
#include "HAL/Platform.h"
#include "IDetailCustomization.h"
#include "IDetailCustomNodeBuilder.h"
#include "Internationalization/Text.h"
#include "Styling/SlateColor.h"
#include "Templates/SharedPointer.h"
#include "UObject/NameTypes.h"

class FDetailWidgetRow;
class IDetailCategoryBuilder;
class IDetailLayoutBuilder;
class IPropertyHandle;

enum class EJoltMobility : uint8;

class FJoltMobilityCustomization : public IDetailCustomNodeBuilder, public TSharedFromThis<FJoltMobilityCustomization>
{
public:
	explicit FJoltMobilityCustomization(TSharedPtr<IPropertyHandle> InMobilityHandle);

	virtual void GenerateHeaderRowContent(FDetailWidgetRow& WidgetRow) override;
	virtual FName GetName() const override;
	virtual TSharedPtr<IPropertyHandle> GetPropertyHandle() const override { return MobilityHandle; }

	virtual void SetOnRebuildChildren(FSimpleDelegate InOnRegenerateChildren) override {}
	virtual void GenerateChildContent(IDetailChildrenBuilder& ChildrenBuilder) override {}
	virtual bool RequiresTick() const override { return false; }
	virtual bool InitiallyCollapsed() const override { return false; }

private:
	EJoltMobility GetActiveMobility() const;
	FSlateColor GetMobilityTextColor(EJoltMobility InMobility) const;
	void OnMobilityChanged(EJoltMobility InMobility);
	FText GetMobilityToolTip() const;

	TSharedPtr<IPropertyHandle> MobilityHandle;
};

class FJoltPhysicsComponentDetails : public IDetailCustomization
{
public:
	static TSharedRef<IDetailCustomization> MakeInstance();

	virtual void CustomizeDetails(IDetailLayoutBuilder& DetailBuilder) override;
};