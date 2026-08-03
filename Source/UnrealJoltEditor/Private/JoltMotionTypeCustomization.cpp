#include "JoltMotionTypeCustomization.h"
#include "JoltPhysicsComponent.h"

#include "DetailLayoutBuilder.h"
#include "DetailWidgetRow.h"
#include "Fonts/SlateFontInfo.h"
#include "Internationalization/Internationalization.h"
#include "PropertyHandle.h"
#include "Styling/AppStyle.h"
#include "Widgets/DeclarativeSyntaxSupport.h"
#include "Widgets/Input/SSegmentedControl.h"
#include "Widgets/Text/STextBlock.h"

#define LOCTEXT_NAMESPACE "JoltMotionTypeCustomization"

TSharedRef<IPropertyTypeCustomization> FJoltMotionTypeCustomization::MakeInstance()
{
	return MakeShared<FJoltMotionTypeCustomization>();
}

void FJoltMotionTypeCustomization::CustomizeHeader(TSharedRef<IPropertyHandle> PropertyHandle, FDetailWidgetRow& HeaderRow, IPropertyTypeCustomizationUtils& CustomizationUtils)
{
	// Most of this more or less mirrors how Unreal customizes their mobility enum
	
	MotionTypeHandle = PropertyHandle;

	TSharedRef<SSegmentedControl<EJoltMotionType>> ButtonOptionsPanel =
		SNew(SSegmentedControl<EJoltMotionType>)
		.Value(this, &FJoltMotionTypeCustomization::GetActiveMotionType)
		.OnValueChanged(this, &FJoltMotionTypeCustomization::OnMotionTypeChanged);

	HeaderRow
	.NameContent()
	[
		SNew(STextBlock)
		.Text(LOCTEXT("MotionType", "Motion Type"))
		.ToolTipText(this, &FJoltMotionTypeCustomization::GetMotionTypeToolTip)
		.Font(IDetailLayoutBuilder::GetDetailFont())
	]
	.ValueContent()
	.MaxDesiredWidth(0)
	[
		ButtonOptionsPanel
	]
	.FilterString(LOCTEXT("MotionType", "Motion Type"));

	// Static Motion Type
	ButtonOptionsPanel->AddSlot(EJoltMotionType::Static)
	.HAlign(HAlign_Center)
	.VAlign(VAlign_Center)
	[
		SNew(STextBlock)
		.Font(FAppStyle::Get().GetFontStyle("PropertyWindow.MobilityFont"))
		.Text(LOCTEXT("Static", "Static"))
	]
	.ToolTip(LOCTEXT("MotionType_Static_Tooltip", "A static body cannot move and has no mass. Ideal for level geometry.\n* Fastest simulation\n* Collides with dynamic bodies"));

	// Dynamic Motion Type
	ButtonOptionsPanel->AddSlot(EJoltMotionType::Dynamic)
	.HAlign(HAlign_Center)
	.VAlign(VAlign_Center)
	[
		SNew(STextBlock)
		.Font(FAppStyle::Get().GetFontStyle("PropertyWindow.MobilityFont"))
		.Text(LOCTEXT("Dynamic", "Dynamic"))
	]
	.ToolTip(LOCTEXT("MotionType_Dynamic_Tooltip", "A dynamic body is simulated by Jolt and affected by forces.\n* Collides with static and dynamic bodies\n* Fully driven by physics at runtime"));

	ButtonOptionsPanel->RebuildChildren();
}

EJoltMotionType FJoltMotionTypeCustomization::GetActiveMotionType() const
{
	if (MotionTypeHandle.IsValid())
	{
		uint8 MotionTypeByte;
		MotionTypeHandle->GetValue(MotionTypeByte);

		return (EJoltMotionType)MotionTypeByte;
	}

	return EJoltMotionType::Static;
}

FSlateColor FJoltMotionTypeCustomization::GetMotionTypeTextColor(EJoltMotionType InMotionType) const
{
	if (MotionTypeHandle.IsValid())
	{
		uint8 MotionTypeByte;
		MotionTypeHandle->GetValue(MotionTypeByte);

		return MotionTypeByte == (uint8)InMotionType ? FSlateColor(FLinearColor(0, 0, 0)) : FSlateColor(FLinearColor(0.72f, 0.72f, 0.72f, 1.f));
	}

	return FSlateColor(FLinearColor(0.72f, 0.72f, 0.72f, 1.f));
}

void FJoltMotionTypeCustomization::OnMotionTypeChanged(EJoltMotionType InMotionType)
{
	if (MotionTypeHandle.IsValid())
	{
		MotionTypeHandle->SetValue((uint8)InMotionType);
	}
}

FText FJoltMotionTypeCustomization::GetMotionTypeToolTip() const
{
	if (MotionTypeHandle.IsValid())
	{
		return MotionTypeHandle->GetToolTipText();
	}

	return FText::GetEmpty();
}

#undef LOCTEXT_NAMESPACE