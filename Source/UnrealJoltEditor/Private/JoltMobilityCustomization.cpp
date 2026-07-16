#include "JoltMobilityCustomization.h"
#include "JoltPhysicsComponent.h"
#include "DetailCategoryBuilder.h"
#include "DetailLayoutBuilder.h"
#include "DetailWidgetRow.h"
#include "Fonts/SlateFontInfo.h"
#include "Internationalization/Internationalization.h"
#include "PropertyHandle.h"
#include "Styling/AppStyle.h"
#include "Widgets/DeclarativeSyntaxSupport.h"
#include "Widgets/Input/SSegmentedControl.h"
#include "Widgets/Text/STextBlock.h"

#define LOCTEXT_NAMESPACE "JoltMobilityCustomization"

FJoltMobilityCustomization::FJoltMobilityCustomization(TSharedPtr<IPropertyHandle> InMobilityHandle)
{
    MobilityHandle = InMobilityHandle;
    MobilityHandle->MarkHiddenByCustomization();
}

FName FJoltMobilityCustomization::GetName() const
{
    const FProperty* Property = MobilityHandle->GetProperty();
    if (Property != nullptr)
    {
       return Property->GetFName();
    }
    return NAME_None;
}

void FJoltMobilityCustomization::GenerateHeaderRowContent(FDetailWidgetRow& WidgetRow)
{
    TSharedRef<SSegmentedControl<EJoltMobility>> ButtonOptionsPanel =
       SNew(SSegmentedControl<EJoltMobility>)
       .Value(this, &FJoltMobilityCustomization::GetActiveMobility)
       .OnValueChanged(this, &FJoltMobilityCustomization::OnMobilityChanged);

    WidgetRow
    .NameContent()
    [
       SNew(STextBlock)
       .Text(LOCTEXT("Mobility", "Mobility"))
       .ToolTipText(this, &FJoltMobilityCustomization::GetMobilityToolTip)
       .Font(IDetailLayoutBuilder::GetDetailFont())
    ]
    .ValueContent()
    .MaxDesiredWidth(0)
    [
       ButtonOptionsPanel
    ]
    .FilterString(LOCTEXT("Mobility", "Mobility"));

    // Static Mobility
    ButtonOptionsPanel->AddSlot(EJoltMobility::Static)
    .HAlign(HAlign_Center)
    .VAlign(VAlign_Center)
    [
       SNew(STextBlock)
       .Font(FAppStyle::Get().GetFontStyle("PropertyWindow.MobilityFont"))
       .Text(LOCTEXT("Static", "Static"))
    ]
    .ToolTip(LOCTEXT("Mobility_Static_Tooltip", "A static body cannot move and has no mass.\n* Fastest simulation\n* Cannot be moved at runtime"));

    // Dynamic Mobility
    ButtonOptionsPanel->AddSlot(EJoltMobility::Dynamic)
    .HAlign(HAlign_Center)
    .VAlign(VAlign_Center)
    [
       SNew(STextBlock)
       .Font(FAppStyle::Get().GetFontStyle("PropertyWindow.MobilityFont"))
       .Text(LOCTEXT("Dynamic", "Dynamic"))
    ]
    .ToolTip(LOCTEXT("Mobility_Dynamic_Tooltip", "A dynamic body is simulated by Jolt and affected by forces, mass, friction, and restitution."));

    ButtonOptionsPanel->RebuildChildren();
}

EJoltMobility FJoltMobilityCustomization::GetActiveMobility() const
{
    if (MobilityHandle.IsValid())
    {
       uint8 MobilityByte;
       MobilityHandle->GetValue(MobilityByte);

       return (EJoltMobility)MobilityByte;
    }

    return EJoltMobility::Static;
}

FSlateColor FJoltMobilityCustomization::GetMobilityTextColor(EJoltMobility InMobility) const
{
    if (MobilityHandle.IsValid())
    {
       uint8 MobilityByte;
       MobilityHandle->GetValue(MobilityByte);

       return MobilityByte == (uint8)InMobility ? FSlateColor(FLinearColor(0, 0, 0)) : FSlateColor(FLinearColor(0.72f, 0.72f, 0.72f, 1.f));
    }

    return FSlateColor(FLinearColor(0.72f, 0.72f, 0.72f, 1.f));
}

void FJoltMobilityCustomization::OnMobilityChanged(EJoltMobility InMobility)
{
    if (MobilityHandle.IsValid())
    {
       MobilityHandle->SetValue((uint8)InMobility);
    }
}

FText FJoltMobilityCustomization::GetMobilityToolTip() const
{
    if (MobilityHandle.IsValid())
    {
       return MobilityHandle->GetToolTipText();
    }

    return FText::GetEmpty();
}

TSharedRef<IDetailCustomization> FJoltPhysicsComponentDetails::MakeInstance()
{
    return MakeShared<FJoltPhysicsComponentDetails>();
}

void FJoltPhysicsComponentDetails::CustomizeDetails(IDetailLayoutBuilder& DetailBuilder)
{
    TSharedPtr<IPropertyHandle> MobilityHandle = DetailBuilder.GetProperty(GET_MEMBER_NAME_CHECKED(UJoltPhysicsComponent, Mobility));

    IDetailCategoryBuilder& Category = DetailBuilder.EditCategory("Jolt Physics");
    Category.AddCustomBuilder(MakeShared<FJoltMobilityCustomization>(MobilityHandle));
}

#undef LOCTEXT_NAMESPACE