#pragma once

#include "Components/TextRenderComponent.h"
#include "UnrealJolt/Helpers.h"
#include "UnrealJolt/JoltMain.h"
#include "DrawDebugHelpers.h"

class UEJoltDebugRenderer final : public JPH::DebugRendererSimple
{
private:
	UWorld* World;

public:
	UEJoltDebugRenderer(UWorld* world)
	{
		World = world;
	}

	virtual void DrawLine(JPH::RVec3Arg inFrom, JPH::RVec3Arg inTo, JPH::ColorArg inColor) override
	{
		if (!World)
		{
			UE_LOG(JoltSubSystemLogs, Warning, TEXT("JoltPhysicsSubSystem::DebugRenderer World is null."));
			return;
		}

		DrawDebugLine(World,
			JoltHelpers::ToUEPos(inFrom),
			JoltHelpers::ToUEPos(inTo),
			JoltHelpers::ToUEColor(inColor));
	}

	virtual void DrawTriangle(JPH::RVec3Arg inV1, JPH::RVec3Arg inV2, JPH::RVec3Arg inV3, JPH::ColorArg inColor, ECastShadow inCastShadow) override
	{
		if (!World)
		{
			UE_LOG(JoltSubSystemLogs, Warning, TEXT("JoltPhysicsSubSystem::DebugRenderer World is null."));
			return;
		}
		FVector V1 = JoltHelpers::ToUEPos(inV1);
		FVector V2 = JoltHelpers::ToUEPos(inV2);
		FVector V3 = JoltHelpers::ToUEPos(inV3);

		FColor Color = JoltHelpers::ToUEColor(inColor);

		DrawDebugLine(World, V1, V2, Color, false, -1, 0, 1);
		DrawDebugLine(World, V2, V3, Color, false, -1, 0, 1);
		DrawDebugLine(World, V3, V1, Color, false, -1, 0, 1);
	}

	virtual void DrawText3D(JPH::RVec3Arg inPosition, const JPH::string_view& inString, JPH::ColorArg inColor, float inHeight) override
	{
		if (!World)
		{
			UE_LOG(JoltSubSystemLogs, Warning, TEXT("JoltPhysicsSubSystem::DebugRenderer World is null."));
			return;
		}

		FVector Position = JoltHelpers::ToUEPos(inPosition);
		FColor	Color = JoltHelpers::ToUEColor(inColor);

		FString TextString(inString.data());

		UTextRenderComponent* TextRenderComponent = NewObject<UTextRenderComponent>(World);
		if (!TextRenderComponent)
		{
			UE_LOG(JoltSubSystemLogs, Warning, TEXT("Failed to create UTextRenderComponent."));
			return;
		}

		TextRenderComponent->SetText(FText::FromString(TextString));
		TextRenderComponent->SetTextRenderColor(Color);
		TextRenderComponent->SetWorldLocation(Position);
		TextRenderComponent->SetWorldScale3D(FVector(inHeight / 100.0f));

		TextRenderComponent->RegisterComponentWithWorld(World);
	}
};
