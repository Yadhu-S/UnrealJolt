#include "JoltDebugRenderer.h"

#include "DrawDebugHelpers.h"

bool UEJoltDebugRenderer::EnsureWorld()
{
	if (World)
	{
		bLoggedMissingWorld = false;
		return true;
	}

	if (!bLoggedMissingWorld)
	{
		UE_LOG(JoltSubSystemLogs, Warning, TEXT("JoltPhysicsSubSystem::DebugRenderer World is null."));
		bLoggedMissingWorld = true;
	}

	return false;
}

UEJoltDebugRenderer::UEJoltDebugRenderer(UWorld* world)
	: World(world)
{
}

bool UEJoltDebugRenderer::ShouldRedrawStatics(JPH::PhysicsSystem* PhysicsSystem, const UJoltSettings* JoltSettings) const
{
	if (!bStaticsDrawn)
	{
		return true;
	}

	if (bCachedDrawStatic != JoltSettings->bDebugDrawStaticBodies || bCachedDrawHeightField != JoltSettings->bDebugDrawHeightFields)
	{
		return true;
	}

	return CachedBodyCount != PhysicsSystem->GetNumBodies();
}

void UEJoltDebugRenderer::CacheStaticState(JPH::PhysicsSystem* PhysicsSystem, const UJoltSettings* JoltSettings)
{
	bStaticsDrawn = true;
	CachedBodyCount = PhysicsSystem->GetNumBodies();
	bCachedDrawStatic = JoltSettings->bDebugDrawStaticBodies;
	bCachedDrawHeightField = JoltSettings->bDebugDrawHeightFields;
}

void UEJoltDebugRenderer::DrawStaticBodies(JPH::PhysicsSystem* PhysicsSystem, const JPH::BodyManager::DrawSettings& Settings, const UJoltSettings* JoltSettings)
{
	FlushPersistentDebugLines(World);

	DrawFilter.bDrawStatic = JoltSettings->bDebugDrawStaticBodies;
	DrawFilter.bDrawDynamic = false;
	DrawFilter.bDrawKinematic = false;
	DrawFilter.bDrawHeightField = JoltSettings->bDebugDrawHeightFields;

	bDrawingStaticsPersistent = true;
	PhysicsSystem->DrawBodies(Settings, this, &DrawFilter);
	bDrawingStaticsPersistent = false;
}

void UEJoltDebugRenderer::DrawDynamicBodies(JPH::PhysicsSystem* PhysicsSystem, const JPH::BodyManager::DrawSettings& Settings, const UJoltSettings* JoltSettings)
{
	DrawFilter.bDrawStatic = false;
	DrawFilter.bDrawDynamic = JoltSettings->bDebugDrawDynamicBodies;
	DrawFilter.bDrawKinematic = JoltSettings->bDebugDrawKinematicBodies;
	DrawFilter.bDrawHeightField = false;

	PhysicsSystem->DrawBodies(Settings, this, &DrawFilter);
}

void UEJoltDebugRenderer::DrawBodiesFiltered(JPH::PhysicsSystem* PhysicsSystem, const JPH::BodyManager::DrawSettings& Settings, const UJoltSettings* JoltSettings)
{
	if (!EnsureWorld())
	{
		return;
	}

	const bool bWantStatics = JoltSettings->bDebugDrawStaticBodies || JoltSettings->bDebugDrawHeightFields;
	if (bWantStatics)
	{
		if (ShouldRedrawStatics(PhysicsSystem, JoltSettings))
		{
			DrawStaticBodies(PhysicsSystem, Settings, JoltSettings);
			CacheStaticState(PhysicsSystem, JoltSettings);
		}
	}
	else if (bStaticsDrawn)
	{
		FlushPersistentDebugLines(World);
		bStaticsDrawn = false;
	}

	if (JoltSettings->bDebugDrawDynamicBodies || JoltSettings->bDebugDrawKinematicBodies)
	{
		DrawDynamicBodies(PhysicsSystem, Settings, JoltSettings);
	}

	NextFrame();
}

void UEJoltDebugRenderer::DrawLine(JPH::RVec3Arg inFrom, JPH::RVec3Arg inTo, JPH::ColorArg inColor)
{
	if (!EnsureWorld())
	{
		return;
	}

	DrawDebugLine(World,
		JoltHelpers::ToUEPos(inFrom),
		JoltHelpers::ToUEPos(inTo),
		JoltHelpers::ToUEColor(inColor),
		bDrawingStaticsPersistent,
		-1.0f);
}

void UEJoltDebugRenderer::DrawTriangle(JPH::RVec3Arg inV1, JPH::RVec3Arg inV2, JPH::RVec3Arg inV3, JPH::ColorArg inColor, ECastShadow inCastShadow)
{
	if (!EnsureWorld())
	{
		return;
	}

	FVector V1 = JoltHelpers::ToUEPos(inV1);
	FVector V2 = JoltHelpers::ToUEPos(inV2);
	FVector V3 = JoltHelpers::ToUEPos(inV3);
	FColor	Color = JoltHelpers::ToUEColor(inColor);

	DrawDebugLine(World, V1, V2, Color, bDrawingStaticsPersistent, -1.0f);
	DrawDebugLine(World, V2, V3, Color, bDrawingStaticsPersistent, -1.0f);
	DrawDebugLine(World, V3, V1, Color, bDrawingStaticsPersistent, -1.0f);
}

void UEJoltDebugRenderer::DrawText3D(JPH::RVec3Arg inPosition, const JPH::string_view& inString, JPH::ColorArg inColor, float inHeight)
{
	if (!EnsureWorld())
	{
		return;
	}

	FVector Position = JoltHelpers::ToUEPos(inPosition);
	FColor	Color = JoltHelpers::ToUEColor(inColor);
	FString TextString(inString.data(), static_cast<int32>(inString.size()));
	float height = JoltHelpers::ToUESize(inHeight);

	DrawDebugString(World, Position, TextString, nullptr, Color, -1.0f, false, height);
}
