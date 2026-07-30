#pragma once


#ifdef JPH_DEBUG_RENDERER

#include "UnrealJolt/Helpers.h"
#include "UnrealJolt/JoltMain.h"
#include "JoltSettings.h"

#include <Jolt/Physics/Body/BodyFilter.h>

class UEJoltBodyDrawFilter : public JPH::BodyDrawFilter
{
	public:
		bool bDrawStatic = true;
		bool bDrawDynamic = true;
		bool bDrawKinematic = true;
		bool bDrawHeightField = true;

		virtual bool ShouldDraw(const JPH::Body& inBody) const override
		{
			if (inBody.GetShape()->GetType() == JPH::EShapeType::HeightField)
			{
				return bDrawHeightField;
			}

			switch (inBody.GetMotionType())
			{
				case JPH::EMotionType::Static:
					return bDrawStatic;
				case JPH::EMotionType::Dynamic:
					return bDrawDynamic;
				case JPH::EMotionType::Kinematic:
					return bDrawKinematic;
				default:
					return true;
			}
		}
};

class UEJoltDebugRenderer final : public JPH::DebugRendererSimple
{
	private:
		UWorld* World;
		bool	bLoggedMissingWorld = false;

		UEJoltBodyDrawFilter DrawFilter;

		bool	  bDrawingStaticsPersistent = false;
		bool	  bStaticsDrawn = false;
		JPH::uint CachedBodyCount = 0;
		bool	  bCachedDrawStatic = false;
		bool	  bCachedDrawHeightField = false;

		bool EnsureWorld();

		bool ShouldRedrawStatics(JPH::PhysicsSystem* PhysicsSystem, const UJoltSettings* JoltSettings) const;
		void CacheStaticState(JPH::PhysicsSystem* PhysicsSystem, const UJoltSettings* JoltSettings);
		void DrawStaticBodies(JPH::PhysicsSystem* PhysicsSystem, const JPH::BodyManager::DrawSettings& Settings, const UJoltSettings* JoltSettings);
		void DrawDynamicBodies(JPH::PhysicsSystem* PhysicsSystem, const JPH::BodyManager::DrawSettings& Settings, const UJoltSettings* JoltSettings);

	public:
		UEJoltDebugRenderer(UWorld* world);

		void DrawBodiesFiltered(JPH::PhysicsSystem* PhysicsSystem, const JPH::BodyManager::DrawSettings& Settings, const UJoltSettings* JoltSettings);

		virtual void DrawLine(JPH::RVec3Arg inFrom, JPH::RVec3Arg inTo, JPH::ColorArg inColor) override;
		virtual void DrawTriangle(JPH::RVec3Arg inV1, JPH::RVec3Arg inV2, JPH::RVec3Arg inV3, JPH::ColorArg inColor, ECastShadow inCastShadow) override;
		virtual void DrawText3D(JPH::RVec3Arg inPosition, const JPH::string_view& inString, JPH::ColorArg inColor, float inHeight) override;
};

#endif // JPH_DEBUG_RENDERER
