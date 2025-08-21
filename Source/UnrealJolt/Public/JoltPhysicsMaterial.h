#pragma once

#include "Chaos/ChaosEngineInterface.h"
#include "UnrealJolt/JoltMain.h"

/**
 *
 */
class JoltPhysicsMaterial : public JPH::PhysicsMaterial
{
public:
	JoltPhysicsMaterial();
	~JoltPhysicsMaterial();

	    bool operator==(const JoltPhysicsMaterial& other) const
    {
        constexpr float EPSILON = 1e-6f; // tolerance for floating point comparisons
        return std::fabs(Friction - other.Friction) < EPSILON &&
               std::fabs(Restitution - other.Restitution) < EPSILON &&
               SurfaceType == other.SurfaceType;
    }

    bool operator!=(const JoltPhysicsMaterial& other) const
    {
        return !(*this == other);
    }

	float Friction;

	float Restitution;

	EPhysicalSurface SurfaceType;
};
