#pragma once

#include "UnrealJolt/JoltMain.h"
#include "JoltSubsystem.h"
#include "UnrealJolt/Helpers.h"

class JoltAxisConstraint
{

public:
	JoltAxisConstraint(UJoltSubsystem* joltSubsystem)
		: AxisConstraint()
	{
		JoltSubsystem = joltSubsystem;
	}

	bool SolveVelocityConstraint(int32 BodyId1, int32 BodyId2, FVector inWorldSpaceAxis, float inMinLambda, float inMaxLambda)
	{

		// inWorldSpaceAxis is a unit vector, we don't have to apply any sort of scaling
		return AxisConstraint.SolveVelocityConstraint(*JoltSubsystem->GetBody(BodyId1), *JoltSubsystem->GetBody(BodyId2), JoltHelpers::ToJoltVec3(inWorldSpaceAxis, false), inMinLambda, inMaxLambda);
	}

	void CalculateConstraintProperties(int32 BodyId1, FVector inR1PlusU, int32 BodyId2, FVector inR2, FVector inWorldSpaceAxis, float inBias = 0.0f)
	{
		AxisConstraint.CalculateConstraintProperties(*JoltSubsystem->GetBody(BodyId1), JoltHelpers::ToJoltVec3(inR1PlusU), *JoltSubsystem->GetBody(BodyId2), JoltHelpers::ToJoltVec3(inR2), JoltHelpers::ToJoltVec3(inWorldSpaceAxis, false));
	}

	void Deactivate()
	{
		AxisConstraint.Deactivate();
	}

	float GetTotalLambda() const
	{
		return AxisConstraint.GetTotalLambda();
	}

	/*
	 * Compute the impulse required to stop a dynamic body (bodyID1) along an axis
	 * This is basically an axis constraint
	 * The second body bodyID2 can be static or dynamic
	 */
	static float ComputeStopImpulse(const UJoltSubsystem* joltsubsystem, uint32 bodyId1, uint32 bodyId2, const FVector& groundPosition, const FVector& inDirection)
	{

		JPH::Body* bodyA = joltsubsystem->GetBody(bodyId1);
		if (bodyA == nullptr)
		{
			UE_LOG(LogTemp, Warning, TEXT("bodyId1 does not belong to valid object"));
			if (bodyA->GetMotionProperties() == nullptr)
			{
				UE_LOG(LogTemp, Warning, TEXT("bodyA can't be static, use body B."));
			}
			return 0.0f;
		}
		JPH::Body* bodyB = joltsubsystem->GetBody(bodyId2);
		if (bodyB == nullptr)
		{
			UE_LOG(LogTemp, Warning, TEXT("bodyId2 does not belong to valid object"));
			return 0.0f;
		}

		ensure(inDirection != FVector::ZeroVector);

		JPH::Vec3 rA = JoltHelpers::ToJoltVec3(groundPosition) - JPH::Vec3(bodyA->GetCenterOfMassPosition());
		JPH::Vec3 rB = JoltHelpers::ToJoltVec3(groundPosition) - JPH::Vec3(bodyB->GetCenterOfMassPosition());

		const JPH::Vec3 direction = JoltHelpers::ToJoltVec3(inDirection, false);

		// Jacobians.
		JPH::Vec3 jLinA = -direction;
		JPH::Vec3 jAngA = -rA.Cross(direction);
		JPH::Vec3 jLinB = direction;
		JPH::Vec3 jAngB = rB.Cross(direction);

		JPH::Vec3 WJTLinA = bodyA->GetMotionProperties()->GetInverseMass() * jLinA;
		JPH::Vec3 WJTangA = bodyA->GetInverseInertia().Multiply3x3(jAngA);

		JPH::Vec3 WJTLinB = JPH::Vec3::sZero();
		JPH::Vec3 WJTangB = JPH::Vec3::sZero();

		if (bodyB->GetMotionPropertiesUnchecked() != nullptr)
		{
			WJTangB = bodyB->GetInverseInertia().Multiply3x3(jAngB);
			WJTLinB = bodyB->GetMotionProperties()->GetInverseMass() * jLinB;
		}

		float JWJT = jLinA.Dot(WJTLinA) + jAngA.Dot(WJTangA)
			+ jLinB.Dot(WJTLinB) + jAngB.Dot(WJTangB);

		float JWJTInverse = 1.0f / JWJT;

		float relativeVelocity;

		if (bodyB->GetMotionPropertiesUnchecked() != nullptr)
		{
			relativeVelocity = jLinA.Dot(bodyA->GetLinearVelocity())
				+ jAngA.Dot(bodyA->GetAngularVelocity())
				+ jLinB.Dot(bodyB->GetLinearVelocity())
				+ jAngB.Dot(bodyB->GetAngularVelocity());
		}
		else
		{
			relativeVelocity = jLinA.Dot(bodyA->GetLinearVelocity())
				+ jAngA.Dot(bodyA->GetAngularVelocity());
		}

		return JoltHelpers::ToUESize(JWJTInverse * relativeVelocity);
	}

private:
	UJoltSubsystem* JoltSubsystem = nullptr;

	JPH::AxisConstraintPart AxisConstraint;
};
