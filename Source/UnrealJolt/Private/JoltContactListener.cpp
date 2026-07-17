#include "JoltContactListener.h"
#include "JoltPhysicsMaterial.h"
#include "UnrealJolt/Helpers.h"

static constexpr float ScrapePersistMinTangentialSpeed = 0.35f; // ~35 cm/s
static constexpr float ScrapePersistMinTangentialSpeedSq = ScrapePersistMinTangentialSpeed * ScrapePersistMinTangentialSpeed;

static EPhysicalSurface ResolveContactSurface(const JPH::Body& Body, const JPH::SubShapeID& SubShapeID)
{
	const JPH::Shape* Shape = Body.GetShape();
	if (Shape == nullptr)
	{
		return SurfaceType_Default;
	}
	const JPH::PhysicsMaterial* Material = Shape->GetMaterial(SubShapeID);
	if (Material == nullptr || Material == JPH::PhysicsMaterial::sDefault.GetPtr())
	{
		// go default if we got no material
		return SurfaceType_Default;
	}
	return static_cast<const JoltPhysicsMaterial*>(Material)->SurfaceType;
}

JPH::ValidateResult UEJoltCallBackContactListener::OnContactValidate(const JPH::Body& inBody1, const JPH::Body& inBody2, JPH::RVec3Arg inBaseOffset, const JPH::CollideShapeResult& inCollisionResult)
{
	// TODO:
	return ContactListener::OnContactValidate(inBody1, inBody2, inBaseOffset, inCollisionResult);
}

void UEJoltCallBackContactListener::OnContactAdded(const JPH::Body& inBody1, const JPH::Body& inBody2, const JPH::ContactManifold& inManifold, JPH::ContactSettings& ioSettings)
{

	JPH::CollisionEstimationResult result;
	EstimateCollisionResponse(inBody1, inBody2, inManifold, result, ioSettings.mCombinedFriction, ioSettings.mCombinedRestitution);

	const TEnumAsByte<EPhysicalSurface> surface1(ResolveContactSurface(inBody1, inManifold.mSubShapeID1));
	const TEnumAsByte<EPhysicalSurface> surface2(ResolveContactSurface(inBody2, inManifold.mSubShapeID2));

	for (uint8 i = 0; const float& contactImpulse : result.mContactImpulse)
	{
		const JPH::RVec3 contactPoint1 = inManifold.GetWorldSpaceContactPointOn1(i);
		const JPH::RVec3 contactPoint2 = inManifold.GetWorldSpaceContactPointOn2(i);

		Queue.Enqueue(
			FContactInfo(
				inBody1.GetID().GetIndexAndSequenceNumber(),
				inBody2.GetID().GetIndexAndSequenceNumber(),
				JoltHelpers::ToUEPos(contactPoint1),
				JoltHelpers::ToUEPos(contactPoint2),
				JoltHelpers::ToUESize(contactImpulse),
				JoltHelpers::ToUESize(inManifold.mWorldSpaceNormal, false),
				surface1,
				surface2,
				JoltHelpers::ToUESize(inBody1.GetPointVelocity(contactPoint1)),
				JoltHelpers::ToUESize(inBody2.GetPointVelocity(contactPoint2)))

		);

		i++;
	}
}

void UEJoltCallBackContactListener::OnContactPersisted(const JPH::Body& inBody1, const JPH::Body& inBody2, const JPH::ContactManifold& inManifold, JPH::ContactSettings& ioSettings)
{
	if (inManifold.mRelativeContactPointsOn1.empty())
	{
		return;
	}

	const JPH::RVec3 contactPoint1 = inManifold.GetWorldSpaceContactPointOn1(0);
	const JPH::RVec3 contactPoint2 = inManifold.GetWorldSpaceContactPointOn2(0);

	const JPH::Vec3 pointVel1 = inBody1.GetPointVelocity(contactPoint1);
	const JPH::Vec3 pointVel2 = inBody2.GetPointVelocity(contactPoint2);

	// Skip resting contacts — only sliding contacts produce scrape FX.
	const JPH::Vec3 relVel = pointVel1 - pointVel2;
	const JPH::Vec3 tangentialVel = relVel - inManifold.mWorldSpaceNormal * relVel.Dot(inManifold.mWorldSpaceNormal);
	if (tangentialVel.LengthSq() < ScrapePersistMinTangentialSpeedSq)
	{
		return;
	}

	const TEnumAsByte<EPhysicalSurface> surface1(ResolveContactSurface(inBody1, inManifold.mSubShapeID1));
	const TEnumAsByte<EPhysicalSurface> surface2(ResolveContactSurface(inBody2, inManifold.mSubShapeID2));

	Queue.Enqueue(
		FContactInfo(
			inBody1.GetID().GetIndexAndSequenceNumber(),
			inBody2.GetID().GetIndexAndSequenceNumber(),
			JoltHelpers::ToUEPos(contactPoint1),
			JoltHelpers::ToUEPos(contactPoint2),
			0.f, // no impulse for a persisted/resting contact
			JoltHelpers::ToUESize(inManifold.mWorldSpaceNormal, false),
			surface1,
			surface2,
			JoltHelpers::ToUESize(pointVel1),
			JoltHelpers::ToUESize(pointVel2),
			EContactPhase::Persisted));
}

void UEJoltCallBackContactListener::OnContactRemoved(const JPH::SubShapeIDPair& inSubShapePair) {
	// TODO:
};
