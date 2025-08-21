#include "JoltContactListener.h"
#include "UnrealJolt/Helpers.h"

JPH::ValidateResult UEJoltCallBackContactListener::OnContactValidate(const JPH::Body& inBody1, const JPH::Body& inBody2, JPH::RVec3Arg inBaseOffset, const JPH::CollideShapeResult& inCollisionResult)
{
	return ContactListener::OnContactValidate(inBody1, inBody2, inBaseOffset, inCollisionResult);
}

void UEJoltCallBackContactListener::OnContactAdded(const JPH::Body& inBody1, const JPH::Body& inBody2, const JPH::ContactManifold& inManifold, JPH::ContactSettings& ioSettings)
{
	FContactInfo con = FContactInfo(inBody1.GetID().GetIndexAndSequenceNumber(), inBody2.GetID().GetIndexAndSequenceNumber(), JoltHelpers::ToUEPos(inManifold.GetWorldSpaceContactPointOn1(0)), JoltHelpers::ToUEPos(inManifold.GetWorldSpaceContactPointOn2(0)));
	Queue.Enqueue(con);
}

void UEJoltCallBackContactListener::OnContactPersisted(const JPH::Body& inBody1, const JPH::Body& inBody2, const JPH::ContactManifold& inManifold, JPH::ContactSettings& ioSettings)
{
	// return ContactListener::OnContactPersisted(inBody1, inBody2, inManifold, ioSettings);
}

void UEJoltCallBackContactListener::OnContactRemoved(const JPH::SubShapeIDPair& inSubShapePair) {
};
