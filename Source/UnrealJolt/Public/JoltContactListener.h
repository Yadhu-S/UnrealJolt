#pragma once

#include "Containers/Queue.h"
#include "UnrealJolt/JoltMain.h"
#include "UObject/ObjectMacros.h"

#include "JoltContactListener.generated.h"

USTRUCT()
struct FContactInfo
{
	GENERATED_USTRUCT_BODY();

	FContactInfo() = default;

	FContactInfo(int32 BodyID1, int32 BodyID2, const FVector& BodyID1ContactLocation, const FVector& BodyID2ContactLocation)
		: BodyID1(BodyID1)
		, BodyID2(BodyID2)
		, BodyID1ContactLocation(BodyID1ContactLocation)
		, BodyID2ContactLocation(BodyID2ContactLocation) {}

	int32 BodyID1;

	int32 BodyID2;

	FVector BodyID1ContactLocation;

	FVector BodyID2ContactLocation;
};

class UEJoltCallBackContactListener : public JPH::ContactListener
{

public:
	virtual JPH::ValidateResult OnContactValidate(const JPH::Body& inBody1, const JPH::Body& inBody2, JPH::RVec3Arg inBaseOffset, const JPH::CollideShapeResult& inCollisionResult) override;

	virtual void OnContactAdded(const JPH::Body& inBody1, const JPH::Body& inBody2, const JPH::ContactManifold& inManifold, JPH::ContactSettings& ioSettings) override;

	virtual void OnContactPersisted(const JPH::Body& inBody1, const JPH::Body& inBody2, const JPH::ContactManifold& inManifold, JPH::ContactSettings& ioSettings) override;

	virtual void OnContactRemoved(const JPH::SubShapeIDPair& inSubShapePair) override;

	bool Consume(FContactInfo& OutItem)
	{
		return Queue.Dequeue(OutItem);
	}

private:
	TQueue<FContactInfo, EQueueMode::Mpsc> Queue = TQueue<FContactInfo, EQueueMode::Mpsc>();
};
