#pragma once

#include "Containers/Queue.h"
#include "UnrealJolt/JoltMain.h"
#include "UObject/ObjectMacros.h"

#include "JoltContactListener.generated.h"

USTRUCT(BlueprintType)
struct FContactInfo
{
	GENERATED_USTRUCT_BODY();

	FContactInfo() = default;

	FContactInfo(int32 BodyID1, int32 BodyID2, const FVector& BodyID1ContactLocation, const FVector& BodyID2ContactLocation, float NormalImpulse, FVector NormalDir)
		: BodyID1(BodyID1)
		, BodyID2(BodyID2)
		, BodyID1ContactLocation(BodyID1ContactLocation)
		, BodyID2ContactLocation(BodyID2ContactLocation)
		, NormalImpulse(NormalImpulse)
		, NormalDir(NormalDir) {}

	int32 BodyID1;

	int32 BodyID2;

	FVector BodyID1ContactLocation;

	FVector BodyID2ContactLocation;

	float NormalImpulse;

	FVector NormalDir;
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

	TQueue<FContactInfo, EQueueMode::Mpsc>* GetContactQueue() { return &Queue; };

private:
	TQueue<FContactInfo, EQueueMode::Mpsc> Queue = TQueue<FContactInfo, EQueueMode::Mpsc>();
};
