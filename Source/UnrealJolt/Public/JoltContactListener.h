#pragma once

#include "Chaos/ChaosEngineInterface.h"
#include "Containers/Queue.h"
#include "UnrealJolt/JoltMain.h"
#include "UObject/ObjectMacros.h"

#include "JoltContactListener.generated.h"

UENUM()
enum class EContactPhase : uint8
{
	Added,
	Persisted,
};

USTRUCT(BlueprintType)
struct FContactInfo
{
	GENERATED_USTRUCT_BODY();

	FContactInfo() = default;

	FContactInfo(int32 BodyID1, int32 BodyID2, const FVector& BodyID1ContactLocation, const FVector& BodyID2ContactLocation, float NormalImpulse, FVector NormalDir, TEnumAsByte<EPhysicalSurface> Surface1 = SurfaceType_Default, TEnumAsByte<EPhysicalSurface> Surface2 = SurfaceType_Default, const FVector& LinearVelocity1 = FVector::ZeroVector, const FVector& LinearVelocity2 = FVector::ZeroVector, EContactPhase Phase = EContactPhase::Added)
		: BodyID1(BodyID1)
		, BodyID2(BodyID2)
		, BodyID1ContactLocation(BodyID1ContactLocation)
		, BodyID2ContactLocation(BodyID2ContactLocation)
		, NormalImpulse(NormalImpulse)
		, NormalDir(NormalDir)
		, Surface1(Surface1)
		, Surface2(Surface2)
		, LinearVelocity1(LinearVelocity1)
		, LinearVelocity2(LinearVelocity2)
		, Phase(Phase) {}

	UPROPERTY(BlueprintReadOnly, Category = "Jolt Physics")
	int32 BodyID1 = 0;

	UPROPERTY(BlueprintReadOnly, Category = "Jolt Physics")
	int32 BodyID2 = 0;

	UPROPERTY(BlueprintReadOnly, Category = "Jolt Physics")
	FVector BodyID1ContactLocation = FVector::ZeroVector;

	UPROPERTY(BlueprintReadOnly, Category = "Jolt Physics")
	FVector BodyID2ContactLocation = FVector::ZeroVector;

	UPROPERTY(BlueprintReadOnly, Category = "Jolt Physics")
	float NormalImpulse = 0.f;

	UPROPERTY(BlueprintReadOnly, Category = "Jolt Physics")
	FVector NormalDir = FVector::ZeroVector;

	// Surface type of body1 at the contact sub-shape — resolved from the
	// Jolt JoltPhysicsMaterial bound to that shape (originating from the UE
	// UPhysicalMaterial assigned to the body setup). Defaults to
	// SurfaceType_Default if the shape has no material.
	UPROPERTY(BlueprintReadOnly, Category = "Jolt Physics")
	TEnumAsByte<EPhysicalSurface> Surface1 = SurfaceType_Default;

	// Surface type of body2 at the contact sub-shape. Same source as Surface1.
	UPROPERTY(BlueprintReadOnly, Category = "Jolt Physics")
	TEnumAsByte<EPhysicalSurface> Surface2 = SurfaceType_Default;

	UPROPERTY(BlueprintReadOnly, Category = "Jolt Physics")
	FVector LinearVelocity1 = FVector::ZeroVector;

	UPROPERTY(BlueprintReadOnly, Category = "Jolt Physics")
	FVector LinearVelocity2 = FVector::ZeroVector;

	UPROPERTY(BlueprintReadOnly, Category = "Jolt Physics")
	EContactPhase Phase = EContactPhase::Added;
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
