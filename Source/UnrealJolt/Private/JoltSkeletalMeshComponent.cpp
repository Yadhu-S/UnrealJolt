// Fill out your copyright notice in the Description page of Project Settings.

#include "JoltSkeletalMeshComponent.h"
#include "Engine/World.h"
#include "UnrealJolt/Helpers.h"
#include "Misc/AssertionMacros.h"
#include "PhysicsEngine/PhysicsAsset.h"
#include "PhysicsEngine/SkeletalBodySetup.h"

void UJoltSkeletalMeshComponent::AddOwnPhysicsAsset()
{
	if (IsSimulatingPhysics()) // Chaos physics, disable it
	{
		UE_LOG(JoltSubSystemLogs, Warning, TEXT("UJoltSkeletalMeshComponent::AddOwnPhysicsAsset: 'Simulate Physics'"));
		SetSimulatePhysics(false);
	}

	if (JoltSubSystem == nullptr)
	{
		UE_LOG(JoltSubSystemLogs, Warning, TEXT("UJoltSkeletalMeshComponent::AddOwnPhysicsAsset: jolt subsystem empty"));
		return;
	}

	UPhysicsAsset* PhysicsAsset = GetPhysicsAsset();
	if (!PhysicsAsset)
	{
		UE_LOG(JoltSubSystemLogs, Warning, TEXT("UJoltSkeletalMeshComponent::AddOwnPhysicsAsset: could not get physics asset, configure physics asset in your editor"));
		return;
	}

	/*
	 * We are assigning a an ID regardless of the body being in the physics simulation or not
	 * Might not be a good idea to waste body ids if we need to simulate lots of bodies
	 * This will do for now.
	 * Also, we are assuming that every body is either dynamic or kinematic
	 */
	if (OwnBodyID.IsInvalid())
	{
		JoltSubSystem->DynamicBodyIDX++;
		OwnBodyID = JPH::BodyID(JoltSubSystem->DynamicBodyIDX);
		UE_LOG(JoltSubSystemLogs, Log, TEXT("Using dynamically generated BodyID: %d"), OwnBodyID.GetIndexAndSequenceNumber());
	}

	std::function<void(const JPH::Shape*, const FTransform&)> callback = [this](const JPH::Shape* shape, const FTransform& transform) {
		JoltSubSystem->SkeletalMeshBodyIDLocalTransformMap.Add(&OwnBodyID, VisualOffset);
		if (!bSimulatePhysics)
		{
			return;
		}

		if (shape == nullptr)
		{
			UE_LOG(JoltSubSystemLogs, Error, TEXT("Invalid or unsupported shape configured for sekeltalmesh: %s, owner: %s"), *GetName(), *GetOwner()->GetName());
			return;
		}

		if (!CentreOfMassOffset.IsZero())
		{
			shape = new JPH::OffsetCenterOfMassShape(shape, JoltHelpers::ToJoltVec3(CentreOfMassOffset));
		}

		bIsKinematicBody ? JoltSubSystem->AddKinematicBodyCollision(OwnBodyID, shape, this->GetOwner()->GetActorTransform(), Friction, Restitution, Mass)
						 : JoltSubSystem->AddDynamicBodyCollision(OwnBodyID, shape, this->GetOwner()->GetActorTransform(), Friction, Restitution, Mass);

		if (bIncludeInSnapshot)
		{
			Filter = new SaveStateFilter();
			Filter->AddToBodyIDAllowList(OwnBodyID);
		}

		JoltSubSystem->DynamicBodyIDActorMap.Add(&OwnBodyID, GetOwner());
		UE_LOG(JoltSubSystemLogs, Log, TEXT("UJoltSkeletalMeshComponent::AddOwnPhysicsAsset: done setting up own rigid body"));
	};

	for (const USkeletalBodySetup* skeletalBodySetup : PhysicsAsset->SkeletalBodySetups)
	{
		JoltSubSystem->ExtractPhysicsGeometry(GetOwner()->GetActorTransform(), skeletalBodySetup, callback);
	}
}

void UJoltSkeletalMeshComponent::SaveState(TArray<uint8>& serverPhysicsState)
{
	if (Filter == nullptr)
	{
		return;
	}
	JoltSubSystem->SaveState(serverPhysicsState, Filter);
}

void UJoltSkeletalMeshComponent::TickComponent(float DeltaTime, enum ELevelTick TickType, FActorComponentTickFunction* ThisTickFunction)
{
	Super::TickComponent(DeltaTime, TickType, ThisTickFunction);
}

void UJoltSkeletalMeshComponent::BeginPlay()
{
	Super::BeginPlay();

	if (bManualInitialization)
	{
		UE_LOG(JoltSubSystemLogs, Log, TEXT("Using manual initialization. Set BodyID and call LoadJoltSubsytem() manually"));
		return;
	}

	UJoltSubsystem* joltSubSystem = GetWorld()->GetSubsystem<UJoltSubsystem>();
	if (joltSubSystem == nullptr)
	{
		UE_LOG(JoltSubSystemLogs, Warning, TEXT("joltSubSystem null, please manually call LoadJoltSubsytem()"));
		return;
	}
	LoadJoltSubsystem(joltSubSystem);
}

void UJoltSkeletalMeshComponent::LoadJoltSubsystem(UJoltSubsystem* joltSubsystem)
{
	JoltSubSystem = joltSubsystem;
	AddOwnPhysicsAsset();
}

void UJoltSkeletalMeshComponent::JoltSetLinearAndAngularVelocity(const FVector& velocity, const FVector& angularVelocity) const
{
	check(JoltSubSystem != nullptr);
	JoltSubSystem->JoltSetLinearAndAngularVelocity(OwnBodyID, velocity, angularVelocity);
}

void UJoltSkeletalMeshComponent::JoltSetLinearVelocity(const FVector& velocity) const
{
	check(JoltSubSystem != nullptr);
	JoltSubSystem->JoltSetLinearVelocity(OwnBodyID, velocity);
}

void UJoltSkeletalMeshComponent::JoltSetPhysicsLocationAndRotation(const FVector& locationWS, const FQuat& rotationWS) const
{
	check(JoltSubSystem != nullptr);
	JoltSubSystem->JoltSetPhysicsLocationAndRotation(OwnBodyID, (locationWS), (rotationWS));
}

void UJoltSkeletalMeshComponent::JoltSetPhysicsLocation(const FVector& locationWS) const
{
	check(JoltSubSystem != nullptr);
	JoltSubSystem->JoltSetPhysicsLocation(OwnBodyID, locationWS);
}

void UJoltSkeletalMeshComponent::JoltSetPhysicsRotation(const FQuat& rotationWS) const
{
	check(JoltSubSystem != nullptr);
	JoltSubSystem->JoltSetPhysicsRotation(OwnBodyID, rotationWS);
}

FVector UJoltSkeletalMeshComponent::JoltGetVelocityAt(const FVector& locationWS) const
{
	return JoltSubSystem->JoltGetVelocityAt(OwnBodyID, locationWS);
}

void UJoltSkeletalMeshComponent::JoltAddForceAtLocation(const FVector& force, const FVector& locationWS) const
{
	check(JoltSubSystem != nullptr);
	JoltSubSystem->JoltAddForceAtLocation(OwnBodyID, force, locationWS);
}

void UJoltSkeletalMeshComponent::JoltAddImpulseLocation(const FVector& force, const FVector& locationWS) const
{
	check(JoltSubSystem != nullptr);
	JoltSubSystem->JoltAddImpulseAtLocation(OwnBodyID, force, locationWS);
}

void UJoltSkeletalMeshComponent::JoltAddCentralForce(const FVector& force) const
{
	check(JoltSubSystem != nullptr);
	JoltSubSystem->JoltAddForce(OwnBodyID, force);
}

void UJoltSkeletalMeshComponent::JoltGetPhysicsState(FTransform& transform, FTransform& transformCOM, FVector& velocity, FVector& angularVelocity) const
{
	check(JoltSubSystem != nullptr);
	JoltSubSystem->JoltGetPhysicsState(OwnBodyID, transform, transformCOM, velocity, angularVelocity);
}

void UJoltSkeletalMeshComponent::JoltAddCentralImpulse(const FVector& impulse) const
{
	check(JoltSubSystem != nullptr);
	JoltSubSystem->JoltAddCentralImpulse(OwnBodyID, impulse);
}

void UJoltSkeletalMeshComponent::JoltAddTorque(const FVector& torque) const
{
	check(JoltSubSystem != nullptr);
	JoltSubSystem->JoltAddTorque(OwnBodyID, torque);
}

void UJoltSkeletalMeshComponent::JoltReadPhysicsTransform(FTransform& outTransform) const
{
	check(JoltSubSystem != nullptr);
	JoltSubSystem->JoltGetPhysicsTransform(OwnBodyID, outTransform);
}

void UJoltSkeletalMeshComponent::JoltSetVisualTransform(FTransform& joltTransformWS)
{
	JoltSubSystem->ApplyLocalTxIfAny(&OwnBodyID, joltTransformWS);
	SetWorldTransform(joltTransformWS);
}
