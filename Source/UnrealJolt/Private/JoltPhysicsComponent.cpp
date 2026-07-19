#include "JoltPhysicsComponent.h"
#include "JoltSubsystem.h"
#include "PhysicsEngine/BodySetup.h"
#include "UnrealJolt/JoltMain.h"

DEFINE_LOG_CATEGORY(LogJoltPhysicsComponent);

/** Gets the Jolt physics component and subsystem for an actor, validates BodyID, returns early if any are invalid. */
#define JOLT_GET_COMPONENT_AND_SUBSYSTEM(ReturnValue) \
if (!Actor) { UE_LOG(LogJoltPhysicsComponent, Error, \
TEXT("%hs: Actor not found"), __FUNCTION__); return ReturnValue; } \
UJoltPhysicsComponent* Component = Actor->GetComponentByClass<UJoltPhysicsComponent>(); \
if (!Component) { UE_LOG(LogJoltPhysicsComponent, Error, \
TEXT("%hs: Component not found on %s - Jolt Physics setters can only be called on actors with a Jolt Physics Component") \
, __FUNCTION__, *Actor->GetName()); return ReturnValue; } \
if (Component->BodyID == JPH::BodyID::cInvalidBodyID) return ReturnValue; \
JPH::BodyID BodyID = JPH::BodyID(Component->BodyID); \
UJoltSubsystem* Subsystem = GetJoltSubsystem(Actor); \
if (!Subsystem) { UE_LOG(LogJoltPhysicsComponent, Error, \
TEXT("%hs: Subsystem not found for %s") \
, __FUNCTION__, *Actor->GetName()); return ReturnValue; }

namespace
{
	// Resolves to the default static/dynamic layer at runtime. Just less messy than having both Static and Dynamic as options for Layer.
	const FName DefaultLayerSentinel(TEXT("Default"));
}

UJoltPhysicsComponent::UJoltPhysicsComponent()
{
	PrimaryComponentTick.bCanEverTick = false;
}

void UJoltPhysicsComponent::BeginPlay()
{
	Super::BeginPlay();
    
	if (!GetOwner()) return;

	TArray<UStaticMeshComponent*> StaticMeshComponents;
	GetOwner()->GetComponents<UStaticMeshComponent>(StaticMeshComponents);

	bool bFoundValidGeometry = false;
	
	for (UStaticMeshComponent* StaticMeshComponent : StaticMeshComponents)
	{
		if (!StaticMeshComponent) continue;
		
		if (!StaticMeshComponent->GetStaticMesh())
		{
			UE_LOG(LogJoltPhysicsComponent, Warning,
				TEXT("%hs: Static mesh component %s has no valid static mesh"), 
				__FUNCTION__, *StaticMeshComponent->GetName());
			continue;
		}
		
		// Dynamic bodies have to be movable, static bodies can be either.
		if (MotionType == EJoltMotionType::Dynamic)
		{
			if (StaticMeshComponent->GetMobility() != EComponentMobility::Movable)
			{
				UE_LOG(LogJoltPhysicsComponent, Warning,
					TEXT("%hs: Component mobility for %s is movable, when %s's Jolt Physics Component is set to dynamic motion type, setting mobility to movable"), 
					__FUNCTION__, *StaticMeshComponent->GetName(), *GetOwner()->GetName());
				StaticMeshComponent->SetMobility(EComponentMobility::Movable);
			}
		}

		// Bodies need to have simulate physics turned off. 
		if (UPrimitiveComponent* PrimitiveComponent = Cast<UPrimitiveComponent>(StaticMeshComponent))
		{
			if (PrimitiveComponent->IsSimulatingPhysics())
			{
				UE_LOG(LogJoltPhysicsComponent, Warning,
					TEXT("%hs: 'Simulate physics' turned on for component %s, disabling chaos"), 
					__FUNCTION__, *PrimitiveComponent->GetName());
				PrimitiveComponent->SetSimulatePhysics(false);
			}
		}
		
		bFoundValidGeometry = true;
	}
	
	if (!bFoundValidGeometry)
	{
		UE_LOG(LogJoltPhysicsComponent, Error,
			TEXT("%hs: %s has no valid geometry to simulate via Jolt Physics"), __FUNCTION__, *GetOwner()->GetName());
		return;
	}
		
	if (UJoltSubsystem* JoltSubsystem = GetJoltSubsystem(GetOwner()))
	{
		const FName ResolvedLayer = ResolveLayer();
		RecalculateMass();
		
		// The benefit of the tagging system is that it sorts the actors after they're iterated -
		// this way, each actor will be added in the same order everytime.
		// The execution order of BeginPlay across actors isn't guaranteed, so we don't have the same luxury.
		// If it causes issues with determinism, we can loop through all jolt physics components in AddAllJoltActors, and sort them there.
		// For now though, this should be okay... hopefully.
		
		BodyID = MotionType == EJoltMotionType::Static ?
		   JoltSubsystem->AddStaticBody(GetOwner(), Friction, Restitution, ResolvedLayer) :
		   JoltSubsystem->AddDynamicBody(GetOwner(), Friction, Restitution, Mass, ResolvedLayer);
		
		if (BodyID == JPH::BodyID::cInvalidBodyID)
		{
			UE_LOG(LogJoltPhysicsComponent, Error,
				TEXT("%hs: Failed to create body on %s - Ensure actor has valid geometry")
				, __FUNCTION__, *GetOwner()->GetName());
			return;
		}
		
		const JPH::BodyID& Body = JPH::BodyID(BodyID);
		
		// Not a massive fan of this, I feel like it would be more appropriate to pass it through AddStatic/DynamicBody.
		// Maybe pass a struct through them instead of a billion parameters - but that would be a decently sized change. 
		if (MotionType != EJoltMotionType::Static)
		{
			JoltSubsystem->JoltSetAllowedDOFs(Body, AllowedDOFs);
			JoltSubsystem->JoltSetGravityFactor(Body, GravityFactor);
			JoltSubsystem->JoltSetApplyGyroscopicForce(Body, bApplyGyroscopicForce);
			JoltSubsystem->JoltSetMaxLinearVelocity(Body, MaxLinearVelocity);
			JoltSubsystem->JoltSetMaxAngularVelocity(Body, MaxAngularVelocity);
			JoltSubsystem->JoltSetLinearDamping(Body, LinearDamping);
			JoltSubsystem->JoltSetAngularDamping(Body, AngularDamping);
			JoltSubsystem->JoltSetAllowSleeping(Body, bAllowSleeping);

			// 0 means unset, so skip the override and let Jolt use its default
			if (NumVelocityStepsOverride != 0) JoltSubsystem->JoltSetNumVelocityStepsOverride(Body, NumVelocityStepsOverride);
			if (NumPositionStepsOverride != 0) JoltSubsystem->JoltSetNumPositionStepsOverride(Body, NumPositionStepsOverride);
		}
		
		JoltSubsystem->JoltSetEnhancedInternalEdgeRemoval(Body, bEnhancedInternalEdgeRemoval);
	}
}

void UJoltPhysicsComponent::OnRegister()
{
	Super::OnRegister();

	#if WITH_EDITOR
	if (!bOverrideMass) RecalculateMass();
	#endif
}

void UJoltPhysicsComponent::SetObjectLayer(AActor* Actor, FName NewObjectLayer)
{
	JOLT_GET_COMPONENT_AND_SUBSYSTEM()
	
	Component->Layer = NewObjectLayer;
	Subsystem->JoltSetObjectLayer(BodyID, NewObjectLayer);
}

void UJoltPhysicsComponent::SetMass(AActor* Actor, const float NewMass)
{
	JOLT_GET_COMPONENT_AND_SUBSYSTEM()

	Component->Mass = NewMass;
	Subsystem->JoltSetMass(BodyID, NewMass);
}

void UJoltPhysicsComponent::SetGravityFactor(AActor* Actor, float NewGravityFactor)
{
	JOLT_GET_COMPONENT_AND_SUBSYSTEM()

	Component->GravityFactor = NewGravityFactor;
	Subsystem->JoltSetGravityFactor(BodyID, NewGravityFactor);
}

void UJoltPhysicsComponent::SetApplyGyroscopicForce(AActor* Actor, bool bNewApplyGyroscopicForce)
{
	JOLT_GET_COMPONENT_AND_SUBSYSTEM()
	
	Component->bApplyGyroscopicForce = bNewApplyGyroscopicForce;
	Subsystem->JoltSetApplyGyroscopicForce(BodyID, bNewApplyGyroscopicForce);
}

void UJoltPhysicsComponent::SetMaxLinearVelocity(AActor* Actor, float NewMaxLinearVelocity)
{
	JOLT_GET_COMPONENT_AND_SUBSYSTEM()
	
	Component->MaxLinearVelocity = NewMaxLinearVelocity;
	Subsystem->JoltSetMaxLinearVelocity(BodyID, NewMaxLinearVelocity);
}

float UJoltPhysicsComponent::GetMaxLinearVelocity(AActor* Actor)
{
	JOLT_GET_COMPONENT_AND_SUBSYSTEM(-1)

	return Subsystem->JoltGetMaxLinearVelocity(BodyID);
}

void UJoltPhysicsComponent::SetMaxAngularVelocity(AActor* Actor, float NewMaxAngularVelocity)
{
	JOLT_GET_COMPONENT_AND_SUBSYSTEM()

	Component->MaxAngularVelocity = NewMaxAngularVelocity;
	Subsystem->JoltSetMaxAngularVelocity(BodyID, NewMaxAngularVelocity);
}

void UJoltPhysicsComponent::SetFriction(AActor* Actor, const float NewFriction)
{
	JOLT_GET_COMPONENT_AND_SUBSYSTEM()

	Component->Friction = NewFriction;
	Subsystem->JoltSetFriction(BodyID, NewFriction);
}

void UJoltPhysicsComponent::SetRestitution(AActor* Actor, const float NewRestitution)
{
	JOLT_GET_COMPONENT_AND_SUBSYSTEM()

	Component->Restitution = NewRestitution;
	Subsystem->JoltSetRestitution(BodyID, NewRestitution);
}

void UJoltPhysicsComponent::SetLinearDamping(AActor* Actor, float NewLinearDamping)
{
	JOLT_GET_COMPONENT_AND_SUBSYSTEM()

	Component->LinearDamping = NewLinearDamping;
	Subsystem->JoltSetLinearDamping(BodyID, NewLinearDamping);
}

void UJoltPhysicsComponent::SetAngularDamping(AActor* Actor, float NewAngularDamping)
{
	JOLT_GET_COMPONENT_AND_SUBSYSTEM()

	Component->AngularDamping = NewAngularDamping;
	Subsystem->JoltSetAngularDamping(BodyID, NewAngularDamping);
}

void UJoltPhysicsComponent::SetAllowSleeping(AActor* Actor, bool bNewAllowSleeping)
{
	JOLT_GET_COMPONENT_AND_SUBSYSTEM()
	
	Component->bAllowSleeping = bNewAllowSleeping;
	Subsystem->JoltSetAllowSleeping(BodyID, bNewAllowSleeping);
}

void UJoltPhysicsComponent::SetNumVelocityStepsOverride(AActor* Actor, int NewNumVelocityStepsOverride)
{
	JOLT_GET_COMPONENT_AND_SUBSYSTEM()

	Component->NumVelocityStepsOverride = NewNumVelocityStepsOverride;
	Subsystem->JoltSetNumVelocityStepsOverride(BodyID, NewNumVelocityStepsOverride);
}

void UJoltPhysicsComponent::SetNumPositionStepsOverride(AActor* Actor, int NewNumPositionStepsOverride) 
{
	JOLT_GET_COMPONENT_AND_SUBSYSTEM()

	Component->NumPositionStepsOverride = NewNumPositionStepsOverride;
	Subsystem->JoltSetNumPositionStepsOverride(BodyID, NewNumPositionStepsOverride);
}

void UJoltPhysicsComponent::SetEnhancedInternalEdgeRemoval(AActor* Actor, bool bNewEnhancedInternalEdgeRemoval) 
{
	JOLT_GET_COMPONENT_AND_SUBSYSTEM()

	Component->bEnhancedInternalEdgeRemoval = bNewEnhancedInternalEdgeRemoval;
	Subsystem->JoltSetEnhancedInternalEdgeRemoval(BodyID, bNewEnhancedInternalEdgeRemoval);
}

bool UJoltPhysicsComponent::GetBodyID(int& OutBodyID) const
{
	if (BodyID == JPH::BodyID::cInvalidBodyID) return false;
	OutBodyID = BodyID;
	return true;
}

TArray<FString> UJoltPhysicsComponent::GetObjectLayerNames() const
{
	const UJoltSettings* Settings = GetDefault<UJoltSettings>();
	if (!Settings) return {};

	const FName ThisMotionTypeDefault = (MotionType == EJoltMotionType::Static) ? Settings->DefaultStaticLayer : Settings->DefaultDynamicLayer;
	const FName OtherMotionTypeDefault = (MotionType == EJoltMotionType::Static) ? Settings->DefaultDynamicLayer : Settings->DefaultStaticLayer;

	TArray<FString> Names;
	Names.Reserve(Settings->ObjectLayers.Num());
	for (const FJoltObjectLayer& ObjLayer : Settings->ObjectLayers)
	{
		// Hide the other MotionType's default layer. That way "Static" doesn't show up when MotionType is "Dynamic", and vice-versa.
		if (ObjLayer.Name.IsNone() || ObjLayer.Name == OtherMotionTypeDefault) continue;
		
		// Show default instead of the real layer name. It'll always reflect the sentinel value, so it won't go stale if we change the default layer.
		Names.Add(ObjLayer.Name == ThisMotionTypeDefault ? DefaultLayerSentinel.ToString() : ObjLayer.Name.ToString());
	}
	return Names;
}

FName UJoltPhysicsComponent::ResolveLayer() const
{
	if (Layer != DefaultLayerSentinel) return Layer;

	const UJoltSettings* Settings = GetDefault<UJoltSettings>();
	if (!Settings) return Layer;

	return (MotionType == EJoltMotionType::Static) ? Settings->DefaultStaticLayer : Settings->DefaultDynamicLayer;
}

void UJoltPhysicsComponent::RecalculateMass()
{
	// bOverrideMass means the user is hand-authoring mass.
	if (bOverrideMass || !GetOwner()) return;

	TArray<UPrimitiveComponent*> PrimitiveComponents;
	GetOwner()->GetComponents<UPrimitiveComponent>(PrimitiveComponents);

	float TotalMass = 0.0f;
	for (UPrimitiveComponent* PrimitiveComponent : PrimitiveComponents)
	{
		if (!PrimitiveComponent) continue;

		if (const UBodySetup* BodySetup = PrimitiveComponent->GetBodySetup())
		{
			if (const float ComputedMass = BodySetup->CalculateMass(PrimitiveComponent); ComputedMass > 0.0f)
			{
				TotalMass += ComputedMass;
			}
		}
	}

	if (TotalMass > 0.0f) 
		Mass = TotalMass;
	else 
		UE_LOG(LogJoltPhysicsComponent, Error,
		TEXT("%hs: %s has a zero or negative mass - Ensure actor has valid geometry"), __FUNCTION__, *GetOwner()->GetName());
}

#if WITH_EDITOR
void UJoltPhysicsComponent::PostEditChangeProperty(FPropertyChangedEvent& PropertyChangedEvent)
{
	Super::PostEditChangeProperty(PropertyChangedEvent);

	if (PropertyChangedEvent.Property == nullptr) return;

	const FName ChangedProp = PropertyChangedEvent.Property->GetFName();

	if (ChangedProp == GET_MEMBER_NAME_CHECKED(UJoltPhysicsComponent, MotionType))
	{
		// Static/Dynamic have different default layers, so fall back to the sentinel
		Layer = DefaultLayerSentinel;
	}
	else if (ChangedProp == GET_MEMBER_NAME_CHECKED(UJoltPhysicsComponent, bOverrideMass))
	{
		RecalculateMass();
	}
}
#endif