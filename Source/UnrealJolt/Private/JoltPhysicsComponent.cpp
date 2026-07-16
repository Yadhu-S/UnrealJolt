#include "JoltPhysicsComponent.h"
#include "JoltSubsystem.h"
#include "PhysicsEngine/BodySetup.h"
#include "UnrealJolt/JoltMain.h"

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

	TArray<USceneComponent*> SceneComponents;
	GetOwner()->GetComponents<USceneComponent>(SceneComponents);

	for (USceneComponent* SceneComponent : SceneComponents)
	{
		if (!SceneComponent) continue;
		
		// Dynamic bodies have to be movable, static bodies can be either.
		if (Mobility == EJoltMobility::Dynamic)
			SceneComponent->SetMobility(EComponentMobility::Movable);

		// This is also set in ExtractPhysicsGeometry, but I felt it was good practice to set it here too. 
		if (UPrimitiveComponent* PrimitiveComponent = Cast<UPrimitiveComponent>(SceneComponent))
			PrimitiveComponent->SetSimulatePhysics(false);
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
		BodyID = Mobility == EJoltMobility::Static ?
		   JoltSubsystem->AddStaticBody(GetOwner(), Friction, Restitution, ResolvedLayer) :
		   JoltSubsystem->AddDynamicBody(GetOwner(), Friction, Restitution, Mass, ResolvedLayer);
	}
}

void UJoltPhysicsComponent::OnRegister()
{
	Super::OnRegister();

	#if WITH_EDITOR
	if (!bOverrideMass) RecalculateMass();
	#endif
}

bool UJoltPhysicsComponent::GetBodyID(int& OutBodyID) const
{
	if (BodyID == -1) return false;
	OutBodyID = BodyID;
	return true;
}

void UJoltPhysicsComponent::SetFriction(const float NewFriction)
{
	if (!GetOwner()) return;

	if (const UJoltSubsystem* JoltSubsystem = GetJoltSubsystem(GetOwner()))
	{
		Friction = NewFriction;
		JoltSubsystem->GetBodyInterface()->SetFriction(JPH::BodyID(BodyID), Friction);
	}
}

void UJoltPhysicsComponent::SetRestitution(float NewRestitution)
{
	if (!GetOwner()) return;

	if (const UJoltSubsystem* JoltSubsystem = GetJoltSubsystem(GetOwner()))
	{
		Restitution = NewRestitution;
		JoltSubsystem->GetBodyInterface()->SetRestitution(JPH::BodyID(BodyID), Restitution);
	}
}

void UJoltPhysicsComponent::SetMass(float NewMass)
{
	if (!GetOwner()) return;

	if (const UJoltSubsystem* JoltSubsystem = GetJoltSubsystem(GetOwner()))
	{
		Mass = NewMass;
		JoltSubsystem->JoltSetMass(BodyID, NewMass);
	}
}

TArray<FString> UJoltPhysicsComponent::GetObjectLayerNames() const
{
	const UJoltSettings* Settings = GetDefault<UJoltSettings>();
	if (!Settings) return {};

	const FName ThisMobilityDefault = (Mobility == EJoltMobility::Static) ? Settings->DefaultStaticLayer : Settings->DefaultDynamicLayer;
	const FName OtherMobilityDefault = (Mobility == EJoltMobility::Static) ? Settings->DefaultDynamicLayer : Settings->DefaultStaticLayer;

	TArray<FString> Names;
	Names.Reserve(Settings->ObjectLayers.Num());
	for (const FJoltObjectLayer& ObjLayer : Settings->ObjectLayers)
	{
		// Hide the other mobility's default layer. That way "Static" doesn't show up when mobility is "Dynamic", and vice-versa.
		if (ObjLayer.Name.IsNone() || ObjLayer.Name == OtherMobilityDefault) continue;
		
		// Show default instead of the real layer name. It'll always reflect the sentinel value, so it won't go stale if we change the default layer.
		Names.Add(ObjLayer.Name == ThisMobilityDefault ? DefaultLayerSentinel.ToString() : ObjLayer.Name.ToString());
	}
	return Names;
}

FName UJoltPhysicsComponent::ResolveLayer() const
{
	if (Layer != DefaultLayerSentinel) return Layer;

	const UJoltSettings* Settings = GetDefault<UJoltSettings>();
	if (!Settings) return Layer;

	return (Mobility == EJoltMobility::Static) ? Settings->DefaultStaticLayer : Settings->DefaultDynamicLayer;
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

		if (UBodySetup* BodySetup = PrimitiveComponent->GetBodySetup())
		{
			const float ComputedMass = BodySetup->CalculateMass(PrimitiveComponent);
			
			if (ComputedMass > 0.0f)
			{
				TotalMass += ComputedMass;
			}
		}
	}

	if (TotalMass > 0.0f)
	{
		Mass = TotalMass;
	}
}

#if WITH_EDITOR
void UJoltPhysicsComponent::PostEditChangeProperty(FPropertyChangedEvent& PropertyChangedEvent)
{
	Super::PostEditChangeProperty(PropertyChangedEvent);

	if (PropertyChangedEvent.Property == nullptr) return;

	const FName ChangedProp = PropertyChangedEvent.Property->GetFName();

	if (ChangedProp == GET_MEMBER_NAME_CHECKED(UJoltPhysicsComponent, Mobility))
	{
		Layer = DefaultLayerSentinel;

		if (Mobility == EJoltMobility::Static)
		{
			bOverrideMass = false;
			Mass = 0.f;
		} else {
			RecalculateMass();
		}

	}
	else if (ChangedProp == GET_MEMBER_NAME_CHECKED(UJoltPhysicsComponent, bOverrideMass))
	{
		RecalculateMass();
	}
}
#endif