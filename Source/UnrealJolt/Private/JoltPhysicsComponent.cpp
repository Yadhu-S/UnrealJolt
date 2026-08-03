#include "JoltPhysicsComponent.h"
#include "JoltSubsystem.h"
#include "PhysicsEngine/BodySetup.h"
#include "UnrealJolt/JoltMain.h"

DEFINE_LOG_CATEGORY(LogJoltPhysicsComponent);

static const FName DefaultLayerSentinel(TEXT("Default"));

UJoltPhysicsComponent::UJoltPhysicsComponent()
{
	PrimaryComponentTick.bCanEverTick = false;
}

void UJoltPhysicsComponent::OnComponentCreated()
{
	Super::OnComponentCreated();

	SortID = FGuid::NewGuid();
}

void UJoltPhysicsComponent::OnRegister()
{
	Super::OnRegister();

#if WITH_EDITOR
	// Transient preview only. Writing an authored UPROPERTY here would be unsafe: registration also runs
	// during PIE duplication/restore, before per-instance overrides land, so bOverrideMass can still read
	// false and an authored Mass would be silently overwritten.
	ComputedMass = ComputeAutoMass();
#endif
}

void UJoltPhysicsComponent::PostLoad()
{
	Super::PostLoad();

#if WITH_EDITOR
	if (!SortID.IsValid())
	{
		SortID = FGuid::NewGuid();
		MarkPackageDirty();
	}
#endif
}

void UJoltPhysicsComponent::PostDuplicate(bool bDuplicateForPIE)
{
	Super::PostDuplicate(bDuplicateForPIE);

#if WITH_EDITOR
	if (!bDuplicateForPIE)
	{
		SortID = FGuid::NewGuid();
		MarkPackageDirty();
	}
#endif
}

#if WITH_EDITOR
void UJoltPhysicsComponent::PostEditImport()
{
	SortID = FGuid::NewGuid();
	MarkPackageDirty();
}
#endif

void UJoltPhysicsComponent::BeginPlay()
{
	Super::BeginPlay();
	CreateBody();
}

void UJoltPhysicsComponent::EndPlay(const EEndPlayReason::Type EndPlayReason)
{
	Super::EndPlay(EndPlayReason);

	if (BodyID == JPH::BodyID::cInvalidBodyID)
		return;

	if (JoltSubsystem)
		JoltSubsystem->RemoveBodyForExternalOwner(JPH::BodyID(BodyID));

	BodyID = JPH::BodyID::cInvalidBodyID;
	JoltSubsystem = nullptr;
}

void UJoltPhysicsComponent::CreateBody()
{
	if (BodyID != JPH::BodyID::cInvalidBodyID)
		return;
	if (!GetOwner())
		return;

	SanitizeMeshComponents();

	const float BodyMass = bOverrideMass ? Mass : ComputeAutoMass();
	if (MotionType != EJoltMotionType::Static && BodyMass <= 0.0f)
	{
		UE_LOG(LogJoltPhysicsComponent, Error,
			TEXT("%hs: Failed to create body on %s, mass is zero — check its static mesh(es) have valid collision, and any mass override"),
			__FUNCTION__, *GetOwner()->GetName());
		return;
	}

	// Cached for the lifetime of the body: a world subsystem outlives every component in its world,
	// and the setters rely on this being set whenever BodyID is valid.
	JoltSubsystem = GetWorld() ? GetWorld()->GetSubsystem<UJoltSubsystem>() : nullptr;
	if (!JoltSubsystem)
	{
		UE_LOG(LogJoltPhysicsComponent, Error,
			TEXT("%hs: Failed to create body on %s, no Jolt subsystem for this world"),
			__FUNCTION__, *GetOwner()->GetName());
		return;
	}

	if (!JoltSubsystem->HasBodyCapacity())
	{
		UE_LOG(LogJoltPhysicsComponent, Error,
			TEXT("%hs: Failed to create body on %s, at MaxBodies limit (%d)"),
			__FUNCTION__, *GetOwner()->GetName(), GetDefault<UJoltSettings>()->MaxBodies);
		return;
	}

	// An unknown layer, or geometry that yielded no shapes, is reported by the subsystem.
	const FName ResolvedLayer = ResolveLayer();
	BodyID = MotionType == EJoltMotionType::Static ? JoltSubsystem->AddStaticBody(GetOwner(), Friction, Restitution, ResolvedLayer) : JoltSubsystem->AddDynamicBody(GetOwner(), Friction, Restitution, BodyMass, ResolvedLayer);

	if (BodyID == JPH::BodyID::cInvalidBodyID)
	{
		UE_LOG(LogJoltPhysicsComponent, Error,
			TEXT("%hs: Failed to create body on %s"), __FUNCTION__, *GetOwner()->GetName());
		return;
	}

	ApplyBodyProperties(JPH::BodyID(BodyID));
}

void UJoltPhysicsComponent::SanitizeMeshComponents() const
{
	TArray<UStaticMeshComponent*> StaticMeshComponents;
	GetOwner()->GetComponents<UStaticMeshComponent>(StaticMeshComponents);

	for (UStaticMeshComponent* StaticMeshComponent : StaticMeshComponents)
	{
		if (!StaticMeshComponent || !StaticMeshComponent->GetStaticMesh())
			continue;

		// Dynamic bodies have to be movable, static bodies can be either.
		if (MotionType == EJoltMotionType::Dynamic && StaticMeshComponent->GetMobility() != EComponentMobility::Movable)
		{
			UE_LOG(LogJoltPhysicsComponent, Warning,
				TEXT("%hs: %s is not movable but %s's Jolt Physics Component is dynamic, setting mobility to movable"),
				__FUNCTION__, *StaticMeshComponent->GetName(), *GetOwner()->GetName());
			StaticMeshComponent->SetMobility(EComponentMobility::Movable);
		}

		// Bodies need to have simulate physics turned off.
		if (StaticMeshComponent->IsSimulatingPhysics())
		{
			UE_LOG(LogJoltPhysicsComponent, Warning,
				TEXT("%hs: 'Simulate physics' turned on for component %s, disabling chaos"),
				__FUNCTION__, *StaticMeshComponent->GetName());
			StaticMeshComponent->SetSimulatePhysics(false);
		}
	}
}

void UJoltPhysicsComponent::ApplyBodyProperties(const JPH::BodyID& Body) const
{
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
		if (NumVelocityStepsOverride != 0)
			JoltSubsystem->JoltSetNumVelocityStepsOverride(Body, NumVelocityStepsOverride);
		if (NumPositionStepsOverride != 0)
			JoltSubsystem->JoltSetNumPositionStepsOverride(Body, NumPositionStepsOverride);
	}

	JoltSubsystem->JoltSetEnhancedInternalEdgeRemoval(Body, bEnhancedInternalEdgeRemoval);
}

void UJoltPhysicsComponent::SetObjectLayer(FName NewObjectLayer)
{
	Layer = NewObjectLayer;
	if (HasBody())
		JoltSubsystem->JoltSetObjectLayer(JPH::BodyID(BodyID), ResolveLayer());
}

FName UJoltPhysicsComponent::ResolveLayer() const
{
	if (Layer != DefaultLayerSentinel && !Layer.IsNone())
		return Layer;

	const UJoltSettings* Settings = GetDefault<UJoltSettings>();
	if (!Settings)
		return NAME_None;

	return (MotionType == EJoltMotionType::Static) ? Settings->DefaultStaticLayer : Settings->DefaultDynamicLayer;
}

void UJoltPhysicsComponent::SetMass(const float NewMass)
{
	Mass = NewMass;
	if (HasBody())
		JoltSubsystem->JoltSetMass(JPH::BodyID(BodyID), NewMass);
}

void UJoltPhysicsComponent::SetGravityFactor(float NewGravityFactor)
{
	GravityFactor = NewGravityFactor;
	if (HasBody())
		JoltSubsystem->JoltSetGravityFactor(JPH::BodyID(BodyID), NewGravityFactor);
}

void UJoltPhysicsComponent::SetApplyGyroscopicForce(bool bNewApplyGyroscopicForce)
{
	bApplyGyroscopicForce = bNewApplyGyroscopicForce;
	if (HasBody())
		JoltSubsystem->JoltSetApplyGyroscopicForce(JPH::BodyID(BodyID), bNewApplyGyroscopicForce);
}

void UJoltPhysicsComponent::SetMaxLinearVelocity(float NewMaxLinearVelocity)
{
	MaxLinearVelocity = NewMaxLinearVelocity;
	if (HasBody())
		JoltSubsystem->JoltSetMaxLinearVelocity(JPH::BodyID(BodyID), NewMaxLinearVelocity);
}

float UJoltPhysicsComponent::GetMaxLinearVelocity() const
{
	if (!HasBody())
		return MaxLinearVelocity;

	return JoltSubsystem->JoltGetMaxLinearVelocity(JPH::BodyID(BodyID));
}

void UJoltPhysicsComponent::SetMaxAngularVelocity(float NewMaxAngularVelocity)
{
	MaxAngularVelocity = NewMaxAngularVelocity;
	if (HasBody())
		JoltSubsystem->JoltSetMaxAngularVelocity(JPH::BodyID(BodyID), NewMaxAngularVelocity);
}

void UJoltPhysicsComponent::SetFriction(const float NewFriction)
{
	Friction = NewFriction;
	if (HasBody())
		JoltSubsystem->JoltSetFriction(JPH::BodyID(BodyID), NewFriction);
}

void UJoltPhysicsComponent::SetRestitution(const float NewRestitution)
{
	Restitution = NewRestitution;
	if (HasBody())
		JoltSubsystem->JoltSetRestitution(JPH::BodyID(BodyID), NewRestitution);
}

void UJoltPhysicsComponent::SetLinearDamping(float NewLinearDamping)
{
	LinearDamping = NewLinearDamping;
	if (HasBody())
		JoltSubsystem->JoltSetLinearDamping(JPH::BodyID(BodyID), NewLinearDamping);
}

void UJoltPhysicsComponent::SetAngularDamping(float NewAngularDamping)
{
	AngularDamping = NewAngularDamping;
	if (HasBody())
		JoltSubsystem->JoltSetAngularDamping(JPH::BodyID(BodyID), NewAngularDamping);
}

void UJoltPhysicsComponent::SetAllowSleeping(bool bNewAllowSleeping)
{
	bAllowSleeping = bNewAllowSleeping;
	if (HasBody())
		JoltSubsystem->JoltSetAllowSleeping(JPH::BodyID(BodyID), bNewAllowSleeping);
}

void UJoltPhysicsComponent::SetNumVelocityStepsOverride(int NewNumVelocityStepsOverride)
{
	NumVelocityStepsOverride = NewNumVelocityStepsOverride;
	if (HasBody())
		JoltSubsystem->JoltSetNumVelocityStepsOverride(JPH::BodyID(BodyID), NewNumVelocityStepsOverride);
}

void UJoltPhysicsComponent::SetNumPositionStepsOverride(int NewNumPositionStepsOverride)
{
	NumPositionStepsOverride = NewNumPositionStepsOverride;
	if (HasBody())
		JoltSubsystem->JoltSetNumPositionStepsOverride(JPH::BodyID(BodyID), NewNumPositionStepsOverride);
}

void UJoltPhysicsComponent::SetEnhancedInternalEdgeRemoval(bool bNewEnhancedInternalEdgeRemoval)
{
	bEnhancedInternalEdgeRemoval = bNewEnhancedInternalEdgeRemoval;
	if (HasBody())
		JoltSubsystem->JoltSetEnhancedInternalEdgeRemoval(JPH::BodyID(BodyID), bNewEnhancedInternalEdgeRemoval);
}

bool UJoltPhysicsComponent::GetBodyID(int& OutBodyID) const
{
	if (BodyID == JPH::BodyID::cInvalidBodyID)
		return false;
	OutBodyID = BodyID;
	return true;
}

TArray<FString> UJoltPhysicsComponent::GetObjectLayerNames() const
{
	const UJoltSettings* Settings = GetDefault<UJoltSettings>();
	if (!Settings)
		return {};

	const FName ThisMotionTypeDefault = (MotionType == EJoltMotionType::Static) ? Settings->DefaultStaticLayer : Settings->DefaultDynamicLayer;
	const FName OtherMotionTypeDefault = (MotionType == EJoltMotionType::Static) ? Settings->DefaultDynamicLayer : Settings->DefaultStaticLayer;

	TArray<FString> Names;
	Names.Reserve(Settings->ObjectLayers.Num() + 1);

	Names.Add(DefaultLayerSentinel.ToString());
	for (const FJoltObjectLayer& ObjLayer : Settings->ObjectLayers)
	{
		// Both defaults sit behind the "Default" entry above: picking the concrete name would freeze the
		// layer against later MotionType changes, and the other MotionType's default is never valid here.
		if (ObjLayer.Name.IsNone() || ObjLayer.Name == OtherMotionTypeDefault || ObjLayer.Name == ThisMotionTypeDefault)
			continue;

		Names.Add(ObjLayer.Name.ToString());
	}
	return Names;
}

float UJoltPhysicsComponent::ComputeAutoMass() const
{
	if (!GetOwner())
		return 0.0f;

	// Static meshes only: they're the sole source of geometry for the Jolt body, so anything
	// else with a BodySetup (shape components, etc.) would add mass the body doesn't represent.
	TArray<UStaticMeshComponent*> StaticMeshComponents;
	GetOwner()->GetComponents<UStaticMeshComponent>(StaticMeshComponents);

	float TotalMass = 0.0f;
	for (UStaticMeshComponent* StaticMeshComponent : StaticMeshComponents)
	{
		if (!StaticMeshComponent)
			continue;

		if (const UBodySetup* BodySetup = StaticMeshComponent->GetBodySetup())
		{
			if (const float ElementMass = BodySetup->CalculateMass(StaticMeshComponent); ElementMass > 0.0f)
			{
				TotalMass += ElementMass;
			}
		}
	}

	return TotalMass;
}

void UJoltPhysicsComponent::WakeBody() const
{
	if (HasBody())
		JoltSubsystem->JoltActivateBody(JPH::BodyID(BodyID));
}

#if WITH_EDITOR
void UJoltPhysicsComponent::PostEditChangeProperty(FPropertyChangedEvent& PropertyChangedEvent)
{
	Super::PostEditChangeProperty(PropertyChangedEvent);

	if (PropertyChangedEvent.Property == nullptr)
		return;

	const FName ChangedProp = PropertyChangedEvent.Property->GetFName();

	if (ChangedProp == GET_MEMBER_NAME_CHECKED(UJoltPhysicsComponent, MotionType))
	{
		// Back to the sentinel, never the concrete name: a written name becomes a per-instance override,
		// so a later MotionType change on a Blueprint could no longer reach placed instances.
		Layer = DefaultLayerSentinel;
	}
	else if (ChangedProp == GET_MEMBER_NAME_CHECKED(UJoltPhysicsComponent, bOverrideMass))
	{
		ComputedMass = ComputeAutoMass();
	}
}
#endif
