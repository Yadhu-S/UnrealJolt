#pragma once

#include "CoreMinimal.h"
#include "Components/ActorComponent.h"
#include "UnrealJolt/Helpers.h"
#include "JoltPhysicsComponent.generated.h"

DECLARE_LOG_CATEGORY_EXTERN(LogJoltPhysicsComponent, Log, All);

class UJoltSubsystem;

UENUM(BlueprintType)
enum class EJoltMotionType : uint8
{
	Static,
	Dynamic,
};

UENUM(BlueprintType, meta = (Bitflags, UseEnumValuesAsMaskValuesInEditor = "true"))
enum class EJoltAllowedDOFs : uint8
{
	None = 0 UMETA(Hidden),
	TranslationX = 1 << 0,
	TranslationY = 1 << 1,
	TranslationZ = 1 << 2,
	RotationX = 1 << 3,
	RotationY = 1 << 4,
	RotationZ = 1 << 5,
};
 
ENUM_CLASS_FLAGS(EJoltAllowedDOFs)

/** An alternative to the tagging system. Allows you to specify physics properties. */
UCLASS(ClassGroup=(Custom), meta=(BlueprintSpawnableComponent))
class UNREALJOLT_API UJoltPhysicsComponent : public UActorComponent
{
	GENERATED_BODY()

public:
	UJoltPhysicsComponent();

protected:
	/** Assigns a new SortID when a fresh instance is created. */
	virtual void OnComponentCreated() override;
	
	/** Refreshes the ComputedMass preview in editor whenever the component is registered. */
	virtual void OnRegister() override;

	/** Regenerates SortID if invalid. */
	virtual void PostLoad() override;
	
	/** Regenerates SortID on duplication so copies don't share the same sort identity. */
	virtual void PostDuplicate(bool bDuplicateForPIE) override;

	#if WITH_EDITOR
	/** Regenerates SortID after in-editor actor duplication. */
	virtual void PostEditImport() override;
	#endif
	
	/** Creates body if one doesn't exist already. */
	virtual void BeginPlay() override;
	
	/** Removes the body from the Jolt subsystem when the actor ends play. */
	virtual void EndPlay(const EEndPlayReason::Type EndPlayReason) override;
	
public:
	/** Called by Jolt Physics Subsystem to create the body for this component. */
	void CreateBody();
	
	/** Whether this body is static (immovable, zero mass) or dynamic (simulated, affected by forces). */
	UPROPERTY(EditAnywhere, BlueprintReadOnly, Category = "Jolt Physics|Motion")
	EJoltMotionType MotionType = EJoltMotionType::Static;
	
	// Can't have a motion type setter here. If a body is statically added, MotionProperties does not get created.
	// We could set mAllowDynamicOrKinematic to true to get around this.

	/** Jolt object layer this body is placed on, used for collision filtering. "Default" follows the project's default layer for the selected MotionType. */
	UPROPERTY(EditAnywhere, Category = "Jolt Physics|Motion", meta = (GetOptions = "GetObjectLayerNames"))
	FName Layer = FName("Default");
	
	/** Sets a body's object layer for collision filtering. */
	UFUNCTION(BlueprintCallable, Category = "Jolt Physics|Motion")
	void SetObjectLayer(FName NewObjectLayer);
	
	/** Indicates which degrees of freedom this body has. Can be used to limit simulation to 2D. */
	UPROPERTY(EditAnywhere, BlueprintReadOnly, Category = "Jolt Physics|Motion", 
		meta = (Bitmask, BitmaskEnum = "/Script/UnrealJolt.EJoltAllowedDOFs", EditCondition = "MotionType != EJoltMotionType::Static"))
	int32 AllowedDOFs = 0b111111;
	
	/** Overrides the automatically computed mass with a fixed value */
	UPROPERTY(EditAnywhere, BlueprintReadOnly, Category = "Jolt Physics|Motion", meta = (EditCondition = "MotionType != EJoltMotionType::Static"))
	bool bOverrideMass = false;

	/** Mass of the body in KG. Only used when bOverrideMass is on — otherwise mass comes from the collision geometry and physical material. */
	UPROPERTY(EditAnywhere, BlueprintReadOnly, Category = "Jolt Physics|Motion", meta = (EditCondition = "bOverrideMass && MotionType != EJoltMotionType::Static", UIMin = "0.001", Units = "Kilograms"))
	float Mass = 100.0f;

	/** Mass the body will actually be created with, in KG. Preview only — never saved, and ignored when bOverrideMass is on. */
	UPROPERTY(VisibleAnywhere, Transient, Category = "Jolt Physics|Motion", meta = (EditCondition = "!bOverrideMass && MotionType != EJoltMotionType::Static", Units = "Kilograms"))
	float ComputedMass = 0.0f;
	
	/** Sets the mass of the body in KG. */
	UFUNCTION(BlueprintCallable, Category = "Jolt Physics|Motion")
	void SetMass(float NewMass);
	
	/** Value to multiply gravity with for this body. */
	UPROPERTY(EditAnywhere, BlueprintReadOnly, Category = "Jolt Physics|Forces", 
		meta = (EditCondition = "MotionType != EJoltMotionType::Static"))
	float GravityFactor = 1.f;

	/** Sets the gravity factor for this body. */
	UFUNCTION(BlueprintCallable, Category = "Jolt Physics|Forces")
	void SetGravityFactor(float NewGravityFactor);
	
	/** Simulates gyroscopic torque so spinning bodies resist changes to their spin axis. */
	UPROPERTY(EditAnywhere, BlueprintReadOnly, Category = "Jolt Physics|Forces", 
		meta = (EditCondition = "MotionType != EJoltMotionType::Static"))
	bool bApplyGyroscopicForce = false;
	
	/** Sets whether gyroscopic torque is simulated for this body. */
	UFUNCTION(BlueprintCallable, Category = "Jolt Physics|Forces")
	void SetApplyGyroscopicForce(bool bNewApplyGyroscopicForce);
	
	/** Maximum linear velocity that this body can reach (cm/s) */
	UPROPERTY(EditAnywhere, Category = "Jolt Physics|Motion", 
		meta = (Units = "cm/s", EditCondition = "MotionType != EJoltMotionType::Static"))
	float MaxLinearVelocity = 500.f * JOLT_TO_WORLD_SCALE;
	
	/** Sets the maximum linear velocity this body can reach (cm/s). */
	UFUNCTION(BlueprintCallable, Category = "Jolt Physics|Motion")
	void SetMaxLinearVelocity(float NewMaxLinearVelocity);
	
	/** Gets the maximum linear velocity this body can reach (cm/s). */
	UFUNCTION(BlueprintCallable, Category = "Jolt Physics|Motion")
	float GetMaxLinearVelocity() const;
	
	/** Maximum angular velocity that this body can reach (deg/s). */
	UPROPERTY(EditAnywhere, BlueprintReadOnly, Category = "Jolt Physics|Motion",
		meta = (Units = "deg/s", EditCondition = "MotionType != EJoltMotionType::Static"))
	float MaxAngularVelocity = 2700.f;

	/** Sets the maximum angular velocity this body can reach (deg/s). */
	UFUNCTION(BlueprintCallable, Category = "Jolt Physics|Motion")
	void SetMaxAngularVelocity(float NewMaxAngularVelocity);
	
	/** Coefficient of friction applied to this body. Higher values resist sliding against other surfaces. */
	UPROPERTY(EditAnywhere, BlueprintReadOnly, Category = "Jolt Physics|Surface")
	float Friction = 2.0f;
	
	/** Sets the friction coefficient applied to this body. */
	UFUNCTION(BlueprintCallable, Category = "Jolt Physics|Surface")
	void SetFriction(float NewFriction);
	
	/** Coefficient of restitution (bounciness). 0 = no bounce, 1 = fully elastic bounce. */
	UPROPERTY(EditAnywhere, BlueprintReadOnly, Category = "Jolt Physics|Surface")
	float Restitution = 0.5f;
	
	/** Sets the restitution coefficient (bounciness) of this body. */
	UFUNCTION(BlueprintCallable, Category = "Jolt Physics|Surface")
	void SetRestitution(float NewRestitution);
	
	/** Drag force added to reduce linear movement, applies dv/dt = -c * v. */
	UPROPERTY(EditAnywhere, BlueprintReadOnly, Category = "Jolt Physics|Damping", 
		meta = (EditCondition = "MotionType != EJoltMotionType::Static"))
	float LinearDamping = 0.05f;

	/** Sets the linear damping applied to this body. */
	UFUNCTION(BlueprintCallable, Category = "Jolt Physics|Damping")
	void SetLinearDamping(float NewLinearDamping);

	/** Drag force added to reduce angular movement, applies dw/dt = -c * w. */
	UPROPERTY(EditAnywhere, BlueprintReadOnly, Category = "Jolt Physics|Damping", 
		meta = (EditCondition = "MotionType != EJoltMotionType::Static"))
	float AngularDamping = 0.05f;

	/** Sets the angular damping applied to this body. */
	UFUNCTION(BlueprintCallable, Category = "Jolt Physics|Damping")
	void SetAngularDamping(float NewAngularDamping);
	
	/** Whether this body can go to sleep. */
	UPROPERTY(EditAnywhere, BlueprintReadOnly, Category = "Jolt Physics|Advanced", 
		meta = (EditCondition = "MotionType != EJoltMotionType::Static"))
	bool bAllowSleeping = true;

	/** Sets whether this body is allowed to go to sleep. */
	UFUNCTION(BlueprintCallable, Category = "Jolt Physics|Solver")
	void SetAllowSleeping(bool bNewAllowSleeping);
	
	/** Overrides the number of solver velocity iterations, 0 uses the project default */
	UPROPERTY(EditAnywhere, BlueprintReadOnly, Category = "Jolt Physics|Advanced", 
		meta = (EditCondition = "MotionType != EJoltMotionType::Static"))
	int NumVelocityStepsOverride = 0;

	/** Sets the solver velocity iteration override for this body, 0 uses the project default. */
	UFUNCTION(BlueprintCallable, Category = "Jolt Physics|Solver")
	void SetNumVelocityStepsOverride(int NewNumVelocityStepsOverride);

	/** Overrides the number of solver position iterations, 0 uses the project default */
	UPROPERTY(EditAnywhere, BlueprintReadOnly, Category = "Jolt Physics|Advanced", 
		meta = (EditCondition = "MotionType != EJoltMotionType::Static"))
	int NumPositionStepsOverride = 0;

	/** Sets the solver position iteration override for this body, 0 uses the project default. */
	UFUNCTION(BlueprintCallable, Category = "Jolt Physics|Solver")
	void SetNumPositionStepsOverride(int NewNumPositionStepsOverride);

	/** Makes extra effort to remove ghost collisions on internal mesh edges, at a performance cost */
	UPROPERTY(EditAnywhere, BlueprintReadOnly, Category = "Jolt Physics|Advanced")
	bool bEnhancedInternalEdgeRemoval = false;

	/** Sets whether extra effort is made to remove ghost collisions on internal mesh edges. */
	UFUNCTION(BlueprintCallable, Category = "Jolt Physics|Solver")
	void SetEnhancedInternalEdgeRemoval(bool bNewEnhancedInternalEdgeRemoval);
	
	/**
	 * Wakes this body if Jolt has put it to sleep. Property setters do not wake a sleeping body,
	 * so call this after changing one on a body that has come to rest.
	 */
	UFUNCTION(BlueprintCallable, Category = "Jolt Physics|Motion")
	void WakeBody() const;

	UFUNCTION(BlueprintPure, Category = "Jolt Physics|Helpers")
	bool GetBodyID(int& OutBodyID) const;

	/** Stable identifier used to order body creation deterministically. */
	UPROPERTY(VisibleAnywhere, BlueprintReadOnly, Category = "Jolt Physics|Advanced")
	FGuid SortID;
	
private:
	UPROPERTY()
	int64 BodyID = JPH::BodyID::cInvalidBodyID;

	/** Resolved in CreateBody and held for the body's lifetime. Set whenever BodyID is valid. */
	UPROPERTY(Transient)
	TObjectPtr<UJoltSubsystem> JoltSubsystem;

	/** True once CreateBody has a live body, so the setters know whether to push straight to Jolt. */
	bool HasBody() const { return JoltSubsystem != nullptr && BodyID != JPH::BodyID::cInvalidBodyID; }

	UFUNCTION()
    TArray<FString> GetObjectLayerNames() const;
	

	/** Forces the owner's static meshes into a state Jolt can drive: movable when dynamic, Chaos simulation off. */
	void SanitizeMeshComponents() const;

	/** Pushes the authored properties onto a freshly created body. */
	void ApplyBodyProperties(const JPH::BodyID& Body) const;
	/** The layer this body is actually created on: the authored name, or the MotionType default when set to "Default". */
	FName ResolveLayer() const;

	/** Sums the mass of the owner's static meshes. Pure — callers decide what to do with it. */
	float ComputeAutoMass() const;
	
	#if WITH_EDITOR
	virtual void PostEditChangeProperty(FPropertyChangedEvent& PropertyChangedEvent) override;
	#endif
};