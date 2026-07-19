#pragma once

#include "CoreMinimal.h"
#include "Components/ActorComponent.h"
#include "JoltPhysicsComponent.generated.h"

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
	virtual void BeginPlay() override;
	virtual void OnRegister() override;
	
public:
	/** Whether this body is static (immovable, zero mass) or dynamic (simulated, affected by forces). */
	UPROPERTY(EditAnywhere, BlueprintReadOnly, Category = "Jolt Physics|Motion")
	EJoltMotionType MotionType = EJoltMotionType::Static;
	
	// Can't have a motion type setter here. If a body is statically added, MotionProperties does not get created.
	// We could set mAllowDynamicOrKinematic to true to get around this.

	/** Indicates which degrees of freedom this body has. Can be used to limit simulation to 2D. */
	UPROPERTY(EditAnywhere, BlueprintReadOnly, Category = "Jolt Physics|Motion", meta = (Bitmask, BitmaskEnum = "/Script/UnrealJolt.EJoltAllowedDOFs"))
	int32 AllowedDOFs = 0b111111;

	/** Jolt object layer this body is placed on, used for collision filtering. "Default" resolves to the project's default layer for the selected MotionType. */
	UPROPERTY(EditAnywhere, Category = "Jolt Physics|Motion", meta = (GetOptions = "GetObjectLayerNames"))
	FName Layer = FName("Default");
	
	/** Sets a body's object layer for collision filtering. */
	UFUNCTION(BlueprintCallable, Category = "Jolt Physics|Motion", meta = (DefaultToSelf = "Actor"))
	static void SetObjectLayer(UPARAM(meta=(RequireActorComponent = "JoltPhysicsComponent")) AActor* Actor, FName NewObjectLayer);
	
	UPROPERTY(EditAnywhere, BlueprintReadOnly, Category = "Jolt Physics|Motion", meta = (InlineEditConditionToggle))
	bool bOverrideMass = false;
	
	/** Mass of the body in KG. When bOverrideMass is off, this is computed based on physical material and collision geometry. */
	UPROPERTY(EditAnywhere, BlueprintReadOnly, Category = "Jolt Physics|Motion", meta = (EditCondition = "bOverrideMass", UIMin = "0.001", Units = "Kilograms"))
	float Mass = 100.0f;
	
	/** Sets the mass of the body in KG. */
	UFUNCTION(BlueprintCallable, Category = "Jolt Physics|Motion", meta = (DefaultToSelf = "Actor"))
	static void SetMass(UPARAM(meta=(RequireActorComponent = "JoltPhysicsComponent")) AActor* Actor, float NewMass);
	
	/** Value to multiply gravity with for this body. */
	UPROPERTY(EditAnywhere, BlueprintReadOnly, Category = "Jolt Physics|Forces")
	float GravityFactor = 1.f;

	/** Sets the gravity factor for this body. */
	UFUNCTION(BlueprintCallable, Category = "Jolt Physics|Forces", meta = (DefaultToSelf = "Actor"))
	static void SetGravityFactor(UPARAM(meta=(RequireActorComponent = "JoltPhysicsComponent")) AActor* Actor, float NewGravityFactor);
	
	/** Simulates gyroscopic torque so spinning bodies resist changes to their spin axis. */
	UPROPERTY(EditAnywhere, BlueprintReadOnly, Category = "Jolt Physics|Forces")
	bool bApplyGyroscopicForce = false;
	
	/** Sets whether gyroscopic torque is simulated for this body. */
	UFUNCTION(BlueprintCallable, Category = "Jolt Physics|Forces", meta = (DefaultToSelf = "Actor"))
	static void SetApplyGyroscopicForce(UPARAM(meta=(RequireActorComponent = "JoltPhysicsComponent")) AActor* Actor, bool bNewApplyGyroscopicForce);
	
	/** Maximum linear velocity that this body can reach (m/s) */
	UPROPERTY(EditAnywhere, BlueprintReadOnly, Category = "Jolt Physics|Motion", meta = (Units = "m/s"))
	float MaxLinearVelocity = 500.f;
	
	/** Sets the maximum linear velocity this body can reach (m/s). */
	UFUNCTION(BlueprintCallable, Category = "Jolt Physics|Motion", meta = (DefaultToSelf = "Actor"))
	static void SetMaxLinearVelocity(UPARAM(meta=(RequireActorComponent = "JoltPhysicsComponent")) AActor* Actor, float NewMaxLinearVelocity);
	
	/** Maximum angular velocity that this body can reach (rad/s). */
	UPROPERTY(EditAnywhere, BlueprintReadOnly, Category = "Jolt Physics|Motion", meta = (Units = "rad/s"))
	float MaxAngularVelocity = 0.25f * PI * 60.0f;
	
	/** Sets the maximum angular velocity this body can reach (rad/s). */
	UFUNCTION(BlueprintCallable, Category = "Jolt Physics|Motion", meta = (DefaultToSelf = "Actor"))
	static void SetMaxAngularVelocity(UPARAM(meta=(RequireActorComponent = "JoltPhysicsComponent")) AActor* Actor, float NewMaxAngularVelocity);
	
	/** Coefficient of friction applied to this body. Higher values resist sliding against other surfaces. */
	UPROPERTY(EditAnywhere, BlueprintReadOnly, Category = "Jolt Physics|Surface")
	float Friction = 2.0f;
	
	/** Sets the friction coefficient applied to this body. */
	UFUNCTION(BlueprintCallable, Category = "Jolt Physics|Surface", meta = (DefaultToSelf = "Actor"))
	static void SetFriction(UPARAM(meta=(RequireActorComponent = "JoltPhysicsComponent")) AActor* Actor, float NewFriction);
	
	/** Coefficient of restitution (bounciness). 0 = no bounce, 1 = fully elastic bounce. */
	UPROPERTY(EditAnywhere, BlueprintReadOnly, Category = "Jolt Physics|Surface")
	float Restitution = 0.5f;
	
	/** Sets the restitution coefficient (bounciness) of this body. */
	UFUNCTION(BlueprintCallable, Category = "Jolt Physics|Surface", meta = (DefaultToSelf = "Actor"))
	static void SetRestitution(UPARAM(meta=(RequireActorComponent = "JoltPhysicsComponent")) AActor* Actor, float NewRestitution);
	
	/** Drag force added to reduce linear movement, applies dv/dt = -c * v. */
	UPROPERTY(EditAnywhere, BlueprintReadOnly, Category = "Jolt Physics|Damping")
	float LinearDamping = 0.05f;

	/** Sets the linear damping applied to this body. */
	UFUNCTION(BlueprintCallable, Category = "Jolt Physics|Damping", meta = (DefaultToSelf = "Actor"))
	static void SetLinearDamping(UPARAM(meta=(RequireActorComponent = "JoltPhysicsComponent")) AActor* Actor, float NewLinearDamping);

	/** Drag force added to reduce angular movement, applies dw/dt = -c * w. */
	UPROPERTY(EditAnywhere, BlueprintReadOnly, Category = "Jolt Physics|Damping")
	float AngularDamping = 0.05f;

	/** Sets the angular damping applied to this body. */
	UFUNCTION(BlueprintCallable, Category = "Jolt Physics|Damping", meta = (DefaultToSelf = "Actor"))
	static void SetAngularDamping(UPARAM(meta=(RequireActorComponent = "JoltPhysicsComponent")) AActor* Actor, float NewAngularDamping);
	
	/** Whether this body can go to sleep. */
	UPROPERTY(EditAnywhere, BlueprintReadOnly, Category = "Jolt Physics", AdvancedDisplay)
	bool bAllowSleeping = true;

	/** Sets whether this body is allowed to go to sleep. */
	UFUNCTION(BlueprintCallable, Category = "Jolt Physics|Solver", meta = (DefaultToSelf = "Actor"))
	static void SetAllowSleeping(UPARAM(meta=(RequireActorComponent = "JoltPhysicsComponent")) AActor* Actor, bool bNewAllowSleeping);
	
	/** Overrides the number of solver velocity iterations, 0 uses the project default */
	UPROPERTY(EditAnywhere, BlueprintReadOnly, Category = "Jolt Physics", AdvancedDisplay)
	int NumVelocityStepsOverride = 0;

	/** Sets the solver velocity iteration override for this body, 0 uses the project default. */
	UFUNCTION(BlueprintCallable, Category = "Jolt Physics|Solver", meta = (DefaultToSelf = "Actor"))
	static void SetNumVelocityStepsOverride(UPARAM(meta=(RequireActorComponent = "JoltPhysicsComponent")) AActor* Actor, int NewNumVelocityStepsOverride);

	/** Overrides the number of solver position iterations, 0 uses the project default */
	UPROPERTY(EditAnywhere, BlueprintReadOnly, Category = "Jolt Physics", AdvancedDisplay)
	int NumPositionStepsOverride = 0;

	/** Sets the solver position iteration override for this body, 0 uses the project default. */
	UFUNCTION(BlueprintCallable, Category = "Jolt Physics|Solver", meta = (DefaultToSelf = "Actor"))
	static void SetNumPositionStepsOverride(UPARAM(meta=(RequireActorComponent = "JoltPhysicsComponent")) AActor* Actor, int NewNumPositionStepsOverride);

	/** Makes extra effort to remove ghost collisions on internal mesh edges, at a performance cost */
	UPROPERTY(EditAnywhere, BlueprintReadOnly, Category = "Jolt Physics", AdvancedDisplay)
	bool bEnhancedInternalEdgeRemoval = false;

	/** Sets whether extra effort is made to remove ghost collisions on internal mesh edges. */
	UFUNCTION(BlueprintCallable, Category = "Jolt Physics|Solver", meta = (DefaultToSelf = "Actor"))
	static void SetEnhancedInternalEdgeRemoval(UPARAM(meta=(RequireActorComponent = "JoltPhysicsComponent")) AActor* Actor, bool bNewEnhancedInternalEdgeRemoval);
	
	UFUNCTION(BlueprintPure, Category = "Jolt Physics|Helpers")
	bool GetBodyID(int& OutBodyID) const;

private:
	UPROPERTY()
	int64 BodyID = -1;
	
	UFUNCTION()
    TArray<FString> GetObjectLayerNames() const;
	
	FName ResolveLayer() const;
	void  RecalculateMass();
	
	#if WITH_EDITOR
	virtual void PostEditChangeProperty(FPropertyChangedEvent& PropertyChangedEvent) override;
	#endif
};