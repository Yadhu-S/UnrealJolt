#pragma once

#include "CoreMinimal.h"
#include "Components/ActorComponent.h"
#include "UnrealJolt/Helpers.h"
#include "JoltPhysicsComponent.generated.h"

DECLARE_LOG_CATEGORY_EXTERN(LogJoltPhysicsComponent, Log, All);

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

	/** Jolt object layer this body is placed on, used for collision filtering. "Default" resolves to the project's default layer for the selected MotionType. */
	UPROPERTY(EditAnywhere, Category = "Jolt Physics|Motion", meta = (GetOptions = "GetObjectLayerNames"))
	FName Layer = FName("Default");
	
	/** Sets a body's object layer for collision filtering. */
	UFUNCTION(BlueprintCallable, Category = "Jolt Physics|Motion", meta = (DefaultToSelf = "Actor"))
	static void SetObjectLayer(AActor* Actor, FName NewObjectLayer);
	
	/** Indicates which degrees of freedom this body has. Can be used to limit simulation to 2D. */
	UPROPERTY(EditAnywhere, BlueprintReadOnly, Category = "Jolt Physics|Motion", 
		meta = (Bitmask, BitmaskEnum = "/Script/UnrealJolt.EJoltAllowedDOFs", EditCondition = "MotionType != EJoltMotionType::Static"))
	int32 AllowedDOFs = 0b111111;
	
	/** Overrides the automatically computed mass with a fixed value */
	UPROPERTY(EditAnywhere, BlueprintReadOnly, Category = "Jolt Physics|Motion", meta = (EditCondition = "MotionType != EJoltMotionType::Static"))
	bool bOverrideMass = false;

	/** Mass of the body in KG. When bOverrideMass is off, this is computed based on physical material and collision geometry. */
	UPROPERTY(EditAnywhere, BlueprintReadOnly, Category = "Jolt Physics|Motion", meta = (EditCondition = "bOverrideMass && MotionType != EJoltMotionType::Static", UIMin = "0.001", Units = "Kilograms"))
	float Mass = 100.0f;
	
	/** Sets the mass of the body in KG. */
	UFUNCTION(BlueprintCallable, Category = "Jolt Physics|Motion", meta = (DefaultToSelf = "Actor"))
	static void SetMass(AActor* Actor, float NewMass);
	
	/** Value to multiply gravity with for this body. */
	UPROPERTY(EditAnywhere, BlueprintReadOnly, Category = "Jolt Physics|Forces", 
		meta = (EditCondition = "MotionType != EJoltMotionType::Static"))
	float GravityFactor = 1.f;

	/** Sets the gravity factor for this body. */
	UFUNCTION(BlueprintCallable, Category = "Jolt Physics|Forces", meta = (DefaultToSelf = "Actor"))
	static void SetGravityFactor(AActor* Actor, float NewGravityFactor);
	
	/** Simulates gyroscopic torque so spinning bodies resist changes to their spin axis. */
	UPROPERTY(EditAnywhere, BlueprintReadOnly, Category = "Jolt Physics|Forces", 
		meta = (EditCondition = "MotionType != EJoltMotionType::Static"))
	bool bApplyGyroscopicForce = false;
	
	/** Sets whether gyroscopic torque is simulated for this body. */
	UFUNCTION(BlueprintCallable, Category = "Jolt Physics|Forces", meta = (DefaultToSelf = "Actor"))
	static void SetApplyGyroscopicForce(AActor* Actor, bool bNewApplyGyroscopicForce);
	
	/** Maximum linear velocity that this body can reach (cm/s) */
	UPROPERTY(EditAnywhere, Category = "Jolt Physics|Motion", 
		meta = (Units = "m/s", EditCondition = "MotionType != EJoltMotionType::Static"))
	float MaxLinearVelocity = 500.f * JOLT_TO_WORLD_SCALE;
	
	/** Sets the maximum linear velocity this body can reach (cm/s). */
	UFUNCTION(BlueprintCallable, Category = "Jolt Physics|Motion", meta = (DefaultToSelf = "Actor"))
	static void SetMaxLinearVelocity(AActor* Actor, float NewMaxLinearVelocity);
	
	/** Gets the maximum linear velocity this body can reach (cm/s). */
	UFUNCTION(BlueprintCallable, Category = "Jolt Physics|Motion", meta = (DefaultToSelf = "Actor"))
	static float GetMaxLinearVelocity(AActor* Actor);
	
	/** Maximum angular velocity that this body can reach (rad/s). */
	UPROPERTY(EditAnywhere, BlueprintReadOnly, Category = "Jolt Physics|Motion", 
		meta = (Units = "rad/s", EditCondition = "MotionType != EJoltMotionType::Static"))
	float MaxAngularVelocity = 0.25f * PI * 60.0f;
	
	/** Sets the maximum angular velocity this body can reach (rad/s). */
	UFUNCTION(BlueprintCallable, Category = "Jolt Physics|Motion", meta = (DefaultToSelf = "Actor"))
	static void SetMaxAngularVelocity(AActor* Actor, float NewMaxAngularVelocity);
	
	/** Coefficient of friction applied to this body. Higher values resist sliding against other surfaces. */
	UPROPERTY(EditAnywhere, BlueprintReadOnly, Category = "Jolt Physics|Surface")
	float Friction = 2.0f;
	
	/** Sets the friction coefficient applied to this body. */
	UFUNCTION(BlueprintCallable, Category = "Jolt Physics|Surface", meta = (DefaultToSelf = "Actor"))
	static void SetFriction(AActor* Actor, float NewFriction);
	
	/** Coefficient of restitution (bounciness). 0 = no bounce, 1 = fully elastic bounce. */
	UPROPERTY(EditAnywhere, BlueprintReadOnly, Category = "Jolt Physics|Surface")
	float Restitution = 0.5f;
	
	/** Sets the restitution coefficient (bounciness) of this body. */
	UFUNCTION(BlueprintCallable, Category = "Jolt Physics|Surface", meta = (DefaultToSelf = "Actor"))
	static void SetRestitution(AActor* Actor, float NewRestitution);
	
	/** Drag force added to reduce linear movement, applies dv/dt = -c * v. */
	UPROPERTY(EditAnywhere, BlueprintReadOnly, Category = "Jolt Physics|Damping", 
		meta = (EditCondition = "MotionType != EJoltMotionType::Static"))
	float LinearDamping = 0.05f;

	/** Sets the linear damping applied to this body. */
	UFUNCTION(BlueprintCallable, Category = "Jolt Physics|Damping", meta = (DefaultToSelf = "Actor"))
	static void SetLinearDamping(AActor* Actor, float NewLinearDamping);

	/** Drag force added to reduce angular movement, applies dw/dt = -c * w. */
	UPROPERTY(EditAnywhere, BlueprintReadOnly, Category = "Jolt Physics|Damping", 
		meta = (EditCondition = "MotionType != EJoltMotionType::Static"))
	float AngularDamping = 0.05f;

	/** Sets the angular damping applied to this body. */
	UFUNCTION(BlueprintCallable, Category = "Jolt Physics|Damping", meta = (DefaultToSelf = "Actor"))
	static void SetAngularDamping(AActor* Actor, float NewAngularDamping);
	
	/** Whether this body can go to sleep. */
	UPROPERTY(EditAnywhere, BlueprintReadOnly, Category = "Jolt Physics|Advanced", 
		meta = (EditCondition = "MotionType != EJoltMotionType::Static"))
	bool bAllowSleeping = true;

	/** Sets whether this body is allowed to go to sleep. */
	UFUNCTION(BlueprintCallable, Category = "Jolt Physics|Solver", meta = (DefaultToSelf = "Actor"))
	static void SetAllowSleeping(AActor* Actor, bool bNewAllowSleeping);
	
	/** Overrides the number of solver velocity iterations, 0 uses the project default */
	UPROPERTY(EditAnywhere, BlueprintReadOnly, Category = "Jolt Physics|Advanced", 
		meta = (EditCondition = "MotionType != EJoltMotionType::Static"))
	int NumVelocityStepsOverride = 0;

	/** Sets the solver velocity iteration override for this body, 0 uses the project default. */
	UFUNCTION(BlueprintCallable, Category = "Jolt Physics|Solver", meta = (DefaultToSelf = "Actor"))
	static void SetNumVelocityStepsOverride(AActor* Actor, int NewNumVelocityStepsOverride);

	/** Overrides the number of solver position iterations, 0 uses the project default */
	UPROPERTY(EditAnywhere, BlueprintReadOnly, Category = "Jolt Physics|Advanced", 
		meta = (EditCondition = "MotionType != EJoltMotionType::Static"))
	int NumPositionStepsOverride = 0;

	/** Sets the solver position iteration override for this body, 0 uses the project default. */
	UFUNCTION(BlueprintCallable, Category = "Jolt Physics|Solver", meta = (DefaultToSelf = "Actor"))
	static void SetNumPositionStepsOverride(AActor* Actor, int NewNumPositionStepsOverride);

	/** Makes extra effort to remove ghost collisions on internal mesh edges, at a performance cost */
	UPROPERTY(EditAnywhere, BlueprintReadOnly, Category = "Jolt Physics|Advanced")
	bool bEnhancedInternalEdgeRemoval = false;

	/** Sets whether extra effort is made to remove ghost collisions on internal mesh edges. */
	UFUNCTION(BlueprintCallable, Category = "Jolt Physics|Solver", meta = (DefaultToSelf = "Actor"))
	static void SetEnhancedInternalEdgeRemoval(AActor* Actor, bool bNewEnhancedInternalEdgeRemoval);
	
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