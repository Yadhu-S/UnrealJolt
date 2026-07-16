#pragma once

#include "CoreMinimal.h"
#include "Components/ActorComponent.h"
#include "JoltPhysicsComponent.generated.h"

UENUM(BlueprintType)
enum class EJoltMobility : uint8
{
	Static,
	Dynamic,
};

/** An alternative to the tagging system. Allows you to specify mass, friction, restitution, and layer.
    TO-DO: Helper functions (set mass, set restitution, set friction, etc.) */
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
	UPROPERTY(EditAnywhere, BlueprintReadOnly, Category = "Jolt Physics")
	EJoltMobility Mobility = EJoltMobility::Static;

	/** Jolt object layer this body is placed on, used for collision filtering. "Default" resolves to the project's default layer for the selected Mobility. */
	UPROPERTY(EditAnywhere, Category = "Jolt Physics", meta = (GetOptions = "GetObjectLayerNames"))
	FName Layer = FName("Default");

	UPROPERTY(EditAnywhere, BlueprintReadOnly, Category = "Jolt Physics", meta = (InlineEditConditionToggle))
	bool bOverrideMass = false;
	
	/** Mass of the body in KG. When bOverrideMass is off, this is computed based on physical material and collision geometry. */
	UPROPERTY(EditAnywhere, BlueprintReadOnly, Category = "Jolt Physics", meta = (EditCondition = "bOverrideMass", UIMin = "0.001", Units = "Kilograms"))
	float Mass = 100.0f;

	/** Coefficient of friction applied to this body. Higher values resist sliding against other surfaces. */
	UPROPERTY(EditAnywhere, BlueprintReadOnly, Category = "Jolt Physics")
	float Friction = 2.0f;

	/** Coefficient of restitution (bounciness). 0 = no bounce, 1 = fully elastic bounce. */
	UPROPERTY(EditAnywhere, BlueprintReadOnly, Category = "Jolt Physics")
	float Restitution = 0.5f;
	
private:
	UFUNCTION()
    TArray<FString> GetObjectLayerNames() const;
	
	FName ResolveLayer() const;
	void  RecalculateMass();
	
	#if WITH_EDITOR
	virtual void PostEditChangeProperty(FPropertyChangedEvent& PropertyChangedEvent) override;
	#endif
};