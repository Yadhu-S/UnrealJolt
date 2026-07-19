#pragma once

#include "CoreMinimal.h"
#include "UObject/Interface.h"
#include "JoltPhysicsCallbackInterface.generated.h"

UINTERFACE(BlueprintType)
class UNREALJOLT_API UJoltPhysicsCallbackInterface : public UInterface
{
	GENERATED_BODY()
};

/** Implement to receive pre and post physics step callbacks from the Jolt subsystem. */
class UNREALJOLT_API IJoltPhysicsCallbackInterface
{
	GENERATED_BODY()

public:
	/** Called before the physics step is simulated, on the game thread. */
	UFUNCTION(BlueprintNativeEvent, Category = "Jolt Physics")
	void OnPrePhysicsStep(float DeltaTime);

	/** Called after the physics step is simulated, on the game thread. */
	UFUNCTION(BlueprintNativeEvent, Category = "Jolt Physics")
	void OnPostPhysicsStep(float DeltaTime);
};