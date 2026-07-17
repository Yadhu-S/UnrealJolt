#pragma once

#include "CoreMinimal.h"
#include "BlueprintCompilerExtension.h"
#include "JoltPhysicsBlueprintCompilerExtension.generated.h"

/** Fails Blueprint compilation if a function tagged RequireActorComponent is called on an Actor missing that component. 
	Potentially overkill, but it's more informative than silent failing if, say, you call SetMass(Actor, Mass) on an actor
	which doesn't have a Jolt Physics Component. Generic, so you can use it elsewhere too. */
UCLASS()
class UJoltPhysicsBlueprintCompilerExtension : public UBlueprintCompilerExtension
{
	GENERATED_BODY()

protected:
	virtual void ProcessBlueprintCompiled(const FKismetCompilerContext& CompilationContext, const FBlueprintCompiledData& Data) override;
};