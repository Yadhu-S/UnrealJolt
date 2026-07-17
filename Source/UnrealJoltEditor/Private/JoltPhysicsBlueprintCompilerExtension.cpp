#include "JoltPhysicsBlueprintCompilerExtension.h"
#include "K2Node_CallFunction.h"
#include "Engine/SCS_Node.h"
#include "Kismet2/BlueprintEditorUtils.h"
#include "Kismet2/CompilerResultsLog.h"

bool UJoltPhysicsBlueprintCompilerExtension::DoesActorClassHaveComponent(UClass* ActorClass, UClass* RequiredComponentClass)
{
	if (!ActorClass || !RequiredComponentClass)
		return false;

	if (AActor* ActorCDO = Cast<AActor>(ActorClass->GetDefaultObject()))
	{
		if (ActorCDO->FindComponentByClass(RequiredComponentClass))
		{
			return true;
		}
	}

	if (UBlueprintGeneratedClass* BPClass = Cast<UBlueprintGeneratedClass>(ActorClass))
	{
		if (USimpleConstructionScript* SCS = BPClass->SimpleConstructionScript)
		{
			for (USCS_Node* Node : SCS->GetAllNodes())
			{
				if (Node && Node->ComponentClass && Node->ComponentClass->IsChildOf(RequiredComponentClass))
				{
					return true;
				}
			}
		}
	}

	return DoesActorClassHaveComponent(ActorClass->GetSuperClass(), RequiredComponentClass);
}

void UJoltPhysicsBlueprintCompilerExtension::ProcessBlueprintCompiled(const FKismetCompilerContext& CompilationContext, const FBlueprintCompiledData& Data)
{
	TArray<UK2Node_CallFunction*> CallNodes;
	FBlueprintEditorUtils::GetAllNodesOfClass(CompilationContext.Blueprint, CallNodes);

	for (UK2Node_CallFunction* CallNode : CallNodes)
	{
		UFunction* Function = CallNode->GetTargetFunction();
		if (!Function || !Function->HasMetaData(TEXT("RequireActorComponent")))
			continue;

		// Metadata is formatted as "ParamName,ComponentClassName"
		FString ParamName;
		FString ComponentClassName;
		if (!Function->GetMetaData(TEXT("RequireActorComponent")).Split(TEXT(","), &ParamName, &ComponentClassName))
			continue;
		
		ComponentClassName.RemoveFromStart(TEXT("U"));

		UEdGraphPin* ActorPin = CallNode->FindPin(*ParamName);
		if (!ActorPin) continue;

		const bool bDefaultsToSelf = Function->GetMetaData(TEXT("DefaultToSelf")) == ParamName;
		UClass* RequiredComponentClass = FindFirstObject<UClass>(*ComponentClassName);

		if (ActorPin->LinkedTo.Num() == 0)
		{
			// If we default to self - unconnected pin refers to Self
			if (bDefaultsToSelf)
			{
				UClass* SelfClass = CompilationContext.Blueprint->SkeletonGeneratedClass.Get();
				if (!DoesActorClassHaveComponent(SelfClass, RequiredComponentClass))
				{
					CompilationContext.MessageLog.Error(*FString::Printf(TEXT("@@ requires %s to have a %s"), *ParamName, *ComponentClassName), CallNode);
				}
				continue;
			}

			if (ActorPin->DefaultObject == nullptr && ActorPin->DefaultValue.IsEmpty())
			{
				CompilationContext.MessageLog.Error(*FString::Printf(TEXT("@@ has a null %s, requires an Actor with a %s"), *ParamName, *ComponentClassName), CallNode);
			}
			continue;
		}
		
		UEdGraphPin* SourcePin = ActorPin->LinkedTo[0];
		UK2Node* SourceNode = Cast<UK2Node>(SourcePin->GetOwningNode());
		AActor* ReferencedLevelActor = SourceNode ? SourceNode->GetReferencedLevelActor() : nullptr;

		bool bHasComponent;

		// A level actor reference lets us check its components directly
		if (ReferencedLevelActor)
		{
			bHasComponent = ReferencedLevelActor->FindComponentByClass(RequiredComponentClass) != nullptr;
		}
		else
		{
			UClass* ConnectedClass;

			// If self, we resolve against the blueprint's class
			if (SourcePin->PinType.PinSubCategory == UEdGraphSchema_K2::PSC_Self)
			{
				ConnectedClass = CompilationContext.Blueprint->SkeletonGeneratedClass.Get();
			}
			else
			{
				ConnectedClass = Cast<UClass>(SourcePin->PinType.PinSubCategoryObject.Get());
			}

			bHasComponent = ConnectedClass && DoesActorClassHaveComponent(ConnectedClass, RequiredComponentClass);
		}
		
		// Fail compilation when an actor does not have required component
		if (RequiredComponentClass && !bHasComponent)
		{
			CompilationContext.MessageLog.Error(*FString::Printf(TEXT("@@ requires %s to have a %s"), *ParamName, *ComponentClassName), CallNode);
		}
	}
}