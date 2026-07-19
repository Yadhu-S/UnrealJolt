// Pixel Di

#pragma once

#include "CoreMinimal.h"
#include "JoltSettings.generated.h"

USTRUCT(BlueprintType)
struct UNREALJOLT_API FJoltBroadphaseLayer
{
	GENERATED_BODY()

	UPROPERTY(Config, EditAnywhere, Category = "Layers")
	FName Name;

	FJoltBroadphaseLayer() = default;
	explicit FJoltBroadphaseLayer(FName InName) : Name(InName) {}
};

USTRUCT(BlueprintType)
struct UNREALJOLT_API FJoltObjectLayer
{
	GENERATED_BODY()

	UPROPERTY(Config, EditAnywhere, Category = "Layers")
	FName Name;

	// Which broadphase layer this object layer maps to. Must match a name in UJoltSettings::BroadphaseLayers.
	UPROPERTY(Config, EditAnywhere, Category = "Layers", meta = (GetOptions = "GetBroadphaseLayerNames"))
	FName BroadphaseLayer;

	// Names of other object layers this layer collides with. Collision is symmetric:
	// UJoltSettings::PostEditChangeProperty mirrors edits so both directions stay in sync.
	UPROPERTY(Config, VisibleAnywhere, Category = "Layers")
	TSet<FName> CollidesWith;
};

/**
 *
 */
UCLASS(config = Jolt)
class UNREALJOLT_API UJoltSettings : public UObject
{
	GENERATED_BODY()
public:
	UJoltSettings(const FObjectInitializer& obj);

	// Returns the names of all broadphase layers. Used by the editor to populate the BroadphaseLayer dropdown.
	UFUNCTION()
	TArray<FString> GetBroadphaseLayerNames() const;
	/*
	 * 	Maximum number of bodies to support.
	 * 	This will be divided by 3
	 * 	each chunk will then be shared between custom, static, and dynamic
	 * 	Increasing this will increase the amount of memory used for simulation
	 * 	https://github.com/jrouwe/JoltPhysics/discussions/917
	 */
	UPROPERTY(Config, EditAnywhere, Category = Settings)
	int32 MaxBodies;

	/*
	 * This will always start from 0
	 * This is for usecases where you are not using automatic BodyID allocation
	 */
	UPROPERTY(Config, VisibleAnywhere, BlueprintReadWrite, Category = "Settings")
	int32 CustomBodyIDStart;

	/*
	 * Starting point of Static BodyID
	 * Will change depending on MaxBodies
	 */
	UPROPERTY(Config, VisibleAnywhere, BlueprintReadOnly, Category = "Settings")
	int32 StaticBodyIDStart;

	/*
	 * Will change depending on MaxBodies
	 */
	UPROPERTY(Config, VisibleAnywhere, BlueprintReadOnly, Category = "Settings")
	int32 DynamicBodyIDStart;

	/*
	 * The world steps for a total of FixedDeltaTime seconds.
	 * This is divided in InCollisionSteps iterations(SubSteps).
	 */
	UPROPERTY(Config, EditAnywhere, BlueprintReadOnly, Category = "Settings")
	int32 InCollisionSteps;

	/*
	 * Number of body mutexes to use. Should be a power of 2 in the range [1, 64], use 0 to auto detect.
	 */
	UPROPERTY(Config, EditAnywhere, Category = Settings)
	int32 NumBodyMutexes;

	/*
	 * Maximum amount of body pairs to process (anything else will fall through the world), this number should generally be much higher than the max amount of contact points as there will be lots of bodies close that are not actually touching.
	 */
	UPROPERTY(Config, EditAnywhere, Category = Settings)
	int32 MaxBodyPairs;

	/*
	 * Maximum amount of contact constraints to process (anything else will fall through the world).
	 */
	UPROPERTY(Config, EditAnywhere, Category = Settings)
	int32 MaxContactConstraints;

	/*
	 *MaxJobs Max number of jobs that can be allocated at any time
	 */
	UPROPERTY(Config, EditAnywhere, Category = Settings)
	int MaxPhysicsJobs;

	/*
	 * Multithreading currently uses the example implementation in jolt, which works but might need a proper implementation as suggesed by jolt
	 * using the task system
	 */
	UPROPERTY(Config, EditAnywhere, Category = Settings)
	bool bEnableMultithreading;

	/*
	 *MaxBarriers Max number of barriers that can be allocated at any time
	 */
	UPROPERTY(Config, EditAnywhere, Category = Settings, meta = (EditCondition = "bEnableMultithreading"))
	int MaxPhysicsBarriers;

	/*
	 *Number of threads to start (the number of concurrent jobs is 1 more because the main thread will also run jobs while waiting for a barrier to complete). Use -1 to auto detect the amount of CPU's.
	 */
	UPROPERTY(Config, EditAnywhere, Category = Settings, meta = (EditCondition = "bEnableMultithreading"))
	int MaxThreads;

	/*
	 * The calculated deltatime between each physics frames. (1/TickRate);
	 */
	UPROPERTY(Config, VisibleAnywhere, Category = Settings)
	float FixedDeltaTime;

	/*
	 * Jolt physics tickrate. This is divided in inCollisionSteps iterations
	 */
	UPROPERTY(Config, EditAnywhere, Category = Settings)
	int TickRate;
	
	/*
	 * Default gravity in Unreal units. Can be overridden by SetGravity. 
	 */
	UPROPERTY(Config, EditAnywhere, Category = Settings)
	FVector DefaultGravity = FVector(0.f, 0.f, -980.f);

	/*
	 * We need a temp allocator for temporary allocations during the physics update. We're
	 * pre-allocating to avoid having to do allocations during the physics update.
	 * Value in MB
	 */
	UPROPERTY(Config, EditAnywhere, Category = Settings)
	int PreAllocatedMemory;

	/*
	 * Jolts debug renderer
	 * currently very slow when rendering landscape shape
	 * TODO: update the draw triangle batch function for faster debug renderer
	 */
	UPROPERTY(Config, EditAnywhere, Category = "Debug Rendering")
	bool bEnableDebugRenderer;

	UPROPERTY(Config, EditAnywhere, Category = "Debug Rendering", meta = (EditCondition = "bEnableDebugRenderer"))
	bool bDebugDrawStaticBodies;

	UPROPERTY(Config, EditAnywhere, Category = "Debug Rendering", meta = (EditCondition = "bEnableDebugRenderer"))
	bool bDebugDrawDynamicBodies;

	UPROPERTY(Config, EditAnywhere, Category = "Debug Rendering", meta = (EditCondition = "bEnableDebugRenderer"))
	bool bDebugDrawKinematicBodies;

	UPROPERTY(Config, EditAnywhere, Category = "Debug Rendering", meta = (EditCondition = "bEnableDebugRenderer"))
	bool bDebugDrawHeightFields;

	/*
	 * Broadphase layers. Each broadphase layer becomes a separate bounding volume tree in Jolt's
	 * broadphase. Typical setups have 2 (Static / Dynamic). Jolt's internal type is uint8 so the
	 * maximum is 255.
	 */
	UPROPERTY(Config, EditAnywhere, Category = "Layers", meta = (TitleProperty = "Name"))
	TArray<FJoltBroadphaseLayer> BroadphaseLayers;

	/*
	 * Object layers. Each body is assigned to one. ObjectLayers[i]'s array index becomes its
	 * JPH::ObjectLayer (uint16) at runtime, so order is load-bearing for code that caches IDs —
	 * prefer using name-based lookup (UJoltSubsystem::ResolveObjectLayer) in new code.
	 */
	UPROPERTY(Config, EditAnywhere, Category = "Layers", meta = (TitleProperty = "Name"))
	TArray<FJoltObjectLayer> ObjectLayers;

	/*
	 * Default layer name used when AddDynamicBody / AddKinematicBody is called without an explicit
	 * layer parameter. Must match an entry in ObjectLayers.
	 */
	UPROPERTY(Config, EditAnywhere, Category = "Layers")
	FName DefaultDynamicLayer;

	/*
	 * Default layer name used when AddStaticBody is called without an explicit layer parameter.
	 * Must match an entry in ObjectLayers.
	 */
	UPROPERTY(Config, EditAnywhere, Category = "Layers")
	FName DefaultStaticLayer;

#if WITH_EDITOR
	virtual void PostEditChangeProperty(FPropertyChangedEvent& PropertyChangedEvent) override;
#endif
};
