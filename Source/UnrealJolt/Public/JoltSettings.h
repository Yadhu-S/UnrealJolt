// Pixel Di

#pragma once

#include "CoreMinimal.h"
#include "JoltSettings.generated.h"

/**
 *
 */
UCLASS(config = Jolt)
class UNREALJOLT_API UJoltSettings : public UObject
{
	GENERATED_BODY()
public:
	UJoltSettings(const FObjectInitializer& obj);
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
	UPROPERTY(Config, EditAnywhere, Category = Settings)
	bool bEnableDebugRenderer;

#if WITH_EDITOR
	virtual void PostEditChangeProperty(FPropertyChangedEvent& PropertyChangedEvent) override;
#endif
};
