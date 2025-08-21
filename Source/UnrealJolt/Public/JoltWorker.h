// Copyright notice

#pragma once

#include "Containers/SpscQueue.h"
#include "CoreMinimal.h"
#include "Delegates/Delegate.h"
#include "UnrealJolt/JoltMain.h"

struct FJoltWorkerOptions
{
	JPH::PhysicsSystem* physicsSystem;
	int					cMaxPhysicsJobs;
	int					cMaxPhysicsBarriers;
	int					cMaxThreads;
	float				cFixedDeltaTime;
	int					cInCollisionSteps;
	int					cPreAllocatedMemory;
	bool				cEnableMultithreading;

	FJoltWorkerOptions(
		JPH::PhysicsSystem* physicsSystem,
		int					cMaxPhysicsJobs,
		int					cMaxPhysicsBarriers,
		int					cMaxThreads,
		float				cFixedDeltaTime,
		int					cInCollisionSteps,
		int					cPreAllocatedMemory,
		bool cEnableMultithreading)

		: physicsSystem(physicsSystem), cMaxPhysicsJobs(cMaxPhysicsJobs), cMaxPhysicsBarriers(cMaxPhysicsBarriers), cMaxThreads(cMaxThreads), cFixedDeltaTime(cFixedDeltaTime), cInCollisionSteps(cInCollisionSteps), cPreAllocatedMemory(cPreAllocatedMemory),cEnableMultithreading(cEnableMultithreading)
	{
	}
};

/**
 *
 */
class UNREALJOLT_API FJoltWorker
{
public:
	explicit FJoltWorker(const FJoltWorkerOptions* workerOptions);

	~FJoltWorker();

	uint16 AddPrePhysicsCallback(const TDelegate<void(float)>& callback) { return PrePhysicsCallbacks.Add(callback); }

	uint16 AddPostPhysicsCallback(const TDelegate<void(float)>& callback) { return PostPhysicsCallbacks.Add(callback); }

	void StepPhysicsWithCallBacks();

	void StepPhysics();

private:
	static constexpr uint8 MaxPhysicsFrames = 8;

	const FJoltWorkerOptions* WorkerOptions = nullptr;

	TArray<TDelegate<void(float)>> PrePhysicsCallbacks;

	TArray<TDelegate<void(float)>> PostPhysicsCallbacks;

	JPH::PhysicsSystem* PhysicsSystem = nullptr;

	JPH::TempAllocator* TempAllocator = nullptr;

	JPH::JobSystem* JobSystem = nullptr;
};
