// Copyright notice

#pragma once

#include "JoltCallbackOrder.h"
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

	// Run in ascending order
	uint16 AddPrePhysicsCallback(const TDelegate<void(float)>& callback, int32 order = JoltCallbackOrder::Default) { return PrePhysicsCallbacks.Add(callback, order); }

	uint16 AddPostPhysicsCallback(const TDelegate<void(float)>& callback, int32 order = JoltCallbackOrder::Default) { return PostPhysicsCallbacks.Add(callback, order); }

	void StepPhysicsWithCallBacks();

	void StepPhysics();

private:
	static constexpr uint8 MaxPhysicsFrames = 8;

	const FJoltWorkerOptions* WorkerOptions = nullptr;

	FJoltOrderedCallbacks PrePhysicsCallbacks;

	FJoltOrderedCallbacks PostPhysicsCallbacks;

	JPH::PhysicsSystem* PhysicsSystem = nullptr;

	JPH::TempAllocator* TempAllocator = nullptr;

	JPH::JobSystem* JobSystem = nullptr;
};
