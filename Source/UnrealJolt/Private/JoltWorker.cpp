#include "JoltWorker.h"
#include "UnrealJolt/JoltMain.h"
#include "Misc/AssertionMacros.h"
#include "ProfilingDebugging/CpuProfilerTrace.h"

FJoltWorker::FJoltWorker(const FJoltWorkerOptions* workerOptions)
{
	UE_LOG(JoltSubSystemLogs, Log, TEXT("Jolt constructor"));

	WorkerOptions = workerOptions;
	check(WorkerOptions != nullptr);

	PhysicsSystem = WorkerOptions->physicsSystem;
	check(PhysicsSystem != nullptr);

	TempAllocator = new JPH::TempAllocatorImpl(WorkerOptions->cPreAllocatedMemory * 1024 * 1024);

	WorkerOptions->cEnableMultithreading
		// The JobSystemThreadPool is an example implementaion, and should be rewritten using the unreal task system
		? JobSystem = new JPH::JobSystemThreadPool(WorkerOptions->cMaxPhysicsJobs, WorkerOptions->cMaxPhysicsBarriers, WorkerOptions->cMaxThreads)
		: JobSystem = new JPH::JobSystemSingleThreaded(WorkerOptions->cMaxPhysicsJobs);
}

FJoltWorker::~FJoltWorker()
{
	delete PhysicsSystem;
	delete TempAllocator;
	delete JobSystem;
	delete JPH::Factory::sInstance;
	JPH::Factory::sInstance = nullptr;
	JPH::UnregisterTypes();
}

void FJoltWorker::StepPhysicsWithCallBacks()
{
	TRACE_CPUPROFILER_EVENT_SCOPE_STR("Jolt_PhysicsStep");
	for (const TDelegate<void(float)>& preCallback : PrePhysicsCallbacks)
	{
		if (preCallback.IsBound())
		{
			preCallback.Execute(WorkerOptions->cFixedDeltaTime);
		}
	}

	StepPhysics();

	for (const TDelegate<void(float)>& postCallback : PostPhysicsCallbacks)
	{
		if (postCallback.IsBound())
		{
			postCallback.Execute(WorkerOptions->cFixedDeltaTime);
		}
	}
}

void FJoltWorker::StepPhysics()
{
	PhysicsSystem->Update(WorkerOptions->cFixedDeltaTime, WorkerOptions->cInCollisionSteps, TempAllocator, JobSystem);
}
