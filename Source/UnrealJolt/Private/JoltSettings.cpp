// Pixel Di

#include "JoltSettings.h"

UJoltSettings::UJoltSettings(const FObjectInitializer& obj)
{
	MaxBodies = 65536;
	NumBodyMutexes = 0;
	MaxBodyPairs = 65536;
	MaxContactConstraints = 10240;
	bEnableDebugRenderer = true;
	CustomBodyIDStart = 0;
	StaticBodyIDStart = 21845,
	DynamicBodyIDStart = 43690;
	MaxPhysicsJobs = 2048;
	MaxPhysicsBarriers = 8;
	MaxThreads = 2;
	TickRate = 60;
	FixedDeltaTime = 1.0f / 60.0f; // 60Hz
	InCollisionSteps = 1;
	PreAllocatedMemory = 10; // 10MB
	bEnableMultithreading = false; // 10MB
}

#if WITH_EDITOR
void UJoltSettings::PostEditChangeProperty(FPropertyChangedEvent& PropertyChangedEvent)
{
	Super::PostEditChangeProperty(PropertyChangedEvent);

	if (PropertyChangedEvent.Property != nullptr && PropertyChangedEvent.Property->GetFName() == GET_MEMBER_NAME_CHECKED(UJoltSettings, MaxBodies))
	{
		StaticBodyIDStart = MaxBodies / 3;
		DynamicBodyIDStart = MaxBodies / 3 * 2;
	}

	if (PropertyChangedEvent.Property != nullptr && PropertyChangedEvent.Property->GetFName() == GET_MEMBER_NAME_CHECKED(UJoltSettings, TickRate))
	{

		FixedDeltaTime = 1.0f / static_cast<float>(TickRate);
	}
}
#endif
