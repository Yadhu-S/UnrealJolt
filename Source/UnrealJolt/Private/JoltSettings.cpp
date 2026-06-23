// Pixel Di

#include "JoltSettings.h"

static const FName StaticLayerName(TEXT("Static"));
static const FName DynamicLayerName(TEXT("Dynamic"));

constexpr int32 MaxBroadphaseLayerCount = 255; // Jolt's JPH::BroadPhaseLayer is a uint8
constexpr int32 MaxObjectLayerCount = 65535;   // Jolt's JPH::ObjectLayer is a uint16

UJoltSettings::UJoltSettings(const FObjectInitializer& obj)
{
	MaxBodies = 65536;
	NumBodyMutexes = 0;
	MaxBodyPairs = 65536;
	MaxContactConstraints = 10240;
	bEnableDebugRenderer = true;
	bDebugDrawStaticBodies = true;
	bDebugDrawDynamicBodies = true;
	bDebugDrawKinematicBodies = true;
	bDebugDrawHeightFields = true;
	CustomBodyIDStart = 0;
	StaticBodyIDStart = 21845,
	DynamicBodyIDStart = 43690;
	MaxPhysicsJobs = 2048;
	MaxPhysicsBarriers = 8;
	MaxThreads = 2;
	TickRate = 60;
	FixedDeltaTime = 1.0f / 60.0f; // 60Hz
	InCollisionSteps = 1;
	PreAllocatedMemory = 10;	   // 10MB
	bEnableMultithreading = false; // 10MB

	// Default layer setup: two broadphase layers, two object layers. Dynamic collides with
	// everything; Static only collides with Dynamic.
	if (BroadphaseLayers.Num() == 0)
	{
		BroadphaseLayers.Add(FJoltBroadphaseLayer(StaticLayerName));
		BroadphaseLayers.Add(FJoltBroadphaseLayer(DynamicLayerName));
	}

	if (ObjectLayers.Num() == 0)
	{
		FJoltObjectLayer staticLayer;
		staticLayer.Name = StaticLayerName;
		staticLayer.BroadphaseLayer = StaticLayerName;
		staticLayer.CollidesWith.Add(DynamicLayerName);
		ObjectLayers.Add(staticLayer);

		FJoltObjectLayer dynamicLayer;
		dynamicLayer.Name = DynamicLayerName;
		dynamicLayer.BroadphaseLayer = DynamicLayerName;
		dynamicLayer.CollidesWith.Add(StaticLayerName);
		dynamicLayer.CollidesWith.Add(DynamicLayerName);
		ObjectLayers.Add(dynamicLayer);
	}

	if (DefaultDynamicLayer.IsNone())
	{
		DefaultDynamicLayer = DynamicLayerName;
	}

	if (DefaultStaticLayer.IsNone())
	{
		DefaultStaticLayer = StaticLayerName;
	}
}

TArray<FString> UJoltSettings::GetBroadphaseLayerNames() const
{
	TArray<FString> Names;
	Names.Reserve(BroadphaseLayers.Num());
	for (const FJoltBroadphaseLayer& Layer : BroadphaseLayers)
	{
		if (!Layer.Name.IsNone())
		{
			Names.Add(Layer.Name.ToString());
		}
	}
	return Names;
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

	if (PropertyChangedEvent.Property != nullptr && PropertyChangedEvent.Property->GetFName() == GET_MEMBER_NAME_CHECKED(UJoltSettings, bEnableDebugRenderer))
	{
		if (bEnableDebugRenderer)
		{
			bDebugDrawStaticBodies = true;
			bDebugDrawDynamicBodies = true;
			bDebugDrawKinematicBodies = true;
			bDebugDrawHeightFields = true;
		}
	}

	// Layer validation / symmetry enforcement. Runs on any edit under BroadphaseLayers / ObjectLayers
	// (including nested struct member edits and array add/remove/clear). Cheap enough to run on every
	// layer edit since the arrays are tiny.
	const FName		   changedPropName = PropertyChangedEvent.Property != nullptr ? PropertyChangedEvent.Property->GetFName() : NAME_None;
	const FName		   memberPropName = PropertyChangedEvent.MemberProperty != nullptr ? PropertyChangedEvent.MemberProperty->GetFName() : NAME_None;
	static const FName BroadphaseLayersName = GET_MEMBER_NAME_CHECKED(UJoltSettings, BroadphaseLayers);
	static const FName ObjectLayersName = GET_MEMBER_NAME_CHECKED(UJoltSettings, ObjectLayers);
	static const FName DefaultDynamicLayerName = GET_MEMBER_NAME_CHECKED(UJoltSettings, DefaultDynamicLayer);
	static const FName DefaultStaticLayerName = GET_MEMBER_NAME_CHECKED(UJoltSettings, DefaultStaticLayer);

	const bool bLayersTouched = memberPropName == BroadphaseLayersName || memberPropName == ObjectLayersName || memberPropName == DefaultDynamicLayerName || memberPropName == DefaultStaticLayerName || changedPropName == BroadphaseLayersName || changedPropName == ObjectLayersName;

	if (!bLayersTouched)
	{
		return;
	}

	if (BroadphaseLayers.Num() > MaxBroadphaseLayerCount)
	{
		BroadphaseLayers.SetNum(MaxBroadphaseLayerCount);
	}

	if (ObjectLayers.Num() > MaxObjectLayerCount)
	{
		ObjectLayers.SetNum(MaxObjectLayerCount);
	}

	// Static and Dynamic are required layers — restore them silently if deleted.
	auto EnsureBroadphaseLayer = [this](FName LayerName)
	{
		const bool bExists = BroadphaseLayers.ContainsByPredicate(
			[&](const FJoltBroadphaseLayer& L) { return L.Name == LayerName; });
		if (!bExists)
		{
			BroadphaseLayers.Insert(FJoltBroadphaseLayer(LayerName), 0);
		}
	};
	EnsureBroadphaseLayer(StaticLayerName);
	EnsureBroadphaseLayer(DynamicLayerName);

	auto EnsureObjectLayer = [this](FName LayerName)
	{
		const bool bExists = ObjectLayers.ContainsByPredicate(
			[&](const FJoltObjectLayer& L) { return L.Name == LayerName; });
		if (!bExists)
		{
			FJoltObjectLayer Layer;
			Layer.Name = LayerName;
			Layer.BroadphaseLayer = LayerName;
			ObjectLayers.Insert(Layer, 0);
		}
	};
	EnsureObjectLayer(StaticLayerName);
	EnsureObjectLayer(DynamicLayerName);

	// Build a set of valid object layer names so we can filter out stale references.
	TSet<FName> validObjectLayerNames;
	validObjectLayerNames.Reserve(ObjectLayers.Num());
	for (const FJoltObjectLayer& layer : ObjectLayers)
	{
		if (!layer.Name.IsNone())
		{
			validObjectLayerNames.Add(layer.Name);
		}
	}

	TSet<FName> validBroadphaseNames;
	validBroadphaseNames.Reserve(BroadphaseLayers.Num());
	for (const FJoltBroadphaseLayer& layer : BroadphaseLayers)
	{
		if (!layer.Name.IsNone())
		{
			validBroadphaseNames.Add(layer.Name);
		}
	}

	// Drop any CollidesWith entries pointing at deleted/renamed layers, then mirror the edit so the
	// collision relation stays symmetric (A collides with B ⇒ B collides with A).
	for (FJoltObjectLayer& layer : ObjectLayers)
	{
		TSet<FName> cleaned;
		for (const FName& otherName : layer.CollidesWith)
		{
			if (validObjectLayerNames.Contains(otherName))
			{
				cleaned.Add(otherName);
			}
		}
		layer.CollidesWith = MoveTemp(cleaned);
	}

	TMap<FName, int32> nameToIndex;
	for (int32 i = 0; i < ObjectLayers.Num(); ++i)
	{
		if (!ObjectLayers[i].Name.IsNone())
		{
			nameToIndex.Add(ObjectLayers[i].Name, i);
		}
	}

	for (int32 i = 0; i < ObjectLayers.Num(); ++i)
	{
		FJoltObjectLayer& layer = ObjectLayers[i];
		for (const FName& otherName : layer.CollidesWith)
		{
			const int32* otherIdx = nameToIndex.Find(otherName);
			if (otherIdx != nullptr && ObjectLayers.IsValidIndex(*otherIdx))
			{
				ObjectLayers[*otherIdx].CollidesWith.Add(layer.Name);
			}
		}
	}

	// If a layer's broadphase reference is missing, fall back to the first broadphase layer so the
	// filter table still builds cleanly. User will see the reverted value in the details panel.
	const FName fallbackBroadphase = BroadphaseLayers.Num() > 0 ? BroadphaseLayers[0].Name : NAME_None;
	for (FJoltObjectLayer& layer : ObjectLayers)
	{
		if (!validBroadphaseNames.Contains(layer.BroadphaseLayer))
		{
			layer.BroadphaseLayer = fallbackBroadphase;
		}
	}

	// Default layer names must also point at valid object layers.
	if (!validObjectLayerNames.Contains(DefaultDynamicLayer))
	{
		DefaultDynamicLayer = validObjectLayerNames.Contains(DynamicLayerName) ? DynamicLayerName : (ObjectLayers.Num() > 0 ? ObjectLayers[0].Name : NAME_None);
	}

	if (!validObjectLayerNames.Contains(DefaultStaticLayer))
	{
		DefaultStaticLayer = validObjectLayerNames.Contains(StaticLayerName) ? StaticLayerName : (ObjectLayers.Num() > 0 ? ObjectLayers[0].Name : NAME_None);
	}
}
#endif
