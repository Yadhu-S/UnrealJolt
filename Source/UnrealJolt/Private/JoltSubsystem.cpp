#include "JoltSubsystem.h"
#include "Components/BoxComponent.h"
#include "Components/CapsuleComponent.h"
#include "Components/SphereComponent.h"
#include "Components/StaticMeshComponent.h"
#include "Components/SplineMeshComponent.h"
#include "Components/ShapeComponent.h"
#include "Containers/Array.h"
#include "Containers/Map.h"
#include "Engine/StaticMesh.h"
#include "EngineUtils.h"
#include "JoltDataAsset.h"
#include "JoltWorker.h"
#include "JoltPhysicsMaterial.h"
#include "UObject/WeakObjectPtrTemplates.h"
#include "UnrealJolt/Helpers.h"
#include "JoltFilters.h"
#include "Engine/Engine.h"
#include "Landscape.h"
#include "LandscapeComponent.h"
#include "LandscapeDataAccess.h"
#include "LandscapeHeightfieldCollisionComponent.h"
#include "LandscapeSplineSegment.h"
#include "LandscapeSplinesComponent.h"
#include "PhysicalMaterials/PhysicalMaterial.h"
#include "Misc/AssertionMacros.h"
#include "PhysicsEngine/BodySetup.h"
#include "Chaos/TriangleMeshImplicitObject.h"
#include "Chaos/Particles.h"
#include "Chaos/ImplicitFwd.h"
#include "Templates/Casts.h"
#include "UObject/Package.h"
#include "UObject/SavePackage.h"
#if WITH_EDITOR
	#include "Editor.h"
#endif

DEFINE_LOG_CATEGORY(JoltSubSystemLogs);

void UJoltSubsystem::Initialize(FSubsystemCollectionBase& Collection)
{

	Super::Initialize(Collection);
	JPH::Trace = JoltHelpers::UETrace;
#ifdef JPH_ENABLE_ASSERTS
	JPH::AssertFailed = JoltHelpers::UEAssertFailed;
#endif
	JPH::RegisterDefaultAllocator();
	JPH::Factory::sInstance = new JPH::Factory();
	JPH::RegisterTypes();
	JoltSettings = GetDefault<UJoltSettings>();
	StaticBodyIDX = JoltSettings->StaticBodyIDStart;
	DynamicBodyIDX = JoltSettings->DynamicBodyIDStart;
	InitPhysicsSystem(JoltSettings->MaxBodies, JoltSettings->NumBodyMutexes, JoltSettings->MaxBodyPairs, JoltSettings->MaxContactConstraints);
}

void UJoltSubsystem::Deinitialize()
{
	Super::Deinitialize();
	UE_LOG(JoltSubSystemLogs, Log, TEXT("Jolt Deinitialize"));

	bIsReady = false;

	for (TPair<uint32, JPH::Body*>& pair : BodyIDBodyMap)
	{
		BodyInterface->RemoveBody(pair.Value->GetID());
		BodyInterface->DestroyBody(pair.Value->GetID());
	}

	delete JoltWorker;

	// delete SaveStateFilterImpl;
#ifdef JPH_DEBUG_RENDERER
	delete JoltDebugRendererImpl;
	delete DrawSettings;
#endif
	delete BroadPhaseLayerInterface;
	delete ObjectVsBroadphaseLayerFilter;
	delete ObjectVsObjectLayerFilter;
	delete WorkerOptions;

	/*
	 * Ref counting memory management as described here
	 * https://jrouwe.github.io/JoltPhysicsDocs/5.2.0/index.html#memory-management
	 */
	for (const JPH::BoxShape*& box : BoxShapes)
	{
		box = nullptr;
	}

	for (const JPH::SphereShape*& sphere : SphereShapes)
	{
		sphere = nullptr;
	}

	for (const JPH::CapsuleShape*& capsule : CapsuleShapes)
	{
		capsule = nullptr;
	}

	for (ConvexHullShapeHolder& convexShape : ConvexShapes)
	{
		convexShape.Shape = nullptr;
	}

	for (const JPH::HeightFieldShapeSettings*& hf : HeightFieldShapes)
	{
		hf = nullptr;
	}

	delete ContactListener;

	/*
	for (const JPH::Body* b : SavedBodies)
	{
		b = nullptr;
	}

	for (TPair<EPhysicalSurface, const JoltPhysicsMaterial*>& Pair : SurfaceJoltPhysicsMaterialMap)
	{
		 delete Pair.Value;
	}
	*/
}

uint16 UJoltSubsystem::AddPrePhysicsCallback(const TDelegate<void(float)>& callback) const
{
	return JoltWorker->AddPrePhysicsCallback(callback);
}

uint16 UJoltSubsystem::AddPostPhysicsCallback(const TDelegate<void(float)>& callback) const
{
	return JoltWorker->AddPostPhysicsCallback(callback);
}

void UJoltSubsystem::AddPostInterpolationCallback(const TDelegate<void(float)>& callback)
{
	PostInterpolationCallbacks.Add(callback);
}

void UJoltSubsystem::RegisterPhysicsListener(AActor* Listener)
{
	if (!Listener)
	{
		UE_LOG(JoltSubSystemLogs, Warning, TEXT("RegisterPhysicsListener: Listener is invalid"))
		return;
	}

	if (Listener->Implements<UJoltPhysicsCallbackInterface>())
	{
		PhysicsListeners.AddUnique(TWeakObjectPtr<UObject>(Listener));
	}
	else
	{
		UE_LOG(JoltSubSystemLogs, Warning, TEXT("RegisterPhysicsListener: %s does not implement IJoltPhysicsCallbackInterface"), *Listener->GetName());
	}
}

void UJoltSubsystem::UnregisterPhysicsListener(AActor* Listener)
{
    if (!Listener)
    {
        UE_LOG(JoltSubSystemLogs, Warning, TEXT("UnregisterPhysicsListener: Listener is invalid"))
        return;
    }
	
    PhysicsListeners.RemoveAll([Listener](const TWeakObjectPtr<>& Entry)
    {
        return Entry.Get() == Listener;
    });
}

void UJoltSubsystem::BroadcastPrePhysicsListeners(float DeltaTime)
{
	for (auto It = PhysicsListeners.CreateIterator(); It; ++It)
	{
		UObject* Obj = It->Get();

		// Prune invalid physics listeners
		if (!IsValid(Obj))
		{
			It.RemoveCurrent();
			continue;
		}

		IJoltPhysicsCallbackInterface::Execute_OnPrePhysicsStep(Obj, DeltaTime);
	}
}

void UJoltSubsystem::BroadcastPostPhysicsListeners(float DeltaTime)
{
	for (auto It = PhysicsListeners.CreateIterator(); It; ++It)
	{
		UObject* Obj = It->Get();

		if (!IsValid(Obj))
		{
			It.RemoveCurrent();
			continue;
		}

		IJoltPhysicsCallbackInterface::Execute_OnPostPhysicsStep(Obj, DeltaTime);
	}
}

void UJoltSubsystem::SetTimeScale(double deltaSeconds)
{
	ConfiguredDeltaSeconds = deltaSeconds;
}

void UJoltSubsystem::SetPaused(bool bPaused)
{
	bStepPaused = bPaused;
	if (bPaused)
	{
		Accumulator = 0.0;
	}
}

void UJoltSubsystem::SetGravity(const FVector& gravity)
{
	if (!MainPhysicsSystem)
		return;
	MainPhysicsSystem->SetGravity(JoltHelpers::ToJoltVec3(gravity));

	// We need to wake up all dynamic bodies once gravity is set
	for (const FJoltBodyActor& JoltBodyActor : JoltBodyActors)
	{
		if (GetBodyInterface()->GetMotionType(*JoltBodyActor.JoltBodyID) == JPH::EMotionType::Dynamic)
			GetBodyInterface()->ActivateBody(*JoltBodyActor.JoltBodyID);
	}
}

FVector UJoltSubsystem::GetGravity() const
{
	if (!MainPhysicsSystem)
		return FVector();
	return JoltHelpers::ToUESize(MainPhysicsSystem->GetGravity());
}

// Called when world is ready to start gameplay before the game mode transitions to the correct state and call BeginPlay on all actors
void UJoltSubsystem::OnWorldBeginPlay(UWorld& InWorld)
{

	Super::OnWorldBeginPlay(InWorld);

	UE_LOG(JoltSubSystemLogs, Log, TEXT("Jolt worker running "));
	AddAllJoltActors(GetWorld());

	if (ALandscape* landscape = FindSingleLandscape(&InWorld))
	{
#if WITH_EDITOR
		GetAllLandscapeHeights(landscape);
		HandleLandscapeMeshes(landscape);
		if (!CookBodies())
		{
			UE_LOG(JoltSubSystemLogs, Error, TEXT("Landscape data package wasn't saved!"));
		}
		UE_LOG(JoltSubSystemLogs, Log, TEXT("Landscape data saved to file"));
#endif
		LoadLandscapeFromDataAsset();
	}

	// We were adding bodies one by one above, so need to call this.
	// TODO: need to look into adding bodies as a batch, as recommended by jolt
	// https://jrouwe.github.io/JoltPhysics/#creating-bodies
	MainPhysicsSystem->OptimizeBroadPhase();
	ConfiguredDeltaSeconds = JoltSettings->FixedDeltaTime;

	WorkerOptions = new FJoltWorkerOptions(
		MainPhysicsSystem,
		JoltSettings->MaxPhysicsJobs,
		JoltSettings->MaxPhysicsBarriers,
		JoltSettings->MaxThreads,
		JoltSettings->FixedDeltaTime,
		JoltSettings->InCollisionSteps,
		JoltSettings->PreAllocatedMemory,
		JoltSettings->bEnableMultithreading);

	JoltWorker = new FJoltWorker(WorkerOptions);

	JoltWorker->AddPrePhysicsCallback(TDelegate<void(float)>::CreateUObject(this, &UJoltSubsystem::BroadcastPrePhysicsListeners));
	JoltWorker->AddPostPhysicsCallback(TDelegate<void(float)>::CreateUObject(this, &UJoltSubsystem::BroadcastPostPhysicsListeners));

	bIsReady = true;
	OnReady.Broadcast();
}

void UJoltSubsystem::AddAllJoltActors(const UWorld* World)
{
	TArray<const AActor*> staticActors;
	TArray<AActor*>		  dynamicActors;

	if (!World)
	{
		UE_LOG(JoltSubSystemLogs, Warning, TEXT("Invalid World context."));
		return;
	}

	FName joltStaticTag = FName("jolt-static");
	FName joltDynamicTag = FName("jolt-dynamic");
	// Iterate over all actors in the world
	for (TActorIterator<AActor> actorItr(World); actorItr; ++actorItr)
	{
		AActor* actor = *actorItr;
		if (!actor)
			continue;
		
		if (actor->FindComponentByClass<UJoltPhysicsComponent>())
			continue;

		if (actor->ActorHasTag(joltStaticTag))
		{
			staticActors.Add(actor);
		}
		else if (actor->ActorHasTag(joltDynamicTag))
		{
			dynamicActors.Add(actor);
		}
	}

	// Might not be needed, but keeping it because I don't want to debug
	// deterministic behaviour changes across multiple instances...
	staticActors.Sort([](const AActor& A, const AActor& B) {
		return A.GetName() < B.GetName();
	});

	dynamicActors.Sort([](const AActor& A, const AActor& B) {
		return A.GetName() < B.GetName();
	});

	for (const AActor*& staticActor : staticActors)
	{
		// FIXME: read friction and restitution from the physics material
		AddStaticBody(staticActor, 0.2f, 0.1f);
	}

	for (AActor*& dynamicActor : dynamicActors)
	{
		// FIXME: Read all this values from editor
		AddDynamicBody(dynamicActor, 0.2f, 0.1f, 100.0f);
	}
}

void UJoltSubsystem::Tick(float deltaSeconds)
{
	TRACE_CPUPROFILER_EVENT_SCOPE(UJoltSubsystem::Tick);
	Super::Tick(deltaSeconds);

	if (JoltWorker == nullptr)
	{
		return;
	}

	if (!bStepPaused)
	{
		Accumulator += deltaSeconds;

		while (Accumulator >= ConfiguredDeltaSeconds)
		{
			StepPhysics();
			Accumulator -= ConfiguredDeltaSeconds;
		}
	}

	const double alpha = Accumulator / ConfiguredDeltaSeconds;
	PhysicsAlpha_ = alpha;

	InterpolatePhysicsFrame(alpha);

	for (const TDelegate<void(float)>& cb : PostInterpolationCallbacks)
	{
		if (cb.IsBound())
			cb.Execute(deltaSeconds);
	}
}

void UJoltSubsystem::StepPhysics(bool bWithCallbacks)
{
	bWithCallbacks
		? JoltWorker->StepPhysicsWithCallBacks()
		: JoltWorker->StepPhysics();

	RecordFrames();
#ifdef JPH_DEBUG_RENDERER
	DrawDebugLines();
#endif
}

void UJoltSubsystem::RecordFrames()
{

	for (FJoltBodyActor& Entry : JoltBodyActors)
	{
		Entry.FrameHistory.PrevLocation = Entry.FrameHistory.CurrentLocation;
		Entry.FrameHistory.PrevRotation = Entry.FrameHistory.CurrentRotation;

		FTransform CurrentTransform;
		JoltGetPhysicsTransform(*Entry.JoltBodyID, CurrentTransform);
		ApplyLocalTxIfAny(Entry.JoltBodyID, CurrentTransform);
		Entry.FrameHistory.CurrentLocation = CurrentTransform.GetLocation();
		Entry.FrameHistory.CurrentRotation = CurrentTransform.GetRotation().Rotator();
	}
}

void UJoltSubsystem::InterpolatePhysicsFrame(const double& alpha)
{
	TRACE_CPUPROFILER_EVENT_SCOPE(UJoltSubsystem::InterpolatePhysicsFrame);
	for (const FJoltBodyActor& JoltBodyActor : JoltBodyActors)
	{
		auto OwningActor = JoltBodyActor.Actor.Pin();
		if (OwningActor == nullptr)
			continue;

		OwningActor->SetActorLocationAndRotation(
			FMath::Lerp(
				JoltBodyActor.FrameHistory.PrevLocation,
				JoltBodyActor.FrameHistory.CurrentLocation,
				alpha),
			FQuat::Slerp(
				JoltBodyActor.FrameHistory.PrevRotation.Quaternion(),
				JoltBodyActor.FrameHistory.CurrentRotation.Quaternion(),
				alpha),
			false, nullptr, ETeleportType::TeleportPhysics);
	}
}

void UJoltSubsystem::ApplyLocalTxIfAny(const JPH::BodyID* bodyID, FTransform& actorTransform) const
{
	const FTransform* localTx = SkeletalMeshBodyIDLocalTransformMap.Find(bodyID);
	if (localTx == nullptr)
	{
		return;
	}
	actorTransform = localTx->Inverse() * actorTransform;
}

TStatId UJoltSubsystem::GetStatId() const
{
	// This provides Unreal with performance tracking and profiling stats.
	RETURN_QUICK_DECLARE_CYCLE_STAT(UMyTickableWorldSubsystem, STATGROUP_Tickables);
}

// Called after world components (e.g. line batches and all level components) have been updated
void UJoltSubsystem::OnWorldComponentsUpdated(UWorld& InWorld)
{
	Super::OnWorldComponentsUpdated(InWorld);
}

void UJoltSubsystem::InitPhysicsSystem(
	int cMaxBodies,
	int cNumBodyMutexes,
	int cMaxBodyPairs,
	int cMaxContactConstraints)
{

#ifdef JPH_DEBUG_RENDERER
	DrawSettings = new JPH::BodyManager::DrawSettings;
	DrawSettings->mDrawShape = true;		// Draw the shapes of the bodies
	DrawSettings->mDrawBoundingBox = false; // Optionally, draw bounding boxes
	DrawSettings->mDrawShapeWireframe = false;
	DrawSettings->mDrawWorldTransform = true;
	// DrawSettings->mDrawShapeWireframe
#endif

	// Build the runtime layer table from UJoltSettings before constructing the filter classes —
	// they hold a reference to it for their entire lifetime.
	LayerTable = FJoltLayerTable::BuildFromSettings(*JoltSettings);

	BroadPhaseLayerInterface = new BPLayerInterfaceImpl(LayerTable);
	// Create class that filters object vs broadphase layers
	// Note: As this is an interface, PhysicsSystem will take a reference to this so this instance needs to stay alive!
	ObjectVsBroadphaseLayerFilter = new ObjectVsBroadPhaseLayerFilterImpl(LayerTable);

	// Create class that filters object vs object layers
	// Note: As this is an interface, PhysicsSystem will take a reference to this so this instance needs to stay alive!
	ObjectVsObjectLayerFilter = new ObjectLayerPairFilterImpl(LayerTable);

	MainPhysicsSystem = new JPH::PhysicsSystem;

#ifdef JPH_DEBUG_RENDERER
	JoltDebugRendererImpl = new UEJoltDebugRenderer(GetWorld());
#endif
	// Set gravity according to the default gravity vector in settings
	MainPhysicsSystem->SetGravity(JoltHelpers::ToJoltVec3(JoltSettings->DefaultGravity));
	MainPhysicsSystem->Init(
		cMaxBodies,
		cNumBodyMutexes,
		cMaxBodyPairs,
		cMaxContactConstraints,
		*BroadPhaseLayerInterface,
		*ObjectVsBroadphaseLayerFilter,
		*ObjectVsObjectLayerFilter);

	BodyInterface = &MainPhysicsSystem->GetBodyInterface();
	ContactListener = new UEJoltCallBackContactListener();
	MainPhysicsSystem->SetContactListener(ContactListener);
	// Spawn jolt worker
	UE_LOG(JoltSubSystemLogs, Log, TEXT("Jolt subsystem init complete"));
}

int64 UJoltSubsystem::AddDynamicBody(AActor* body, const float& friction, const float& restitution, const float& mass, FName Layer)
{

	int64 ID = JPH::BodyID::cInvalidBodyID;
	ExtractPhysicsGeometry(body, [body, this, friction, restitution, mass, Layer, &ID](const JPH::Shape* shape, const FTransform& RelTransform) {
		// Every sub-collider in the actor is passed to this callback function
		// We're baking this in world space, so apply actor transform to relative
		const FTransform   FinalXform = body->GetActorTransform();
		const JPH::BodyID* joltBodyID = AddDynamicBodyCollision(shape, FinalXform, friction, restitution, mass, Layer);
		if (joltBodyID != nullptr)
		{
			JoltBodyActors.Emplace(joltBodyID, body);
			ID = joltBodyID->GetIndexAndSequenceNumber();
		}
	});
	return ID;
}

int64 UJoltSubsystem::AddStaticBody(const AActor* Body, const float& Friction, const float& Restitution, FName Layer)
{
	int64 ID = JPH::BodyID::cInvalidBodyID;
	ExtractPhysicsGeometry(Body, [Body, this, Friction, Restitution, Layer, &ID](const JPH::Shape* Shape, const FTransform& RelTransform) mutable {
		// Every sub-collider in the actor is passed to this callback function
		// We're baking this in world space, so apply actor transform to relative
		// const FTransform FinalXform = RelTransform * Body->GetActorTransform();
		const FTransform FinalXform = Body->GetActorTransform();
		if (const JPH::BodyID* bodyID = AddStaticBodyCollision(Shape, FinalXform, Friction, Restitution, Layer))
			ID = bodyID->GetIndexAndSequenceNumber();
	});
	return ID;
}

void UJoltSubsystem::ExtractPhysicsGeometry(const AActor* actor, PhysicsGeometryCallback callback)
{
	TInlineComponentArray<UStaticMeshComponent*, 20> Components;
	const FTransform								 actorTransform = actor->GetActorTransform();

	actor->GetComponents(UStaticMeshComponent::StaticClass(), Components);
	for (UStaticMeshComponent*& Comp : Components)
	{
		if (Comp->IsSimulatingPhysics())
		{
			UE_LOG(JoltSubSystemLogs, Error, TEXT("'Simulate physics' turned on for : '%s' which is marked as a jolt body, disabling chaos"), *Comp->GetOwner()->GetActorNameOrLabel());
			Comp->SetSimulatePhysics(false);
		}
		ExtractPhysicsGeometry(Comp, actorTransform, callback);
	}
}

void UJoltSubsystem::ExtractPhysicsGeometry(const UStaticMeshComponent* SMC, const FTransform& actorTransform, PhysicsGeometryCallback callback)
{
	UStaticMesh* Mesh = SMC->GetStaticMesh();
	if (!Mesh)
		return;

	switch (Mesh->GetBodySetup()->CollisionTraceFlag)
	{

		case ECollisionTraceFlag::CTF_UseComplexAsSimple:
			ExtractComplexPhysicsGeometry(actorTransform, Mesh->GetBodySetup(), Mesh->GetName(), callback);
			break;
		default:
			ExtractPhysicsGeometry(actorTransform, Mesh->GetBodySetup(), callback);
			break;
	}
}

void UJoltSubsystem::ExtractComplexPhysicsGeometry(const FTransform& xformSoFar, const UBodySetup* bodySetup, const FString& meshName, PhysicsGeometryCallback callback)
{
	// Grab the trimesh on UBodySetup
	if (!bodySetup || bodySetup->TriMeshGeometries.Num() == 0)
	{
		UE_LOG(JoltSubSystemLogs, Error, TEXT("No cooked tri-mesh on BodySetup for '%s' — complex collision skipped. Re-cook the asset or set bNeverNeedsCookedCollisionData intentionally."),
			*meshName);
		return;
	}

	const Chaos::FTriangleMeshImplicitObjectPtr& TriMesh = bodySetup->TriMeshGeometries[0];
	if (!TriMesh.IsValid())
	{
		UE_LOG(JoltSubSystemLogs, Error, TEXT("Cooked tri-mesh on BodySetup for '%s' is invalid — complex collision skipped."), *meshName);
		return;
	}

	const FVector scale = xformSoFar.GetScale3D();

	JPH::VertexList			 vertices;
	JPH::IndexedTriangleList triangles;
	JPH::PhysicsMaterialList physicsMaterialList;
	const int				 MaterialIDX = 0;

	const auto&						  Particles = TriMesh->Particles();
	const Chaos::FTrimeshIndexBuffer& Elements = TriMesh->Elements();
	const int32						  NumVerts = Particles.Size();
	const int32						  NumTris = Elements.GetNumTriangles();

	vertices.reserve(NumVerts);
	triangles.reserve(NumTris);

	for (int32 i = 0; i < NumVerts; ++i)
	{
		const Chaos::TVec3<Chaos::FRealSingle>& P = Particles.GetX(i);
		vertices.push_back(JoltHelpers::ToJoltFloat3(
			FVector3f(P[0] * static_cast<float>(scale.X),
				P[1] * static_cast<float>(scale.Y),
				P[2] * static_cast<float>(scale.Z))));
	}

	auto pushTri = [&](uint32 a, uint32 b, uint32 c) {
		if (a < static_cast<uint32>(NumVerts) && b < static_cast<uint32>(NumVerts) && c < static_cast<uint32>(NumVerts))
		{
			triangles.push_back(JPH::IndexedTriangle(a, c, b, MaterialIDX)); // Swap b and c so the triangles face outward
		}
		else
		{
			UE_LOG(JoltSubSystemLogs, Error, TEXT("Invalid triangle indices in cooked tri-mesh for '%s'!"), *meshName);
		}
	};

	if (Elements.RequiresLargeIndices())
	{
		for (const Chaos::TVec3<int32>& T : Elements.GetLargeIndexBuffer())
		{
			pushTri(static_cast<uint32>(T[0]), static_cast<uint32>(T[1]), static_cast<uint32>(T[2]));
		}
	}
	else
	{
		for (const Chaos::TVec3<uint16>& T : Elements.GetSmallIndexBuffer())
		{
			pushTri(static_cast<uint32>(T[0]), static_cast<uint32>(T[1]), static_cast<uint32>(T[2]));
		}
	}

	physicsMaterialList.push_back(GetJoltPhysicsMaterial(bodySetup->GetPhysMaterial()));

	if (vertices.empty() || triangles.empty())
	{
		UE_LOG(JoltSubSystemLogs, Error, TEXT("Skipping complex collision for '%s': empty cooked tri-mesh. Verts=%llu Tris=%llu"),
			*meshName,
			static_cast<uint64>(vertices.size()),
			static_cast<uint64>(triangles.size()));
		return;
	}

	// TODO: Caching mechanism for MeshShapes
	JPH::MeshShapeSettings	meshSettings(vertices, triangles, physicsMaterialList);
	JPH::Shape::ShapeResult res = meshSettings.Create();

	if (!res.IsValid())
	{
		UE_LOG(JoltSubSystemLogs, Error, TEXT("Failed to create mesh for '%s'. Error: %s"), *meshName, *FString(res.GetError().c_str()));
		return;
	}
	callback(res.Get(), xformSoFar);
}

void UJoltSubsystem::RayCastNarrowPhase(const FVector& start, const FVector& end, const FNarrowPhaseQueryDelegate& hitCallback)
{
	if (!hitCallback.IsBound())
	{
		UE_LOG(JoltSubSystemLogs, Error, TEXT("hitcallback not bound"));
		return;
	}

	RayCastNarrowPhase(
		start,
		end,
		[&hitCallback](const FVector& hitLoc, const FVector& hitNormal, const bool& hasHit, const uint32& bodyID, const UPhysicalMaterial*) {
			// TODO: add support to return material
			hitCallback.Execute(hitLoc, hitNormal, (hasHit), bodyID);
		});
}

void UJoltSubsystem::ExtractPhysicsGeometry(const FTransform& xformSoFar, const UBodySetup* bodySetup, PhysicsGeometryCallback callback)
{
	const FVector				scale = xformSoFar.GetScale3D();
	const JPH::Shape*			joltShape = nullptr;
	JPH::CompoundShapeSettings* compoundShapeSettings = nullptr;

	if (!ensure(bodySetup != nullptr))
	{
		return;
	}

	const JoltPhysicsMaterial* physicsMaterial = GetJoltPhysicsMaterial(bodySetup->GetPhysMaterial());

	//  if the total makes up more than 1, we have a compound shape configured in USkeletalMeshComponent
	if (bodySetup->AggGeom.BoxElems.Num() + bodySetup->AggGeom.SphereElems.Num() + bodySetup->AggGeom.SphylElems.Num() > 1)
	{
		compoundShapeSettings = new JPH::StaticCompoundShapeSettings();
	}

	for (const FKBoxElem& ueBox : bodySetup->AggGeom.BoxElems)
	{
		FVector Dimensions = FVector(ueBox.X, ueBox.Y, ueBox.Z) * scale;
		// We'll re-use based on just the LxWxH, including actor scale
		// Rotation and centre will be baked in world space
		const JPH::BoxShape* joltBox = GetBoxCollisionShape(Dimensions, physicsMaterial);
		joltShape = joltBox;

		if (compoundShapeSettings)
		{
			compoundShapeSettings->AddShape(
				JoltHelpers::ToJoltVec3(ueBox.GetTransform().GetLocation()),
				JoltHelpers::ToJoltRot(ueBox.GetTransform().GetRotation()),
				joltShape);
			continue;
		}
		FTransform ShapeXform(ueBox.Rotation, ueBox.Center);
		// Shape transform adds to any relative transform already here
		FTransform XForm = ShapeXform * xformSoFar;
		callback(joltShape, XForm);
	}
	for (const FKSphereElem& ueSphere : bodySetup->AggGeom.SphereElems)
	{
		// Only support uniform scale so use X
		const JPH::SphereShape* joltSphere = GetSphereCollisionShape(ueSphere.Radius * scale.X, physicsMaterial);
		joltShape = joltSphere;
		if (compoundShapeSettings)
		{
			compoundShapeSettings->AddShape(
				JoltHelpers::ToJoltVec3(ueSphere.GetTransform().GetLocation()),
				JoltHelpers::ToJoltRot(ueSphere.GetTransform().GetRotation()),
				joltShape);
			continue;
		}

		FTransform ShapeXform(FRotator::ZeroRotator, ueSphere.Center);
		// Shape transform adds to any relative transform already here
		FTransform XForm = ShapeXform * xformSoFar;
		callback(joltShape, XForm);
	}
	// Sphyl == Capsule (??)
	for (const FKSphylElem& Capsule : bodySetup->AggGeom.SphylElems)
	{
		// X scales radius, Z scales height
		const JPH::CapsuleShape* capsule = GetCapsuleCollisionShape(Capsule.Radius * scale.X, Capsule.Length * scale.Z, physicsMaterial);
		joltShape = capsule;
		if (compoundShapeSettings)
		{
			compoundShapeSettings->AddShape(
				JoltHelpers::ToJoltVec3(Capsule.GetTransform().GetLocation()),
				JoltHelpers::ToJoltRot(Capsule.GetTransform().GetRotation()),
				joltShape);
			continue;
		}

		FTransform ShapeXform(Capsule.GetTransform().GetRotation(), Capsule.Center);
		// Shape transform adds to any relative transform already here
		FTransform XForm = ShapeXform * xformSoFar;
		callback(joltShape, XForm);
	}

	// Convex hull
	for (uint16 i = 0; const FKConvexElem& ConVexElem : bodySetup->AggGeom.ConvexElems)
	{
		const JPH::ConvexHullShape* convexHull = GetConvexHullCollisionShape(bodySetup, i, scale);
		joltShape = convexHull;
		i++;
		if (compoundShapeSettings)
		{
			compoundShapeSettings->AddShape(
				JoltHelpers::ToJoltVec3(ConVexElem.GetTransform().GetLocation()),
				JoltHelpers::ToJoltRot(ConVexElem.GetTransform().GetRotation()),
				joltShape);
			continue;
		}

		callback(joltShape, xformSoFar);
	}

	if (compoundShapeSettings)
	{
		joltShape = compoundShapeSettings->Create().Get();
		callback(joltShape, xformSoFar);
		delete compoundShapeSettings;
	}
}

const JPH::Shape* UJoltSubsystem::ProcessShapeElement(const UShapeComponent* shapeComponent)
{
	if (!shapeComponent)
	{
		UE_LOG(LogTemp, Warning, TEXT("Invalid Shape Component"));
		return nullptr;
	}

	if (const USphereComponent* sphereComponent = Cast<const USphereComponent>(shapeComponent))
	{
		return GetSphereCollisionShape(sphereComponent->GetScaledSphereRadius());
	}
	else if (const UBoxComponent* boxComponent = Cast<const UBoxComponent>(shapeComponent))
	{
		FVector BoxElem = boxComponent->GetScaledBoxExtent();
		return GetBoxCollisionShape(FVector(BoxElem.X, BoxElem.Y, BoxElem.Z));
	}
	else if (const UCapsuleComponent* capsuleComponent = Cast<const UCapsuleComponent>(shapeComponent))
	{
		return GetCapsuleCollisionShape(capsuleComponent->GetScaledCapsuleRadius(), capsuleComponent->GetScaledCapsuleHalfHeight());
	}

	UE_LOG(LogTemp, Warning, TEXT("Unknown or unsupported UShapeComponent type"));
	return nullptr;
}

const JoltPhysicsMaterial* UJoltSubsystem::GetJoltPhysicsMaterial(const UPhysicalMaterial* UEPhysicsMat)
{
	const EPhysicalSurface surfaceType = UEPhysicsMat->SurfaceType.GetValue();

	if (const JoltPhysicsMaterial** FoundPhysicsMaterial = SurfaceJoltMaterialMap.Find(surfaceType))
	{
		return *FoundPhysicsMaterial;
	}

	JoltPhysicsMaterial* newPhysicsMaterial = JoltHelpers::ToJoltPhysicsMaterial(UEPhysicsMat);
	SurfaceJoltMaterialMap.Add(surfaceType, newPhysicsMaterial);
	SurfaceUEMaterialMap.Add(surfaceType, TWeakObjectPtr<const UPhysicalMaterial>(UEPhysicsMat));
	return newPhysicsMaterial;
}

const JoltPhysicsMaterial* UJoltSubsystem::GetOrCreateJoltMaterialForSurface(const EPhysicalSurface surfaceType, const float friction, const float restitution)
{
	if (const JoltPhysicsMaterial** FoundPhysicsMaterial = SurfaceJoltMaterialMap.Find(surfaceType))
	{
		if (((*FoundPhysicsMaterial)->Friction != friction || (*FoundPhysicsMaterial)->Restitution != restitution))
		{
			UE_LOG(JoltSubSystemLogs, Warning,
				TEXT("Conflicting values for physics materials. Cook stale, re cook"));
		}
		return *FoundPhysicsMaterial;
	}

	JoltPhysicsMaterial* newPhysicsMaterial = new JoltPhysicsMaterial();
	newPhysicsMaterial->Friction = friction;
	newPhysicsMaterial->Restitution = restitution;
	newPhysicsMaterial->SurfaceType = surfaceType;
	SurfaceJoltMaterialMap.Add(surfaceType, newPhysicsMaterial);
	return newPhysicsMaterial;
}

void UJoltSubsystem::RestoreShapeMaterials(const FJoltShapeData& shapeData, const JPH::Ref<JPH::Shape>& loadedShape)
{
	if (shapeData.Materials.IsEmpty())
	{
		return;
	}

	JPH::PhysicsMaterialList materialList;
	materialList.reserve(shapeData.Materials.Num());
	for (const FJoltShapeMaterialData& materialData : shapeData.Materials)
	{
		if (materialData.SurfaceType == FJoltShapeMaterialData::NullMaterialSlot)
		{
			materialList.push_back(nullptr);
			continue;
		}
		materialList.push_back(GetOrCreateJoltMaterialForSurface(static_cast<EPhysicalSurface>(materialData.SurfaceType), materialData.Friction, materialData.Restitution));
	}
	loadedShape->RestoreMaterialState(materialList.data(), static_cast<JPH::uint>(materialList.size()));
}

const UPhysicalMaterial* UJoltSubsystem::GetUEPhysicsMaterial(const JoltPhysicsMaterial* JoltPhysicsMat) const
{
	if (JoltPhysicsMat == nullptr)
	{
		return nullptr;
	}

	const TWeakObjectPtr<const UPhysicalMaterial>* FoundPhysicsMaterial = SurfaceUEMaterialMap.Find(JoltPhysicsMat->SurfaceType);
	if (FoundPhysicsMaterial == nullptr)
	{
		return nullptr;
	}
	return FoundPhysicsMaterial->Get();
}

const JPH::ConvexHullShape* UJoltSubsystem::GetConvexHullCollisionShape(const UBodySetup* bodySetup, int convexIndex, const FVector& scale, const JoltPhysicsMaterial* material)
{
	for (const ConvexHullShapeHolder& S : ConvexShapes)
	{
		if (S.BodySetup != bodySetup || S.HullIndex != convexIndex)
		{
			continue;
		}

		if (!S.Scale.Equals(scale))
		{
			continue;
		}

		if (material && S.Shape->GetMaterial() != material)
		{
			continue;
		}

		return S.Shape;
	}

	const FKConvexElem&	  Elem = bodySetup->AggGeom.ConvexElems[convexIndex];
	JPH::Array<JPH::Vec3> points;
	for (const FVector& P : Elem.VertexData)
	{
		points.push_back(JoltHelpers::ToJoltVec3(P * scale));
	}

	JPH::ConvexHullShapeSettings val(points);
	JPH::Shape::ShapeResult		 result;

	JPH::Ref<JPH::ConvexHullShape> shape = new JPH::ConvexHullShape(val, result);
	shape->AddRef();
	shape->SetMaterial(material);

	ConvexShapes.Add(ConvexHullShapeHolder{ bodySetup, convexIndex, scale, shape });
	return shape;
}

const JPH::BoxShape* UJoltSubsystem::GetBoxCollisionShape(const FVector& dimensions, const JoltPhysicsMaterial* material)
{
	// Simple brute force lookup for now, probably doesn't need anything more clever
	JPH::Vec3 HalfSize = JoltHelpers::ToJoltVec3(dimensions * 0.5);
	for (const JPH::BoxShape*& S : BoxShapes)
	{
		JPH::Vec3 Sz = S->GetHalfExtent();

		if (!FMath::IsNearlyEqual(Sz.GetX(), HalfSize.GetX()) || !FMath::IsNearlyEqual(Sz.GetY(), HalfSize.GetY()) || !FMath::IsNearlyEqual(Sz.GetZ(), HalfSize.GetZ()))
		{
			continue;
		}

		// Material check (if material specified)
		if (material && S->GetMaterial() != material)
		{
			continue;
		}

		return S;
	}

	// Not found, create
	JPH::Ref<JPH::BoxShape> S = new JPH::BoxShape(HalfSize);
	S->AddRef();
	S->SetMaterial(material);
	BoxShapes.Add(S);
	return S;
}

const JPH::SphereShape* UJoltSubsystem::GetSphereCollisionShape(const float& radius, const JoltPhysicsMaterial* material)
{
	// Simple brute force lookup for now, probably doesn't need anything more clever
	float Rad = JoltHelpers::ToJoltSize(radius);

	for (const JPH::SphereShape*& S : SphereShapes)
	{
		if (!FMath::IsNearlyEqual(S->GetRadius(), Rad))
		{
			continue;
		}

		if (material && S->GetMaterial() != material)
		{
			continue;
		}

		return S;
	}

	// Not found, create
	JPH::Ref<JPH::SphereShape> S = new JPH::SphereShape(Rad);
	S->AddRef();
	S->SetMaterial(material);
	SphereShapes.Add(S);

	return S;
}

const JPH::CapsuleShape* UJoltSubsystem::GetCapsuleCollisionShape(const float& radius, const float& height, const JoltPhysicsMaterial* material)
{
	// Simple brute force lookup for now, probably doesn't need anything more clever
	float R = JoltHelpers::ToJoltSize(radius);
	float H = JoltHelpers::ToJoltSize(height);
	float HalfH = H * 0.5f;

	for (const JPH::CapsuleShape*& S : CapsuleShapes)
	{
		if (!FMath::IsNearlyEqual(S->GetRadius(), R) || !FMath::IsNearlyEqual(S->GetHalfHeightOfCylinder(), HalfH))
		{
			continue;
		}

		if (material && S->GetMaterial() != material)
		{
			continue;
		}

		return S;
	}

	JPH::Ref<JPH::CapsuleShape> capsule = new JPH::CapsuleShape(HalfH, R);
	capsule->AddRef();
	capsule->SetMaterial(material);
	CapsuleShapes.Add(capsule);

	return capsule;
}

const JPH::BodyID* UJoltSubsystem::AddDynamicBodyCollision(const JPH::BodyID& bodyID, const JPH::Shape* shape, const FTransform& initialWorldTransform, float friction, float restitution, float mass, FName layerName)
{
	JPH::BodyCreationSettings shapeSettings(
		shape,
		JoltHelpers::ToJoltPos(initialWorldTransform.GetLocation()),
		JoltHelpers::ToJoltRot(initialWorldTransform.GetRotation()),
		JPH::EMotionType::Dynamic,
		ResolveDynamicLayer(layerName));

	// Override mass, and calculate inerta
	JPH::MassProperties msp;
	msp.ScaleToMass(mass);
	shapeSettings.mMassPropertiesOverride = msp;
	shapeSettings.mOverrideMassProperties = JPH::EOverrideMassProperties::CalculateInertia;

	return AddBodyToSimulation(&bodyID, shapeSettings, friction, restitution);
}

const JPH::BodyID* UJoltSubsystem::AddDynamicBodyForExternalOwner(
	const JPH::BodyID& bodyID,
	const JPH::Shape*  shape,
	const FTransform&  initialWorldTransform,
	float friction, float restitution, float mass,
	FName layerName)
{
	return AddDynamicBodyCollision(bodyID, shape, initialWorldTransform, friction, restitution, mass, layerName);
}

const JPH::BodyID* UJoltSubsystem::AddDynamicBodyCollision(const JPH::Shape* shape, const FTransform& initialWorldTransform, float friction, float restitution, float mass, FName layerName)
{
	JPH::BodyCreationSettings shapeSettings(
		shape,
		JoltHelpers::ToJoltPos(initialWorldTransform.GetLocation()),
		JoltHelpers::ToJoltRot(initialWorldTransform.GetRotation()),
		JPH::EMotionType::Dynamic,
		ResolveDynamicLayer(layerName));

	// Override mass, and calculate inertia
	JPH::MassProperties msp;
	msp.ScaleToMass(mass);
	shapeSettings.mMassPropertiesOverride = msp;
	shapeSettings.mOverrideMassProperties = JPH::EOverrideMassProperties::CalculateInertia;

	DynamicBodyIDX++;
	JPH::BodyID* bodyID = new JPH::BodyID(DynamicBodyIDX);
	return AddBodyToSimulation(bodyID, shapeSettings, friction, restitution);
}

const JPH::BodyID* UJoltSubsystem::AddStaticBodyCollision(const JPH::Shape* shape, const FTransform& transform, float friction, float restitution, FName layerName)
{
	check(shape != nullptr);
	JPH::BodyCreationSettings shapeSettings(
		shape,
		JoltHelpers::ToJoltPos(transform.GetLocation()),
		JoltHelpers::ToJoltRot(transform.GetRotation()),
		JPH::EMotionType::Static,
		ResolveStaticLayer(layerName));

	StaticBodyIDX++;
	JPH::BodyID* bodyID = new JPH::BodyID(StaticBodyIDX);
	return AddBodyToSimulation(bodyID, shapeSettings, friction, restitution);
}

const JPH::BodyID* UJoltSubsystem::AddStaticBodyCollision(const JPH::BodyID& bodyID, const JPH::Shape* shape, const FTransform& initialWorldTransform, float friction, float restitution, FName layerName)
{
	JPH::BodyCreationSettings shapeSettings(
		shape,
		JoltHelpers::ToJoltPos(initialWorldTransform.GetLocation()),
		JoltHelpers::ToJoltRot(initialWorldTransform.GetRotation()),
		JPH::EMotionType::Static,
		ResolveStaticLayer(layerName));

	return AddBodyToSimulation(&bodyID, shapeSettings, friction, restitution);
}

const JPH::BodyID* UJoltSubsystem::AddKinematicBodyCollision(const JPH::BodyID& bodyID, const JPH::Shape* shape, const FTransform& initialWorldTransform, float friction, float restitution, float mass, FName layerName)
{
	JPH::BodyCreationSettings shapeSettings(
		shape,
		JoltHelpers::ToJoltPos(initialWorldTransform.GetLocation()),
		JoltHelpers::ToJoltRot(initialWorldTransform.GetRotation()),
		JPH::EMotionType::Kinematic,
		ResolveDynamicLayer(layerName));

	JPH::MassProperties msp;
	msp.ScaleToMass(mass);
	shapeSettings.mMassPropertiesOverride = msp;
	shapeSettings.mOverrideMassProperties = JPH::EOverrideMassProperties::CalculateInertia;

	return AddBodyToSimulation(&bodyID, shapeSettings, friction, restitution);
}

const JPH::BodyID* UJoltSubsystem::AddKinematicBodyCollision(const JPH::Shape* shape, const FTransform& initialWorldTransform, float friction, float restitution, float mass, FName layerName)
{
	JPH::BodyCreationSettings shapeSettings(
		shape,
		JoltHelpers::ToJoltPos(initialWorldTransform.GetLocation()),
		JoltHelpers::ToJoltRot(initialWorldTransform.GetRotation()),
		JPH::EMotionType::Kinematic,
		ResolveDynamicLayer(layerName));

	JPH::MassProperties msp;
	msp.ScaleToMass(mass);
	shapeSettings.mMassPropertiesOverride = msp;
	shapeSettings.mOverrideMassProperties = JPH::EOverrideMassProperties::CalculateInertia;

	DynamicBodyIDX++;
	JPH::BodyID* bodyID = new JPH::BodyID(DynamicBodyIDX);
	return AddBodyToSimulation(bodyID, shapeSettings, friction, restitution);
}

const JPH::BodyID* UJoltSubsystem::AddBodyToSimulation(const JPH::BodyID* bodyID, const JPH::BodyCreationSettings& shapeSettings, float friction, float restitution)
{

	check(BodyInterface != nullptr);
	check(bodyID != nullptr);

	// Refuse to create a body with an unresolved layer — Jolt would otherwise stuff cObjectLayerInvalid
	// into the broadphase and trip an assert deeper in the simulation. The Resolve* helpers have already
	// logged which name failed to resolve.
	if (shapeSettings.mObjectLayer == JPH::cObjectLayerInvalid)
	{
		UE_LOG(JoltSubSystemLogs, Error, TEXT("Refusing to create %s body with ID %d: object layer is invalid"),
			*JoltHelpers::EMotionTypeToString(shapeSettings.mMotionType), bodyID->GetIndexAndSequenceNumber());
		return nullptr;
	}

	JPH::Body* createdBody = BodyInterface->CreateBodyWithID(*bodyID, shapeSettings);
	if (createdBody == nullptr)
	{
		UE_LOG(JoltSubSystemLogs, Error, TEXT("failed to create %s body with ID: %d"), *JoltHelpers::EMotionTypeToString(shapeSettings.mMotionType), bodyID->GetIndexAndSequenceNumber());
		return nullptr;
	}
	createdBody->SetRestitution(restitution);
	createdBody->SetFriction(friction);

	BodyIDBodyMap.Add(createdBody->GetID().GetIndexAndSequenceNumber(), createdBody);
	BodyInterface->AddBody(createdBody->GetID(), JPH::EActivation::Activate);
	return bodyID;
}

void UJoltSubsystem::RemoveBodyForExternalOwner(const JPH::BodyID& bodyID)
{
	if (bodyID.IsInvalid() || BodyInterface == nullptr)
		return;

	BodyIDBodyMap.Remove(bodyID.GetIndexAndSequenceNumber());
	BodyInterface->RemoveBody(bodyID);
	BodyInterface->DestroyBody(bodyID);
}

TArray<int32> UJoltSubsystem::CollideShape(const UShapeComponent* shape, const FVector& shapeScale, const FTransform& shapeCOM, const FVector& offset)
{
	JPH::AllHitCollisionCollector<JPH::CollideShapeCollector> collector;

	const JPH::Shape* joltShape = ProcessShapeElement(shape);
	check(joltShape != nullptr);
	check(MainPhysicsSystem != nullptr);

	JPH::CollideShapeSettings settings;
	// settings.mActiveEdgeMode = JPH::EActiveEdgeMode::CollideWithAll;
	settings.mBackFaceMode = JPH::EBackFaceMode::CollideWithBackFaces;

	const USphereComponent* SphereComponent = Cast<const USphereComponent>(shape);

#ifdef JPH_DEBUG_RENDERER
	if (JoltSettings->bEnableDebugRenderer)
	{
		DrawDebugSphere(GetWorld(), shapeCOM.GetLocation(), SphereComponent->GetScaledSphereRadius(), 32, FColor::Magenta, false, 2.0f);
	}
#endif

	MainPhysicsSystem->GetNarrowPhaseQuery().CollideShape(
		joltShape,
		JoltHelpers::ToJoltVec3(shapeScale, false), // We don't want to adjust the scale multiplier
		JoltHelpers::ToJoltTransform(shapeCOM),
		settings,
		JoltHelpers::ToJoltPos(shapeCOM.GetLocation()),
		collector);

	TArray<int32> foundBodyIDs = TArray<int32>();
	for (JPH::CollideShapeResult& val : collector.mHits)
	{
		foundBodyIDs.Add(val.mBodyID2.GetIndexAndSequenceNumber());
	}
	return foundBodyIDs;
}

void UJoltSubsystem::RayCastShapeNarrowPhase(const UShapeComponent* shape, const FVector& shapeScale, const FTransform& shapeCOM, const FVector& offset, NarrowPhaseQueryCallback& hitCallback)
{
	/* FIXME: Probable performance issues because of the bruteforce look up in the cache?
	 * We'll worry about this later
	 */
	const JPH::Shape* joltShape = ProcessShapeElement(shape);

	check(joltShape != nullptr);
	check(MainPhysicsSystem != nullptr);

	JPH::RShapeCast shape_cast{ joltShape, JoltHelpers::ToJoltVec3(shapeScale, false), JoltHelpers::ToJoltTransform(shapeCOM), JPH::Vec3(0, 0, 0) };

	JPH::ShapeCastSettings settings;
	settings.mReturnDeepestPoint = false;
	settings.mBackFaceModeTriangles = JPH::EBackFaceMode::IgnoreBackFaces;
	settings.mBackFaceModeConvex = JPH::EBackFaceMode::IgnoreBackFaces;

	JPH::ClosestHitCollisionCollector<JPH::CastShapeCollector> collector;

	JPH::SpecifiedObjectLayerFilter mov_filter(ResolveObjectLayer(JoltSettings->DefaultDynamicLayer));

	MainPhysicsSystem->GetNarrowPhaseQuery().CastShape(
		shape_cast,
		settings,
		JPH::RVec3::sZero(),
		collector,
		{},
		mov_filter);

	bool val = collector.HadHit();

	hitCallback(
		JoltHelpers::ToUESize(collector.mHit.mContactPointOn2),
		JoltHelpers::ToUESize(-collector.mHit.mPenetrationAxis),
		val,
		collector.mHit.mBodyID2.GetIndexAndSequenceNumber(),
		nullptr); // TODO: add support to return material
}

TArray<FCastShapeResult> UJoltSubsystem::CastShape(const UShapeComponent* shape, const FVector& shapeScale, const FTransform& shapeCOM, const FVector& offset)
{

	const JPH::Shape* joltShape = ProcessShapeElement(shape);
	check(joltShape != nullptr);
	check(MainPhysicsSystem != nullptr);

	const USphereComponent* SphereComponent = Cast<const USphereComponent>(shape);

#ifdef JPH_DEBUG_RENDERER
	if (JoltSettings->bEnableDebugRenderer)
	{
		DrawDebugSphere(GetWorld(), shapeCOM.GetLocation(), SphereComponent->GetScaledSphereRadius(), 32, FColor::Magenta, false, 2.0f);
	}
#endif

	JPH::RShapeCast shape_cast{ joltShape, JoltHelpers::ToJoltVec3(shapeScale, false), JoltHelpers::ToJoltTransform(shapeCOM), JPH::Vec3(0, 0, 0) };

	JPH::ShapeCastSettings settings;
	settings.mReturnDeepestPoint = false;
	settings.mBackFaceModeTriangles = JPH::EBackFaceMode::CollideWithBackFaces;
	settings.mBackFaceModeConvex = JPH::EBackFaceMode::CollideWithBackFaces;

	JPH::AllHitCollisionCollector<JPH::CastShapeCollector> collector;

	MainPhysicsSystem->GetNarrowPhaseQuery().CastShape(
		shape_cast,
		settings,
		JPH::RVec3::sZero(),
		collector);

	TArray<FCastShapeResult> shapeCastResult = TArray<FCastShapeResult>();
	for (JPH::CollideShapeResult& val : collector.mHits)
	{
		shapeCastResult.Add({ val.mBodyID2.GetIndexAndSequenceNumber(), JoltHelpers::ToUESize(val.mContactPointOn2), JoltHelpers::ToUESize(val.mContactPointOn1) });
	}
	return shapeCastResult;
}

void UJoltSubsystem::RayCastNarrowPhase(const FVector& start, const FVector& end, NarrowPhaseQueryCallback& hitCallback, const JPH::BodyFilter& bodyFilter) const
{

	JPH::RayCastSettings	 settings;
	FVector					 dir = end - start;
	JPH::RRayCast			 ray{ JoltHelpers::ToJoltPos(start), JoltHelpers::ToJoltVec3(dir) };
	FirstRayCastHitCollector collector(*MainPhysicsSystem, ray);
	MainPhysicsSystem->GetNarrowPhaseQuery().CastRay(ray, settings, collector, {}, {}, bodyFilter);

	const UPhysicalMaterial* UEMat = nullptr;
	if (collector.mHasHit)
	{
		const JPH::PhysicsMaterial* foundMat = BodyInterface->GetMaterial(collector.mBodyID, collector.mSubShapeID2);
		UEMat = GetUEPhysicsMaterial(static_cast<const JoltPhysicsMaterial*>(foundMat));
	}

	hitCallback(
		JoltHelpers::ToUEPos(collector.mContactPosition),
		JoltHelpers::ToUESize(collector.mContactNormal),
		collector.mHasHit,
		collector.mBodyID.GetIndexAndSequenceNumber(),
		UEMat);
}

FTransform UJoltSubsystem::GetBodyCOM(int32 inBodyID)
{
	return JoltHelpers::ToUETransform(GetBodyInterface()->GetCenterOfMassTransform(JPH::BodyID(inBodyID)));
}

void UJoltSubsystem::SaveState(TArray<uint8>& serverPhysicsState, JPH::StateRecorderFilter* saveFilterImpl) const
{
	JPH::StateRecorderImpl* stateRecorder = new JPH::StateRecorderImpl;
	MainPhysicsSystem->SaveState(*stateRecorder, JPH::EStateRecorderState::All, saveFilterImpl);
	std::string physicsState = stateRecorder->GetData();
	delete stateRecorder;
	serverPhysicsState.Append(reinterpret_cast<const uint8*>(physicsState.data()), physicsState.size());
}

void UJoltSubsystem::RestoreState(const TArray<uint8>& serverPhysicsState) const
{
	JPH::StateRecorderImpl state;
	state.WriteBytes(serverPhysicsState.GetData(), serverPhysicsState.Num());
	MainPhysicsSystem->RestoreState(state);
}

void UJoltSubsystem::LoadLandscapeFromDataAsset()
{

	FString PackageName;
	FString AssetName;
	JoltHelpers::GenerateAssetNames(GetWorld(), PackageName, AssetName);

	JoltDataAsset = LoadObject<UJoltDataAsset>(nullptr, *PackageName);
	if (!JoltDataAsset)
	{
		UE_LOG(JoltSubSystemLogs, Log, TEXT("Could not find jolt asset"));
		return;
	}

	for (const FJoltShapeData& shape : *JoltDataAsset->GetAllBodyData())
	{
		ShapeDataReader*		shapeDataReader = new ShapeDataReader(shape.BinaryData);
		JPH::Shape::ShapeResult result = JPH::Shape::sRestoreFromBinaryState(*shapeDataReader);
		if (!result.IsValid())
		{
			UE_LOG(JoltSubSystemLogs, Error, TEXT("Loaded landscape asset is invalid. Error: %s"), *FString(result.GetError().c_str()));
			continue;
		}

		const JPH::ObjectLayer resolvedLayer = ResolveObjectLayer(shape.LayerName);
		if (resolvedLayer == JPH::cObjectLayerInvalid)
		{
			continue;
		}

		JPH::Ref<JPH::Shape> loadedShape = result.Get();
		RestoreShapeMaterials(shape, loadedShape);

		JPH::BodyCreationSettings bodyCreationSettings(
			loadedShape,
			JoltHelpers::ToJoltPos(shape.WorldTransform.GetLocation()),
			JoltHelpers::ToJoltRot(shape.WorldTransform.GetRotation()),
			shape.MotionType,
			resolvedLayer);

		uint32 bodyID;
		if (shape.MotionType == JPH::EMotionType::Static)
		{
			StaticBodyIDX++;
			bodyID = StaticBodyIDX;
		}
		else
		{
			DynamicBodyIDX++;
			bodyID = DynamicBodyIDX;
		}

		AddBodyToSimulation(new JPH::BodyID(bodyID), bodyCreationSettings, shape.Friction, shape.Restitution);
		delete shapeDataReader;
	};
}

ALandscape* UJoltSubsystem::FindSingleLandscape(const UWorld* world)
{
	if (!world)
	{
		UE_LOG(JoltSubSystemLogs, Error, TEXT("World is null!"));
		return nullptr;
	}

	for (TActorIterator<ALandscape> LandscapeIt(world); LandscapeIt; ++LandscapeIt)
	{
		if (ALandscape* Landscape = *LandscapeIt)
		{
			UE_LOG(JoltSubSystemLogs, Log, TEXT("Found Landscape: %s"), *Landscape->GetName());
			return Landscape;
		}
	}

	UE_LOG(JoltSubSystemLogs, Log, TEXT("No Landscape found in the current world."));
	return nullptr;
}

#if WITH_EDITOR

bool UJoltSubsystem::CookBodies() const
{

	UJoltDataAsset* joltDataasset = NewObject<UJoltDataAsset>();
	joltDataasset->LoadBodies(SavedBodies);

	FString PackageName;
	FString AssetName;
	JoltHelpers::GenerateAssetNames(GetWorld(), PackageName, AssetName);

	UPackage* Package = CreatePackage(*PackageName);
	if (!Package)
	{
		UE_LOG(JoltSubSystemLogs, Error, TEXT("Failed to create package: %s"), *PackageName);
		return false;
	}

	Package->SetPackageFlags(PKG_CompiledIn | RF_Standalone | RF_Public);
	UObject* SavedAsset = StaticDuplicateObject(joltDataasset, Package, FName(*AssetName));
	if (!SavedAsset)
	{
		UE_LOG(JoltSubSystemLogs, Error, TEXT("Failed to duplicate object into package."));
		return false;
	}

	SavedAsset->SetFlags(RF_Public | RF_Standalone);

	FSavePackageArgs SaveArgs;
	SaveArgs.TopLevelFlags = RF_Public | RF_Standalone;
	SaveArgs.SaveFlags = SAVE_NoError;

	FString FilePath = FPackageName::LongPackageNameToFilename(PackageName, FPackageName::GetAssetPackageExtension());
	FilePath = FPaths::ConvertRelativePathToFull(FilePath);

	return UPackage::Save(Package, SavedAsset, *FilePath, SaveArgs).IsSuccessful();
}

bool UJoltSubsystem::BuildLandscapeMaterialIndices(ULandscapeComponent* landscapeComponent, const uint32 componentSize, TArray<uint8>& outMaterialIndices, JPH::PhysicsMaterialList& outMaterialList)
{
	ULandscapeHeightfieldCollisionComponent* collisionComponent = landscapeComponent->GetCollisionComponent();
	if (collisionComponent == nullptr)
	{
		UE_LOG(JoltSubSystemLogs, Warning, TEXT("No collision component on landscape component '%s' — physics materials skipped"), *landscapeComponent->GetName());
		return false;
	}

	/*Jolt material limit is 256
	 * https://github.com/jrouwe/JoltPhysics/blob/master/Jolt/Physics/Collision/Shape/HeightFieldShape.cpp#L495
	 */

	// usually 1
	const float collisionScale = landscapeComponent->ComponentSizeQuads > 0
		? static_cast<float>(collisionComponent->CollisionSizeQuads) / static_cast<float>(landscapeComponent->ComponentSizeQuads)
		: 1.0f;

	TMap<EPhysicalSurface, uint8> materialIndexBySurface;

	const uint32 quadCount = componentSize - 1;
	outMaterialIndices.SetNumUninitialized(quadCount * quadCount);

	// jolt material of quad (x, y) = heights[(y * ComponentSize) + x] same as heightfield
	for (uint32 y = 0; y < quadCount; ++y)
	{
		for (uint32 x = 0; x < quadCount; ++x)
		{
			// Quad-center sample so the query lands inside the matching collision cell
			const UPhysicalMaterial* physMaterial = collisionComponent->GetPhysicalMaterial(
				(x + 0.5f) * collisionScale,
				(y + 0.5f) * collisionScale,
				EHeightfieldSource::Complex);

			if (physMaterial == nullptr)
			{
				// fallback
				physMaterial = GEngine->DefaultPhysMaterial.Get();
			}

			uint8 materialIndex = 0;
			if (physMaterial != nullptr)
			{
				const EPhysicalSurface surfaceType = physMaterial->SurfaceType.GetValue();
				if (const uint8* existingIndex = materialIndexBySurface.Find(surfaceType))
				{
					materialIndex = *existingIndex;
				}
				else
				{
					materialIndex = static_cast<uint8>(outMaterialList.size());
					outMaterialList.push_back(GetJoltPhysicsMaterial(physMaterial));
					materialIndexBySurface.Add(surfaceType, materialIndex);
				}
			}
			outMaterialIndices[(y * quadCount) + x] = materialIndex;
		}
	}

	if (outMaterialList.empty())
	{
		UE_LOG(JoltSubSystemLogs, Warning, TEXT("no physics materials on landscape component '%s'"), *landscapeComponent->GetName());
		return false;
	}

	return true;
}

void UJoltSubsystem::GetAllLandscapeHeights(const ALandscape* landscapeActor)
{
	FString PackageName;
	FString AssetName;
	JoltHelpers::GenerateAssetNames(GetWorld(), PackageName, AssetName);

	if (landscapeActor == nullptr || landscapeActor->LandscapeComponents.Num() == 0)
	{
		UE_LOG(LogTemp, Log, TEXT("No landscape components found"));
		return;
	}

	for (ULandscapeComponent* landscapeComponent : landscapeActor->LandscapeComponents)
	{
		if (!landscapeComponent)
			continue;

		FLandscapeComponentDataInterface DataInterface(landscapeComponent);

		const uint32 ComponentSize = landscapeComponent->ComponentSizeQuads + 1;

		FVector scale = landscapeComponent->GetComponentTransform().GetScale3D();
		uint32	arrSize = ComponentSize * ComponentSize;
		float*	heights = new float[arrSize];
		for (uint32 y = 0; y < ComponentSize; ++y)
		{
			for (uint32 x = 0; x < ComponentSize; ++x)
			{
				// (https://dev.epicgames.com/documentation/en-us/unreal-engine/landscape-technical-guide-in-unreal-engine?application_version=5.4)
				heights[(y * ComponentSize) + x] = JoltHelpers::ToJoltSize(
					LandscapeDataAccess::GetLocalHeight(DataInterface.GetHeight(x, y)) * scale.Z);
			}
		}

		JPH::Vec3 joltScale = JoltHelpers::ToJoltVec3(scale);
		joltScale.SetY(1); // We've already scaled Z(Up is Y in Jolt). Don't need to scale again

		TArray<uint8>			 materialIndices;
		JPH::PhysicsMaterialList materialList;

		const bool bHasMaterials = BuildLandscapeMaterialIndices(landscapeComponent, ComponentSize, materialIndices, materialList);

		JPH::Ref<JPH::HeightFieldShapeSettings> heightFieldShapeSettigns = new JPH::HeightFieldShapeSettings(
			heights,
			JPH::Vec3(0, 0, 0),
			joltScale,
			ComponentSize,
			bHasMaterials ? materialIndices.GetData() : nullptr,
			bHasMaterials ? materialList : JPH::PhysicsMaterialList());
		heightFieldShapeSettigns->AddRef();
		HeightFieldShapes.Add(heightFieldShapeSettigns);

		FTransform finalTransform = landscapeComponent->GetRelativeTransform() * landscapeActor->GetActorTransform();

		const JPH::ObjectLayer landscapeLayer = ResolveObjectLayer(JoltSettings->DefaultStaticLayer);
		if (landscapeLayer == JPH::cObjectLayerInvalid)
		{
			delete[] heights;
			heights = nullptr;
			continue;
		}
		JPH::Body& floor = *BodyInterface->CreateBodyWithoutID(JPH::BodyCreationSettings(heightFieldShapeSettigns, JoltHelpers::ToJoltPos(finalTransform.GetLocation()), JoltHelpers::ToJoltRot(finalTransform.GetRotation()), JPH::EMotionType::Static, landscapeLayer));

		SavedBodies.Add(&floor);
		delete[] heights;
		heights = nullptr;
	}
}

void UJoltSubsystem::HandleLandscapeMeshes(const ALandscape* LandscapeActor)
{
	if (LandscapeActor == nullptr)
	{
		UE_LOG(JoltSubSystemLogs, Log, TEXT("HandleLandscapeMeshes() LandscapeActor empty"));
		return;
	}

	ULandscapeSplinesComponent* landscapeSplineComponent = LandscapeActor->GetSplinesComponent();
	if (landscapeSplineComponent == nullptr)
	{
		UE_LOG(JoltSubSystemLogs, Log, TEXT("HandleLandscapeMeshes() GetSplinesComponent() returned null"));
		return;
	}

	TArray<TObjectPtr<ULandscapeSplineSegment>> splineSegments = landscapeSplineComponent->GetSegments();
	if (splineSegments.IsEmpty())
	{
		UE_LOG(JoltSubSystemLogs, Warning, TEXT("HandleLandscapeMeshes() no landscapesplinesegments found"));
		return;
	}

	// ULandscapeSplineSegment::GetLocalMeshComponents() is not exported with LANDSCAPE_API,
	// So, this hacky way to using reflection works for now
	static const FName LocalMeshComponentsName(TEXT("LocalMeshComponents"));
	FArrayProperty*	   localMeshesProp = FindFProperty<FArrayProperty>(
		ULandscapeSplineSegment::StaticClass(), LocalMeshComponentsName);
	if (localMeshesProp == nullptr)
	{
		UE_LOG(JoltSubSystemLogs, Warning,
			TEXT("HandleLandscapeMeshes() LocalMeshComponents UPROPERTY not found on ULandscapeSplineSegment"));
		return;
	}

	for (const TObjectPtr<ULandscapeSplineSegment>& splineSegment : splineSegments)
	{
		if (splineSegment->SplineMeshes.IsEmpty())
		{
			UE_LOG(JoltSubSystemLogs, Warning, TEXT("HandleLandscapeMeshes() no spline meshes found for segment"));
			continue;
		}

		FScriptArrayHelper arrayHelper(localMeshesProp, localMeshesProp->ContainerPtrToValuePtr<void>(splineSegment));
		for (int32 i = 0; i < arrayHelper.Num(); ++i)
		{
			UObject*					element = *reinterpret_cast<UObject**>(arrayHelper.GetRawPtr(i));
			const USplineMeshComponent* splineMesh = Cast<USplineMeshComponent>(element);
			if (splineMesh == nullptr || splineMesh->GetBodySetup() == nullptr)
			{
				continue;
			}
			ExtractSplineMeshGeometry(splineMesh->GetBodySetup(), splineMesh->GetComponentTransform());
		}
	}
}

void UJoltSubsystem::ExtractSplineMeshGeometry(const UBodySetup* splineMeshBodySetup, const FTransform& splineMeshTransform)
{
	ensure(splineMeshBodySetup != nullptr);
	if (splineMeshBodySetup == nullptr)
	{
		return;
	}

	const JPH::ObjectLayer splineMeshLayer = ResolveObjectLayer(JoltSettings->DefaultStaticLayer);
	if (splineMeshLayer == JPH::cObjectLayerInvalid)
	{
		return;
	}
	ExtractPhysicsGeometry(splineMeshTransform, splineMeshBodySetup, [this, splineMeshLayer](const JPH::Shape* Shape, const FTransform& RelTransform) mutable {
		JPH::BodyCreationSettings shapeSettings(
			Shape,
			JoltHelpers::ToJoltPos(RelTransform.GetLocation()),
			JoltHelpers::ToJoltRot(RelTransform.GetRotation()),
			JPH::EMotionType::Static,
			splineMeshLayer);

		StaticBodyIDX++;
		JPH::Body* createdBody = BodyInterface->CreateBodyWithoutID(shapeSettings);
		check(createdBody != nullptr);
		// TODO read this from the physics material
		createdBody->SetRestitution(0.7f);
		createdBody->SetFriction(0.5f);
		SavedBodies.Add(createdBody);
	});
}

#endif

#ifdef JPH_DEBUG_RENDERER
void UJoltSubsystem::DrawDebugLines() const
{
	if (!JoltSettings->bEnableDebugRenderer)
	{
		return;
	}
	if (MainPhysicsSystem == nullptr || DrawSettings == nullptr || JoltDebugRendererImpl == nullptr)
	{
		UE_LOG(JoltSubSystemLogs, Warning, TEXT("Debug renderer disabled"));
		return;
	}

	JoltDebugRendererImpl->DrawBodiesFiltered(MainPhysicsSystem, *DrawSettings, JoltSettings);
}
#endif

void UJoltSubsystem::JoltSetLinearAndAngularVelocity(const int64& bodyID, const FVector& velocity, const FVector& angularVelocity) const
{
	GetBodyInterface()->SetLinearAndAngularVelocity(JPH::BodyID(bodyID), JoltHelpers::ToJoltVec3(velocity), JoltHelpers::ToJoltVec3(angularVelocity));
}

void UJoltSubsystem::JoltSetLinearAndAngularVelocity(const JPH::BodyID& bodyID, const FVector& velocity, const FVector& angularVelocity) const
{
	GetBodyInterface()->SetLinearAndAngularVelocity(bodyID, JoltHelpers::ToJoltVec3(velocity), JoltHelpers::ToJoltVec3(angularVelocity));
}

void UJoltSubsystem::JoltGetPhysicsState(const int64& bodyID, FTransform& transform, FTransform& transformCOM, FVector& velocity, FVector& angularVelocity) const
{
	transform = JoltHelpers::ToUETransform(GetBodyInterface()->GetWorldTransform(JPH::BodyID(bodyID)));
	transformCOM = JoltHelpers::ToUETransform(GetBodyInterface()->GetCenterOfMassTransform(JPH::BodyID(bodyID)));
	velocity = JoltHelpers::ToUESize(GetBodyInterface()->GetLinearVelocity(JPH::BodyID(bodyID)));
	angularVelocity = JoltHelpers::ToUESize(GetBodyInterface()->GetAngularVelocity(JPH::BodyID(bodyID)));
}

void UJoltSubsystem::JoltGetPhysicsState(const JPH::BodyID& bodyID, FTransform& transform, FTransform& transformCOM, FVector& velocity, FVector& angularVelocity) const
{
	transform = JoltHelpers::ToUETransform(GetBodyInterface()->GetWorldTransform(bodyID));
	transformCOM = JoltHelpers::ToUETransform(GetBodyInterface()->GetCenterOfMassTransform(bodyID));
	velocity = JoltHelpers::ToUESize(GetBodyInterface()->GetLinearVelocity(bodyID));
	angularVelocity = JoltHelpers::ToUESize(GetBodyInterface()->GetAngularVelocity(bodyID));
}

void UJoltSubsystem::JoltGetPhysicsTransform(const int64& bodyID, FTransform& transform) const
{
	transform = JoltHelpers::ToUETransform(GetBodyInterface()->GetWorldTransform(JPH::BodyID(bodyID)));
}

void UJoltSubsystem::JoltSetAllowedDOFs(const int64& bodyID, int32 allowedDOFs) const
{
	JoltSetAllowedDOFs(JPH::BodyID(bodyID), allowedDOFs);
}

void UJoltSubsystem::JoltSetObjectLayer(const int64& bodyID, FName layer) const
{
	JoltSetObjectLayer(JPH::BodyID(bodyID), layer);
}

void UJoltSubsystem::JoltSetMass(const int64& bodyID, const float& mass) const
{
	JoltSetMass(JPH::BodyID(bodyID), mass);
}

void UJoltSubsystem::JoltSetGravityFactor(const int64& bodyID, const float& gravityFactor) const
{
	JoltSetGravityFactor(JPH::BodyID(bodyID), gravityFactor);
}

void UJoltSubsystem::JoltSetApplyGyroscopicForce(const int64& bodyID, bool bApplyGyroscopicForce) const
{
	JoltSetApplyGyroscopicForce(JPH::BodyID(bodyID), bApplyGyroscopicForce);
}

void UJoltSubsystem::JoltSetMaxLinearVelocity(const int64& bodyID, float maxLinearVelocity) const
{
	JoltSetMaxLinearVelocity(JPH::BodyID(bodyID), maxLinearVelocity);
}

float UJoltSubsystem::JoltGetMaxLinearVelocity(const int64& bodyID) const
{
	return JoltGetMaxLinearVelocity(JPH::BodyID(bodyID));
}

void UJoltSubsystem::JoltSetMaxAngularVelocity(const int64& bodyID, float maxAngularVelocity) const
{
	JoltSetMaxAngularVelocity(JPH::BodyID(bodyID), maxAngularVelocity);
}

void UJoltSubsystem::JoltSetFriction(const int64& bodyID, float friction) const
{
	JoltSetFriction(JPH::BodyID(bodyID), friction);
}

void UJoltSubsystem::JoltSetRestitution(const int64& bodyID, float restitution) const
{
	JoltSetRestitution(JPH::BodyID(bodyID), restitution);
}

void UJoltSubsystem::JoltSetLinearDamping(const int64& bodyID, float linearDamping) const
{
	JoltSetLinearDamping(JPH::BodyID(bodyID), linearDamping);
}

void UJoltSubsystem::JoltSetAngularDamping(const int64& bodyID, float angularDamping) const
{
	JoltSetAngularDamping(JPH::BodyID(bodyID), angularDamping);
}

void UJoltSubsystem::JoltSetAllowSleeping(const int64& bodyID, bool bAllowSleeping) const
{
	JoltSetAllowSleeping(JPH::BodyID(bodyID), bAllowSleeping);
}

void UJoltSubsystem::JoltSetNumVelocityStepsOverride(const int64& bodyID, int numVelocityStepsOverride) const
{
	JoltSetNumVelocityStepsOverride(JPH::BodyID(bodyID), numVelocityStepsOverride);
}

void UJoltSubsystem::JoltSetNumPositionStepsOverride(const int64& bodyID, int numPositionStepsOverride) const
{
	JoltSetNumPositionStepsOverride(JPH::BodyID(bodyID), numPositionStepsOverride);
}

void UJoltSubsystem::JoltSetEnhancedInternalEdgeRemoval(const int64& bodyID, bool bEnhancedInternalEdgeRemoval) const
{
	JoltSetEnhancedInternalEdgeRemoval(JPH::BodyID(bodyID), bEnhancedInternalEdgeRemoval);
}

void UJoltSubsystem::JoltGetPhysicsTransform(const JPH::BodyID& bodyID, FTransform& transform) const
{
	transform = JoltHelpers::ToUETransform(GetBodyInterface()->GetWorldTransform(bodyID));
}

void UJoltSubsystem::JoltSetAllowedDOFs(const JPH::BodyID& bodyID, int32 allowedDOFs) const
{
	if (JPH::Body* body = GetBody(bodyID.GetIndexAndSequenceNumber()))
	{
		if (JPH::MotionProperties* MotionProperties = body->GetMotionProperties())
		{
			JPH::EAllowedDOFs JoltDOFs = JPH::EAllowedDOFs::None;

			// X Axis maps 1:1
			if (allowedDOFs & (int32)EJoltAllowedDOFs::TranslationX) JoltDOFs |= JPH::EAllowedDOFs::TranslationX;
			if (allowedDOFs & (int32)EJoltAllowedDOFs::RotationX) JoltDOFs |= JPH::EAllowedDOFs::RotationX;

			// Unreal's Y maps to Jolt's Z
			if (allowedDOFs & (int32)EJoltAllowedDOFs::TranslationY) JoltDOFs |= JPH::EAllowedDOFs::TranslationZ;
			if (allowedDOFs & (int32)EJoltAllowedDOFs::RotationY) JoltDOFs |= JPH::EAllowedDOFs::RotationZ;

			// Unreal's Z maps to Jolt's Y
			if (allowedDOFs & (int32)EJoltAllowedDOFs::TranslationZ) JoltDOFs |= JPH::EAllowedDOFs::TranslationY;
			if (allowedDOFs & (int32)EJoltAllowedDOFs::RotationZ) JoltDOFs |= JPH::EAllowedDOFs::RotationY;
			
			JPH::MassProperties MassProperties = body->GetShape()->GetMassProperties();
			MassProperties.ScaleToMass(1.0f / MotionProperties->GetInverseMass());
			MotionProperties->SetMassProperties(JoltDOFs, MassProperties);
		}
	}
}

void UJoltSubsystem::JoltSetObjectLayer(const JPH::BodyID& bodyID, FName layer) const
{
	JPH::ObjectLayer objectLayer = ResolveObjectLayer(layer);
	if (objectLayer == JPH::cObjectLayerInvalid) return;
	
	GetBodyInterface()->SetObjectLayer(bodyID, objectLayer);
}

void UJoltSubsystem::JoltSetMass(const JPH::BodyID& bodyID, const float& mass) const
{
	if (JPH::Body* body = GetBody(bodyID.GetIndexAndSequenceNumber()))
	{
		if (body->GetMotionProperties())
		body->GetMotionProperties()->ScaleToMass(mass);
	}
}

void UJoltSubsystem::JoltSetGravityFactor(const JPH::BodyID& bodyID, const float& gravityFactor) const
{
	GetBodyInterface()->SetGravityFactor(bodyID, gravityFactor);
}

void UJoltSubsystem::JoltSetApplyGyroscopicForce(const JPH::BodyID& bodyID, bool bApplyGyroscopicForce) const
{
	if (JPH::Body* body = GetBody(bodyID.GetIndexAndSequenceNumber()))
	{
		body->SetApplyGyroscopicForce(bApplyGyroscopicForce);
	}
}

void UJoltSubsystem::JoltSetMaxLinearVelocity(const JPH::BodyID& bodyID, float maxLinearVelocity) const
{
	GetBodyInterface()->SetMaxLinearVelocity(bodyID, JoltHelpers::ToJoltSize(maxLinearVelocity));
}

float UJoltSubsystem::JoltGetMaxLinearVelocity(const JPH::BodyID& bodyID) const
{
	return JoltHelpers::ToUESize(GetBodyInterface()->GetMaxLinearVelocity(bodyID));
}

void UJoltSubsystem::JoltSetMaxAngularVelocity(const JPH::BodyID& bodyID, float maxAngularVelocity) const
{
	GetBodyInterface()->SetMaxAngularVelocity(bodyID, maxAngularVelocity);
}

void UJoltSubsystem::JoltSetFriction(const JPH::BodyID& bodyID, float friction) const
{
	GetBodyInterface()->SetFriction(bodyID, friction);
}

void UJoltSubsystem::JoltSetRestitution(const JPH::BodyID& bodyID, float restitution) const
{
	GetBodyInterface()->SetRestitution(bodyID, restitution);
}

void UJoltSubsystem::JoltSetLinearDamping(const JPH::BodyID& bodyID, float linearDamping) const
{
	if (JPH::Body* body = GetBody(bodyID.GetIndexAndSequenceNumber()))
	{
		if (body->GetMotionProperties())
		body->GetMotionProperties()->SetLinearDamping(linearDamping);
	}
}

void UJoltSubsystem::JoltSetAngularDamping(const JPH::BodyID& bodyID, float angularDamping) const
{
	if (JPH::Body* body = GetBody(bodyID.GetIndexAndSequenceNumber()))
	{
		if (body->GetMotionProperties())
		body->GetMotionProperties()->SetAngularDamping(angularDamping);
	}
}

void UJoltSubsystem::JoltSetAllowSleeping(const JPH::BodyID& bodyID, bool bAllowSleeping) const
{
	if (JPH::Body* body = GetBody(bodyID.GetIndexAndSequenceNumber()))
		body->SetAllowSleeping(bAllowSleeping);
}

void UJoltSubsystem::JoltSetNumVelocityStepsOverride(const JPH::BodyID& bodyID, int numVelocityStepsOverride) const
{
	if (JPH::Body* body = GetBody(bodyID.GetIndexAndSequenceNumber()))
	{
		if (JPH::MotionProperties* MotionProperties = body->GetMotionProperties())
		{
			MotionProperties->SetNumVelocityStepsOverride(numVelocityStepsOverride);
		}
	}
}

void UJoltSubsystem::JoltSetNumPositionStepsOverride(const JPH::BodyID& bodyID, int numPositionStepsOverride) const
{
	if (JPH::Body* body = GetBody(bodyID.GetIndexAndSequenceNumber()))
	{
		if (JPH::MotionProperties* MotionProperties = body->GetMotionProperties())
		{
			MotionProperties->SetNumPositionStepsOverride(numPositionStepsOverride);
		}
	}
}

void UJoltSubsystem::JoltSetEnhancedInternalEdgeRemoval(const JPH::BodyID& bodyID, bool bEnhancedInternalEdgeRemoval) const
{
	if (JPH::Body* body = GetBody(bodyID.GetIndexAndSequenceNumber()))
	{
		body->SetEnhancedInternalEdgeRemoval(bEnhancedInternalEdgeRemoval);
	}
}

void UJoltSubsystem::JoltAddCentralImpulse(const JPH::BodyID& bodyID, const FVector& impulse) const
{
	GetBodyInterface()->AddImpulse(bodyID, JoltHelpers::ToJoltVec3(impulse));
}

void UJoltSubsystem::JoltAddCentralImpulse(const int64& bodyID, const FVector& impulse) const
{
	GetBodyInterface()->AddImpulse(JPH::BodyID(bodyID), JoltHelpers::ToJoltVec3(impulse));
}

void UJoltSubsystem::JoltAddTorque(const JPH::BodyID& bodyID, const FVector& torque) const
{
	GetBodyInterface()->AddTorque(bodyID, JoltHelpers::ToJoltVec3(torque));
}

void UJoltSubsystem::JoltAddTorque(const int64& bodyID, const FVector& torque) const
{
	GetBodyInterface()->AddTorque(JPH::BodyID(bodyID), JoltHelpers::ToJoltVec3(torque));
}

void UJoltSubsystem::JoltAddForce(const JPH::BodyID& bodyID, const FVector& torque) const
{
	GetBodyInterface()->AddForce(bodyID, JoltHelpers::ToJoltVec3(torque));
}

void UJoltSubsystem::JoltAddForce(const int64& bodyID, const FVector& torque) const
{
	GetBodyInterface()->AddForce(JPH::BodyID(bodyID), JoltHelpers::ToJoltVec3(torque));
}

void UJoltSubsystem::JoltAddImpulseAtLocation(const int64& BodyID, const FVector& impulse, const FVector& locationWS) const
{
	GetBodyInterface()->AddImpulse(JPH::BodyID(BodyID), JoltHelpers::ToJoltVec3(impulse), JoltHelpers::ToJoltPos(locationWS));
}

void UJoltSubsystem::JoltAddImpulseAtLocation(const JPH::BodyID& bodyID, const FVector& impulse, const FVector& locationWS) const
{
	GetBodyInterface()->AddImpulse(bodyID, JoltHelpers::ToJoltVec3(impulse), JoltHelpers::ToJoltPos(locationWS));
}

void UJoltSubsystem::JoltAddForceAtLocation(const JPH::BodyID& bodyID, const FVector& force, const FVector& locationWS) const
{
	GetBodyInterface()->AddForce(bodyID, JoltHelpers::ToJoltVec3(force), JoltHelpers::ToJoltPos(locationWS));
}

void UJoltSubsystem::JoltAddForceAtLocation(const int64& bodyID, const FVector& force, const FVector& locationWS) const
{
	GetBodyInterface()->AddForce(JPH::BodyID(bodyID), JoltHelpers::ToJoltVec3(force), JoltHelpers::ToJoltPos(locationWS));
}

FVector UJoltSubsystem::JoltGetVelocityAt(const int64& bodyID, const FVector& locationWS) const
{
	return JoltHelpers::ToUESize(GetBodyInterface()->GetPointVelocity(JPH::BodyID(bodyID), JoltHelpers::ToJoltPos(locationWS)));
}

FVector UJoltSubsystem::JoltGetVelocityAt(const JPH::BodyID& bodyID, const FVector& locationWS) const
{
	return JoltHelpers::ToUESize(GetBodyInterface()->GetPointVelocity(bodyID, JoltHelpers::ToJoltPos(locationWS)));
}

void UJoltSubsystem::JoltSetPhysicsLocationAndRotation(const int32& bodyID, const FVector& locationWS, const FQuat& rotationWS) const
{
	GetBodyInterface()->SetPositionAndRotation(JPH::BodyID(bodyID), JoltHelpers::ToJoltPos(locationWS), JoltHelpers::ToJoltRot(rotationWS), JPH::EActivation::Activate);
}

void UJoltSubsystem::JoltSetPhysicsLocationAndRotation(const JPH::BodyID& bodyID, const FVector& locationWS, const FQuat& rotationWS) const
{
	GetBodyInterface()->SetPositionAndRotation(bodyID, JoltHelpers::ToJoltPos(locationWS), JoltHelpers::ToJoltRot(rotationWS), JPH::EActivation::Activate);
}

void UJoltSubsystem::JoltSetLinearVelocity(const int& bodyID, const FVector& velocity) const
{
	GetBodyInterface()->SetLinearVelocity(JPH::BodyID(bodyID), JoltHelpers::ToJoltVec3(velocity));
}

void UJoltSubsystem::JoltSetLinearVelocity(const JPH::BodyID& bodyID, const FVector& velocity) const
{
	GetBodyInterface()->SetLinearVelocity(bodyID, JoltHelpers::ToJoltVec3(velocity));
}

void UJoltSubsystem::JoltSetPhysicsLocation(const int& bodyID, const FVector& locationWS) const
{
	GetBodyInterface()->SetPosition(JPH::BodyID(bodyID), JoltHelpers::ToJoltPos(locationWS), JPH::EActivation::Activate);
}

void UJoltSubsystem::JoltSetPhysicsLocation(const JPH::BodyID& bodyID, const FVector& locationWS) const
{
	GetBodyInterface()->SetPosition(bodyID, JoltHelpers::ToJoltPos(locationWS), JPH::EActivation::Activate);
}

void UJoltSubsystem::JoltSetPhysicsRotation(const JPH::BodyID& bodyID, const FQuat& rotationWS) const
{
	GetBodyInterface()->SetRotation(bodyID, JoltHelpers::ToJoltRot(rotationWS), JPH::EActivation::Activate);
}

void UJoltSubsystem::JoltSetPhysicsRotation(const int64& bodyID, const FQuat& rotationWS) const
{
	GetBodyInterface()->SetRotation(JPH::BodyID(bodyID), JoltHelpers::ToJoltRot(rotationWS), JPH::EActivation::Activate);
}

int32 UJoltSubsystem::GetObjectLayerByName(FName LayerName) const
{
	const int32* idx = LayerTable.NameToObjectLayer.Find(LayerName);
	return idx != nullptr ? *idx : INDEX_NONE;
}

JPH::ObjectLayer UJoltSubsystem::ResolveObjectLayer(FName LayerName, JPH::ObjectLayer Fallback) const
{
	if (LayerName.IsNone())
	{
		return Fallback;
	}
	const int32* idx = LayerTable.NameToObjectLayer.Find(LayerName);
	if (idx == nullptr)
	{
		if (Fallback == JPH::cObjectLayerInvalid)
		{
			UE_LOG(JoltSubSystemLogs, Error, TEXT("ResolveObjectLayer: layer '%s' not found in UJoltSettings — body creation will be refused"),
				*LayerName.ToString());
		}
		else
		{
			UE_LOG(JoltSubSystemLogs, Warning, TEXT("ResolveObjectLayer: layer '%s' not found in UJoltSettings, falling back to id %u"),
				*LayerName.ToString(), static_cast<uint32>(Fallback));
		}
		return Fallback;
	}
	return static_cast<JPH::ObjectLayer>(*idx);
}
