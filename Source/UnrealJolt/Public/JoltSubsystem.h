#pragma once

#include "Containers/Array.h"
#include "Containers/Map.h"
#include "GenericPlatform/GenericPlatformMisc.h"
#include "JoltContactListener.h"
#include "JoltDataAsset.h"
#include "JoltWorker.h"
#include "JoltSettings.h"
#include "Landscape.h"
#include "Subsystems/WorldSubsystem.h"
#include "JoltLayerTable.h"
#include "JoltFilters.h"
#include "JoltPhysicsCallbackInterface.h"
#include "Delegates/DelegateCombinations.h"
#include "UObject/ObjectMacros.h"

#ifdef JPH_DEBUG_RENDERER
	#include "JoltDebugRenderer.h"
#endif

#include "JoltSubsystem.generated.h"

/* Just used to make them friend classess of UJoltSubSystem class because it needs some private methods
 * that should not be exposed to avoid accidental usage in project code  */
class UJoltSkeletalMeshComponent;
class JoltAxisConstraint;
class JoltPhysicsMaterial;
class ULandscapeComponent;

UDELEGATE(BlueprintCallable)
DECLARE_DYNAMIC_DELEGATE_FourParams(FNarrowPhaseQueryDelegate, const FVector&, hitLocation, const FVector&, hitNormal, bool, bHasHit, const int32, hitBodyID);

struct FFrameHistory
{
	FVector	 PrevLocation = FVector::ZeroVector;
	FRotator PrevRotation = FRotator::ZeroRotator;

	FVector	 CurrentLocation = FVector::ZeroVector;
	FRotator CurrentRotation = FRotator::ZeroRotator;

	// Just getters so friend class subsystem's wont be tied to struct field names
	const FVector&	GetPreviousFrameLocation() const { return PrevLocation; }
	const FRotator& GetPreviousFrameRotation() const { return PrevRotation; }

	const FVector&	GetCurrentFrameLocation() const { return CurrentLocation; }
	const FRotator& GetCurrentFrameRotation() const { return CurrentRotation; }
};

USTRUCT(BlueprintType)
struct FCastShapeResult
{
	GENERATED_USTRUCT_BODY()

	uint32 ContactBodyID;

	FVector ContactLocationFoundShape;

	FVector ContactLocationCastedShape;
};

struct FJoltBodyActor
{
	FJoltBodyActor(const JPH::BodyID* JoltBodyID, const TWeakObjectPtr<AActor>& Actor)
		: JoltBodyID(JoltBodyID), Actor(Actor) {}

	const JPH::BodyID*	   JoltBodyID;
	TWeakObjectPtr<AActor> Actor;
	FFrameHistory		   FrameHistory;
};

typedef const std::function<void(const FVector&, const FVector&, const bool&, const uint32&, const UPhysicalMaterial*)> NarrowPhaseQueryCallback;

DECLARE_MULTICAST_DELEGATE(FOnJoltReady);

UCLASS()
class UNREALJOLT_API UJoltSubsystem : public UTickableWorldSubsystem
{
	GENERATED_BODY()

public:
	UFUNCTION(BlueprintCallable, Category = "Jolt Physics", meta = (AdvancedDisplay = "Layer"))
	int64 AddDynamicBody(AActor* body, const float& friction, const float& restitution, const float& mass, FName Layer = NAME_None);

	/*
	 * Adds a static body to the jolt physics system.
	 * The other way is to just add 'jolt-static' tag to the actor
	 */
	UFUNCTION(BlueprintCallable, Category = "Jolt Physics", meta = (AdvancedDisplay = "Layer"))
	int64 AddStaticBody(const AActor* body, const float& friction, const float& restitution, FName Layer = NAME_None);

	/*
	 * Layer lookup. Returns INDEX_NONE if the name isn't a configured object layer.
	 * Use when gameplay code needs the raw id for a custom body creation path.
	 */
	UFUNCTION(BlueprintCallable, Category = "Jolt|Layers")
	int32 GetObjectLayerByName(FName LayerName) const;

	// Returns the JPH::ObjectLayer for the given name, or Fallback when the name is unknown.
	// Default fallback is JPH::cObjectLayerInvalid so callers can detect & refuse to create a body
	// rather than silently dumping it into whichever layer happens to be at index 0.
	JPH::ObjectLayer ResolveObjectLayer(FName LayerName, JPH::ObjectLayer Fallback = JPH::cObjectLayerInvalid) const;

	JPH::ObjectLayer ResolveDynamicLayer(FName LayerName) const
	{
		return ResolveObjectLayer(LayerName.IsNone() ? JoltSettings->DefaultDynamicLayer : LayerName);
	}

	JPH::ObjectLayer ResolveStaticLayer(FName LayerName) const
	{
		return ResolveObjectLayer(LayerName.IsNone() ? JoltSettings->DefaultStaticLayer : LayerName);
	}

	// Read-only access to the runtime layer table (valid after Initialize / before Deinitialize).
	const FJoltLayerTable& GetLayerTable() const { return LayerTable; }

	/*
	 * Sweeps a ray from start to end.
	 * This will first perform a broadphase, then a narrow phase query
	 */
	UFUNCTION(BlueprintCallable, Category = "Jolt Physics")
	void RayCastNarrowPhase(const FVector& start, const FVector& end, const FNarrowPhaseQueryDelegate& hitCallback);

	/*
	 * Used to check a collision by placing the shape at a static location
	 */
	UFUNCTION(BlueprintCallable, Category = "Jolt Physics")
	TArray<int32> CollideShape(const UShapeComponent* shape, const FVector& shapeScale, const FTransform& shapeCOM, const FVector& offset);

	/*
	 * Sweep a shape to detect collision
	 */
	UFUNCTION(BlueprintCallable, Category = "Jolt Physics")
	TArray<FCastShapeResult> CastShape(const UShapeComponent* shape, const FVector& shapeScale, const FTransform& shapeCOM, const FVector& offset);

	/*
	 * Fetch the centre of mass of the body
	 */
	UFUNCTION(BlueprintCallable, Category = "Jolt Physics")
	FTransform GetBodyCOM(int32 bodyID);

	/*
	 * This fetches the current call interval beween each calls to update()
	 * This will be equal to the JoltSettings->FixedDeltaTime in most cases.
	 * This will only change if SetTimeScale() is called manually
	 * Useful for client-server configs where scaling time is desired
	 */
	UFUNCTION(BlueprintCallable, Category = "Jolt Physics")
	double GetTimeScale() const { return ConfiguredDeltaSeconds; };

	/*
	 * This is the value that is passed to the physics update() function
	 * Changing the TimeScale will not change this value
	 * The physics system is esseintailly goint to "pretent" like the deltatime has scaled
	 * Call interval can change, the physicsDeltaTime will not (at runtime only)
	 */
	UFUNCTION(BlueprintCallable, Category = "Jolt Physics")
	double GetDeltaTime() const { return JoltSettings->FixedDeltaTime; };

	/*
	 * Jolt physics tickrate. This is divided in inCollisionSteps iterations
	 * Number of physics ticks per second
	 */
	UFUNCTION(BlueprintCallable, Category = "Jolt Physics")
	int GetTickRate() { return JoltSettings->TickRate; };

#if WITH_EDITOR
	UFUNCTION(BlueprintCallable, Category = "Jolt Physics")
	void ExtractSplineMeshGeometry(const UBodySetup* splineMeshBodySetup, const FTransform& splineMeshTransform);
#endif

	UFUNCTION(BlueprintCallable, Category = "Jolt Physics", BlueprintPure = false)
	void JoltSetLinearAndAngularVelocity(const int64& bodyID, const FVector& velocity, const FVector& angularVelocity) const;

	UFUNCTION(BlueprintCallable, Category = "Jolt Physics", BlueprintPure = false)
	void JoltGetPhysicsState(const int64& bodyID, FTransform& transform, FTransform& transformCOM, FVector& velocity, FVector& angularVelocity) const;

	UFUNCTION(BlueprintCallable, Category = "Jolt Physics", BlueprintPure = false)
	void JoltAddCentralImpulse(const int64& bodyID, const FVector& impulse) const;

	UFUNCTION(BlueprintCallable, Category = "Jolt Physics", BlueprintPure = false)
	void JoltAddTorque(const int64& bodyID, const FVector& torque) const;

	UFUNCTION(BlueprintCallable, Category = "Jolt Physics", BlueprintPure = false)
	void JoltAddForce(const int64& bodyID, const FVector& force) const;

	UFUNCTION(BlueprintCallable, Category = "Jolt Physics", BlueprintPure = false)
	void JoltAddImpulseAtLocation(const int64& bodyID, const FVector& impulse, const FVector& locationWS) const;

	UFUNCTION(BlueprintCallable, Category = "Jolt Physics", BlueprintPure = false)
	void JoltAddForceAtLocation(const int64& bodyID, const FVector& force, const FVector& locationWS) const;

	UFUNCTION(BlueprintCallable, Category = "Jolt Physics", BlueprintPure = false)
	FVector JoltGetVelocityAt(const int64& bodyID, const FVector& locationWS) const;

	UFUNCTION(BlueprintCallable, Category = "Jolt Physics", BlueprintPure = false)
	void JoltSetPhysicsLocationAndRotation(const int32& bodyID, const FVector& locationWS, const FQuat& rotationWS) const;

	UFUNCTION(BlueprintCallable, Category = "Jolt Physics", BlueprintPure = false)
	void JoltSetLinearVelocity(const int& bodyID, const FVector& velocity) const;

	UFUNCTION(BlueprintCallable, Category = "Jolt Physics", BlueprintPure = false)
	void JoltSetPhysicsLocation(const int& bodyID, const FVector& locationWS) const;

	UFUNCTION(BlueprintCallable, Category = "Jolt Physics", BlueprintPure = false)
	void JoltSetPhysicsRotation(const int64& bodyID, const FQuat& rotationWS) const;

	UFUNCTION(BlueprintCallable, Category = "Jolt Physics", BlueprintPure = false)
	void JoltGetPhysicsTransform(const int64& bodyID, FTransform& transform) const;

	UFUNCTION(BlueprintCallable, Category = "Jolt Physics")
	void SetTimeScale(double deltaTime);

	/* Will zero the accumulator and stop the physics tick. Will not stop interpolating actors
	you can still call stepPhysics manually for a reply system or something*/
	UFUNCTION(BlueprintCallable, Category = "Jolt Physics")
	void SetPaused(bool bPaused);

	UFUNCTION(BlueprintCallable, Category = "Jolt Physics")
	bool IsPaused() const { return bStepPaused; }

	UFUNCTION(BlueprintCallable, Category = "Jolt Physics")
	void SetGravity(const FVector& gravity);

	UFUNCTION(BlueprintCallable, Category = "Jolt Physics")
	FVector GetGravity() const;

	UFUNCTION(BlueprintCallable, Category = "Jolt Physics")
	bool GetContactInfo(FContactInfo& contactInfo) const { return ContactListener->Consume(contactInfo); };

	UEJoltCallBackContactListener* GetContactListener() { return ContactListener; };

	void JoltSetLinearAndAngularVelocity(const JPH::BodyID& bodyID, const FVector& velocity, const FVector& angularVelocity) const;

	void JoltGetPhysicsState(const JPH::BodyID& bodyID, FTransform& transform, FTransform& transformCOM, FVector& velocity, FVector& angularVelocity) const;

	void JoltAddCentralImpulse(const JPH::BodyID& bodyID, const FVector& impulse) const;

	void JoltAddTorque(const JPH::BodyID& bodyID, const FVector& torque) const;

	void JoltAddForce(const JPH::BodyID& bodyID, const FVector& torque) const;

	void JoltAddImpulseAtLocation(const JPH::BodyID& bodyID, const FVector& impulse, const FVector& locationWS) const;

	void JoltAddForceAtLocation(const JPH::BodyID& bodyID, const FVector& force, const FVector& locationWS) const;

	FVector JoltGetVelocityAt(const JPH::BodyID& bodyID, const FVector& locationWS) const;

	void JoltSetPhysicsLocationAndRotation(const JPH::BodyID& bodyID, const FVector& locationWS, const FQuat& rotationWS) const;

	void JoltSetLinearVelocity(const JPH::BodyID& bodyID, const FVector& velocity) const;

	void JoltSetPhysicsLocation(const JPH::BodyID& bodyID, const FVector& locationWS) const;

	void JoltSetPhysicsRotation(const JPH::BodyID& bodyID, const FQuat& rotationWS) const;

	void JoltGetPhysicsTransform(const JPH::BodyID& bodyID, FTransform& transform) const;

	// This will first perform a broadphase, and then a narrow phase query
	void RayCastNarrowPhase(const FVector& start, const FVector& end, NarrowPhaseQueryCallback& hitCallback, const JPH::BodyFilter& bodyFilter = {}) const;

	void RayCastShapeNarrowPhase(const UShapeComponent* shape, const FVector& shapeScale, const FTransform& shapeCOM, const FVector& offset, NarrowPhaseQueryCallback& hitCallback);

	uint16 AddPrePhysicsCallback(const TDelegate<void(float)>& callback) const;

	uint16 AddPostPhysicsCallback(const TDelegate<void(float)>& callback) const;

	/** Fired once per frame after InterpolatePhysicsFrame completes. dt = frame delta. */
	void AddPostInterpolationCallback(const TDelegate<void(float)>& callback);

	// Blueprint physics callback listeners
	
	/** Register an actor as a listener for pre and post physics step callbacks. Must implement IJoltPhysicsCallbackInterface. */
	UFUNCTION(BlueprintCallable, Category = "Jolt Physics|Callbacks", meta = (DefaultToSelf = "Listener"))
	void RegisterPhysicsListener(AActor* Listener);

	/** Unregister an actor as a listener for pre and post physics step callbacks. Must implement IJoltPhysicsCallbackInterface. */
	UFUNCTION(BlueprintCallable, Category = "Jolt Physics|Callbacks", meta = (DefaultToSelf = "Listener"))
	void UnregisterPhysicsListener(AActor* Listener);
	
	/** Physics-frame interpolation alpha (0..1) computed in the most recent Tick. */
	double GetPhysicsAlpha() const { return PhysicsAlpha_; }

	void RestoreState(const TArray<uint8>& serverPhysicsState) const;

	// Saves the current physics states of bodies (optionally filtered)
	void SaveState(TArray<uint8>& serverPhysicsState, JPH::StateRecorderFilter* saveFilterImpl = nullptr) const;

	/* This will automatically be called in the tick function
	 * for use cases where you just need to step the physics only
	 * set bWithCallbacks to false and disable all the binded callbacks
	 */
	void StepPhysics(bool bWithCallbacks = true);

	// --- Public external-owner API ---
	// Stable contract for plugins that drive their own physics state on top
	// of UJoltSubsystem. Not BlueprintCallable.
	JPH::BodyInterface* GetBodyInterface() const { return BodyInterface; }
	JPH::PhysicsSystem* GetPhysicsSystem() const { return MainPhysicsSystem; }

	const JPH::ObjectLayerPairFilter* GetObjectLayerPairFilter() const { return ObjectVsObjectLayerFilter; }

	const JPH::BodyID* AddDynamicBodyForExternalOwner(
		const JPH::BodyID& bodyID,
		const JPH::Shape*  shape,
		const FTransform&  initialWorldTransform,
		float friction, float restitution, float mass,
		FName layerName = NAME_None);

	// Mirror of AddDynamicBodyForExternalOwner: removes and destroys the body
	// in Jolt AND drops the BodyIDBodyMap entry.
	void RemoveBodyForExternalOwner(const JPH::BodyID& bodyID);

	const UPhysicalMaterial* GetUEPhysicsMaterial(const JoltPhysicsMaterial* JoltPhysicsMat) const;

private:
	const JoltPhysicsMaterial* GetJoltPhysicsMaterial(const UPhysicalMaterial* UEPhysicsMat);

	/* we are mappign a surface to a physics material unlike in ue where you can have many materials with same surface
	 * for now, if you want to spawn and effect for same surface for example, use one fo the EPhysicsSurface
	 * Dry_Asphalt, and Wet_Asphalt for example
	 */
	const JoltPhysicsMaterial* GetOrCreateJoltMaterialForSurface(EPhysicalSurface surfaceType, float friction, float restitution);

	void RestoreShapeMaterials(const FJoltShapeData& shapeData, const JPH::Ref<JPH::Shape>& loadedShape);

	const JPH::BoxShape* GetBoxCollisionShape(const FVector& dimensions, const JoltPhysicsMaterial* material = nullptr);

	const JPH::SphereShape* GetSphereCollisionShape(const float& radius, const JoltPhysicsMaterial* material = nullptr);

	const JPH::CapsuleShape* GetCapsuleCollisionShape(const float& radius, const float& height, const JoltPhysicsMaterial* material = nullptr);

	const JPH::ConvexHullShape* GetConvexHullCollisionShape(const UBodySetup* bodySetup, int convexIndex, const FVector& scale, const JoltPhysicsMaterial* material = nullptr);

	const JPH::BodyID* AddDynamicBodyCollision(const JPH::BodyID& bodyID, const JPH::Shape* shape, const FTransform& initialWorldTransform, float friction, float restitution, float mass, FName layerName = NAME_None);

	const JPH::BodyID* AddDynamicBodyCollision(const JPH::Shape* shape, const FTransform& initialWorldTransform, float friction, float restitution, float mass, FName layerName = NAME_None);

	const JPH::BodyID* AddStaticBodyCollision(const JPH::Shape* shape, const FTransform& transform, float friction, float restitution, FName layerName = NAME_None);

	const JPH::BodyID* AddStaticBodyCollision(const JPH::BodyID& bodyID, const JPH::Shape* shape, const FTransform& initialWorldTransform, float friction, float restitution, FName layerName = NAME_None);

	const JPH::BodyID* AddKinematicBodyCollision(const JPH::BodyID& bodyID, const JPH::Shape* shape, const FTransform& initialWorldTransform, float friction, float restitution, float mass, FName layerName = NAME_None);

	const JPH::BodyID* AddKinematicBodyCollision(const JPH::Shape* shape, const FTransform& initialWorldTransform, float friction, float restitution, float mass, FName layerName = NAME_None);

	const JPH::BodyID* AddBodyToSimulation(const JPH::BodyID* bodyID, const JPH::BodyCreationSettings& shapeSettings, float friction, float restitution);

	JPH::Body* GetBody(uint32 bodyID) const { return BodyIDBodyMap[bodyID]; }

	UPROPERTY()
	UJoltDataAsset* JoltDataAsset = nullptr;

	UPROPERTY()
	const UJoltSettings* JoltSettings = nullptr;

	FJoltWorkerOptions* WorkerOptions = nullptr;

	FJoltWorker* JoltWorker = nullptr;

	UEJoltCallBackContactListener* ContactListener = nullptr;

	JPH::PhysicsSystem* MainPhysicsSystem = nullptr;

	JPH::BodyInterface* BodyInterface = nullptr;

	uint32 StaticBodyIDX;

	uint32 DynamicBodyIDX;

	// Runtime layer lookup table — built from UJoltSettings in InitPhysicsSystem and referenced by
	// all three filter implementations for the lifetime of MainPhysicsSystem.
	FJoltLayerTable LayerTable;

	BPLayerInterfaceImpl* BroadPhaseLayerInterface = nullptr;

	// Create class that filters object vs broadphase layers
	// Note: As this is an interface, PhysicsSystem will take a reference to this so this instance needs to stay alive!
	ObjectVsBroadPhaseLayerFilterImpl* ObjectVsBroadphaseLayerFilter = nullptr;

	// Create class that filters object vs object layers
	// Note: As this is an interface, PhysicsSystem will take a reference to this so this instance needs to stay alive!
	ObjectLayerPairFilterImpl* ObjectVsObjectLayerFilter = nullptr;

	TArray<const JPH::BoxShape*> BoxShapes;

	TArray<const JPH::SphereShape*> SphereShapes;

	TArray<const JPH::CapsuleShape*> CapsuleShapes;

	TArray<const JPH::HeightFieldShapeSettings*> HeightFieldShapes;

	TArray<JPH::Body*> SavedBodies;

	TMap<uint32, JPH::Body*> BodyIDBodyMap;

	// JPH::Array<const JPH::Body*> HeightMapArray;

	// JPH::Array<const JPH::Body*> LandscapeSplines;

	TMap<EPhysicalSurface, const JoltPhysicsMaterial*> SurfaceJoltMaterialMap;

	TMap<EPhysicalSurface, TWeakObjectPtr<const UPhysicalMaterial>> SurfaceUEMaterialMap;

	TArray<FJoltBodyActor> JoltBodyActors;

	TMap<const JPH::BodyID*, FTransform> SkeletalMeshBodyIDLocalTransformMap;

	struct ConvexHullShapeHolder
	{
		const UBodySetup*			BodySetup;
		int							HullIndex;
		FVector						Scale;
		const JPH::ConvexHullShape* Shape;
	};

	TArray<ConvexHullShapeHolder> ConvexShapes;

#ifdef JPH_DEBUG_RENDERER
	UEJoltDebugRenderer* JoltDebugRendererImpl = nullptr;

	JPH::BodyManager::DrawSettings* DrawSettings = nullptr;

	void DrawDebugLines() const;
#endif

	typedef const std::function<void(const JPH::Shape*, const FTransform&)>& PhysicsGeometryCallback;

	void LoadLandscapeFromDataAsset();

	static ALandscape* FindSingleLandscape(const UWorld* world);

#if WITH_EDITOR
	void GetAllLandscapeHeights(const ALandscape* landscapeActor);

	bool BuildLandscapeMaterialIndices(ULandscapeComponent* landscapeComponent, uint32 componentSize, TArray<uint8>& outMaterialIndices, JPH::PhysicsMaterialList& outMaterialList);

	bool CookBodies() const;

	void HandleLandscapeMeshes(const ALandscape* LandscapeActor);

#endif

	void ExtractPhysicsGeometry(const AActor* actor, PhysicsGeometryCallback callback);

	void ExtractPhysicsGeometry(const UStaticMeshComponent* SMC, const FTransform& actorTransform, PhysicsGeometryCallback callback);

	void ExtractPhysicsGeometry(const FTransform& xformSoFar, const UBodySetup* bodySetup, PhysicsGeometryCallback callback);

	void ExtractComplexPhysicsGeometry(const FTransform& xformSoFar, const UBodySetup* bodySetup, const FString& meshName, PhysicsGeometryCallback callback);

	const JPH::Shape* ProcessShapeElement(const UShapeComponent* shapeComponent);

	/*
	 * Fetch all the actors in UE world and add them to jolt simulation
	 * "jolt-static" tag should be added for static objects (from UE editor)
	 * "jolt-dynamic" tag should be added for dynamic objects (from UE editor)
	 */
	void AddAllJoltActors(const UWorld* World);

	void InterpolatePhysicsFrame(const double& alpha);

	void ApplyLocalTxIfAny(const JPH::BodyID* bodyID, FTransform& interpolatedTransform) const;

	virtual void Tick(float deltaSeconds) override;

	virtual TStatId GetStatId() const override;

	virtual void Initialize(FSubsystemCollectionBase& Collection) override;

	virtual void Deinitialize() override;

	virtual void OnWorldBeginPlay(UWorld& InWorld) override;

	virtual void OnWorldComponentsUpdated(UWorld& InWorld) override;

	void InitPhysicsSystem(
		int cMaxBodies,
		int cNumBodyMutexes,
		int cMaxBodyPairs,
		int cMaxContactConstraints);

	double CurrentTime = FPlatformTime::Seconds();

	double Accumulator = 0.0f;

	double ConfiguredDeltaSeconds = 1.0f / 60.0f;

	bool bStepPaused = false;

	double PhysicsAlpha_ = 0.0;

	void RecordFrames();

	TMap<const JPH::BodyID*, FFrameHistory> JoltBodyTransformHistory;

	TArray<TDelegate<void(float)>> PostInterpolationCallbacks;
	
	void BroadcastPrePhysicsListeners(float DeltaTime);
	void BroadcastPostPhysicsListeners(float DeltaTime);
	
	UPROPERTY() 
	TArray<TWeakObjectPtr<UObject>> PhysicsListeners;

	bool bIsReady = false;

public:
	FOnJoltReady OnReady;

	/** True once OnWorldBeginPlay has finished initializing the worker — for listeners that bind after BeginPlay. */
	bool IsReady() const { return bIsReady; }

	JPH::Vec3 GetLinearVelocity(const JPH::BodyID& bodyID) const
	{
		return BodyInterface->GetLinearVelocity(bodyID);
	}

	JPH::Vec3 GetAngularVelocity(const JPH::BodyID& bodyID) const
	{
		return BodyInterface->GetAngularVelocity(bodyID);
	}

	uint32 GetNumBodies() const
	{
		return MainPhysicsSystem->GetNumBodies();
	}

	friend class UJoltSkeletalMeshComponent;
	friend class JoltAxisConstraint;
};
