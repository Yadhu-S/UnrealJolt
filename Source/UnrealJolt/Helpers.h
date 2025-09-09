#pragma once
#include "Engine/World.h"
#include "JoltPhysicsMaterial.h"
#include "PhysicalMaterials/PhysicalMaterial.h"
#include "JoltMain.h"

// Jolt scale is 1=1m, UE is 1=1cm So x100
// Also keep in mind that Jolt uses Y axis as the "UP" while UE uses Z.
#define JOLT_TO_WORLD_SCALE 100.f
#define WORLD_TO_JOLT_SCALE 0.01f

class JoltHelpers
{
public:
	// Sizes, float
	inline static float ToUESize(const float& Sz)
	{
		return Sz * JOLT_TO_WORLD_SCALE;
	}

	inline static float ToJoltSize(const float& Sz)
	{
		return Sz * WORLD_TO_JOLT_SCALE;
	}

	inline static JPH::Float3 ToJoltFloat3(const FVector3f& fvector3f)
	{
		return JPH::Float3(fvector3f.X * WORLD_TO_JOLT_SCALE, fvector3f.Z * WORLD_TO_JOLT_SCALE, fvector3f.Y * WORLD_TO_JOLT_SCALE);
	}

	inline static JPH::Vec3 ToJoltVec3(const FVector& Sv, const bool& adjustScale = true)
	{
		if (adjustScale)
			return JPH::Vec3(Sv.X, Sv.Z, Sv.Y) * WORLD_TO_JOLT_SCALE;

		return JPH::Vec3(Sv.X, Sv.Z, Sv.Y);
	}

	inline static FVector ToUESize(const JPH::Vec3& Sv, const bool& adjustScale = true)
	{
		if (adjustScale)
			return FVector(Sv.GetX(), Sv.GetZ(), Sv.GetY()) * JOLT_TO_WORLD_SCALE;

		return FVector(Sv.GetX(), Sv.GetZ(), Sv.GetY());
	}

	inline static FVector ToUEPos(const JPH::RVec3& V, const FVector& WorldOrigin = FVector(0))
	{

		return FVector(V.GetX(), V.GetZ(), V.GetY()) * JOLT_TO_WORLD_SCALE + WorldOrigin;
	}

	inline static JPH::RVec3 ToJoltPos(const FVector& V, const FVector& WorldOrigin = FVector(0))
	{
		return JPH::RVec3(V.X - WorldOrigin.X, V.Z - WorldOrigin.Z, V.Y - WorldOrigin.Y) * WORLD_TO_JOLT_SCALE;
	}

	inline static FQuat ToUERot(const JPH::Quat& Q)
	{
		return FQuat(-Q.GetX(), -Q.GetZ(), -Q.GetY(), Q.GetW());
	}

	inline static JPH::Quat ToJoltRot(const FQuat& Q)
	{
		return JPH::Quat(-Q.X, -Q.Z, -Q.Y, Q.W);
	}

	inline static JPH::Quat ToJoltRot(const FRotator& r)
	{
		return ToJoltRot(r.Quaternion());
	}

	// Transforms
	inline static FTransform ToUETransform(const JPH::RMat44& T, const FVector& WorldOrigin = FVector(0))
	{
		const FQuat	  Rot = ToUERot(T.GetQuaternion());
		const FVector Pos = ToUEPos(T.GetTranslation(), WorldOrigin);
		return FTransform(Rot, Pos);
	}

	inline static JPH::RMat44 ToJoltTransform(const FTransform& T)
	{
		return JPH::RMat44::sRotationTranslation(ToJoltRot(T.GetRotation()), ToJoltPos(T.GetTranslation()));
	}

	inline static FColor ToUEColor(const JPH::Color& C)
	{
		return FColor(C.r, C.g, C.b, C.a);
	}

	inline static void UETrace([[maybe_unused]] const char* inFMT, ...)
	{
		va_list args;
		va_start(args, inFMT);

		char buffer[1024];
		vsnprintf(buffer, sizeof(buffer), inFMT, args);

		va_end(args);

		UE_LOG(JoltSubSystemLogs, Warning, TEXT("JoltPhysicsSubSystem: %s"), ANSI_TO_TCHAR(buffer));
	}

	inline static bool UEAssertFailed(const char* inExpression, const char* inMessage, const char* inFile, uint inLine)
	{
		UE_LOG(JoltSubSystemLogs, Error, TEXT("Assertion failed!"));
		UE_LOG(JoltSubSystemLogs, Error, TEXT("Expression: %s"), ANSI_TO_TCHAR(inExpression));
		if (inMessage)
		{
			UE_LOG(JoltSubSystemLogs, Error, TEXT("Message: %s"), ANSI_TO_TCHAR(inMessage));
		}
		UE_LOG(JoltSubSystemLogs, Error, TEXT("File: %s:%u"), ANSI_TO_TCHAR(inFile), inLine);

		return false;
	}

	inline static void GenerateAssetNames(const UWorld* World, FString& OutPackageName, FString& OutAssetName)
	{
		FString LevelName = TEXT("UnknownLevel");
		if (World)
		{
			LevelName = World->GetMapName();
			LevelName.RemoveFromStart(World->StreamingLevelsPrefix);
		}

		OutPackageName = FString::Printf(TEXT("/Game/JoltData/BinaryData_%s"), *LevelName);
		OutAssetName = FString::Printf(TEXT("BinaryData_%s"), *LevelName);
	}

	inline static JoltPhysicsMaterial* ToJoltPhysicsMaterial(const UPhysicalMaterial* UEPhysicsMat)
	{
		if (!UEPhysicsMat)
			return nullptr;

		JoltPhysicsMaterial* dst = new JoltPhysicsMaterial();
		dst->Friction = UEPhysicsMat->Friction;
		dst->Restitution = UEPhysicsMat->Restitution;
		dst->SurfaceType = UEPhysicsMat->SurfaceType;
		return dst;
	}

	inline static UPhysicalMaterial* ToUEPhysicsMaterial(const JoltPhysicsMaterial* UEPhysicsMat)
	{
		if (!UEPhysicsMat)
			return nullptr;

		UPhysicalMaterial* dst = NewObject<UPhysicalMaterial>();
		dst->Friction = UEPhysicsMat->Friction;
		dst->Restitution = UEPhysicsMat->Restitution;
		dst->SurfaceType = UEPhysicsMat->SurfaceType;
		return dst;
	}

	inline static FString EMotionTypeToString(JPH::EMotionType MotionType)
	{
		switch (MotionType)
		{
			case JPH::EMotionType::Kinematic:
			{
				return TEXT("Kinematic");
			}
			case JPH::EMotionType::Dynamic:
			{
				return TEXT("Dynamic");
			}
			case JPH::EMotionType::Static:
			{
				return TEXT("Static");
			}
			default:
			{
				return TEXT("Invalid Motion type");
			}
		}
	}
};

class FirstRayCastHitCollector : public JPH::CastRayCollector
{
public:
	FirstRayCastHitCollector(const JPH::PhysicsSystem& inPhysicsSystem, const JPH::RRayCast& inRay)
		: mPhysicsSystem(inPhysicsSystem), mRay(inRay)
	{
	}

	virtual void AddHit(const JPH::RayCastResult& inResult) override
	{
		// Test if this collision is closer than the previous one
		if (inResult.mFraction < GetEarlyOutFraction())
		{
			// Lock the body
			JPH::BodyLockRead lock(mPhysicsSystem.GetBodyLockInterfaceNoLock(), inResult.mBodyID);
			JPH_ASSERT(lock.Succeeded()); // When this runs all bodies are locked so this should not fail
			mBody = &lock.GetBody();

			if (mBody->IsSensor())
				return;

			UpdateEarlyOutFraction(inResult.mFraction);

			// Get the contact properties
			if (mBody != nullptr)
			{
				mHasHit = true;
			}
			mSubShapeID2 = inResult.mSubShapeID2;
			mContactPosition = mRay.GetPointOnRay(inResult.mFraction);
			mContactNormal = mBody->GetWorldSpaceSurfaceNormal(inResult.mSubShapeID2, mContactPosition);
			mBodyID = inResult.mBodyID;
		}
	}

	// Configuration
	const JPH::PhysicsSystem& mPhysicsSystem;
	JPH::RRayCast			  mRay;

	// Resulting closest collision
	const JPH::Body* mBody = nullptr;
	JPH::BodyID		 mBodyID;
	JPH::SubShapeID	 mSubShapeID2;
	JPH::RVec3		 mContactPosition;
	JPH::Vec3		 mContactNormal;
	bool			 mHasHit = false;
};

class JoltShapeDataWriter : public JPH::StreamOut
{
public:
	JoltShapeDataWriter(TArray<uint8>& InArray)
		: WrappedArray(InArray)
	{
	}

	virtual void WriteBytes(const void* InData, size_t InNumBytes) override
	{
		int32 CurrentSize = WrappedArray.Num();

		WrappedArray.AddUninitialized(InNumBytes);

		FMemory::Memcpy(WrappedArray.GetData() + CurrentSize, InData, InNumBytes);
	}

	virtual bool IsFailed() const override { return false; };

	TArray<uint8>& GetTArray() { return WrappedArray; }

private:
	TArray<uint8>& WrappedArray;
};

class ShapeDataReader : public JPH::StreamIn
{
public:
	explicit ShapeDataReader(const TArray<uint8>& InData)
		: Data(InData), CurrentPosition(0) {}

	virtual void ReadBytes(void* OutData, size_t InNumBytes) override
	{
		size_t BytesAvailable = Data.Num() - CurrentPosition;
		size_t BytesToRead = FMath::Min(InNumBytes, BytesAvailable);

		if (BytesToRead > 0)
		{
			FMemory::Memcpy(OutData, Data.GetData() + CurrentPosition, BytesToRead);
			CurrentPosition += BytesToRead;
		}
	}

	virtual bool IsEOF() const override
	{
		return CurrentPosition > Data.Num();
	}

	virtual bool IsFailed() const override
	{
		return false;
	}

private:
	const TArray<uint8>& Data;
	size_t				 CurrentPosition;
};
