// Fill out your copyright notice in the Description page of Project Settings.

#pragma once

#include "Containers/Array.h"
#include "CoreMinimal.h"
#include "Engine/DataAsset.h"
#include "UObject/ObjectMacros.h"
#include "UnrealJolt/JoltMain.h"

#include "JoltDataAsset.generated.h"

USTRUCT()
struct FJoltShapeMaterialData
{
	GENERATED_USTRUCT_BODY()

	static constexpr uint8 NullMaterialSlot = 255;

	uint8 SurfaceType = NullMaterialSlot;

	float Friction = 0.f;

	float Restitution = 0.f;

	friend FArchive& operator<<(FArchive& Ar, FJoltShapeMaterialData& Material)
	{
		Ar << Material.SurfaceType;
		Ar << Material.Friction;
		Ar << Material.Restitution;
		return Ar;
	}
};

USTRUCT()
struct FJoltShapeData
{
	GENERATED_USTRUCT_BODY()

	TArray<uint8> BinaryData;

	FTransform WorldTransform;

	JPH::EMotionType MotionType;

	FName LayerName;

	float Friction;

	float Restitution;

	TArray<FJoltShapeMaterialData> Materials;

	friend FArchive& operator<<(FArchive& Ar, FJoltShapeData& Component)
	{
		Ar << Component.BinaryData;
		Ar << Component.WorldTransform;
		Ar << Component.MotionType;
		Ar << Component.LayerName;
		Ar << Component.Friction;
		Ar << Component.Restitution;
		Ar << Component.Materials;
		return Ar;
	}
};

/**
 * Data asset to hold the jolt properties, currently it is tied to levels
 */
UCLASS(BlueprintType, EditInlineNew)
class UNREALJOLT_API UJoltDataAsset : public UDataAsset
{
	GENERATED_BODY()

public:
	UJoltDataAsset();

	void AddShapeData(const FJoltShapeData& InData);

	TArray<FJoltShapeData>* GetAllBodyData();

	void GetJoltShapeBinaryData(const JPH::Body* inBody, FJoltShapeData& outShapeBinary) const;

	void LoadBodies(const TArray<JPH::Body*> allBodies);

protected:
	virtual void Serialize(FArchive& Ar) override;

private:
	TArray<FJoltShapeData> SahpeDataArr;
};
