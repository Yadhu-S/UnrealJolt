// Fill out your copyright notice in the Description page of Project Settings.

#include "JoltDataAsset.h"
#include "JoltPhysicsMaterial.h"
#include "JoltSettings.h"
#include "UnrealJolt/Helpers.h"

UJoltDataAsset::UJoltDataAsset()
{
	// Optionally initialize the binary data
	SahpeDataArr.Empty();
}

void UJoltDataAsset::GetJoltShapeBinaryData(const JPH::Body* inBody, FJoltShapeData& outShapeBinary) const
{
	TArray<uint8> shapeBinaryData = TArray<uint8>();
	// Load our landscapeHeightMapData which is a TArray to our custom wrapper for jolt to save the binary data into
	JoltShapeDataWriter* joltShapeDataWriter = new JoltShapeDataWriter(shapeBinaryData);
	inBody->GetShape()->SaveBinaryState(*joltShapeDataWriter);
	delete joltShapeDataWriter;
	joltShapeDataWriter = nullptr;
	// After we have the binary data of a landscape component, save that in the data asset
	// This binary data will later be then retried during runtime
	outShapeBinary.BinaryData = shapeBinaryData;
	outShapeBinary.MotionType = inBody->GetMotionType();

	const UJoltSettings* Settings = GetDefault<UJoltSettings>();
	const int32 layerIndex = static_cast<int32>(inBody->GetObjectLayer());
	if (Settings != nullptr && Settings->ObjectLayers.IsValidIndex(layerIndex))
	{
		outShapeBinary.LayerName = Settings->ObjectLayers[layerIndex].Name;
	}
	else
	{
		outShapeBinary.LayerName = NAME_None;
	}

	outShapeBinary.Friction = inBody->GetFriction();
	outShapeBinary.Restitution = inBody->GetRestitution();
	outShapeBinary.WorldTransform = JoltHelpers::ToUETransform(inBody->GetWorldTransform());

	// Save material data
	JPH::PhysicsMaterialList materialList;
	inBody->GetShape()->SaveMaterialState(materialList);
	outShapeBinary.Materials.Empty(materialList.size());
	for (const JPH::PhysicsMaterialRefC& material : materialList)
	{
		// Every non-null material on cooked bodies is a JoltPhysicsMaterial (see UJoltSubsystem::GetJoltPhysicsMaterial)
		const JoltPhysicsMaterial* joltMaterial = static_cast<const JoltPhysicsMaterial*>(material.GetPtr());
		FJoltShapeMaterialData materialData;
		if (joltMaterial != nullptr)
		{
			materialData.SurfaceType = static_cast<uint8>(joltMaterial->SurfaceType);
			materialData.Friction = joltMaterial->Friction;
			materialData.Restitution = joltMaterial->Restitution;
		}
		outShapeBinary.Materials.Add(materialData);
	}
}

void UJoltDataAsset::LoadBodies(const TArray<JPH::Body*> allBodies)
{
	for (const JPH::Body* body : allBodies)
	{
		FJoltShapeData data;
		GetJoltShapeBinaryData(body, data);
		AddShapeData(data);
	}
}

void UJoltDataAsset::AddShapeData(const FJoltShapeData& InData)
{
	SahpeDataArr.Add(InData);
}

TArray<FJoltShapeData>* UJoltDataAsset::GetAllBodyData()
{
	return &SahpeDataArr;
}

void UJoltDataAsset::Serialize(FArchive& Ar)
{
	Super::Serialize(Ar);

	Ar << SahpeDataArr;
}
