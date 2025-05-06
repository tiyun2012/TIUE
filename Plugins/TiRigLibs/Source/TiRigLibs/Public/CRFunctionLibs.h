// Fill out your copyright notice in the Description page of Project Settings.

#pragma once

#include "CoreMinimal.h"
#include "RigVMCore/RigVMStruct.h"
#include "CRFunctionLibs.generated.h"

USTRUCT(meta = (Abstract, NodeColor = "0.60 0.00 1.00"))
struct TIRIGLIBS_API FVectorMath_Base : public FRigVMStruct
{
	GENERATED_BODY()

	virtual void Execute() {};
};
USTRUCT(meta = (DisplayName = "MakeVector", TemplateName = "MakeVectorTemplate", Keywords = "len, product, TiRig, make, vector", Category = "CRFunctions"))
struct TIRIGLIBS_API FMakeVector : public FVectorMath_Base
{
	GENERATED_BODY()

	RIGVM_METHOD()
		virtual void Execute() override;

	UPROPERTY(meta = (Input))
	FVector VectorA = FVector::ZeroVector;

	UPROPERTY(meta = (Input))
	FVector VectorB = FVector::ZeroVector;

	UPROPERTY(meta = (Input))
	bool bIsRotationNormalized = false;

	UPROPERTY(meta = (Output))
	float Result = 0.f;
};