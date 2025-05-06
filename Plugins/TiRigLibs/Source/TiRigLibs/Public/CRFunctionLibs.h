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
USTRUCT(meta = (DisplayName = "MakeVector", TemplateName = "MakeVectorTemplate", Keywords = "len, product, TiRig, make, vector", Category = "TIRIG CRFunctions"))
struct TIRIGLIBS_API FMakeVector : public FVectorMath_Base
{
	GENERATED_BODY()

	RIGVM_METHOD()
		virtual void Execute() override;

	UPROPERTY(meta = (Input, ToolTip = "First input vector"))
	FVector VectorA = FVector::ZeroVector;

	UPROPERTY(meta = (Input, ToolTip = "Second input vector"))
	FVector VectorB = FVector::ZeroVector;

	UPROPERTY(meta = (Input, ToolTip = "Normalize vectors before operation"))
	bool bNormalized = false;
	UPROPERTY(meta = (Input, ToolTip = "Output vector length instead of difference"))
	bool bGetLen = false;

	UPROPERTY(meta = (Output, ToolTip = "Output vector "))
	FVector OutputVector = FVector(0.f,0.f,0.f);
	UPROPERTY(meta = (Output, ToolTip = "Output len "))
	float len = 1.0f;
};