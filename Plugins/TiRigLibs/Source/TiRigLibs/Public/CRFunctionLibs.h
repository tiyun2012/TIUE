﻿// Fill out your copyright notice in the Description page of Project Settings.

#pragma once

#include "CoreMinimal.h"
#include "RigVMCore/RigVMStruct.h"
#include "Units/RigUnit.h"
#include "ControlRigDefines.h" 
#include "ControlRig.h"

#include "CRFunctionLibs.generated.h"

USTRUCT(meta = (Abstract, NodeColor = "0.60 0.00 1.00", Category = "TIRIG|Math|Vector"))
struct TIRIGLIBS_API FVectorMath_Base : public FRigVMStruct
{
	GENERATED_BODY()

	virtual void Execute() {};
};
USTRUCT(meta = (DisplayName = "MakeVector", TemplateName = "MakeVectorTemplate", Keywords = "len, product, TiRig, make, vector"))
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

USTRUCT(meta = (DisplayName = "Spawn Controller", Category = "TIRIG|CRFunctions"))
struct TIRIGLIBS_API FSpawnController : public FRigVMStruct
{
	GENERATED_BODY()

	RIGVM_METHOD()
		virtual void Execute() override;

	UPROPERTY(meta = (Input, Constant, CustomWidget = "BoneName"))
	FName ControllerName;

	UPROPERTY(meta = (Input))
	FTransform InitialTransform;

	UPROPERTY(meta = (Input))
	bool bParentToHierarchy = true;

	UPROPERTY(meta = (Output))
	FRigElementKey CreatedControllerKey;
};


USTRUCT(meta = (Abstract, NodeColor = "0.262745, 0.8, 0, 0.229412", Category = "TIRIG|DynamicHierarchy"))
struct TIRIGLIBS_API FRigUnit_DynamicHierarchyBase_dev : public FRigUnit
{
	GENERATED_BODY()

	static bool IsValidToRunInContext(
		const FControlRigExecuteContext& InExecuteContext,
		bool bAllowOnlyConstructionEvent,
		FString* OutErrorMessage = nullptr);
};



USTRUCT(meta = (Abstract, NodeColor = "0.262745, 0.8, 0, 0.229412", Category = "TIRIG|DynamicHierarchy"))
struct TIRIGLIBS_API FRigUnit_DynamicHierarchyBaseMutable_dev : public FRigUnitMutable
{
	GENERATED_BODY()
};

/**
* Adds a new parent to an element. The weight for the new parent will be 0.0.
* You can use the SetParentWeights node to change the parent weights later.
*/
USTRUCT(meta = (DisplayName = "Add Parent", Keywords = "Children,Parent,Constraint,Space", Varying))
struct TIRIGLIBS_API FRigUnit_AddParent_dev: public FRigUnit_DynamicHierarchyBaseMutable_dev
{
	GENERATED_BODY()

	FRigUnit_AddParent_dev()
	{
		Child = Parent = FRigElementKey(NAME_None, ERigElementType::Control);
	}

	RIGVM_METHOD()
		virtual void Execute() override;

	/*
	 * The child to be parented under the new parent
	 */
	UPROPERTY(meta = (Input, ExpandByDefault))
	FRigElementKey Child;

	/*
	 * The new parent to be added to the child
	 */
	UPROPERTY(meta = (Input, ExpandByDefault))
	FRigElementKey Parent;
};


/**
 *  Constrains DrivenItem to Driver (parent-like) while optionally caching
 *  an offset on the first tick.
 */
USTRUCT(meta = (DisplayName = "Parent Constraint",
	Category = "TIRIG|Constraints",
	Keywords = "Parent,Constraint"))
	struct FRigUnit_Parent_Constraint : public FRigUnitMutable
{
	GENERATED_BODY()

	FRigUnit_Parent_Constraint()
		: DrivenItem(NAME_None, ERigElementType::Bone)
		, Driver(NAME_None, ERigElementType::Bone)
		, MaintainOffset(true)
		, bInitialized(false)
		, CachedOffset(FTransform::Identity)
		, Count_(0)
	{
		//ExecuteContext.GetNumExecutions
		//UE_LOG(LogTemp, Log, TEXT("[DEBUG]: FRigUnit_Parent_Constraint constructor called :%s "),*CachedOffset.ToString());
	}

	/** Item that will follow Driver */
	UPROPERTY(meta = (Input))
	FRigElementKey DrivenItem;

	/** Item that drives the constraint */
	UPROPERTY(meta = (Input))
	FRigElementKey Driver;

	/** If true, compute and keep an offset on first tick */
	UPROPERTY(meta = (Input))
	bool MaintainOffset;

	/** Flag to track if offset is initialized */
	UPROPERTY(meta = (Input, Output,Hidden))
	bool bInitialized;

	/** Internal cache for offset */
	UPROPERTY(meta = (Transient)) // 
		FTransform CachedOffset;
	UPROPERTY(meta = (Input, Output,Hidden))
	int32 Count_ = 0;
	RIGVM_METHOD()
		virtual void Execute() override;
};

/*
*  cache transform
* don once
*
*/
USTRUCT(meta = (DisplayName = "Cache Constraint",
	Category = "TIRIG|Utilities",
	Keywords = "Tranform,Cached,do once"))
	struct FRigUnit_CachedTransform : public FRigUnitMutable
{
	GENERATED_BODY()

	FRigUnit_CachedTransform()
		: CacheTransform(FTransform::Identity)
		, Count(0)
		,NumberDo(1)
	{
		

		/*
		*/

	}

	/** Item that will follow Driver */
	UPROPERTY(meta = (Input,Transient,Output))
	FTransform CacheTransform;
	UPROPERTY(meta=(Transient,Output))
	int32 Count;
	UPROPERTY(meta = (Transient,OutPut))
	int32 CountEnd;
	UPROPERTY(meta = (Input))
	int32 NumberDo;
	RIGVM_METHOD()
		virtual void Execute() override;
};