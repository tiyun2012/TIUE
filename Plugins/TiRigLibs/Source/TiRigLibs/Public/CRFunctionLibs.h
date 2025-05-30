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
USTRUCT(meta = (DisplayName = "CacheTransform",
	Category = "TIRIG|Utilities",
	Keywords = "Tranform,Cached,do once"))
	struct FRigUnit_CachedTransform : public FRigUnitMutable
{
	GENERATED_BODY()

	FRigUnit_CachedTransform()
		: CacheTransform(FTransform::Identity)
		, CachedTransform(FTransform::Identity)
		, Count(0)
		,NumberDo(1)
	{
		

		/*
		*/

	}

	/** Item that will follow Driver */
	UPROPERTY(meta = (Input))
	FTransform CacheTransform;

	UPROPERTY(meta=(Transient,Output))
	FTransform CachedTransform;
	UPROPERTY(meta = (Output))

	int32 Count;
	UPROPERTY(meta = (Transient,OutPut))
	int32 CountEnd;

	UPROPERTY(meta = (Input,ToolTip="cached after NumberDo execute"))
	int32 NumberDo;

	RIGVM_METHOD()
		virtual void Execute() override;
};

/**
 * A simple CCDIK implementation as a Control Rig unit.
 */
USTRUCT(meta = (DisplayName = "My CCDIK", Category = "TIRIG|Custom IK", Keywords = "CCDIK,IK"))
struct FMyRigUnit_CCDIK : public FRigUnitMutable
{
	GENERATED_BODY()

	FMyRigUnit_CCDIK()
		:EffectorTransform(FTransform::Identity)
		, Precision(0.1f)
		, MaxIterations(5)
		, bPropagateToChildren(false)
		, bDebug(false)
	{
	}
	/** Ordered list of bone names from root → effector */
	UPROPERTY(meta = (Input, ExpandByDefault))
	FRigElementKeyCollection BoneChain;
	
	/** Desired world transform of the effector */
	UPROPERTY(meta = (Input))
	FTransform EffectorTransform;

	/** Stopping distance in Unreal units */
	UPROPERTY(meta = (Input, UIMin = "0.001", UIMax = "1.0"))
	float Precision;

	/** Maximum number of CCD iterations */
	UPROPERTY(meta = (Input, UIMin = "1", UIMax = "100"))
	int32 MaxIterations;
	/**
	 * Bias exponent for distributing angle.
	 *   0 = uniform,
	 *  >0 = more rotation on bones near the root,
	 *  <0 = more rotation on bones near the effector
	 */
	UPROPERTY(meta = (Input, UIMin = "-5.0", UIMax = "5.0"))
	float RootBias;
	/** If true, child bones will be affected by upstream rotations */
	UPROPERTY(meta = (Input))
	bool bPropagateToChildren;
	UPROPERTY(meta = (Input))
	bool bDebug;

	// Executes once per Evaluate() call
	RIGVM_METHOD()
		virtual void Execute() override;
};

USTRUCT(meta = (DisplayName = "Two Bone IK Custom", Category = "TIRIG|Hierarchy", Keywords = "IK, Two Bone"))
struct FRigUnit_TwoBoneIKCustom : public FRigUnitMutable
{
	GENERATED_BODY()

	FRigUnit_TwoBoneIKCustom()
		: 
		FirstBoneScale(1.f)
		, SecondBoneScale(1.f)
		, PolePin(0.f)
		, Stretchy(0.0f)
		, bPropagateToChildren(true)
		, bDebug(true)
		,Thickness(0.0f)
		, bIsCached(false)
		, RootRelativeCached(FTransform::Identity)
		, MidRelativeCached(FTransform::Identity)
		, EndRelativeCached(FTransform::Identity)
		, P0RootControlRelativeCached(FVector())
		, initL1Cached(1.0f)
		,initL2Cached(1.0f)
	{
	}

	/** The root bone of the chain (P0) */
	UPROPERTY(meta = (Input, ExpandByDefault))
	FRigElementKey BoneRoot;

	/** The middle joint of the chain (P1) */
	UPROPERTY(meta = (Input))
	FRigElementKey BoneMid;

	/** The end joint of the chain (P2) */
	UPROPERTY(meta = (Input))
	FRigElementKey BoneEnd;
	/** The RigElementKey will move root bone */
	UPROPERTY(meta = (Input))
	FRigElementKey ParentControl;
	/** The the RigElement key will be poleVector control */
	UPROPERTY(meta = (Input))
	FRigElementKey PoleItem;

	/** Target the end joint should try to reach */
	UPROPERTY(meta = (Input))
	FRigElementKey TargetItem;

	
	/** Scale factor for the first bone length (default=1.0) */
	UPROPERTY(meta = (Input, UIMin = "0.0", UIMax = "100.0"))
	float FirstBoneScale;

	/** Scale factor for the second bone length (default=1.0) */
	UPROPERTY(meta = (Input, UIMin = "0.0", UIMax = "100.0"))
	float SecondBoneScale;

	/** Blend between original bone length and pole distance (0=use scaled length, 1=use pole distance) */
	UPROPERTY(meta = (Input, UIMin = "0.0", UIMax = "1.0"))
	float PolePin;
	/** when joints are in the same line, it will be tretch with factor strechy (0= no stretching, 1= will reach target) */
	UPROPERTY(meta=(Input, UIMin = "0.0", UIMax = "1.0"))
	float Stretchy;
	UPROPERTY(meta = (Input))
	bool bPropagateToChildren;
	UPROPERTY(meta = (Input))
	bool bDebug;
	UPROPERTY(meta = (Input, UIMin = "0.0", UIMax = "2.0"))
	float Thickness;
	/** If true, propagate the change down the hierarchy */
	UPROPERTY()
	bool bIsCached;
	
	UPROPERTY()
	FTransform RootRelativeCached;
	UPROPERTY()
	FTransform MidRelativeCached;
	UPROPERTY()
	FTransform EndRelativeCached;
	UPROPERTY()
	FVector P0RootControlRelativeCached;
	UPROPERTY()
	float initL1Cached;
	UPROPERTY()
	float initL2Cached;
	RIGVM_METHOD()
		virtual void Execute() override;
};

USTRUCT(meta = (DisplayName = "quadLeg IK ", Category = "TIRIG|Hierarchy", Keywords = "IK, Two Bone,Quad"))
struct FRigUnit_QuadLegIK : public FRigUnitMutable
{
	GENERATED_BODY()

	// Constructor with member initializer list
	FRigUnit_QuadLegIK()
		: Joint1(FRigElementKey(NAME_None, ERigElementType::Bone))
		, Joint2(FRigElementKey(NAME_None, ERigElementType::Bone))
		, Joint3(FRigElementKey(NAME_None, ERigElementType::Bone))
		, Joint4(FRigElementKey(NAME_None, ERigElementType::Bone))
		, ParentControl(FRigElementKey(NAME_None, ERigElementType::Bone))
		, PoleItem(FRigElementKey(NAME_None, ERigElementType::Bone))
		, TargetItem(FRigElementKey(NAME_None, ERigElementType::Bone))
		, bLimitJoint3Rotate(true)
		, maxRotateIn(10.f)
		, RatioClosestPoint(0.5f)
		, maxRotateOut(10.f)
		, RatioFarthestPoint(1.25f) // Renamed for spelling
		, FirstBoneScale(1.f)
		, SecondBoneScale(1.f)
		, PolePin(0.f)
		, Stretchy(0.f)
		, bPropagateToChildren(true)
		, bDebug(true)
		, Thickness(0.f)
		, bIsCached(false)
		, Joint1Joint4Dis(0.0f)
		, Joint1RelativeCached(FTransform::Identity)
		, Joint1LoInParentControlCached(FVector())
		, Joint2AimCached(FTransform::Identity)
		, Joint2RelativeCached(FTransform::Identity)
		, Joint3AimCached(FTransform::Identity)
		, Joint3RelativeCached(FTransform::Identity)
	{
	}

	// Bone references
	UPROPERTY(meta = (Input, ExpandByDefault))
	FRigElementKey Joint1;
	UPROPERTY(meta = (Input, ExpandByDefault))
	FRigElementKey Joint2;
	UPROPERTY(meta = (Input, ExpandByDefault))
	FRigElementKey Joint3;
	UPROPERTY(meta = (Input, ExpandByDefault))
	FRigElementKey Joint4;
	UPROPERTY(meta = (Input, ExpandByDefault))
	FRigElementKey ParentControl;
	UPROPERTY(meta = (Input, ExpandByDefault))
	FRigElementKey PoleItem;
	UPROPERTY(meta = (Input, ExpandByDefault))
	FRigElementKey TargetItem;

	/** Limit rotation of Joint3 */
	UPROPERTY(meta = (Input, ExpandByDefault))
	bool bLimitJoint3Rotate;

	/** Maximum inward rotation (in degrees) for Joint3 when target is closest to Joint1 (0 = no rotate in) */
	UPROPERTY(meta = (Input))
	float maxRotateIn;

	/** Ratio for closest point along the chain (0-1), at which Joint3 will rotate in by maxRotateIn */
	UPROPERTY(meta = (Input))
	float RatioClosestPoint;

	/** Maximum outward rotation (in degrees) for Joint3 when target is farthest from Joint1 */
	UPROPERTY(meta = (Input))
	float maxRotateOut;

	/** Ratio for farthest point along the chain (0-1), at which Joint3 will rotate out by maxRotateOut */
	UPROPERTY(meta = (Input))
	float RatioFarthestPoint; // fixed spelling

	/** Scale for the first bone */
	UPROPERTY(meta = (Input, UIMin = "0.0", UIMax = "100.0"))
	float FirstBoneScale;

	/** Scale for the second bone */
	UPROPERTY(meta = (Input, UIMin = "0.0", UIMax = "100.0"))
	float SecondBoneScale;

	/** Pinning amount for the pole vector (0-1) */
	UPROPERTY(meta = (Input, UIMin = "0.0", UIMax = "1.0"))
	float PolePin;

	/** Amount of stretchiness (0-1) */
	UPROPERTY(meta = (Input, UIMin = "0.0", UIMax = "1.0"))
	float Stretchy;

	/** Whether to propagate changes to children */
	UPROPERTY(meta = (Input))
	bool bPropagateToChildren;

	/** Enable debug drawing */
	UPROPERTY(meta = (Input))
	bool bDebug;

	/** Debug visualization thickness (0-2) */
	UPROPERTY(meta = (Input, UIMin = "0.0", UIMax = "2.0"))
	float Thickness;

	// Internal state cache (should not be exposed to Blueprint)
	UPROPERTY()
	bool bIsCached;
	UPROPERTY()
	float Joint1Joint4Dis;
	UPROPERTY()
	FTransform Joint1RelativeCached;
	UPROPERTY()
	FVector Joint1LoInParentControlCached;
	UPROPERTY()
	FTransform Joint2AimCached;
	UPROPERTY()
	FTransform Joint2RelativeCached;
	UPROPERTY()
	FTransform Joint3AimCached;
	UPROPERTY()
	FTransform Joint3RelativeCached;
	RIGVM_METHOD()
		virtual void Execute() override;
};