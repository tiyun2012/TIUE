// Fill out your copyright notice in the Description page of Project Settings.


#include "CRFunctionLibs.h"
#include "Kismet/KismetMathLibrary.h"
#include "Engine/SkeletalMesh.h"
#include "Rigs/RigHierarchyController.h"
#include "Units/RigUnitContext.h"
#include "ControlRig.h"
#include "Components/SkeletalMeshComponent.h"
#include "Units/Execution/RigUnit_PrepareForExecution.h"

FMakeVector_Execute()
{
	//const float dotProductNormalized = UKismetMathLibrary::Dot_VectorVector(UKismetMathLibrary::Normal(VectorA), UKismetMathLibrary::Normal(VectorB));
	FVector vector_ = UKismetMathLibrary::Subtract_VectorVector(VectorA, VectorB);
	if (bNormalized)
		OutputVector = UKismetMathLibrary::Normal(vector_);
	else
		OutputVector = vector_;
	if (bGetLen)
		len = UKismetMathLibrary::VSize(vector_);
	else
		len = 1.0f;
}

FSpawnController_Execute()
{

}

FRigUnit_AddParent_dev_Execute()
{
	

	FRigTransformElement* ChildElement = ExecuteContext.Hierarchy->Find<FRigTransformElement>(Child);
	if (ChildElement == nullptr)
	{
		UE_CONTROLRIG_RIGUNIT_REPORT_WARNING(TEXT("Child item %s does not exist."), *Child.ToString())
			return;
	}

	//FRigTransformElement* ParentElement = ExecuteContext.Hierarchy->Find<FRigTransformElement>(Parent);
	//if (ParentElement == nullptr)
	//{
	//	UE_CONTROLRIG_RIGUNIT_REPORT_WARNING(TEXT("Parent item %s does not exist."), *Parent.ToString())
	//		return;
	//}

	//// Get the hierarchy controller explicitly without needing the EnableControllerBracket
	//if (URigHierarchyController* Controller = ExecuteContext.Hierarchy->GetController(true))
	//{
	//	Controller->AddParent(ChildElement, ParentElement, 0.f, true, false);
	//}
	//else
	//{
	//	UE_CONTROLRIG_RIGUNIT_REPORT_WARNING(TEXT("Cannot retrieve hierarchy controller."));
	//}
}

FRigUnit_Parent_Constraint_Execute()
{
    DECLARE_SCOPE_HIERARCHICAL_COUNTER_FUNC()
    
    URigHierarchy* Hierarchy = ExecuteContext.Hierarchy;
    if (!Hierarchy)
    {
        UE_CONTROLRIG_RIGUNIT_REPORT_ERROR(TEXT("[ERROR]: Hierarchy is null."));
        return;
    }

    if (DrivenItem.Name == NAME_None || !Hierarchy->Contains(DrivenItem))
    {
        UE_CONTROLRIG_RIGUNIT_REPORT_ERROR(TEXT("[ERROR]: Invalid or missing DrivenItem: %s"), *DrivenItem.Name.ToString());
        return;
    }
    if (Driver.Name == NAME_None || !Hierarchy->Contains(Driver))
    {
        UE_CONTROLRIG_RIGUNIT_REPORT_ERROR(TEXT("[ERROR]: Invalid or missing Driver: %s"), *Driver.Name.ToString());
        return;
    }
    //MetaData Driver.Name_DrivenItem.Name_RelativeTransform
    FName RelativeTransform = FName((Driver.Name.ToString())+TEXT("_") + (DrivenItem.Name.ToString()) + TEXT("_RelativeTransform"));
    Hierarchy->SetBoolMetadata(DrivenItem, RelativeTransform, false);
    //Hierarchy->getMetaDatastora
        // Compute offset each frame
	FTransform SafeOffset = FTransform::Identity;
    if (MaintainOffset)
    {
        const FTransform DriverTransform = Hierarchy->GetGlobalTransform(Driver, true);
        const FTransform DrivenTransform = Hierarchy->GetGlobalTransform(DrivenItem, true);

        if (!DriverTransform.IsValid() || !DrivenTransform.IsValid() || DriverTransform.GetScale3D().IsNearlyZero() || DrivenTransform.GetScale3D().IsNearlyZero())
        {
            UE_CONTROLRIG_RIGUNIT_REPORT_ERROR(TEXT("[ERROR]: Invalid DriverTransform or DrivenTransform"));
            return;
        }

        SafeOffset = DrivenTransform.GetRelativeTransform(DriverTransform);
        if (!SafeOffset.IsValid() || SafeOffset.GetScale3D().IsNearlyZero())
        {
            UE_CONTROLRIG_RIGUNIT_REPORT_WARNING(TEXT("[WARNING]: Invalid SafeOffset computed. Using Identity."));
            SafeOffset = FTransform::Identity;
        }
    }

    // Log SafeOffset
    UE_CONTROLRIG_RIGUNIT_LOG_MESSAGE(TEXT("[DEBUG]: SafeOffset: Location=%s, Rotation=%s, Scale=%s"),
        *SafeOffset.GetLocation().ToString(),
        *SafeOffset.GetRotation().ToString(),
        *SafeOffset.GetScale3D().ToString());

    // Apply constraint
    const FTransform DriverTransform = Hierarchy->GetGlobalTransform(Driver);
    if (!DriverTransform.IsValid() || DriverTransform.GetScale3D().IsNearlyZero())
    {
        UE_CONTROLRIG_RIGUNIT_REPORT_ERROR(TEXT("[ERROR]: Invalid DriverTransform"));
        return;
    }

    const FTransform NewTransform = MaintainOffset ? SafeOffset * DriverTransform : DriverTransform;
    if (!NewTransform.IsValid() || NewTransform.GetScale3D().IsNearlyZero())
    {
        UE_CONTROLRIG_RIGUNIT_REPORT_ERROR(TEXT("[ERROR]: Invalid NewTransform for DrivenItem: %s"), *DrivenItem.Name.ToString());
        return;
    }

    // Log NewTransform
    UE_CONTROLRIG_RIGUNIT_LOG_MESSAGE(TEXT("[DEBUG]: NewTransform: Location=%s, Rotation=%s, Scale=%s"),
        *NewTransform.GetLocation().ToString(),
        *NewTransform.GetRotation().ToString(),
        *NewTransform.GetScale3D().ToString());

    Hierarchy->SetGlobalTransform(DrivenItem, NewTransform);
    FString Even = (ExecuteContext.GetFunctionName()).ToString();
    //FString::Printf(TEXT("%d"), 1.0f);
   
}


/*
*/
FRigUnit_CachedTransform_Execute()
{
    Count++;
    FVector Inc(0,0,0);
    if (Count <= NumberDo)
    {
        Inc = FVector(Count, Count, Count);
        //UE_LOG(LogTemp, Log, TEXT("[DEBUG]: ---COUNT___ :%s "), *(Inc.ToString()));
        CountEnd = Count;
        //CachedTransform= UKismetMathLibrary::MakeTransform(FVector(Count,Count,Count),FRotator(),FVector());
        CachedTransform = CacheTransform;
        return;
    }
	//CachedTransform = UKismetMathLibrary::MakeTransform(FVector(Count, Count, Count), FRotator(), FVector());

    
}

FMyRigUnit_CCDIK_Execute()
{

    DECLARE_SCOPE_HIERARCHICAL_COUNTER_FUNC();

    URigHierarchy* Hierarchy = ExecuteContext.Hierarchy;
    if (!Hierarchy || BoneChain.Num() < 2)
    {
        
        return;
    }
    const FVector TargetPos = EffectorTransform.GetLocation();
    for (int32 Iter = 0; Iter < MaxIterations; ++Iter)
    {
        bool bDidSomething = false;

        for (int32 i = BoneChain.Num() - 2; i >= 0; --i)
        {
            const FRigElementKey& BoneKey = BoneChain[i];
            const FRigElementKey& EffKey = BoneChain.Last();

            FTransform BoneGlobal = Hierarchy->GetGlobalTransform(BoneKey);
            FTransform EffGlobal = Hierarchy->GetGlobalTransform(EffKey);

            const FVector BonePos = BoneGlobal.GetLocation();
            const FVector EffPos = EffGlobal.GetLocation();

            FVector ToEff = EffPos - BonePos;
            FVector ToTarg = TargetPos - BonePos;
            if (ToEff.IsNearlyZero() || ToTarg.IsNearlyZero())
            {
                continue;
            }
            ToEff.Normalize();
            ToTarg.Normalize();

            float CosAngle = FVector::DotProduct(ToEff, ToTarg);
            float Angle = FMath::Acos(FMath::Clamp(CosAngle, -1.f, 1.f));
            if (Angle <= FMath::DegreesToRadians(Precision))
            {
                continue;
            }

            const FVector Axis = FVector::CrossProduct(ToEff, ToTarg).GetSafeNormal();
            const FQuat   DeltaRot = FQuat(Axis, Angle);

            FQuat NewQuat = (DeltaRot * BoneGlobal.GetRotation()).GetNormalized();
            BoneGlobal.SetRotation(NewQuat);

            Hierarchy->SetGlobalTransform(BoneKey, BoneGlobal, bPropagateToChildren);

            bDidSomething = true;
        }

        const FVector NewEffPos = Hierarchy->GetGlobalTransform(BoneChain.Last()).GetLocation();
        if (!bDidSomething || FVector::Dist(NewEffPos, TargetPos) <= Precision)
        {
            break;
        }
    }

}