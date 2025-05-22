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
    const int32 NumJoints = BoneChain.Num();
    if (!Hierarchy || NumJoints < 2)
    {
        return;
    }

    // 1) Gather **initial** joint positions (use bInitial=true)
    TArray<FVector> Positions;
    Positions.SetNum(NumJoints);
    for (int32 i = 0; i < NumJoints; ++i)
    {
        Positions[i] = Hierarchy->GetGlobalTransform(BoneChain[i], /*bInitial=*/true)
            .GetLocation();
    }

    const FVector RootPos = Positions[0];
    const FVector TargetPos = EffectorTransform.GetLocation();
    const float   RootToTarget = FVector::Distance(RootPos, TargetPos);

    // 2) Compute each segment length & total chain length (initial pose)
    TArray<float> SegmentLen;
    SegmentLen.SetNum(NumJoints - 1);
    float SumLen = 0.f;
    for (int32 i = 0; i < NumJoints - 1; ++i)
    {
        SegmentLen[i] = FVector::Distance(Positions[i], Positions[i + 1]);
        SumLen += SegmentLen[i];
    }
    if (SumLen <= KINDA_SMALL_NUMBER)
    {
        return;
    }

    // 3) Ratios for each segment
    TArray<float> Ratio;
    Ratio.SetNum(NumJoints - 1);
    for (int32 i = 0; i < NumJoints - 1; ++i)
    {
        Ratio[i] = SegmentLen[i] / SumLen;
    }

    // 4) Clamp how far we can reach
    const float Reach = FMath::Min(RootToTarget, SumLen);

    // 5) Build the root→target direction
    FVector Dir = TargetPos - RootPos;
    const float DirLen = Dir.Size();
    if (DirLen <= KINDA_SMALL_NUMBER)
    {
        return;
    }
    Dir /= DirLen;

    // 6) Iteratively rotate each segment toward its sub-target
    for (int32 Iter = 0; Iter < MaxIterations; ++Iter)
    {
        bool bDidSomething = false;

        for (int32 i = 0; i < NumJoints - 1; ++i)
        {
            const FRigElementKey& BoneKey = BoneChain[i];
            const FRigElementKey& NextKey = BoneChain[i + 1];

            // use **current** transforms here
            FTransform BoneGlobal = Hierarchy->GetGlobalTransform(BoneKey);
            FTransform NextGlobal = Hierarchy->GetGlobalTransform(NextKey);

            const FVector BonePos = BoneGlobal.GetLocation();
            const FVector ChildPos = NextGlobal.GetLocation();

            // 6a) compute this sub-target along the root→target ray
            const float   Li = Ratio[i] * Reach;
            const FVector SubTgt = RootPos + Dir * Li;

            // 6b) current vs desired direction
            FVector CurrDir = ChildPos - BonePos;
            const float CurrLen = CurrDir.Size();
            if (CurrLen <= KINDA_SMALL_NUMBER) continue;
            CurrDir /= CurrLen;

            FVector WantDir = SubTgt - BonePos;
            const float WantLen = WantDir.Size();
            if (WantLen <= KINDA_SMALL_NUMBER) continue;
            WantDir /= WantLen;

            // 6c) angle & axis
            const float CosA = FVector::DotProduct(CurrDir, WantDir);
            const float Angle = FMath::Acos(FMath::Clamp(CosA, -1.f, 1.f));
            if (Angle <= FMath::DegreesToRadians(Precision)) continue;

            const FVector Axis = FVector::CrossProduct(CurrDir, WantDir).GetSafeNormal();
            const FQuat   DeltaRot(Axis, Angle);

            // apply
            FQuat NewRot = (DeltaRot * BoneGlobal.GetRotation()).GetNormalized();
            BoneGlobal.SetRotation(NewRot);
            Hierarchy->SetGlobalTransform(BoneKey, BoneGlobal, bPropagateToChildren);

            bDidSomething = true;
        }

        if (!bDidSomething)
        {
            break;
        }
    }

    // 7) Final root‐level tweak to align the effector exactly
    {
        FVector NewEffPos = Hierarchy->GetGlobalTransform(BoneChain.Last()).GetLocation();
        FVector NewDir = NewEffPos - RootPos;
        const float NewLen = NewDir.Size();
        if (NewLen > KINDA_SMALL_NUMBER)
        {
            NewDir /= NewLen;
            const float CosA = FVector::DotProduct(NewDir, Dir);
            const float Angle = FMath::Acos(FMath::Clamp(CosA, -1.f, 1.f));
            if (Angle > KINDA_SMALL_NUMBER)
            {
                const FVector Axis = FVector::CrossProduct(NewDir, Dir).GetSafeNormal();
                const FQuat   Delta(Axis, Angle);

                // modify **current** root transform
                FTransform RootGlobal = Hierarchy->GetGlobalTransform(BoneChain[0]);
                FQuat FinalRot = (Delta * RootGlobal.GetRotation()).GetNormalized();
                RootGlobal.SetRotation(FinalRot);
                Hierarchy->SetGlobalTransform(BoneChain[0], RootGlobal, bPropagateToChildren);
            }
        }
    }
}

FRigUnit_TwoBoneIKCustom_Execute()
{
    DECLARE_SCOPE_HIERARCHICAL_COUNTER_FUNC();
    URigHierarchy* Hierarchy = ExecuteContext.Hierarchy;
    if (!Hierarchy) return;

    // clamp stretchy

    // 1) Read initial (default) and current globals
    const FVector P0Init = Hierarchy->GetGlobalTransform(BoneRoot, true).GetLocation();
    const FVector P1Init = Hierarchy->GetGlobalTransform(BoneMid, true).GetLocation();
    const FVector P2Init = Hierarchy->GetGlobalTransform(BoneEnd, true).GetLocation();

    const FTransform RootXf = Hierarchy->GetGlobalTransform(BoneRoot);
    const FTransform MidXf = Hierarchy->GetGlobalTransform(BoneMid);
    const FVector    P0 = RootXf.GetLocation();
    const FVector    P1 = MidXf.GetLocation();
    const FVector    P2 = Hierarchy->GetGlobalTransform(BoneEnd).GetLocation();

    // 2) Pole‐distances
    const float joint1Pole = (PoleVectorTarget - P0).Size();
    const float joint2Pole = (PoleVectorTarget - P2).Size();

    // 3) Unscaled lengths (dynamic vs initial)
    const float dynL1 = (P1 - P0).Size();
    const float dynL2 = (P2 - P1).Size();
    const float initL1 = (P1Init - P0Init).Size();
    const float initL2 = (P2Init - P1Init).Size();
    const float initTotal = initL1 + initL2;

    // 4) Blend between scaled length and pole‐distance
    const float blendedL1 = FMath::Lerp(initL1 * FirstBoneScale, joint1Pole, PolePin);
    const float blendedL2 = FMath::Lerp(initL2 * SecondBoneScale, joint2Pole, PolePin);

    // 5) Target vector & raw distance
    const FVector ToTarget = Target - P0;
    const float   Dist = ToTarget.Size();

    // 6) Stretchy‐blend towards keeping proportions of initTotal
	const float L1= (Dist > (blendedL1+ blendedL2)) ? ((Dist*blendedL1/ (blendedL1+blendedL2))*Stretchy+(1-Stretchy)* blendedL1) : blendedL1;
    //const float L1 = FMath::Lerp(blendedL1, (initL1 / initTotal) * Dist, Stretchy);
    const float L2 = (Dist > (blendedL1 + blendedL2)) ? ((Dist * blendedL2 / (blendedL1 + blendedL2)) * Stretchy + (1 - Stretchy) * blendedL2) : blendedL2;

    //const float L2 = FMath::Lerp(blendedL2, (initL2 / initTotal) * Dist, Stretchy);

    // 7) Clamp target distance against L1+L2
    const float MinD = FMath::Max(KINDA_SMALL_NUMBER, FMath::Abs(L1 - L2));
    const float MaxD = L1 + L2;
    const float ClampedD = FMath::Clamp(Dist, MinD, MaxD);

    // 8) Law of cosines for the shoulder angle
    const float Cos0Value = (ClampedD * ClampedD + L1 * L1 - L2 * L2) / (2 * ClampedD * L1);
    const float Angle0 = FMath::Acos(FMath::Clamp(Cos0Value, -1.f, 1.f));

    // 9) Compute pole‐dir
    FVector poleDir = PoleVectorTarget - P0;
    if (poleDir.SizeSquared() < KINDA_SMALL_NUMBER) poleDir = FVector::UpVector;
    poleDir.Normalize();

    // 10) Build bend‐plane basis
    const FVector Dir = (Dist > KINDA_SMALL_NUMBER) ? ToTarget / Dist : FVector::ForwardVector;
    const FVector Normal = FVector::CrossProduct(Dir, poleDir).GetSafeNormal();
    const FVector BiTangent = FVector::CrossProduct(Normal, Dir).GetSafeNormal();

    // 11) Desired first‐bone direction
    const FVector DesiredDir1 = Dir * FMath::Cos(Angle0)
        + BiTangent * FMath::Sin(Angle0);

    // 12) Shoulder (root) rotation
    const FVector CurrDir1 = (P1 - P0).GetSafeNormal();
    const FQuat   RootRot = FQuat::FindBetweenNormals(CurrDir1, DesiredDir1);

    // 13) New mid‐joint position
    const FVector NewP1 = P0 + RootRot.RotateVector(CurrDir1 * L1);

    // 14) Elbow (mid) rotation in world space
    const FVector CurrDir2Rot = RootRot.RotateVector((P2 - P1).GetSafeNormal());
    const FVector DesiredDir2 = (Target - NewP1).GetSafeNormal();
    const FQuat   MidRot = FQuat::FindBetweenNormals(CurrDir2Rot, DesiredDir2);

    // 15) Apply Root (no propagate)
    {
        FTransform NewRootXf = RootXf;
        NewRootXf.SetRotation(RootRot * RootXf.GetRotation());
        Hierarchy->SetGlobalTransform(
            BoneRoot,
            NewRootXf,
            /*bInitial=*/false,
            /*bPropagateToChildren=*/false
        );
    }

    // 16) Apply Mid (no propagate)
    {
        FQuat MidGlobalQuat = RootRot * MidXf.GetRotation();
        MidGlobalQuat = MidRot * MidGlobalQuat;

        FTransform NewMidXf = MidXf;
        NewMidXf.SetLocation(NewP1);
        NewMidXf.SetRotation(MidGlobalQuat);

        Hierarchy->SetGlobalTransform(
            BoneMid,
            NewMidXf,
            /*bInitial=*/false,
            /*bPropagateToChildren=*/false
        );
    }

    // 17) Apply End manually based on L2
    {
        // compute new end position
        const FVector NewEndPos = NewP1 + DesiredDir2 * L2;

        FTransform NewEndXf = Hierarchy->GetGlobalTransform(BoneEnd);
        NewEndXf.SetLocation(NewEndPos);
        // (optionally set NewEndXf.SetRotation(...) if you need end orientation)

        Hierarchy->SetGlobalTransform(
            BoneEnd,
            NewEndXf,
            /*bInitial=*/false,
            /*bPropagateToChildren=*/false
        );
    }

    // 18) Debug drawing
    if (bDebug)
    {
        if (UWorld* World = Hierarchy->GetWorld())
        {
            // re‐use NewEndPos from above
            const FVector NewP2 = NewP1 + DesiredDir2 * L2;

            // trip-line: PoleTarget → P0 → NewP1 → NewP2
            DrawDebugLine(World, PoleVectorTarget, P0, FColor::Yellow, false, -1.f, 0, Thickness);
            DrawDebugLine(World, P0, NewP1, FColor::Yellow, false, -1.f, 0, Thickness);
            DrawDebugLine(World, NewP1, NewP2, FColor::Yellow, false, -1.f, 0, Thickness);

            // reach cubes
            const FVector MinPos = P0 + Dir * MinD;
            const FVector MaxPos = P0 + Dir * MaxD;
            const FVector Extents(DebugCubeSize);
            DrawDebugBox(World, MinPos, Extents, FColor::Red, false, -1.f, 0, 1.f);
            DrawDebugBox(World, MaxPos, Extents, FColor::Green, false, -1.f, 0, 1.f);
        }
    }
}