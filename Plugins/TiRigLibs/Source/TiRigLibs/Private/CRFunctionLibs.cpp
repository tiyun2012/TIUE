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