// Fill out your copyright notice in the Description page of Project Settings.


#include "CRFunctionLibs.h"
#include "Kismet/KismetMathLibrary.h"

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
