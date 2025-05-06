// Fill out your copyright notice in the Description page of Project Settings.


#include "CRFunctionLibs.h"
#include "Kismet/KismetMathLibrary.h"

FMakeVector_Execute()
{
	const float dotProductNormalized = UKismetMathLibrary::Dot_VectorVector(UKismetMathLibrary::Normal(VectorA), UKismetMathLibrary::Normal(VectorB));

	Result = UKismetMathLibrary::RadiansToDegrees(UKismetMathLibrary::Acos(dotProductNormalized));

}