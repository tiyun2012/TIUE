// Fill out your copyright notice in the Description page of Project Settings.

#pragma once

#include "CoreMinimal.h"
#include "ControlRig.h"
#include "CR_TiRig.generated.h"

/**
 * 
 */
UCLASS(Blueprintable, BlueprintType)
class TIRIGLIBS_API UCR_TiRig : public UControlRig
{
	GENERATED_BODY()

	UCR_TiRig()
	{
		// Add default variables
		
		// Initialize the variable
	}
protected:
    virtual void Initialize(bool bRequestInit) override
    {
        Super::Initialize(bRequestInit);
    }

    virtual bool Execute(const FName& InEventName) override
    {
        Super::Execute(InEventName);
        return true;
    }

};
