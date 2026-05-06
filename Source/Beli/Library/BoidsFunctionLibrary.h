// Fill out your copyright notice in the Description page of Project Settings.

#pragma once

#include "CoreMinimal.h"
#include "Kismet/BlueprintFunctionLibrary.h"

#include "BoidsFunctionLibrary.generated.h"

/**
 * 
 */
UCLASS()
class BELI_API UBoidsFunctionLibrary : public UBlueprintFunctionLibrary
{
	GENERATED_BODY()
	
public:
	/** X축을 기준으로 황금비를 이용한 피보나치 방향을 구한다. X축으로 부터 정렬이 되어 있다. */
	TArray<FVector> static GetFibonacciDirections(int32 NumRays, float MaxAngleDegrees);
};
