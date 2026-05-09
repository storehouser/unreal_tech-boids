// Fill out your copyright notice in the Description page of Project Settings.

#pragma once

#include "CoreMinimal.h"
#include "Kismet/BlueprintFunctionLibrary.h"

#include "BeliFunctionLibrary.generated.h"

/**
 * 
 */
UCLASS()
class BELI_API UBeliFunctionLibrary : public UBlueprintFunctionLibrary
{
	GENERATED_BODY()
	
public:
	/** X축을 기준으로 황금비를 이용한 피보나치 방향을 구한다. X축으로 부터 정렬이 되어 있다. */
	TArray<FVector> static GetFibonacciDirections(int32 NumRays, float MaxAngleDegrees);
	
	/** 해쉬 값과 황금각을 이용해서 적당한 색상 값을 구한다 */
	FColor static GetColorFromHash(int32 HashKey, int32 Alpha = 255);
};
