// Fill out your copyright notice in the Description page of Project Settings.


#include "BeliFunctionLibrary.h"


TArray<FVector> UBeliFunctionLibrary::GetFibonacciDirections(int32 NumRays, float MaxAngleDegrees)
{
	if (!ensure(NumRays > 0))
	{
		return TArray<FVector>();
	}
	
	TArray<FVector> Directions;
	Directions.Reserve(NumRays);
	
	// 황금각 (약 137.5도)
	const float GoldenAngle = PI * (3.0f - FMath::Sqrt(5.0f));
	
	// MaxAngleDegrees:90 -> MinX:0, MaxAngleDegrees:120 -> MinX:-0.5(살짝 뒤)
	const float MinX = FMath::Cos(FMath::DegreesToRadians(MaxAngleDegrees));
	const float RangeX = 1.0f - MinX;
	
	for (int32 i = 0; i < NumRays; ++i)
	{
		// t: (0.0, 1.0]
		const float t = StaticCast<float>(i + 1) / StaticCast<float>(NumRays);
		const float X = 1.0f - (t * RangeX);
		const float Radius = FMath::Sqrt(1.0f - (X * X));
		
		const float Theta = GoldenAngle * i;
		
		const float Y = Radius * FMath::Cos(Theta);
		const float Z = Radius * FMath::Sin(Theta);
		
		Directions.Add(FVector(X, Y, Z).GetSafeNormal());
	}
	
	return Directions;
}

FColor UBeliFunctionLibrary::GetColorFromHash(int32 HashKey, int32 Alpha)
{
	// 해시 키를 이용하여 황금값을 위한 Hue 값 게산
	const float Hue = FMath::Fmod(HashKey * 137.508f, 360.0f);
	FLinearColor HashLinearColor(Hue, 0.9f, 0.9f, 1.0f);
	
	return HashLinearColor.HSVToLinearRGB().ToFColor(true).WithAlpha(Alpha);
}
