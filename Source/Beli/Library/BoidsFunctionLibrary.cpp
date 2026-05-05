// Fill out your copyright notice in the Description page of Project Settings.


#include "BoidsFunctionLibrary.h"


TArray<FVector> UBoidsFunctionLibrary::GetFibonacciDirections(int32 NumRays, float MaxAngleDegrees)
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
		// t: [0.0, 1.0)
		const float t = StaticCast<float>(i) / StaticCast<float>(NumRays);
		const float X = 1.0f - (t * RangeX);
		const float Radius = FMath::Sqrt(1.0f - (X * X));
		
		const float Theta = GoldenAngle * i;
		
		const float Y = Radius * FMath::Cos(Theta);
		const float Z = Radius * FMath::Sin(Theta);
		
		Directions.Add(FVector(X, Y, Z).GetSafeNormal());
	}
	
	return Directions;
}
