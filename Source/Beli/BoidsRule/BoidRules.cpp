// Fill out your copyright notice in the Description page of Project Settings.


#include "BoidRules.h"

#include "Library/BoidsFunctionLibrary.h"


FVector UBoidRuleBase::CalculateForce(const FBoidData& Boid, const TArray<FBoidData>& Neighbors, const FBoidSceneContext& BoidSceneContext) const
{
	return CalculateForce_Internal(Boid, Neighbors, BoidSceneContext) * Weight;
}


UBoidRule_Cohesion::UBoidRule_Cohesion()
{
	Weight = 0.1f;
}

FVector UBoidRule_Cohesion::CalculateForce_Internal(const FBoidData& Boid, const TArray<FBoidData>& Neighbors, const FBoidSceneContext& BoidSceneContext) const
{
	FVector CohesionForce = FVector::ZeroVector;
	FVector CenterOfMass = FVector::ZeroVector; // 이웃들의 위치를 다 더할 변수
	int32 NumNeighborhood = 0;
	const float CohesionDistSquared = FMath::Square(CohesionRadius);

	for (const FBoidData& NeighborBoid : Neighbors)
	{
		if (NeighborBoid.Index == Boid.Index)
		{
			continue;
		}
		
		const float DiffDistSquared = (Boid.Location - NeighborBoid.Location).SquaredLength();
		if (DiffDistSquared < CohesionDistSquared)
		{
			CenterOfMass += NeighborBoid.Location;
			++NumNeighborhood;
		}
	}
	
	if (NumNeighborhood > 0)
	{
		CenterOfMass /= NumNeighborhood;
		const FVector DesiredDirection = CenterOfMass - Boid.Location;
		
		if (!DesiredDirection.IsNearlyZero())
		{
			// 목적지에 가까워질수록 약한 힘을 준다.
			const float Distance = DesiredDirection.Length();
			const float SlowingScale = FMath::Max(Distance / SlowingRadius, 1.0f);
			
			// 조향력(Steering) 도출: 목표 속도 - 현재 속도
			const FVector DesiredVelocity = DesiredDirection.GetUnsafeNormal() * BoidSceneContext.BoidMaxSpeed * SlowingScale;
			CohesionForce = DesiredVelocity - Boid.Velocity;
		}
	}

	return CohesionForce;
}


UBoidRule_Separation::UBoidRule_Separation()
{
	Weight = 800.f;
}

FVector UBoidRule_Separation::CalculateForce_Internal(const FBoidData& Boid, const TArray<FBoidData>& Neighbors, const FBoidSceneContext& BoidSceneContext) const
{
	FVector SeparationForce = FVector::ZeroVector;
	int32 NumNeighborhood = 0;
	const float SeparationRadiusSquared = FMath::Square(SeparationRadius);

	for (const FBoidData& NeighborBoid : Neighbors)
	{
		if (NeighborBoid.Index == Boid.Index)
		{
			continue;
		}
		
		FVector DiffVector = Boid.Location - NeighborBoid.Location;
		
		// 만약 다른 boid가 거의 겹쳐 있다면 기존 속도 방향에서 매우 작은 크기로의 속도 값을 구한다.
		if (DiffVector.IsNearlyZero())
		{
			DiffVector = FMath::VRand() * 0.01f;
		}
		
		if (DiffVector.SizeSquared() < SeparationRadiusSquared)
		{
			// Diff 길이를 SeparationRadius를 나눠 정규화 처리한다. (거리에 반비례하여 Force가 커짐)
			// DiffVector.GetUnsafeNormal() / ((DiffVector.Length() / SeparationRadius)
			// == (DiffVector / DiffVector.Length()) / ((DiffVector.Length() / SeparationRadius)
			SeparationForce += DiffVector * (SeparationRadius / DiffVector.SizeSquared());
			++NumNeighborhood;
		}
	}
	
	// 이웃 갯수만큼 평균 값을 계산
	if (NumNeighborhood > 0)
	{
		SeparationForce /= NumNeighborhood;
	}
	
	return SeparationForce;
}


UBoidRule_Alignment::UBoidRule_Alignment()
{
	Weight = 0.2f;
}

FVector UBoidRule_Alignment::CalculateForce_Internal(const FBoidData& Boid, const TArray<FBoidData>& Neighbors, const FBoidSceneContext& BoidSceneContext) const
{
	FVector AlignmentForce = FVector::ZeroVector;
	FVector AverageVelocity = FVector::ZeroVector; // 이웃들의 속도를 다 더할 변수
	int32 NumNeighborhood = 0;
	const float AlignmentDistSquared = FMath::Square(AlignmentRadius);

	for (const FBoidData& NeighborBoid : Neighbors)
	{
		if (NeighborBoid.Index == Boid.Index)
		{
			continue;
		}
		
		const float DiffDistSquared = (Boid.Location - NeighborBoid.Location).SquaredLength();
		if (DiffDistSquared < AlignmentDistSquared)
		{
			AverageVelocity += NeighborBoid.Velocity;
			++NumNeighborhood;
		}
	}
	
	if (NumNeighborhood > 0)
	{
		AverageVelocity /= NumNeighborhood;
    
		if (!AverageVelocity.IsNearlyZero())
		{
			// 조향력(Steering) 도출: 목표 속도 - 현재 속도
			const FVector DesiredVelocity = AverageVelocity.GetSafeNormal() * BoidSceneContext.BoidMaxSpeed;
			AlignmentForce = DesiredVelocity - Boid.Velocity;
		}
	}

	return AlignmentForce;
}


UBoidRule_TendingToPlace::UBoidRule_TendingToPlace()
{
	Weight = 0.2f;
}

FVector UBoidRule_TendingToPlace::CalculateForce_Internal(const FBoidData& Boid, const TArray<FBoidData>& Neighbors, const FBoidSceneContext& BoidSceneContext) const
{
	FVector PlaceForce = FVector::ZeroVector;
	
	const FVector Distance = BoidSceneContext.ManagerTransform.TransformPosition(PlaceLocation) - Boid.Location;
	if (!Distance.IsNearlyZero())
	{
		// 조향력(Steering) 도출: 목표 속도 - 현재 속도
		const FVector DesiredVelocity = Distance.GetUnsafeNormal() * BoidSceneContext.BoidMaxSpeed;
		PlaceForce = DesiredVelocity - Boid.Velocity;
	}
	
	return PlaceForce;
}

UBoidRule_AvoidanceObstacle::UBoidRule_AvoidanceObstacle()
{
	Weight = 3500.f;
}

void UBoidRule_AvoidanceObstacle::Initialize()
{
	Super::Initialize();
	
	// 충돌 검사를 위한 피보나치 레이를 캐싱 - 약간 뒤쪽까지 검사. 충돌이 걸리면 바로 뒤쪽 방향부터 찾아보기 위해 배열을 뒤집어 준다.
	FibonacciDirections = UBoidsFunctionLibrary::GetFibonacciDirections(NumRays, MaxRayDegree);
	Algo::Reverse(FibonacciDirections);
}

FVector UBoidRule_AvoidanceObstacle::CalculateForce_Internal(const FBoidData& Boid, const TArray<FBoidData>& Neighbors, const FBoidSceneContext& BoidSceneContext) const
{
	const UWorld* World = GetWorld();
	check(IsValid(World));
	FVector AvoidanceForce = FVector::ZeroVector;

	// 1. 가장 기본인 '정면'으로 먼저 두꺼운 구(Sphere)를 던져봅니다.
	FHitResult HitResult;
	FCollisionShape SphereShape = FCollisionShape::MakeSphere(BoidRadius); // 새의 크기(여유 공간)만큼의 구형
    
	// 이동 방향 전방으로 AvoidDistance 만큼 검사
	const FVector ForwardDir = Boid.Velocity.GetSafeNormal();
	const FVector EndPos = Boid.Location + (ForwardDir * AvoidDistance);

	const bool bHitForward = World->SweepSingleByChannel(
		HitResult, 
		Boid.Location, 
		EndPos, 
		FQuat::Identity, 
		ECC_WorldStatic,
		SphereShape
	);
	
	// DrawDebugLine(World, Boid.Location, EndPos, bHitForward ? FColor::Red : FColor::Green, false, -1.0f, 0, 2.0f);

	// 2. 정면에 아무것도 없다면 굳이 피보나치 연산을 할 필요가 없습니다!
	if (!bHitForward) 
	{
		return AvoidanceForce; 
	}

	// 3. 정면에 무언가 있다면, 비로소 피보나치 방향 배열을 순회하며 '구멍(빈 공간)'을 찾습니다.
	FVector EscapeDirection = FVector::ZeroVector;
	bool bFoundEscape = false;

	// const FQuat ManagerWorldQuat = ManagerWorldTransform.GetRotation();
	// const FRotator BoidWorldRotation = (ManagerWorldQuat * Boid.Rotation.Quaternion()).Rotator();
	
	for (const FVector& RayDir : FibonacciDirections)
	{
		// 피보나치 방향을 Boid의 공간으로 변환
		FVector FibonacciRayDir = Boid.Rotation.RotateVector(RayDir);
		FVector RayEndPos = Boid.Location + (FibonacciRayDir * AvoidDistance);

		const bool bHit = World->SweepSingleByChannel(
			HitResult, 
			Boid.Location, 
			RayEndPos, 
			FQuat::Identity, 
			ECC_WorldStatic, 
			SphereShape
		);
		
		// DrawDebugLine(World, Boid.Location, RayEndPos, bHit ? FColor::Red : FColor::Green, false, -1.0f, 0, 2.0f);

		// 부딪히지 않은 최초의 안전한 방향을 찾으면 탐색 종료! (가장 정면과 가까운 안전한 길)
		if (!bHit)
		{
			EscapeDirection = FibonacciRayDir;
			bFoundEscape = true;
			break; 
		}
	}

	// 4. 안전한 방향으로 조향력 발생
	if (bFoundEscape)
	{
		const FVector DesiredVelocity = EscapeDirection * BoidSceneContext.BoidMaxSpeed;
		AvoidanceForce = (DesiredVelocity - Boid.Velocity);
	}
	else
	{
		// 모든 방향이 막혀있다면 (막다른 골목)? -> 브레이크를 밟거나 뒤로 돌기!
		AvoidanceForce = -Boid.Velocity; 
	}

	return AvoidanceForce;
}
