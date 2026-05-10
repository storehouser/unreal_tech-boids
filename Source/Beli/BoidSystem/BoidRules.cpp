// Fill out your copyright notice in the Description page of Project Settings.


#include "BoidRules.h"

#include "Library/BeliFunctionLibrary.h"


bool UBoidRuleBase::EvaluateBoid(OUT FBoidRuleResult& OutResult, const FBoidBuffer& BoidBuffer, int32 MyIndex, TArrayView<int32> NeighborIndices, const FBoidSceneContext& BoidSceneContext) const
{
	if (EvaluateBoid_Internal(OutResult, BoidBuffer, MyIndex, NeighborIndices, BoidSceneContext))
	{
		OutResult.Force *= ForceWeight;
		return true;
	}
	
	return false;
}


UBoidRule_Cohesion::UBoidRule_Cohesion()
{
	ForceWeight = 0.1f;
	AccumulationMode = EBoidAccumulationMode::General;
}

bool UBoidRule_Cohesion::EvaluateBoid_Internal(OUT FBoidRuleResult& OutResult, const FBoidBuffer& BoidBuffer, int32 MyIndex, TArrayView<int32> NeighborIndices, const FBoidSceneContext& BoidSceneContext) const
{
	FVector3f CenterOfMass = FVector3f::ZeroVector; // 이웃들의 위치를 다 더할 변수
	int32 NumNeighborhood = 0;
	const float CohesionDistSquared = FMath::Square(CohesionRadius);
	
	const FVector3f& MyBoidLocation = BoidBuffer.Locations[MyIndex];
	for (int32 NeighborIndex : NeighborIndices)
	{
		const FVector3f& NeighborLocation = BoidBuffer.Locations[NeighborIndex];
		
		const float DiffDistSquared = FVector3f::DistSquared(MyBoidLocation, NeighborLocation);
		if (DiffDistSquared < CohesionDistSquared)
		{
			CenterOfMass += NeighborLocation;
			++NumNeighborhood;
		}
	}
	
	const bool bHasNeighborhood = NumNeighborhood > 0;
	if (bHasNeighborhood)
	{
		CenterOfMass /= NumNeighborhood;
		const FVector3f DesiredDirection = CenterOfMass - BoidBuffer.Locations[MyIndex];
		
		if (!DesiredDirection.IsNearlyZero())
		{
			// 목적지에 가까워질수록 약한 힘을 준다.
			const float Distance = DesiredDirection.Length();
			const float SlowingScale = FMath::Max(Distance / SlowingRadius, 1.0f);
			
			// 조향력(Steering) 도출: 목표 속도 - 현재 속도
			const FVector3f DesiredVelocity = DesiredDirection.GetUnsafeNormal() * BoidSceneContext.BoidMaxSpeed * SlowingScale;
			OutResult.Force = DesiredVelocity - BoidBuffer.Velocities[MyIndex];
		}
	}

	return bHasNeighborhood;
}


UBoidRule_Separation::UBoidRule_Separation()
{
	ForceWeight = 800.f;
	AccumulationMode = EBoidAccumulationMode::Absolute;
}

bool UBoidRule_Separation::EvaluateBoid_Internal(OUT FBoidRuleResult& OutResult, const FBoidBuffer& BoidBuffer, int32 MyIndex, TArrayView<int32> NeighborIndices, const FBoidSceneContext& BoidSceneContext) const
{
	FVector3f SeparationForce = FVector3f::ZeroVector;
	int32 NumNeighborhood = 0;
	const float SeparationRadiusSquared = FMath::Square(SeparationRadius);
	
	const FVector3f& MyBoidLocation = BoidBuffer.Locations[MyIndex];
	for (int32 NeighborIndex : NeighborIndices)
	{
		FVector3f DiffVector = MyBoidLocation - BoidBuffer.Locations[NeighborIndex];
		
		// 만약 다른 boid가 거의 겹쳐 있다면 랜덤한 방향의 작은 크기로의 속도 값을 구한다.
		if (DiffVector.IsNearlyZero())
		{
			DiffVector = FVector3f(FMath::VRand());
		}
		
		if (DiffVector.SizeSquared() < SeparationRadiusSquared)
		{
			// Diff 길이를 SeparationRadius를 나눠 정규화 처리한다. (거리에 반비례하여 Force가 커짐)
			// DiffVector.GetUnsafeNormal() / ((DiffVector.Length() / SeparationRadius)
			// == (DiffVector / DiffVector.Length()) / ((DiffVector.Length() / SeparationRadius)
			SeparationForce += DiffVector * (SeparationRadius / FMath::Max(DiffVector.SizeSquared(), 1.0f));
			++NumNeighborhood;
		}
	}
	
	// 이웃 갯수만큼 평균 값을 계산
	const bool bHasNeighborhood = NumNeighborhood > 0; 
	if (NumNeighborhood > 0)
	{
		OutResult.Force = SeparationForce /= NumNeighborhood;
	}
	
	return bHasNeighborhood;
}


UBoidRule_Alignment::UBoidRule_Alignment()
{
	ForceWeight = 0.2f;
	AccumulationMode = EBoidAccumulationMode::General;
}

bool UBoidRule_Alignment::EvaluateBoid_Internal(OUT FBoidRuleResult& OutResult, const FBoidBuffer& BoidBuffer, int32 MyIndex, TArrayView<int32> NeighborIndices, const FBoidSceneContext& BoidSceneContext) const
{
	FVector3f AverageVelocity = FVector3f::ZeroVector; // 이웃들의 속도를 다 더할 변수
	int32 NumNeighborhood = 0;
	const float AlignmentDistSquared = FMath::Square(AlignmentRadius);
	
	const FVector3f& MyBoidLocation = BoidBuffer.Locations[MyIndex];
	for (int32 NeighborIndex : NeighborIndices)
	{
		const float DiffDistSquared = FVector3f::DistSquared(MyBoidLocation, BoidBuffer.Locations[NeighborIndex]);
		if (DiffDistSquared < AlignmentDistSquared)
		{
			AverageVelocity += BoidBuffer.Velocities[NeighborIndex];
			++NumNeighborhood;
		}
	}
	
	const bool bHasNeighborhood = NumNeighborhood > 0; 
	if (bHasNeighborhood)
	{
		AverageVelocity /= NumNeighborhood;
    
		if (!AverageVelocity.IsNearlyZero())
		{
			// 조향력(Steering) 도출: 목표 속도 - 현재 속도
			const FVector3f DesiredVelocity = AverageVelocity.GetSafeNormal() * BoidSceneContext.BoidMaxSpeed;
			OutResult.Force = DesiredVelocity - BoidBuffer.Velocities[MyIndex];
		}
	}

	return bHasNeighborhood;
}


UBoidRule_TendingToPlace::UBoidRule_TendingToPlace()
{
	ForceWeight = 0.2f;
	AccumulationMode = EBoidAccumulationMode::General;
}

bool UBoidRule_TendingToPlace::EvaluateBoid_Internal(OUT FBoidRuleResult& OutResult, const FBoidBuffer& BoidBuffer, int32 MyIndex, TArrayView<int32> NeighborIndices, const FBoidSceneContext& BoidSceneContext) const
{
	const FVector3f Distance = BoidSceneContext.SimulationSpace.TransformPosition(PlaceLocation) - BoidBuffer.Locations[MyIndex];
	if (!Distance.IsNearlyZero())
	{
		// 조향력(Steering) 도출: 목표 속도 - 현재 속도
		const FVector3f DesiredVelocity = Distance.GetUnsafeNormal() * BoidSceneContext.BoidMaxSpeed;
		OutResult.Force = DesiredVelocity - BoidBuffer.Velocities[MyIndex];
	}
	
	return true;
}


UBoidRule_AvoidanceObstacle::UBoidRule_AvoidanceObstacle()
{
	ForceWeight = 3500.f;
	AccumulationMode = EBoidAccumulationMode::Exclusive;
}

void UBoidRule_AvoidanceObstacle::Initialize()
{
	Super::Initialize();
	
	// 충돌 검사를 위한 피보나치 레이를 캐싱
	FibonacciDirections = UBeliFunctionLibrary::GetFibonacciDirections(NumRays, MaxRayDegree);
	SphereShape = FCollisionShape::MakeSphere(BoidRadius); // 크기(여유 공간)만큼의 구형
}

bool UBoidRule_AvoidanceObstacle::EvaluateBoid_Internal(OUT FBoidRuleResult& OutResult, const FBoidBuffer& BoidBuffer, int32 MyIndex, TArrayView<int32> NeighborIndices, const FBoidSceneContext& BoidSceneContext) const
{
	const UWorld* World = GetWorld();
	check(IsValid(World));
	
	// 0. 선두를 찾는다. !! TODO @Auggie 추가적인 최적화 필요
#if 1
	const FVector3f& MyBoidLocation = BoidBuffer.Locations[MyIndex];
	const FVector3f& MyBoidVelocity = BoidBuffer.Velocities[MyIndex];
	const FVector3f& MyBoidDirection = MyBoidVelocity.GetSafeNormal();
	bool bIsLeader = true;
	for (int32 NeighborIndex : NeighborIndices)
	{
		FVector3f ToNeighbor = BoidBuffer.Locations[NeighborIndex] - MyBoidLocation;
		
		const float DistSquared = ToNeighbor.SizeSquared();
		if (FMath::IsNearlyZero(DistSquared) || DistSquared > FMath::Square(AvoidDistance * 0.5f))
		{
			// 충돌 영역 절반 거리안에 해당 Boid 자체가 없거나 너무 가까우면 Leader로 간주할 수 있다.
			continue;
		}
		
		// 내 속도의 방향과 이웃으로 향하는 방향이 거의 같으면 
		const FVector3f ToNeighborDirection = ToNeighbor.GetSafeNormal();
		if (FVector3f::DotProduct(MyBoidDirection, ToNeighborDirection) > 0.9f)
        {
            const FVector3f& NeighborVel = BoidBuffer.Velocities[NeighborIndex];
            const FVector3f NeighborDir = NeighborVel.GetSafeNormal();
			
            const float AlignmentDot = FVector3f::DotProduct(MyBoidDirection, NeighborDir);
            
            // 0.7f 이상이면 대략 45도 이내로 꽤 비슷하게 날아가고 있다는 뜻
            if (AlignmentDot > 0.7f) 
            {
                bIsLeader = false;
                break;
            }
        }
	}

	// 내가 리더가 아니면 충돌 검사를 하지 않고 즉시 종료
	if (!bIsLeader)
	{
		return false;
	}
#endif

	// 1. 가장 기본인 '정면'으로 먼저 두꺼운 구(Sphere)를 던져봅니다.
	FHitResult HitResult;
	
	const FVector MyBoidLocationD = FVector(BoidBuffer.Locations[MyIndex]);
	const FVector MyBoidVelocityD = FVector(BoidBuffer.Velocities[MyIndex]);
	const FRotator MyBoidRotationD = FRotator(BoidBuffer.Rotations[MyIndex]);
    
	// 이동 방향 전방으로 AvoidDistance 만큼 검사
	const FVector ForwardDir = MyBoidVelocityD.GetSafeNormal();
	const FVector StartPos = MyBoidLocationD + (ForwardDir * SphereShape.Sphere.Radius); 
	const FVector EndPos = MyBoidLocationD + (ForwardDir * AvoidDistance);

	const bool bHitForward = World->SweepSingleByChannel(
		HitResult, 
		StartPos, 
		EndPos, 
		FQuat::Identity, 
		ECC_WorldStatic,
		SphereShape
	);
	
#if !UE_BUILD_SHIPPING
	if (BoidSceneContext.DebugParam > 0)
	{
		DrawDebugLine(World, MyBoidLocationD, EndPos, FColor::Magenta, false, -1.0f, 0, 2.0f);
	}
#endif

	// 2. 정면에 아무것도 없다면 굳이 피보나치 방향으로 추가적인 충돌 검사가 불필요.
	if (!bHitForward) 
	{
		return false; 
	}

	// 3. 정면에 무언가 있다면, 비로소 피보나치 방향 배열을 순회하며 '구멍(빈 공간)'을 찾습니다.
	FVector3f EscapeDirection = FVector3f::ZeroVector;
	bool bFoundEscape = false;
	
	for (const FVector& RayDir : FibonacciDirections)
	{
		// 피보나치 방향을 Boid의 공간으로 변환
		const FVector FibonacciRayDir = MyBoidRotationD.RotateVector(RayDir);
		const FVector RayStartPos = MyBoidLocationD + (FibonacciRayDir * SphereShape.Sphere.Radius);
		const FVector RayEndPos = MyBoidLocationD + (FibonacciRayDir * AvoidDistance);

		const bool bHit = World->SweepSingleByChannel(
			HitResult, 
			RayStartPos, 
			RayEndPos, 
			FQuat::Identity, 
			ECC_WorldStatic, 
			SphereShape
		);
		
#if !UE_BUILD_SHIPPING
		if (BoidSceneContext.DebugParam > 0)
		{
			DrawDebugLine(World, RayStartPos, RayEndPos, bHit ? FColor::Red : FColor::Green, false, -1.0f, 0, 3.0f);
		}
#endif

		// 부딪히지 않은 최초의 안전한 방향을 찾으면 탐색 종료!
		if (!bHit)
		{
			EscapeDirection = FVector3f(FibonacciRayDir);
			bFoundEscape = true;
			break;
		}
	}
	
	OutResult.ExclusiveLevel = 30.0f;		// 다음 틱에도 중첩될 수 있기 때문에 조금 짧게 잡아야 한다.

	// 4. 안전한 방향으로 조향력 발생
	if (bFoundEscape)
	{
		const FVector3f DesiredVelocity = EscapeDirection * BoidSceneContext.BoidMaxSpeed;
		OutResult.Force = (DesiredVelocity - BoidBuffer.Velocities[MyIndex]);
	}
	else
	{
		// 모든 방향이 막혀있다면 (막다른 골목)? -> 브레이크를 밟거나 뒤로 돌기!
		OutResult.Force = -BoidBuffer.Velocities[MyIndex]; 
	}

	return true;
}
