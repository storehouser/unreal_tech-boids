// Fill out your copyright notice in the Description page of Project Settings.


#include "BeliSwarm.h"

#include "Components/InstancedStaticMeshComponent.h"
#include "Library/BoidsFunctionLibrary.h"


ABeliSwarm::ABeliSwarm()
{
	PrimaryActorTick.bCanEverTick = true;
	
	InstancedMeshComp = CreateDefaultSubobject<UInstancedStaticMeshComponent>(TEXT("InstancedMeshComp"));
	RootComponent = InstancedMeshComp;
	
	InstancedMeshComp->SetCollisionEnabled(ECollisionEnabled::Type::NoCollision);
}

void ABeliSwarm::BeginPlay()
{
	Super::BeginPlay();
	
	check(IsValid(InstancedMeshComp));
	
	Boids.Reserve(MaxBoidsCount);
	
	for (int32 i = 0; i < MaxBoidsCount; ++i)
	{
		FBoidData NewBoid;
		NewBoid.Index = i;
		
		const FVector BoidNormal = FMath::VRand();
		NewBoid.Location = FVector(FMath::RandRange(-500, 500), FMath::RandRange(-500, 500), FMath::RandRange(-500, 500));
		NewBoid.Velocity = BoidNormal * FMath::RandRange(0.f, MaxSpeed);
		NewBoid.Rotation = FRotationMatrix::MakeFromZ(BoidNormal).Rotator();
		
		Boids.Emplace(NewBoid);
		
		InstancedMeshComp->AddInstance(NewBoid.GetTransform());
	}
	
	// Double-Buffer를 패턴을 사용하기 위해 똑같은 크기의 Boids 객체들을 준비. 안의 값은 중요치 않지만 같은 크기로 만들기 위해 그냥 복사로 처리.
	NextBoids = Boids;
	
	// 충돌 검사를 위한 피보나치 레이를 캐싱 - 약간 뒤쪽까지 검사. 충돌이 걸리면 바로 뒤쪽 방향부터 찾아보기 위해 배열을 뒤집어 준다.
	// TODO Config값으로 빼자.
	FibonacciDirections = UBoidsFunctionLibrary::GetFibonacciDirections(20, 140.f);
	Algo::Reverse(FibonacciDirections);
}

void ABeliSwarm::Tick(float DeltaTime)
{
	Super::Tick(DeltaTime);
	
	check(IsValid(InstancedMeshComp));
	
	// TODO 해당 배열을 매 Tick 마다 생성하지 말고 미리 생성.
	// 혹은 Boid 배열 자체를 Transform으로 처리할 수 있게
	TArray<FTransform> NewTransforms;
	NewTransforms.Reserve(MaxBoidsCount);
	for (int32 i = 0; i < MaxBoidsCount; ++i)
	{
		FBoidData& Boid = Boids[i];
		FBoidData& NextBoid = NextBoids[i];
		
		FVector Cohesion = CalcCohesion(Boid);
		FVector Separation = CalcSeparation(Boid);
		FVector Alignment = CalcAlignment(Boid);
		FVector AvoidanceObstacle = CalcObstacleAvoidance(Boid);
		FVector Place = CalcTendingToPlace(Boid);
		
		FVector NewForce = (Cohesion + Separation + Alignment + AvoidanceObstacle + Place).GetClampedToMaxSize(MaxForce);
		
		// 기본 메쉬가 하늘을 바라보고 있어 Rotation값에서 MakeFormZ 함수를 사용.
		NextBoid.Velocity = (Boid.Velocity + NewForce * DeltaTime).GetClampedToMaxSize(MaxSpeed);
		NextBoid.Rotation = FMath::RInterpTo(Boid.Rotation, FRotationMatrix::MakeFromZ(NextBoid.Velocity.GetSafeNormal()).Rotator(), DeltaTime, 30.f);
		NextBoid.Location = Boid.Location + (NextBoid.Velocity * DeltaTime);
		
		NewTransforms.Emplace(NextBoid.GetTransform());
	}
	
	InstancedMeshComp->BatchUpdateInstancesTransforms(0, NewTransforms, false, true, false);
	
	Swap(Boids, NextBoids);
}

FVector ABeliSwarm::CalcCohesion(const FBoidData& InBoidData) const
{
	FVector CohesionForce = FVector::ZeroVector;
	FVector CenterOfMass = FVector::ZeroVector; // 이웃들의 위치를 다 더할 변수
	int32 NumNeighborhood = 0;
	const float CohesionDistSquared = FMath::Square(CohesionRadius);

	for (const FBoidData& OtherBoid : Boids)
	{
		if (OtherBoid.Index == InBoidData.Index)
		{
			continue;
		}
		
		const float DiffDistSquared = (InBoidData.Location - OtherBoid.Location).SquaredLength();
		if (DiffDistSquared < CohesionDistSquared)
		{
			CenterOfMass += OtherBoid.Location;
			++NumNeighborhood;
		}
	}
	
	if (NumNeighborhood > 0)
	{
		CenterOfMass /= NumNeighborhood;
		const FVector DesiredDirection = CenterOfMass - InBoidData.Location;
		
		if (!DesiredDirection.IsNearlyZero())
		{
			// 목적지에 가까워질수록 약한 힘을 준다.
			const float Distance = DesiredDirection.Length();
			const float SlowingScale = FMath::Max(Distance / CohesionSlowingRadius, 1.0f);
			
			// 조향력(Steering) 도출: 목표 속도 - 현재 속도
			const FVector DesiredVelocity = DesiredDirection.GetUnsafeNormal() * MaxSpeed * SlowingScale;
			CohesionForce = DesiredVelocity - InBoidData.Velocity;
		}
	}

	return CohesionForce * CohesionWeight;
}

FVector ABeliSwarm::CalcSeparation(const FBoidData& InBoidData) const
{
	FVector SeparationForce = FVector::ZeroVector;
	int32 NumNeighborhood = 0;
	const float SeparationRadiusSquared = FMath::Square(SeparationRadius);

	for (const FBoidData& OtherBoid : Boids)
	{
		if (OtherBoid.Index == InBoidData.Index)
		{
			continue;
		}
		
		FVector DiffVector = InBoidData.Location - OtherBoid.Location;
		
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
	
	return SeparationForce * SeparationWeight;
}

FVector ABeliSwarm::CalcAlignment(const FBoidData& InBoidData) const
{
	FVector AlignmentForce = FVector::ZeroVector;
	FVector AverageVelocity = FVector::ZeroVector; // 이웃들의 속도를 다 더할 변수
	int32 NumNeighborhood = 0;
	const float AlignmentDistSquared = FMath::Square(AlignmentRadius);

	for (const FBoidData& OtherBoid : Boids)
	{
		if (OtherBoid.Index == InBoidData.Index)
		{
			continue;
		}
		
		const float DiffDistSquared = (InBoidData.Location - OtherBoid.Location).SquaredLength();
		if (DiffDistSquared < AlignmentDistSquared)
		{
			AverageVelocity += OtherBoid.Velocity;
			++NumNeighborhood;
		}
	}
	
	if (NumNeighborhood > 0)
	{
		AverageVelocity /= NumNeighborhood;
    
		if (!AverageVelocity.IsNearlyZero())
		{
			// 조향력(Steering) 도출: 목표 속도 - 현재 속도
			const FVector DesiredVelocity = AverageVelocity.GetSafeNormal() * MaxSpeed;
			AlignmentForce = DesiredVelocity - InBoidData.Velocity;
		}
	}

	return AlignmentForce * AlignmentWeight;
}

FVector ABeliSwarm::CalcObstacleAvoidance(const FBoidData& Boid)
{
	const UWorld* World = GetWorld();
	check(IsValid(World));
	FVector AvoidanceForce = FVector::ZeroVector;

	// 1. 가장 기본인 '정면'으로 먼저 두꺼운 구(Sphere)를 던져봅니다.
	FHitResult HitResult;
	FCollisionShape SphereShape = FCollisionShape::MakeSphere(BoidRadius); // 새의 크기(여유 공간)만큼의 구형
    
	// 이동 방향 전방으로 AvoidDistance 만큼 검사
	const FTransform ManagerWorldTransform = InstancedMeshComp->GetComponentTransform();
	const FVector ForwardDir = ManagerWorldTransform.TransformVector(Boid.Velocity).GetSafeNormal();
	const FVector BoidWorldLocation = ManagerWorldTransform.TransformPosition(Boid.Location);
	const FVector EndPos = BoidWorldLocation + (ForwardDir * AvoidDistance);

	const bool bHitForward = World->SweepSingleByChannel(
		HitResult, 
		BoidWorldLocation, 
		EndPos, 
		FQuat::Identity, 
		ECC_WorldStatic,
		SphereShape
	);
	
	// DrawDebugLine(World, BoidWorldLocation, EndPos, bHitForward ? FColor::Red : FColor::Green, false, -1.0f, 0, 2.0f);

	// 2. 정면에 아무것도 없다면 굳이 피보나치 연산을 할 필요가 없습니다! (최고의 최적화)
	if (!bHitForward) 
	{
		return AvoidanceForce; 
	}

	// 3. 정면에 무언가 있다면, 비로소 피보나치 방향 배열을 순회하며 '구멍(빈 공간)'을 찾습니다.
	FVector EscapeDirection = FVector::ZeroVector;
	bool bFoundEscape = false;

	const FQuat ManagerWorldQuat = ManagerWorldTransform.GetRotation();
	const FRotator BoidWorldRotation = (ManagerWorldQuat * Boid.Rotation.Quaternion()).Rotator();
	
	// for (const FVector& RayDir : FibonacciDirections)
	// {
	// 	// 피보나치 방향은 Boid의 로컬 좌표계 기준이므로, 월드 방향으로 변환.
	// 	FVector WorldRayDir = BoidWorldRotation.RotateVector(RayDir);
	// 	FVector RayEndPos = BoidWorldLocation + (WorldRayDir * AvoidDistance);
	// 	
	// 	DrawDebugLine(World, BoidWorldLocation, RayEndPos, FColor::Purple, false, -1.0f, 0, 2.0f);
	// }
	
	for (const FVector& RayDir : FibonacciDirections)
	{
		// 피보나치 방향은 Boid의 로컬 좌표계 기준이므로, 월드 방향으로 변환.
		FVector WorldRayDir = BoidWorldRotation.RotateVector(RayDir);
		FVector RayEndPos = BoidWorldLocation + (WorldRayDir * AvoidDistance);

		const bool bHit = World->SweepSingleByChannel(
			HitResult, 
			BoidWorldLocation, 
			RayEndPos, 
			FQuat::Identity, 
			ECC_WorldStatic, 
			SphereShape
		);
		
		DrawDebugLine(World, BoidWorldLocation, RayEndPos, bHit ? FColor::Red : FColor::Green, false, -1.0f, 0, 2.0f);

		// 부딪히지 않은 최초의 안전한 방향을 찾으면 탐색 종료! (가장 정면과 가까운 안전한 길)
		if (!bHit)
		{
			EscapeDirection = WorldRayDir;
			bFoundEscape = true;
			break; 
		}
	}

	// 4. 안전한 방향으로 조향력 발생
	if (bFoundEscape)
	{
		const FVector LocalDirection = ManagerWorldTransform.InverseTransformVector(EscapeDirection);
		const FVector DesiredVelocity = LocalDirection * MaxSpeed;
		AvoidanceForce = (DesiredVelocity - Boid.Velocity);
	}
	else
	{
		// 모든 방향이 막혀있다면 (막다른 골목)? -> 브레이크를 밟거나 뒤로 돌기!
		AvoidanceForce = -Boid.Velocity; 
	}

	return AvoidanceForce * ObstacleAvoidanceWeight;
}

FVector ABeliSwarm::CalcTendingToPlace(const FBoidData& InBoidData) const
{
	FVector DestLocation = FVector::ZeroVector;	// TEMP @Auggie Component Space 중심으로 향하도록 ZeroVector 사용
	FVector PlaceForce = FVector::ZeroVector;
	
	const FVector Distance = DestLocation - InBoidData.Location;
	if (!Distance.IsNearlyZero())
	{
		// 조향력(Steering) 도출: 목표 속도 - 현재 속도
		const FVector DesiredVelocity = Distance.GetUnsafeNormal() * MaxSpeed;
		PlaceForce = DesiredVelocity - InBoidData.Velocity;
	}
	
 	return PlaceForce * TendingToPlaceWeight;
}


