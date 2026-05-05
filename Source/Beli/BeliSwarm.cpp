// Fill out your copyright notice in the Description page of Project Settings.


#include "BeliSwarm.h"

#include "Components/InstancedStaticMeshComponent.h"


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
		
		InstancedMeshComp->AddInstance(NewBoid.GetTransaform());
	}
	
	// Double-Buffer를 패턴을 사용하기 위해 똑같은 크기의 Boids 객체들을 준비. 안의 값은 중요치 않지만 같은 크기로 만들기 위해 그냥 복사로 처리.
	NextBoids = Boids;
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
		FVector Place = CalcTendingToPlace(Boid);
		
		FVector NewForce = (Cohesion + Separation + Alignment + Place).GetClampedToMaxSize(MaxForce);
		
		// 기본 메쉬가 하늘을 바라보고 있어 Rotation값에서 MakeFormZ 함수를 사용.
		NextBoid.Velocity = (Boid.Velocity + NewForce * DeltaTime).GetClampedToMaxSize(MaxSpeed);
		NextBoid.Rotation = FMath::RInterpTo(Boid.Rotation, FRotationMatrix::MakeFromZ(NextBoid.Velocity.GetSafeNormal()).Rotator(), DeltaTime, 30.f);
		NextBoid.Location = Boid.Location + (NextBoid.Velocity * DeltaTime);
		
		NewTransforms.Emplace(NextBoid.GetTransaform());
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


