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
		NewBoid.Location = FVector(FMath::RandRange(-100, 100), FMath::RandRange(-100, 100), FMath::RandRange(-100, 100));
		NewBoid.Velocity = FVector(FMath::RandRange(-100, 100), FMath::RandRange(-100, 100), FMath::RandRange(-100, 100));
		NewBoid.Rotation = NewBoid.Velocity.ToOrientationRotator();
		
		Boids.Emplace(NewBoid);
		
		InstancedMeshComp->AddInstance(NewBoid.GetTransaform());
	}
}

void ABeliSwarm::Tick(float DeltaTime)
{
	Super::Tick(DeltaTime);
	
	check(IsValid(InstancedMeshComp));
	
	TArray<FTransform> NewTransforms;
	NewTransforms.Reserve(MaxBoidsCount);
	for (int32 i = 0; i < MaxBoidsCount; ++i)
	{
		FBoidData& Boid = Boids[i];
		
		FVector Cohesion = CalcCohesion(Boid);
		FVector Seperation = CalcSeperation(Boid);
		FVector Alignment = CalcAlignment(Boid);
		FVector Place = CalcTendingToPlace(Boid);
		
		Boid.Velocity += (Cohesion + Seperation + Alignment + Place);
		Boid.Rotation = FMath::RInterpTo(Boid.Rotation, Boid.Velocity.ToOrientationRotator(), DeltaTime, 0.1f);
		Boid.Location += (Boid.Velocity * DeltaTime);
		
		NewTransforms.Emplace(Boid.GetTransaform());
	}
	
	InstancedMeshComp->BatchUpdateInstancesTransforms(0, NewTransforms, false, true, false);
}

FVector ABeliSwarm::CalcCohesion(const FBoidData& InBoidData) const
{
	FVector TotalOfBoidLocations;
	for (int32 i = 0; i < MaxBoidsCount; ++i)
	{
		const FBoidData& Boid = Boids[i];
		if (Boid.Index != InBoidData.Index)
		{
			TotalOfBoidLocations += Boid.Location;
		}
	}
	
	// TODO 중앙으로 이동하는 값 PROPERTY 처리
	return (TotalOfBoidLocations / (MaxBoidsCount - 1)) * 0.01f;
}

FVector ABeliSwarm::CalcSeperation(const FBoidData& InBoidData) const
{
	FVector TowardLocationToSperation;
	for (int32 i = 0; i < MaxBoidsCount; ++i)
	{
		const FBoidData& Boid = Boids[i];
		if (Boid.Index != InBoidData.Index)
		{
			// TODO 거리 값 PROPERTY 처리
			if (FVector::DistSquared(Boid.Location, InBoidData.Location) < 100.f * 100.f)
			{
				TowardLocationToSperation += (Boid.Location - InBoidData.Location);
			}
		}
	}
	
	return TowardLocationToSperation;
}

FVector ABeliSwarm::CalcAlignment(const FBoidData& InBoidData) const
{
	FVector TotalOfBoidVelocities;
	
	for (int32 i = 0; i < MaxBoidsCount; ++i)
	{
		const FBoidData& Boid = Boids[i];
		if (Boid.Index != InBoidData.Index)
		{
			TotalOfBoidVelocities += Boid.Velocity;
		}
	}
	
	// TODO 보정 값 PROPERTY 처리
	return (TotalOfBoidVelocities / (MaxBoidsCount - 1)) / 8.0f;
}

FVector ABeliSwarm::CalcTendingToPlace(const FBoidData& InBoidData) const
{
	FVector DestLocation = FVector::ZeroVector;
	
	return (DestLocation - InBoidData.Location) / 100.f;
}


