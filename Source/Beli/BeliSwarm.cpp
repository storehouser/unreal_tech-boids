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
	
	CurrentBoids.Reserve(MaxBoidsCount);
	
	for (int32 i = 0; i < MaxBoidsCount; ++i)
	{
		FBoidData NewBoid;
		NewBoid.Index = i;
		NewBoid.Location = FVector(FMath::RandRange(-100, 100), FMath::RandRange(-100, 100), FMath::RandRange(-100, 100));
		NewBoid.Velocity = FVector(FMath::RandRange(-100, 100), FMath::RandRange(-100, 100), FMath::RandRange(-100, 100));
		NewBoid.Rotation = NewBoid.Velocity.ToOrientationRotator();
		
		CurrentBoids.Emplace(NewBoid);
		
		InstancedMeshComp->AddInstance(NewBoid.GetTransaform());
	}
	
	NextBoids = CurrentBoids;
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
		FBoidData& Boid = CurrentBoids[i];
		
		FVector Cohesion = CalcCohesion(Boid);
		FVector Seperation = CalcSeperation(Boid);
		FVector Alignment = CalcAlignment(Boid);
		FVector Place = CalcTendingToPlace(Boid);
		
		FVector NewForce = (Cohesion + Seperation + Alignment + Place).GetClampedToMaxSize(MaxForce);
		
		FBoidData& NextBoid = NextBoids[i];
		NextBoid.Velocity = (Boid.Velocity + NewForce * DeltaTime).GetClampedToMaxSize(MaxSpeed);
		NextBoid.Rotation = FMath::RInterpTo(Boid.Rotation, FRotationMatrix::MakeFromZ(NextBoid.Velocity.GetSafeNormal()).Rotator(), DeltaTime, 30.f);
		NextBoid.Location = Boid.Location + (NextBoid.Velocity * DeltaTime); 
		
		NewTransforms.Emplace(NextBoid.GetTransaform());
	}
	
	InstancedMeshComp->BatchUpdateInstancesTransforms(0, NewTransforms, false, true, false);
	
	Swap(CurrentBoids, NextBoids);
}

FVector ABeliSwarm::CalcCohesion(const FBoidData& InBoidData) const
{
	FVector TotalOfBoidLocations;
	for (int32 i = 0; i < MaxBoidsCount; ++i)
	{
		const FBoidData& Boid = CurrentBoids[i];
		if (Boid.Index != InBoidData.Index)
		{
			TotalOfBoidLocations += Boid.Location;
		}
	}
	
	return (TotalOfBoidLocations / (MaxBoidsCount - 1)) * CohesionWeight;
}

FVector ABeliSwarm::CalcSeperation(const FBoidData& InBoidData) const
{
	FVector TowardLocationToSeperation;
	for (int32 i = 0; i < MaxBoidsCount; ++i)
	{
		const FBoidData& Boid = CurrentBoids[i];
		if (Boid.Index != InBoidData.Index)
		{
			if (FVector::DistSquared(Boid.Location, InBoidData.Location) < 100.f * 100.f)
			{
				TowardLocationToSeperation += (Boid.Location - InBoidData.Location);
			}
		}
	}
	
	return TowardLocationToSeperation * SeperationWeight;
}

FVector ABeliSwarm::CalcAlignment(const FBoidData& InBoidData) const
{
	FVector TotalOfBoidVelocities;
	
	for (int32 i = 0; i < MaxBoidsCount; ++i)
	{
		const FBoidData& Boid = CurrentBoids[i];
		if (Boid.Index != InBoidData.Index)
		{
			TotalOfBoidVelocities += Boid.Velocity;
		}
	}
	
	return (TotalOfBoidVelocities / (MaxBoidsCount - 1)) * AlignmentWeight;
}

FVector ABeliSwarm::CalcTendingToPlace(const FBoidData& InBoidData) const
{
	FVector DestLocation = FVector::ZeroVector;
	
	return (DestLocation - InBoidData.Location) * TendingToPlaceWeight;
}


