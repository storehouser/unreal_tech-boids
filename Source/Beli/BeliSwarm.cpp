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
	
	Boids.Reserve(MaxNumBoids);
	
	for (int32 i = 0; i < MaxNumBoids; ++i)
	{
		FBoidData& Boid = Boids[i];
		Boid.Location = FVector(FMath::RandRange(-100, 100), FMath::RandRange(-100, 100), FMath::RandRange(-100, 100));
		Boid.Velocity = FVector(FMath::RandRange(-100, 100), FMath::RandRange(-100, 100), FMath::RandRange(-100, 100));
		Boid.Rotation = Boid.Velocity.ToOrientationRotator();
		
		InstancedMeshComp->AddInstance(Boid.GetTransaform());
	}
}

void ABeliSwarm::Tick(float DeltaTime)
{
	Super::Tick(DeltaTime);
	
	check(IsValid(InstancedMeshComp));
	
	for (int32 i = 0; i < MaxNumBoids; ++i)
	{
		FBoidData& Boid = Boids[i];
		
		const FVector& Seperation = CalcSeperation(Boid);
		const FVector& Alignment = CalcAlignment(Boid);
		const FVector& Cohesion = CalcCohesion(Boid);
		
		Boid.Velocity += (Seperation + Alignment + Cohesion) * DeltaTime;
		Boid.Rotation = FMath::RInterpTo(Boid.Rotation, Boid.Velocity.ToOrientationRotator(), DeltaTime, 0.1f);
		Boid.Location += Boid.Velocity;
	}
}

const FVector& ABeliSwarm::CalcSeperation(const FBoidData& InBoidData) const
{
	return FVector::ZeroVector;
}

const FVector& ABeliSwarm::CalcAlignment(const FBoidData& InBoidData) const
{
	return FVector::ZeroVector;
}

const FVector& ABeliSwarm::CalcCohesion(const FBoidData& InBoidData) const
{
	return FVector::ZeroVector;
}

