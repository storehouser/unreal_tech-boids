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
	
	for (int32 i = 0; i < MaxNumBoids; ++i)
	{
		FTransform Dummy = FTransform::Identity;
		Dummy.SetTranslation(FVector(FMath::RandRange(-100, 100), FMath::RandRange(-100, 100), FMath::RandRange(-100, 100)));
		
		InstancedMeshComp->AddInstance(Dummy);
	}
}

void ABeliSwarm::Tick(float DeltaTime)
{
	Super::Tick(DeltaTime);
}

