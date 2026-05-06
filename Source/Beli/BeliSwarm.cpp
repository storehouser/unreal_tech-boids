// Fill out your copyright notice in the Description page of Project Settings.


#include "BeliSwarm.h"

#include "Components/InstancedStaticMeshComponent.h"

#include "Beli.h"
#include "Library/SpatialGridHashHelper.h"



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
	
	BoidSystem.Initialize(InstancedMeshComp->GetComponentTransform());
	
	const TArray<FTransform>& BoidTransforms = BoidSystem.GetBoidTransforms();
	InstancedMeshComp->AddInstances(BoidTransforms, false, false);
}

void ABeliSwarm::Tick(float DeltaTime)
{
	Super::Tick(DeltaTime);
	
	check(IsValid(InstancedMeshComp));

	// 1. 지나간 시간만큼 Boid 객체들의 위치 값을 갱신하고,
	BoidSystem.UpdateBoids_Concurrent(DeltaTime, InstancedMeshComp->GetComponentTransform());
	
	// 2. 갱신된 위치, 회전 등을 얻어와 ISMC에 적용
	const TArray<FTransform>& BoidTransforms = BoidSystem.GetBoidTransforms();
	InstancedMeshComp->BatchUpdateInstancesTransforms(0, BoidTransforms, true, true, false);
	
	// 3. System 내부에 있는 Boid 객체들의 배열 정보를 Swap 처리
	BoidSystem.SwapBuffers();
}

