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
	
	BoidSystem.Initialize(GetWorld(), InstancedMeshComp->GetComponentTransform());
	
	const TArray<FTransform>& BoidTransforms = BoidSystem.GetSpatialContext().GetTransforms();
	InstancedMeshComp->SetNumCustomDataFloats(4);
	InstancedMeshComp->AddInstances(BoidTransforms, false, false);
}

void ABeliSwarm::Tick(float DeltaTime)
{
	Super::Tick(DeltaTime);
	
	check(IsValid(InstancedMeshComp));

	// 1. 지나간 시간만큼 Boid 객체들의 위치 값을 갱신하고,
	BoidSystem.UpdateBoids_Concurrent(DeltaTime, InstancedMeshComp->GetComponentTransform());
	
	// 2. 보이드의 정보를 얻어와 ISMC에 적용
	const TArray<FTransform>& BoidTransforms = BoidSystem.GetSpatialContext().GetTransforms();
	const TArray<FVector3f>& BoidVels = BoidSystem.GetSpatialContext().GetVelocities();
	const TArray<float>& ExclTimes = BoidSystem.GetSpatialContext().GetExclusiveTimes();

	InstancedMeshComp->BatchUpdateInstancesTransforms(0, BoidTransforms, true, false, false);
	for (int32 i = 0; i < BoidTransforms.Num(); ++i)
	{
		// ISMC에 Instance들에게 각 데이터를 넘겨준다. 메테리얼에서 추출해서 자유롭게 사용.
		InstancedMeshComp->SetCustomDataValue(i, 0, BoidVels[i].X, false);
		InstancedMeshComp->SetCustomDataValue(i, 1, BoidVels[i].Y, false);
		InstancedMeshComp->SetCustomDataValue(i, 2, BoidVels[i].Z, false);
		InstancedMeshComp->SetCustomDataValue(i, 3, ExclTimes[i], false);
	}
	
	InstancedMeshComp->MarkRenderStateDirty();
}

