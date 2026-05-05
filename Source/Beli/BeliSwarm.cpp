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
		
		// Swarm이 설치된 위치를 기준으로 랜덤값을 구하기 위해 Local Space 상의 위치 값을 구하고 그 값을 토대로 WorldSpace로 변환.
		const FVector BoidNormal = FMath::VRand();
		const FVector RelRandLocation = FVector(FMath::RandRange(-500, 500), FMath::RandRange(-500, 500), FMath::RandRange(-500, 500));
		NewBoid.Location = InstancedMeshComp->GetComponentTransform().TransformPosition(RelRandLocation);
		NewBoid.Velocity = BoidNormal * FMath::RandRange(0.f, MaxSpeed);
		NewBoid.Rotation = FRotationMatrix::MakeFromZ(BoidNormal).Rotator();
		
		Boids.Emplace(NewBoid);
		
		InstancedMeshComp->AddInstance(NewBoid.GetTransform());
	}
	
	// Double-Buffer를 패턴을 사용하기 위해 똑같은 크기의 Boids 객체들을 준비. 안의 값은 중요치 않지만 같은 크기로 만들기 위해 그냥 복사로 처리.
	NextBoids = Boids;
	
	// 규칙 초기화
	for (UBoidRuleBase* RuleBase : ActiveRules)
	{
		RuleBase->Initialize();
	}
}

void ABeliSwarm::Tick(float DeltaTime)
{
	Super::Tick(DeltaTime);
	
	check(IsValid(InstancedMeshComp));
	
	FBoidSceneContext Context;
	Context.BoidMaxSpeed = MaxSpeed;
	Context.ManagerTransform = InstancedMeshComp->GetComponentTransform();
	
	// TODO 해당 배열을 매 Tick 마다 생성하지 말고 미리 생성.
	// 혹은 Boid 배열 자체를 Transform으로 처리할 수 있게
	TArray<FTransform> NewTransforms;
	NewTransforms.Reserve(MaxBoidsCount);
	for (int32 i = 0; i < MaxBoidsCount; ++i)
	{
		FBoidData& Boid = Boids[i];
		FBoidData& NextBoid = NextBoids[i];
		
		FVector CalculatedForce = FVector::ZeroVector;
		for (const UBoidRuleBase* ActiveRule : ActiveRules)
		{
			CalculatedForce += ActiveRule->CalculateForce(Boid, Boids, Context);
		}
		
		FVector NewForce = CalculatedForce.GetClampedToMaxSize(MaxForce);
		
		// 기본 메쉬가 하늘을 바라보고 있어 Rotation값에서 MakeFormZ 함수를 사용.
		NextBoid.Velocity = (Boid.Velocity + NewForce * DeltaTime).GetClampedToMaxSize(MaxSpeed);
		NextBoid.Rotation = FMath::RInterpTo(Boid.Rotation, FRotationMatrix::MakeFromZ(NextBoid.Velocity.GetSafeNormal()).Rotator(), DeltaTime, 30.f);
		NextBoid.Location = Boid.Location + (NextBoid.Velocity * DeltaTime);
		
		NewTransforms.Emplace(NextBoid.GetTransform());
	}
	
	InstancedMeshComp->BatchUpdateInstancesTransforms(0, NewTransforms, true, true, false);
	
	Swap(Boids, NextBoids);
}
