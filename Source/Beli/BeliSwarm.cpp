// Fill out your copyright notice in the Description page of Project Settings.


#include "BeliSwarm.h"

#include "Components/InstancedStaticMeshComponent.h"
#include "Library/BeliFunctionLibrary.h"


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
	InstancedMeshComp->SetNumCustomDataFloats(3);
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

	InstancedMeshComp->BatchUpdateInstancesTransforms(0, BoidTransforms, true, false, false);
	for (int32 i = 0; i < BoidTransforms.Num(); ++i)
	{
		FRotator Rotator = BoidTransforms[i].Rotator();

		// 1. Yaw(좌우 회전)를 0 ~ 360 사이의 깔끔한 각도로 보정합니다.
		// ClampAxis는 -90도 같은 음수나 400도 같은 오버된 값을 0~360 안으로 예쁘게 말아줍니다.
		float Hue = FRotator::ClampAxis(Rotator.Yaw);

		// 2. Pitch(위아래 고갯짓, -90 ~ 90)를 명도(Value)로 씁니다.
		// 위로 솟구치면(90) 밝아지고, 아래로 곤두박질치면(-90) 어두워지게 만듭니다.
		// 어둠 속으로 사라지는 걸 막기 위해 최소 밝기를 0.2로 잡았습니다.
		float Brightness = FMath::GetMappedRangeValueClamped(
			FVector2D(-90.0f, 90.0f),
			FVector2D(0.2f, 1.0f),
			Rotator.Pitch
		);

		// 3. FLinearColor에 R, G, B 대신 Hue, Saturation(채도=1.0), Value(명도)를 넣고,
		// HSVToLinearRGB() 함수를 호출해 진짜 RGB 색상으로 변환합니다!
		FLinearColor Color = FLinearColor(Hue, 1.0f, Brightness, 1.0f).HSVToLinearRGB();
		
		InstancedMeshComp->SetCustomDataValue(i, 0, Color.R, false);
		InstancedMeshComp->SetCustomDataValue(i, 1, Color.G, false);
		InstancedMeshComp->SetCustomDataValue(i, 2, Color.B, false);
	}
	
	InstancedMeshComp->MarkRenderStateDirty();
}

