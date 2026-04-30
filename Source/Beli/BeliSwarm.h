// Fill out your copyright notice in the Description page of Project Settings.

#pragma once

#include "CoreMinimal.h"
#include "GameFramework/Actor.h"
#include "BeliSwarm.generated.h"


/**
 * 각 Boid 객체에서 관리되는 정보 - 위치, 속도, 방향
 */
USTRUCT()
struct FBoidData
{
	GENERATED_BODY()
	
public:
	int32 Index = -1;
	
	/** 현재 위치 */
	FVector Location = FVector::ZeroVector;
	
	/** 방향 */
	FRotator Rotation = FRotator::ZeroRotator;
	
	/** 이름 방향 및 속력 */
	FVector Velocity = FVector::ZeroVector;
	
public:
	FTransform GetTransaform() const
	{
		return FTransform(Rotation, Location, FVector::OneVector * 0.5f);
	}
};


/**
 * Static Mesh로 이루어진 각 객체(Boid)들을 관리하는 SWarm 객체.
 */
UCLASS()
class BELI_API ABeliSwarm : public AActor
{
	GENERATED_BODY()

public:
	ABeliSwarm();

protected:
	virtual void BeginPlay() override;

public:
	virtual void Tick(float DeltaTime) override;
	
protected:
	/** 응집 계산 - 모든 이웃 사이의 중간점을 찾고 중간점을 향해 이동하는 규칙 */
	FVector CalcCohesion(const FBoidData& InBoidData) const;
	
	/** 분리 계산 - 자기 주변의 객체들이 붐비는 것을 피하기 위해 근처 이웃들에서 벗어나는 규칙 */
	FVector CalcSeperation(const FBoidData& InBoidData) const;
	
	/** 정렬 계산 - 이웃 객체들의 평균 방향으로 이동하는 규칙 */
	FVector CalcAlignment(const FBoidData& InBoidData) const;

	/** 목적지 계산 */
    FVector CalcTendingToPlace(const FBoidData& InBoidData) const;
	
public:
	/** 최대 Boid 객체의 수 */
	UPROPERTY(EditAnywhere, Category = "Boid")
	int32 MaxBoidsCount = 50;
	
protected:
	/** */
	UPROPERTY(VisibleAnywhere)
	TObjectPtr<class UInstancedStaticMeshComponent> InstancedMeshComp;
	
	TArray<FBoidData> Boids;
};
