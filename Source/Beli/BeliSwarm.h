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
	
	/** 현재 위치 */
	FVector Location;
	
	/** 이름 방향 및 속력 */
	FVector Velocity;
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
	
public:
	/** 최대 Boid 객체의 수 */
	UPROPERTY(EditAnywhere, Category = "Boid")
	int32 MaxNumBoids = 10;
	
protected:
	/** */
	UPROPERTY(VisibleAnywhere)
	TObjectPtr<class UInstancedStaticMeshComponent> InstancedMeshComp;
};
