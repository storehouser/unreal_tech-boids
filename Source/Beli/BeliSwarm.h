// Fill out your copyright notice in the Description page of Project Settings.

#pragma once

#include "CoreMinimal.h"
#include "GameFramework/Actor.h"
#include "BoidsRule/BoidRules.h"

#include "BeliSwarm.generated.h"


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
	UPROPERTY(EditAnywhere, Category = "Boids", meta = (UIMax = "3000"))
	int32 MaxBoidsCount = 50;
	
	UPROPERTY(EditAnywhere, Category = "Boids|Limitation")
	float MaxForce = 4000.f;
	
	UPROPERTY(EditAnywhere, Category = "Boids|Limitation")
	float MaxSpeed = 1500.f;
	
protected:
	UPROPERTY(VisibleAnywhere)
	TObjectPtr<class UInstancedStaticMeshComponent> InstancedMeshComp;
	
	TArray<FBoidData> Boids;
	TArray<FBoidData> NextBoids;
	
protected:
	UPROPERTY(EditAnywhere, Instanced, Category = "Boids|Rules")
	TArray<class UBoidRuleBase*> ActiveRules;
};
