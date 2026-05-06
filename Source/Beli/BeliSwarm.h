// Fill out your copyright notice in the Description page of Project Settings.

#pragma once

#include "CoreMinimal.h"
#include "GameFramework/Actor.h"
#include "BoidSystem/BoidSystem.h"

#include "BeliSwarm.generated.h"


/**
 * Static Mesh로 이루어진 각 객체(Boid)들을 관리하는 Swarm 객체.
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
	UPROPERTY(VisibleAnywhere)
	TObjectPtr<class UInstancedStaticMeshComponent> InstancedMeshComp;
	
	UPROPERTY(EditAnywhere, Category = "Boids")
	FBoidSystem BoidSystem;
};
