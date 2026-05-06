// Fill out your copyright notice in the Description page of Project Settings.

#pragma once

#include "CoreMinimal.h"
#include "GameFramework/Actor.h"
#include "BoidsRule/BoidRules.h"

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
	
private:
	/** 현재 Boids들에 대한 위치 등을 토대로 ActiveRules 적용. 현재 Boids의 데이터를 읽기만 하고 NextBoids에 쓰기만 수행. */
	void UpdateBoids(float DeltaTime);
	
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
	
	UPROPERTY(EditAnywhere, Instanced, Category = "Boids|Rules")
	TArray<class UBoidRuleBase*> ActiveRules;
	
	TArray<FBoidData> Boids;
	TArray<FBoidData> NextBoids;
	
private:
	TSharedPtr<struct FSpatialGridHashHelper> GridHashHelper;
	
	// FIXME - Grid를 촘촘하게 나눠서 검색할 수 있게 하거나 실제 Rule에서 가장 큰 값을 가져와서 취합해야 함
	float SearchRadius = 500.f;
	
	// TODO @Auggie FBoidSystem으로 통합을 고려
	TArray<FTransform> NextTransforms;
	TArray<int32> CellStartIndex;	// HashTable 크기 (일반적으로 Boids의 전체 크기 * 2)
	TArray<int32> BoidNextIndex;	// Boids의 전체 크기
	
	/** 최대 이웃 갯수, Stack 메모리를 사용하는 배열을 위해 Compile 타임에 지정 필요 */
	constexpr static int32 MaxNeighborsNum = 32;
};
