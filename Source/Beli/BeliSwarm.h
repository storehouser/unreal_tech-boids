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
	FTransform GetTransform() const
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
	/** 응집 Force 계산 - 모든 이웃 사이의 중간점을 찾고 중간점을 향해 이동하는 규칙 */
	FVector CalcCohesion(const FBoidData& InBoidData) const;
	
	/** 분리 Force 계산 - 자기 주변의 객체들이 붐비는 것을 피하기 위해 근처 이웃들에서 벗어나는 규칙 */
	FVector CalcSeparation(const FBoidData& InBoidData) const;
	
	/** 정렬 Force 계산 - 이웃 객체들의 평균 방향으로 이동하는 규칙 */
	FVector CalcAlignment(const FBoidData& InBoidData) const;
	
	/** 충돌 체 회피 Force 계산 - 월드에 있는 정적 객체 회피 */
	FVector CalcObstacleAvoidance(const FBoidData& InBoidData) const;

	/** 목적지 Force 계산 */
    FVector CalcTendingToPlace(const FBoidData& InBoidData) const;
	
public:
	/** 최대 Boid 객체의 수 */
	UPROPERTY(EditAnywhere, Category = "Boid", meta = (UIMax = "3000"))
	int32 MaxBoidsCount = 50;
	
	UPROPERTY(EditAnywhere, Category = "Boid|Limitation")
	float MaxForce = 4000.f;
	
	UPROPERTY(EditAnywhere, Category = "Boid|Limitation")
	float MaxSpeed = 1500.f;
	
	UPROPERTY(EditAnywhere, Category = "Boid|Cohesion", meta = (ClampMin = "0.0", ClampMax = "1.0", UIMin = "0.0", UIMax = "1.0"))
	float CohesionWeight = 0.05f;
	
	UPROPERTY(EditAnywhere, Category = "Boid|Cohesion")
	float CohesionRadius = 200.f;
	
	UPROPERTY(EditAnywhere, Category = "Boid|Cohesion")
	float CohesionSlowingRadius = 100.f;
	
	UPROPERTY(EditAnywhere, Category = "Boid|Separation", meta = (ClampMin = "0.0", ClampMax = "1000.0", UIMin = "0.0", UIMax = "1000.0"))
	float SeparationWeight = 800.f;
	
	UPROPERTY(EditAnywhere, Category = "Boid|Separation")
	float SeparationRadius = 70.f;
	
	UPROPERTY(EditAnywhere, Category = "Boid|Alignment", meta = (UIMin = "0.0", UIMax = "1.0"))
	float AlignmentWeight = 0.2f;
	
	UPROPERTY(EditAnywhere, Category = "Boid|Alignment")
	float AlignmentRadius = 500.f;
	
	UPROPERTY(EditAnywhere, Category = "Boid|ObstacleAvoidance", meta = (ClampMin = "0.0", ClampMax = "5000.0", UIMin = "0.0", UIMax = "5000.0"))
	float ObstacleAvoidanceWeight = 3500.f;
	
	UPROPERTY(EditAnywhere, Category = "Boid|ObstacleAvoidance")
	float BoidRadius = 50.f;
	
	UPROPERTY(EditAnywhere, Category = "Boid|ObstacleAvoidance")
	float AvoidDistance = 600.f;
	
	UPROPERTY(EditAnywhere, Category = "Boid|TendingToPlace", meta = (ClampMin = "0.0", ClampMax = "1.0", UIMin = "0.0", UIMax = "1.0"))
	float TendingToPlaceWeight = 0.2f;
	
protected:
	UPROPERTY(VisibleAnywhere)
	TObjectPtr<class UInstancedStaticMeshComponent> InstancedMeshComp;
	
	TArray<FBoidData> Boids;
	TArray<FBoidData> NextBoids;
	
private:
	TArray<FVector> FibonacciDirections;
};
