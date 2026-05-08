// Fill out your copyright notice in the Description page of Project Settings.

#pragma once

#include "CoreMinimal.h"
#include "UObject/Object.h"
#include "BoidData.h"

#include "BoidRules.generated.h"

/**
 * BoidRule 계산식, Class Property를 설정한 추상 클래스
 */
UCLASS(Abstract, Blueprintable, EditInlineNew, DefaultToInstanced)
class BELI_API UBoidRuleBase : public UObject
{
	GENERATED_BODY()
	
public:
	virtual void Initialize() { }
	
	/** 각 룰에서 구한 값을 토대로 Weight 값을 곱해서 최종 값 */
	virtual FVector3f CalculateForce(const FBoidBuffer& BoidBuffer, int32 MyIndex, TArrayView<int32> NeighborIndices, const FBoidSceneContext& BoidSceneContext) const;
	
protected:
	/** 내부 룰 안에서 계산하여 최종 Force 값 계산 */
	virtual FVector3f CalculateForce_Internal(const FBoidBuffer& BoidBuffer, int32 MyIndex, TArrayView<int32> NeighborIndices, const FBoidSceneContext& BoidSceneContext) const { return FVector3f::ZeroVector; }
	
public:
	UPROPERTY(EditAnywhere, Category = "Boids")
	float Weight = 1.0f;
};


/**
 * 크레이그 기본 규칙 1 - 응집 : 인접한 Boid 객체들의 중심으로 이동
 */
UCLASS()
class BELI_API UBoidRule_Cohesion : public UBoidRuleBase
{
	GENERATED_BODY()
	
public:
	UBoidRule_Cohesion();
	
protected:
	virtual FVector3f CalculateForce_Internal(const FBoidBuffer& BoidBuffer, int32 MyIndex, TArrayView<int32> NeighborIndices, const FBoidSceneContext& BoidSceneContext) const override;
	
public:
	UPROPERTY(EditAnywhere, Category = "Boids")
	float CohesionRadius = 200.f;
	
	UPROPERTY(EditAnywhere, Category = "Boids")
	float SlowingRadius = 100.f; 
};

/**
 * 크레이그 기본 규칙 2 - 분리 : Boid 객체 간의 거리를 유지
 */
UCLASS()
class BELI_API UBoidRule_Separation : public UBoidRuleBase
{
	GENERATED_BODY()
	
public:
	UBoidRule_Separation();
	
protected:
	virtual FVector3f CalculateForce_Internal(const FBoidBuffer& BoidBuffer, int32 MyIndex, TArrayView<int32> NeighborIndices, const FBoidSceneContext& BoidSceneContext) const override;
	
public:
	UPROPERTY(EditAnywhere, Category = "Boids")
	float SeparationRadius = 70.f;
};


/**
 * 크레이그 기본 규칙 3 - 정렬 : 인접한 Boid 객체들의 속도 모방
 */
UCLASS()
class BELI_API UBoidRule_Alignment : public UBoidRuleBase
{
	GENERATED_BODY()
	
public:
	UBoidRule_Alignment();
	
protected:
	virtual FVector3f CalculateForce_Internal(const FBoidBuffer& BoidBuffer, int32 MyIndex, TArrayView<int32> NeighborIndices, const FBoidSceneContext& BoidSceneContext) const override;
	
public:
	UPROPERTY(EditAnywhere, Category = "Boids")
	float AlignmentRadius = 500.f;
};

/**
 * 특정 목적지로 향하게 하는 규칙 - PlaceLocation은 ManagerComponent Space 기준
 */
UCLASS()
class BELI_API UBoidRule_TendingToPlace : public UBoidRuleBase
{
	GENERATED_BODY()
	
public:
	UBoidRule_TendingToPlace();
	
protected:
	virtual FVector3f CalculateForce_Internal(const FBoidBuffer& BoidBuffer, int32 MyIndex, TArrayView<int32> NeighborIndices, const FBoidSceneContext& BoidSceneContext) const override;
	
public:
	UPROPERTY(EditAnywhere, Category = "Boids")
	FVector3f PlaceLocation = FVector3f::ZeroVector;
};


/**
 * 장애물 피하는 규칙
 */
UCLASS()
class BELI_API UBoidRule_AvoidanceObstacle : public UBoidRuleBase
{
	GENERATED_BODY()
	
public:
	UBoidRule_AvoidanceObstacle();
	
	virtual void Initialize() override;
	
protected:
	virtual FVector3f CalculateForce_Internal(const FBoidBuffer& BoidBuffer, int32 MyIndex, TArrayView<int32> NeighborIndices, const FBoidSceneContext& BoidSceneContext) const override;
	
public:
	UPROPERTY(EditAnywhere, Category = "Boids")
	float BoidRadius = 100.f;
	
	UPROPERTY(EditAnywhere, Category = "Boids")
	float AvoidDistance = 600.f;
	
	UPROPERTY(EditAnywhere, Category = "Boids")
	int32 NumRays = 20;
	
	UPROPERTY(EditAnywhere, Category = "Boids")
	float MaxRayDegree = 140.f;
	
private:
	TArray<FVector> FibonacciDirections;
	
	FCollisionShape SphereShape;
};
