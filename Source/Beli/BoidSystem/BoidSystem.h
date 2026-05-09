#pragma once

#include "BoidData.h"

#include "BoidSystem.generated.h"


/**
 * 
 */
USTRUCT(BlueprintType)
struct FBoidSystem
{
	GENERATED_BODY()
	
public:
	/** */
	void Initialize(class UWorld* InWorld, const FTransform& SimulationSpace);
	
	/** */
	void UpdateBoids_Concurrent(float DeltaTime, const FTransform& SimulationSpace);
	
	/** */
	const TArray<FTransform>& GetBoidTransforms() const { return SpatialContext.GetTransforms(); }
	
public:
	/** 최대 Boid 객체의 수 */
	UPROPERTY(EditAnywhere, Category = "Config", meta = (UIMax = "3000"))
	int32 MaxBoidCount = 50;
	
	UPROPERTY(EditAnywhere, Category = "Config")
	float MaxForce = 4000.f;
	
	UPROPERTY(EditAnywhere, Category = "Config")
	float MaxSpeed = 1500.f;
	
	UPROPERTY(EditAnywhere, Category = "Config")
	float MaxExclusiveTime = 5.0f;
	
	UPROPERTY(EditAnywhere, Category = "Config")
	FVector DefaultBoidMeshScale = FVector::OneVector * 0.5f;
	
protected:
	UPROPERTY(EditAnywhere, Instanced, Category = "Rules")
	TArray<class UBoidRuleBase*> ActiveRules;
	
private:
	UPROPERTY()
	TObjectPtr<class UWorld> World;
	
	/** 실제 Boid 규칙이 적용될 때는 Excl 규칙이 적용된 후에 General 규칙을 적용하지 않는다. 해당 기능을 수행하기 위해 누적 모드에 따라 규칙을 정렬해 놓는다 */
	UPROPERTY(Transient)
	TArray<class UBoidRuleBase*> SortedRules;
	
	/** 최대 이웃 갯수, Stack 메모리를 사용하는 배열을 위해 Compile 타임에 지정 필요 */
	constexpr static int32 MaxNeighborsNum = 16;
	
	FBoidSpatialContext SpatialContext;
	
	// TODO @Auggie Rule 중에 가장 큰 탐색 범위를 가져와서 검색할 수 있게 변경 필요
	float GridCellSize = 500.f;
};
