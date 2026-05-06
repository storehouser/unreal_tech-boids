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
	void Initialize(const FTransform& SimulationSpace);
	
	/** */
	void UpdateBoids_Concurrent(float DeltaTime, const FTransform& SimulationSpace);
	
	/** */
	const TArray<FTransform>& GetBoidTransforms() const;
	
	/** */
	void SwapBuffers();
	
public:
	/** 최대 Boid 객체의 수 */
	UPROPERTY(EditAnywhere, Category = "Config", meta = (UIMax = "3000"))
	int32 MaxBoidCount = 50;
	
	UPROPERTY(EditAnywhere, Category = "Config")
	float MaxForce = 4000.f;
	
	UPROPERTY(EditAnywhere, Category = "Config")
	float MaxSpeed = 1500.f;
	
protected:
	UPROPERTY(EditAnywhere, Instanced, Category = "Rules")
	TArray<class UBoidRuleBase*> ActiveRules;
	
	TArray<FBoidData> Boids;
	TArray<FBoidData> NextBoids;
	
	TSharedPtr<struct FSpatialGridHashHelper> GridHashHelper;
	
private:
	/** */
	TArray<FTransform> BoidTransforms;
	
	/** HashTable 크기 (일반적으로 Boids의 전체 크기 * 2) */
	TArray<int32> CellStartIndex;
	
	/** Boids의 전체 크기 */
	TArray<int32> BoidNextIndex;	
	
	/** 최대 이웃 갯수, Stack 메모리를 사용하는 배열을 위해 Compile 타임에 지정 필요 */
	constexpr static int32 MaxNeighborsNum = 32;
	
	// FIXME - Grid를 촘촘하게 나눠서 검색할 수 있게 하거나 실제 Rule에서 가장 큰 값을 가져와서 취합해야 함
	float SearchRadius = 500.f;
};
