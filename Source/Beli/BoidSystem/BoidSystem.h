#pragma once

#include "BoidData.h"

#include "BoidSystem.generated.h"


#define USE_CACHE_OPTIMIZED_LOGIC 1


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
	const TArray<FTransform>& GetBoidTransforms() const;
	
	/** */
	void SwapBuffers();
	
	/** */
	void ShowDebugGrid();
	
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
	
	FBoidBuffer BoidReadBuffer;
	FBoidBuffer BoidWriteBuffer;
	
	TSharedPtr<struct FSpatialGridHashHelper> GridHashHelper;
	
private:
	/** */
	TArray<FTransform> BoidTransforms;
	
	/** Hash값을 기반으로 해당 Hash를 가지고 있는 최초의 Boid Index 위치 정보 저장 - HashTable 크기 (일반적으로 Boids의 전체 크기 * 2) */
	TArray<int32> CellStartIndex;

#if USE_CACHE_OPTIMIZED_LOGIC
	/** 해당 Hash에 같은 Boid 갯수 저장 */
	TArray<int32> CellBoidCount;
#else
	/** 해쉬에 따른 Boid들이 정렬이 되어 있지 않을 경우 인덱스 기반 링크드 리스트 같은 해쉬를 가진 보이드들을 연결 */
	TArray<int32> BoidNextIndex;
#endif
	
	/** 최대 이웃 갯수, Stack 메모리를 사용하는 배열을 위해 Compile 타임에 지정 필요 */
	constexpr static int32 MaxNeighborsNum = 16;
	
	// TODO @Auggie Rule 중에 가장 큰 탐색 범위를 가져와서 검색할 수 있게 변경 필요
	float GridCellSize = 500.f;
	
	struct FBoidHashPair
	{
		int32 Index;
		int32 HashKey;
	};
	
	TArray<FBoidHashPair> HashPairs;
	
	UPROPERTY()
	TObjectPtr<class UWorld> World;
};
