#pragma once

#define USE_CACHE_OPTIMIZED_LOGIC 1

#include "Library/SpatialGridHashHelper.h"

/**
 * 
 */
struct FBoidBuffer
{
public:
	void SetNumUninitialized(int32 MaxBoidCount)
	{
		Locations.SetNumUninitialized(MaxBoidCount);
		Rotations.SetNumUninitialized(MaxBoidCount);
		Velocities.SetNumUninitialized(MaxBoidCount);
		ExclusiveTimes.SetNumUninitialized(MaxBoidCount);
	}
	
	FORCEINLINE void CopyDataFrom(int32 DestIndex, const FBoidBuffer& SourceBuffer, int32 SourceIndex)
	{
		Locations[DestIndex] = SourceBuffer.Locations[SourceIndex];
		Rotations[DestIndex] = SourceBuffer.Rotations[SourceIndex];
		Velocities[DestIndex] = SourceBuffer.Velocities[SourceIndex];
		ExclusiveTimes[DestIndex] = SourceBuffer.ExclusiveTimes[SourceIndex];
	}
	
public:
	TArray<FVector3f> Locations;
	TArray<FRotator3f> Rotations;
	TArray<FVector3f> Velocities;
	TArray<float> ExclusiveTimes;
};


/**
 * Double Boid Buffer를 관리하고, 공간 해쉬를 통해 해당 버퍼에서 원하는 Boid 정보를 조회할 수 있다.
 */
struct FBoidSpatialContext
{
public:
	/** BoidWriteBuffer에는 Double-Buffer 패턴을 사용하기 위해 BoidReadBuffer와 똑같은 크기의 Boids 객체들을 할당하여 준비한다. */
	void Initialize(int32 InMaxBoidCount, float InGridCellSize, const FVector& InMeshScale);
	
	FORCEINLINE const FBoidBuffer& GetReadBuffer() const { return ReadBuffer; }
	FORCEINLINE FVector3f ReadLocation(int32 Index) const { check(Index < MaxBoidCount); return ReadBuffer.Locations[Index]; }
	FORCEINLINE FRotator3f ReadRotation(int32 Index) const { check(Index < MaxBoidCount); return ReadBuffer.Rotations[Index]; }
	FORCEINLINE FVector3f ReadVelocity(int32 Index) const { check(Index < MaxBoidCount); return ReadBuffer.Velocities[Index]; }
	FORCEINLINE float ReadExclTime(int32 Index) const { check(Index < MaxBoidCount); return ReadBuffer.ExclusiveTimes[Index]; }
	
	/** 쓰기 전용 버퍼에 최신화 된 값을 넣는다. Transform의 값도 즉시 갱신해 놓는다. */
	FORCEINLINE void WriteBoidData(int32 InIndex, const FVector3f& InLocation, const FRotator3f& InRotation, const FVector3f& InVelocity, float InExclusiveTime)
	{
		check(InIndex < MaxBoidCount);
	
		WriteBuffer.Locations[InIndex] = InLocation;
		WriteBuffer.Rotations[InIndex] = InRotation;
		WriteBuffer.Velocities[InIndex] = InVelocity;
		WriteBuffer.ExclusiveTimes[InIndex] = InExclusiveTime;
	
		check(BoidTransforms.IsValidIndex(InIndex));
		BoidTransforms[InIndex] = FTransform(FRotator(InRotation), FVector(InLocation), MeshScale);
	}
	
	FORCEINLINE int32 GetStartBoidIndexByHashKey(const FSpatialGrid& InGridIndex) const { return CellStartIndex[GridHashHelper.GetHashKey(InGridIndex)]; }
	
#if USE_CACHE_OPTIMIZED_LOGIC
	FORCEINLINE void GetBoidIndicesInGridHash(FSpatialGrid InGridIndex, OUT int32& StartIndex, OUT int32& Count)
	{
		const int32 TargetHash = GridHashHelper.GetHashKey(InGridIndex);
	
		StartIndex = CellStartIndex[TargetHash];
		Count = CellBoidCount[TargetHash];
	}
#else
	FORCEINLINE int32 GetNextBoidByBoidIndex(int32 InBoidIndex) const { return BoidNextIndex[InBoidIndex]; }
#endif
	
	FORCEINLINE const TArray<FTransform>& GetTransforms() const { return BoidTransforms; }
	
	/** */
	void SwapBuffer();
	
	/** */
	void ShowDebugGrid(const class UWorld* World, int32 ThresholdPct);
	
	const FSpatialGridHashHelper& GetGridHashHelper() const { return GridHashHelper; } 
	
	
private:
	int32 MaxBoidCount = 0;
	
	/** Hash값을 기반으로 해당 Hash를 가지고 있는 최초의 Boid Index 위치 정보 저장 - HashTable 크기 (일반적으로 Boids의 전체 크기 * 2) */
	TArray<int32> CellStartIndex;
	
#if USE_CACHE_OPTIMIZED_LOGIC
	/** 해당 Hash에 같은 Boid 갯수 저장 */
	TArray<int32> CellBoidCount;
	
	struct FBoidHashPair
	{
		int32 Index;
		int32 HashKey;
	};
	
	TArray<FBoidHashPair> HashPairs;
#else
	/** 해쉬에 따른 Boid들이 정렬이 되어 있지 않을 경우 인덱스 기반 링크드 리스트 같은 해쉬를 가진 보이드들을 연결 */
	TArray<int32> BoidNextIndex;
#endif
	
	float GridCellSize = 500.f;
	FVector MeshScale = FVector::OneVector;
	
	FBoidBuffer ReadBuffer;
	FBoidBuffer WriteBuffer;
	TArray<FTransform> BoidTransforms;
	
	FSpatialGridHashHelper GridHashHelper;
};


/**
 * 각 Boid 계산 시 필요한 정보 (글로벌 상수 등)
 */
struct FBoidSceneContext
{
	float BoidMaxSpeed = 0.f;
	FTransform3f SimulationSpace = FTransform3f::Identity;
	
#if !UE_BUILD_SHIPPING
	/** 디버그 정보 DebugParam이 0보다 크면 켜진 걸로 간주 */
	int32 DebugParam = 0;
#endif
};


/**
 * 
 */
struct FBoidRuleResult
{
	FVector3f Force = FVector3f::ZeroVector;
	float ExclusiveLevel = 0.f;
	// int32 AttractionID = 0;		// FIXME @Auggie State Enum을 새로 정의할까???
	
	FORCEINLINE FBoidRuleResult& operator+=(const FBoidRuleResult& Other)
	{
		Force += Other.Force;
		ExclusiveLevel += Other.ExclusiveLevel;
    
		return *this;
	}
};

