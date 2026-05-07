#pragma once


#include "BoidData.generated.h"


/**
 * 
 */
struct FBoidBuffer
{
	
public:
	void Reserve(int32 MaxBoidCount);
	
	void Add(int32 InID, FVector3f InLocation, FRotator3f InRotation, FVector3f InVelocity);
	void SetData(int32 InIndex, int32 InID, FVector3f InLocation, FRotator3f InRotation, FVector3f InVelocity);
	
	FORCEINLINE FTransform3f GetTransform(int32 Index) const
	{
		return FTransform3f(Rotations[Index], Locations[Index], FVector3f::OneVector * 0.5f);	// TEMP Scale은 일단 0.5 임시로
	}
	
private:
	TArray<int32> BoidIDs;
	
	TArray<FVector3f> Locations;
	TArray<FRotator3f> Rotations;
	TArray<FVector3f> Velocities;
	
	int32 NumBufferSize = 0;
};


/**
 * 각 Boid 계산 시 필요한 정보 (글로벌 상수 등)
 */
struct FBoidSceneContext
{
	float BoidMaxSpeed = 0.f;
	FTransform SimulationSpace = FTransform::Identity;
	
#if !UE_BUILD_SHIPPING
	/** 디버그 정보 DebugParam이 0보다 크면 켜진 걸로 간주 - 일반적으로 BoidIndex 값과 Mod 연산자를 이용해 일부 Boid에 출력을 설정 처리 */
	int32 DebugParam = 0;
#endif
};

