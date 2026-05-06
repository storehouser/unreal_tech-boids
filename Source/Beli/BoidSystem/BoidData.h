#pragma once


#include "BoidData.generated.h"


/**
 * 각 Boid 객체에서 관리되는 정보 - 위치, 속도, 방향
 */
USTRUCT()
struct FBoidData
{
	GENERATED_BODY()
	
public:
	int32 Index = -1;
	
	FVector Location = FVector::ZeroVector;
	FRotator Rotation = FRotator::ZeroRotator;
	FVector Velocity = FVector::ZeroVector;
	
public:
	FTransform GetTransform() const
	{
		return FTransform(Rotation, Location, FVector::OneVector * 0.5f);
	}
};


/**
 * 각 Boid 계산 시 필요한 정보 (글로벌 상수 등)
 */
USTRUCT()
struct FBoidSceneContext
{
	GENERATED_BODY()
	
	float BoidMaxSpeed;
	FTransform SimulationSpace;
	
#if !UE_BUILD_SHIPPING
	/** 디버그 정보 DebugParam이 0보다 크면 켜진 걸로 간주 - 일반적으로 BoidIndex 값과 Mod 연산자를 이용해 일부 Boid에 출력을 설정 처리 */
	int32 DebugParam;
#endif
};

