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
 * 각 Boid 계산 시 필요한 정보 (글로벌 상수 등)
 */
USTRUCT()
struct FBoidSceneContext
{
	GENERATED_BODY()
	
	float BoidMaxSpeed;
	
	FTransform ManagerTransform;
	
#if !UE_BUILD_SHIPPING
	bool bIsDebugMode;
#endif
};

