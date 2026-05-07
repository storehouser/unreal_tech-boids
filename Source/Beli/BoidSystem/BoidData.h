#pragma once


/**
 * 
 */
struct FBoidBuffer
{
	
public:
	void Reserve(int32 MaxBoidCount);
	
	void Add(int32 InID, FVector3f InLocation, FRotator3f InRotation, FVector3f InVelocity);
	void SetData(int32 InIndex, int32 InID, FVector3f InLocation, FRotator3f InRotation, FVector3f InVelocity);
	
	FORCEINLINE int32 GetID(int32 Index) const { check (Index < NumBufferSize); return BoidIDs[Index]; }
	FORCEINLINE FVector3f GetLocation(int32 Index) const { check(Index < NumBufferSize); return Locations[Index]; }
	FORCEINLINE FRotator3f GetRotation(int32 Index) const { check(Index < NumBufferSize); return Rotations[Index]; }
	FORCEINLINE FVector3f GetVelocity(int32 Index) const { check(Index < NumBufferSize); return Velocities[Index]; }
	
	FORCEINLINE FTransform GetTransform(int32 Index) const
	{
		return FTransform(FRotator(Rotations[Index]), FVector(Locations[Index]), FVector::OneVector * 0.5f);	// TEMP Scale은 일단 0.5 임시로
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
	FTransform3f SimulationSpace = FTransform3f::Identity;
	
#if !UE_BUILD_SHIPPING
	/** 디버그 정보 DebugParam이 0보다 크면 켜진 걸로 간주 - 일반적으로 BoidIndex 값과 Mod 연산자를 이용해 일부 Boid에 출력을 설정 처리 */
	int32 DebugParam = 0;
#endif
};

