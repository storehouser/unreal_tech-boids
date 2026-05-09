#pragma once


/**
 * 
 */
struct FBoidBuffer
{
	
public:
	void SetNumUninitialized(int32 MaxBoidCount);
	void Reserve(int32 MaxBoidCount);
	
	void Add(int32 InID, const FVector3f& InLocation, const FRotator3f& InRotation, const FVector3f& InVelocity);
	void SetBoidData(int32 InIndex, const FVector3f& InLocation, const FRotator3f& InRotation, const FVector3f& InVelocity, float InExclusiveTime);
	
	FORCEINLINE int32 GetID(int32 Index) const { check (Index < NumBufferSize); return BoidIDs[Index]; }
	FORCEINLINE FVector3f GetLocation(int32 Index) const { check(Index < NumBufferSize); return Locations[Index]; }
	FORCEINLINE FRotator3f GetRotation(int32 Index) const { check(Index < NumBufferSize); return Rotations[Index]; }
	FORCEINLINE FVector3f GetVelocity(int32 Index) const { check(Index < NumBufferSize); return Velocities[Index]; }
	FORCEINLINE float GetExclTime(int32 Index) const { check(Index < NumBufferSize); return ExclusiveTimes[Index]; }  
	
	FORCEINLINE void CopyBoidDataFrom(int32 DestIndex, const FBoidBuffer& SourceBuffer, int32 SourceIndex)
	{
		check(NumBufferSize == SourceBuffer.NumBufferSize);
		check(DestIndex < NumBufferSize);
		check(SourceIndex < SourceBuffer.NumBufferSize);
		
		BoidIDs[DestIndex] = SourceBuffer.BoidIDs[SourceIndex];
		Locations[DestIndex] = SourceBuffer.Locations[SourceIndex];
		Rotations[DestIndex] = SourceBuffer.Rotations[SourceIndex];
		Velocities[DestIndex] = SourceBuffer.Velocities[SourceIndex];
		ExclusiveTimes[DestIndex] = SourceBuffer.ExclusiveTimes[SourceIndex];
	}
	
private:
	TArray<int32> BoidIDs;
	
	TArray<FVector3f> Locations;
	TArray<FRotator3f> Rotations;
	TArray<FVector3f> Velocities;
	TArray<float> ExclusiveTimes;
	
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


/**
 * 
 */
struct FBoidRuleResult
{
	FVector3f Force = FVector3f::ZeroVector;
	float ExclusiveTime = 0.f;
	// int32 AttractionID = 0;		// FIXME @Auggie State Enum을 새로 정의할까???
	
	FORCEINLINE FBoidRuleResult& operator+=(const FBoidRuleResult& Other)
	{
		Force += Other.Force;
		ExclusiveTime += Other.ExclusiveTime;
    
		return *this;
	}
};

