
#include "BoidSystem.h"

#include "Beli.h"
#include "BoidRules.h"

#include "Library/SpatialGridHashHelper.h"


namespace BeliConsole
{
	static TAutoConsoleVariable CVarShowSweepTest(
	TEXT("Boids.ShowSweepTest"),
	0, // 기본값: 0 (디버그 끄기, 멀티스레드 켜기), 
	TEXT("Draw debug lines for Boids. 0=Off(MultiThread), 1 = On (ForceSingleThread)(DebugParam)"),
	ECVF_Cheat);
	
	static TAutoConsoleVariable CVarBoidGridDensityThreshold(
	TEXT("Boid.GridDensityThreshold"),
	0,
	TEXT("Threshold percentage for Spatial Hash Grid debug drawing.\n")
	TEXT("  0     : Disable grid debug drawing.\n")
	TEXT("  1~100 : Draw grid cells where boid density exceeds this percentage (%)."), 
	ECVF_Cheat);
}


DECLARE_CYCLE_STAT(TEXT("Total Boids Update"), STAT_BoidsTotalUpdate, STATGROUP_Boids);
DECLARE_CYCLE_STAT(TEXT("Parallel Rule Calc"), STAT_BoidsRuleCalc, STATGROUP_Boids);
DECLARE_CYCLE_STAT(TEXT("Swap Buffer"), STAT_BoidsSwapBuffers, STATGROUP_Boids);


// 중심(Center) -> 면(Face) -> 모서리(Edge) -> 꼭짓점(Corner) 순서
namespace BoidSystem
{
	static const FSpatialGrid GridSearchOffsets[27] = {
		// 1. 내 현재 그리드 (가장 가깝고 가장 먼저 찾아야 함)
		FSpatialGrid(0, 0, 0),
    
		// 2. 면 (상하좌우앞뒤 - 거리 1)
		FSpatialGrid(1, 0, 0), FSpatialGrid(-1, 0, 0),
		FSpatialGrid(0, 1, 0), FSpatialGrid(0, -1, 0),
		FSpatialGrid(0, 0, 1), FSpatialGrid(0, 0, -1),
    
		// 3. 모서리 (대각선 면 - 거리 약 1.414)
		FSpatialGrid(1, 1, 0),  FSpatialGrid(1, -1, 0),  FSpatialGrid(-1, 1, 0),  FSpatialGrid(-1, -1, 0),
		FSpatialGrid(1, 0, 1),  FSpatialGrid(1, 0, -1),  FSpatialGrid(-1, 0, 1),  FSpatialGrid(-1, 0, -1),
		FSpatialGrid(0, 1, 1),  FSpatialGrid(0, 1, -1),  FSpatialGrid(0, -1, 1),  FSpatialGrid(0, -1, -1),
    
		// 4. 꼭짓점 (완전 대각선 끝 - 거리 약 1.732)
		FSpatialGrid(1, 1, 1),   FSpatialGrid(1, 1, -1),   FSpatialGrid(1, -1, 1),   FSpatialGrid(1, -1, -1),
		FSpatialGrid(-1, 1, 1),  FSpatialGrid(-1, 1, -1),  FSpatialGrid(-1, -1, 1),  FSpatialGrid(-1, -1, -1)
	};
}


void FBoidSystem::Initialize(UWorld* InWorld, const FTransform& SimulationSpace)
{
	World = InWorld;
	
	// Boids 초기화 - 임의의 값들을 정해준다. 해당 임의 값을 토대로 BoidTransforms을 채운다.
	SpatialContext.Initialize(MaxBoidCount, GridCellSize, FVector::OneVector * DefaultBoidMeshScale);
	for (int32 i = 0; i < MaxBoidCount; ++i)
	{
		// Swarm이 설치된 위치를 기준으로 랜덤값을 구하기 위해 Local Space 상의 위치 값을 구하고 그 값을 토대로 WorldSpace로 변환.
		const FVector3f BoidNormal = FVector3f(FMath::VRand());
		const FVector3f RelRandLocation = FVector3f(FMath::RandRange(-1000, 1000), FMath::RandRange(-1000, 1000), FMath::RandRange(-1000, 1000));
		const FVector3f NewLocation = (FTransform3f(SimulationSpace).TransformPosition)(RelRandLocation);
		const FVector3f NewVelocity = BoidNormal * FMath::RandRange(0.f, MaxSpeed * 0.5f);
		const FRotator3f NewRotation = FRotator3f(FRotationMatrix44f::MakeFromZ(BoidNormal).Rotator());
		
		SpatialContext.WriteBoidData(i, NewLocation, NewRotation, NewVelocity, 0.f);
	}
	
	// 쓰기 버퍼에 할당된 값들을 읽기 버퍼로 옮겨 데이터 읽을 수 있게 준비해 놓는다.
	SpatialContext.SwapBuffer();
	
	// 규칙들 초기화, 규칙의 Force 쌓는 방식에 따라 내림차순 정렬.
	SortedRules = ActiveRules;
	for (UBoidRuleBase* RuleBase : SortedRules)
	{
		RuleBase->Initialize();
	}
	SortedRules.Sort([](const UBoidRuleBase& A, const UBoidRuleBase& B)
	{
		return B.AccumulationMode < A.AccumulationMode;
	});
}

void FBoidSystem::UpdateBoids_Concurrent(float DeltaTime, const FTransform& SimulationSpace)
{
	SCOPE_CYCLE_COUNTER(STAT_BoidsTotalUpdate);
	EParallelForFlags ParallelFlag = EParallelForFlags::Unbalanced;
	
	// Debug 모드일때는 MultiThread를 사용하지 않고 강제로 Single Thread를 사용하여 내부에서 DrawDebug 등을 문제없이 사용할 수 있게 처리.
#if !UE_BUILD_SHIPPING
	const bool bIsDebugSweepTest = BeliConsole::CVarShowSweepTest.GetValueOnGameThread() != 0;
	if (bIsDebugSweepTest)
	{
		ParallelFlag = EParallelForFlags::ForceSingleThread;
	}
#endif
	
	// Rule 내부에서 사용할 Context 준비, 각 Boid를 부가적으로 알아야 할 추가 정보 구조체.
	FBoidSceneContext Context;
	Context.BoidMaxSpeed = MaxSpeed;
	Context.SimulationSpace = FTransform3f(SimulationSpace);
#if !UE_BUILD_SHIPPING
	Context.DebugParam = BeliConsole::CVarShowSweepTest.GetValueOnGameThread();
#endif
	
	const float SearchRadiusSquared = GridCellSize * GridCellSize;
	
	{
		SCOPE_CYCLE_COUNTER(STAT_BoidsRuleCalc);
		
		const FBoidBuffer& BoidReadBuffer = SpatialContext.GetReadBuffer();
		const FSpatialGridHashHelper& GridHashHelper = SpatialContext.GetGridHashHelper();
		
		// for (int32 i = 0; i < MaxBoidsCount; ++i) 대신 ParallelFor를 사용하여 Multi-Thread 기반 반복문 사용 (최적화)
		ParallelFor(MaxBoidCount, [&](int32 i)
		{
			const FVector3f& BoidLocation = BoidReadBuffer.Locations[i]; 
		
			// 인접한 Grid 영역 탐색
			TArray<int32, TInlineAllocator<MaxNeighborsNum>> NeighborIndices;
			
			const FSpatialGrid& MyGrid = GridHashHelper.GetGridIndex(BoidLocation);
			for (int32 GridSearchIndex = 0; GridSearchIndex < 27; ++GridSearchIndex)
			{
				const FSpatialGrid NeighborGrid = MyGrid + BoidSystem::GridSearchOffsets[GridSearchIndex];
#if USE_CACHE_OPTIMIZED_LOGIC
				int32 StartIndex = 0;
				int32 BoidCount = 0;
				SpatialContext.GetBoidIndicesInGridHash(NeighborGrid, StartIndex, BoidCount);
				
				// 해당 그리드에 보이드가 1마리라도 들어있다면 탐색 시작!
				if (StartIndex != -1 && BoidCount > 0)
				{
					for (int32 Offset = 0; Offset < BoidCount; ++Offset)
					{
						const int32 TargetIndex = StartIndex + Offset;
						
						if (TargetIndex != i)
						{
							const float DistSquared = FVector3f::DistSquared(BoidLocation, BoidReadBuffer.Locations[TargetIndex]);
				            
							if (DistSquared < SearchRadiusSquared)
							{
								NeighborIndices.Add(TargetIndex);
				            
								// 추가적인 데이터 검색이 필요없을 때 중첩된 Loop문을 전부 종료 (최적화)
								if (NeighborIndices.Num() == MaxNeighborsNum)
								{
									goto EndSearch;
								}
							}
						}
					}
				}
#else
				for (int32 TargetIndex = SpatialContext.GetStartBoidIndexByHashKey(NeighborGrid); TargetIndex != -1; TargetIndex = SpatialContext.GetNextBoidByBoidIndex(TargetIndex))
				{
					if (TargetIndex != i)
					{
						const float DistSquared = FVector3f::DistSquared(BoidLocation, BoidReadBuffer.Locations[TargetIndex]);
				            
							if (DistSquared < SearchRadiusSquared)
							{
								NeighborIndices.Add(TargetIndex);
				            
								// 추가적인 데이터 검색이 필요없을 때 중첩된 Loop문을 전부 종료 (최적화)
								if (NeighborIndices.Num() == MaxNeighborsNum)
								{
									goto EndSearch;
								}
							}
					}
				}
#endif
			}
		
			EndSearch:
		
			// 인접한 Grid 영역에서 도출한 Neighbors(최대 갯수는 MaxNeighborsNum)에 대해서만 계산을 수행
			bool bShouldSkipGeneral = SpatialContext.ReadExclTime(i) > 0.f;
			FBoidRuleResult AccumulatedResult;
			for (const UBoidRuleBase* BoidRule : SortedRules)
			{
				// 가장 최근에 적용한 규칙이 Exc이면서 현재 규칙이 General 일때는 Loop를 종료.
				if (BoidRule->AccumulationMode == EBoidAccumulationMode::General)
				{
					if (bShouldSkipGeneral)
					{
						break;
					}
				}
				
				FBoidRuleResult OutResult;
				if (BoidRule->EvaluateBoid(OutResult, BoidReadBuffer, i,NeighborIndices, Context))
				{
					AccumulatedResult += OutResult;
					
					// Excl 규칙이 적용되면 General 규칙을 적용하지 못하도록 해당 플래그 값을 갱신
					if (BoidRule->AccumulationMode == EBoidAccumulationMode::Exclusive)
					{
						bShouldSkipGeneral = true;
					}
				}
			}
			
			// 누적된 값으로 Force값을 구하고 이 값을 기반으로 속도, 위치, 
			const FVector3f NewForce = AccumulatedResult.Force.GetClampedToMaxSize(MaxForce);
		
			// Result - WriteBuffer에 값을 적용하고 Transform 획득, 반영
			const FVector3f MyBoidVelocity = SpatialContext.ReadVelocity(i);
			const FVector3f MyBoidLocation = SpatialContext.ReadLocation(i);
			
			const FVector3f NewVelocity = (MyBoidVelocity + NewForce * DeltaTime).GetClampedToMaxSize(MaxSpeed);
			const FVector3f NewLocation = MyBoidLocation + (NewVelocity * DeltaTime);
			const FVector SmoothedVelocity = FMath::VInterpTo(FVector(MyBoidVelocity), FVector(NewVelocity), DeltaTime, 15.f);
			const FRotator3f NewRotation = FRotator3f(FRotationMatrix::MakeFromZ(SmoothedVelocity).Rotator());
			
			// 독점 레벨을 현재 DeltaTime만큼 주어 레벨에 비례해서 독점 시간을 일정 기간 가져간다. 매 수행 될때마다 DeltaTime을 빼줘서 일정 시간이 지나면 독점 시간이 사라지게 한다.
			// 독점 레벨을 획득한 프레임이였다면, 획득하자 마자 DeltaTime을 바로 빼주기 때문에 조금 부정확할 수 있다.
			const float NewExclusiveTime = FMath::Clamp(AccumulatedResult.ExclusiveLevel * DeltaTime - DeltaTime, 0.f, MaxExclusiveTime);
			
			SpatialContext.WriteBoidData(i, NewLocation, NewRotation, NewVelocity, NewExclusiveTime);
			
		}, ParallelFlag);
	}
	
	{
		SCOPE_CYCLE_COUNTER(STAT_BoidsSwapBuffers);
		SpatialContext.SwapBuffer();	
	}
	
#if !UE_BUILD_SHIPPING
	const int32 ThresholdPct = BeliConsole::CVarBoidGridDensityThreshold.GetValueOnGameThread();
	if (ThresholdPct > 0)
	{
		SpatialContext.ShowDebugGrid(World, ThresholdPct);
	}
	
	// 보다 원활한 테스팅을 위해 화면 좌상단에 항상 현재 Boid의 최대 갯수를 띄워준다. (여러 개 있을 땐... 
	if (GEngine)
	{
		GEngine->AddOnScreenDebugMessage(StaticCast<int32>(GetTypeHash(this)), 0.0f, FColor::Green, 
			FString::Printf(TEXT("Boid System (MaxBoidCount: %d)"), MaxBoidCount));
	}
#endif
}
