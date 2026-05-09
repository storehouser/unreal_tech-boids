
#include "BoidSystem.h"

#include "Beli.h"
#include "BoidRules.h"

#include "Library/SpatialGridHashHelper.h"


namespace BeliConsole
{
	static TAutoConsoleVariable CVarShowSweepTest(
	TEXT("Boids.ShowSweepTest"),
	0, // 기본값: 0 (디버그 끄기, 멀티스레드 켜기), 
	TEXT("Draw debug lines for Boids. 0=Off(MultiThread), N > 0 = On (ForceSingleThread)(DebugParam)"),
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
DECLARE_CYCLE_STAT(TEXT("Spatial Hashing"), STAT_BoidsSpatialHash, STATGROUP_Boids);
DECLARE_CYCLE_STAT(TEXT("Parallel Rule Calc"), STAT_BoidsRuleCalc, STATGROUP_Boids);
DECLARE_CYCLE_STAT(TEXT("Buffer Sort Based On Hash"), STAT_BoidsBufferSort, STATGROUP_Boids);
DECLARE_CYCLE_STAT(TEXT("Data Shuffling"), STAT_BoidsDataShuffling, STATGROUP_Boids);


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
	// BoidWriteBuffer에는 Double-Buffer 패턴을 사용하기 위해 BoidReadBuffer와 똑같은 크기의 Boids 객체들을 할당하여 준비한다.
	BoidWriteBuffer.Reserve(MaxBoidCount);
	BoidTransforms.Reserve(MaxBoidCount);
	for (int32 i = 0; i < MaxBoidCount; ++i)
	{
		int32 ID = i;
		
		// Swarm이 설치된 위치를 기준으로 랜덤값을 구하기 위해 Local Space 상의 위치 값을 구하고 그 값을 토대로 WorldSpace로 변환.
		const FVector3f BoidNormal = FVector3f(FMath::VRand());
		const FVector3f RelRandLocation = FVector3f(FMath::RandRange(-1000, 1000), FMath::RandRange(-1000, 1000), FMath::RandRange(-1000, 1000));
		const FVector3f NewLocation = (FTransform3f(SimulationSpace).TransformPosition)(RelRandLocation);
		const FVector3f NewVelocity = BoidNormal * FMath::RandRange(0.f, MaxSpeed * 0.5f);
		const FRotator3f NewRotation = FRotator3f(FRotationMatrix44f::MakeFromZ(BoidNormal).Rotator());
		
		BoidWriteBuffer.Add(ID, NewLocation, NewRotation, NewVelocity);
		BoidTransforms.Emplace(BoidWriteBuffer.GetTransform(i));
	}
	
	GridHashHelper = MakeShared<FSpatialGridHashHelper>(GridCellSize, MaxBoidCount);
	
	// 랜덤하게 적재된 WriteBuffer의 값을 ReadBuffer로 옮긴다.
	BoidReadBuffer.SetNumUninitialized(MaxBoidCount);
	SwapBuffers();
	
	// 규칙들 초기화
	for (UBoidRuleBase* RuleBase : ActiveRules)
	{
		RuleBase->Initialize();
	}
	
	BoidTransforms.Init(FTransform::Identity, MaxBoidCount);
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
		
		// for (int32 i = 0; i < MaxBoidsCount; ++i) 대신 ParallelFor를 사용하여 Multi-Thread 기반 반복문 사용 (최적화)
		ParallelFor(MaxBoidCount, [&](int32 i)
		{
			const FVector3f& BoidLocation = BoidReadBuffer.GetLocation(i); 
		
			// 인접한 Grid 영역 탐색
			TArray<int32, TInlineAllocator<MaxNeighborsNum>> NeighborIndices;
			const FSpatialGrid& MyGrid = GridHashHelper->GetGridIndex(BoidLocation);
			for (int32 GridSearchIndex = 0; GridSearchIndex < 27; ++GridSearchIndex)
			{
				const FSpatialGrid NeighborGrid = MyGrid + BoidSystem::GridSearchOffsets[GridSearchIndex];
				
				const int32 TargetHash = GridHashHelper->GetHashKey(NeighborGrid);
#if USE_CACHE_OPTIMIZED_LOGIC
				const int32 StartIndex = CellStartIndex[TargetHash];
				const int32 BoidCount = CellBoidCount[TargetHash]; 
				
				// 해당 그리드에 보이드가 1마리라도 들어있다면 탐색 시작!
				if (StartIndex != -1 && BoidCount > 0)
				{
					for (int32 Offset = 0; Offset < BoidCount; ++Offset)
					{
						const int32 TargetIndex = StartIndex + Offset;
						
						if (TargetIndex != i)
						{
							const float DistSquared = FVector3f::DistSquared(BoidLocation, BoidReadBuffer.GetLocation(TargetIndex));
				            
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
				for (int32 TargetIndex = CellStartIndex[TargetHash]; TargetIndex != -1; TargetIndex = BoidNextIndex[TargetIndex])
				{
					if (TargetIndex != i)
					{
						const float DistSquared = FVector3f::DistSquared(BoidLocation, BoidReadBuffer.GetLocation(TargetIndex));
				            
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
			FVector3f CalculatedForce = FVector3f::ZeroVector;
			for (const UBoidRuleBase* ActiveRule : ActiveRules)
			{
				CalculatedForce += ActiveRule->CalculateForce(BoidReadBuffer, i,NeighborIndices, Context);
			}
			const FVector3f NewForce = CalculatedForce.GetClampedToMaxSize(MaxForce);
		
			// WriteBuffer에 값을 적용하고 Transform 획득, 반영
			const FVector3f MyBoidVelocity = BoidReadBuffer.GetVelocity(i);
			const FVector3f MyBoidLocation = BoidReadBuffer.GetLocation(i);
			
			const FVector3f NewVelocity = (MyBoidVelocity + NewForce * DeltaTime).GetClampedToMaxSize(MaxSpeed);
			const FVector3f NewLocation = MyBoidLocation + (NewVelocity * DeltaTime);
			const FVector SmoothedVelocity = FMath::VInterpTo(FVector(MyBoidVelocity), FVector(NewVelocity), DeltaTime, 15.f);
			const FRotator3f NewRotation = FRotator3f(FRotationMatrix::MakeFromZ(SmoothedVelocity).Rotator());
			
			BoidWriteBuffer.SetBoidData(i, NewLocation, NewRotation, NewVelocity);
			
			// 
			BoidTransforms[i] = BoidWriteBuffer.GetTransform(i);
		}, ParallelFlag);
	}
	
	SwapBuffers();
	
#if !UE_BUILD_SHIPPING
	const bool bIsShowingBoid = BeliConsole::CVarBoidGridDensityThreshold.GetValueOnGameThread() > 0;
	if (bIsShowingBoid)
	{
		ShowDebugGrid();	
	}
	
	// 보다 원활한 테스팅을 위해 화면 좌상단에 항상 현재 Boid의 최대 갯수를 띄워준다. (여러 개 있을 땐... 
	if (GEngine)
	{
		GEngine->AddOnScreenDebugMessage(StaticCast<int32>(GetTypeHash(this)), 0.0f, FColor::Green, 
			FString::Printf(TEXT("Boid System (MaxBoidCount: %d)"), MaxBoidCount));
	}
#endif
}

const TArray<FTransform>& FBoidSystem::GetBoidTransforms() const
{
	return BoidTransforms;
}

void FBoidSystem::SwapBuffers()
{
#if USE_CACHE_OPTIMIZED_LOGIC
	// 
	{
		SCOPE_CYCLE_COUNTER(STAT_BoidsBufferSort);
		
		HashPairs.SetNumUninitialized(MaxBoidCount);
		
		ParallelFor(MaxBoidCount, [&](int32 i)
		{
			HashPairs[i].Index = i;
			HashPairs[i].HashKey = GridHashHelper->GetHashKeyFromLocation(BoidWriteBuffer.GetLocation(i));
		});
		
		HashPairs.Sort([](const FBoidHashPair& A, const FBoidHashPair& B)
		{
			return A.HashKey < B.HashKey;
		});
		
		// 최신화된 WriteBuffer에 있는 데이터들을 ReadBuffer로 옮긴다 (Swap). CPU 캐시 적중을 위해 Hash값 기준으로 정렬된 Index를 사용한다.
		ParallelFor(MaxBoidCount, [&](int32 i)
		{
			const int32 OriginIndex = HashPairs[i].Index;
			BoidReadBuffer.CopyBoidDataFrom(i, BoidWriteBuffer, OriginIndex);
		});
	}
	
	// 해쉬 기반으로 모든 보이드들을 추적할 수 있게 값을 저장. 해쉬를 기준으로 정렬이 됐으므로 최초의 StartIndex에 기록, 갯수를 기입한다.  
	{
		SCOPE_CYCLE_COUNTER(STAT_BoidsSpatialHash);
		
		CellStartIndex.Init(-1, GridHashHelper->GetHashSize());
		CellBoidCount.Init(0, GridHashHelper->GetHashSize());
		
		for (int32 i = 0; i < MaxBoidCount; ++i)
		{
			const int32 HashKey = GridHashHelper->GetHashKeyFromLocation(BoidReadBuffer.GetLocation(i));
			
			// 해당 해시의 첫 번째 보이드라면 시작 인덱스로 기록
			if (CellBoidCount[HashKey] == 0)
			{
				CellStartIndex[HashKey] = i;
			}
        
			// 해당 해시 셀의 보이드 카운트 1 증가
			CellBoidCount[HashKey]++;
		}
	}
#else
	Swap(BoidReadBuffer, BoidWriteBuffer);
	
	CellStartIndex.Init(-1, GridHashHelper->GetHashSize());
	BoidNextIndex.Init(-1, MaxBoidCount);

	{
		SCOPE_CYCLE_COUNTER(STAT_BoidsSpatialHash);

		for (int32 i = 0; i < MaxBoidCount; ++i)
		{
			const int32 HashKey = GridHashHelper->GetHashKeyFromLocation(BoidReadBuffer.GetLocation(i));

			// 고정된 배열의 크기에서 같은 Hash를 가지는 Boid Index 값을 저장 - 배열 기반의 LinkedList
			BoidNextIndex[i] = CellStartIndex[HashKey];
			CellStartIndex[HashKey] = i;
		}
	}
#endif
}

void FBoidSystem::ShowDebugGrid()
{
	check(IsValid(World));
	
	FBox BoidBounds;
	for (int32 i = 0; i < MaxBoidCount; ++i)
	{
		BoidBounds += FVector(BoidReadBuffer.GetLocation(i));
	}
	
	FSpatialGrid MinGrid = FSpatialGrid(
		FMath::FloorToInt(BoidBounds.Min.X / GridCellSize),
		FMath::FloorToInt(BoidBounds.Min.Y / GridCellSize),
		FMath::FloorToInt(BoidBounds.Min.Z / GridCellSize)
	);

	FSpatialGrid MaxGrid = FSpatialGrid(
		FMath::FloorToInt(BoidBounds.Max.X / GridCellSize),
		FMath::FloorToInt(BoidBounds.Max.Y / GridCellSize),
		FMath::FloorToInt(BoidBounds.Max.Z / GridCellSize)
	);
	
	int32 ThresholdPct = BeliConsole::CVarBoidGridDensityThreshold.GetValueOnGameThread();
	const int32 AbsoluteDensityThreshold = FMath::Floor(MaxBoidCount * (StaticCast<float>(ThresholdPct) * 0.01f));
	
	// 화면 좌측 상단에 텍스트 출력
	if (GEngine)
	{
		GEngine->AddOnScreenDebugMessage(StaticCast<int32>(GetTypeHash(this)) + 1, 0.0f, FColor::Cyan, 
			FString::Printf(TEXT("Boid Grid Debug : ON (Threshold - Percent: %d%%, Count: %d)"), ThresholdPct, AbsoluteDensityThreshold));
	}
	
	int32 DrawCount = 0;
	for (int32 X = MinGrid.X; X <= MaxGrid.X; X++)
	{
		for (int32 Y = MinGrid.Y; Y <= MaxGrid.Y; Y++)
		{
			for (int32 Z = MinGrid.Z; Z <= MaxGrid.Z; Z++)
			{
				++DrawCount;
				
				// 해시 테이블에서 해당 인덱스의 밀집도 검사 후 DrawDebugSolidBox 호출
				const FSpatialGrid GridIndex = FSpatialGrid(X, Y, Z);
				const int32 GridHashKey = GridHashHelper->GetHashKey(GridIndex);
				check(CellBoidCount.IsValidIndex(GridHashKey));
				const int32 NumBoidsInHash = CellBoidCount[GridHashKey];
				
				const FVector CellCenter = FVector(GridIndex) * GridCellSize + FVector(GridCellSize * 0.5f);
				DrawDebugBox(World, CellCenter, FVector(GridCellSize * 0.5f), FColor::Black, false, -1.0f, 0, 2.0f); // 외곽선
				
				if (NumBoidsInHash > AbsoluteDensityThreshold)
				{
					// 해시 키를 이용하여 황금값을 위한 Hue 값 게산
					const float Hue = FMath::Fmod(GridHashKey * 137.508f, 360.0f);
					FLinearColor HashLinearColor(Hue, 0.9f, 0.9f, 1.0f);
					FColor HeatColor = HashLinearColor.HSVToLinearRGB().ToFColor(true).WithAlpha(125);
					
					DrawDebugSolidBox(World, CellCenter, FVector(GridCellSize * 0.5f), HeatColor, false, -1.0f, 0);
				}
			}
		}
	}
	
	UE_LOG(LogBeli, Verbose, TEXT("Grid DebugDraw Count - %d"), DrawCount);
}
