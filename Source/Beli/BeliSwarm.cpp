// Fill out your copyright notice in the Description page of Project Settings.


#include "BeliSwarm.h"

#include "Beli.h"
#include "Components/InstancedStaticMeshComponent.h"

#include "Library/SpatialGridHashHelper.h"


namespace BeliConsole
{
	static TAutoConsoleVariable CVarDebugBoids(
	TEXT("Boids.DebugMode"),
	0, // 기본값: 0 (디버그 끄기, 멀티스레드 켜기), 
	TEXT("Draw debug lines for Boids. 0=Off(MultiThread), N > 0 = On(ForceSingleThread)(DebugParam)"),
	ECVF_Cheat);	
}


DECLARE_CYCLE_STAT(TEXT("Total Boids Update"), STAT_BoidsTotalUpdate, STATGROUP_Boids);
DECLARE_CYCLE_STAT(TEXT("Spatial Hashing"), STAT_BoidsSpatialHash, STATGROUP_Boids);
DECLARE_CYCLE_STAT(TEXT("Parallel Rule Calc"), STAT_BoidsRuleCalc, STATGROUP_Boids);


ABeliSwarm::ABeliSwarm()
{
	PrimaryActorTick.bCanEverTick = true;
	
	InstancedMeshComp = CreateDefaultSubobject<UInstancedStaticMeshComponent>(TEXT("InstancedMeshComp"));
	RootComponent = InstancedMeshComp;
	
	InstancedMeshComp->SetCollisionEnabled(ECollisionEnabled::Type::NoCollision);
}

void ABeliSwarm::BeginPlay()
{
	Super::BeginPlay();
	
	check(IsValid(InstancedMeshComp));
	
	// Boids 초기화 - 임의의 값들을 정해준다.
	// NextBoids에는 Double-Buffer를 패턴을 사용하기 위해 똑같은 크기의 Boids 객체들을 준비. 안의 값은 중요치 않지만 같은 크기로 만들기 위해 복사 처리
	Boids.Reserve(MaxBoidsCount);
	
	for (int32 i = 0; i < MaxBoidsCount; ++i)
	{
		FBoidData NewBoid;
		NewBoid.Index = i;
		
		// Swarm이 설치된 위치를 기준으로 랜덤값을 구하기 위해 Local Space 상의 위치 값을 구하고 그 값을 토대로 WorldSpace로 변환.
		const FVector BoidNormal = FMath::VRand();
		const FVector RelRandLocation = FVector(FMath::RandRange(-500, 500), FMath::RandRange(-500, 500), FMath::RandRange(-500, 500));
		NewBoid.Location = InstancedMeshComp->GetComponentTransform().TransformPosition(RelRandLocation);
		NewBoid.Velocity = BoidNormal * FMath::RandRange(0.f, MaxSpeed);
		NewBoid.Rotation = FRotationMatrix::MakeFromZ(BoidNormal).Rotator();
		
		Boids.Emplace(NewBoid);
		
		InstancedMeshComp->AddInstance(NewBoid.GetTransform());
	}
	
	NextBoids = Boids;
	
	// 규칙 초기화
	for (UBoidRuleBase* RuleBase : ActiveRules)
	{
		RuleBase->Initialize();
	}
	
	NextTransforms.Init(FTransform::Identity, MaxBoidsCount);
	
	// TODO SearchRadius는 Rule에서 가장 큰 값으로 가져오거나, 효율적으로 쪼개는 방법을 생각해보자.
	GridHashHelper = MakeShared<FSpatialGridHashHelper>(SearchRadius, MaxBoidsCount);
}

void ABeliSwarm::Tick(float DeltaTime)
{
	Super::Tick(DeltaTime);
	
	check(IsValid(InstancedMeshComp));
	
	UpdateBoids(DeltaTime);
}

void ABeliSwarm::UpdateBoids(float DeltaTime)
{
	SCOPE_CYCLE_COUNTER(STAT_BoidsTotalUpdate);
	
	// 효과적인 탐색 기법을 위해 해쉬 각 보이드 들의 위치 값을 기반으로 해쉬 값 처리 및 연걸 처리
	CellStartIndex.Init(-1, GridHashHelper->GetHashSize());
	BoidNextIndex.Init(-1, MaxBoidsCount);
	
	{
		SCOPE_CYCLE_COUNTER(STAT_BoidsSpatialHash);
		
		for (int32 i = 0; i < MaxBoidsCount; ++i)
		{
			const int32 HashKey = GridHashHelper->GetHashKeyFromLocation(Boids[i].Location);
		
			// 고정된 배열의 크기에서 같은 Hash를 가지는 Boid Index 값을 저장 - 배열 기반의 LinkedList
			BoidNextIndex[i] = CellStartIndex[HashKey];
			CellStartIndex[HashKey] = i;
		}	
	}
	
	EParallelForFlags ParallelFlag = EParallelForFlags::Unbalanced;
	
	// Debug 모드일때는 MultiThread를 사용하지 않고 강제로 Single Thread를 사용하여 내부에서 DrawDebug 등을 용이하게 할 수 있게 처리
#if !UE_BUILD_SHIPPING
	const bool bIsDebugMode = BeliConsole::CVarDebugBoids.GetValueOnGameThread() != 0;
	if (bIsDebugMode)
	{
		ParallelFlag = EParallelForFlags::ForceSingleThread;
	}
#endif
	
	// Rule 내부에서 사용할 Context 준비, 각 Boid를 부가적으로 알아야 할 추가 정보 구조체.
	FBoidSceneContext Context;
	Context.BoidMaxSpeed = MaxSpeed;
	Context.ManagerTransform = InstancedMeshComp->GetComponentTransform();
#if !UE_BUILD_SHIPPING
	Context.DebugParam = BeliConsole::CVarDebugBoids.GetValueOnGameThread();
#endif
	
	const float SearchRadiusSquared = SearchRadius * SearchRadius;
	
	{
		SCOPE_CYCLE_COUNTER(STAT_BoidsRuleCalc);
		
		// for (int32 i = 0; i < MaxBoidsCount; ++i) ; 최적화를 위해 Multi-Thread 기반 반복문 사용
		ParallelFor(MaxBoidsCount, [&](int32 i)
		{
			const FBoidData& Boid = Boids[i];
		
			// 인접한 Grid 영역 탐색
			TArray<const FBoidData*, TInlineAllocator<MaxNeighborsNum>> Neighbors;
			const FSpatialGrid MyGrid = GridHashHelper->GetGridIndex(Boid.Location);
			for (int32 Z = -1; Z <= 1; ++Z)
			{
				for (int32 Y = -1; Y <= 1; ++Y)
				{
					for (int32 X = -1; X <= 1; ++X)
					{
						const FSpatialGrid NeighborGrid(MyGrid.X + X, MyGrid.Y + Y, MyGrid.Z + Z);
						const int32 TargetHash = GridHashHelper->GetHashKey(NeighborGrid);
						for (int32 TargetIndex = CellStartIndex[TargetHash]; TargetIndex != -1; TargetIndex = BoidNextIndex[TargetIndex])
						{
							if (TargetIndex != i)
							{
								const float DistSquared = FVector::DistSquared(Boid.Location, Boids[TargetIndex].Location);
								if (DistSquared < SearchRadiusSquared)
								{
									Neighbors.Add(&Boids[TargetIndex]);
								
									// 추가적인 데이터 검색이 필요없을 때 중첩된 Loop문을 전부 종료 (최적화)
									if (Neighbors.Num() == MaxNeighborsNum)
									{
										goto EndSearch;
									}
								}
							}
						}
					}
				}
			}
		
			EndSearch:
		
			// 인접한 Grid 영역에서 도출한 Neighbors에 대해서만 계산을 수행
			FVector CalculatedForce = FVector::ZeroVector;
			for (const UBoidRuleBase* ActiveRule : ActiveRules)
			{
				CalculatedForce += ActiveRule->CalculateForce(Boid, Neighbors, Context);
			}
			const FVector NewForce = CalculatedForce.GetClampedToMaxSize(MaxForce);
		
			// 쓰기 전용인 NextBoids에 값을 적용하고 Transform 획득, 반영
			FBoidData& NextBoid = NextBoids[i];
			NextBoid.Velocity = (Boid.Velocity + NewForce * DeltaTime).GetClampedToMaxSize(MaxSpeed);
			NextBoid.Rotation = FMath::RInterpTo(Boid.Rotation, FRotationMatrix::MakeFromZ(NextBoid.Velocity.GetSafeNormal()).Rotator(), DeltaTime, 15.f);	// 기본 메쉬가 하늘을 바라보고 있어 Rotation값에서 MakeFormZ 함수를 사용.
			NextBoid.Location = Boid.Location + (NextBoid.Velocity * DeltaTime);
		
			NextTransforms[i] = NextBoid.GetTransform();
		}, ParallelFlag);
	}
	
	InstancedMeshComp->BatchUpdateInstancesTransforms(0, NextTransforms, true, true, false);
	
	Swap(Boids, NextBoids);
}
