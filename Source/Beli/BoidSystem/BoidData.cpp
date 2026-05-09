
#include "BoidData.h"

#include "Beli.h"
#include "Library/SpatialGridHashHelper.h"


void FBoidSpatialContext::Initialize(int32 InMaxBoidCount, float InGridCellSize, const FVector& InMeshScale)
{
	MaxBoidCount = InMaxBoidCount;
	GridCellSize = InGridCellSize;
	MeshScale = InMeshScale;
	
	WriteBuffer.SetNumUninitialized(MaxBoidCount);
	ReadBuffer.SetNumUninitialized(MaxBoidCount);
	BoidTransforms.SetNumUninitialized(MaxBoidCount);
	
	GridHashHelper = FSpatialGridHashHelper(GridCellSize, MaxBoidCount);
}

void FBoidSpatialContext::SwapBuffer()
{
#if USE_CACHE_OPTIMIZED_LOGIC
	// 최신화된 WriteBuffer에 있는 데이터들을 ReadBuffer로 옮긴다 (Swap). CPU 캐시 적중을 위해 Hash값 기준으로 정렬된 Index를 사용한다.  
	{
		HashPairs.SetNumUninitialized(MaxBoidCount);
		
		ParallelFor(MaxBoidCount, [&](int32 i)
		{
			HashPairs[i].Index = i;
			HashPairs[i].HashKey = GridHashHelper.GetHashKeyFromLocation(WriteBuffer.Locations[i]);
		});
		
		HashPairs.Sort([](const FBoidHashPair& A, const FBoidHashPair& B)
		{
			return A.HashKey < B.HashKey;
		});
		
		ParallelFor(MaxBoidCount, [&](int32 i)
		{
			const int32 OriginIndex = HashPairs[i].Index;
			ReadBuffer.CopyDataFrom(i, WriteBuffer, OriginIndex);
		});
	}

	// 모든 보이드들을 추적할 수 있게 값을 저장. 해쉬를 기준으로 정렬이 됐으므로 최초의 StartIndex에 기록, 갯수를 기입한다.
	{
		CellStartIndex.Init(-1, GridHashHelper.GetHashSize());
		CellBoidCount.Init(0, GridHashHelper.GetHashSize());

		for (int32 i = 0; i < MaxBoidCount; ++i)
		{
			const int32 HashKey = GridHashHelper.GetHashKeyFromLocation(ReadBuffer.Locations[i]);

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
	Swap(ReadBuffer, WriteBuffer);
	
	CellStartIndex.Init(-1, GridHashHelper.GetHashSize());
	BoidNextIndex.Init(-1, MaxBoidCount);

	for (int32 i = 0; i < MaxBoidCount; ++i)
	{
		const int32 HashKey = GridHashHelper.GetHashKeyFromLocation(ReadBuffer.Locations[i]);

		// 고정된 배열의 크기에서 같은 Hash를 가지는 Boid Index 값을 저장 - 배열 기반의 LinkedList
		BoidNextIndex[i] = CellStartIndex[HashKey];
		CellStartIndex[HashKey] = i;
	}
#endif
}

void FBoidSpatialContext::ShowDebugGrid(const UWorld* World, int32 ThresholdPct)
{
#if USE_CACHE_OPTIMIZED_LOGIC
	check(IsValid(World));
	
	FBox BoidBounds;
	for (int32 i = 0; i < MaxBoidCount; ++i)
	{
		BoidBounds += FVector(ReadBuffer.Locations[i]);
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
				const int32 GridHashKey = GridHashHelper.GetHashKey(GridIndex);
				check(CellBoidCount.IsValidIndex(GridHashKey));
				const int32 NumBoidsInHash = CellBoidCount[GridHashKey];
				
				const FVector CellCenter = FVector(GridIndex) * GridCellSize + FVector(GridCellSize * 0.5f);
				DrawDebugBox(World, CellCenter, FVector(GridCellSize * 0.5f), FColor::Black, false, -1.0f, 0, 2.0f); // 외곽선
				
				if (NumBoidsInHash > AbsoluteDensityThreshold)
				{
					// 해시 키를 이용하여 황금값을 위한 Hue 값 게산
					const float Hue = FMath::Fmod(GridHashKey * 137.508f, 360.0f);
					FLinearColor HashLinearColor(Hue, 0.9f, 0.9f, 1.0f);
					FColor HeatColor = HashLinearColor.HSVToLinearRGB().ToFColor(true).WithAlpha(75);
					
					DrawDebugSolidBox(World, CellCenter, FVector(GridCellSize * 0.5f), HeatColor, false, -1.0f, 0);
				}
			}
		}
	}
	
	UE_LOG(LogBeli, Verbose, TEXT("Grid DebugDraw Count - %d"), DrawCount);
#endif
}
