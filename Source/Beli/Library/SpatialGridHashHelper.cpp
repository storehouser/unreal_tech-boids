
#include "SpatialGridHashHelper.h"


FSpatialGridHashHelper::FSpatialGridHashHelper(float InCellSize, int32 TotalNums)
{
	CellSize = InCellSize;
	
	// 50000만개 이상은 테스트를 좀 해봐야 한다.
	check(TotalNums < 50000);
	
	// 해쉬 충돌을 피하기 위해 실제 데이터 수보다 크게 잡고, 빠른 키 값 연산을 위해 Mask값을 설정해 놓는다.
	HashTableSize = FMath::RoundUpToPowerOfTwo(TotalNums * 2);
	HashMask = HashTableSize - 1;
}

FSpatialGrid FSpatialGridHashHelper::GetGridIndex(const FVector3f& Location) const
{
	return FSpatialGrid(FMath::FloorToInt(Location.X / CellSize), FMath::FloorToInt(Location.Y / CellSize), FMath::FloorToInt(Location.Z / CellSize));
}

uint32 FSpatialGridHashHelper::GetHashKey(const FSpatialGrid& Grid) const
{
	constexpr int32 Prime1 = 73856093;
	constexpr int32 Prime2 = 19349663;
	constexpr int32 Prime3 = 83492791;
	
	const int32 Hash = (Grid.X * Prime1) ^ (Grid.Y & Prime2) ^ (Grid.Z * Prime3);
	return StaticCast<uint32>(Hash) & HashMask;
}

uint32 FSpatialGridHashHelper::GetHashKeyFromLocation(const FVector3f& Location) const
{
	return GetHashKey(GetGridIndex(Location));
}
