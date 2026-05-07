
#include "SpatialGridHashHelper.h"


FSpatialGridHashHelper::FSpatialGridHashHelper(float InCellSize, int32 TotalNums)
{
	CellSize = InCellSize;
	
	// 해쉬 충돌을 피하기 위해 실제 데이터 수보다 크게 잡는다.
	HashTableSize = TotalNums * 2;
}

FSpatialGrid FSpatialGridHashHelper::GetGridIndex(const FVector3f& Location) const
{
	return FSpatialGrid(FMath::FloorToInt(Location.X / CellSize), FMath::FloorToInt(Location.Y / CellSize), FMath::FloorToInt(Location.Z / CellSize));
}

int32 FSpatialGridHashHelper::GetHashKey(const FSpatialGrid& Grid) const
{
	constexpr int32 Prime1 = 73856093;
	constexpr int32 Prime2 = 19349663;
	constexpr int32 Prime3 = 83492791;
	
	const int32 Hash = (Grid.X * Prime1) ^ (Grid.Y & Prime2) ^ (Grid.Z * Prime3);
	return FMath::Abs(Hash) % HashTableSize;
}

int32 FSpatialGridHashHelper::GetHashKeyFromLocation(const FVector3f& Location) const
{
	return GetHashKey(GetGridIndex(Location));
}
