#pragma once

#include "CoreMinimal.h"

/** 공간 상의 위치 값과 공간에 나뉘어진 그리드의 기반 위치 값에 혼용을 줄이기 위해 별칭 선언. */
using FSpatialGrid = FIntVector;


/**
 * Cell의 크기와 Hash이 필요한 Object들의 갯수를 미리 지정해 놓고 필요할 때 변환을 처리.
 */
struct FSpatialGridHashHelper
{
public:
	FSpatialGridHashHelper(float InCellSize, int32 TotalNums);
	
	FSpatialGrid GetGridIndex(const FVector& Location) const;
	int32 GetHashKey(const FSpatialGrid& Grid) const;
	int32 GetHashKeyFromLocation(const FVector& Location) const;
	
	int32 GetHashSize() const { return HashTableSize; }
	
private:
	float CellSize;
	int32 HashTableSize;	
};
