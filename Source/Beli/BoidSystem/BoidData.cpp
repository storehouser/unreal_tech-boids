#include "BoidData.h"

#include "NiagaraRibbonRendererProperties.h"

void FBoidBuffer::Reserve(int32 MaxBoidCount)
{
	BoidIDs.Reserve(MaxBoidCount);
	Locations.Reserve(MaxBoidCount);
	Rotations.Reserve(MaxBoidCount);
	Velocities.Reserve(MaxBoidCount);
}

void FBoidBuffer::Add(int32 InID, FVector3f InLocation, FRotator3f InRotation, FVector3f InVelocity)
{
	BoidIDs.Add(InID);
	Locations.Add(InLocation);
	Rotations.Add(InRotation);
	Velocities.Add(InVelocity);
	
	++NumBufferSize;
}

void FBoidBuffer::SetData(int32 InIndex, int32 InID, FVector3f InLocation, FRotator3f InRotation, FVector3f InVelocity)
{
	check(InIndex < NumBufferSize);
	
	BoidIDs[InIndex] = InID;
	Locations[InIndex] = InLocation;
	Rotations[InIndex] = InRotation;
	Velocities[InIndex] = InVelocity;
}
