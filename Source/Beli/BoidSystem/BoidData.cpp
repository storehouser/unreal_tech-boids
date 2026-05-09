
#include "BoidData.h"


void FBoidBuffer::SetNumUninitialized(int32 MaxBoidCount)
{
	BoidIDs.SetNumUninitialized(MaxBoidCount);
	Locations.SetNumUninitialized(MaxBoidCount);
	Rotations.SetNumUninitialized(MaxBoidCount);
	Velocities.SetNumUninitialized(MaxBoidCount);
	
	NumBufferSize = MaxBoidCount;
}

void FBoidBuffer::Reserve(int32 MaxBoidCount)
{
	BoidIDs.Reserve(MaxBoidCount);
	Locations.Reserve(MaxBoidCount);
	Rotations.Reserve(MaxBoidCount);
	Velocities.Reserve(MaxBoidCount);
}

void FBoidBuffer::Add(int32 InID, const FVector3f& InLocation, const FRotator3f& InRotation, const FVector3f& InVelocity)
{
	BoidIDs.Add(InID);
	Locations.Add(InLocation);
	Rotations.Add(InRotation);
	Velocities.Add(InVelocity);
	
	++NumBufferSize;
}

void FBoidBuffer::SetBoidData(int32 InIndex, const FVector3f& InLocation, const FRotator3f& InRotation, const FVector3f& InVelocity)
{
	check(InIndex < NumBufferSize);
	
	Locations[InIndex] = InLocation;
	Rotations[InIndex] = InRotation;
	Velocities[InIndex] = InVelocity;
}
