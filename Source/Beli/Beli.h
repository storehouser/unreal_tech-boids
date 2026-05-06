// Copyright Epic Games, Inc. All Rights Reserved.

#pragma once

#include "CoreMinimal.h"

/** Main log category used across the project */
DECLARE_LOG_CATEGORY_EXTERN(LogBeli, Log, All);

/** 프로파일링을 위한 Stat Group 선언 */
DECLARE_STATS_GROUP(TEXT("Boids Simulation"), STATGROUP_Boids, STATCAT_Advanced);
