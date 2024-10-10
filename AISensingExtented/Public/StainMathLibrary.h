#pragma once

#include "Math/UnrealMathUtility.h"

/*Calculate degrees with sign of 2 vectors on XY plane*/
static constexpr FORCEINLINE float XYDegrees(FVector const& A, FVector const& B)
{
	return FMath::RadiansToDegrees(
		FMath::Acos(FVector::DotProduct(FVector(A.X, A.Y, 0.f), FVector(B.X, B.Y, 0.f)))) * FMath::Sign(FVector::CrossProduct(A, B).Z);
}


static constexpr FORCEINLINE double XYDegrees(double const& RadVal)
{
	return RadVal * (180.0 / UE_DOUBLE_PI);
}