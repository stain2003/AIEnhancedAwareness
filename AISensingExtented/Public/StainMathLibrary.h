#pragma once

#include "Math/UnrealMathUtility.h"

/*Calculate degrees with sign of 2 vectors on XY plane*/
static FORCEINLINE float XYDegrees(FVector const& A, FVector const& B)
{
	FVector ANormal = A.GetSafeNormal();
	FVector BNormal = B.GetSafeNormal();
	return FMath::RadiansToDegrees(
		FMath::Acos(FVector::DotProduct(FVector(ANormal.X, ANormal.Y, 0.f), FVector(BNormal.X, BNormal.Y, 0.f)))) * FMath::Sign(FVector::CrossProduct(A, B).Z);
}