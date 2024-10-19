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

static FORCEINLINE FVector GetClosestPointFromLineSegment(FVector& P, FVector& LineStart, FVector& LineEnd)
{
	const float x1 = LineStart.X;
	const float x2 = LineEnd.X;
	const float y1 = LineStart.Y;
	const float y2 = LineEnd.Y;
	const float z1 = LineStart.Z;
	const float z2 = LineEnd.Z;
	const FVector LineSegment = LineEnd - LineStart;
	const FVector PointToLine = P - LineStart;

	const float LSquared = LineSegment.SquaredLength();
	if (LSquared == 0)
	{
		return LineStart;
	}
	
	const float Dot = FVector::DotProduct(LineSegment, PointToLine);
	const float t = Dot/LSquared;

	if (t < 0)
	{
		return LineStart;
	}
	if (t > 1)
	{
		return LineEnd;
	}
	
	return FVector(x1 + t * (x2 - x1), y1 + t * (y2 - y1), z1 + t * (z2 - z1));
}