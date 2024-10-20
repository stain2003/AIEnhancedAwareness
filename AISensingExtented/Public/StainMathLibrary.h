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

static FORCEINLINE FVector GetClosestPointFromLineSegment(const FVector& P, const FVector& LineStart, const FVector& LineEnd)
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

/*Return the shortest line segment connects to two line segment A and B
 *The 1st return FVector is on A, the 2nd FVector is on B
 */
static FORCEINLINE std::tuple<FVector, FVector> GetShortestLineSegBetweenTwoLineSeg(const FVector& EdgeAStart, const FVector& EdgeAEnd, const FVector& EdgeBStart, const FVector& EdgeBEnd)
{
	const float LengthFromAStart = (GetClosestPointFromLineSegment(EdgeAStart, EdgeBStart, EdgeBEnd) - EdgeAStart).Length();
	const float LengthFromAEnd = (GetClosestPointFromLineSegment(EdgeAEnd, EdgeBStart, EdgeBEnd) - EdgeAEnd).Length();
	const float LengthFromBStart = (GetClosestPointFromLineSegment(EdgeBStart, EdgeAStart, EdgeAEnd) - EdgeBStart).Length();
	const float LengthFromBEnd = (GetClosestPointFromLineSegment(EdgeBEnd, EdgeAStart, EdgeAEnd) - EdgeBEnd).Length();

	const float minDist = FMath::Min(FMath::Min(LengthFromAStart, LengthFromAEnd), FMath::Min(LengthFromBStart, LengthFromBEnd));

	if (minDist == LengthFromAStart)
	{
		return std::make_tuple(EdgeAStart, GetClosestPointFromLineSegment(EdgeAStart, EdgeBStart, EdgeBEnd));
	}
	if (minDist == LengthFromAEnd)
	{
		return std::make_tuple(EdgeAEnd, GetClosestPointFromLineSegment(EdgeAEnd, EdgeBStart, EdgeBEnd));
	}
	if (minDist == LengthFromBStart)
	{
		return std::make_tuple(GetClosestPointFromLineSegment(EdgeBStart, EdgeAStart, EdgeAEnd), EdgeBStart);
	}
	if (minDist == LengthFromBEnd)
	{
		return std::make_tuple(GetClosestPointFromLineSegment(EdgeBEnd, EdgeAStart, EdgeAEnd), EdgeBEnd);
	}

	return std::make_tuple(FVector::ZeroVector, FVector::ZeroVector);
}