﻿// Fill out your copyright notice in the Description page of Project Settings.

#pragma once

#include "CoreMinimal.h"
#include "GameFramework/Actor.h"
#include "NavMesh/RecastNavMesh.h"

#include "NavAwareEnhancedBase.generated.h"

DECLARE_LOG_CATEGORY_EXTERN(NavAware, Log, All);

class ARecastNavMesh;

UENUM(BlueprintType)
enum class EWallType : uint8
{
    Wall,
    FakeCorner,
    Corner,
    Entry,
};

UENUM(BlueprintType)
enum class ECornerCheck : uint8
{
    None,
    PrevIsCorner,
    NextIsCorner,
    BothAreCorner,
};


USTRUCT(BlueprintType)
struct FNavPoint
{
	GENERATED_BODY()
	
	UPROPERTY(BlueprintReadWrite, Category="Navigation")
	FVector Start = FVector::ZeroVector;
	
	UPROPERTY(BlueprintReadWrite, Category="Navigation")
	FVector End = FVector::ZeroVector;;
	
	UPROPERTY(BlueprintReadWrite, Category="Navigation")
	uint8 EdgeID = 0;
	
	UPROPERTY(BlueprintReadWrite, Category="Navigation")
	uint8 LineID = 0;

	UPROPERTY(BlueprintReadWrite, Category="Navigation")
	EWallType Type = EWallType::Wall;

	UPROPERTY(BlueprintReadWrite, Category="Navigation")
	float Degree = 0;

	FNavPoint* PrevEdge;
	FNavPoint* NextEdge;

	FORCEINLINE FNavPoint(const FVector& InStart = FVector::ZeroVector, const FVector& InEnd = FVector::ZeroVector,
		uint8 InEdgeID = 0, uint8 InLineID = 0, EWallType InType = EWallType::Wall, float InDegree = 0.f, FNavPoint* InPrevEdge = nullptr, FNavPoint* InNextEdge = nullptr)
		: Start(InStart), End(InEnd), EdgeID(InEdgeID), LineID(InLineID), Type(InType), Degree(InDegree), PrevEdge(InPrevEdge), NextEdge(InNextEdge)
	{
	}
};

UCLASS()
class AISENSINGEXTENTED_API ANavAwareEnhancedBase : public AActor
{
	GENERATED_BODY()

public:
	ANavAwareEnhancedBase();

protected:
	virtual void BeginPlay() override;

public:
	virtual void Tick(float DeltaTime) override;
	
/*Navigation*/
protected:
	UPROPERTY()
	UNavigationSystemV1* MainNavSystem;
	
	UPROPERTY()
	ARecastNavMesh* MainRecastNavMesh;
	
	/*Array that used to be store nearby edges*/
	UPROPERTY(VisibleInstanceOnly, BlueprintReadOnly, Category= "TerranInfo")
	TArray<FNavPoint> WallEdges;
	
	/*Prints out elements from stored array with info, only works when bDebug is ture*/
	UPROPERTY(EditAnywhere, Category= "TerranInfo")
	bool bShowLog = false;
	
	/**/
	UPROPERTY(EditAnywhere, Category= "TerranInfo|Edge Detection")
	uint8 minCount = 0;
	
	/*Max distance between two corner that can be marked as fake
	 * larger than which will not be checked for fake corners
	 */
	UPROPERTY(EditAnywhere, Category= "TerranInfo|Edge Detection")
	float maxDistForFakeCorner = 250.f;
	
	/*Min degree required for a point that can be marked as a corner*/
	UPROPERTY(EditAnywhere, Category= "TerranInfo|Edge Detection")
	float minCurDeg = 35.f;
	
	/*Min compensation: added up of two degrees that smaller than this will be marked as fake*/
	UPROPERTY(EditAnywhere, Category= "TerranInfo|Edge Detection")
	float minCompens = 45.f;

	/**/
	UPROPERTY(EditAnywhere, Category= "TerranInfo|Edge Detection")
	float CornerBlur = 100.f;

	/*
	 * Find walls & corners around
	 */
	UFUNCTION(BlueprintCallable)
	void FindWall(bool bDebug = false, float radius = 550.f);
	
public:
private:
	
	/*
	 * Takes in an TArray<FNavigationWallEdge>, sorts element in the order of head & tail, into separate lines.
	 */
	void GatherEdgesWithSorting(TArray<FNavigationWallEdge>& InArray, TArray<FNavPoint>& OutArray, bool bDebug = false);
	FCriticalSection GatherSortingEdgesSection;

	/*
	 * Make array a chain that every edge contains address of their prev and next edge
	 */
	void EdgeLinker(TArray<FNavPoint>& InOutArray);

	/*
	 * Caller function to add corner & wall and so on information for an TArray<FNavPoint>;
	 */
	void MarkCorner(TArray<FNavPoint>& InOutArray);
	FCriticalSection MarkingCornerSection;
	
	/*
	 * Takes into two edges: current & next, and calculate current edge's degree.
	 * In the meantime check if it is fake
	 */
	void DetectCorner(TArray<FNavPoint>& InOutArray, FNavPoint& CurEdge, FNavPoint& NextEdge, FNavPoint& LastEdge, float& curDeg, float& lastDeg, uint8 i) const;
	
	FORCEINLINE bool CheckCorner(const float& curDeg) const
	{
		return curDeg >= minCurDeg || curDeg <= -minCurDeg;
	};

	/*
	 * Check if current corner is fake, when last degree != 0.f,
	 * in another word, this edge is not the first of the current array/line,
	 * use compensation of the 'last' edge's degree and 'current' edge's
	 */
	template <typename T>
	FORCEINLINE bool CheckFakeCorner(T& curDeg, T& lastDeg) const
	{
		const float Compensation = FMath::Abs(static_cast<float>(lastDeg) + static_cast<float>(curDeg));
		return lastDeg != 0.f && Compensation < minCompens;
	}

	/*
	 * Filter out the outer edges from a curves, which won't be needed to calculate the cross road entries
	 */
	void FilterOnlyInnerEdge(TArray<FNavPoint>& InOutArray);
	FCriticalSection FilterSection;
	
	/*Only can be used on edge!
	 * Need to check if return vector if is zero vector!
	 */
	FORCEINLINE FVector GetEdgePolyCenter(const FNavPoint& Edge, NavNodeRef* OutPoly = nullptr) const
	{
		FVector OutVector;
		if (MainRecastNavMesh)
		{
			const NavNodeRef Poly = MainRecastNavMesh->FindNearestPoly((Edge.Start + Edge.End)/2, FVector(50.f, 50.f, 50.f));
			if (OutPoly) *OutPoly = Poly;
		
			MainRecastNavMesh->GetPolyCenter(Poly, OutVector);

			DrawDebugDirectionalArrow(GetWorld(), (Edge.Start + Edge.End)/2, OutVector, 5.f, FColor::Yellow, false, 1.f);
		}
	
		return OutVector;
	}

	/*
	 * Mark road entries
	 */
	void MarkEntry(TArray<FNavPoint>& InOutArray);
	FCriticalSection MarkingEntrySection;

	static FORCEINLINE ECornerCheck CheckNeighborCorner(const FNavPoint& Edge)
	{
		const bool prevIsCorner = Edge.PrevEdge && Edge.PrevEdge->Type == EWallType::Corner;
		const bool nextIsCorner = Edge.NextEdge && Edge.NextEdge->Type == EWallType::Corner;
		
		if (prevIsCorner && nextIsCorner)
		{
			return ECornerCheck::BothAreCorner;
		}
		if (!prevIsCorner && !nextIsCorner)
		{
			return ECornerCheck::None;
		}
		if (prevIsCorner)
		{
			return ECornerCheck::PrevIsCorner;
		}
		if (nextIsCorner)
		{
			return ECornerCheck::NextIsCorner;
		}
		
		return ECornerCheck::None;
	}

	FORCEINLINE float GetEdgeNeighborDist(const FNavPoint& Edge)
	{
		float OutDistance = 0.f;
		
		const FVector CurEdgeMiddlePoint = (Edge.Start + Edge.End)/2;
		
		if (Edge.NextEdge)
		{
			const FVector NextEdgeMiddlePoint = (Edge.NextEdge->Start + Edge.NextEdge->End)/2;
			OutDistance += (CurEdgeMiddlePoint - NextEdgeMiddlePoint).Length();
		}
		
		if (Edge.PrevEdge)
		{
			const FVector PrevEdgeMiddlePoint = (Edge.PrevEdge->Start + Edge.PrevEdge->End)/2;
			OutDistance += (CurEdgeMiddlePoint - PrevEdgeMiddlePoint).Length();
		}
		
		return OutDistance;
	}
};
