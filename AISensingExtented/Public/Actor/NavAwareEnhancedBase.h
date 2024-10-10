// Fill out your copyright notice in the Description page of Project Settings.

#pragma once

#include "CoreMinimal.h"
#include "GameFramework/Actor.h"
#include "NavMesh/RecastNavMesh.h"

#include "NavAwareEnhancedBase.generated.h"

DECLARE_LOG_CATEGORY_EXTERN(NavAware, Log, All);

UENUM(BlueprintType)
enum class EWallType : uint8
{
	Wall,
	Corner,
	Entry,
	FakeCorner,
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
	/*Array that used to be store nearby edges*/
	UPROPERTY(VisibleInstanceOnly, BlueprintReadOnly, Category= "TerranInfo")
	TArray<FNavPoint> WallEdges;
	/*Prints out elements from stored array with info, only works when bDebug is ture*/
	UPROPERTY(EditAnywhere, Category= "TerranInfo")
	bool bShowLog = false;
	/**/
	UPROPERTY(EditAnywhere, Category= "TerranInfo|Edge Detection")
	uint8 minCount = 0;
	/*Min distance between two corner that can be marked as fake*/
	UPROPERTY(EditAnywhere, Category= "TerranInfo|Edge Detection")
	float minFakeCornerDist = 50.f;
	/*Min degree required for a point to be recognized as a corner*/
	UPROPERTY(EditAnywhere, Category= "TerranInfo|Edge Detection")
	float minCurDeg = 35.f;
	/*Min compensation of current and last degree under which will be recognize the current and the last point is not a corner*/
	UPROPERTY(EditAnywhere, Category= "TerranInfo|Edge Detection")
	float minCompens = 45.f;

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
	 * Caller function to add corner & wall and so on information for an TArray<FNavPoint>;
	 */
	void MarkCorner(TArray<FNavPoint>& InOutArray);
	FCriticalSection MarkingCornerSection;
	
	/*
	 * Takes into two edges: current & next, and calculate current edge's degree.
	 * In the meantime check if it is fake: use compensation of the 'last' edge's degree and 'current' edge's
	 */
	void DetectCorner(TArray<FNavPoint>& InOutArray, FNavPoint& CurEdge, FNavPoint& nxtEdge, float& curDeg, float& lastDeg, bool& bisEdging, uint8 i) const;
	
	bool CheckCorner(const float& curDeg) const;
	
	template <typename T>
	bool CheckFakeCorner(T& curDeg, T& lastDeg) const;
};
