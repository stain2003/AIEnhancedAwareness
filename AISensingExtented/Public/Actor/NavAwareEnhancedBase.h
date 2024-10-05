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
	Edge,
};

USTRUCT(BlueprintType)
struct FNavPoint
{
	GENERATED_BODY()
	
	UPROPERTY(BlueprintReadWrite, Category="Navigation")
	FVector Start;
	
	UPROPERTY(BlueprintReadWrite, Category="Navigation")
	FVector End;
	
	UPROPERTY(BlueprintReadWrite, Category="Navigation")
	uint8 EdgeID;
	
	UPROPERTY(BlueprintReadWrite, Category="Navigation")
	uint8 LineID;

	UPROPERTY(BlueprintReadWrite, Category="Navigation")
	EWallType Type;
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
	UPROPERTY(VisibleInstanceOnly, BlueprintReadOnly, Category= "TerranInfo")
	TArray<FNavPoint> WallEdges;

	/*
	 * Find edges around
	 */
	UFUNCTION(BlueprintCallable)
	void FindWall(bool bDebug = false, float radius = 550.f);

	UPROPERTY(EditAnywhere, Category= "TerranInfo")
	bool bShowLog = false;
	
public:
private:
	void GatherEdgesWithSorting(TArray<FNavigationWallEdge>& InArray, TArray<FNavPoint>& OutArray, bool bDebug = false) const;
	void DefineCorner() const;
};
