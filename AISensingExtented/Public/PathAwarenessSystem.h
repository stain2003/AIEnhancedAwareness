// #pragma once
//
// #include "NavigationPath.h"
//
// #include "PathAwarenessSystem.generated.h"
//
// struct FNavigationWallEdge;
//
// UCLASS()
// class APathAwarenessSystem : public AActor
// {
// public:
// 	GENERATED_BODY()
// 	
// 	virtual void Tick(float DeltaSeconds) override;
// 	
// protected:
// 	APathAwarenessSystem();
// 	TArray<FNavigationWallEdge> OutEdges;
// 	NavNodeRef CenterNodeRef;
// 	
// 	UFUNCTION(BlueprintCallable)
// 	TArray<UNavigationPath*> FindPath(FVector Origin, float Radius);
// };
