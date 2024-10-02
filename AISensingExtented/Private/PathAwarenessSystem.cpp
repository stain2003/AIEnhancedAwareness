// #include "PathAwareness/PathAwarenessSystem.h"
// #include "NavigationSystem.h"
// #include "AI/NavigationSystemBase.h"
// #include "NavMesh/RecastNavMesh.h"

/*
TArray<UNavigationPath*> APathAwarenessSystem::FindPath(FVector Origin, float Radius)
{
	UNavigationSystemV1* NavSystem = FNavigationSystem::GetCurrent<UNavigationSystemV1>(GetWorld());
	if (NavSystem)
	{
		ARecastNavMesh* RecastNavMesh = Cast<ARecastNavMesh>(NavSystem->GetDefaultNavDataInstance());
		NavNodeRef NodeRef = RecastNavMesh->FindNearestPoly(GetActorLocation(), FVector(500.f, 500.f, 500.f));
		RecastNavMesh->FindEdges(NodeRef, GetActorLocation(), 500.f, NavSystem->CreateDefaultQueryFilterCopy(), OutEdges);
	}
	if (GEngine)
	{
		for (int32 i = 0; i <= OutEdges.Num(); i++)
        {
        	DrawDebugBox(GetWorld(), OutEdges[i].Start, FVector(20.f, 20.f, 80.f), FColor::Red);
        }
	}
	return TArray<UNavigationPath*>();
}
*/
