#include "Actor/NavAwareEnhancedBase.h"

#include "NavigationSystem.h"
#include "AI/NavigationSystemBase.h"
#include "Evaluation/Blending/MovieSceneBlendType.h"
#include "NavMesh/RecastNavMesh.h"

ANavAwareEnhancedBase::ANavAwareEnhancedBase()
{
	PrimaryActorTick.bCanEverTick = true;
}

void ANavAwareEnhancedBase::BeginPlay()
{
	Super::BeginPlay();
}

void ANavAwareEnhancedBase::Tick(float DeltaTime)
{
	Super::Tick(DeltaTime);
}

void ANavAwareEnhancedBase::FindWall(bool bDebug, float radius)
{
	if (const UNavigationSystemV1* NavSystem = FNavigationSystem::GetCurrent<UNavigationSystemV1>(GetWorld()))
	{
		const ARecastNavMesh* RecastNavMesh = Cast<ARecastNavMesh>(NavSystem->GetDefaultNavDataInstance());
		const NavNodeRef NodeRef = RecastNavMesh->FindNearestPoly(GetActorLocation(), FVector(500.f, 500.f, 500.f));
		TArray<FNavigationWallEdge> GetEdges;
		
		RecastNavMesh->FindEdges(NodeRef, GetActorLocation(), radius, NavSystem->CreateDefaultQueryFilterCopy(), GetEdges);
		
		GatherEdgesWithSorting(GetEdges, WallEdges, bDebug);
	}
	
	if (bDebug)
	{
		for (int32 i = 0; i < WallEdges.Num(); i++)
		{
			FVector start = WallEdges[i].Start;
			FVector end = WallEdges[i].End;
			DrawDebugBox(GetWorld(), start, FVector(20.f, 20.f, 80.f), FColor::Red, false, 1.1f);
		}
	}
}

void ANavAwareEnhancedBase::GatherEdgesWithSorting(TArray<FNavigationWallEdge>& InArray, TArray<FNavPoint>& OutArray, bool bDebug)
{
	if(InArray.Num() == 0)
	{
		return;
	}

	//Clear target output array before adding items
	OutArray.Empty();
	//Transfer points from InArray into a TMap
	TMap<FVector, FVector> EdgesMap;
	for (const auto& [x, y] : InArray)
	{
		EdgesMap.Emplace(x, y);
	}


	
	/*algorithm to transfer points from TMap to OutArray, sorted by these orders:
	 * the points on the same line has the same LineID
	 * the points on the same line must connect with heads to tails(Start and End)
	 */
	//CurrentLineEntry: supposed to be the entry index of the current line
	int32 CurrentLineEntry = 0;
	//LineID: supposed to be the id of the current line
	int32 CurrentLineID = -1;
	//Take the first in the array as the start point, finding the corresponding Start with its End.
	//Insert the first element as the beginning, if its single, insert it as lineID '0'
	if (EdgesMap.Find(InArray[0].End) != nullptr || EdgesMap.FindKey(InArray[0].Start) != nullptr)
	{
		CurrentLineID = 1;
		OutArray.Add(FNavPoint(InArray[0].Start, InArray[0].End, CurrentLineID));
	}
	else
	{
		CurrentLineID = 0;
		OutArray.Add(FNavPoint(InArray[0].Start, InArray[0].End, CurrentLineID));
	}
	UE_LOG(LogTemp, Warning, TEXT("adding first line: [Start: [%.0f, %.0f] End: [%.0f, %.0f]] with lineID: %d"),
		InArray[0].Start.X, InArray[0].Start.Y, InArray[0].End.X, InArray[0].End.Y, CurrentLineID)
	//Remove transferred elem from maps, because it is not needed anymore
	EdgesMap.Remove(InArray[0].Start);
	//LineHeader: supposed to be the head of the current line
	FNavPoint LineHeader = OutArray[0];
	//Connector: supposed to be the tail of the current line
	FNavPoint Connector = LineHeader;
	
	/*The main part of the algorithm, loop until map is empty, which means the transfer is completed
	 */
	while(EdgesMap.Num() != 0)
	{
		//LineHeader = OutArray[CurrentLineEntry];
		//CurrentLineID = OutArray.Last().LineID;
		/*UE_LOG(LogTemp, Warning, TEXT("CurrentLineEntry: %d, Head: [%.0f, %.0f], tail: [%.0f, %.0f], adding next element......"), CurrentLineEntry,
		LineHeader.Start.X, LineHeader.Start.Y, LineHeader.Start.Z,
		Connector.End.X, Connector.End.Y, Connector.End.Z)*/
		
		//Find the edge connected by connector if there is one
		if (const FVector* Value = EdgesMap.Find(Connector.End))
		{
			OutArray.Push(FNavPoint(Connector.End, *Value, CurrentLineID));
			UE_LOG(LogTemp, Warning, TEXT("adding tail: [%.0f, %.0f], [%.0f, %.0f]"),
				Connector.End.X, Connector.End.Y,
				Value->X, Value->Y)
			EdgesMap.Remove(Connector.End);
			Connector = OutArray.Last();
			continue;
		}
		//Find the edge connected by header if there is one
		if (const FVector* Key = EdgesMap.FindKey(LineHeader.Start))
		{
			OutArray.Insert(FNavPoint(*Key, LineHeader.Start, CurrentLineID), CurrentLineEntry);
			UE_LOG(LogTemp, Warning, TEXT("adding head: [%.0f, %.0f], [%.0f, %.0f]"),
				Key->X, Key->Y,
				LineHeader.Start.X, LineHeader.Start.Y)
			EdgesMap.Remove(*Key);
			continue;
		}
		
		//Starts the new line: push in a new value from map if no connected edges found for both LineHeader and Connector,
		const auto it = EdgesMap.CreateIterator();
		//check the incoming line as if a single line or not, if not, add its found head or tail along into the map
		if (const FVector* TailValue = EdgesMap.Find(it.Value()))
		{
			//if it has tail
			CurrentLineID += 1;
			OutArray.Push(FNavPoint(it.Key(), it.Value(), CurrentLineID));
			EdgesMap.Remove(it.Value());
			LineHeader = OutArray.Last();
			CurrentLineEntry = OutArray.Num() - 1;
			OutArray.Push(FNavPoint(it.Value(), *TailValue, CurrentLineID));
			Connector = OutArray.Last();
			UE_LOG(LogTemp, Warning, TEXT("adding new line: first: [Start: [%.0f, %.0f], End: [%.0f, %.0f] Second: [Start: [%.0f, %.0f], End: [%.0f, %.0f]")
				,it.Key().X, it.Key().Y, it.Value().X, it.Value().Y, it.Value().X, it.Value().Y, TailValue->X, TailValue->Y)
		}
		else if (const FVector* HeadKey = EdgesMap.FindKey(it.Key()))
		{
			//if it has head
			CurrentLineID += 1;
			OutArray.Push(FNavPoint(*HeadKey, it.Key(), CurrentLineID));
			EdgesMap.Remove(*HeadKey);
			LineHeader = OutArray.Last();
			CurrentLineEntry = OutArray.Num() - 1;
			OutArray.Push(FNavPoint(it.Key(), it.Value(), CurrentLineID));
			Connector = OutArray.Last();
			UE_LOG(LogTemp, Warning, TEXT("adding new line: first: [Start: [%.0f, %.0f], End: [%.0f, %.0f] Second: [Start: [%.0f, %.0f], End: [%.0f, %.0f]"),
				HeadKey->X, HeadKey->Y, it.Key().X, it.Key().Y, it.Key().X, it.Key().Y, it.Value().X, it.Value().Y)
		}
		else
		{
			//if it is single, insert it to the top
			OutArray.Insert(FNavPoint(it.Key(), it.Value(), 0), 0);
			UE_LOG(LogTemp, Warning, TEXT("adding single line: [Start: [%.0f, %.0f], End: [%.0f, %.0f]"),
				it.Key().X, it.Key().Y, it.Value().X, it.Value().Y)
			CurrentLineEntry += 1;
		}
		EdgesMap.Remove(it.Key());
		//CurrentLineID = NewPoint.LineID;
		//CurrentLineEntry = OutArray.Num() - 1;
		/*UE_LOG(LogTemp, Warning, TEXT("starting new line: [%.0f, %.0f], tail: [%.0f, %.0f]"),
			it.Key().X, it.Key().Y, it.Key().Z,
			it.Value().X, it.Value().Y, OutArray.Top().End.Z)*/
	}
	
	/*Debugger*/
	if (bDebug)
	{
		for (auto& [Start, End, LineID] : OutArray)
        {
        	
        	UE_LOG(LogTemp, Warning,
        		TEXT("[Start: [%.0f, %.0f], End: [%.0f, %.0f], LineID: %d]"),
        		Start.X, Start.Y, End.X, End.Y, LineID)

			DrawDebugDirectionalArrow(GetWorld(), Start, End, 1.f, FColor::MakeRedToGreenColorFromScalar(LineID * 0.15f), false, 1.f);
        }
	}
}

