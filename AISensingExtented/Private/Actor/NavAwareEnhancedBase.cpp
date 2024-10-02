﻿#include "Actor/NavAwareEnhancedBase.h"

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
	
	// TMap<FVector*, FVector*> EdgesMap2;
	// for (const auto& [x, y] : InArray)
	// {
	// 	EdgesMap2.Emplace(x, y);
	// }


	
	/*algorithm to transfer points from TMap to OutArray, sorted by these orders:
	 * the points on the same line has the same LineID
	 * the points on the same line must connect with heads to tails(Start and End)
	 */
	//Take the first in the array as the start point, finding the corresponding Start with its End.
	//Insert the first element as the beginning
	OutArray.Add(FNavPoint(InArray[0].Start, InArray[0].End, 1));
	//Remove transfered elem from maps, because it is not needed anymore
	EdgesMap.Remove(InArray[0].Start);
	
	/*The main part of the algorithm, loop until map is empty, which means the transfer is completed
	 */
	int32 CurrentLineID = -1;
	//LineHeader: supposed to be the head of the current line
	FNavPoint LineHeader = OutArray[0];
	//Connector: supposed to be the tail of the current line
	FNavPoint Connector = LineHeader;
	//CurrentLineEntry: supposed to be the entry index of the current line
	int32 CurrentLineEntry = 0;
	while(EdgesMap.Num() != 0)
	{
		//declare the top in the array is the "Connector"
		//Check if the next line comes in;
		LineHeader = OutArray[CurrentLineEntry];
		if (CurrentLineID != Connector.LineID)
		{
			//Update
			LineHeader = Connector;
			CurrentLineID = Connector.LineID;
			CurrentLineEntry = OutArray.Num() - 1;
			//UE_LOG(LogTemp, Warning, TEXT("Updating info...\nCurrentLineEntry: %d"), CurrentLineEntry)
		}
		
		UE_LOG(LogTemp, Warning, TEXT("CurrentLineEntry: %d, Head: [%f, %f, %f], tail: [%f, %f, %f], adding next element......"), CurrentLineEntry,
		LineHeader.Start.X, LineHeader.Start.Y, LineHeader.Start.Z,
		Connector.End.X, Connector.End.Y, Connector.End.Z)
		
		//find edges connected by connector's tail if there is one
		if (const FVector* Value = EdgesMap.Find(Connector.End))
		{
			OutArray.Push(FNavPoint(Connector.End, *Value, CurrentLineID));
			EdgesMap.Remove(Connector.End);
			Connector = OutArray.Last();
			// UE_LOG(LogTemp, Warning, TEXT("adding tail: [%f, %f, %f], [%f, %f, %f]"),
			// 	Connector.End.X, Connector.End.Y, Connector.End.Z,
			// 	Value->X, Value->Y, Value->Z)
			continue;
		}
		//find edges connected by header's head if there is one
		if (const FVector* Key = EdgesMap.FindKey(LineHeader.Start))
		{
			OutArray.Insert(FNavPoint(*Key, LineHeader.Start, CurrentLineID), CurrentLineEntry);
			EdgesMap.Remove(*Key);
			// UE_LOG(LogTemp, Warning, TEXT("adding head: [%f, %f, %f], [%f, %f, %f]"),
			// 	Key->X, Key->Y, Key->Z,
			// 	LineHeader.Start.X, LineHeader.Start.Y, LineHeader.Start.Z)
			continue;
		}
		//if no connected edges found for both LineHeader and Connector, starts the new line: push in a new value from map
		
		auto it = EdgesMap.CreateIterator();
		OutArray.Push(FNavPoint(it.Key(), it.Value(), CurrentLineID + 1));
		// UE_LOG(LogTemp, Warning, TEXT("starting new line: [%f, %f, %f], tail: [%f, %f, %f]"),
		// 	it.Key().X, it.Key().Y, it.Key().Z,
		// 	it.Value().X, it.Value().Y, OutArray.Top().End.Z)
		Connector = OutArray.Last();
		EdgesMap.Remove(it.Key());
	}
	
	/*Debugger*/
	if (bDebug)
	{
		for (auto& [Start, End, LineID] : OutArray)
        {
        	
        	UE_LOG(LogTemp, Warning,
        		TEXT("[Start: [%f, %f, %f], End: [%f, %f, %f], LineID: %d]"),
        		Start.X, Start.Y, Start.Z, End.X, End.Y, End.Z, LineID)

			DrawDebugDirectionalArrow(GetWorld(), Start, End, 1.f, FColor::MakeRedToGreenColorFromScalar(LineID * 0.15f), false, 1.f);
        }
	}
}

