﻿#include "Actor/NavAwareEnhancedBase.h"

#include "NavigationSystem.h"
#include "AI/NavigationSystemBase.h"
#include "NavMesh/RecastNavMesh.h"

DEFINE_LOG_CATEGORY(NavAware);


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
		for (const auto edge : WallEdges)
		{
			FVector lstart = edge.Start;
			FVector lend = edge.End;
			int lID = edge.LineID;
			DrawDebugBox(GetWorld(), lend, FVector(10.f, 10.f, 20.f), FColor::Red, false, 1.1f);
			DrawDebugDirectionalArrow(GetWorld(), lstart, lend, 1.f, FColor::MakeRedToGreenColorFromScalar(lID * 0.15f), false, 1.f);
		}
	}
}

void ANavAwareEnhancedBase::GatherEdgesWithSorting(TArray<FNavigationWallEdge>& InArray, TArray<FNavPoint>& OutArray, bool bDebug) const
{
	OutArray.Empty();
	
	if(InArray.Num() == 0)
	{
		return;
	}

	//Transfer points from InArray into a TMap
	TMap<FVector, FVector> EdgesMap;
	for (const auto& [x, y] : InArray)
	{
		EdgesMap.Emplace(x, y);
	}


	
	/*algorithm to transfer points from TMap to OutArray, sorted by these orders:
	 * edges on the same line has the same LineID
	 * edges on the same line must connect with heads to tails(Start and End)
	 * single edge will be sent to the top of the array, with LineID '0'
	 */
	
	int32 CurrentLineEntry = 0;	//CurrentLineEntry: supposed to be the entry index of the current line
	uint8 CurrentIndex = 0;		//For marking id
	int32 CurrentLineID = -1;	//LineID: supposed to be the id of the current line
	
	//Take the first in the array as the start point, finding the corresponding Start with its End.
	//Insert the first element as the beginning, if its single, insert it as lineID '0'
	if (EdgesMap.Find(InArray[0].End) != nullptr || EdgesMap.FindKey(InArray[0].Start) != nullptr)
	{
		CurrentIndex++;
		CurrentLineID = 1;	
		OutArray.Add(FNavPoint(InArray[0].Start, InArray[0].End, CurrentIndex, CurrentLineID));
		/*UE_LOG(LogTemp, Warning, TEXT("adding first line[%02d]: [Start: [%.0f, %.0f] End: [%.0f, %.0f]] with lineID: %d"),
			CurrentIndex, InArray[0].Start.X, InArray[0].Start.Y, InArray[0].End.X, InArray[0].End.Y, CurrentLineID)*/
	}
	else
	{
		CurrentLineID = 0;
		OutArray.Add(FNavPoint(InArray[0].Start, InArray[0].End, CurrentIndex, CurrentLineID));
		/*UE_LOG(LogTemp, Warning, TEXT("adding first line(single)[%02d]: [Start: [%.0f, %.0f] End: [%.0f, %.0f]] with lineID: %d"),
			CurrentIndex, InArray[0].Start.X, InArray[0].Start.Y, InArray[0].End.X, InArray[0].End.Y, CurrentLineID)*/
	}
	EdgesMap.Remove(InArray[0].Start);
	
	FNavPoint LineHeader = OutArray[0];	//LineHeader: supposed to be the head of the current line
	FNavPoint Connector = LineHeader;	//Connector: supposed to be the tail of the current line
	
	/*The main part of the algorithm, loop through every element until map is empty, which means the transfer is completed
	 */
	while(EdgesMap.Num() != 0)
	{
		CurrentIndex++;
		//Find the edge connected by connector if there is one(tail)
		if (const FVector* Value = EdgesMap.Find(Connector.End))
		{
			OutArray.Push(FNavPoint(Connector.End, *Value, CurrentIndex, CurrentLineID));
			EdgesMap.Remove(Connector.End);
			Connector = OutArray.Last();
			
			/*UE_LOG(LogTemp, Warning, TEXT("adding tail[%02d]: [%.0f, %.0f], [%.0f, %.0f]"),
				CurrentIndex, Connector.End.X, Connector.End.Y, Value->X, Value->Y)*/
			continue;
		}
		//Find the edge connected by header if there is one(head)
		if (const FVector* Key = EdgesMap.FindKey(LineHeader.Start))
		{
			OutArray.Insert(FNavPoint(*Key, LineHeader.Start, CurrentIndex, CurrentLineID), CurrentLineEntry);
			EdgesMap.Remove(*Key);
			LineHeader = OutArray[CurrentLineEntry];
			
			/*UE_LOG(LogTemp, Warning, TEXT("adding head[%02d]: [%.0f, %.0f], [%.0f, %.0f]"),
				CurrentIndex, Key->X, Key->Y, LineHeader.Start.X, LineHeader.Start.Y)*/
			continue;
		}
		
		//adding a new line: push in a new value from map if no connected edges found for both LineHeader and Connector,
		const auto it = EdgesMap.CreateIterator();
		//check the incoming line as if a single line or not, if not, add its found head or tail along into the map
		if (const FVector* TailValue = EdgesMap.Find(it.Value()))
		{
			//if it has tail, add this first
			CurrentLineID += 1;
			OutArray.Push(FNavPoint(it.Key(), it.Value(), CurrentIndex, CurrentLineID));
			LineHeader = OutArray.Last();
			//then add its tail
			CurrentIndex++;
			OutArray.Push(FNavPoint(it.Value(), *TailValue, CurrentIndex, CurrentLineID));
			Connector = OutArray.Last();
			CurrentLineEntry = OutArray.Num() - 2;
			EdgesMap.Remove(it.Value());
			
			/*UE_LOG(LogTemp, Warning, TEXT("adding new line[%02d]: first: [Start: [%.0f, %.0f], End: [%.0f, %.0f] Second[%d]: [Start: [%.0f, %.0f], End: [%.0f, %.0f]"),
				CurrentIndex - 1, it.Key().X, it.Key().Y, it.Value().X, it.Value().Y, CurrentIndex, it.Value().X, it.Value().Y, TailValue->X, TailValue->Y)*/
		}
		else if (const FVector* HeadKey = EdgesMap.FindKey(it.Key()))
		{
			//if it has head, add its head first
			CurrentLineID += 1;
			OutArray.Push(FNavPoint(*HeadKey, it.Key(), CurrentIndex, CurrentLineID));
			LineHeader = OutArray.Last();
			//then add this
			CurrentIndex++;
			OutArray.Push(FNavPoint(it.Key(), it.Value(), CurrentIndex, CurrentLineID));
			Connector = OutArray.Last();
			CurrentLineEntry = OutArray.Num() - 2;
			EdgesMap.Remove(*HeadKey);
			
			/*UE_LOG(LogTemp, Warning, TEXT("adding new line[%02d]: first: [Start: [%.0f, %.0f], End: [%.0f, %.0f] Second[%d]: [Start: [%.0f, %.0f], End: [%.0f, %.0f]"),
				CurrentIndex - 1, HeadKey->X, HeadKey->Y, it.Key().X, it.Key().Y, CurrentIndex, it.Key().X, it.Key().Y, it.Value().X, it.Value().Y)*/
		}
		else
		{
			//if it is single, insert it to the top
			OutArray.Insert(FNavPoint(it.Key(), it.Value(), CurrentIndex, 0), 0);
			/*UE_LOG(LogTemp, Warning, TEXT("adding single line[%02d]: [Start: [%.0f, %.0f], End: [%.0f, %.0f]"),
				CurrentIndex, it.Key().X, it.Key().Y, it.Value().X, it.Value().Y)*/
			CurrentLineEntry += 1;
		}
		EdgesMap.Remove(it.Key());
	}

	//Define the start index, to skip LineID '0'
	uint8 StartIndex = 0;
	uint8 EndIndex = OutArray.Num() - 1;
	for (auto& currentElem : OutArray)
	{
		if (currentElem.LineID != 0)
		{
			break;
		}
		StartIndex++;
	}

	//Main loop
	FNavPoint curEdge;
	FNavPoint nxtEdge;
	float curDeg = 0.f;
	float lastDeg = 0.f;
	FVector curVect = FVector::ZeroVector;
	FVector nxtVect = FVector::ZeroVector;
	bool isEdging = false;
	
	for(uint8 i = StartIndex; i < EndIndex; i++)
	{
		curEdge = OutArray[i];
		nxtEdge = OutArray[i+1];
		
		if (curEdge.LineID == nxtEdge.LineID)
		{
			curVect = (curEdge.End - curEdge.Start).GetSafeNormal2D();
			nxtVect = (nxtEdge.End - nxtEdge.Start).GetSafeNormal2D();
			curDeg = FMath::RadiansToDegrees(FMath::Acos(FVector::DotProduct(curVect, nxtVect))) * FMath::Sign(FVector::CrossProduct(curVect, nxtVect).Z);
			if (curDeg > minCurDeg)	//if this edge is a corner
			{
				const int Compensation = FMath::Abs(lastDeg + curDeg);
				if (lastDeg != 0.f && Compensation < minCompens)	//if this edge is a fake corner, redo last edge
				{
					OutArray[i-1].Type = EWallType::Wall;
					UE_LOG(LogTemp, Display, TEXT("Edge[%02d] is a fake corner!"), OutArray[i-1].EdgeID)
				}
				else  //this corner is ture
				{
					OutArray[i].Type = EWallType::Corner;
					UE_LOG(LogTemp, Display, TEXT("Found a corner: Edge[%02d]!"), curEdge.EdgeID)
					if (!isEdging)
					{
						isEdging = true;
						UE_LOG(LogTemp, Display, TEXT("Entering a corner on Edge[%02d]!"), curEdge.EdgeID)
					}
				}
			}
			else //if this edge is not a corner
			{
				if (isEdging) //exit isEdging
				{
					isEdging = false;
					UE_LOG(LogTemp, Display, TEXT("Exit corner on Edge[%02d]!"), curEdge.EdgeID)
				}
			}

			//do every edge when in the same line:
			lastDeg = curDeg;
		}
		else //when reach the end of the current line
		{
			lastDeg = 0.f;
			UE_LOG(LogTemp, Display, TEXT("Reaching the end of the line on Edge[%02d]!"), curEdge.EdgeID)
		}
		
		FString printstring = FString::Printf(TEXT("[%02d]Deg: %.2f "), curEdge.EdgeID, curDeg);
		DrawDebugString(GetWorld(), OutArray[i].End + FVector(0.f,0.f,0.f), printstring, 0, FColor::White, 1.f, false, 1.f);
	}
	UE_LOG(LogTemp, Warning, TEXT("Finished corner marking!"))
	
	/*Debugger*/
	if (bShowLog)
	{
		for (auto& [Start, End, ID, LineID, Type] : OutArray)
        {
        	//prints out all elements
        	UE_LOG(NavAware, Display,
        		TEXT("[Start: [%.0f, %.0f], End: [%.0f, %.0f], ID: %02d, LineID: %d, Type: %d]"),
        		Start.X, Start.Y, End.X, End.Y, ID, LineID, Type)
			
        }
		UE_LOG(NavAware, Warning, TEXT("Sorting finished, InArray count: %d, OutArray count: %d"), InArray.Num(), OutArray.Num())
	}
}

void ANavAwareEnhancedBase::DefineCorner() const
{
	
}

