#include "Actor/NavAwareEnhancedBase.h"

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
		
		GatherEdgesWithSorting(TestEdges2, WallEdges, bDebug);
		MarkCorner(WallEdges);
	}
	
	/*
	 *Debugger*/
	if (bDebug)
	{
		for (auto& [Start, End, ID, LineID, Type, Degree] : WallEdges)
		{
			DrawDebugBox(GetWorld(), End, FVector(10.f, 10.f, 20.f), FColor::Red, false, 1.1f);
			DrawDebugDirectionalArrow(GetWorld(), Start, End, 2.5f, FColor::MakeRedToGreenColorFromScalar(LineID * 0.15f), false, 1.f);
			
			if (bShowLog)
			{
				//prints out all elements
				UE_LOG(NavAware, Display,
					TEXT("[Start: [%.0f, %.0f], End: [%.0f, %.0f], ID: %02d, LineID: %d, Type: %d, Degree: %.2f]"),
					Start.X, Start.Y, End.X, End.Y, ID, LineID, Type, Degree)
			}
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

	/*
	Transfer points from InArray into a TMap*/
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
	
	/*Initialize the first element
	 */
	//Check if this element is single
	if (EdgesMap.Find(InArray[0].End) != nullptr || EdgesMap.FindKey(InArray[0].Start) != nullptr)
	{
		CurrentIndex++;
		CurrentLineID = 1;	
		OutArray.Add(FNavPoint(InArray[0].Start, InArray[0].End, CurrentIndex, CurrentLineID));
	}
	else
	{
		CurrentLineID = 0;
		OutArray.Add(FNavPoint(InArray[0].Start, InArray[0].End, CurrentIndex, CurrentLineID));
	}
	EdgesMap.Remove(InArray[0].Start);
	
	FNavPoint LineHeader = OutArray[0];	//LineHeader: supposed to be the head of the current line
	FNavPoint Connector = LineHeader;	//Connector: supposed to be the tail of the current line
	
	/*The main part of the algorithm, loop through every element until map is empty, which means the transfer is completed
	 */
	while(EdgesMap.Num() != 0)
	{
		CurrentIndex++;
		/*
		 *Try to find edges that connected to current line*/
		//Edge found that connected by connector if there is one(tail)
		if (const FVector* Value = EdgesMap.Find(Connector.End))
		{
			OutArray.Push(FNavPoint(Connector.End, *Value, CurrentIndex, CurrentLineID));
			EdgesMap.Remove(Connector.End);
			Connector = OutArray.Last();
			
			continue;
		}
		//Edge found that connected by header if there is one(head)
		if (const FVector* Key = EdgesMap.FindKey(LineHeader.Start))
		{
			OutArray.Insert(FNavPoint(*Key, LineHeader.Start, CurrentIndex, CurrentLineID), CurrentLineEntry);
			EdgesMap.Remove(*Key);
			LineHeader = OutArray[CurrentLineEntry];
			
			continue;
		}

		/*
		 *No connected edge found, starts a new line*/
		const auto it = EdgesMap.CreateIterator();
		//1)if its single 2)if it has head 2)if it has tail 
			//if it has tail, add this first
		if (const FVector* TailValue = EdgesMap.Find(it.Value()))
		{
			CurrentLineID += 1;
			OutArray.Push(FNavPoint(it.Key(), it.Value(), CurrentIndex, CurrentLineID));
			LineHeader = OutArray.Last();
			
			CurrentIndex++;
			OutArray.Push(FNavPoint(it.Value(), *TailValue, CurrentIndex, CurrentLineID));
			Connector = OutArray.Last();
			CurrentLineEntry = OutArray.Num() - 2;
			EdgesMap.Remove(it.Value());
		}
			//if it has head, add its head first
		else if (const FVector* HeadKey = EdgesMap.FindKey(it.Key()))
		{
			CurrentLineID += 1;
			OutArray.Push(FNavPoint(*HeadKey, it.Key(), CurrentIndex, CurrentLineID));
			LineHeader = OutArray.Last();
			EdgesMap.Remove(*HeadKey);
			
			CurrentIndex++;
			OutArray.Push(FNavPoint(it.Key(), it.Value(), CurrentIndex, CurrentLineID));
			Connector = OutArray.Last();
			CurrentLineEntry = OutArray.Num() - 2;
		}
			//if it is single, insert it to the top
		else
		{
			OutArray.Insert(FNavPoint(it.Key(), it.Value(), CurrentIndex, 0), 0);
			CurrentLineEntry += 1;
		}
		EdgesMap.Remove(it.Key());
	}
	
	UE_LOG(NavAware, Warning, TEXT("Finished sorting, InArray count: %d, OutArray count: %d"), InArray.Num(), OutArray.Num())
}

void ANavAwareEnhancedBase::MarkCorner(TArray<FNavPoint>& InOutArray) const
{
	/*
	 * Filtering wall type
	 */
	//Define the start index, to skip LineID '0'
	uint8 StartIndex = 0;
	uint8 EndIndex = InOutArray.Num() - 1;
	for (auto& currentElem : InOutArray)
	{
		if (currentElem.LineID != 0) break;
		StartIndex++;
	}

	
	float curDeg = 0.f;
	float lastDeg = 0.f;
	bool isEdging = false;
	uint8 headerIndex = 0;
	
	/*
	 *Main loop*/
	for(uint8 i = StartIndex; i < EndIndex; i++)	//note: 'i + 1' can be used safely
	{
		FNavPoint& curEdge = InOutArray[i];
		FNavPoint& nxtEdge = InOutArray[i+1];

		//Check if reach the end of the line yet 
		if (curEdge.LineID == nxtEdge.LineID)
		{
			DetectCorner(InOutArray, curEdge, nxtEdge, curDeg, lastDeg, isEdging, i);
		}
		else
		{
			//check if this line is a circle, and do corner detection for the end edge if so
			if (curEdge.End == InOutArray[headerIndex].Start)
			{
				UE_LOG(LogTemp, Display, TEXT("This Line %d is a circle"), curEdge.LineID)
				FNavPoint& headEdge = InOutArray[headerIndex];
				DetectCorner(InOutArray, curEdge, headEdge, curDeg, lastDeg, isEdging, i);
				if (curEdge.Type == EWallType::Corner && headEdge.Type == EWallType::Corner)
				{
					//check if two vectors are fake corners
					if(CheckFakeCorner(headEdge.Degree, curEdge.Degree) && InOutArray[i-1].Type != EWallType::FakeCorner)
					{
						curEdge.Type = EWallType::FakeCorner;
						headEdge.Type = EWallType::FakeCorner;
					}
				}
			}
			//update state variable for next line comes in
			headerIndex = i + 1;
			lastDeg = 0.f;
		}
		
		FString printstring = FString::Printf(TEXT("[%02d]Deg: %.2f, headindex: %d"), curEdge.EdgeID, curDeg, headerIndex);
		DrawDebugString(GetWorld(), InOutArray[i].End + FVector(0.f,0.f,0.f), printstring, 0, FColor::White, 1.f, false, 1.f);
	}
	/*
	 *End of the array, need to be dealt carefully*/
	//if is a circle
	FNavPoint& curEdge = InOutArray.Last();
	FNavPoint& headEdge = InOutArray[headerIndex];
	if (curEdge.End == headEdge.Start)
	{
		DetectCorner(InOutArray, curEdge, headEdge, curDeg, lastDeg, isEdging, InOutArray.Num() - 1);
		/*The reason why I check the head edge if is not already marked as fake instead checking the last edge,
		 * is because when head edge is marked as fake, and if the tail edge is also fake, that will be 2 of fake in a row,
		 * so if the next of the head edge is fake, is should be unmarked in order to keep 2 and 2 in the count of fake,
		 * and on and on it will trigger a chain effect till the end.
		 */
		if (CheckFakeCorner(headEdge.Degree, curEdge.Degree) && headEdge.Type != EWallType::FakeCorner)
		{
			curEdge.Type = EWallType::FakeCorner;
		}
	}
	
	UE_LOG(LogTemp, Warning, TEXT("Finished corner marking!"))
}

void ANavAwareEnhancedBase::DetectCorner(TArray<FNavPoint>& InOutArray, FNavPoint& curEdge, FNavPoint& nxtEdge, float& curDeg, float& lastDeg, bool& bisEdging, uint8 i) const
{
	FVector curVect = (curEdge.End - curEdge.Start).GetSafeNormal2D();
	FVector nxtVect = (nxtEdge.End - nxtEdge.Start).GetSafeNormal2D();
	curDeg = FMath::RadiansToDegrees(FMath::Acos(FVector::DotProduct(curVect, nxtVect))) * FMath::Sign(FVector::CrossProduct(curVect, nxtVect).Z);
	curEdge.Degree = curDeg;
	
	UE_LOG(LogTemp, Display, TEXT("[%02d]Current Deg = %.2f"), curEdge.EdgeID, curEdge.Degree)
	
	if (CheckCorner(curDeg))	//if this edge is a corner
	{
		// const int Compensation = FMath::Abs(lastDeg + curDeg);
		// if (lastDeg != 0.f && Compensation < minCompens)	//if this edge is a fake corner, redo last edge
		// {
		if (CheckFakeCorner(curDeg, lastDeg) && InOutArray[i-1].Type != EWallType::FakeCorner)
		{
			InOutArray[i].Type = EWallType::FakeCorner;
			InOutArray[i-1].Type = EWallType::FakeCorner;
			UE_LOG(LogTemp, Display, TEXT("[%02d]Found a fake corner: Edge[%02d][%02d], cur Deg: %1f, last Deg: %1f"), curEdge.EdgeID, InOutArray[i-1].EdgeID, curEdge.EdgeID, curDeg, lastDeg)
		}
		else  //this corner is ture
		{
			InOutArray[i].Type = EWallType::Corner;
			UE_LOG(LogTemp, Display, TEXT("[%02d]Found a corner: Edge[%02d], cur Deg: %1f, last Deg: %1f!"), curEdge.EdgeID, curEdge.EdgeID, curDeg, lastDeg)
			if (!bisEdging) bisEdging = true;
		}
	}
	else //if this edge is not a corner
	{
		if (bisEdging)	bisEdging = false;
	}
	
	//do every edge when in the same line:
	lastDeg = curDeg;
}

bool ANavAwareEnhancedBase::CheckCorner(const float& curDeg) const
{
	return curDeg >= minCurDeg || curDeg <= -minCurDeg;
}

template <typename T>
bool ANavAwareEnhancedBase::CheckFakeCorner(T& curDeg, T& lastDeg) const
{
	const float Compensation = FMath::Abs(static_cast<float>(lastDeg) + static_cast<float>(curDeg));
	return lastDeg != 0.f && Compensation < minCompens;
}

