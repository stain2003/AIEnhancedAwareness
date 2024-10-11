#include "Actor/NavAwareEnhancedBase.h"

#include "NavigationSystem.h"
#include "AI/NavigationSystemBase.h"
#include "NavMesh/RecastNavMesh.h"
#include "StainMathLibrary.h"

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
	MainNavSystem = FNavigationSystem::GetCurrent<UNavigationSystemV1>(GetWorld());
	if (MainNavSystem)
	{
		MainRecastNavMesh = Cast<ARecastNavMesh>(MainNavSystem->GetDefaultNavDataInstance());
		const NavNodeRef NodeRef = MainRecastNavMesh->FindNearestPoly(GetActorLocation(), FVector(500.f, 500.f, 500.f));
		TArray<FNavigationWallEdge> GetEdges;
		
		MainRecastNavMesh->FindEdges(NodeRef, GetActorLocation(), radius, MainNavSystem->CreateDefaultQueryFilterCopy(), GetEdges);
		
		GatherEdgesWithSorting(GetEdges, WallEdges, bDebug);
		EdgeLinker(WallEdges);
		MarkCorner(WallEdges);
		FilterOnlyInnerEdge(WallEdges);
		//MarkEntry(WallEdges);
	}
	else
	{
		UE_LOG(LogTemp, Error, TEXT("No RecastNavMesh availiable!"))
	}
	
	/*
	 *Debugger*/
	if (bDebug)
	{
		for (auto& [Start, End, ID, LineID, Type, Degree, Prev, Next] : WallEdges)
		{
			DrawDebugBox(GetWorld(), End, FVector(10.f, 10.f, 20.f), FColor::Red, false, 1.1f);
			DrawDebugDirectionalArrow(GetWorld(), Start, End, 2.5f, FColor::MakeRedToGreenColorFromScalar(LineID * 0.15f), false, 1.f);
			if (Type == EWallType::Corner)
			{
				DrawDebugSphere(GetWorld(), End, 30.f, 12, FColor::Cyan, false, 1.f);
			}
			else if (Type == EWallType::Entry)
			{
				DrawDebugSphere(GetWorld(), End, 30.f, 12, FColor::Purple, false, 1.f);
			}
			if (bShowLog)
			{
				//prints out all elements
				uint8 PrevID = 0;
				uint8 NextID = 0;
				if (Prev) PrevID = Prev->EdgeID;
				if (Next) NextID = Next->EdgeID;
				
				UE_LOG(NavAware, Display,
                    TEXT("[Start: [%.0f, %.0f], End: [%.0f, %.0f], ID: %02d, LineID: %d, Type: %d, Degree: %.2f, Prev: [%02d], Next: [%02d]]"),
                    Start.X, Start.Y, End.X, End.Y, ID, LineID, Type, Degree, PrevID, NextID)
			}
		}
	}
}

void ANavAwareEnhancedBase::GatherEdgesWithSorting(TArray<FNavigationWallEdge>& InArray, TArray<FNavPoint>& OutArray, bool bDebug)
{
	FScopeLock Lock(&GatherSortingEdgesSection);
	
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

void ANavAwareEnhancedBase::EdgeLinker(TArray<FNavPoint>& InOutArray)
{
	if (InOutArray.Num() == 0) return;

	uint8 LineHeader = 0;
	const uint8 Num = InOutArray.Num();
	for (uint8 i = 0; i < Num - 1; i++)
	{
		if (InOutArray[i].LineID == 0) {LineHeader++; continue;}

		FNavPoint& CurEdge = InOutArray[i];
		FNavPoint& NxtEdge = InOutArray[i+1];
		
		if (CurEdge.LineID == NxtEdge.LineID)
		{
			CurEdge.NextEdge = &NxtEdge;
			NxtEdge.PrevEdge = &CurEdge;
		}
		else//end of the current line
		{
			FNavPoint& HeadEdge = InOutArray[LineHeader];
			if (CurEdge.End == HeadEdge.Start)	//check if line is circle
			{
				CurEdge.NextEdge = &HeadEdge;
				HeadEdge.PrevEdge = &CurEdge;
			}
			LineHeader = i + 1;
		}
	}
	//End of the array
	FNavPoint& CurEdge = InOutArray.Last();
	FNavPoint& HeadEdge = InOutArray[LineHeader];
	if (CurEdge.End == HeadEdge.Start)	//check if line is circle
	{
		CurEdge.NextEdge = &HeadEdge;
		HeadEdge.PrevEdge = &CurEdge;
	}
}

void ANavAwareEnhancedBase::MarkCorner(TArray<FNavPoint>& InOutArray)
{
	
	FScopeLock Lock(&MarkingCornerSection);
	
	if (InOutArray.Num() == 0) return;
	
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
	uint8 headerIndex = 0;
	uint8 Num = InOutArray.Num();
	for (uint8 i = StartIndex; i < Num; i++)
	{
		FNavPoint& CurEdge = InOutArray[i];
		FNavPoint* NextEdge = nullptr;
		FNavPoint* PrevEdge = nullptr;
		if (CurEdge.NextEdge) NextEdge = CurEdge.NextEdge;
		if (CurEdge.PrevEdge) PrevEdge = CurEdge.PrevEdge;
		
		if (i < Num - 1 && CurEdge.LineID != InOutArray[i+1].LineID)
		{
			lastDeg = 0.f;
		}
		
		if (NextEdge != nullptr)	//when not reach to the end of the line/array
		{
			if (PrevEdge)
			{
				DetectCorner(InOutArray, CurEdge, *NextEdge, *PrevEdge, curDeg, lastDeg, i);
			}
		}
	}
	
	/*
	 *****************************************************Main loop*****************************************************
	 */
	/*for(uint8 i = StartIndex; i < EndIndex; i++)	//note: 'i + 1' can be used safely
	{
		FNavPoint& curEdge = InOutArray[i];
		FNavPoint& nxtEdge = InOutArray[i+1];

		//Check if reach the end of the line yet 
		if (curEdge.LineID == nxtEdge.LineID)
		{
			DetectCorner(InOutArray, curEdge, nxtEdge, curDeg, lastDeg, i);
		}
		else//end of the line
		{
			//check if this line is a circle, and do corner detection for the end edge if so
			if (curEdge.End == InOutArray[headerIndex].Start)
			{
				FNavPoint& headEdge = InOutArray[headerIndex];
				DetectCorner(InOutArray, curEdge, headEdge, curDeg, lastDeg, i);
				if (curEdge.Type == EWallType::Corner && headEdge.Type == EWallType::Corner)
				{
					//check if two vectors are fake corners
					if((curEdge.End - curEdge.Start).Length() <= maxDistForFakeCorner && CheckFakeCorner(headEdge.Degree, curEdge.Degree) && InOutArray[i-1].Type != EWallType::FakeCorner)
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
		
		FString PrintString = FString::Printf(TEXT("[%02d]Deg: %.2f, headindex: %d"), curEdge.EdgeID, curDeg, headerIndex);
		DrawDebugString(GetWorld(), InOutArray[i].End + FVector(0.f,0.f,0.f), PrintString, 0, FColor::White, 1.f, false, 1.f);
	}
	
	/*
	 *End of the array, need to be dealt carefully#1#
	//if is a circle
	FNavPoint& curEdge = InOutArray.Last();
	FNavPoint& headEdge = InOutArray[headerIndex];
	if (curEdge.End == headEdge.Start)
	{
		DetectCorner(InOutArray, curEdge, headEdge, curDeg, lastDeg, InOutArray.Num() - 1);
		/*The reason why I check the head edge if is not already marked as fake instead checking the last edge,
		 * is because when head edge is marked as fake, and if the tail edge is also fake, that will be 2 of fake in a row,
		 * so if the next of the head edge is fake, is should be unmarked in order to keep 2 and 2 in the count of fake,
		 * and on and on it will trigger a chain effect till the end.
		 #1#
		if (curEdge.Type == EWallType::Corner && headEdge.Type == EWallType::Corner)
		{
			if ((curEdge.End - curEdge.Start).Length() <= maxDistForFakeCorner && CheckFakeCorner(headEdge.Degree, curEdge.Degree) && headEdge.Type != EWallType::FakeCorner)
            {
				headEdge.Type = EWallType::FakeCorner;
            	curEdge.Type = EWallType::FakeCorner;
            }
		}
	}*/

	
	
	UE_LOG(LogTemp, Warning, TEXT("Finished corner marking!"))
}

void ANavAwareEnhancedBase::DetectCorner(TArray<FNavPoint>& InOutArray, FNavPoint& CurEdge, FNavPoint& NextEdge, FNavPoint& LastEdge, float& curDeg, float& lastDeg, uint8 i) const
{
	FVector CurVect = CurEdge.End - CurEdge.Start;
	FVector NxtVect = NextEdge.End - NextEdge.Start;
	curDeg = XYDegrees(CurVect, NxtVect);
	//curDeg = FMath::RadiansToDegrees(FMath::Acos(FVector::DotProduct(CurVect, NxtVect))) * FMath::Sign(FVector::CrossProduct(CurVect, NxtVect).Z);
	CurEdge.Degree = curDeg;
	
	//UE_LOG(LogTemp, Display, TEXT("[%02d]Current Deg = %.2f"), CurEdge.EdgeID, CurEdge.Degree)
	
	/*if (CheckCorner(curDeg))	//if this edge is a corner
	{
		if (CurVect.Length() <= maxDistForFakeCorner && CheckFakeCorner(curDeg, lastDeg) && InOutArray[i-1].Type != EWallType::FakeCorner)
		{
			CurEdge.Type = EWallType::FakeCorner;
			InOutArray[i-1].Type = EWallType::FakeCorner;
			//UE_LOG(LogTemp, Display, TEXT("[%02d]Found a fake corner: Edge[%02d][%02d], cur Deg: %1f, last Deg: %1f"), CurEdge.EdgeID, InOutArray[i-1].EdgeID, CurEdge.EdgeID, curDeg, lastDeg)
		}
		else  //this corner is ture
		{
			CurEdge.Type = EWallType::Corner;
			//UE_LOG(LogTemp, Display, TEXT("[%02d]Found a corner: Edge[%02d], cur Deg: %1f, last Deg: %1f!"), CurEdge.EdgeID, CurEdge.EdgeID, curDeg, lastDeg)
		}
	}*/

	if (CheckCorner(curDeg))
	{
		//We don't check if LastEdge is nullptr directly,
		//We checked it in CheckFakeCorner():
		//When LastDeg is equal to 0, we skip calling LastEdge
		//LastDeg is handled back in MarkCorner
		if (CurVect.Length() <= maxDistForFakeCorner && CheckFakeCorner(curDeg, lastDeg) && LastEdge.Type != EWallType::FakeCorner)
		{
			CurEdge.Type = EWallType::FakeCorner;
			LastEdge.Type = EWallType::FakeCorner;
		}
		else
		{
			CurEdge.Type = EWallType::Corner;
		}
	}
	//do every edge when in the same line:
	lastDeg = curDeg;
}

template <typename T>
bool ANavAwareEnhancedBase::CheckFakeCorner(T& curDeg, T& lastDeg) const
{
	const float Compensation = FMath::Abs(static_cast<float>(lastDeg) + static_cast<float>(curDeg));
	return lastDeg != 0.f && Compensation < minCompens;
}

void ANavAwareEnhancedBase::FilterOnlyInnerEdge(TArray<FNavPoint>& InOutArray)
{
	FScopeLock Lock(&FilterSection);
	
	if (InOutArray.Num() == 0) return;

	for (auto& CurEdge : InOutArray)
	{
		if (CurEdge.Type != EWallType::Corner) continue;
		FVector PolyCenter = GetEdgePolyCenter(CurEdge);
		
		FVector CurVect = CurEdge.End - CurEdge.Start;
		FVector StartToCenter = PolyCenter - CurEdge.Start;
		const float DegToPolyCenter = XYDegrees(CurVect, StartToCenter);

		if (CurEdge.Degree * DegToPolyCenter >= 0.f)
		{
			CurEdge.Type = EWallType::Wall;
		}
	}
}

FVector ANavAwareEnhancedBase::GetEdgePolyCenter(const FNavPoint& Edge, NavNodeRef* OutPoly)
{
	FVector OutVector;
	if (MainRecastNavMesh)
	{
		NavNodeRef Poly = MainRecastNavMesh->FindNearestPoly((Edge.Start + Edge.End)/2, FVector(50.f, 50.f, 50.f));
		if (OutPoly) *OutPoly = Poly;
		
		MainRecastNavMesh->GetPolyCenter(Poly, OutVector);
	}
	
	return OutVector;
}

void ANavAwareEnhancedBase::MarkEntry(TArray<FNavPoint>& InOutArray)
{
	FScopeLock Lock(&MarkingEntrySection);

	//if num is 0 or 1, theres no need to mark
	uint8 Num = InOutArray.Num();
	if (Num < 2) return;
	uint8 LineHeader = 0;
	for (int i = 0; i < InOutArray.Num() - 1; i++)
	{
		//skip edge in line '0'
		if (InOutArray[i].LineID == 0)
		{
			LineHeader++;
			continue;
		}
		
		FNavPoint& CurEdge = InOutArray[i];
		FNavPoint& NxtEdge = InOutArray[i+1];
		if (CurEdge.LineID == NxtEdge.LineID)
		{
			if (CurEdge.Type < EWallType::Corner && NxtEdge.Type == EWallType::Corner)
			{
				CurEdge.Type = EWallType::Entry;
			}
			else if (CurEdge.Type == EWallType::Corner && NxtEdge.Type < EWallType::Corner)
			{
				NxtEdge.Type = EWallType::Entry;
			}
		}
		else // reach the end of the current line
		{
			FNavPoint& HeadEdge = InOutArray[LineHeader];
			if (CurEdge.End == HeadEdge.Start)	//if this line is circle, check CurEdge and HeadEdge for entry
			{
				if (CurEdge.Type < EWallType::Corner && HeadEdge.Type == EWallType::Corner)
				{
					CurEdge.Type = EWallType::Entry;
				}
				else if (CurEdge.Type == EWallType::Corner && HeadEdge.Type < EWallType::Corner)
				{
					HeadEdge.Type = EWallType::Entry;
				}
			}
			LineHeader = i + 1;
		}
	}
	// for the very last of the edge
	FNavPoint& CurEdge = InOutArray.Last();
	FNavPoint& HeadEdge = InOutArray[LineHeader];
	UE_LOG(LogTemp, Warning, TEXT("Cheking entries: [%02d]"), CurEdge.EdgeID)
	if (CurEdge.End == HeadEdge.Start)	//if this line is circle, check CurEdge and HeadEdge for entry
	{
		if (CurEdge.Type < EWallType::Corner && HeadEdge.Type == EWallType::Corner)
		{
			CurEdge.Type = EWallType::Entry;
		}
		else if (CurEdge.Type == EWallType::Corner && HeadEdge.Type < EWallType::Corner)
		{
			HeadEdge.Type = EWallType::Entry;
		}
	}

	UE_LOG(LogTemp, Warning, TEXT("Finished marking entries of the corner!"))
}
