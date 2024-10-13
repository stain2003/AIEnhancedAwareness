﻿#include "Actor/NavAwareEnhancedBase.h"

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

void ANavAwareEnhancedBase::FindNearestEdges(bool bDebug, float radius)
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
		MarkEntry(WallEdges);
	}
	else
	{
		UE_LOG(NavAware, Error, TEXT("No RecastNavMesh availiable!"))
	}
	
	/*
	 *Debugger*/
	if (bDebug)
	{
		for (const auto&  [Start, End, ID, LineID, Type, Degree, Prev, Next] : WallEdges)
		{
			DrawDebugBox(GetWorld(), End, FVector(10.f, 10.f, 20.f), FColor::Red, false, 1.1f);
			DrawDebugDirectionalArrow(GetWorld(), Start, End, 20.f, FColor::MakeRedToGreenColorFromScalar(LineID * 0.15f), false, 1.f);
			
			FString PrintString = FString::Printf(TEXT("[%d][%02d]Deg: %.2f, Length: %.2f"), LineID, ID, Degree, (End - Start).Length());
			DrawDebugString(GetWorld(), End + FVector(0.f,0.f,0.f), PrintString, 0, FColor::White, 1.f, false, 1.f);
			
			if (Type == EWallType::Corner)
			{
				DrawDebugSphere(GetWorld(), End, 20.f, 8, FColor::Cyan, false, 1.f);
			}
			else if (Type == EWallType::Entry)
			{
				DrawDebugSphere(GetWorld(), End, 20.f, 8, FColor::Purple, false, 1.f);
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
	
	//OutArray.Empty();
	TArray<FNavPoint> TempArray;
	
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
		TempArray.Add(FNavPoint(InArray[0].Start, InArray[0].End, CurrentIndex, CurrentLineID));
	}
	else
	{
		CurrentLineID = 0;
		TempArray.Add(FNavPoint(InArray[0].Start, InArray[0].End, CurrentIndex, CurrentLineID));
	}
	EdgesMap.Remove(InArray[0].Start);
	
	FNavPoint LineHeader = TempArray[0];	//LineHeader: supposed to be the head of the current line
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
			TempArray.Push(FNavPoint(Connector.End, *Value, CurrentIndex, CurrentLineID));
			EdgesMap.Remove(Connector.End);
			Connector = TempArray.Last();
			
			continue;
		}
		//Edge found that connected by header if there is one(head)
		if (const FVector* Key = EdgesMap.FindKey(LineHeader.Start))
		{
			TempArray.Insert(FNavPoint(*Key, LineHeader.Start, CurrentIndex, CurrentLineID), CurrentLineEntry);
			EdgesMap.Remove(*Key);
			LineHeader = TempArray[CurrentLineEntry];
			
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
			TempArray.Push(FNavPoint(it.Key(), it.Value(), CurrentIndex, CurrentLineID));
			LineHeader = TempArray.Last();
			
			CurrentIndex++;
			TempArray.Push(FNavPoint(it.Value(), *TailValue, CurrentIndex, CurrentLineID));
			Connector = TempArray.Last();
			CurrentLineEntry = TempArray.Num() - 2;
			EdgesMap.Remove(it.Value());
		}
			//if it has head, add its head first
		else if (const FVector* HeadKey = EdgesMap.FindKey(it.Key()))
		{
			CurrentLineID += 1;
			TempArray.Push(FNavPoint(*HeadKey, it.Key(), CurrentIndex, CurrentLineID));
			LineHeader = TempArray.Last();
			EdgesMap.Remove(*HeadKey);
			
			CurrentIndex++;
			TempArray.Push(FNavPoint(it.Key(), it.Value(), CurrentIndex, CurrentLineID));
			Connector = TempArray.Last();
			CurrentLineEntry = TempArray.Num() - 2;
		}
			//if it is single, insert it to the top
		else
		{
			TempArray.Insert(FNavPoint(it.Key(), it.Value(), CurrentIndex, 0), 0);
			CurrentLineEntry += 1;
		}
		EdgesMap.Remove(it.Key());
	}

	OutArray = TempArray;
	
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
	for (auto& currentElem : InOutArray)
	{
		if (currentElem.LineID != 0) break;
		StartIndex++;
	}
	
	float curDeg = 0.f;
	float lastDeg = 0.f;
	uint8 Num = InOutArray.Num();
	for (uint8 i = StartIndex; i < Num; i++)	//loop through every element
	{
		FNavPoint& CurEdge = InOutArray[i];
		FNavPoint* NextEdge = nullptr;
		FNavPoint* PrevEdge = nullptr;
		if (CurEdge.NextEdge) NextEdge = CurEdge.NextEdge;
		if (CurEdge.PrevEdge) PrevEdge = CurEdge.PrevEdge;

		//reset lastDeg when entering a new line
		if (i > 0 && CurEdge.LineID != InOutArray[i - 1].LineID)
		{
			lastDeg = 0.f;
		}
		
		if (NextEdge != nullptr)	//when not reach to the end of the line/array
		{
			DetectCorner(InOutArray, CurEdge, *NextEdge, *PrevEdge, curDeg, lastDeg, i);
		}
	}
	UE_LOG(NavAware, Warning, TEXT("Finished corner marking!"))
}

void ANavAwareEnhancedBase::DetectCorner(TArray<FNavPoint>& InOutArray, FNavPoint& CurEdge, FNavPoint& NextEdge, FNavPoint& LastEdge, float& curDeg, float& lastDeg, uint8 i) const
{
	FVector CurVect = CurEdge.End - CurEdge.Start;
	FVector NxtVect = NextEdge.End - NextEdge.Start;
	curDeg = XYDegrees(CurVect, NxtVect);
	
	CurEdge.Degree = curDeg;
	
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

void ANavAwareEnhancedBase::MarkEntry(TArray<FNavPoint>& InOutArray)
{
#define BOTH ECornerCheck::BothAreCorner
#define ONLYNEXT ECornerCheck::NextIsCorner
#define ONLYPREV ECornerCheck::PrevIsCorner
#define NONE ECornerCheck::None
	
	FScopeLock Lock(&MarkingEntrySection);

	//if num is 0 or 1, theres no need to mark
	uint8 Num = InOutArray.Num();
	if (Num < 2) return;
	
	for (int i = 0; i < Num; i++)
	{
		FNavPoint& CurEdge = InOutArray[i];
		
		if (CurEdge.LineID == 0) continue;
		
		if (CurEdge.Type < EWallType::Corner)
		{
			switch (CheckNeighborCorner(CurEdge))
			{
			case ONLYNEXT:
			case ONLYPREV:
				CurEdge.Type = EWallType::Entry;
				break;
				
			case BOTH:
				if (GetEdgeNeighborDist(CurEdge) <= CornerBlur)
				{
					CurEdge.Type = EWallType::Corner;
				}
				else
				{
					CurEdge.Type = EWallType::Entry;
				}
				break;
				
			case NONE:
				default:
				break;
			}
		}
	}

	UE_LOG(NavAware, Warning, TEXT("Finished marking entries of the corner!"))
}

void ANavAwareEnhancedBase::TakeSteps(const TArray<FNavPoint>& InOutArray)
{
	for (auto& CurEdge : InOutArray)
	{
		if (CurEdge.Type == EWallType::Corner || CurEdge.NextEdge && CurEdge.NextEdge->Type >= EWallType::Corner )
		{
			uint8 Step = 0;
			FVector Point = CurEdge.Start;
			while (CheckIfWithinEdge(CurEdge.Start, CurEdge.End, Point))
			{
				Point = TakeStepOnEdge(CurEdge.Start, CurEdge.End, 30.f, Step);
				DrawDebugBox(GetWorld(), Point, FVector(5.f, 5.f, 5.f), FColor::Magenta, false, 1.f, 0, 2.f);
				Step++;
			}
		}
	}
}
