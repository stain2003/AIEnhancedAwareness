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
		MarkEntryEdges(WallEdges);
		MakeCornerArray(WallEdges, Corners);
		TakeSteps(WallEdges, true);
	}
	else
	{
		UE_LOG(NavAware, Error, TEXT("No NavSystem availiable!"))
	}
	
	/*
	 *Debugger*/
	if (bDebug)
	{
		for (const auto&  [Start, End, ID, LineID, Type, Degree, Prev, Next] : WallEdges)
		{
			DrawDebugBox(GetWorld(), End, FVector(10.f, 10.f, 20.f), FColor::Red, false, 1.1f);
			DrawDebugDirectionalArrow(GetWorld(), Start, End, 20.f, FColor::MakeRedToGreenColorFromScalar(LineID * 0.15f), false, 1.f, 0);
			
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
                    TEXT("[Start: [%04.1f, %04.1f], End: [%04.1f, %04.1f], ID: %02d, LineID: %d, Type: %d, Degree: %.2f, Prev: [%02d], Next: [%02d]]"),
                    Start.X, Start.Y, End.X, End.Y, ID, LineID, Type, Degree, PrevID, NextID)
			}
		}
		for (const auto& [Start, End, ID] : Corners)
		{
			if (bShowLog)
			{
				UE_LOG(NavAware, Display, TEXT("Corner[%d]: Start: %02d, End: %02d"), ID, Start->EdgeID, End->EdgeID)
			}
			
			FNavPoint* DrawingEdge = Start;
			while (true)
			{
				DrawDebugDirectionalArrow(GetWorld(), DrawingEdge->Start, DrawingEdge->End, 10.f, FColor::Purple, false, 1.3, 0, 5.f);
				if (DrawingEdge == End) break;
				DrawingEdge = DrawingEdge->NextEdge;
			}
			
			DrawDebugBox(GetWorld(), Start->Start, FVector(5.f, 5.f, 50.f), FColor::Green, false, 1.f);
			DrawDebugBox(GetWorld(), End->End, FVector(5.f, 5.f, 50.f), FColor::Green, false, 1.f);
		}
		for (const auto& Entry : Entries)
		{
			DrawDebugDirectionalArrow(GetWorld(), Entry.Start, Entry.End, 5.f, FColor::Green, false, 1.f);
			DrawDebugDirectionalArrow(GetWorld(), Entry.Start, GetPerpendicularLineFromPointOnEdgeInPolySide(Entry.Start, *Entry.EdgeA), 5.f, FColor::Yellow, false, 1.f);
			
			if (bShowLog)
			{
				UE_LOG(NavAware, Display, TEXT("Entry: CornerEdge: [%02d], TargetEdge: [%02d], width: %.1f, LineA: %d, LineB: %d"), Entry.EdgeA->EdgeID, Entry.EdgeB->EdgeID, Entry.Width, *Entry.CurrentLineID, *Entry.TargetLineID)
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

void ANavAwareEnhancedBase::MarkEntryEdges(TArray<FNavPoint>& InOutArray)
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

void ANavAwareEnhancedBase::MakeCornerArray(TArray<FNavPoint>& InArray, TArray<FCorner>& OutCorners)
{
	FScopeLock Lock(&MakeCornerArraySection);
	
	uint8 Num = InArray.Num();
	if (Num == 0) return;

	OutCorners.Empty();
	
	for (uint8 i = 0; i < Num; i++)
	{
		FNavPoint& CurEdge = InArray[i];
		//UE_LOG(NavAware, Warning, TEXT("Checking [%02d] if is a corner start"), CurEdge.EdgeID)

		/*Additional step, check current line if is a loop that only contains corners
		 * if so, use the length of each edge to determine corners from the line
		 * usually this only happen when a wall is small and straight enough to be wrapped around by edges
		 */
		bool isLineStart = i == 0 || i != 0 && CurEdge.LineID != InArray[i - 1].LineID;
		bool hasPrevEdge = CurEdge.PrevEdge != nullptr;
		if (isLineStart && hasPrevEdge)
		{
			//UE_LOG(NavAware, Warning, TEXT("[%02d] of [%d]this line is a loop"), CurEdge.EdgeID, CurEdge.LineID)
			uint8 EdgeIteratedAlready = 0;;
			bool hasOnlyCorner = false;
			
			if (CurEdge.Type == EWallType::Corner)
			{
				//UE_LOG(NavAware, Warning, TEXT("[%02d] of [%d]start looking for suitable corners for this line"), CurEdge.EdgeID, CurEdge.LineID)
				FNavPoint* NextEdge = CurEdge.NextEdge;
				while (true)
				{
					if (NextEdge == &CurEdge)
					{	
						//UE_LOG(NavAware, Warning, TEXT("this line [%d] is a loop that only contains Corner!"), NextEdge->LineID)
						hasOnlyCorner = true;
						break;
					}
					
					//UE_LOG(NavAware, Warning, TEXT("checking if [%02d] of [%d] is corner"), NextEdge->EdgeID, NextEdge->LineID)
					if (NextEdge->Type != EWallType::Corner)
					{
						//UE_LOG(NavAware, Warning, TEXT("[%02d] of [%d] is not a corner! this line doesnt need to do samll wall test"), NextEdge->EdgeID, NextEdge->LineID)
						break;
					}
					NextEdge = NextEdge->NextEdge;
					EdgeIteratedAlready++;
				}
			}

			//UE_LOG(NavAware, Warning, TEXT("There are %d edges in this looping line"), EdgeIteratedAlready + 1)
			
			if (hasOnlyCorner)
            {
	            FNavPoint& ItEdge = InArray[i];
				//往回寻，找到第一个长度小于 300.f的作为起点：
	            uint8 timesWentBack = 0;
	            FNavPoint* StartPoint = &ItEdge;
				while ((StartPoint->PrevEdge->End - StartPoint->PrevEdge->Start).Length() < 300.f && timesWentBack < EdgeIteratedAlready)
				{
					StartPoint = StartPoint->PrevEdge;
					timesWentBack++;
				}
				//UE_LOG(NavAware, Warning, TEXT("[%d] seemed to be a good start point..."), StartPoint->EdgeID)
				//UE_LOG(NavAware, Warning, TEXT("Starting grouping corners by distance for line: [%d]..."), CurEdge.LineID)
				
				uint8 CornerChecked = 0;
				FNavPoint* StartEdge = nullptr;
				FNavPoint* EndEdge = nullptr;
				
				FNavPoint* LoopEdge = StartPoint;
				
	            while (CornerChecked < EdgeIteratedAlready + 1)
	            {
	            	//UE_LOG(NavAware, Warning, TEXT("looping thourgh line [%d], now on [%02d]..."), LoopEdge->EdgeID, StartPoint->LineID)
		            CornerChecked++;
		            if ((LoopEdge->End - LoopEdge->Start).Length() < 300.f)
		            {
			            if (StartEdge == nullptr)
			            {
				            StartEdge = LoopEdge;
			            }
		            	EndEdge = LoopEdge;
		            }
		            else
		            {
			            if (EndEdge != nullptr)
			            {
			            	//UE_LOG(NavAware, Warning, TEXT("found a corner: start[%02d], end[%02d]..."), StartEdge->EdgeID, EndEdge->EdgeID)
				            OutCorners.Push(FCorner(StartEdge, EndEdge, EndEdge->EdgeID));
			            	EndEdge = nullptr;
			            	StartEdge = nullptr;
			            }
		            }
	            	LoopEdge = LoopEdge->NextEdge;
	            }
				
            	//skip this line, for the future loop
				//UE_LOG(NavAware, Warning, TEXT("Finished small wall corner grouping for line: [%d]"), CurEdge.LineID)
            	i += EdgeIteratedAlready;
            	continue;
            }
		}

		
		const bool CornerStart1 = CurEdge.Type == EWallType::Corner && CurEdge.PrevEdge && CurEdge.PrevEdge->Type == EWallType::Entry;
		const bool CornerStart2 = CurEdge.Type == EWallType::Corner && CurEdge.PrevEdge == nullptr;
		if (CornerStart1 || CornerStart2)
		{
			//UE_LOG(NavAware, Warning, TEXT("Found corner start: [%02d]"), CurEdge.EdgeID)
			FNavPoint* CornerStart = &CurEdge;
			FNavPoint* CornerEnd = nullptr;
			FNavPoint* NextEdge = &CurEdge;
			//Situation 1: keep iterate until find the next entry, or hit the end of the line
			while (NextEdge->NextEdge != nullptr)
			{
				if (i < Num - 1 && InArray[i].LineID == InArray[i + 1].LineID)
				{
					i++;
				}
				
				NextEdge = NextEdge->NextEdge;
				if (NextEdge->Type == EWallType::Entry)
				{
					CornerEnd = NextEdge;
					break;
				}
			}
			
			//Situation 2: hit the end of the line
			// 1) when NextEdge == nullptr: CurEdge is TET line
			// 2) when NextEdge != nullptr: CurEdge is not TET line, but TET line is not entry
			if (CornerEnd == nullptr)
			{
				if (NextEdge == nullptr)
				{
					CornerEnd = &CurEdge;
				}
				else
				{
					CornerEnd = NextEdge;
				}
			}
			OutCorners.Push(FCorner(CornerStart, CornerEnd, CornerEnd->EdgeID));
		}
	}
}

void ANavAwareEnhancedBase::TakeSteps(const TArray<FNavPoint>& InOutArray, bool bDebug)
{
	Entries.Empty();
	if (InOutArray.Num() < 2)	return;
	
	UE_LOG(NavAware, Warning, TEXT("Starting to steps for each corner and find entries from them"))
	/*For every corner*/
	for (auto& CurCorner : Corners)
	{
		/*CurEntries: stores entries derived from current corner to other lines*/
		TMap<uint8, FEntry> FoundEntries;
		/*For every edge on this corner*/
		FNavPoint* LoopingEdge = nullptr;
		while (LoopingEdge != CurCorner.CornerEnd)
		{
			//Init current edge
			if (LoopingEdge != nullptr)
			{
				LoopingEdge = LoopingEdge->NextEdge;
			}
			else
			{
				LoopingEdge = CurCorner.CornerStart;
			}
			
			FVector EdgeStart = LoopingEdge->Start;
			FVector EdgeEnd = LoopingEdge->End;

			//Get nearest edges to this edge from other lines
			TArray<FNavPoint> NearestEdges;
			SortEdgesByDistanceToGivenEdge(*LoopingEdge, InOutArray, NearestEdges);

			//For every target edge
			for (auto& CurTargetEdge : NearestEdges)
			{
				const FVector& TargetEdgeStart = CurTargetEdge.Start;
				const FVector& TargetEdgeEnd = CurTargetEdge.End;
				uint8& TargetLineID = CurTargetEdge.LineID;

				
				FVector PointOnLoopingEdge;
				FVector PointOnTargeEdge;
				std::tie(PointOnLoopingEdge, PointOnTargeEdge) = GetShortestLineSegBetweenTwoLineSeg(EdgeStart, EdgeEnd, TargetEdgeStart, TargetEdgeEnd);
				float NewWidth = (PointOnTargeEdge - PointOnLoopingEdge).Length();
					
				const float DegreeBetweenPerpendicularLineAndEntryLine = XYDegrees(GetPerpendicularLineFromPointOnEdgeInPolySide(PointOnLoopingEdge, *LoopingEdge) - PointOnLoopingEdge, PointOnTargeEdge - PointOnLoopingEdge);
				UE_LOG(NavAware, Warning, TEXT("CurEdge: [%02d], TargetEdge: [%02d], Degree: %.1f"), LoopingEdge->EdgeID, CurTargetEdge.EdgeID, DegreeBetweenPerpendicularLineAndEntryLine)
				if (DegreeBetweenPerpendicularLineAndEntryLine < 45.f && DegreeBetweenPerpendicularLineAndEntryLine > -45.f)
				{
					if (FoundEntries.Find(TargetLineID))
                    {
                    	if (NewWidth < FoundEntries[TargetLineID].Width)
                    	{
                    		continue;
                    	}
                    }
					FoundEntries.FindOrAdd(CurTargetEdge.LineID) =
						FEntry(&CurCorner.CornerID, &LoopingEdge->LineID, &TargetLineID, LoopingEdge, &CurTargetEdge, PointOnLoopingEdge, PointOnTargeEdge, NewWidth);
				}
			}
		}
		//Push result to a global array
		for (auto& Elem : FoundEntries)
		{
			FEntry& Value = Elem.Value;
			Entries.Push(Value);
		}
		
	}
	UE_LOG(NavAware, Warning, TEXT("Stepping finished"))
}

void ANavAwareEnhancedBase::SortEdgesByDistanceToGivenEdge(const FNavPoint& CurEdge, const TArray<FNavPoint>& EdgesCollection, TArray<FNavPoint>& OutArray, bool bOnlyOneForEachLine)
{
	OutArray = EdgesCollection;
	/*
	 *Delete elements in CurEdge's line including itself, preventing calculating their distance
	 */
	bool EnteredSameLine = false;
	for (uint8 Index = 0; Index < OutArray.Num(); Index++)
	{
		if (OutArray[Index].LineID == CurEdge.LineID)
		{
			EnteredSameLine = true;
			OutArray.RemoveAt(Index);
			Index--;
		}
		else
		{
			if (EnteredSameLine) break;
		}
	}
	//Sorting the Array by distance
	FVector MiddlePointOnCurEdge = (CurEdge.Start+CurEdge.End)/2;
	OutArray.Sort([&MiddlePointOnCurEdge](const FNavPoint& EdgeA, const FNavPoint& EdgeB)
	{
		return FVector::Dist(MiddlePointOnCurEdge, (EdgeA.Start + EdgeA.End)/2) < FVector::Dist(MiddlePointOnCurEdge, (EdgeB.Start + EdgeB.End)/2);
	});

	if (bOnlyOneForEachLine)
	{
		//UE_LOG(NavAware, Warning, TEXT("Extracting the nearest edges of each line for edge: [%02d]"), CurEdge.EdgeID)
		TArray<uint8> ContainedLine;
		for (uint8 i = 0; i < OutArray.Num(); i++)
		{
			if (ContainedLine.Contains(OutArray[i].LineID))
			{
				//UE_LOG(NavAware, Warning, TEXT("The nearest edge on line [%d] is already found, ditching: [%02d]"), ArrayByDist[i].LineID, ArrayByDist[i].EdgeID)
				OutArray.RemoveAt(i);
				i--;
			}
			else
			{
				//UE_LOG(NavAware, Warning, TEXT("[%02d] is the first edge found on line [%d], kept"), ArrayByDist[i].EdgeID, ArrayByDist[i].LineID)
				ContainedLine.Push(OutArray[i].LineID);
			}
		}
	}
}
