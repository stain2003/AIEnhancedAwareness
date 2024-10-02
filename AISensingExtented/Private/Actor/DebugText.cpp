#include "Actor/DebugText.h"

#include "Components/TextRenderComponent.h"


ADebugText::ADebugText()
{
	PrimaryActorTick.bCanEverTick = false;
}

void ADebugText::BeginPlay()
{
	Super::BeginPlay();
	RenderText = CreateDefaultSubobject<UTextRenderComponent>(FName(TEXT("Display Text")));
	RenderText->SetText(DisplayText);
	RenderText->SetVisibility(true);
}

void ADebugText::Tick(float DeltaTime)
{
	Super::Tick(DeltaTime);
}

