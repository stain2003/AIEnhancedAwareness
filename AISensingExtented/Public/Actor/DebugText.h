// Fill out your copyright notice in the Description page of Project Settings.

#pragma once

#include "CoreMinimal.h"
#include "GameFramework/Actor.h"
#include "DebugText.generated.h"

class UTextRenderComponent;

UCLASS()
class AISENSINGEXTENTED_API ADebugText : public AActor
{
	GENERATED_BODY()

public:
	ADebugText();

protected:
	virtual void BeginPlay() override;

public:
	virtual void Tick(float DeltaTime) override;

protected:
	UPROPERTY(EditDefaultsOnly, BlueprintReadWrite, Category= "Debug")
	FText DisplayText;
	UPROPERTY(EditDefaultsOnly, BlueprintReadWrite)
	TObjectPtr<UTextRenderComponent> RenderText;
};
