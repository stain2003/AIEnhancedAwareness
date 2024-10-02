// Fill out your copyright notice in the Description page of Project Settings.

#pragma once

#include "CoreMinimal.h"
#include "Runtime/AIModule/Classes/Perception/PawnSensingComponent.h"
#include "SensingComponentExtented.generated.h"


UCLASS(ClassGroup=(Custom), meta=(BlueprintSpawnableComponent))
class AISENSINGEXTENTED_API USensingComponentExtented : public UPawnSensingComponent
{
	GENERATED_BODY()

	
public:
	USensingComponentExtented();

protected:
	virtual void BeginPlay() override;

public:
	virtual void TickComponent(float DeltaTime, ELevelTick TickType,
	                           FActorComponentTickFunction* ThisTickFunction) override;

	
protected:
	virtual void SensePawn(APawn& Pawn) override;
	
/**Extension thingy**/
private:
	
protected:
	
	DECLARE_DYNAMIC_MULTICAST_DELEGATE_OneParam( FUnSeePawnDelegate, APawn*, Pawn );
	
	//Broadcast UnSeePawn delegate
	void BroadCastUnseenPawn(APawn& Pawn);

	//Array to store seen pawns
	UPROPERTY(VisibleInstanceOnly, Category= "State")
	TArray<APawn*> ListOfSeenPawn;

public:
	/** Delegate to execute when we unsee a Pawn (Pawn is seen last frame). */
	UPROPERTY(BlueprintAssignable)
	FUnSeePawnDelegate UnSeePawn;
};
