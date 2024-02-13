// Fill out your copyright notice in the Description page of Project Settings.


#include "AirplaneAnimInstance.h"
#include "AeroPhysicsComponent.h"

void UAirplaneAnimInstance::NativeInitializeAnimation()
{
	Super::NativeInitializeAnimation();
}

void UAirplaneAnimInstance::NativeUpdateAnimation(float DeltaSeconds)
{
	Super::NativeUpdateAnimation(DeltaSeconds);

	if (!AeroPhysicsComponent) return;

	InitializeArray();

	for (int i = 0; i < AeroPhysicsComponent->WheelAnimVaribles.Num(); ++i)
	{
		SuspensionDisplacements[i] = AeroPhysicsComponent->WheelAnimVaribles[i].SuspensionDisplacement;
		TyresRotation[i] = AeroPhysicsComponent->WheelAnimVaribles[i].WheelRotation;
	}

	bIsWheelsRetreated = AeroPhysicsComponent->GetIsWheelsRetreated();
}

void UAirplaneAnimInstance::InitializeArray()
{
	if (AeroPhysicsComponent->WheelAnimVaribles.Num() == SuspensionDisplacements.Num())
	{
		return;
	}

	int size = AeroPhysicsComponent->WheelAnimVaribles.Num();

	SuspensionDisplacements.Init(0.0f, size);
	TyresRotation.Init(FRotator(), size);
}
