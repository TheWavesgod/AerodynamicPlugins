#pragma once

#include "CoreMinimal.h"
#include "Animation/AnimInstance.h"
#include "AirplaneAnimInstance.generated.h"

/**
 * 
 */
UCLASS()
class AERODYNAMICPHYSICS_API UAirplaneAnimInstance : public UAnimInstance
{
	GENERATED_BODY()
	
public:
	virtual void NativeInitializeAnimation() override;
	virtual void NativeUpdateAnimation(float DeltaSeconds) override;

protected:
	UPROPERTY(BlueprintReadOnly, Category = AeroPhysicsComponent, meta = (AllowPrivateAccess = "true"))
	class UAeroPhysicsComponent* AeroPhysicsComponent;

	UPROPERTY(BlueprintReadOnly, Category = Movement, meta = (AllowPrivateAccess = "true"))
	TArray<float> SuspensionDisplacements;

	UPROPERTY(BlueprintReadOnly, Category = Movement, meta = (AllowPrivateAccess = "true"))
	TArray<FRotator> TyresRotation;

	UPROPERTY(BlueprintReadOnly, Category = Movement, meta = (AllowPrivateAccess = "true"))
	bool bIsWheelsRetreated;

	void InitializeArray();

public:
	FORCEINLINE void SetAeroPhysicsComponent(UAeroPhysicsComponent* AnimInsPtr) { AeroPhysicsComponent = AnimInsPtr; }
	FORCEINLINE UAeroPhysicsComponent* GetAeroPhysicsComponent() const { return AeroPhysicsComponent; }
};
