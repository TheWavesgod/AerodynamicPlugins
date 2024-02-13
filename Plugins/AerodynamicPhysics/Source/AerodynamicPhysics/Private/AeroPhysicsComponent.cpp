#include "AeroPhysicsComponent.h"
#include "Airplane.h"
#include "Components/SkeletalMeshComponent.h"
#include "Kismet/GameplayStatics.h"
#include "Kismet/KismetSystemLibrary.h"
#include "AirplaneAnimInstance.h"

UAeroPhysicsComponent::UAeroPhysicsComponent()
{
	PrimaryComponentTick.bCanEverTick = true;

	Airplane = GetOwner<AAirplane>();
	if (Airplane)
	{
		Mesh = Airplane->GetMesh();
		Mesh->SetSimulatePhysics(true);
		Mesh->SetEnableGravity(true);
	}
}

void UAeroPhysicsComponent::BeginPlay()
{
	Super::BeginPlay();

	InitializeAnimationInstance();

	if (Mesh)
	{
		Mesh->SetCollisionEnabled(ECollisionEnabled::QueryAndPhysics);
		Mesh->SetCollisionObjectType(ECC_Pawn);
		Mesh->SetMassOverrideInKg(FName("cog_jnt"), EmptyWeight);
		FVector WorldLocationOfMassCenter = Mesh->GetCenterOfMass();
		FVector LocalLocationOfMassCenter = Mesh->GetComponentTransform().InverseTransformPosition(WorldLocationOfMassCenter);
		Mesh->SetCenterOfMass(CenterOfMass - LocalLocationOfMassCenter, FName("cog_jnt"));
	}

	InitializeArray();
}

void UAeroPhysicsComponent::TickComponent(float DeltaTime, ELevelTick TickType, FActorComponentTickFunction* ThisTickFunction)
{
	Super::TickComponent(DeltaTime, TickType, ThisTickFunction);

	DebugTick(DeltaTime);

	InitializeAnimationInstance();

	AeroPhysicsTick(DeltaTime);

	AddForceToMesh();
}

void UAeroPhysicsComponent::InitializeArray()
{
	// Wheel Array Init
	int32 TyreNum = Tyres.Num();
	WheelCalculationVariblesCache.Init(FWheelCacheVaribles(), TyreNum);
	WheelsForcesToAdd.Init(FVector::ZeroVector, TyreNum);
	WheelAnimVaribles.Init(FWheelAnimVaribles(), TyreNum);

	int32 ThrusterNum = ThrusterSettings.Num();
	ThrusterForcesToAdd.Init(FVector::ZeroVector, ThrusterNum);
	CurrentThrusters.Init(0.0f, ThrusterNum);
}

void UAeroPhysicsComponent::InitializeAnimationInstance()
{
	if (!Mesh || !Mesh->GetAnimInstance()) return;

	MeshAnimInstance = MeshAnimInstance == nullptr ? Cast<UAirplaneAnimInstance>(Mesh->GetAnimInstance()) : MeshAnimInstance;
	if (MeshAnimInstance)
	{
		if (MeshAnimInstance->GetAeroPhysicsComponent() == nullptr)
		{
			MeshAnimInstance->SetAeroPhysicsComponent(this);
		}
	}
}

void UAeroPhysicsComponent::AddForceToMesh()
{
	if (!Mesh) return;

	for (int i = 0; i < WheelsForcesToAdd.Num(); ++i)
	{
		Mesh->AddForceAtLocation(
			WheelsForcesToAdd[i] * 100.0f,
			WheelCalculationVariblesCache[i].ImpactLocation
		);
	}
	for (int i = 0; i < ThrusterForcesToAdd.Num(); ++i)
	{
		Mesh->AddForceAtLocationLocal(
			ThrusterForcesToAdd[i] * 100.0f,
			ThrusterSettings[i].EngineLocation
		);
		DrawDebugLine(
			GetWorld(), 
			Airplane->GetTransform().TransformPosition(ThrusterSettings[i].EngineLocation), 
			Airplane->GetTransform().TransformPosition(ThrusterSettings[i].EngineLocation) + Airplane->GetTransform().TransformVector(ThrusterForcesToAdd[i]), FColor::Red, false, 0.0f);
	}
}

void UAeroPhysicsComponent::AeroPhysicsTick(float DeltaTime)
{
	if (!Mesh) return;

	AeroParametersCalculation(DeltaTime);

	WheelsForceCalculation(DeltaTime);

	ThrusterForceCalculation(DeltaTime);
}

void UAeroPhysicsComponent::AeroParametersCalculation(float DeltaTime)
{
	LastFrameMeshVelocity = MeshVelocity;
	MeshVelocity = Mesh->GetPhysicsLinearVelocity();
	MeshAcceleration = (MeshVelocity - LastFrameMeshVelocity) / DeltaTime;

	MeshAngularVelocityInRadians = Mesh->GetPhysicsAngularVelocityInRadians();

	GForce = CalculateCurrentGForce(DeltaTime);

	FVector ForwardVelocity = MeshVelocity.ProjectOnTo(Mesh->GetForwardVector());
	GroundSpeed = ForwardVelocity.Size() * 0.036;

	FVector RightPlaneVelocity = FVector::VectorPlaneProject(MeshVelocity, Mesh->GetRightVector());
	if (RightPlaneVelocity.Size() > 100.0f)
	{
		FVector RightPlaneVelocityDir = RightPlaneVelocity;
		RightPlaneVelocityDir.Normalize();
		float SignValue = FMath::RadiansToDegrees(FMath::Acos(FVector::DotProduct(RightPlaneVelocity, Mesh->GetUpVector()))) < 90.0f ? -1.0f : 1.0f;
		AngleOfAttack = SignValue * FMath::RadiansToDegrees(FMath::Acos(FVector::DotProduct(RightPlaneVelocityDir, Mesh->GetForwardVector())));
	}
	else
	{
		AngleOfAttack = 0.0f;
	}
}

float UAeroPhysicsComponent::CalculateCurrentGForce(float DeltaTime)
{
	return MeshAcceleration.Dot(Mesh->GetUpVector()) / 981.0f;
}

void UAeroPhysicsComponent::WheelsForceCalculation(float DeltaTime)
{
	WheelsRaycastAndVariablesCache();

	for (int i = 0; i < WheelCalculationVariblesCache.Num(); ++i)
	{
		if (!WheelCalculationVariblesCache[i].bIsHit)
		{
			WheelsForcesToAdd[i] = FVector::ZeroVector;
			continue;
		}

		/** Suspension Force Calculation */
		float SpringWholeLength = Tyres[i].SuspensionSettings.SuspensionMaxRaise + Tyres[i].SuspensionSettings.SuspensionMaxDrop + Tyres[i].SuspensionSettings.SpringPreLoadLength;
		float SpringCompressLength = WheelCalculationVariblesCache[i].Distance - Tyres[i].WheelSettings.WheelRadius - WheelRayOffset;
		float SpringForceLength = SpringWholeLength - SpringCompressLength;
		float SpringForce = SpringForceLength * Tyres[i].SuspensionSettings.SpringRate * 0.01f;

		float SuspensionDistanceDisplacementVel = (WheelCalculationVariblesCache[i].LastFrameDistance - WheelCalculationVariblesCache[i].Distance) / DeltaTime;
		float SuspensionDampingAspect = FMath::Abs(SuspensionDistanceDisplacementVel) > SuspensionStaticThreshold ? SuspensionDistanceDisplacementVel : 0.0f;
		float SuspensionDampingForce = Tyres[i].SuspensionSettings.SuspensionDampingRatio * SuspensionDampingAspect;
		//SuspensionDampingForce = FMath::Abs(SuspensionDampingForce) > SpringForce ? 0.0f : SuspensionDampingForce;

		if (FMath::Abs(SuspensionDistanceDisplacementVel) > SuspensionStaticThreshold)
		{
			AddDebugMessageOnScreen(0.0f, FColor::Blue, FString::Printf(TEXT("Suspension %d: Moving"), i));
		}
		else
		{
			AddDebugMessageOnScreen(0.0f, FColor::Blue, FString::Printf(TEXT("Suspension %d: Static"), i));
		}

		float ActualSuspensionForce = SpringForce + SuspensionDampingForce;

		FVector SuspensionWorldAxis = Airplane->GetActorTransform().TransformVector(Tyres[i].SuspensionSettings.SuspensionAxis);
		FVector SuspensionForce = ActualSuspensionForce * (-SuspensionWorldAxis);

		//WheelsForcesToAdd[i] = SuspensionForce;

		WheelAnimVaribles[i].SuspensionDisplacement = SpringForceLength - Tyres[i].SuspensionSettings.SpringPreLoadLength - Tyres[i].SuspensionSettings.SuspensionMaxDrop;


		/** Friction Calculation */
		FVector WheelVelocity = (WheelCalculationVariblesCache[i].ImpactLocation - WheelCalculationVariblesCache[i].LastFrameImpactLocation) / DeltaTime;
		FVector WheelPlaneVelocity = FVector::VectorPlaneProject(WheelVelocity, WheelCalculationVariblesCache[i].Normal);

		float SurfaceFrictionRatio = WheelCalculationVariblesCache[i].SurfaceMatrial ? WheelCalculationVariblesCache[i].SurfaceMatrial->Friction : 0.7;
		float WheelFrictionRatio = Tyres[i].WheelSettings.WheelFrictionRatio;
		// Friction Ratio Combine mode = Average
		float FrictionRatio = (SurfaceFrictionRatio + WheelFrictionRatio) / 2.0f;
		float StaticFrictionRatio = FrictionRatio + 0.1;
		// Suspension Force resolusion
		FVector NormalSuspensionForce = SuspensionForce.ProjectOnToNormal(WheelCalculationVariblesCache[i].Normal);
		FVector PlaneSuspensionForce = FVector::VectorPlaneProject(SuspensionForce, WheelCalculationVariblesCache[i].Normal);
		// Calculate the wheels' forward and right direction
		FVector WheelForwardDir = Mesh->GetForwardVector();
		FVector WheelRightDir = Mesh->GetRightVector();
		if (Tyres[i].WheelSettings.bAffectedBySteering)
		{
			WheelCalculationVariblesCache[i].CurrentWheelTurnRate = FMath::FInterpTo(WheelCalculationVariblesCache[i].CurrentWheelTurnRate, TargetWheelTurnRate, DeltaTime, 5.0f);
			float WheelSteeringDegree = WheelCalculationVariblesCache[i].CurrentWheelTurnRate * Tyres[i].WheelSettings.MaxSteerAngle;
			WheelAnimVaribles[i].WheelRotation.Yaw = WheelSteeringDegree;

			WheelForwardDir = WheelForwardDir.RotateAngleAxis(WheelSteeringDegree, -SuspensionWorldAxis);
			WheelRightDir = WheelRightDir.RotateAngleAxis(WheelSteeringDegree, -SuspensionWorldAxis);
		}
		WheelForwardDir = FVector::VectorPlaneProject(WheelForwardDir, WheelCalculationVariblesCache[i].Normal);
		WheelForwardDir = WheelForwardDir.Normalize() ? WheelForwardDir : FVector::ZeroVector;
		WheelRightDir = FVector::VectorPlaneProject(WheelRightDir, WheelCalculationVariblesCache[i].Normal);
		WheelRightDir = WheelRightDir.Normalize() ? WheelRightDir : FVector::ZeroVector;

		FVector WheelRightVelocity = WheelPlaneVelocity.ProjectOnTo(WheelRightDir);
		FVector WheelForwardVelocity = WheelPlaneVelocity.ProjectOnTo(WheelForwardDir);

		FVector WheelFrictionForce;
		FVector RightSuspensionForce = PlaneSuspensionForce.ProjectOnTo(WheelRightDir);
		FVector ForwardSuspensionForce = PlaneSuspensionForce.ProjectOnTo(WheelForwardDir);
		if (FMath::Abs(WheelRightVelocity.Size()) > WheelStaticThreshold)
		{
			// Wheel slides horizontally
			float WheelFriction = FMath::Clamp(WheelRightVelocity.Size() * 5000.0f, 0.0f, NormalSuspensionForce.Size() * FrictionRatio);
			WheelFrictionForce = -WheelPlaneVelocity / WheelPlaneVelocity.Size() * WheelFriction + ForwardSuspensionForce;
			if (WheelFrictionForce.Size() > NormalSuspensionForce.Size() * FrictionRatio)
			{
				WheelFrictionForce = -WheelPlaneVelocity / WheelPlaneVelocity.Size() * NormalSuspensionForce.Size() * FrictionRatio;
			}

			AddDebugMessageOnScreen(0.0f, FColor::Green, FString::Printf(TEXT("Wheel %d: Slide Happening"), i));
		}
		else
		{
			float MaxStaticWheelFriction = NormalSuspensionForce.Size() * StaticFrictionRatio;
			float MaxStaticForwardFriction = FMath::Sqrt(FMath::Square(MaxStaticWheelFriction) - FMath::Square(RightSuspensionForce.Size()));

			float TargetWheelForwardFriction = 0.0f;
			if (Tyres[i].WheelSettings.bAffectedByBrake && BrakeForceRatio > 0.0f)
			{
				TargetWheelForwardFriction = MaxStaticForwardFriction * BrakeForceRatio;
			}
			// Drag
			TargetWheelForwardFriction += NormalSuspensionForce.Size() * Tyres[i].WheelSettings.WheelDragRatio;

			if (FMath::Abs(WheelForwardVelocity.Size()) > WheelStaticThreshold)
			{
				WheelFrictionForce = -WheelForwardVelocity / WheelForwardVelocity.Size() * TargetWheelForwardFriction + ForwardSuspensionForce;
				AddDebugMessageOnScreen(0.0f, FColor::Green, FString::Printf(TEXT("Wheel %d: Moving"), i));
			}
			else
			{
				if (ForwardSuspensionForce.Size() > TargetWheelForwardFriction)
				{
					WheelFrictionForce = -WheelForwardVelocity / WheelForwardVelocity.Size() * TargetWheelForwardFriction;
				}
				else
				{
					WheelFrictionForce = PlaneSuspensionForce - PlaneSuspensionForce;
				}
				AddDebugMessageOnScreen(0.0f, FColor::Green, FString::Printf(TEXT("Wheel %d: Static"), i));
			}
		}

		WheelsForcesToAdd[i] = NormalSuspensionForce + WheelFrictionForce;

		float WheelPitchDegree = WheelForwardVelocity.Size() * DeltaTime / Tyres[i].WheelSettings.WheelRadius / PI * 180.0f; 
		float WheelPitchDir = WheelForwardVelocity.Dot(WheelForwardDir) > 0.0f ? 1.0f : -1.0f;
		WheelAnimVaribles[i].WheelRotation.Pitch -= WheelPitchDegree * WheelPitchDir;
		WheelAnimVaribles[i].WheelRotation.Pitch = WheelAnimVaribles[i].WheelRotation.Pitch > 360.0f ? WheelAnimVaribles[i].WheelRotation.Pitch - 360.0f : WheelAnimVaribles[i].WheelRotation.Pitch;
		WheelAnimVaribles[i].WheelRotation.Pitch = WheelAnimVaribles[i].WheelRotation.Pitch < 0.0f ? WheelAnimVaribles[i].WheelRotation.Pitch + 360.0f : WheelAnimVaribles[i].WheelRotation.Pitch;

		DrawDebugLine(GetWorld(), WheelCalculationVariblesCache[i].ImpactLocation, WheelCalculationVariblesCache[i].ImpactLocation + SuspensionForce * 10.0f, FColor::Green, false, 0.0f);
	}
}

void UAeroPhysicsComponent::WheelsRaycastAndVariablesCache()
{
	UWorld* World = GetWorld();
	if (World)
	{
		for (int i = 0; i < Tyres.Num(); ++i)
		{
			FVector CurrentWheelLocation = Mesh->GetSocketLocation(Tyres[i].WheelBoneName);
			float WheelRadius = Tyres[i].WheelSettings.WheelRadius;
			FVector ChassisAxis = Airplane->GetActorTransform().TransformVector(Tyres[i].SuspensionSettings.SuspensionAxis);
			float ChassisMaxRaise = Tyres[i].SuspensionSettings.SuspensionMaxRaise;
			float ChassisMaxDrop = Tyres[i].SuspensionSettings.SuspensionMaxDrop;

			FVector Start = CurrentWheelLocation - ChassisAxis * (ChassisMaxRaise + WheelRayOffset);
			FVector End = CurrentWheelLocation + ChassisAxis * (ChassisMaxDrop + WheelRadius);
			FHitResult WheelHitResult;
			FCollisionQueryParams QueryParams = FCollisionQueryParams::DefaultQueryParam;
			QueryParams.AddIgnoredActor(Airplane);
			QueryParams.AddIgnoredComponent(Mesh);
			QueryParams.bReturnPhysicalMaterial = true;

			World->LineTraceSingleByChannel(
				WheelHitResult,
				Start,
				End,
				ECollisionChannel::ECC_Pawn,
				QueryParams
			);

			WheelCalculationVariblesCache[i].bIsHit = WheelHitResult.bBlockingHit;
			WheelCalculationVariblesCache[i].LastFrameImpactLocation = WheelCalculationVariblesCache[i].ImpactLocation;
			WheelCalculationVariblesCache[i].ImpactLocation = WheelHitResult.bBlockingHit ? WheelHitResult.ImpactPoint : FVector::ZeroVector;
			WheelCalculationVariblesCache[i].LastFrameDistance = WheelCalculationVariblesCache[i].Distance;
			WheelCalculationVariblesCache[i].Distance = WheelHitResult.bBlockingHit ? (WheelHitResult.TraceStart - WheelHitResult.Location).Size() : (Start - End).Size();
			WheelCalculationVariblesCache[i].Normal = WheelHitResult.ImpactNormal;
			WheelCalculationVariblesCache[i].SurfaceMatrial = WheelHitResult.PhysMaterial.Get();
		}
	}
}

void UAeroPhysicsComponent::ThrusterForceCalculation(float DeltaTime)
{
	for (int i = 0; i < ThrusterSettings.Num(); ++i)
	{
		float TargetThruster = CurrentThrusterRatio * ThrusterSettings[i].MaxExtraThrust * 9.8f;

		// Consider the effect of air density
		TargetThruster *= 1.0f;

		CurrentThrusters[i] = FMath::FInterpTo(CurrentThrusters[i], TargetThruster, DeltaTime, 3.0f);

		FVector ThrusterForcesAtLocal = ThrusterSettings[i].ThrustAixs * CurrentThrusters[i];

		ThrusterForcesToAdd[i] = ThrusterForcesAtLocal;
		//ThrusterForcesToAdd[i] = Airplane->GetTransform().TransformVector(ThrusterForcesAtLocal);
	}
}

void UAeroPhysicsComponent::SetWheelsBrake(float AxisValue)
{
	BrakeForceRatio = FMath::Clamp(AxisValue, 0.0f, 1.0f);
}

void UAeroPhysicsComponent::SetSteeringWheels(float AxisValue)
{
	TargetWheelTurnRate = FMath::Clamp(AxisValue, -1.0f, 1.0f);
}

void UAeroPhysicsComponent::SetWheelsRetreated(bool bIsRetreated)
{
	bIsWheelsRetreated = bIsRetreated;
}

void UAeroPhysicsComponent::SetAddThruster(float AxisValue)
{
	float Axis = FMath::Clamp(AxisValue, -1.0f, 1.0f);

	float TargetCurrentThrusterRatio = CurrentThrusterRatio + Axis * ThrusterRatioAddPerSecond * GetWorld()->GetDeltaSeconds();

	CurrentThrusterRatio = FMath::Clamp(TargetCurrentThrusterRatio, 0.0f, 1.0f);
}

void UAeroPhysicsComponent::DebugTick(float DeltaTime)
{
	AddDebugMessageOnScreen(0.0f, FColor::Blue, FString::Printf(TEXT("Jet's Weight: %d kg"), FMath::RoundToInt32(Mesh->GetMass())));
	for (auto i : WheelsForcesToAdd)
	{
		AddDebugMessageOnScreen(0.0f, FColor::Green, FString::Printf(TEXT("WheelForce: %d"), FMath::RoundToInt32(i.Length())));
	}
	if (Mesh)
	{
		DrawDebugSphere(GetWorld(), Mesh->GetCenterOfMass(), 30.0f, 8, FColor::Red, false, 0.0f);
	}
	/*for (int i = 0; i < CurrentThrusters.Num(); ++i)
	{
		AddDebugMessageOnScreen(0.0f, FColor::Red, FString::Printf(TEXT("Thruster %d: %d"), i, FMath::RoundToInt32(CurrentThrusters[i])));
	}*/
}

void UAeroPhysicsComponent::AddDebugMessageOnScreen(const float DisplayTime, const FColor Color, const FString DiplayString)
{
	if (GEngine)
	{
		GEngine->AddOnScreenDebugMessage(
			-1,
			DisplayTime,
			Color,
			DiplayString
		);
	}
}

