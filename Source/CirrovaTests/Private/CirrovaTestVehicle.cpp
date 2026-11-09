#include "CirrovaTestVehicle.h"

#include "CirrovaLog.h"
#include "Components/BoxComponent.h"
#include "Engine/World.h"
#include "PhysicsEngine/BodyInstance.h"

ACirrovaTestVehicle::ACirrovaTestVehicle()
{
	PrimaryActorTick.bCanEverTick = false;

	Chassis = CreateDefaultSubobject<UBoxComponent>(TEXT("Chassis"));
	SetRootComponent(Chassis);
	Chassis->SetBoxExtent(FVector(200.0f, 100.0f, 50.0f));
	Chassis->SetCollisionEnabled(ECollisionEnabled::QueryAndPhysics);
	Chassis->SetCollisionObjectType(ECC_Vehicle);
	Chassis->SetCollisionResponseToAllChannels(ECR_Block);
	Chassis->SetGenerateOverlapEvents(false);
	Chassis->SetCanEverAffectNavigation(false);
	Chassis->SetLinearDamping(0.0f);
	Chassis->SetAngularDamping(0.0f);

	Vehicle = CreateDefaultSubobject<UCirrovaVehicleComponent>(TEXT("CirrovaVehicle"));
	Vehicle->bLogTelemetryPeriodically = false;
	Vehicle->bDrawDebug = false;
}

void ACirrovaTestVehicle::PostInitializeComponents()
{
	Super::PostInitializeComponents();

	Chassis->SetBoxExtent(ChassisExtentCm);
	// 测试载具禁用休眠：休眠会跳过约束求解，静态斜坡与气动载荷会读到零轮胎力。
	Chassis->BodyInstance.SleepFamily = ESleepFamily::Custom;
	Chassis->BodyInstance.CustomSleepThresholdMultiplier = 0.0f;
	Chassis->SetSimulatePhysics(true);
	Chassis->SetMassOverrideInKg(NAME_None, FMath::Max(ChassisMassKg, 1.0f), true);
	Chassis->SetPhysicsLinearVelocity(FVector::ZeroVector);
	Chassis->SetPhysicsAngularVelocityInDegrees(FVector::ZeroVector);

	Vehicle->DriveLayout = DriveLayout;
	Vehicle->bUseSphereContactProbe = bUseSphereContactProbe;
	Vehicle->bUseNativeSuspension = bUseNativeSuspension;
	Vehicle->bAllowPhysicsSleep = false;
	Vehicle->TireRelaxationLength = TireRelaxationLength;
	Vehicle->bEnableAerodynamics = bEnableAerodynamics;
}

void ACirrovaTestVehicle::Configure(const FCirrovaTestVehicleSetup& InSetup)
{
	ChassisExtentCm = InSetup.ChassisExtentCm;
	ChassisMassKg = InSetup.ChassisMassKg;
	DriveLayout = InSetup.DriveLayout;
	bUseSphereContactProbe = InSetup.bUseSphereContactProbe;
	bUseNativeSuspension = InSetup.bUseNativeSuspension;
	TireRelaxationLength = InSetup.TireRelaxationLength;
	bEnableAerodynamics = InSetup.bEnableAerodynamics;
}

void ACirrovaTestVehicle::SetInputs(
	const float Throttle,
	const float Steering,
	const float Brake,
	const int32 Gear
) const
{
	if (!Vehicle) { return; }

	Vehicle->SetThrottle(FMath::Clamp(Throttle, 0.0f, 1.0f));
	Vehicle->SetSteering(FMath::Clamp(Steering, -1.0f, 1.0f));
	Vehicle->SetBrake(FMath::Clamp(Brake, 0.0f, 1.0f));
	Vehicle->SetGear(FMath::Max(Gear, 0));
}

void ACirrovaTestVehicle::ZeroMotion() const
{
	if (!Chassis) { return; }

	Chassis->SetPhysicsLinearVelocity(FVector::ZeroVector);
	Chassis->SetPhysicsAngularVelocityInDegrees(FVector::ZeroVector);
}

void ACirrovaTestVehicle::SetHorizontalVelocity(const FVector& Velocity) const
{
	if (!Chassis) { return; }

	Chassis->SetPhysicsLinearVelocity(FVector(Velocity.X, Velocity.Y, 0.0f));
	Chassis->SetPhysicsAngularVelocityInDegrees(FVector::ZeroVector);
}

void ACirrovaTestVehicle::KeepAwake() const
{
	if (Chassis) { Chassis->WakeRigidBody(); }
}

float ACirrovaTestVehicle::GetChassisMass() const
{
	return Chassis ? Chassis->GetMass() : 0.0f;
}

float ACirrovaTestVehicle::GetBodyMass() const
{
	const FBodyInstance* BodyInstance = Chassis ? Chassis->GetBodyInstance() : nullptr;
	return BodyInstance ? BodyInstance->GetBodyMass() : 0.0f;
}
