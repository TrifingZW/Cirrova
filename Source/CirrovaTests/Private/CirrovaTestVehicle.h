#pragma once

#include "CoreMinimal.h"
#include "CirrovaVehicleComponent.h"
#include "Components/BoxComponent.h"
#include "GameFramework/Actor.h"
#include "CirrovaTestVehicle.generated.h"

class UBoxComponent;

/** 测试载具装配参数；在 FinishSpawning 前写入，组件 BeginPlay 时生效。 */
struct FCirrovaTestVehicleSetup
{
	FVector ChassisExtentCm = FVector(200.0f, 100.0f, 50.0f);
	float ChassisMassKg = 1000.0f;
	ECirrovaDriveLayout DriveLayout = ECirrovaDriveLayout::AWD;
	ECirrovaWheelProbeMode ProbeMode = ECirrovaWheelProbeMode::Cylinder;
	bool bUseNativeSuspension = true;
	float TireRelaxationLength = 0.3f;
	float TireLateralFrictionScale = 1.0f;
	float TireStaticFrictionScale = 1.0f;
	bool bEnableAerodynamics = true;
	bool bUseBaselineSuspension = false;
	bool bUseProgressiveSuspension = false;
};

/**
 * 自动化测试载具：盒形底盘 + Cirrova 载具组件。
 *
 * 不包含任何车轮视觉件或其它碰撞体，避免与解析悬挂构成双重支撑。
 * 底盘质量通过 BodyInstance 质量覆盖固定，默认 1000 kg。
 */
UCLASS()
class ACirrovaTestVehicle : public AActor
{
	GENERATED_BODY()

public:
	ACirrovaTestVehicle();

	virtual void PostInitializeComponents() override;

	// 在 FinishSpawning 前写入装配参数。
	void Configure(const FCirrovaTestVehicleSetup& InSetup);

	UCirrovaVehicleComponent* GetVehicleComponent() const { return Vehicle.Get(); }
	UPrimitiveComponent* GetChassis() const { return Chassis.Get(); }

	// 底盘报告质量与物理体质量（kg），用于装配诊断。
	float GetChassisMass() const;
	float GetBodyMass() const;

	// 写入驾驶输入。
	void SetInputs(float Throttle, float Steering, float Brake, int32 Gear) const;

	// 清零线速度与角速度。
	void ZeroMotion() const;

	// 保持物理体唤醒；测试场景需要持续测量静摩擦与悬挂力，不能依赖休眠。
	void KeepAwake() const;

	// 设置水平线速度并清除角速度；Z 分量被忽略。
	void SetHorizontalVelocity(const FVector& Velocity) const;

private:
	UPROPERTY(VisibleAnywhere)
	TObjectPtr<UBoxComponent> Chassis;

	UPROPERTY(VisibleAnywhere)
	TObjectPtr<UCirrovaVehicleComponent> Vehicle;

	UPROPERTY()
	FVector ChassisExtentCm = FVector(200.0f, 100.0f, 50.0f);

	UPROPERTY()
	float ChassisMassKg = 1000.0f;

	UPROPERTY()
	ECirrovaDriveLayout DriveLayout = ECirrovaDriveLayout::AWD;

	UPROPERTY()
	ECirrovaWheelProbeMode ProbeMode = ECirrovaWheelProbeMode::Cylinder;

	UPROPERTY()
	bool bUseNativeSuspension = true;

	UPROPERTY()
	float TireRelaxationLength = 0.3f;

	UPROPERTY()
	float TireLateralFrictionScale = 1.0f;

	UPROPERTY()
	float TireStaticFrictionScale = 1.0f;

	UPROPERTY()
	bool bEnableAerodynamics = true;

	UPROPERTY()
	bool bUseBaselineSuspension = false;

	UPROPERTY()
	bool bUseProgressiveSuspension = false;
};
