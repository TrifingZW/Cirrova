#pragma once

#include "CoreMinimal.h"
#include "CirrovaLiminalApi.h"
#include "Components/ActorComponent.h"
#include "Engine/EngineTypes.h"
#include "CirrovaVehicleComponent.generated.h"

class UPrimitiveComponent;

/** 单轮运行态：来自 liminal 编译产物的装配数据与本帧接触解。 */
struct FCirrovaWheelRuntime
{
	// 悬挂上挂点（底盘本地空间，UE 坐标 cm）。
	FVector MountLocation = FVector::ZeroVector;

	// 车轮旋转轴端口（读角速度、注入反作用力矩）。
	uint64 ShaftPort = 0;

	// 转向控制端口，0 表示不可转向。
	uint64 SteeringPort = 0;

	// 悬挂参数（SI）。
	float SuspensionStiffness = 0.0f;
	float SuspensionDamping = 0.0f;
	float RestLength = 0.0f;
	float MinLength = 0.0f;
	float MaxLength = 0.0f;

	// 车轮半径（m）。
	float Radius = 0.35f;

	// 当前弹簧长度（m）。
	float CurrentLength = 0.0f;

	// 上一帧压缩量（m）。
	float PreviousCompression = 0.0f;

	// 首次求解标记：跳过一次压缩速度计算，避免出生时的阻尼冲击。
	bool bSolved = false;

	// 车轮角速度（rad/s）。
	float Omega = 0.0f;

	// 本帧接触解（世界坐标）。
	bool bInContact = false;
	FVector ContactPoint = FVector::ZeroVector;
	FVector ContactNormal = FVector::UpVector;
	FVector LongitudinalAxis = FVector::ForwardVector;
	FVector LateralAxis = FVector::RightVector;

	// 接触点速度缓存（世界坐标，cm/s），供各子步复用。
	FVector ContactVelocity = FVector::ZeroVector;

	// 本帧各子步轮胎合力累计（世界坐标，N）。
	FVector AccumulatedForce = FVector::ZeroVector;

	// 本帧力解（SI，调试读出）。
	float SuspensionForce = 0.0f;
	FLiminalTireForces TireForces{};
};

#pragma region 委托签名

DECLARE_DYNAMIC_MULTICAST_DELEGATE_OneParam(FOnVehicleGroundedSignature, bool, bGrounded);

#pragma endregion

/** 单轮配置：装配位姿与悬挂参数（UE 坐标，单位厘米；悬挂参数为 SI）。 */
USTRUCT(BlueprintType)
struct FCirrovaWheelConfig
{
	GENERATED_BODY()

	// 悬挂上挂点在底盘本地空间的位置。
	UPROPERTY(
		EditAnywhere,
		BlueprintReadOnly,
		Category = "Cirrova载具组件|车轮",
		meta = (DisplayName = "挂点位置")
	)
	FVector MountLocation = FVector::ZeroVector;

	// 车轮滚动半径（cm）。
	UPROPERTY(
		EditAnywhere,
		BlueprintReadOnly,
		Category = "Cirrova载具组件|车轮",
		meta = (DisplayName = "车轮半径")
	)
	float Radius = 35.0f;

	// 是否参与转向。
	UPROPERTY(
		EditAnywhere,
		BlueprintReadOnly,
		Category = "Cirrova载具组件|车轮",
		meta = (DisplayName = "可转向")
	)
	bool bSteerable = false;

	// 悬挂刚度（N/m）。
	UPROPERTY(
		EditAnywhere,
		BlueprintReadOnly,
		Category = "Cirrova载具组件|车轮",
		meta = (DisplayName = "悬挂刚度")
	)
	float SuspensionStiffness = 35000.0f;

	// 悬挂阻尼（N·s/m）。
	UPROPERTY(
		EditAnywhere,
		BlueprintReadOnly,
		Category = "Cirrova载具组件|车轮",
		meta = (DisplayName = "悬挂阻尼")
	)
	float SuspensionDamping = 3000.0f;

	// 悬挂自由长度（m）。
	UPROPERTY(
		EditAnywhere,
		BlueprintReadOnly,
		Category = "Cirrova载具组件|车轮",
		meta = (DisplayName = "悬挂自由长度")
	)
	float SuspensionRestLength = 0.35f;

	// 悬挂行程下限（m）。
	UPROPERTY(
		EditAnywhere,
		BlueprintReadOnly,
		Category = "Cirrova载具组件|车轮",
		meta = (DisplayName = "悬挂行程下限")
	)
	float SuspensionMinLength = 0.2f;

	// 悬挂行程上限（m）。
	UPROPERTY(
		EditAnywhere,
		BlueprintReadOnly,
		Category = "Cirrova载具组件|车轮",
		meta = (DisplayName = "悬挂行程上限")
	)
	float SuspensionMaxLength = 0.5f;
};

UCLASS(
	Blueprintable,
	BlueprintType,
	ClassGroup = ("Cirrova"),
	meta = (BlueprintSpawnableComponent, DisplayName = "Cirrova载具组件")
)
class CIRROVA_API UCirrovaVehicleComponent : public UActorComponent
{
	GENERATED_BODY()

public:
#pragma region Parameter

	// 车轮配置列表，顺序即轮轴输出顺序。
	UPROPERTY(
		EditAnywhere,
		BlueprintReadOnly,
		Category = "Cirrova载具组件",
		meta = (DisplayName = "车轮配置")
	)
	TArray<FCirrovaWheelConfig> WheelConfigs{};

	// 发动机最大扭矩（N·m）。
	UPROPERTY(
		EditAnywhere,
		BlueprintReadOnly,
		Category = "Cirrova载具组件|动力",
		meta = (DisplayName = "发动机最大扭矩")
	)
	float EngineMaxTorque = 300.0f;

	// 发动机最大角速度（rad/s）。
	UPROPERTY(
		EditAnywhere,
		BlueprintReadOnly,
		Category = "Cirrova载具组件|动力",
		meta = (DisplayName = "发动机最大角速度")
	)
	float EngineMaxAngularVelocity = 600.0f;

	// 发动机转动惯量（kg·m²）。
	UPROPERTY(
		EditAnywhere,
		BlueprintReadOnly,
		Category = "Cirrova载具组件|动力",
		meta = (DisplayName = "发动机转动惯量")
	)
	float EngineInertia = 0.5f;

	// 变速箱传动比列表，挡位信号 1..N 依次选择。
	UPROPERTY(
		EditAnywhere,
		BlueprintReadOnly,
		Category = "Cirrova载具组件|动力",
		meta = (DisplayName = "变速箱传动比")
	)
	TArray<float> GearRatios{0.15f, 0.25f, 0.4f, 0.6f};

	// 变速箱转动惯量（kg·m²）。
	UPROPERTY(
		EditAnywhere,
		BlueprintReadOnly,
		Category = "Cirrova载具组件|动力",
		meta = (DisplayName = "变速箱转动惯量")
	)
	float GearboxInertia = 0.2f;

	// 轮轴转动惯量（kg·m²）。
	UPROPERTY(
		EditAnywhere,
		BlueprintReadOnly,
		Category = "Cirrova载具组件|动力",
		meta = (DisplayName = "轮轴转动惯量")
	)
	float AxleInertia = 0.5f;

	// 是否使用粘性差速器（开式差速器的近似）；关闭则轮轴为刚性锁止。
	UPROPERTY(
		EditAnywhere,
		BlueprintReadOnly,
		Category = "Cirrova载具组件|动力",
		meta = (DisplayName = "粘性差速器")
	)
	bool bUseDifferential = true;

	// 差速器转速差阻尼系数（N·s/rad），越大越接近锁止，越小越接近开式。
	UPROPERTY(
		EditAnywhere,
		BlueprintReadOnly,
		Category = "Cirrova载具组件|动力",
		meta = (DisplayName = "差速器耦合系数")
	)
	float DifferentialCoupling = 50.0f;

	// 差速器扭矩容量（N·m）。
	UPROPERTY(
		EditAnywhere,
		BlueprintReadOnly,
		Category = "Cirrova载具组件|动力",
		meta = (DisplayName = "差速器扭矩容量")
	)
	float DifferentialMaxTorque = 2000.0f;

	// 车轮转动惯量（kg·m²）。
	UPROPERTY(
		EditAnywhere,
		BlueprintReadOnly,
		Category = "Cirrova载具组件|动力",
		meta = (DisplayName = "车轮转动惯量")
	)
	float WheelInertia = 1.5f;

	// 车轮摩擦阻力矩（N·m）。
	UPROPERTY(
		EditAnywhere,
		BlueprintReadOnly,
		Category = "Cirrova载具组件|动力",
		meta = (DisplayName = "车轮摩擦阻力矩")
	)
	float WheelFrictionTorque = 5.0f;

	// 制动器最大扭矩（N·m）。
	UPROPERTY(
		EditAnywhere,
		BlueprintReadOnly,
		Category = "Cirrova载具组件|动力",
		meta = (DisplayName = "制动器最大扭矩")
	)
	float MaxBrakeTorque = 2000.0f;

	// 轮胎峰值附着系数。
	UPROPERTY(
		EditAnywhere,
		BlueprintReadOnly,
		Category = "Cirrova载具组件|轮胎",
		meta = (DisplayName = "轮胎附着系数")
	)
	float TireFriction = 1.0f;

	// 轮胎侧向刚度（N/rad）。
	UPROPERTY(
		EditAnywhere,
		BlueprintReadOnly,
		Category = "Cirrova载具组件|轮胎",
		meta = (DisplayName = "轮胎侧向刚度")
	)
	float TireLateralStiffness = 12000.0f;

	// 最大转向角（度）。
	UPROPERTY(
		EditAnywhere,
		BlueprintReadOnly,
		Category = "Cirrova载具组件|操控",
		meta = (DisplayName = "最大转向角")
	)
	float MaxSteeringAngle = 35.0f;

	// 车轮射线碰撞通道。
	UPROPERTY(
		EditAnywhere,
		BlueprintReadOnly,
		Category = "Cirrova载具组件|调试",
		meta = (DisplayName = "射线碰撞通道")
	)
	TEnumAsByte<ECollisionChannel> WheelTraceChannel = ECC_Visibility;

	// 绘制车轮射线与受力调试图元。
	UPROPERTY(
		EditAnywhere,
		BlueprintReadOnly,
		Category = "Cirrova载具组件|调试",
		meta = (DisplayName = "绘制调试图元")
	)
	bool bDrawDebug = false;

#pragma endregion

#pragma region Event Dispatcher

	UPROPERTY(
		BlueprintAssignable,
		Category = "Cirrova载具组件|事件分发器",
		meta = (DisplayName = "接地状态变化")
	)
	FOnVehicleGroundedSignature OnVehicleGroundedEvent{};

#pragma endregion

private:
#pragma region State

	// liminal 载具句柄。
	FLiminalVehicleHandle VehicleHandle = nullptr;

	// 底盘物理组件（拥有者根组件）。
	TWeakObjectPtr<UPrimitiveComponent> ChassisPrimitive = nullptr;

	// 车轮运行态。
	TArray<FCirrovaWheelRuntime> Wheels;

	// 驾驶舱信号端口。
	uint64 ThrottleSignalPort = 0;
	uint64 SteeringSignalPort = 0;
	uint64 GearSignalPort = 0;
	uint64 BrakeSignalPort = 0;

	// 发动机轴端口（转速读出）。
	uint64 EngineShaftPort = 0;

	// 底盘质量（kg），初始化时缓存。
	float ChassisMass = 0.0f;

	// 驾驶输入缓存。
	float ThrottleInput = 0.0f;
	float SteeringInput = 0.0f;
	float BrakeInput = 0.0f;
	int32 GearInput = 1;

	// 上一帧是否有任一车轮接地。
	bool bWasGrounded = false;

#pragma endregion

public:
#pragma region Constructors

	UCirrovaVehicleComponent();

#pragma endregion

#pragma region ActorComponent Interface

	virtual void BeginPlay() override;
	virtual void EndPlay(const EEndPlayReason::Type EndPlayReason) override;
	virtual void TickComponent(
		float DeltaTime,
		ELevelTick TickType,
		FActorComponentTickFunction* ThisTickFunction
	) override;

#pragma endregion

#pragma region API

	// 设置油门输入 [0, 1]。
	UFUNCTION(
		BlueprintCallable,
		Category = "Cirrova载具组件|API",
		meta = (DisplayName = "设置油门")
	)
	void SetThrottle(float Value);

	// 设置转向输入 [-1, 1]，正值为左转。
	UFUNCTION(
		BlueprintCallable,
		Category = "Cirrova载具组件|API",
		meta = (DisplayName = "设置转向")
	)
	void SetSteering(float Value);

	// 设置制动输入 [0, 1]。
	UFUNCTION(
		BlueprintCallable,
		Category = "Cirrova载具组件|API",
		meta = (DisplayName = "设置制动")
	)
	void SetBrake(float Value);

	// 设置挡位：0 为空挡，k 选择第 k 个传动比。
	UFUNCTION(
		BlueprintCallable,
		Category = "Cirrova载具组件|API",
		meta = (DisplayName = "设置挡位")
	)
	void SetGear(int32 Gear);

#pragma endregion

#pragma region SPI

	// 发动机转速（RPM）。
	UFUNCTION(
		BlueprintCallable,
		Category = "Cirrova载具组件|SPI",
		meta = (DisplayName = "获取发动机转速")
	)
	float GetEngineRPM() const;

	// 指定车轮角速度（rad/s）。
	UFUNCTION(
		BlueprintCallable,
		Category = "Cirrova载具组件|SPI",
		meta = (DisplayName = "获取车轮角速度")
	)
	float GetWheelAngularVelocity(int32 WheelIndex) const;

	// 是否有任一车轮接地。
	UFUNCTION(
		BlueprintCallable,
		Category = "Cirrova载具组件|SPI",
		meta = (DisplayName = "是否接地")
	)
	bool IsGrounded() const;

#pragma endregion

private:
#pragma region RPC

#pragma endregion

#pragma region Response

#pragma endregion

#pragma region Callback

#pragma endregion

#pragma region Internal Function

	// 按默认布局构建 liminal 拓扑：驾驶舱、底盘、动力链与逐轮空间链。
	void BuildTopology();

	// 从编译产物读取空间装配：悬挂参数、转向控制端口与轮心静止位姿。
	void ReadSpatialLinks(
		const TArray<uint64>& SuspensionNodes,
		const TArray<uint64>& KnuckleNodes,
		const TArray<uint64>& WheelNodes
	);

	// 求解单轮接触：射线、悬挂力、接触坐标系与接触点速度（帧内几何不变）。
	void SolveContact(FCirrovaWheelRuntime& Wheel, float DeltaTime, float SteeringAngle);

	// 求解单轮轮胎力并注入反作用力矩（按内部子步调用）。
	void SolveTire(FCirrovaWheelRuntime& Wheel, float SubDeltaTime);

	// 绘制单轮调试图元。
	void DrawWheelDebug(const FCirrovaWheelRuntime& Wheel, const FVector& MountWorld, const FVector& ChassisUp) const;

	// 读取节点端口句柄，失败返回 0。
	uint64 PortOf(uint64 Node, uint32 Index) const;

	// 建立端口连接，失败时记录日志。
	bool Connect(uint64 SourcePort, uint64 TargetPort) const;

#pragma endregion
};
