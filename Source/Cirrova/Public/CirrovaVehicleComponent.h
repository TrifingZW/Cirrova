#pragma once

#include "CoreMinimal.h"
#include "CirrovaLiminalApi.h"
#include "Components/ActorComponent.h"
#include "Engine/EngineTypes.h"
#include "CirrovaVehicleComponent.generated.h"

class UPrimitiveComponent;
struct FCirrovaVehicleAsyncInput;
struct FCirrovaVehicleOutput;
struct FCirrovaVehicleRuntime;

/** 单轮游戏线程快照：由调度器在物理步进后写入。 */
struct FCirrovaWheelState
{
	// 悬挂行程上限（m）与车轮半径（m），装配后不变。
	float MaxLength = 0.0f;
	float Radius = 0.35f;

	// 车轮角速度（rad/s）。
	float Omega = 0.0f;

	// 当前弹簧长度（m）。
	float SuspensionLength = 0.0f;

	// 悬挂法向力（N）。
	float SuspensionForce = 0.0f;

	// 轮胎力解（SI）。
	FLiminalTireForces TireForces{};

	// 接触解（世界坐标）。
	bool bInContact = false;
	FVector ContactPoint = FVector::ZeroVector;
	FVector ContactNormal = FVector::UpVector;
	FVector LongitudinalAxis = FVector::ForwardVector;
	FVector LateralAxis = FVector::RightVector;
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

	// 使用 Chaos 原生悬挂约束；仅初始化时读取，运行时修改不生效。
	UPROPERTY(
		EditAnywhere,
		BlueprintReadOnly,
		Category = "Cirrova载具组件|悬挂",
		meta = (DisplayName = "原生悬挂约束")
	)
	bool bUseNativeSuspension = true;

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

	// 底盘物理组件（拥有者根组件）。
	TWeakObjectPtr<UPrimitiveComponent> ChassisPrimitive = nullptr;

	// 游戏线程单轮快照。
	TArray<FCirrovaWheelState> WheelStates;

	// 悬挂上挂点（底盘本地空间，cm），供调试绘制。
	TArray<FVector> WheelMountLocations;

	// 发动机角速度（rad/s），由最近一次物理输出写入。
	float EngineOmega = 0.0f;

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

#pragma region Scheduler Interface

	// 由调度器在物理步进前调用：写入驾驶输入与底盘代理。
	void ProduceAsyncInput(FCirrovaVehicleAsyncInput& Out) const;

	// 由调度器在注册后调用：创建原生悬挂约束，失败时逐轮回退为力模式。
	void InitializeSuspensionConstraints(FCirrovaVehicleRuntime& Runtime);

	// 由调度器在物理步进后调用：刷新游戏线程快照并广播接地事件。
	void ConsumeAsyncOutput(const FCirrovaVehicleOutput& In);

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
	bool BuildTopology(FCirrovaVehicleRuntime& Runtime);

	// 从编译产物读取空间装配：悬挂参数、转向控制端口与轮心静止位姿。
	bool ReadSpatialLinks(
		FCirrovaVehicleRuntime& Runtime,
		const TArray<uint64>& SuspensionNodes,
		const TArray<uint64>& KnuckleNodes,
		const TArray<uint64>& WheelNodes
	);

	// 绘制单轮调试图元。
	void DrawWheelDebug(
		const FCirrovaWheelState& WheelState,
		const FVector& MountWorld,
		const FVector& ChassisUp
	) const;

	// 读取节点端口句柄，失败返回 0。
	uint64 PortOf(FLiminalVehicleHandle Handle, uint64 Node, uint32 Index) const;

	// 建立端口连接，失败时记录日志。
	bool Connect(FLiminalVehicleHandle Handle, uint64 SourcePort, uint64 TargetPort) const;

#pragma endregion
};
