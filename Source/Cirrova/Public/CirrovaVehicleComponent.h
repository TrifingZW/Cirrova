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

/**
 * 单轮遥测快照：由物理线程在每一步计算，游戏线程只读。
 *
 * 单位默认 SI：长度 m、速度 m/s、力 N、力矩 N·m；位置与线速度沿用 UE 的 cm 与 cm/s
 * 并在字段名中标注。轮胎力只含摩擦分量，滚动阻力单独成项。
 */
USTRUCT(BlueprintType)
struct FCirrovaWheelState
{
	GENERATED_BODY()

#pragma region 装配

	// 悬挂行程上限（m）与车轮半径（m），装配后不变。
	UPROPERTY(
		BlueprintReadOnly,
		Category = "Cirrova载具组件|遥测",
		meta = (DisplayName = "悬挂行程上限 (m)")
	)
	float MaxLength = 0.0f;

	UPROPERTY(
		BlueprintReadOnly,
		Category = "Cirrova载具组件|遥测",
		meta = (DisplayName = "车轮半径 (m)")
	)
	float Radius = 0.35f;

#pragma endregion

#pragma region 车轮

	// 车轮角速度（rad/s）。
	UPROPERTY(
		BlueprintReadOnly,
		Category = "Cirrova载具组件|遥测",
		meta = (DisplayName = "车轮角速度 (rad/s)")
	)
	float Omega = 0.0f;

	// 轮面线速度（m/s）。
	UPROPERTY(
		BlueprintReadOnly,
		Category = "Cirrova载具组件|遥测",
		meta = (DisplayName = "轮面线速度 (m/s)")
	)
	float SurfaceSpeed = 0.0f;

	// 轮端等效转动惯量（kg·m²），含传动链折算。
	UPROPERTY(
		BlueprintReadOnly,
		Category = "Cirrova载具组件|遥测",
		meta = (DisplayName = "轮端等效惯量 (kg·m²)")
	)
	float EffectiveInertia = 0.0f;

	// 轮胎约束当前的轮端惯量消元项 R²/J（1/kg）；制动锁止或未启用时为 0。
	UPROPERTY(
		BlueprintReadOnly,
		Category = "Cirrova载具组件|遥测",
		meta = (DisplayName = "轮端惯量消元项 (1/kg)")
	)
	float ConstraintWheelInverseMass = 0.0f;

#pragma endregion

#pragma region 接触

	// 是否接地。
	UPROPERTY(
		BlueprintReadOnly,
		Category = "Cirrova载具组件|遥测",
		meta = (DisplayName = "接地")
	)
	bool bInContact = false;

	// 接触点与法线（世界坐标，cm）。
	UPROPERTY(
		BlueprintReadOnly,
		Category = "Cirrova载具组件|遥测",
		meta = (DisplayName = "接触点 (cm)")
	)
	FVector ContactPoint = FVector::ZeroVector;

	UPROPERTY(
		BlueprintReadOnly,
		Category = "Cirrova载具组件|遥测",
		meta = (DisplayName = "接触法线")
	)
	FVector ContactNormal = FVector::UpVector;

	// 接触坐标系：纵向为轮面在接触平面上的投影，侧向为法线与纵向的叉积。
	UPROPERTY(
		BlueprintReadOnly,
		Category = "Cirrova载具组件|遥测",
		meta = (DisplayName = "纵向轴")
	)
	FVector LongitudinalAxis = FVector::ForwardVector;

	UPROPERTY(
		BlueprintReadOnly,
		Category = "Cirrova载具组件|遥测",
		meta = (DisplayName = "侧向轴")
	)
	FVector LateralAxis = FVector::RightVector;

	// 接触点相对速度与地面速度（世界坐标，cm/s）。
	UPROPERTY(
		BlueprintReadOnly,
		Category = "Cirrova载具组件|遥测",
		meta = (DisplayName = "接触点相对速度 (cm/s)")
	)
	FVector ContactVelocity = FVector::ZeroVector;

	UPROPERTY(
		BlueprintReadOnly,
		Category = "Cirrova载具组件|遥测",
		meta = (DisplayName = "地面速度 (cm/s)")
	)
	FVector GroundVelocity = FVector::ZeroVector;

	// 接触点在接触坐标系下的相对速度（m/s）。
	UPROPERTY(
		BlueprintReadOnly,
		Category = "Cirrova载具组件|遥测",
		meta = (DisplayName = "纵向相对速度 (m/s)")
	)
	float ContactLongitudinalSpeed = 0.0f;

	UPROPERTY(
		BlueprintReadOnly,
		Category = "Cirrova载具组件|遥测",
		meta = (DisplayName = "侧向相对速度 (m/s)")
	)
	float ContactLateralSpeed = 0.0f;

	// 滑移率：(ω·R − v_l) / max(|v_l|, 1 m/s)；抱死时趋近 -1。
	UPROPERTY(
		BlueprintReadOnly,
		Category = "Cirrova载具组件|遥测",
		meta = (DisplayName = "滑移率")
	)
	float SlipRatio = 0.0f;

	// 侧偏角：atan2(v_a, max(|v_l|, 1 m/s))，单位度。
	UPROPERTY(
		BlueprintReadOnly,
		Category = "Cirrova载具组件|遥测",
		meta = (DisplayName = "侧偏角 (deg)")
	)
	float SlipAngleDegrees = 0.0f;

	// 一阶松弛过滤后的侧偏角（deg），仅用于侧向力模型。
	UPROPERTY(
		BlueprintReadOnly,
		Category = "Cirrova载具组件|遥测",
		meta = (DisplayName = "松弛侧偏角 (deg)")
	)
	float RelaxedSlipAngleDegrees = 0.0f;

#pragma endregion

#pragma region 悬挂

	// 悬挂轴（底盘本地空间，单位向量）。
	UPROPERTY(
		BlueprintReadOnly,
		Category = "Cirrova载具组件|遥测",
		meta = (DisplayName = "悬挂轴")
	)
	FVector SuspensionAxis = FVector::UpVector;

	// 当前弹簧长度与压缩量（m）。
	UPROPERTY(
		BlueprintReadOnly,
		Category = "Cirrova载具组件|遥测",
		meta = (DisplayName = "弹簧长度 (m)")
	)
	float SuspensionLength = 0.0f;

	UPROPERTY(
		BlueprintReadOnly,
		Category = "Cirrova载具组件|遥测",
		meta = (DisplayName = "压缩量 (m)")
	)
	float Compression = 0.0f;

	// 压缩速度（m/s），已按减振器泄压阀钳制。
	UPROPERTY(
		BlueprintReadOnly,
		Category = "Cirrova载具组件|遥测",
		meta = (DisplayName = "压缩速度 (m/s)")
	)
	float CompressionVelocity = 0.0f;

	// 原始压缩速度是否超过 ±2 m/s 钳制。
	UPROPERTY(
		BlueprintReadOnly,
		Category = "Cirrova载具组件|遥测",
		meta = (DisplayName = "压缩速度已钳制")
	)
	bool bCompressionVelocityClamped = false;

	// 悬挂法向力（N）。
	UPROPERTY(
		BlueprintReadOnly,
		Category = "Cirrova载具组件|遥测",
		meta = (DisplayName = "悬挂法向力 (N)")
	)
	float SuspensionForce = 0.0f;

	// 防倾杆附加法向力（N），正值为增加载荷。
	UPROPERTY(
		BlueprintReadOnly,
		Category = "Cirrova载具组件|遥测",
		meta = (DisplayName = "防倾杆附加力 (N)")
	)
	float AntiRollForce = 0.0f;

	// 轮胎摩擦圆使用的总法向载荷（N）。
	UPROPERTY(
		BlueprintReadOnly,
		Category = "Cirrova载具组件|遥测",
		meta = (DisplayName = "轮胎法向载荷 (N)")
	)
	float NormalLoad = 0.0f;

	// 该轮是否启用了原生悬挂约束。
	UPROPERTY(
		BlueprintReadOnly,
		Category = "Cirrova载具组件|遥测",
		meta = (DisplayName = "原生悬挂约束")
	)
	bool bNativeSuspensionActive = false;

	// 弹簧长度是否取自约束求解值；false 时取自射线估计。
	UPROPERTY(
		BlueprintReadOnly,
		Category = "Cirrova载具组件|遥测",
		meta = (DisplayName = "约束长度有效")
	)
	bool bUsingConstraintLength = false;

	// 原生悬挂约束上一物理步解出的长度（m）；无效时为 0。
	UPROPERTY(
		BlueprintReadOnly,
		Category = "Cirrova载具组件|遥测",
		meta = (DisplayName = "约束求解长度 (m)")
	)
	float SuspensionConstraintLength = 0.0f;

#pragma endregion

#pragma region 轮胎

	// 摩擦圆内的轮胎纵向力（N），不含滚动阻力。
	UPROPERTY(
		BlueprintReadOnly,
		Category = "Cirrova载具组件|遥测",
		meta = (DisplayName = "轮胎纵向力 (N)")
	)
	float TireLongitudinalForce = 0.0f;

	// 摩擦圆内的轮胎侧向力（N）。
	UPROPERTY(
		BlueprintReadOnly,
		Category = "Cirrova载具组件|遥测",
		meta = (DisplayName = "轮胎侧向力 (N)")
	)
	float TireLateralForce = 0.0f;

	// 摩擦对车轮的反作用力矩（N·m），不含滚动阻力。
	UPROPERTY(
		BlueprintReadOnly,
		Category = "Cirrova载具组件|遥测",
		meta = (DisplayName = "车轮反作用力矩 (N·m)")
	)
	float WheelTorque = 0.0f;

	// 滚动阻力（N，带符号，逆向地面速度）。
	UPROPERTY(
		BlueprintReadOnly,
		Category = "Cirrova载具组件|遥测",
		meta = (DisplayName = "滚动阻力 (N)")
	)
	float RollingResistance = 0.0f;

	// 载荷敏感后的摩擦圆上限（N）。
	UPROPERTY(
		BlueprintReadOnly,
		Category = "Cirrova载具组件|遥测",
		meta = (DisplayName = "摩擦圆上限 (N)")
	)
	float FrictionForceLimit = 0.0f;

	// 轮胎模型给出的侧向力界（N）。
	UPROPERTY(
		BlueprintReadOnly,
		Category = "Cirrova载具组件|遥测",
		meta = (DisplayName = "侧向力界 (N)")
	)
	float LateralForceLimit = 0.0f;

	// 摩擦圆利用率：hypot(纵向力, 侧向力) / 摩擦圆上限。
	UPROPERTY(
		BlueprintReadOnly,
		Category = "Cirrova载具组件|遥测",
		meta = (DisplayName = "摩擦圆利用率")
	)
	float FrictionUtilization = 0.0f;

#pragma endregion

#pragma region 耦合

	// 该轮本步是否以轮胎摩擦约束进入求解器。
	UPROPERTY(
		BlueprintReadOnly,
		Category = "Cirrova载具组件|遥测",
		meta = (DisplayName = "轮胎约束已启用")
	)
	bool bTireConstraintApplied = false;

	// 上一物理帧求解的纵向冲量（N·s）。
	UPROPERTY(
		BlueprintReadOnly,
		Category = "Cirrova载具组件|遥测",
		meta = (DisplayName = "纵向冲量 (N·s)")
	)
	float LongitudinalImpulse = 0.0f;

	// 上一物理帧求解的侧向冲量（N·s）。
	UPROPERTY(
		BlueprintReadOnly,
		Category = "Cirrova载具组件|遥测",
		meta = (DisplayName = "侧向冲量 (N·s)")
	)
	float LateralImpulse = 0.0f;

	// 冲量累计窗口（s），即冲量换算为力的平均步长。
	UPROPERTY(
		BlueprintReadOnly,
		Category = "Cirrova载具组件|遥测",
		meta = (DisplayName = "冲量统计窗口 (s)")
	)
	float ImpulseWindow = 0.0f;

#pragma endregion
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

/** 防倾杆配置：成对车轮的扭转刚度。 */
USTRUCT(BlueprintType)
struct FCirrovaAntiRollBarConfig
{
	GENERATED_BODY()

	// 车轮 A 在车轮配置中的索引。
	UPROPERTY(
		EditAnywhere,
		BlueprintReadOnly,
		Category = "Cirrova载具组件|悬挂",
		meta = (DisplayName = "车轮 A")
	)
	int32 WheelA = 0;

	// 车轮 B 在车轮配置中的索引。
	UPROPERTY(
		EditAnywhere,
		BlueprintReadOnly,
		Category = "Cirrova载具组件|悬挂",
		meta = (DisplayName = "车轮 B")
	)
	int32 WheelB = 1;

	// 扭转刚度（N/m）；0 表示禁用。
	UPROPERTY(
		EditAnywhere,
		BlueprintReadOnly,
		Category = "Cirrova载具组件|悬挂",
		meta = (DisplayName = "防倾杆刚度")
	)
	float Stiffness = 0.0f;
};

/** 发动机全油门扭矩曲线采样点。 */
USTRUCT(BlueprintType)
struct FCirrovaEngineCurvePoint
{
	GENERATED_BODY()

	// 采样点转速（RPM）。
	UPROPERTY(
		EditAnywhere,
		BlueprintReadOnly,
		Category = "Cirrova载具组件|动力",
		meta = (DisplayName = "转速")
	)
	float RPM = 0.0f;

	// 全油门扭矩相对峰值的比例 [0, 1]。
	UPROPERTY(
		EditAnywhere,
		BlueprintReadOnly,
		Category = "Cirrova载具组件|动力",
		meta = (DisplayName = "扭矩比例")
	)
	float TorqueScale = 1.0f;
};

/** 驱动布局：按挂点前后坐标自动划分前/后轴。 */
UENUM(BlueprintType)
enum class ECirrovaDriveLayout : uint8
{
	RWD UMETA(DisplayName = "后驱"),
	FWD UMETA(DisplayName = "前驱"),
	AWD UMETA(DisplayName = "四驱")
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

	// 全油门扭矩曲线；为空时扭矩随转速线性衰减。
	UPROPERTY(
		EditAnywhere,
		BlueprintReadOnly,
		Category = "Cirrova载具组件|动力",
		meta = (DisplayName = "扭矩曲线")
	)
	TArray<FCirrovaEngineCurvePoint> EngineTorqueCurve{};

	// 怠速转速（RPM）；0 表示关闭怠速控制。
	UPROPERTY(
		EditAnywhere,
		BlueprintReadOnly,
		Category = "Cirrova载具组件|动力",
		meta = (DisplayName = "怠速转速")
	)
	float EngineIdleRPM = 0.0f;

	// 怠速扭矩上限（N·m），仅在怠速转速大于 0 时生效。
	UPROPERTY(
		EditAnywhere,
		BlueprintReadOnly,
		Category = "Cirrova载具组件|动力",
		meta = (DisplayName = "怠速扭矩上限")
	)
	float EngineIdleTorque = 40.0f;

	// 满转速下的发动机制动扭矩（N·m）；0 表示关闭。
	UPROPERTY(
		EditAnywhere,
		BlueprintReadOnly,
		Category = "Cirrova载具组件|动力",
		meta = (DisplayName = "发动机制动扭矩")
	)
	float EngineBrakingTorque = 0.0f;

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

	// 是否使用差速器（开式差速 + 摩擦限滑）；关闭则轮轴为刚性锁止。
	UPROPERTY(
		EditAnywhere,
		BlueprintReadOnly,
		Category = "Cirrova载具组件|动力",
		meta = (DisplayName = "差速器")
	)
	bool bUseDifferential = true;

	// 限滑摩擦约束的扭矩容量（N·m）；0 为纯开式差速，越大越接近锁止。
	UPROPERTY(
		EditAnywhere,
		BlueprintReadOnly,
		Category = "Cirrova载具组件|动力",
		meta = (DisplayName = "差速器限滑扭矩容量")
	)
	float DifferentialMaxTorque = 100.0f;

	// 驱动布局：按挂点前后坐标（UE +X 为前）自动划分前/后轴。
	UPROPERTY(
		EditAnywhere,
		BlueprintReadOnly,
		Category = "Cirrova载具组件|动力",
		meta = (DisplayName = "驱动布局")
	)
	ECirrovaDriveLayout DriveLayout = ECirrovaDriveLayout::AWD;

	// 四驱中央差速器限滑扭矩容量（N·m）；0 为纯开式。
	UPROPERTY(
		EditAnywhere,
		BlueprintReadOnly,
		Category = "Cirrova载具组件|动力",
		meta = (DisplayName = "中央差速器限滑扭矩容量")
	)
	float CenterDifferentialMaxTorque = 200.0f;

	// 中央差速器/轮轴转动惯量（kg·m²）。
	UPROPERTY(
		EditAnywhere,
		BlueprintReadOnly,
		Category = "Cirrova载具组件|动力",
		meta = (DisplayName = "中央轮轴转动惯量")
	)
	float CenterAxleInertia = 0.3f;

	// 车轮转动惯量（kg·m²）。
	UPROPERTY(
		EditAnywhere,
		BlueprintReadOnly,
		Category = "Cirrova载具组件|动力",
		meta = (DisplayName = "车轮转动惯量")
	)
	float WheelInertia = 1.5f;

	// 车轮库仑摩擦扭矩上限（N·m）；静止时在扭矩上限内锁止车轮。
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

	// 使用球体扫掠接触检测（扫掠半径 = 车轮半径）；关闭则使用零厚度射线。
	UPROPERTY(
		EditAnywhere,
		BlueprintReadOnly,
		Category = "Cirrova载具组件|车轮",
		meta = (DisplayName = "球体接触扫掠")
	)
	bool bUseSphereContactProbe = true;

	// 使用 Chaos 原生悬挂约束（默认物理路径）；轮胎求解器约束独立于该开关。仅初始化时读取，运行时修改不生效。
	UPROPERTY(
		EditAnywhere,
		BlueprintReadOnly,
		Category = "Cirrova载具组件|悬挂",
		meta = (DisplayName = "原生悬挂约束")
	)
	bool bUseNativeSuspension = true;

	// 允许底盘物理休眠；关闭后每步保持唤醒，适合需要持续静态摩擦/约束测量的载具。仅初始化时读取。
	UPROPERTY(
		EditAnywhere,
		BlueprintReadOnly,
		Category = "Cirrova载具组件|物理",
		meta = (DisplayName = "允许物理休眠")
	)
	bool bAllowPhysicsSleep = true;

	// 防倾杆：每项连接一对车轮，刚度 0 表示禁用。
	UPROPERTY(
		EditAnywhere,
		BlueprintReadOnly,
		Category = "Cirrova载具组件|悬挂",
		meta = (DisplayName = "防倾杆")
	)
	TArray<FCirrovaAntiRollBarConfig> AntiRollBars{};

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

	// 载荷敏感度：μ = μ₀·(F_N/F_ref)^(-k)；0 关闭。
	UPROPERTY(
		EditAnywhere,
		BlueprintReadOnly,
		Category = "Cirrova载具组件|轮胎",
		meta = (DisplayName = "轮胎载荷敏感度")
	)
	float TireLoadSensitivity = 0.1f;

	// 载荷敏感的参考法向载荷（N）。
	UPROPERTY(
		EditAnywhere,
		BlueprintReadOnly,
		Category = "Cirrova载具组件|轮胎",
		meta = (DisplayName = "轮胎参考载荷")
	)
	float TireReferenceLoad = 2500.0f;

	// 滚动阻力系数（无量纲）；0 关闭。
	UPROPERTY(
		EditAnywhere,
		BlueprintReadOnly,
		Category = "Cirrova载具组件|轮胎",
		meta = (DisplayName = "轮胎滚动阻力")
	)
	float TireRollingResistance = 0.015f;

	// 轮胎侧向松弛长度（m）；0 关闭瞬态过滤，侧偏力对侧偏角即时响应。
	UPROPERTY(
		EditAnywhere,
		BlueprintReadOnly,
		Category = "Cirrova载具组件|轮胎",
		meta = (DisplayName = "轮胎松弛长度")
	)
	float TireRelaxationLength = 0.3f;

	// 最大转向角（度）。
	UPROPERTY(
		EditAnywhere,
		BlueprintReadOnly,
		Category = "Cirrova载具组件|操控",
		meta = (DisplayName = "最大转向角")
	)
	float MaxSteeringAngle = 35.0f;

	// 车轮接触检测碰撞通道。
	UPROPERTY(
		EditAnywhere,
		BlueprintReadOnly,
		Category = "Cirrova载具组件|调试",
		meta = (DisplayName = "车轮接触通道")
	)
	TEnumAsByte<ECollisionChannel> WheelTraceChannel = ECC_Visibility;

	// 绘制车轮接触与受力调试图元。
	UPROPERTY(
		EditAnywhere,
		BlueprintReadOnly,
		Category = "Cirrova载具组件|调试",
		meta = (DisplayName = "绘制调试图元")
	)
	bool bDrawDebug = false;

	// P0 验证：按固定间隔输出完整遥测日志。
	UPROPERTY(
		EditAnywhere,
		BlueprintReadOnly,
		Category = "Cirrova载具组件|调试",
		meta = (DisplayName = "周期输出遥测")
	)
	bool bLogTelemetryPeriodically = false;

	// P0 验证：周期遥测输出间隔（s）。
	UPROPERTY(
		EditAnywhere,
		BlueprintReadOnly,
		Category = "Cirrova载具组件|调试",
		meta = (DisplayName = "遥测输出间隔 (s)", ClampMin = "0.1")
	)
	float TelemetryLogIntervalSeconds = 1.0f;

#pragma region 空气动力学

	// 启用空气动力学（阻力与下压力）。
	UPROPERTY(
		EditAnywhere,
		BlueprintReadOnly,
		Category = "Cirrova载具组件|空气动力学",
		meta = (DisplayName = "启用空气动力学")
	)
	bool bEnableAerodynamics = true;

	// 空气密度（kg/m³）。
	UPROPERTY(
		EditAnywhere,
		BlueprintReadOnly,
		Category = "Cirrova载具组件|空气动力学",
		meta = (DisplayName = "空气密度")
	)
	float AirDensity = 1.225f;

	// 风阻系数。
	UPROPERTY(
		EditAnywhere,
		BlueprintReadOnly,
		Category = "Cirrova载具组件|空气动力学",
		meta = (DisplayName = "风阻系数")
	)
	float DragCoefficient = 0.35f;

	// 迎风面积（m²）。
	UPROPERTY(
		EditAnywhere,
		BlueprintReadOnly,
		Category = "Cirrova载具组件|空气动力学",
		meta = (DisplayName = "迎风面积")
	)
	float FrontalArea = 2.2f;

	// 下压力系数（正值为下压）。
	UPROPERTY(
		EditAnywhere,
		BlueprintReadOnly,
		Category = "Cirrova载具组件|空气动力学",
		meta = (DisplayName = "下压力系数")
	)
	float DownforceCoefficient = 0.8f;

	// 下压力作用点相对底盘原点的偏移（cm），用于俯仰/侧倾力矩。
	UPROPERTY(
		EditAnywhere,
		BlueprintReadOnly,
		Category = "Cirrova载具组件|空气动力学",
		meta = (DisplayName = "下压力作用点 (cm)")
	)
	FVector AeroCenterOfPressure = FVector::ZeroVector;

#pragma endregion

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

	// 游戏线程单轮遥测快照；由调度器在物理步进后写入。
	UPROPERTY(
		Transient,
		BlueprintReadOnly,
		Category = "Cirrova载具组件|遥测",
		meta = (AllowPrivateAccess = "true", DisplayName = "车轮遥测")
	)
	TArray<FCirrovaWheelState> WheelStates;

	// 悬挂上挂点（底盘本地空间，cm），供调试绘制。
	TArray<FVector> WheelMountLocations;

	// 发动机角速度（rad/s），由最近一次物理输出写入。
	UPROPERTY(
		Transient,
		BlueprintReadOnly,
		Category = "Cirrova载具组件|遥测",
		meta = (AllowPrivateAccess = "true", DisplayName = "发动机角速度 (rad/s)")
	)
	float EngineOmega = 0.0f;

	// 驾驶输入缓存。
	float ThrottleInput = 0.0f;
	float SteeringInput = 0.0f;
	float BrakeInput = 0.0f;
	int32 GearInput = 1;

	// 上一帧是否有任一车轮接地。
	bool bWasGrounded = false;

	// 子步预算耗尽警告是否已记录，避免物理线程逐帧刷日志。
	bool bWarnedSubstepBudget = false;

	// 耦合回退警告是否已记录。
	bool bWarnedCouplingFallback = false;

	// 接地但无悬挂载荷的累计时间与告警状态（检测双重支撑）。
	float NoSuspensionLoadSeconds = 0.0f;
	bool bWarnedNoSuspensionLoad = false;

	// 地面代理不被原生悬挂约束支持时的告警状态。
	bool bWarnedGroundProxyFallback = false;

	// P0 验证：周期遥测累计时间与不变量告警状态。
	float TelemetryLogAccumulator = 0.0f;
	bool bWarnedNonFiniteTelemetry = false;
	bool bWarnedNativeSuspensionInactive = false;

	// 本步轮胎耦合形态：true 为求解器约束，false 为外力回退。
	UPROPERTY(
		Transient,
		BlueprintReadOnly,
		Category = "Cirrova载具组件|遥测",
		meta = (AllowPrivateAccess = "true", DisplayName = "轮胎约束耦合")
	)
	bool bDebugTireConstraintCoupling = false;

	// 外力回退的内部子步数；约束模式恒为 1。
	UPROPERTY(
		Transient,
		BlueprintReadOnly,
		Category = "Cirrova载具组件|遥测",
		meta = (AllowPrivateAccess = "true", DisplayName = "耦合子步数")
	)
	int32 DebugAppliedSubsteps = 1;

	// 本步求解器步长（s），用于校验悬挂约束阻尼换算。
	UPROPERTY(
		Transient,
		BlueprintReadOnly,
		Category = "Cirrova载具组件|遥测",
		meta = (AllowPrivateAccess = "true", DisplayName = "求解器步长 (s)")
	)
	float DebugSolverDeltaTime = 0.0f;

	// 子步预算耗尽标志，仅外力回退可能置位。
	UPROPERTY(
		Transient,
		BlueprintReadOnly,
		Category = "Cirrova载具组件|遥测",
		meta = (AllowPrivateAccess = "true", DisplayName = "子步预算耗尽")
	)
	bool bDebugSubstepBudgetExceeded = false;

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

	// 最近一次物理步的车轮遥测快照（只读）。
	const TArray<FCirrovaWheelState>& GetWheelStates() const { return WheelStates; }

	// 最近一次物理步的发动机角速度（rad/s）。
	float GetEngineOmega() const { return EngineOmega; }

	// 轮胎是否以求解器约束耦合；false 表示外力回退。
	bool IsTireConstraintCoupling() const { return bDebugTireConstraintCoupling; }

	// 轮胎耦合的内部子步数；约束模式恒为 1。
	int32 GetAppliedSubsteps() const { return DebugAppliedSubsteps; }

	// 最近一次物理步的求解器步长（s）。
	float GetSolverDeltaTime() const { return DebugSolverDeltaTime; }

	// 指定车轮的多行遥测文本，供调试 HUD 直接显示。
	UFUNCTION(
		BlueprintPure,
		Category = "Cirrova载具组件|调试",
		meta = (DisplayName = "获取车轮调试文本")
	)
	FString GetWheelDebugText(int32 WheelIndex) const;

	// 将耦合诊断与全部车轮遥测写入日志，供对照测试采集。
	UFUNCTION(
		BlueprintCallable,
		Category = "Cirrova载具组件|调试",
		meta = (DisplayName = "输出调试快照")
	)
	void LogDebugSnapshot() const;

#pragma endregion

#pragma region Scheduler Interface

	// 由调度器在物理步进前调用：写入驾驶输入与底盘代理。
	void ProduceAsyncInput(FCirrovaVehicleAsyncInput& Out) const;

	// 由调度器在注册后调用：创建原生悬挂约束，失败时逐轮回退为弹簧力施加。
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

	// 校验组件参数；失败时写入可直接展示的原因。
	bool ValidateConfiguration(FString& OutError) const;

	// 按默认布局构建 liminal 拓扑：驾驶舱、底盘、动力链与逐轮空间链。
	bool BuildTopology(FCirrovaVehicleRuntime& Runtime);

	// 校验防倾杆配置并写入运行态。
	void BuildAntiRollBars(FCirrovaVehicleRuntime& Runtime) const;

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
		const FTransform& ChassisToWorld
	) const;

	// 读取节点端口句柄，失败返回 0。
	uint64 PortOf(FLiminalVehicleHandle Handle, uint64 Node, uint32 Index) const;

	// 建立端口连接，失败时记录日志。
	bool Connect(FLiminalVehicleHandle Handle, uint64 SourcePort, uint64 TargetPort) const;

#pragma endregion
};
