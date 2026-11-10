#pragma once

#include "CoreMinimal.h"
#include "CirrovaLiminalApi.h"
#include "CirrovaTypes.h"
#include "Chaos/ChaosEngineInterface.h"
#include "Chaos/PhysicsObject.h"

class IPhysicsProxyBase;

namespace Chaos
{
	class FSuspensionConstraint;
}

/** 运行态悬挂刚度曲线采样点。 */
struct FCirrovaSuspensionCurveRuntimePoint
{
	float Compression = 0.0f;
	float StiffnessScale = 1.0f;
};

/** 运行态发动机制动曲线采样点。 */
struct FCirrovaEngineBrakingRuntimePoint
{
	float RPM = 0.0f;
	float Torque = 0.0f;
};

/** 单轮物理运行态，仅在物理线程读写。 */
struct FCirrovaWheelRuntime
{
	// 悬挂上挂点（底盘本地空间，cm）。
	FVector MountLocation = FVector::ZeroVector;

	// 悬挂轴与转向轴（底盘本地空间，单位向量）。由空间域关节描述导出。
	FVector SuspensionAxis = FVector::UpVector;
	FVector SteeringAxis = FVector::UpVector;

	// 车轮旋转轴端口（读角速度、注入反作用力矩）。
	uint64 ShaftPort = 0;

	// 轮胎摩擦约束在共享容器中的索引；仅物理线程读写。
	int32 TireConstraintIndex = INDEX_NONE;

	// 是否参与转向（决定接触坐标系的转向角）。
	bool bSteerable = false;

	// 悬挂参数（SI）。
	float SuspensionStiffness = 0.0f;
	float SuspensionDamping = 0.0f;
	float CompressionDamping = 0.0f;
	float ReboundDamping = 0.0f;
	float SuspensionPreload = 0.0f;
	float BumpStopStiffness = 0.0f;
	float BumpStopStart = 0.0f;
	TArray<FCirrovaSuspensionCurveRuntimePoint> SuspensionCurve;
	float RestLength = 0.0f;
	float MinLength = 0.0f;
	float MaxLength = 0.0f;

	// 车轮半径（m）。
	float Radius = 0.35f;

	// 车轮宽度（m），圆柱包络使用。
	float WheelWidth = 0.25f;

	// 当前弹簧长度（m）。
	float CurrentLength = 0.0f;

	// 射线估计的弹簧长度（m），未启用原生约束时使用。
	float RaycastLength = 0.0f;

	// 原生悬挂约束上一物理步解出的弹簧长度（m）。
	float ConstraintLength = 0.0f;
	bool bConstraintLengthValid = false;

	// 原生悬挂约束上一物理步的世界空间位置修正（kg·cm），用于还原真实支撑力。
	FVector ConstraintPushOut = FVector::ZeroVector;
	FVector ConstraintHardStopPushOut = FVector::ZeroVector;
	bool bConstraintPushOutValid = false;

	// 上一步压缩量（m）。
	float PreviousCompression = 0.0f;

	// 当前压缩量（m），供防倾杆成对求解。
	float Compression = 0.0f;

	// 首次求解标记：跳过一次压缩速度计算，避免出生时的阻尼冲击。
	bool bSolved = false;

	// 车轮角速度（rad/s）。
	float Omega = 0.0f;

	// 本步转向节目标转角（rad；liminal 车辆坐标系 X=右，绕上轴正值为右转）。
	float SteeringAngleRad = 0.0f;

	// 本步制动信号（归一化 [0,1]，含手刹与 ABS 调制），相对转向节制动容量。
	float BrakeSignal = 0.0f;

	// 轮胎冲量模型使用的轮端等效转动惯量（kg·m²）。
	float EffectiveInertia = 0.0f;

	// 轮胎约束当前采用的轮端惯量消元项 R²/J（1/kg）；约束未启用时为 0。
	float ConstraintWheelInverseMass = 0.0f;

	// 轮心（世界坐标，cm）。
	FVector WheelCenter = FVector::ZeroVector;

	// 接触解（世界坐标与接触坐标系）。
	bool bInContact = false;
	FVector ContactPoint = FVector::ZeroVector;
	FVector ContactNormal = FVector::UpVector;
	FVector LongitudinalAxis = FVector::ForwardVector;
	FVector LateralAxis = FVector::RightVector;

	// 接触点速度缓存（世界坐标，cm/s）。
	FVector ContactVelocity = FVector::ZeroVector;

	// 接触点地面速度（世界坐标，cm/s），含地面刚体的平动与转动贡献。
	FVector GroundVelocity = FVector::ZeroVector;

	// 滚动阻力（N），作为外部力施加于接触点；不产生车轮反作用。
	float RollingResistance = 0.0f;

	// 本求解器步内的支撑力（悬挂 + 防倾杆，世界坐标，N），施加后清零。
	FVector SupportForce = FVector::ZeroVector;

	// 本求解器步内待施加于接触点的轮胎力（世界坐标，N）：
	// 外力回退为各内部子步的合力累计，约束模式为滚动阻力。
	FVector TireForce = FVector::ZeroVector;

	// 本步力解（SI，供输出快照）。
	float SuspensionForce = 0.0f;
	float AntiRollForce = 0.0f;
	float NormalLoad = 0.0f;
	FLiminalTireForces TireForces{};

	// 接触点相对速度在接触坐标系下的分量（m/s）与侧偏角（rad）。
	float ContactLongitudinalSpeed = 0.0f;
	float ContactLateralSpeed = 0.0f;
	float SlipAngleRad = 0.0f;

	// 滑移率与轮面线速度（m/s）。
	float SlipRatio = 0.0f;
	float SurfaceSpeed = 0.0f;

	// 侧向松弛：一阶过滤后的侧偏角（rad）与初始化标志。
	float RelaxedSlipAngleRad = 0.0f;
	bool bRelaxationInitialized = false;

	// 压缩速度（m/s）与钳制标志。
	float CompressionVelocity = 0.0f;
	bool bCompressionVelocityClamped = false;

	// 弹簧长度是否取自原生约束求解值。
	bool bUsingConstraintLength = false;

	// 输出的轮胎力只含摩擦分量，滚动阻力单独记录；力矩不含滚动阻力。
	float TireFrictionLongitudinal = 0.0f;
	float TireFrictionLateral = 0.0f;
	float WheelTorque = 0.0f;

	// 载荷敏感与地面材质合并后的纵/侧向摩擦界（N）与椭圆利用率。
	float FrictionForceLimit = 0.0f;
	float PendingFrictionForceLimit = 0.0f;
	float LateralForceLimit = 0.0f;
	float StaticFrictionForceLimit = 0.0f;
	float StaticLateralForceLimit = 0.0f;
	float KineticFrictionForceLimit = 0.0f;
	float KineticLateralForceLimit = 0.0f;
	float FrictionUtilization = 0.0f;

	// 接触地面材质的动/静摩擦系数；无材质时均为 1。
	float GroundFriction = 1.0f;
	float GroundStaticFriction = 1.0f;

	// 纵向松弛后的轮面线速度（m/s）与初始化标志。
	float RelaxedSurfaceSpeed = 0.0f;
	bool bLongitudinalRelaxationInitialized = false;

	// 该轮本步是否以轮胎约束进入求解器。
	bool bTireConstraintApplied = false;

	// 本步制动扭矩容量（N·m）：脚刹（含 ABS 调制）与手刹取较大值。
	float BrakeTorqueCapacity = 0.0f;

	// 锁止观测：约束求解后轮端静止（|ω| < 0.5 rad/s）且制动容量非零；
	// P1.4 起锁止是涌现行为，不再是输入判据。
	bool bWheelLocked = false;

	// 上一物理帧求解的冲量（N·s）与累计窗口（s）。
	float LongitudinalImpulse = 0.0f;
	float LateralImpulse = 0.0f;
	float ImpulseWindow = 0.0f;

	// 原生悬挂约束：句柄在游戏线程创建与释放，物理线程只更新目标。
	bool bUseSuspensionConstraint = false;
	FPhysicsConstraintHandle SuspensionConstraintHandle;
	Chaos::FSuspensionConstraint* SuspensionConstraint = nullptr;

	// 本步射线命中的地面物理代理（非拥有指针）。
	IPhysicsProxyBase* GroundProxy = nullptr;

	// 地面代理是否可被原生悬挂约束使用（引擎只接受单粒子代理）。
	bool bGroundSupportsConstraint = false;
};

/** 按压缩量线性插值刚度倍率；曲线为空时返回 1。 */
inline float EvaluateSuspensionStiffnessScale(
	const TArray<FCirrovaSuspensionCurveRuntimePoint>& Curve,
	const float Compression
)
{
	if (Curve.IsEmpty()) { return 1.0f; }
	if (Compression <= Curve[0].Compression) { return Curve[0].StiffnessScale; }

	for (int32 Index = 1; Index < Curve.Num(); ++Index)
	{
		if (Compression > Curve[Index].Compression) { continue; }

		const float Span = Curve[Index].Compression - Curve[Index - 1].Compression;
		const float Alpha = Span > KINDA_SMALL_NUMBER
			? (Compression - Curve[Index - 1].Compression) / Span
			: 0.0f;
		return FMath::Lerp(Curve[Index - 1].StiffnessScale, Curve[Index].StiffnessScale, Alpha);
	}
	return Curve.Last().StiffnessScale;
}

/** 当前压缩量下的弹簧刚度：基础刚度 × 曲线倍率 + 渐进限位刚度。 */
inline float EvaluateSuspensionStiffness(
	const FCirrovaWheelRuntime& Wheel,
	const float Compression
)
{
	float Stiffness = Wheel.SuspensionStiffness
		* EvaluateSuspensionStiffnessScale(Wheel.SuspensionCurve, Compression);
	if (Wheel.BumpStopStiffness > 0.0f && Compression > Wheel.BumpStopStart)
	{
		Stiffness += Wheel.BumpStopStiffness;
	}
	return FMath::Max(Stiffness, 0.0f);
}

/** 按压缩速度方向选择压缩或回弹阻尼；0 时退回基础阻尼。 */
inline float SelectSuspensionDamping(
	const FCirrovaWheelRuntime& Wheel,
	const float CompressionVelocity
)
{
	const float Selected = CompressionVelocity >= 0.0f
		? Wheel.CompressionDamping
		: Wheel.ReboundDamping;
	return Selected > 0.0f ? Selected : Wheel.SuspensionDamping;
}

/** 防倾杆运行态：成对车轮的附加法向力，物理线程读写。 */
struct FCirrovaAntiRollBarRuntime
{
	int32 WheelA = 0;
	int32 WheelB = 1;
	float Stiffness = 0.0f;
};

/** 载具物理运行态：构建完成后归物理线程所有。 */
struct FCirrovaVehicleRuntime
{
	FCirrovaVehicleRuntime() = default;
	FCirrovaVehicleRuntime(const FCirrovaVehicleRuntime&) = delete;
	FCirrovaVehicleRuntime& operator=(const FCirrovaVehicleRuntime&) = delete;

	~FCirrovaVehicleRuntime() { if (Handle) { liminal_vehicle_free(Handle); } }

	// liminal 句柄。构建于游戏线程，注册后仅由物理线程访问；
	// 销毁经求解器命令队列在物理线程执行，全程无并发访问。
	FLiminalVehicleHandle Handle = nullptr;

	TArray<FCirrovaWheelRuntime> Wheels;

	// 防倾杆：成对车轮索引与扭转刚度（N/m）。
	TArray<FCirrovaAntiRollBarRuntime> AntiRollBars;

	float ChassisMass = 0.0f;

	// 轮胎以求解器约束耦合；容器不可用时为 false（与原生悬挂开关解耦）。
	bool bUseTireConstraint = false;

	// 允许底盘休眠；false 时物理线程每步保持唤醒。
	bool bAllowPhysicsSleep = true;

	// 强制悬挂走射线弹簧回退（忽略地面代理类型），用于验证 Landscape 回退路径。
	bool bForceSuspensionFallback = false;

	// 本步耦合诊断：外力回退的内部子步数、求解器步长与轮胎约束形态。
	int32 AppliedSubsteps = 1;
	float SolverDeltaTime = 0.0f;
	bool bTireConstraintCoupling = false;
	bool bSubstepBudgetExceeded = false;

	// 最大转向角（弧度）。
	float MaxSteeringAngle = 0.0f;

	// 车轮接触检测探针形态。
	ECirrovaWheelProbeMode WheelProbeMode = ECirrovaWheelProbeMode::Cylinder;

	// 轮胎纵/侧向松弛长度（m）；0 关闭。
	float TireRelaxationLength = 0.3f;

	// ABS 参数：进入/退出滑移阈值、泄压/建压速率（每秒）、最低生效车速（m/s）。
	float ABSSlipThreshold = 0.3f;
	float ABSReleaseThreshold = 0.15f;
	float ABSReleaseRate = 40.0f;
	float ABSApplyRate = 60.0f;
	float ABSMinSpeed = 2.0f;

	// TCS 参数：滑移阈值与泄压速率（每秒）。
	float TCSSlipThreshold = 0.20f;
	float TCSReleaseRate = 10.0f;

	// 逐轮转向节信号端口（转向节端口索引 2）。
	TArray<uint64> SteeringSignalPorts;

	// 参考转向角（rad；liminal 约定正值为右转），受转向速率限制。
	float SteeringAngleRad = 0.0f;

	// 阿克曼转向与转向速率限制。
	bool bEnableAckermannSteering = true;
	float SteeringRateLimitRad = 0.0f;

	// 手刹：最大扭矩（N·m）与是否仅后轴。
	float MaxHandbrakeTorque = 0.0f;
	bool bHandbrakeRearOnly = true;

	// 发动机制动曲线（启用时 liminal 内置线性制动置 0）。
	bool bUseEngineBrakingCurve = false;
	// 怠速角速度（rad/s），用于判定是否进入发动机制动区间。
	float EngineIdleAngularVelocity = 0.0f;
	TArray<FCirrovaEngineBrakingRuntimePoint> EngineBrakingCurve;

	// 扭矩矢量：开关与强度（N·m）。
	bool bEnableTorqueVectoring = false;
	float TorqueVectoringStrength = 0.0f;

	// 最近一步的驾驶输入缓存（扭矩矢量用）。
	float LastThrottleInput = 0.0f;
	float LastSteeringInput = 0.0f;

	// 空气动力学：阻力与下压力（SI）。
	bool bEnableAerodynamics = true;
	float AeroAirDensity = 1.225f;
	float AeroDragCoefficient = 0.35f;
	float AeroFrontalArea = 2.2f;
	float AeroDownforceCoefficient = 0.8f;
	FVector AeroCenterOfPressure = FVector::ZeroVector;

	float TireFriction = 1.0f;
	float TireLateralFrictionScale = 1.0f;
	float TireStaticFrictionScale = 1.0f;
	float TireLateralStiffness = 0.0f;
	float TireLoadSensitivity = 0.0f;
	float TireReferenceLoad = 0.0f;
	float TireRollingResistance = 0.0f;
	float WheelInertia = 0.0f;
	// 逐轮制动扭矩容量（N·m）；含前后轴比例。
	TArray<float> MaxBrakeTorquePerWheel;

	// 逐轮转向节制动力矩容量（N·m）：脚刹与手刹容量的较大值。
	TArray<float> KnuckleBrakeTorquePerWheel;

	// 逐轮转向节制动信号端口；P1.4 起信号经此直写，不再经过制动分发器。
	TArray<uint64> BrakeSignalPorts;

	// ABS 逐轮调制（[0,1]，1 为全压）；物理线程读写，游戏线程只读遥测。
	TArray<float> ABSModulation;

	// 逐轮是否连接驱动链（决定 TCS 滑移监控范围）。
	TArray<bool> DrivenWheels;

	// TCS 油门倍率（[0,1]）；驱动轮滑移超限时泄压。
	float TCSModulation = 1.0f;

	// 是否启用 ABS 与 TCS。
	bool bEnableABS = false;
	bool bEnableTCS = false;

	uint64 ThrottleSignalPort = 0;
	uint64 GearSignalPort = 0;
	uint64 EngineShaftPort = 0;
};

/** 游戏线程写入、物理线程消费的单载具输入。 */
struct FCirrovaVehicleAsyncInput
{
	int32 VehicleId = INDEX_NONE;
	FCirrovaVehicleRuntime* Runtime = nullptr;
	IPhysicsProxyBase* Proxy = nullptr;
	// 游戏线程收集的宽相候选，物理线程只做窄相查询。
	TArray<Chaos::FConstPhysicsObjectHandle> BlockingObjects;
	float Throttle = 0.0f;
	float Steering = 0.0f;
	float Brake = 0.0f;
	float Handbrake = 0.0f;
	int32 Gear = 0;
};

/** 单轮输出快照，游戏线程只读。 */
struct FCirrovaWheelOutput
{
	bool bInContact = false;
	float Omega = 0.0f;
	float SurfaceSpeed = 0.0f;
	float EffectiveInertia = 0.0f;
	float ConstraintWheelInverseMass = 0.0f;
	float SuspensionLength = 0.0f;
	float SuspensionForce = 0.0f;
	float AntiRollForce = 0.0f;
	float NormalLoad = 0.0f;
	float Compression = 0.0f;
	float CompressionVelocity = 0.0f;
	bool bCompressionVelocityClamped = false;
	bool bNativeSuspensionActive = false;
	bool bGroundProxyUnsupported = false;
	bool bUsingConstraintLength = false;
	float SuspensionConstraintLength = 0.0f;
	float TireLongitudinalForce = 0.0f;
	float TireLateralForce = 0.0f;
	float WheelTorque = 0.0f;
	float RollingResistance = 0.0f;
	float FrictionForceLimit = 0.0f;
	float LateralForceLimit = 0.0f;
	float StaticFrictionForceLimit = 0.0f;
	float StaticLateralForceLimit = 0.0f;
	float KineticFrictionForceLimit = 0.0f;
	float KineticLateralForceLimit = 0.0f;
	float GroundFriction = 1.0f;
	float GroundStaticFriction = 1.0f;
	float RelaxedSurfaceSpeed = 0.0f;
	float FrictionUtilization = 0.0f;
	bool bTireConstraintApplied = false;
	bool bWheelLocked = false;
	float BrakeTorqueCapacity = 0.0f;
	float ABSModulation = 1.0f;
	float SteeringAngleDegrees = 0.0f;
	float LongitudinalImpulse = 0.0f;
	float LateralImpulse = 0.0f;
	float ImpulseWindow = 0.0f;
	float ContactLongitudinalSpeed = 0.0f;
	float ContactLateralSpeed = 0.0f;
	float SlipRatio = 0.0f;
	float SlipAngleDegrees = 0.0f;
	float RelaxedSlipAngleDegrees = 0.0f;
	FVector SuspensionAxis = FVector::UpVector;
	FVector ContactPoint = FVector::ZeroVector;
	FVector ContactNormal = FVector::UpVector;
	FVector LongitudinalAxis = FVector::ForwardVector;
	FVector LateralAxis = FVector::RightVector;
	FVector ContactVelocity = FVector::ZeroVector;
	FVector GroundVelocity = FVector::ZeroVector;
};

/** 单载具输出快照，游戏线程只读。 */
struct FCirrovaVehicleOutput
{
	int32 VehicleId = INDEX_NONE;
	bool bGrounded = false;
	bool bSubstepBudgetExceeded = false;
	bool bTireConstraintCoupling = false;
	int32 AppliedSubsteps = 1;
	float SolverDeltaTime = 0.0f;
	float EngineOmega = 0.0f;
	TArray<FCirrovaWheelOutput> Wheels;
};
