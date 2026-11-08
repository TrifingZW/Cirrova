#pragma once

#include <cstddef>

#include "CoreMinimal.h"

// liminal C ABI 绑定，与 ThirdParty/liminal/src/capi.rs 一一对应。
// 修改任何结构体布局或函数签名时须同步递增 LiminalAbiVersion 与 Rust 侧常量。
constexpr uint32 LiminalAbiVersion = 2;
// 契约见 ThirdParty/liminal/docs/adapter-contract.md。
// 使用前须先构建 Rust 静态库：cd ThirdParty/liminal && cargo build --release。

enum class ELiminalStatus : int32
{
	Ok = 0,
	InvalidHandle = 1,
	NodeNotFound = 2,
	PortNotFound = 3,
	SelfConnection = 4,
	DomainMismatch = 5,
	DirectionIncompatible = 6,
	CapacityReached = 7,
	CompileFailed = 8,
	WrongPayload = 9,
	IndexOutOfRange = 10,
	InvalidArgument = 11,
};

enum class ELiminalJointKind : int32
{
	Rigid = 0,
	Revolute = 1,
	PrismaticSpring = 2,
};

struct FLiminalVec3
{
	float X;
	float Y;
	float Z;
};

struct FLiminalTireForces
{
	float Longitudinal;
	float Lateral;
	// 摩擦对车轮的反作用力矩（N·m），不含滚动阻力。
	float WheelTorque;
};

// 轮胎模型参数：载荷敏感与滚动阻力非正值时关闭。
struct FLiminalTireModel
{
	float FrictionCoefficient;
	float LateralStiffness;
	float LoadSensitivity;
	float ReferenceLoad;
	float RollingResistance;
};

// 约束式轮胎耦合的力界与外部力（N）。
struct FLiminalTireConstraint
{
	// 载荷敏感后的摩擦圆上限，作为纵向与侧向冲量的共同界。
	float FrictionForce;
	// 侧向力（含低速速度阻尼，已按摩擦圆裁剪）。
	float LateralForce;
	// 滚动阻力（逆向地面速度，不产生车轮反作用）。
	float RollingForce;
};

struct FLiminalJointDesc
{
	ELiminalJointKind Kind;
	FLiminalVec3 Axis;
	float Stiffness;
	float Damping;
	float RestLength;
	float MinLength;
	float MaxLength;
};

// 发动机全油门扭矩曲线采样点：归一化角速度与扭矩比例。
struct FLiminalEngineCurvePoint
{
	float SpeedRatio;
	float TorqueScale;
};

// 发动机参数：曲线为空时线性衰减；怠速与制动参数为 0 时关闭。
struct FLiminalEngineParams
{
	float MaxTorque;
	float MaxAngularVelocity;
	float Inertia;
	float IdleAngularVelocity;
	float IdleTorqueLimit;
	float EngineBrakingTorque;
	const FLiminalEngineCurvePoint* Curve;
	size_t CurveCount;
};

// 刚体变换：旋转分量顺序为 (x, y, z, w)。
struct FLiminalFrame
{
	FLiminalVec3 Translation;
	float Rotation[4];
};

struct FLiminalSignalOp
{
	uint64 SourcePort;
	uint64 TargetPort;
};

struct FLiminalSpatialLink
{
	uint64 Parent;
	uint64 Child;
	FLiminalJointDesc Joint;
	uint64 ControlPort;
	uint32 HasControl;
	FLiminalFrame RestPose;
	FLiminalFrame PoseInRoot;
};

using FLiminalVehicleHandle = void*;

// 布局校验：Rust 侧为 #[repr(C)]，字段偏移必须与 C++ 侧一致。
static_assert(sizeof(FLiminalVec3) == 12, "FLiminalVec3 布局漂移");
static_assert(sizeof(FLiminalTireForces) == 12, "FLiminalTireForces 布局漂移");
static_assert(sizeof(FLiminalTireModel) == 20, "FLiminalTireModel 布局漂移");
static_assert(sizeof(FLiminalTireConstraint) == 12, "FLiminalTireConstraint 布局漂移");
static_assert(sizeof(FLiminalJointDesc) == 36, "FLiminalJointDesc 布局漂移");
static_assert(sizeof(FLiminalEngineCurvePoint) == 8, "FLiminalEngineCurvePoint 布局漂移");
static_assert(sizeof(FLiminalFrame) == 28, "FLiminalFrame 布局漂移");
static_assert(sizeof(FLiminalSignalOp) == 16, "FLiminalSignalOp 布局漂移");
static_assert(offsetof(FLiminalSpatialLink, Joint) == 16, "FLiminalSpatialLink.Joint 偏移漂移");
static_assert(sizeof(FLiminalEngineParams) == 40, "FLiminalEngineParams 布局漂移");
static_assert(offsetof(FLiminalEngineParams, Curve) == 24, "FLiminalEngineParams.Curve 偏移漂移");
static_assert(offsetof(FLiminalSpatialLink, ControlPort) == 56, "FLiminalSpatialLink.ControlPort 偏移漂移");
static_assert(offsetof(FLiminalSpatialLink, HasControl) == 64, "FLiminalSpatialLink.HasControl 偏移漂移");
static_assert(offsetof(FLiminalSpatialLink, RestPose) == 68, "FLiminalSpatialLink.RestPose 偏移漂移");
static_assert(offsetof(FLiminalSpatialLink, PoseInRoot) == 96, "FLiminalSpatialLink.PoseInRoot 偏移漂移");
static_assert(sizeof(FLiminalSpatialLink) == 128, "FLiminalSpatialLink 布局漂移");

extern "C" {
uint32 liminal_abi_version();
FLiminalVehicleHandle liminal_vehicle_new();
void liminal_vehicle_free(FLiminalVehicleHandle Vehicle);

ELiminalStatus liminal_vehicle_add_cockpit(FLiminalVehicleHandle Vehicle, uint64* OutNode);
ELiminalStatus liminal_vehicle_add_engine(
	FLiminalVehicleHandle Vehicle,
	float MaxTorque,
	float MaxAngularVelocity,
	float Inertia,
	uint64* OutNode
);
ELiminalStatus liminal_vehicle_add_engine_with_params(
	FLiminalVehicleHandle Vehicle,
	const FLiminalEngineParams* Params,
	uint64* OutNode
);
ELiminalStatus liminal_vehicle_add_gearbox(
	FLiminalVehicleHandle Vehicle,
	const float* Ratios,
	size_t RatioCount,
	float Inertia,
	uint64* OutNode
);
ELiminalStatus liminal_vehicle_add_axle(
	FLiminalVehicleHandle Vehicle,
	size_t OutputCount,
	float Inertia,
	uint64* OutNode
);
ELiminalStatus liminal_vehicle_add_differential(
	FLiminalVehicleHandle Vehicle,
	size_t OutputCount,
	float MaxTorque,
	float Inertia,
	uint64* OutNode
);
ELiminalStatus liminal_vehicle_add_wheel(
	FLiminalVehicleHandle Vehicle,
	float Inertia,
	float FrictionTorque,
	float Radius,
	uint64* OutNode
);
ELiminalStatus liminal_vehicle_add_brake(FLiminalVehicleHandle Vehicle, float MaxTorque, uint64* OutNode);
ELiminalStatus liminal_vehicle_add_clutch(
	FLiminalVehicleHandle Vehicle,
	float MaxTorque,
	float Inertia,
	uint64* OutNode
);
ELiminalStatus liminal_vehicle_add_chassis(
	FLiminalVehicleHandle Vehicle,
	const FLiminalFrame* Mounts,
	size_t MountCount,
	uint64* OutNode
);
ELiminalStatus liminal_vehicle_add_suspension(
	FLiminalVehicleHandle Vehicle,
	float Stiffness,
	float Damping,
	float RestLength,
	float MinLength,
	float MaxLength,
	uint64* OutNode
);
ELiminalStatus liminal_vehicle_add_knuckle(FLiminalVehicleHandle Vehicle, float MaxBrakeTorque, uint64* OutNode);
ELiminalStatus liminal_vehicle_add_splitter(FLiminalVehicleHandle Vehicle, size_t OutputCount, uint64* OutNode);

ELiminalStatus liminal_vehicle_port_count(FLiminalVehicleHandle Vehicle, uint64 Node, size_t* OutCount);
ELiminalStatus liminal_vehicle_port(FLiminalVehicleHandle Vehicle, uint64 Node, size_t Index, uint64* OutPort);
ELiminalStatus liminal_vehicle_connect(
	FLiminalVehicleHandle Vehicle,
	uint64 SourcePort,
	uint64 TargetPort,
	uint64* OutEdge
);
ELiminalStatus liminal_vehicle_disconnect(FLiminalVehicleHandle Vehicle, uint64 Edge);
ELiminalStatus liminal_vehicle_remove_node(FLiminalVehicleHandle Vehicle, uint64 Node);

int32 liminal_vehicle_is_dirty(FLiminalVehicleHandle Vehicle);
ELiminalStatus liminal_vehicle_recompile(FLiminalVehicleHandle Vehicle);
ELiminalStatus liminal_vehicle_step(FLiminalVehicleHandle Vehicle, float Dt);

ELiminalStatus liminal_vehicle_write_signal(FLiminalVehicleHandle Vehicle, uint64 Port, float Value);
ELiminalStatus liminal_vehicle_write_torque(FLiminalVehicleHandle Vehicle, uint64 Port, float Torque);
ELiminalStatus liminal_vehicle_read_signal(FLiminalVehicleHandle Vehicle, uint64 Port, float* OutValue);
ELiminalStatus liminal_vehicle_read_omega(FLiminalVehicleHandle Vehicle, uint64 Port, float* OutOmega);
// 轮端等效转动惯量（kg·m²），包含传动链折算惯量；端口不在动力岛内时返回 PortNotFound。
ELiminalStatus liminal_vehicle_effective_inertia(
	FLiminalVehicleHandle Vehicle,
	uint64 Port,
	float* OutInertia
);

ELiminalStatus liminal_vehicle_signal_op_count(FLiminalVehicleHandle Vehicle, size_t* OutCount);
ELiminalStatus liminal_vehicle_signal_op(FLiminalVehicleHandle Vehicle, size_t Index, FLiminalSignalOp* OutOp);
ELiminalStatus liminal_vehicle_spatial_link_count(FLiminalVehicleHandle Vehicle, size_t* OutCount);
ELiminalStatus liminal_vehicle_spatial_link(FLiminalVehicleHandle Vehicle, size_t Index, FLiminalSpatialLink* OutLink);

// 轮胎冲量解：LateralSpeed 为接触点侧向相对速度（m/s），用于低速侧向保持。
FLiminalTireForces liminal_tire_impulse_forces(
	const FLiminalTireModel* Model,
	float AngularVelocity,
	float GroundSpeed,
	float Radius,
	float WheelInertia,
	float SlipAngle,
	float LateralSpeed,
	float NormalForce,
	float Dt
);
float liminal_suspension_force(float Stiffness, float Damping, float Compression, float CompressionVelocity);
float liminal_anti_roll_force(float Stiffness, float CompressionA, float CompressionB);
// 约束式轮胎耦合：Sideslip 为侧偏角（弧度），LateralSpeed 与 GroundSpeed 为接触点相对速度（m/s）。
FLiminalTireConstraint liminal_tire_constraint(
	const FLiminalTireModel* Model,
	float GroundSpeed,
	float Sideslip,
	float LateralSpeed,
	float NormalForce
);
}
