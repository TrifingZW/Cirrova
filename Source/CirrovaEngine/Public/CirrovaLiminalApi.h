#pragma once

#include "CoreMinimal.h"

// liminal C ABI 绑定，与 ThirdParty/liminal/src/capi.rs 一一对应。
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

extern "C" {
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
	float Coupling,
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
	float LockThreshold,
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

ELiminalStatus liminal_vehicle_signal_op_count(FLiminalVehicleHandle Vehicle, size_t* OutCount);
ELiminalStatus liminal_vehicle_signal_op(FLiminalVehicleHandle Vehicle, size_t Index, FLiminalSignalOp* OutOp);
ELiminalStatus liminal_vehicle_spatial_link_count(FLiminalVehicleHandle Vehicle, size_t* OutCount);
ELiminalStatus liminal_vehicle_spatial_link(FLiminalVehicleHandle Vehicle, size_t Index, FLiminalSpatialLink* OutLink);

FLiminalTireForces liminal_tire_forces(
	float FrictionCoefficient,
	float LongitudinalStiffness,
	float LateralStiffness,
	float SlipRatio,
	float SlipAngle,
	float NormalForce
);
FLiminalTireForces liminal_tire_impulse_forces(
	float FrictionCoefficient,
	float LateralStiffness,
	float AngularVelocity,
	float GroundSpeed,
	float Radius,
	float WheelInertia,
	float SlipAngle,
	float NormalForce,
	float Dt
);
float liminal_suspension_force(float Stiffness, float Damping, float Compression, float CompressionVelocity);
}
