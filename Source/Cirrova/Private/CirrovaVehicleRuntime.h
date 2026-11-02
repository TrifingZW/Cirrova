#pragma once

#include "CoreMinimal.h"
#include "CirrovaLiminalApi.h"
#include "Chaos/ChaosEngineInterface.h"
#include "CollisionQueryParams.h"
#include "Engine/EngineTypes.h"

class AActor;
class IPhysicsProxyBase;

namespace Chaos
{
	class FSuspensionConstraint;
}

/** 单轮物理运行态，仅在物理线程读写。 */
struct FCirrovaWheelRuntime
{
	// 悬挂上挂点（底盘本地空间，cm）。
	FVector MountLocation = FVector::ZeroVector;

	// 车轮旋转轴端口（读角速度、注入反作用力矩）。
	uint64 ShaftPort = 0;

	// 转向控制端口，0 表示不可转向。
	uint64 SteeringPort = 0;

	// 是否参与转向（决定接触坐标系的转向角）。
	bool bSteerable = false;

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

	// 上一步压缩量（m）。
	float PreviousCompression = 0.0f;

	// 首次求解标记：跳过一次压缩速度计算，避免出生时的阻尼冲击。
	bool bSolved = false;

	// 车轮角速度（rad/s）。
	float Omega = 0.0f;

	// 接触解（世界坐标与接触坐标系）。
	bool bInContact = false;
	FVector ContactPoint = FVector::ZeroVector;
	FVector ContactNormal = FVector::UpVector;
	FVector LongitudinalAxis = FVector::ForwardVector;
	FVector LateralAxis = FVector::RightVector;

	// 接触点速度缓存（世界坐标，cm/s）。
	FVector ContactVelocity = FVector::ZeroVector;

	// 本求解器步内各子步合力累计（世界坐标，N），施加后清零。
	FVector AccumulatedForce = FVector::ZeroVector;

	// 本步力解（SI，供输出快照）。
	float SuspensionForce = 0.0f;
	FLiminalTireForces TireForces{};

	// 原生悬挂约束：句柄在游戏线程创建与释放，物理线程只更新目标。
	bool bUseSuspensionConstraint = false;
	FPhysicsConstraintHandle SuspensionConstraintHandle;
	Chaos::FSuspensionConstraint* SuspensionConstraint = nullptr;

	// 本步射线命中的地面物理代理（非拥有指针）。
	IPhysicsProxyBase* GroundProxy = nullptr;
};

/** 载具物理运行态：构建完成后归物理线程所有。 */
struct FCirrovaVehicleRuntime
{
	FCirrovaVehicleRuntime() = default;
	FCirrovaVehicleRuntime(const FCirrovaVehicleRuntime&) = delete;
	FCirrovaVehicleRuntime& operator=(const FCirrovaVehicleRuntime&) = delete;

	~FCirrovaVehicleRuntime()
	{
		if (Handle) { liminal_vehicle_free(Handle); }
	}

	// liminal 句柄。构建于游戏线程，注册后仅由物理线程访问；
	// 销毁经求解器命令队列在物理线程执行，全程无并发访问。
	FLiminalVehicleHandle Handle = nullptr;

	TArray<FCirrovaWheelRuntime> Wheels;

	float ChassisMass = 0.0f;

	// 最大转向角（弧度）。
	float MaxSteeringAngle = 0.0f;

	float TireFriction = 1.0f;
	float TireLateralStiffness = 0.0f;
	float WheelInertia = 0.0f;

	// 车轮射线碰撞通道。
	ECollisionChannel WheelTraceChannel = ECC_Visibility;

	uint64 ThrottleSignalPort = 0;
	uint64 SteeringSignalPort = 0;
	uint64 GearSignalPort = 0;
	uint64 BrakeSignalPort = 0;
	uint64 EngineShaftPort = 0;
};

/** 游戏线程写入、物理线程消费的单载具输入。 */
struct FCirrovaVehicleAsyncInput
{
	int32 VehicleId = INDEX_NONE;
	FCirrovaVehicleRuntime* Runtime = nullptr;
	IPhysicsProxyBase* Proxy = nullptr;
	FCollisionQueryParams TraceParams;
	float Throttle = 0.0f;
	float Steering = 0.0f;
	float Brake = 0.0f;
	int32 Gear = 0;
};

/** 单轮输出快照，游戏线程只读。 */
struct FCirrovaWheelOutput
{
	bool bInContact = false;
	float Omega = 0.0f;
	float SuspensionLength = 0.0f;
	float SuspensionForce = 0.0f;
	FLiminalTireForces TireForces{};
	FVector ContactPoint = FVector::ZeroVector;
	FVector ContactNormal = FVector::UpVector;
	FVector LongitudinalAxis = FVector::ForwardVector;
	FVector LateralAxis = FVector::RightVector;
};

/** 单载具输出快照，游戏线程只读。 */
struct FCirrovaVehicleOutput
{
	int32 VehicleId = INDEX_NONE;
	bool bGrounded = false;
	float EngineOmega = 0.0f;
	TArray<FCirrovaWheelOutput> Wheels;
};
