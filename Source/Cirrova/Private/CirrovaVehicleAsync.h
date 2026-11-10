#pragma once

#include "CoreMinimal.h"
#include "CirrovaVehicleRuntime.h"
#include "Chaos/SimCallbackInput.h"
#include "Chaos/SimCallbackObject.h"

class FCirrovaWheelFrictionConstraints;

/** 物理线程输入：本物理步要推进的所有载具。 */
struct FCirrovaAsyncInput : Chaos::FSimCallbackInput
{
	TArray<FCirrovaVehicleAsyncInput> Vehicles;

	// 共享的轮胎摩擦约束容器；为空时载具回退为外力耦合。
	FCirrovaWheelFrictionConstraints* WheelConstraints = nullptr;

	void Reset()
	{
		Vehicles.Reset();
		WheelConstraints = nullptr;
	}
};

/** 物理线程输出：本物理步推进后的载具快照。 */
struct FCirrovaAsyncOutput : Chaos::FSimCallbackOutput
{
	TArray<FCirrovaVehicleOutput> Vehicles;

	// P0 性能基线：本步各阶段耗时（ms）与载具数量。
	float PreSimulateMs = 0.0f;
	float StepWallMs = 0.0f;
	float PostSolveMs = 0.0f;
	int32 VehicleCount = 0;

	void Reset() { Vehicles.Reset(); }
};

/**
 * Chaos 物理线程回调：在求解器步内准备并收尾 liminal 载具。
 *
 * Presimulate 完成接触、悬挂目标与轮胎约束输入；PostSolve 消费同一物理步
 * 求解出的轮胎冲量，回注轮端力矩并推进一维动力流，使轮速不再跨帧滞后。
 */
class FCirrovaAsyncCallback final : public Chaos::TSimCallbackObject<
		FCirrovaAsyncInput,
		FCirrovaAsyncOutput,
		Chaos::ESimCallbackOptions::Presimulate | Chaos::ESimCallbackOptions::PostSolve>
{
public:
	virtual void OnPreSimulate_Internal() override;
	virtual void OnPostSolve_Internal() override;

private:
	/** 本物理步需要收尾的载具；指针在求解结束后仍有效（注销延迟到物理帧末）。 */
	struct FPendingVehicle
	{
		FCirrovaVehicleRuntime* Runtime = nullptr;
		IPhysicsProxyBase* Proxy = nullptr;
		int32 VehicleId = INDEX_NONE;
	};

	TArray<FPendingVehicle> PendingVehicles;
	FCirrovaWheelFrictionConstraints* PendingWheelConstraints = nullptr;
	float PendingDeltaTime = 0.0f;
	double PendingStepStartSeconds = 0.0;
	float PendingPreSimulateMs = 0.0f;
};
