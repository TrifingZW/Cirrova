#pragma once

#include "CoreMinimal.h"
#include "CirrovaVehicleRuntime.h"
#include "Chaos/SimCallbackInput.h"
#include "Chaos/SimCallbackObject.h"

namespace Chaos
{
	class FCollisionContactModifier;
}

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

	void Reset() { Vehicles.Reset(); }
};

/** Chaos 物理线程回调：在求解器步内推进 liminal，写入或施加轮胎耦合。 */
class FCirrovaAsyncCallback final : public Chaos::TSimCallbackObject<
		FCirrovaAsyncInput,
		FCirrovaAsyncOutput,
		Chaos::ESimCallbackOptions::Presimulate>
{
public:
	virtual void OnPreSimulate_Internal() override;
	virtual void OnContactModification_Internal(Chaos::FCollisionContactModifier& Modifier) override;
};
