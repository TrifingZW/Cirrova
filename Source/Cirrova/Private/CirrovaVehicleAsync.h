#pragma once

#include "CoreMinimal.h"
#include "CirrovaVehicleRuntime.h"
#include "Chaos/SimCallbackInput.h"
#include "Chaos/SimCallbackObject.h"

class UWorld;

namespace Chaos
{
	class FCollisionContactModifier;
}

/** 物理线程输入：本物理步要推进的所有载具。 */
struct FCirrovaAsyncInput : Chaos::FSimCallbackInput
{
	TArray<FCirrovaVehicleAsyncInput> Vehicles;
	TWeakObjectPtr<UWorld> World;

	void Reset()
	{
		Vehicles.Reset();
		World.Reset();
	}
};

/** 物理线程输出：本物理步推进后的载具快照。 */
struct FCirrovaAsyncOutput : Chaos::FSimCallbackOutput
{
	TArray<FCirrovaVehicleOutput> Vehicles;

	void Reset() { Vehicles.Reset(); }
};

/** Chaos 物理线程回调：在求解器子步内推进 liminal 并施加力。 */
class FCirrovaAsyncCallback final : public Chaos::TSimCallbackObject<
		FCirrovaAsyncInput,
		FCirrovaAsyncOutput,
		Chaos::ESimCallbackOptions::Presimulate>
{
public:
	virtual void OnPreSimulate_Internal() override;
	virtual void OnContactModification_Internal(Chaos::FCollisionContactModifier& Modifier) override;
};
