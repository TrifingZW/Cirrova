#pragma once

#include "CoreMinimal.h"
#include "CirrovaVehicleRuntime.h"
#include "Subsystems/WorldSubsystem.h"
#include "CirrovaSchedulerSubsystem.generated.h"

class FChaosScene;
class FPhysScene_Chaos;
class FCirrovaAsyncCallback;
class UCirrovaVehicleComponent;

/** 世界级调度器：注册 Chaos 异步物理回调，编组载具输入与输出。 */
UCLASS(NotBlueprintable, BlueprintType)
class CIRROVA_API UCirrovaSchedulerSubsystem : public UWorldSubsystem
{
	GENERATED_BODY()

private:
	struct FRegisteredVehicle
	{
		TWeakObjectPtr<UCirrovaVehicleComponent> Component;
		TUniquePtr<FCirrovaVehicleRuntime> Runtime;
		int32 Id = INDEX_NONE;
	};

	// 已注册载具（仅游戏线程访问）。
	TArray<FRegisteredVehicle> Vehicles;

	// 已注销载具的运行态，待物理步进结束后释放 liminal 句柄。
	TArray<TUniquePtr<FCirrovaVehicleRuntime>> PendingRelease;

	// 注册在 Chaos Solver 上的异步回调。
	FCirrovaAsyncCallback* AsyncCallback = nullptr;

	FPhysScene_Chaos* PhysicsScene = nullptr;

	FDelegateHandle OnPhysScenePreTickHandle;
	FDelegateHandle OnPhysScenePostTickHandle;

	int32 NextVehicleId = 0;

public:
#pragma region Subsystem Interface

	virtual void PostInitialize() override;
	virtual void Deinitialize() override;

#pragma endregion

#pragma region API

	// 注册载具运行态；成功返回 true。运行态所有权转移给调度器。
	bool RegisterVehicle(
		UCirrovaVehicleComponent* Component,
		TUniquePtr<FCirrovaVehicleRuntime> Runtime
	);

	// 注销载具；运行态延迟到物理步进结束后释放。
	void UnregisterVehicle(const UCirrovaVehicleComponent* Component);

#pragma endregion

private:
#pragma region Callback

	void OnPhysScenePreTick(FPhysScene_Chaos* PhysScene, float DeltaTime);
	void OnPhysScenePostTick(FChaosScene* ChaosScene);

#pragma endregion

#pragma region Internal Function

	void BindEvent();
	void UnbindEvent();

	// 将已注销载具的运行态销毁入队到求解器；求解器不可用时在游戏线程释放。
	void ReleasePendingRuntimes();

	// 在游戏线程释放载具的全部原生悬挂约束；幂等。
	void ReleaseSuspensionConstraints(FCirrovaVehicleRuntime& Runtime);

	UCirrovaVehicleComponent* FindComponent(int32 Id) const;

#pragma endregion
};
