#pragma once

#include "CoreMinimal.h"
#include "CirrovaVehicleRuntime.h"
#include "Subsystems/WorldSubsystem.h"
#include "CirrovaSchedulerSubsystem.generated.h"

class FChaosScene;
class FPhysScene_Chaos;
class FCirrovaAsyncCallback;
class FCirrovaWheelFrictionConstraints;
class UCirrovaVehicleComponent;

/** 世界级调度器：注册 Chaos 异步物理回调，编组载具输入与输出。 */
UCLASS(NotBlueprintable, BlueprintType)
class CIRROVA_API UCirrovaSchedulerSubsystem : public UWorldSubsystem
{
	GENERATED_BODY()

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

	// 每个物理场景共享一份轮胎摩擦约束容器；随场景存活，子系统销毁时移交延迟回收。
	TUniquePtr<FCirrovaWheelFrictionConstraints> WheelConstraints;

	FPhysScene_Chaos* PhysicsScene = nullptr;

	FDelegateHandle OnPhysScenePreTickHandle;
	FDelegateHandle OnPhysScenePostTickHandle;

	int32 NextVehicleId = 0;

	// P0 性能基线：上次输出物理统计的墙钟时间。
	double PhysicsStatsLastLogSeconds = 0.0;

	// P0 物理步耗时（ms）：最近、峰值与累计平均。
	float LastStepWallMs = 0.0f;
	float PeakStepWallMs = 0.0f;
	double StepWallMsSum = 0.0;
	int64 StepWallMsSamples = 0;

public:
#pragma region Subsystem Interface

	virtual ~UCirrovaSchedulerSubsystem() override;
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

	// 已注册载具数量（仅游戏线程）。
	int32 GetRegisteredVehicleCount() const { return Vehicles.Num(); }

	// 仍绑定粒子的轮胎约束数量（仅游戏线程）。
	int32 GetActiveWheelConstraintCount() const;

	// 物理步墙钟耗时统计（ms，仅游戏线程）。
	float GetLastPhysicsStepMs() const { return LastStepWallMs; }
	float GetAveragePhysicsStepMs() const;
	float GetPeakPhysicsStepMs() const { return PeakStepWallMs; }

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
