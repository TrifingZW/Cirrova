#pragma once

#include "CoreMinimal.h"
#include "CirrovaTestHarness.h"
#include "Subsystems/WorldSubsystem.h"
#include "CirrovaTestWorldSubsystem.generated.h"

class FCirrovaTestScenario;

/**
 * P0 自动化测试世界子系统。
 *
 * 通过命令行 -CirrovaTest=<all|T0,T1,...> 启用，在游戏世界中创建受控地面与载具，
 * 按场景顺序执行并输出 CSV/JSON 报告；完成后按 -CirrovaTestExit 请求退出。
 */
UCLASS()
class UCirrovaTestWorldSubsystem final : public UTickableWorldSubsystem
{
	GENERATED_BODY()

public:
#pragma region Subsystem Interface

	virtual bool ShouldCreateSubsystem(UObject* Outer) const override;
	virtual bool DoesSupportWorldType(const EWorldType::Type WorldType) const override;
	virtual void Initialize(FSubsystemCollectionBase& Collection) override;
	virtual void OnWorldBeginPlay(UWorld& InWorld) override;
	virtual void Deinitialize() override;
	virtual void Tick(float DeltaTime) override;
	virtual TStatId GetStatId() const override;
	virtual bool IsTickable() const override;

#pragma endregion

private:
#pragma region Internal Function

	void ParseCommandLine();
	void StartNextScenario();
	void FinishActiveScenario();
	void FinishAll();
	void RequestExitIfConfigured(bool bAllPassed) const;

#pragma endregion

#pragma region State

	TArray<FString> ScenarioQueue;
	int32 NextScenarioIndex = 0;
	TUniquePtr<FCirrovaTestHarness> Harness;
	TUniquePtr<FCirrovaTestScenario> ActiveScenario;
	TArray<FCirrovaTestScenarioResult> Results;

	FString OutputDir;
	FString ScenarioSpec;
	bool bEnabled = false;
	bool bQuitWhenDone = true;
	bool bFinishedAll = false;
	float CooldownRemaining = 0.0f;
	float TotalElapsed = 0.0f;
	float MaxTotalSeconds = 900.0f;

#pragma endregion
};
