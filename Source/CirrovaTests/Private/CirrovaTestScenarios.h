#pragma once

#include "CoreMinimal.h"
#include "CirrovaTestHarness.h"

/** P0 自动化测试场景基类。 */
class FCirrovaTestScenario
{
public:
	explicit FCirrovaTestScenario(FString InName, float InTimeoutSeconds = 30.0f);
	virtual ~FCirrovaTestScenario() = default;

	FCirrovaTestScenario(const FCirrovaTestScenario&) = delete;
	FCirrovaTestScenario& operator=(const FCirrovaTestScenario&) = delete;

	const FString& GetName() const { return Result.Name; }
	float GetElapsed() const { return Elapsed; }
	float GetTimeoutSeconds() const { return TimeoutSeconds; }
	bool IsFinished() const { return bFinished; }
	const FCirrovaTestScenarioResult& GetResult() const { return Result; }

	void Start(FCirrovaTestHarness& InHarness);
	void Tick(FCirrovaTestHarness& InHarness, float DeltaTime);
	void Abort(const FString& Reason);

protected:
	virtual void OnBegin(FCirrovaTestHarness& Harness) = 0;
	virtual void OnTick(FCirrovaTestHarness& Harness, float DeltaTime) = 0;

	void Pass(const FString& Summary);
	void Fail(const FString& Reason);
	void AddFailure(const FString& Reason);
	void AddCheck(bool bCondition, const FString& FailureReason);
	void AddMetric(const FString& Name, double Value);
	void Complete(const FString& PassSummary);

	// 场景开始后的累计时间（秒）。
	float Elapsed = 0.0f;

private:
	FCirrovaTestScenarioResult Result;
	float TimeoutSeconds = 30.0f;
	bool bFinished = false;
	bool bStarted = false;
};

namespace CirrovaTest
{
	// 全部场景名，按执行顺序排列。
	void GetScenarioNames(TArray<FString>& OutNames);

	// 场景名是否为已知场景。
	bool IsKnownScenario(const FString& Name);

	// 创建场景；未知名称返回空。
	TUniquePtr<FCirrovaTestScenario> CreateScenario(const FString& Name);

	// 分组工厂；由 CirrovaTestScenarios.cpp 汇总。
	TUniquePtr<FCirrovaTestScenario> CreateStaticScenario(const FString& Name);
	TUniquePtr<FCirrovaTestScenario> CreateSlopeScenario(const FString& Name);
	TUniquePtr<FCirrovaTestScenario> CreateDrivetrainScenario(const FString& Name);
}
