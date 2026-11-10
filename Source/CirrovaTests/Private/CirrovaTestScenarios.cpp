#include "CirrovaTestScenarios.h"

#include "CirrovaLog.h"

#pragma region Scenario Base

FCirrovaTestScenario::FCirrovaTestScenario(FString InName, const float InTimeoutSeconds)
	: TimeoutSeconds(InTimeoutSeconds)
{
	Result.Name = MoveTemp(InName);
}

void FCirrovaTestScenario::Start(FCirrovaTestHarness& InHarness)
{
	if (bStarted) { return; }
	bStarted = true;
	OnBegin(InHarness);
}

void FCirrovaTestScenario::Tick(FCirrovaTestHarness& InHarness, const float DeltaTime)
{
	if (bFinished) { return; }
	Elapsed += DeltaTime;
	OnTick(InHarness, DeltaTime);
}

void FCirrovaTestScenario::Abort(const FString& Reason)
{
	if (bFinished) { return; }
	AddFailure(Reason);
	Result.Summary = TEXT("场景中止");
	Result.bPassed = false;
	Result.Duration = Elapsed;
	bFinished = true;
}

void FCirrovaTestScenario::Pass(const FString& Summary)
{
	Result.Summary = Summary;
	Result.bPassed = true;
	Result.Duration = Elapsed;
	bFinished = true;
}

void FCirrovaTestScenario::Fail(const FString& Reason)
{
	if (!Reason.IsEmpty()) { AddFailure(Reason); }
	Result.Summary = TEXT("存在失败项");
	Result.bPassed = false;
	Result.Duration = Elapsed;
	bFinished = true;
}

void FCirrovaTestScenario::AddFailure(const FString& Reason)
{
	Result.Failures.Add(Reason);
	UE_LOG(LogCirrova, Warning, TEXT("Cirrova 测试：%s 失败项：%s"), *Result.Name, *Reason);
}

void FCirrovaTestScenario::AddCheck(const bool bCondition, const FString& FailureReason)
{
	if (!bCondition) { AddFailure(FailureReason); }
}

void FCirrovaTestScenario::AddMetric(const FString& Name, const double Value)
{
	Result.Metrics.Add({Name, Value});
}

void FCirrovaTestScenario::Complete(const FString& PassSummary)
{
	if (Result.Failures.IsEmpty()) { Pass(PassSummary); }
	else { Fail(TEXT("")); }
}

#pragma endregion

#pragma region Factory

void CirrovaTest::GetScenarioNames(TArray<FString>& OutNames)
{
	OutNames = {
		TEXT("T0"),
		TEXT("T1"),
		TEXT("T2"),
		TEXT("T3"),
		TEXT("T4"),
		TEXT("T5"),
		TEXT("T6"),
		TEXT("T7"),
		TEXT("T8"),
		TEXT("T9"),
		TEXT("T10"),
		TEXT("T11"),
		TEXT("T12"),
		TEXT("T13"),
		TEXT("T14")
	};
}

bool CirrovaTest::IsKnownScenario(const FString& Name)
{
	TArray<FString> Names;
	GetScenarioNames(Names);
	return Names.Contains(Name);
}

TUniquePtr<FCirrovaTestScenario> CirrovaTest::CreateScenario(const FString& Name)
{
	if (TUniquePtr<FCirrovaTestScenario> Scenario = CreateStaticScenario(Name)) { return Scenario; }
	if (TUniquePtr<FCirrovaTestScenario> Scenario = CreateSlopeScenario(Name)) { return Scenario; }
	if (TUniquePtr<FCirrovaTestScenario> Scenario = CreateDrivetrainScenario(Name)) { return Scenario; }
	return CreateSuspensionScenario(Name);
}

#pragma endregion
