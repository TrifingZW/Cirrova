#include "CirrovaTestWorldSubsystem.h"

#include "CirrovaLog.h"
#include "CirrovaTestScenarios.h"
#include "HAL/PlatformMisc.h"
#include "Misc/CommandLine.h"
#include "Misc/Parse.h"
#include "Misc/Paths.h"

bool UCirrovaTestWorldSubsystem::ShouldCreateSubsystem(UObject* Outer) const
{
	if (!Super::ShouldCreateSubsystem(Outer)) { return false; }

	FString Spec;
	return FParse::Value(FCommandLine::Get(), TEXT("CirrovaTest="), Spec) && !Spec.IsEmpty();
}

bool UCirrovaTestWorldSubsystem::DoesSupportWorldType(const EWorldType::Type WorldType) const
{
	return WorldType == EWorldType::Game || WorldType == EWorldType::PIE;
}

void UCirrovaTestWorldSubsystem::Initialize(FSubsystemCollectionBase& Collection)
{
	Super::Initialize(Collection);
	ParseCommandLine();
}

void UCirrovaTestWorldSubsystem::OnWorldBeginPlay(UWorld& InWorld)
{
	Super::OnWorldBeginPlay(InWorld);

	if (!bEnabled || ScenarioQueue.IsEmpty()) { return; }

	Harness = MakeUnique<FCirrovaTestHarness>(&InWorld, OutputDir);
	UE_LOG(
		LogCirrova,
		Display,
		TEXT("Cirrova 测试：开始自动化 P0，共 %d 个场景，输出 %s。"),
		ScenarioQueue.Num(),
		*OutputDir
	);
	StartNextScenario();
}

void UCirrovaTestWorldSubsystem::Deinitialize()
{
	ActiveScenario.Reset();
	Harness.Reset();
	Super::Deinitialize();
}

void UCirrovaTestWorldSubsystem::Tick(const float DeltaTime)
{
	if (!Harness || bFinishedAll) { return; }

	TotalElapsed += DeltaTime;
	Harness->Tick(DeltaTime);

	if (ActiveScenario)
	{
		if (ActiveScenario->GetElapsed() > ActiveScenario->GetTimeoutSeconds())
		{
			ActiveScenario->Abort(FString::Printf(
				TEXT("场景执行超过 %.1f s"),
				ActiveScenario->GetTimeoutSeconds()
			));
		}
		else
		{
			ActiveScenario->Tick(*Harness, DeltaTime);
		}

		if (ActiveScenario->IsFinished()) { FinishActiveScenario(); }
		return;
	}

	if (CooldownRemaining > 0.0f)
	{
		CooldownRemaining -= DeltaTime;
		if (CooldownRemaining <= 0.0f) { StartNextScenario(); }
	}

	if (TotalElapsed > MaxTotalSeconds && !bFinishedAll)
	{
		UE_LOG(LogCirrova, Error, TEXT("Cirrova 测试：总执行时间超过 %.0f s，终止。"), MaxTotalSeconds);
		if (ActiveScenario)
		{
			ActiveScenario->Abort(TEXT("总执行时间超出上限"));
			FinishActiveScenario();
		}
		FinishAll();
	}
}

TStatId UCirrovaTestWorldSubsystem::GetStatId() const
{
	RETURN_QUICK_DECLARE_CYCLE_STAT(UCirrovaTestWorldSubsystem, STATGROUP_Tickables);
}

bool UCirrovaTestWorldSubsystem::IsTickable() const
{
	return bEnabled && !bFinishedAll;
}

void UCirrovaTestWorldSubsystem::ParseCommandLine()
{
	// 场景列表本身以逗号分隔，解析时禁用分隔符截断。
	if (!FParse::Value(FCommandLine::Get(), TEXT("CirrovaTest="), ScenarioSpec, false))
	{
		bEnabled = false;
		return;
	}

	ScenarioSpec.TrimStartAndEndInline();
	if (ScenarioSpec.IsEmpty())
	{
		bEnabled = false;
		return;
	}

	FParse::Value(FCommandLine::Get(), TEXT("CirrovaTestOut="), OutputDir, false);
	if (OutputDir.IsEmpty())
	{
		OutputDir = FPaths::Combine(
			FPaths::ProjectSavedDir(),
			TEXT("CirrovaTests"),
			FDateTime::Now().ToString(TEXT("%Y%m%d-%H%M%S"))
		);
	}

	int32 ExitOnFinish = 1;
	FParse::Value(FCommandLine::Get(), TEXT("CirrovaTestExit="), ExitOnFinish);
	bQuitWhenDone = ExitOnFinish != 0;

	float MaxSeconds = 0.0f;
	if (FParse::Value(FCommandLine::Get(), TEXT("CirrovaTestMaxSeconds="), MaxSeconds) && MaxSeconds > 0.0f)
	{
		MaxTotalSeconds = MaxSeconds;
	}

	TArray<FString> Tokens;
	ScenarioSpec.ParseIntoArray(Tokens, TEXT(","), true);
	for (FString Token : Tokens)
	{
		Token.TrimStartAndEndInline();
		Token.ToUpperInline();
		if (Token == TEXT("ALL") || Token == TEXT("*"))
		{
			ScenarioQueue.Reset();
			CirrovaTest::GetScenarioNames(ScenarioQueue);
			break;
		}
		if (!CirrovaTest::IsKnownScenario(Token))
		{
			UE_LOG(LogCirrova, Warning, TEXT("Cirrova 测试：忽略未知场景 %s。"), *Token);
			continue;
		}
		ScenarioQueue.AddUnique(Token);
	}

	if (ScenarioQueue.IsEmpty())
	{
		UE_LOG(LogCirrova, Error, TEXT("Cirrova 测试：没有有效场景，测试停用。"));
		bEnabled = false;
		return;
	}

	bEnabled = true;
}

void UCirrovaTestWorldSubsystem::StartNextScenario()
{
	if (!Harness || NextScenarioIndex >= ScenarioQueue.Num())
	{
		FinishAll();
		return;
	}

	const FString Name = ScenarioQueue[NextScenarioIndex++];
	ActiveScenario = CirrovaTest::CreateScenario(Name);
	if (!ActiveScenario)
	{
		UE_LOG(LogCirrova, Error, TEXT("Cirrova 测试：场景 %s 创建失败。"), *Name);
		CooldownRemaining = 0.1f;
		return;
	}

	Harness->BeginCase(Name);
	UE_LOG(LogCirrova, Display, TEXT("Cirrova 测试：开始 %s。"), *Name);
	ActiveScenario->Start(*Harness);
}

void UCirrovaTestWorldSubsystem::FinishActiveScenario()
{
	if (!ActiveScenario || !Harness) { return; }

	const FCirrovaTestScenarioResult Result = ActiveScenario->GetResult();
	Results.Add(Result);
	Harness->EndCase();
	Harness->DestroySpawned();

	UE_LOG(
		LogCirrova,
		Display,
		TEXT("Cirrova 测试：%s %s（%.2f s）%s"),
		*Result.Name,
		Result.bPassed ? TEXT("PASS") : TEXT("FAIL"),
		Result.Duration,
		*Result.Summary
	);
	for (const FString& Failure : Result.Failures)
	{
		UE_LOG(LogCirrova, Warning, TEXT("Cirrova 测试：%s - %s"), *Result.Name, *Failure);
	}

	ActiveScenario.Reset();
	CooldownRemaining = 0.25f;

	if (NextScenarioIndex >= ScenarioQueue.Num()) { FinishAll(); }
}

void UCirrovaTestWorldSubsystem::FinishAll()
{
	if (bFinishedAll || !Harness) { return; }
	bFinishedAll = true;

	Harness->DestroySpawned();
	bool bAllPassed = Results.Num() == ScenarioQueue.Num();
	for (const FCirrovaTestScenarioResult& Result : Results) { bAllPassed &= Result.bPassed; }

	int32 Passed = 0;
	for (const FCirrovaTestScenarioResult& Result : Results) { Passed += Result.bPassed ? 1 : 0; }

	Harness->WriteReport(Results, bAllPassed);
	UE_LOG(
		LogCirrova,
		Display,
		TEXT("Cirrova 测试：全部完成，通过 %d/%d，总判定 %s。"),
		Passed,
		Results.Num(),
		bAllPassed ? TEXT("PASS") : TEXT("FAIL")
	);

	RequestExitIfConfigured(bAllPassed);
}

void UCirrovaTestWorldSubsystem::RequestExitIfConfigured(const bool bAllPassed) const
{
	if (!bQuitWhenDone) { return; }

	GLog->Flush();
	FPlatformMisc::RequestExitWithStatus(false, bAllPassed ? 0 : 1);
}
