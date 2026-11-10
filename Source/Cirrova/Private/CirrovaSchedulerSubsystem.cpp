#include "CirrovaSchedulerSubsystem.h"

#include "CirrovaLog.h"
#include "CirrovaVehicleAsync.h"
#include "CirrovaVehicleComponent.h"
#include "CirrovaWheelFrictionConstraints.h"
#include "Chaos/PBDRigidsEvolutionGBF.h"
#include "HAL/IConsoleManager.h"
#include "HAL/PlatformTime.h"
#include "PBDRigidsSolver.h"
#include "Engine/World.h"
#include "Physics/PhysicsInterfaceCore.h"
#include "Physics/Experimental/PhysScene_Chaos.h"

namespace
{
	// P0 性能基线：按需输出物理步耗时统计。
	static TAutoConsoleVariable<int32> CVarCirrovaLogPhysicsStats(
		TEXT("Cirrova.LogPhysicsStats"),
		0,
		TEXT("1 = 周期输出 Cirrova 物理步统计。"),
		ECVF_Default
	);
	static TAutoConsoleVariable<float> CVarCirrovaLogPhysicsStatsInterval(
		TEXT("Cirrova.LogPhysicsStatsInterval"),
		1.0f,
		TEXT("物理统计输出间隔（秒）。"),
		ECVF_Default
	);

	// Chaos 悬挂在低速时会把地面法线替换为世界上方（防滑机制）。该机制会掩盖轮胎摩擦，
	// Cirrova 默认关闭，悬挂沿真实悬挂轴作用；设为 1 可恢复引擎原行为用于对照。
	static TAutoConsoleVariable<int32> CVarCirrovaSuspensionSlopeAssist(
		TEXT("Cirrova.SuspensionSlopeAssist"),
		0,
		TEXT("1 = 保留 Chaos 低速斜坡防滑；0 = 使用真实悬挂轴（世界启动时读取）。"),
		ECVF_Default
	);

	/** 将 Cirrova 的斜坡防滑开关同步到 Chaos 悬挂 CVar；仅在世界启动时调用。 */
	void ApplySuspensionSlopeAssistSetting()
	{
		IConsoleVariable* SlopeThreshold = IConsoleManager::Get().FindConsoleVariable(
			TEXT("p.Chaos.Suspension.SlopeThreshold")
		);
		if (!SlopeThreshold) { return; }

		const bool bAssist = CVarCirrovaSuspensionSlopeAssist.GetValueOnGameThread() != 0;
		SlopeThreshold->Set(bAssist ? 0.707f : 1.0f, ECVF_SetByProjectSetting);
		UE_LOG(
			LogCirrova,
			Log,
			TEXT("Cirrova 调度器：Chaos 低速斜坡防滑=%s。"),
			bAssist ? TEXT("启用（对照）") : TEXT("关闭（真实悬挂轴）")
		);
	}

	/** 已脱离求解器、等待世界对象失效后释放的约束容器。 */
	struct FRetiredWheelConstraints
	{
		TWeakObjectPtr<UWorld> World;
		TUniquePtr<FCirrovaWheelFrictionConstraints> Constraints;
	};

	TArray<FRetiredWheelConstraints>& RetiredWheelConstraints()
	{
		static TArray<FRetiredWheelConstraints> Retired;
		return Retired;
	}

	/** 世界对象失效意味着其物理场景已销毁，此时释放容器不再与求解器生命周期交错。 */
	void PruneRetiredWheelConstraints()
	{
		TArray<FRetiredWheelConstraints>& Retired = RetiredWheelConstraints();
		for (int32 Index = Retired.Num() - 1; Index >= 0; --Index)
		{
			if (!Retired[Index].World.IsValid()) { Retired.RemoveAtSwap(Index); }
		}
	}
}

UCirrovaSchedulerSubsystem::~UCirrovaSchedulerSubsystem()
{
	// 正常路径下容器已在 Deinitialize 中移交延迟回收；此处仅处理未初始化即销毁的退路。
	WheelConstraints.Reset();
}

void UCirrovaSchedulerSubsystem::PostInitialize()
{
	Super::PostInitialize();

	UWorld* World = GetWorld();
	if (!World || !World->IsGameWorld()) { return; }

	// 新世界启动时回收已失效世界的容器。
	PruneRetiredWheelConstraints();
	ApplySuspensionSlopeAssistSetting();

	// 编辑器预览等无物理场景的世界同样会创建本子系统，此时跳过绑定。
	PhysicsScene = World->GetPhysicsScene();
	if (!PhysicsScene)
	{
		UE_LOG(
			LogCirrova,
			Warning,
			TEXT("Cirrova 调度器：世界 %s 没有物理场景，载具模拟停用。"),
			*GetNameSafe(World)
		);
		return;
	}

	BindEvent();
}

void UCirrovaSchedulerSubsystem::Deinitialize()
{
	// 先摘除物理回调，再释放运行态，避免物理线程访问已销毁的 liminal 句柄。
	UnbindEvent();

	for (FRegisteredVehicle& Entry : Vehicles)
	{
		if (!Entry.Runtime) { continue; }
		ReleaseSuspensionConstraints(*Entry.Runtime);
		PendingRelease.Add(MoveTemp(Entry.Runtime));
	}
	Vehicles.Reset();
	ReleasePendingRuntimes();

	// 物理已停止步进：摘除共享容器的图边并放弃粒子关联；
	// 容器移交延迟回收，等世界对象失效（物理场景必已销毁）后再释放。
	if (WheelConstraints)
	{
		WheelConstraints->DetachFromSolver();
		FRetiredWheelConstraints& Retired = RetiredWheelConstraints().AddDefaulted_GetRef();
		Retired.World = GetWorld();
		Retired.Constraints = MoveTemp(WheelConstraints);
	}

	Super::Deinitialize();
}

bool UCirrovaSchedulerSubsystem::RegisterVehicle(
	UCirrovaVehicleComponent* Component,
	TUniquePtr<FCirrovaVehicleRuntime> Runtime
)
{
	if (!Component || !Runtime || !Runtime->Handle || !AsyncCallback) { return false; }

	FRegisteredVehicle& Entry = Vehicles.AddDefaulted_GetRef();
	Entry.Component = Component;
	Entry.Runtime = MoveTemp(Runtime);
	Entry.Id = NextVehicleId++;

	// 约束创建依赖物理对象与场景写锁，必须在游戏线程、注册成功后执行。
	Component->InitializeSuspensionConstraints(*Entry.Runtime);

	// 轮胎约束只依赖共享容器，与原生悬挂约束解耦：
	// 悬挂约束不可用或未启用时，悬挂退回射线弹簧力，轮胎仍保留求解器约束耦合。
	bool bNativeSuspension = !Entry.Runtime->Wheels.IsEmpty();
	for (const FCirrovaWheelRuntime& Wheel : Entry.Runtime->Wheels)
	{
		bNativeSuspension &= Wheel.bUseSuspensionConstraint;
	}
	Entry.Runtime->bUseTireConstraint = !Entry.Runtime->Wheels.IsEmpty()
		&& WheelConstraints.IsValid();
	UE_LOG(
		LogCirrova,
		Log,
		TEXT("Cirrova 调度器：载具 %d 的轮胎耦合为%s，悬挂为%s。"),
		Entry.Id,
		Entry.Runtime->bUseTireConstraint ? TEXT("求解器约束") : TEXT("外力回退"),
		bNativeSuspension ? TEXT("原生约束") : TEXT("射线弹簧力")
	);
	return true;
}

void UCirrovaSchedulerSubsystem::UnregisterVehicle(const UCirrovaVehicleComponent* Component)
{
	for (int32 Index = Vehicles.Num() - 1; Index >= 0; --Index)
	{
		if (Vehicles[Index].Component.Get() != Component) { continue; }

		// 先释放原生悬挂约束（游戏线程 + 场景写锁），再移交运行态销毁。
		// 运行态内保存的约束指针与标志在此处清空，在飞物理步会自动回退为弹簧力施加。
		if (Vehicles[Index].Runtime) { ReleaseSuspensionConstraints(*Vehicles[Index].Runtime); }

		PendingRelease.Add(MoveTemp(Vehicles[Index].Runtime));
		Vehicles.RemoveAtSwap(Index);
	}
}

void UCirrovaSchedulerSubsystem::ReleaseSuspensionConstraints(FCirrovaVehicleRuntime& Runtime)
{
	for (FCirrovaWheelRuntime& Wheel : Runtime.Wheels)
	{
		if (Wheel.SuspensionConstraintHandle.IsValid() && PhysicsScene)
		{
			FPhysicsConstraintHandle Handle = Wheel.SuspensionConstraintHandle;
			FPhysicsCommand::ExecuteWrite(
				Handle,
				[&Handle](const FPhysicsConstraintHandle&) { FPhysicsInterface::ReleaseConstraint(Handle); }
			);
		}

		Wheel.SuspensionConstraintHandle = FPhysicsConstraintHandle();
		Wheel.SuspensionConstraint = nullptr;
		Wheel.bUseSuspensionConstraint = false;
		Wheel.GroundProxy = nullptr;
	}
}

void UCirrovaSchedulerSubsystem::ReleasePendingRuntimes()
{
	if (PendingRelease.IsEmpty()) { return; }

	// 物理线程可能仍在执行在飞帧：销毁命令入队到求解器，避免跨线程释放 liminal 句柄。
	Chaos::FPhysicsSolver* Solver = PhysicsScene ? PhysicsScene->GetSolver() : nullptr;
	if (Solver)
	{
		for (TUniquePtr<FCirrovaVehicleRuntime>& Runtime : PendingRelease)
		{
			if (!Runtime) { continue; }

			// 约束已在注销或子系统销毁时释放，此处仅销毁运行态。
			Solver->EnqueueCommandImmediate(
				[Runtime = MoveTemp(Runtime)]() mutable { Runtime.Reset(); }
			);
		}
	}
	PendingRelease.Reset();
}

void UCirrovaSchedulerSubsystem::BindEvent()
{
	if (!PhysicsScene) { return; }

	Chaos::FPhysicsSolver* Solver = PhysicsScene->GetSolver();
	if (!Solver) { return; }

	OnPhysScenePreTickHandle = PhysicsScene->OnPhysScenePreTick.AddUObject(
		this,
		&UCirrovaSchedulerSubsystem::OnPhysScenePreTick
	);
	OnPhysScenePostTickHandle = PhysicsScene->OnPhysScenePostTick.AddUObject(
		this,
		&UCirrovaSchedulerSubsystem::OnPhysScenePostTick
	);

	// 载具模拟运行在物理子步内：PreSimulate 回调同时处理接触解与力施加。
	AsyncCallback = Solver->CreateAndRegisterSimCallbackObject_External<FCirrovaAsyncCallback>();

	// 轮胎摩擦约束容器注册到求解器演化；容器不可移除，随场景存活。
	WheelConstraints = MakeUnique<FCirrovaWheelFrictionConstraints>();
	if (auto* Evolution = Solver->GetEvolution())
	{
		WheelConstraints->SetIslandManager(&Evolution->GetIslandManager());
		Evolution->AddConstraintContainer(*WheelConstraints, 0);
		UE_LOG(LogCirrova, Log, TEXT("Cirrova 调度器：轮胎摩擦约束容器已注册。"));
	}
	else
	{
		WheelConstraints.Reset();
		UE_LOG(
			LogCirrova,
			Warning,
			TEXT("Cirrova 调度器：求解器未暴露演化对象，载具轮胎回退为外力耦合。")
		);
	}
}

void UCirrovaSchedulerSubsystem::UnbindEvent()
{
	if (PhysicsScene)
	{
		PhysicsScene->OnPhysScenePreTick.Remove(OnPhysScenePreTickHandle);
		PhysicsScene->OnPhysScenePostTick.Remove(OnPhysScenePostTickHandle);
		OnPhysScenePreTickHandle.Reset();
		OnPhysScenePostTickHandle.Reset();

		if (AsyncCallback)
		{
			if (auto* Solver = PhysicsScene->GetSolver())
			{
				Solver->UnregisterAndFreeSimCallbackObject_External(AsyncCallback);
			}
		}
	}
	AsyncCallback = nullptr;
}

UCirrovaVehicleComponent* UCirrovaSchedulerSubsystem::FindComponent(int32 Id) const
{
	for (const FRegisteredVehicle& Entry : Vehicles) { if (Entry.Id == Id) { return Entry.Component.Get(); } }
	return nullptr;
}

void UCirrovaSchedulerSubsystem::OnPhysScenePreTick(
	FPhysScene_Chaos* PhysScene,
	const float DeltaTime
)
{
	if (!AsyncCallback) { return; }

	FCirrovaAsyncInput* Input = AsyncCallback->GetProducerInputData_External();
	if (!Input) { return; }

	Input->Reset();
	Input->WheelConstraints = WheelConstraints.Get();
	Input->Vehicles.Reserve(Vehicles.Num());

	for (const FRegisteredVehicle& Entry : Vehicles)
	{
		UCirrovaVehicleComponent* Component = Entry.Component.Get();
		if (!Component || !Entry.Runtime) { continue; }

		FCirrovaVehicleAsyncInput& VehicleInput = Input->Vehicles.AddDefaulted_GetRef();
		VehicleInput.VehicleId = Entry.Id;
		VehicleInput.Runtime = Entry.Runtime.Get();
		Component->ProduceAsyncInput(VehicleInput);
	}
}

void UCirrovaSchedulerSubsystem::OnPhysScenePostTick(FChaosScene* ChaosScene)
{
	float LatestPreSimulateMs = 0.0f;
	float LatestStepWallMs = 0.0f;
	float LatestPostSolveMs = 0.0f;
	int32 LatestVehicleCount = 0;
	bool bHasStats = false;

	if (AsyncCallback)
	{
		while (auto Output = AsyncCallback->PopOutputData_External())
		{
			LatestPreSimulateMs = Output->PreSimulateMs;
			LatestStepWallMs = Output->StepWallMs;
			LatestPostSolveMs = Output->PostSolveMs;
			LatestVehicleCount = Output->VehicleCount;
			bHasStats = true;

			for (const FCirrovaVehicleOutput& VehicleOutput : Output->Vehicles)
			{
				if (UCirrovaVehicleComponent* Component = FindComponent(VehicleOutput.VehicleId))
				{
					Component->ConsumeAsyncOutput(VehicleOutput);
				}
			}
		}
	}

	// P0 性能基线：仅在 CVar 开启时按间隔输出。
	if (bHasStats)
	{
		LastStepWallMs = LatestStepWallMs;
		PeakStepWallMs = FMath::Max(PeakStepWallMs, LatestStepWallMs);
		StepWallMsSum += LatestStepWallMs;
		++StepWallMsSamples;
	}
	if (bHasStats && CVarCirrovaLogPhysicsStats.GetValueOnGameThread() != 0)
	{
		const double Now = FPlatformTime::Seconds();
		const float Interval = FMath::Max(
			CVarCirrovaLogPhysicsStatsInterval.GetValueOnGameThread(),
			0.1f
		);
		if (PhysicsStatsLastLogSeconds <= 0.0) { PhysicsStatsLastLogSeconds = Now; }
		if (Now - PhysicsStatsLastLogSeconds >= Interval)
		{
			PhysicsStatsLastLogSeconds = Now;
			UE_LOG(
				LogCirrova,
				Log,
				TEXT("Cirrova 物理统计：载具 %d，步墙钟 %.2f ms（准备 %.2f ms，收尾 %.2f ms），轮胎约束槽 %d。"),
				LatestVehicleCount,
				LatestStepWallMs,
				LatestPreSimulateMs,
				LatestPostSolveMs,
				WheelConstraints.IsValid() ? WheelConstraints->NumActiveConstraints() : 0
			);
		}
	}

	// 物理步进已结束，释放已注销载具的运行态。
	ReleasePendingRuntimes();
}

int32 UCirrovaSchedulerSubsystem::GetActiveWheelConstraintCount() const
{
	return WheelConstraints.IsValid() ? WheelConstraints->NumActiveConstraints() : 0;
}

float UCirrovaSchedulerSubsystem::GetAveragePhysicsStepMs() const
{
	return StepWallMsSamples > 0
		? static_cast<float>(StepWallMsSum / static_cast<double>(StepWallMsSamples))
		: 0.0f;
}
