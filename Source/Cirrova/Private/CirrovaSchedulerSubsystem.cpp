#include "CirrovaSchedulerSubsystem.h"

#include "CirrovaVehicleAsync.h"
#include "CirrovaVehicleComponent.h"
#include "Engine/World.h"
#include "PBDRigidsSolver.h"
#include "Physics/PhysicsInterfaceCore.h"
#include "Physics/Experimental/PhysScene_Chaos.h"

void UCirrovaSchedulerSubsystem::PostInitialize()
{
	Super::PostInitialize();

	UWorld* World = GetWorld();
	if (!World || !World->IsGameWorld()) { return; }

	// 编辑器预览等无物理场景的世界同样会创建本子系统，此时跳过绑定。
	PhysicsScene = World->GetPhysicsScene();
	if (!PhysicsScene)
	{
		UE_LOG(
			LogTemp,
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
		if (Entry.Runtime) { ReleaseSuspensionConstraints(*Entry.Runtime); }
	}

	ReleasePendingRuntimes();
	Vehicles.Reset();

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
	return true;
}

void UCirrovaSchedulerSubsystem::UnregisterVehicle(const UCirrovaVehicleComponent* Component)
{
	for (int32 Index = Vehicles.Num() - 1; Index >= 0; --Index)
	{
		if (Vehicles[Index].Component.Get() != Component) { continue; }

		// 先释放原生悬挂约束（游戏线程 + 场景写锁），再移交运行态销毁。
		// 运行态内保存的约束指针与标志在此处清空，在飞物理步会自动回退为力模式。
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
				[&Handle](const FPhysicsConstraintHandle&)
				{
					FPhysicsInterface::ReleaseConstraint(Handle);
				}
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

			// 约束已在注销时释放（PendingRelease 的唯一来源是 UnregisterVehicle）。
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
	for (const FRegisteredVehicle& Entry : Vehicles)
	{
		if (Entry.Id == Id) { return Entry.Component.Get(); }
	}
	return nullptr;
}

void UCirrovaSchedulerSubsystem::OnPhysScenePreTick(
	FPhysScene_Chaos* PhysScene,
	const float DeltaTime
)
{
	if (!AsyncCallback || !GetWorld()) { return; }

	FCirrovaAsyncInput* Input = AsyncCallback->GetProducerInputData_External();
	if (!Input) { return; }

	Input->Reset();
	Input->World = GetWorld();
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
	if (AsyncCallback)
	{
		while (auto Output = AsyncCallback->PopOutputData_External())
		{
			for (const FCirrovaVehicleOutput& VehicleOutput : Output->Vehicles)
			{
				if (UCirrovaVehicleComponent* Component = FindComponent(VehicleOutput.VehicleId))
				{
					Component->ConsumeAsyncOutput(VehicleOutput);
				}
			}
		}
	}

	// 物理步进已结束，释放已注销载具的运行态。
	ReleasePendingRuntimes();
}
