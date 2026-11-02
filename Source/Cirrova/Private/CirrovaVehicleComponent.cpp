#include "CirrovaVehicleComponent.h"

#include "CirrovaSchedulerSubsystem.h"
#include "CirrovaVehicleRuntime.h"
#include "DrawDebugHelpers.h"
#include "PBDRigidsSolver.h"
#include "Chaos/ChaosScene.h"
#include "Chaos/PBDSuspensionConstraintData.h"
#include "Components/PrimitiveComponent.h"
#include "Engine/World.h"
#include "Physics/PhysicsInterfaceCore.h"
#include "PhysicsEngine/BodyInstance.h"
#include "PhysicsEngine/PhysicsObjectExternalInterface.h"
// FPhysicsActorHandle 的具体类型，供到 IPhysicsProxyBase* 的派生到基类转换。
#include "PhysicsProxy/SingleParticlePhysicsProxy.h"

namespace
{
	// liminal 基（X 右，Y 上，Z 前）→ UE 基（X 前，Y 右，Z 上）的循环置换。
	const FQuat LiminalToUE(0.5f, 0.5f, 0.5f, 0.5f);

	// 方向向量基变换（不缩放）。
	FVector LiminalDirectionToUE(const FLiminalVec3& Vector)
	{
		return LiminalToUE.RotateVector(FVector(Vector.X, Vector.Y, Vector.Z));
	}

	FVector UEDirectionToLiminal(const FVector& Vector) { return LiminalToUE.Inverse().RotateVector(Vector); }

	// liminal 位姿（m）→ UE 位姿（cm）。
	FTransform LiminalPoseToUE(const FLiminalFrame& Pose)
	{
		FTransform Result = FTransform::Identity;
		Result.SetTranslation(LiminalDirectionToUE(Pose.Translation) * 100.0f);
		Result.SetRotation(
			LiminalToUE * FQuat(Pose.Rotation[0], Pose.Rotation[1], Pose.Rotation[2], Pose.Rotation[3])
		);
		return Result;
	}
}

UCirrovaVehicleComponent::UCirrovaVehicleComponent()
{
	PrimaryComponentTick.bCanEverTick = true;
	PrimaryComponentTick.TickGroup = TG_PrePhysics;

	// 默认四轮布局：前两轮可转向，X 前 Y 右，单位 cm。
	WheelConfigs.SetNum(4);
	WheelConfigs[0].MountLocation = FVector(140.0f, -80.0f, 0.0f);
	WheelConfigs[0].bSteerable = true;
	WheelConfigs[1].MountLocation = FVector(140.0f, 80.0f, 0.0f);
	WheelConfigs[1].bSteerable = true;
	WheelConfigs[2].MountLocation = FVector(-140.0f, -80.0f, 0.0f);
	WheelConfigs[3].MountLocation = FVector(-140.0f, 80.0f, 0.0f);
}

void UCirrovaVehicleComponent::BeginPlay()
{
	Super::BeginPlay();

	ChassisPrimitive = Cast<UPrimitiveComponent>(GetOwner()->GetRootComponent());
	if (!ChassisPrimitive.IsValid() || !ChassisPrimitive->IsSimulatingPhysics())
	{
		UE_LOG(
			LogTemp,
			Warning,
			TEXT("Cirrova 载具组件需要拥有者根组件为模拟物理的 UPrimitiveComponent，组件已停用。")
		);
		ChassisPrimitive = nullptr;
		SetComponentTickEnabled(false);
		return;
	}

	// 默认参数适用于约 1 吨级底盘；质量失配会放大悬挂力与落地冲量。
	const float ChassisMass = ChassisPrimitive->GetMass();
	UE_LOG(
		LogTemp,
		Log,
		TEXT("Cirrova 载具组件初始化：底盘质量 %.1f kg，车轮 %d 个。"),
		ChassisMass,
		WheelConfigs.Num()
	);
	if (ChassisMass < 100.0f)
	{
		UE_LOG(
			LogTemp,
			Warning,
			TEXT("Cirrova 载具组件：底盘质量 %.1f kg 远低于默认参数适用区间（约 1000 kg），请按质量缩放悬挂刚度与阻尼。"),
			ChassisMass
		);
	}

	UCirrovaSchedulerSubsystem* Scheduler = GetWorld()
		                                        ? GetWorld()->GetSubsystem<UCirrovaSchedulerSubsystem>()
		                                        : nullptr;
	if (!Scheduler)
	{
		UE_LOG(LogTemp, Error, TEXT("Cirrova 载具组件：未找到调度器子系统，组件已停用。"));
		SetComponentTickEnabled(false);
		return;
	}

	TUniquePtr<FCirrovaVehicleRuntime> Runtime = MakeUnique<FCirrovaVehicleRuntime>();
	if (!BuildTopology(*Runtime))
	{
		SetComponentTickEnabled(false);
		return;
	}

	WheelStates.SetNum(Runtime->Wheels.Num());
	WheelMountLocations.SetNum(Runtime->Wheels.Num());
	for (int32 Index = 0; Index < Runtime->Wheels.Num(); ++Index)
	{
		const FCirrovaWheelRuntime& Wheel = Runtime->Wheels[Index];
		WheelStates[Index].MaxLength = Wheel.MaxLength;
		WheelStates[Index].Radius = Wheel.Radius;
		WheelMountLocations[Index] = Wheel.MountLocation;
	}

	if (!Scheduler->RegisterVehicle(this, MoveTemp(Runtime)))
	{
		UE_LOG(LogTemp, Error, TEXT("Cirrova 载具组件：注册到调度器失败，组件已停用。"));
		SetComponentTickEnabled(false);
	}
}

void UCirrovaVehicleComponent::EndPlay(const EEndPlayReason::Type EndPlayReason)
{
	if (UCirrovaSchedulerSubsystem* Scheduler = GetWorld()
		                                            ? GetWorld()->GetSubsystem<UCirrovaSchedulerSubsystem>()
		                                            : nullptr) { Scheduler->UnregisterVehicle(this); }

	Super::EndPlay(EndPlayReason);
}

void UCirrovaVehicleComponent::TickComponent(
	float DeltaTime,
	ELevelTick TickType,
	FActorComponentTickFunction* ThisTickFunction
)
{
	Super::TickComponent(DeltaTime, TickType, ThisTickFunction);

	if (!bDrawDebug || !ChassisPrimitive.IsValid()) { return; }

	const FTransform ChassisToWorld = ChassisPrimitive->GetComponentTransform();
	const FVector ChassisUp = ChassisToWorld.GetUnitAxis(EAxis::Z);
	for (int32 Index = 0; Index < WheelStates.Num(); ++Index)
	{
		const FVector MountWorld = WheelMountLocations.IsValidIndex(Index)
			                           ? ChassisToWorld.TransformPosition(WheelMountLocations[Index])
			                           : ChassisToWorld.GetLocation();
		DrawWheelDebug(WheelStates[Index], MountWorld, ChassisUp);
	}
}

void UCirrovaVehicleComponent::SetThrottle(float Value) { ThrottleInput = Value; }

void UCirrovaVehicleComponent::SetSteering(float Value) { SteeringInput = Value; }

void UCirrovaVehicleComponent::SetBrake(float Value) { BrakeInput = Value; }

void UCirrovaVehicleComponent::SetGear(int32 Gear) { GearInput = Gear; }

float UCirrovaVehicleComponent::GetEngineRPM() const { return EngineOmega * 60.0f / static_cast<float>(2.0 * PI); }

float UCirrovaVehicleComponent::GetWheelAngularVelocity(int32 WheelIndex) const
{
	return WheelStates.IsValidIndex(WheelIndex) ? WheelStates[WheelIndex].Omega : 0.0f;
}

bool UCirrovaVehicleComponent::IsGrounded() const { return bWasGrounded; }

void UCirrovaVehicleComponent::ProduceAsyncInput(FCirrovaVehicleAsyncInput& Out) const
{
	Out.Throttle = ThrottleInput;
	Out.Steering = SteeringInput;
	Out.Brake = BrakeInput;
	Out.Gear = GearInput;

	const UPrimitiveComponent* Chassis = ChassisPrimitive.Get();
	if (!Chassis) { return; }

	if (const FBodyInstance* BodyInstance = Chassis->GetBodyInstance()) { Out.Proxy = BodyInstance->GetPhysicsActor(); }
	Out.TraceParams = FCollisionQueryParams(
		SCENE_QUERY_STAT(CirrovaWheelTrace),
		false,
		GetOwner()
	);
	// 只需宽相候选集：窄相射线在物理线程对候选对象执行。
	Out.TraceParams.bIgnoreTouches = true;
	Out.TraceParams.bSkipNarrowPhase = true;
	Out.TraceParams.bTraceComplex = true;
}

void UCirrovaVehicleComponent::InitializeSuspensionConstraints(FCirrovaVehicleRuntime& Runtime)
{
	if (!bUseNativeSuspension || !ChassisPrimitive.IsValid()) { return; }

	const FBodyInstance* BodyInstance = ChassisPrimitive->GetBodyInstance();
	IPhysicsProxyBase* Proxy = BodyInstance ? BodyInstance->GetPhysicsActor() : nullptr;
	if (!Proxy || Proxy->GetType() != EPhysicsProxyType::SingleParticleProxy)
	{
		UE_LOG(
			LogTemp,
			Warning,
			TEXT("Cirrova 载具组件：底盘不是单粒子物理代理，悬挂回退为弹簧力施加。")
		);
		return;
	}

	auto* ParticleProxy = static_cast<Chaos::FSingleParticlePhysicsProxy*>(Proxy);
	Chaos::FPhysicsObjectHandle PhysicsObject = ParticleProxy->GetPhysicsObject();
	Chaos::FPhysicsSolver* Solver = Proxy->GetSolver<Chaos::FPhysicsSolver>();
	if (!PhysicsObject || !Solver)
	{
		UE_LOG(
			LogTemp,
			Warning,
			TEXT("Cirrova 载具组件：物理对象或求解器不可用，悬挂回退为弹簧力施加。")
		);
		return;
	}

	// 约束阻尼项以步长为量纲：D = SpringDamping·dt，按求解器步长换算。
	float SolverDeltaTime = static_cast<float>(Solver->GetAsyncDeltaTime());
	if (SolverDeltaTime <= KINDA_SMALL_NUMBER) { SolverDeltaTime = 1.0f / 60.0f; }
	SolverDeltaTime = FMath::Clamp(SolverDeltaTime, 1.0f / 1000.0f, 0.1f);

	FChaosScene* Scene = FPhysicsObjectExternalInterface::GetScene(PhysicsObject);
	if (!Scene)
	{
		UE_LOG(
			LogTemp,
			Warning,
			TEXT("Cirrova 载具组件：无法获取物理场景，悬挂回退为弹簧力施加。")
		);
		return;
	}

	FLockedWritePhysicsObjectExternalInterface Interface = FPhysicsObjectExternalInterface::LockWrite(Scene);
	if (!Interface->GetParticle(PhysicsObject))
	{
		UE_LOG(
			LogTemp,
			Warning,
			TEXT("Cirrova 载具组件：物理对象未就绪，悬挂回退为弹簧力施加。")
		);
		return;
	}

	int32 NativeCount = 0;
	for (int32 Index = 0; Index < Runtime.Wheels.Num(); ++Index)
	{
		FCirrovaWheelRuntime& Wheel = Runtime.Wheels[Index];
		if (Wheel.RestLength <= Wheel.MinLength)
		{
			UE_LOG(
				LogTemp,
				Warning,
				TEXT("Cirrova 载具组件：车轮 %d 的自由长度不大于行程下限，悬挂回退为弹簧力施加。"),
				Index
			);
			continue;
		}

		const FPhysicsConstraintHandle Handle = FPhysicsInterface::CreateSuspension(
			PhysicsObject,
			Wheel.MountLocation
		);
		if (!Handle.IsValid())
		{
			UE_LOG(
				LogTemp,
				Warning,
				TEXT("Cirrova 载具组件：车轮 %d 悬挂约束创建失败，回退为弹簧力施加。"),
				Index
			);
			continue;
		}

		// 句柄无论约束对象是否可用都留存，由注销路径统一释放。
		Wheel.SuspensionConstraintHandle = Handle;
		auto* Constraint = static_cast<Chaos::FSuspensionConstraint*>(Handle.Constraint);
		if (!Constraint)
		{
			UE_LOG(
				LogTemp,
				Warning,
				TEXT("Cirrova 载具组件：车轮 %d 悬挂约束类型无效，回退为弹簧力施加。"),
				Index
			);
			continue;
		}

		Constraint->SetHardstopStiffness(1.0f);
		Constraint->SetHardstopVelocityCompensation(1.0f);
		Constraint->SetSpringPreload(0.0f);
		Constraint->SetSpringStiffness(FMath::Max(Wheel.SuspensionStiffness, 0.0f));
		Constraint->SetSpringDamping(
			FMath::Max(Wheel.SuspensionDamping, 0.0f) * SolverDeltaTime
		);
		Constraint->SetMinLength(Wheel.MinLength * 100.0f);
		Constraint->SetMaxLength(Wheel.RestLength * 100.0f);
		Constraint->SetAxis(Chaos::FVec3(0.0f, 0.0f, 1.0f));
		// 保持 Enabled=false：目标点与法向在首个物理步由 SetSuspensionTarget 写入。

		Wheel.SuspensionConstraint = Constraint;
		Wheel.bUseSuspensionConstraint = true;
		++NativeCount;
	}

	UE_LOG(
		LogTemp,
		Log,
		TEXT("Cirrova 载具组件：原生悬挂约束启用 %d/%d 轮。"),
		NativeCount,
		Runtime.Wheels.Num()
	);
}

void UCirrovaVehicleComponent::ConsumeAsyncOutput(const FCirrovaVehicleOutput& In)
{
	EngineOmega = In.EngineOmega;
	if (WheelStates.Num() != In.Wheels.Num()) { WheelStates.SetNum(In.Wheels.Num()); }

	for (int32 Index = 0; Index < In.Wheels.Num(); ++Index)
	{
		const FCirrovaWheelOutput& Source = In.Wheels[Index];
		FCirrovaWheelState& Target = WheelStates[Index];
		Target.bInContact = Source.bInContact;
		Target.Omega = Source.Omega;
		Target.SuspensionLength = Source.SuspensionLength;
		Target.SuspensionForce = Source.SuspensionForce;
		Target.AntiRollForce = Source.AntiRollForce;
		Target.NormalLoad = Source.NormalLoad;
		Target.TireForces = Source.TireForces;
		Target.ContactPoint = Source.ContactPoint;
		Target.ContactNormal = Source.ContactNormal;
		Target.LongitudinalAxis = Source.LongitudinalAxis;
		Target.LateralAxis = Source.LateralAxis;
	}

	if (In.bGrounded != bWasGrounded)
	{
		bWasGrounded = In.bGrounded;
		OnVehicleGroundedEvent.Broadcast(bWasGrounded);
	}
}

void UCirrovaVehicleComponent::BuildAntiRollBars(FCirrovaVehicleRuntime& Runtime) const
{
	Runtime.AntiRollBars.Reset();
	TArray<uint64> RegisteredPairs;

	for (int32 Index = 0; Index < AntiRollBars.Num(); ++Index)
	{
		const FCirrovaAntiRollBarConfig& Config = AntiRollBars[Index];
		if (!WheelConfigs.IsValidIndex(Config.WheelA)
			|| !WheelConfigs.IsValidIndex(Config.WheelB)
			|| Config.WheelA == Config.WheelB)
		{
			UE_LOG(
				LogTemp,
				Warning,
				TEXT("Cirrova 载具组件：防倾杆 %d 的车轮索引无效，已跳过。"),
				Index
			);
			continue;
		}
		if (!FMath::IsFinite(Config.Stiffness) || Config.Stiffness < 0.0f)
		{
			UE_LOG(
				LogTemp,
				Warning,
				TEXT("Cirrova 载具组件：防倾杆 %d 的刚度无效，已跳过。"),
				Index
			);
			continue;
		}
		if (Config.Stiffness <= 0.0f) { continue; }

		const int32 Low = FMath::Min(Config.WheelA, Config.WheelB);
		const int32 High = FMath::Max(Config.WheelA, Config.WheelB);
		const uint64 PairKey = (static_cast<uint64>(High) << 32) | static_cast<uint32>(Low);
		if (RegisteredPairs.Contains(PairKey))
		{
			UE_LOG(
				LogTemp,
				Warning,
				TEXT("Cirrova 载具组件：防倾杆 %d 重复连接车轮 %d 与 %d，已跳过。"),
				Index,
				Low,
				High
			);
			continue;
		}
		RegisteredPairs.Add(PairKey);

		FCirrovaAntiRollBarRuntime& Bar = Runtime.AntiRollBars.AddDefaulted_GetRef();
		Bar.WheelA = Config.WheelA;
		Bar.WheelB = Config.WheelB;
		Bar.Stiffness = Config.Stiffness;
	}
}

bool UCirrovaVehicleComponent::BuildTopology(FCirrovaVehicleRuntime& Runtime)
{
	Runtime.Handle = liminal_vehicle_new();
	if (!Runtime.Handle)
	{
		UE_LOG(LogTemp, Error, TEXT("Cirrova 载具组件：liminal 句柄创建失败。"));
		return false;
	}

	const int32 WheelCount = WheelConfigs.Num();
	if (WheelCount == 0 || GearRatios.Num() == 0)
	{
		UE_LOG(LogTemp, Warning, TEXT("Cirrova 载具组件缺少车轮配置或传动比，组件已停用。"));
		return false;
	}

	Runtime.ChassisMass = ChassisPrimitive.IsValid() ? ChassisPrimitive->GetMass() : 0.0f;
	Runtime.MaxSteeringAngle = FMath::DegreesToRadians(MaxSteeringAngle);
	Runtime.TireFriction = TireFriction;
	Runtime.TireLateralStiffness = TireLateralStiffness;
	Runtime.TireLoadSensitivity = TireLoadSensitivity;
	Runtime.TireReferenceLoad = TireReferenceLoad;
	Runtime.TireRollingResistance = TireRollingResistance;
	Runtime.WheelInertia = WheelInertia;
	Runtime.WheelTraceChannel = WheelTraceChannel.GetValue();
	BuildAntiRollBars(Runtime);

	uint64 Cockpit = 0;
	uint64 Chassis = 0;
	uint64 Engine = 0;
	uint64 Gearbox = 0;
	uint64 Distributor = 0;
	uint64 SteeringSplitter = 0;
	uint64 BrakeSplitter = 0;

	liminal_vehicle_add_cockpit(Runtime.Handle, &Cockpit);

	// 挂点：UE cm → liminal m 并做基变换。
	TArray<FLiminalFrame> Mounts;
	Mounts.Reserve(WheelCount);
	for (const FCirrovaWheelConfig& Config : WheelConfigs)
	{
		const FVector Liminal = UEDirectionToLiminal(Config.MountLocation) * 0.01f;
		FLiminalFrame& Mount = Mounts.AddDefaulted_GetRef();
		Mount.Translation = {
			static_cast<float>(Liminal.X),
			static_cast<float>(Liminal.Y),
			static_cast<float>(Liminal.Z)
		};
		Mount.Rotation[0] = 0.0f;
		Mount.Rotation[1] = 0.0f;
		Mount.Rotation[2] = 0.0f;
		Mount.Rotation[3] = 1.0f;
	}
	liminal_vehicle_add_chassis(Runtime.Handle, Mounts.GetData(), Mounts.Num(), &Chassis);

	// 扭矩曲线：RPM → 归一化转速；为空时 liminal 退化为线性衰减。
	constexpr float RpmToRadiansPerSecond = 2.0 * PI / 60.0;
	TArray<FLiminalEngineCurvePoint> EngineCurvePoints;
	EngineCurvePoints.Reserve(EngineTorqueCurve.Num());
	for (const FCirrovaEngineCurvePoint& Point : EngineTorqueCurve)
	{
		FLiminalEngineCurvePoint& Converted = EngineCurvePoints.AddDefaulted_GetRef();
		Converted.SpeedRatio = EngineMaxAngularVelocity > KINDA_SMALL_NUMBER
			                       ? Point.RPM * RpmToRadiansPerSecond / EngineMaxAngularVelocity
			                       : 0.0f;
		Converted.TorqueScale = Point.TorqueScale;
	}

	FLiminalEngineParams EngineParams{};
	EngineParams.MaxTorque = EngineMaxTorque;
	EngineParams.MaxAngularVelocity = EngineMaxAngularVelocity;
	EngineParams.Inertia = EngineInertia;
	EngineParams.IdleAngularVelocity = EngineIdleRPM * RpmToRadiansPerSecond;
	EngineParams.IdleTorqueLimit = EngineIdleTorque;
	EngineParams.EngineBrakingTorque = EngineBrakingTorque;
	EngineParams.Curve = EngineCurvePoints.Num() > 0 ? EngineCurvePoints.GetData() : nullptr;
	EngineParams.CurveCount = static_cast<size_t>(EngineCurvePoints.Num());
	liminal_vehicle_add_engine_with_params(Runtime.Handle, &EngineParams, &Engine);
	liminal_vehicle_add_gearbox(
		Runtime.Handle,
		GearRatios.GetData(),
		static_cast<size_t>(GearRatios.Num()),
		GearboxInertia,
		&Gearbox
	);
	if (bUseDifferential)
	{
		liminal_vehicle_add_differential(
			Runtime.Handle,
			static_cast<size_t>(WheelCount),
			DifferentialMaxTorque,
			AxleInertia,
			&Distributor
		);
	}
	else
	{
		liminal_vehicle_add_axle(
			Runtime.Handle,
			static_cast<size_t>(WheelCount),
			AxleInertia,
			&Distributor
		);
	}
	liminal_vehicle_add_splitter(Runtime.Handle, static_cast<size_t>(WheelCount), &SteeringSplitter);
	liminal_vehicle_add_splitter(Runtime.Handle, static_cast<size_t>(WheelCount), &BrakeSplitter);

	// 驾驶舱信号：油门/挡位直达，转向与制动经分发器到各轮。
	Runtime.ThrottleSignalPort = PortOf(Runtime.Handle, Cockpit, 0);
	Runtime.SteeringSignalPort = PortOf(Runtime.Handle, Cockpit, 1);
	Runtime.GearSignalPort = PortOf(Runtime.Handle, Cockpit, 2);
	Runtime.BrakeSignalPort = PortOf(Runtime.Handle, Cockpit, 3);
	Runtime.EngineShaftPort = PortOf(Runtime.Handle, Engine, 1);
	Connect(Runtime.Handle, Runtime.ThrottleSignalPort, PortOf(Runtime.Handle, Engine, 0));
	Connect(Runtime.Handle, Runtime.GearSignalPort, PortOf(Runtime.Handle, Gearbox, 0));
	Connect(Runtime.Handle, Runtime.SteeringSignalPort, PortOf(Runtime.Handle, SteeringSplitter, 0));
	Connect(Runtime.Handle, Runtime.BrakeSignalPort, PortOf(Runtime.Handle, BrakeSplitter, 0));

	// 动力链：发动机 → 变速箱 → 轮轴/差速器。
	Connect(Runtime.Handle, PortOf(Runtime.Handle, Engine, 1), PortOf(Runtime.Handle, Gearbox, 1));
	Connect(Runtime.Handle, PortOf(Runtime.Handle, Gearbox, 2), PortOf(Runtime.Handle, Distributor, 0));

	// 逐轮：空间链（底盘 → 悬挂 → 转向节 → 车轮）、动力与制动、信号分发。
	TArray<uint64> SuspensionNodes;
	TArray<uint64> KnuckleNodes;
	TArray<uint64> WheelNodes;
	SuspensionNodes.Reserve(WheelCount);
	KnuckleNodes.Reserve(WheelCount);
	WheelNodes.Reserve(WheelCount);

	for (int32 Index = 0; Index < WheelCount; ++Index)
	{
		const FCirrovaWheelConfig& Config = WheelConfigs[Index];

		uint64 Suspension = 0;
		uint64 Knuckle = 0;
		uint64 Wheel = 0;
		liminal_vehicle_add_suspension(
			Runtime.Handle,
			Config.SuspensionStiffness,
			Config.SuspensionDamping,
			Config.SuspensionRestLength,
			Config.SuspensionMinLength,
			Config.SuspensionMaxLength,
			&Suspension
		);
		liminal_vehicle_add_knuckle(Runtime.Handle, MaxBrakeTorque, &Knuckle);
		liminal_vehicle_add_wheel(
			Runtime.Handle,
			WheelInertia,
			WheelFrictionTorque,
			Config.Radius * 0.01f,
			&Wheel
		);

		Connect(Runtime.Handle, PortOf(Runtime.Handle, Chassis, Index), PortOf(Runtime.Handle, Suspension, 0));
		Connect(Runtime.Handle, PortOf(Runtime.Handle, Suspension, 1), PortOf(Runtime.Handle, Knuckle, 0));
		Connect(Runtime.Handle, PortOf(Runtime.Handle, Knuckle, 1), PortOf(Runtime.Handle, Wheel, 1));

		Connect(
			Runtime.Handle,
			PortOf(Runtime.Handle, Distributor, static_cast<uint32>(Index) + 1),
			PortOf(Runtime.Handle, Wheel, 0)
		);
		Connect(Runtime.Handle, PortOf(Runtime.Handle, Knuckle, 4), PortOf(Runtime.Handle, Wheel, 0));

		// 不可转向的车轮不接转向信号，端口保持零值。
		if (Config.bSteerable)
		{
			Connect(
				Runtime.Handle,
				PortOf(Runtime.Handle, SteeringSplitter, static_cast<uint32>(Index) + 1),
				PortOf(Runtime.Handle, Knuckle, 2)
			);
		}
		Connect(
			Runtime.Handle,
			PortOf(Runtime.Handle, BrakeSplitter, static_cast<uint32>(Index) + 1),
			PortOf(Runtime.Handle, Knuckle, 3)
		);

		SuspensionNodes.Add(Suspension);
		KnuckleNodes.Add(Knuckle);
		WheelNodes.Add(Wheel);
	}

	if (liminal_vehicle_recompile(Runtime.Handle) != ELiminalStatus::Ok)
	{
		UE_LOG(LogTemp, Error, TEXT("Cirrova 载具组件：liminal 拓扑编译失败。"));
		return false;
	}
	if (!ReadSpatialLinks(Runtime, SuspensionNodes, KnuckleNodes, WheelNodes)
		|| Runtime.Wheels.Num() != WheelCount)
	{
		UE_LOG(LogTemp, Error, TEXT("Cirrova 载具组件：空间装配读取失败。"));
		return false;
	}
	return true;
}

bool UCirrovaVehicleComponent::ReadSpatialLinks(
	FCirrovaVehicleRuntime& Runtime,
	const TArray<uint64>& SuspensionNodes,
	const TArray<uint64>& KnuckleNodes,
	const TArray<uint64>& WheelNodes
)
{
	Runtime.Wheels.SetNum(WheelNodes.Num());
	TArray<FTransform> RestPoses;
	RestPoses.Init(FTransform::Identity, WheelNodes.Num());

	size_t LinkCount = 0;
	if (liminal_vehicle_spatial_link_count(Runtime.Handle, &LinkCount) != ELiminalStatus::Ok) { return false; }

	for (size_t Index = 0; Index < LinkCount; ++Index)
	{
		FLiminalSpatialLink Link;
		if (liminal_vehicle_spatial_link(Runtime.Handle, Index, &Link) != ELiminalStatus::Ok) { continue; }

		// 悬挂链接（子 = 悬挂节点）：弹簧参数。
		int32 WheelIndex = SuspensionNodes.IndexOfByKey(Link.Child);
		if (WheelIndex != INDEX_NONE)
		{
			FCirrovaWheelRuntime& Wheel = Runtime.Wheels[WheelIndex];
			Wheel.SuspensionStiffness = Link.Joint.Stiffness;
			Wheel.SuspensionDamping = Link.Joint.Damping;
			Wheel.RestLength = Link.Joint.RestLength;
			Wheel.MinLength = Link.Joint.MinLength;
			Wheel.MaxLength = Link.Joint.MaxLength;
			continue;
		}

		// 转向节链接（子 = 转向节节点）：转向控制端口。
		WheelIndex = KnuckleNodes.IndexOfByKey(Link.Child);
		if (WheelIndex != INDEX_NONE)
		{
			Runtime.Wheels[WheelIndex].SteeringPort = Link.HasControl ? Link.ControlPort : 0;
			continue;
		}

		// 车轮链接（子 = 车轮节点）：轮心静止位姿。
		WheelIndex = WheelNodes.IndexOfByKey(Link.Child);
		if (WheelIndex != INDEX_NONE) { RestPoses[WheelIndex] = LiminalPoseToUE(Link.PoseInRoot); }
	}

	// 由轮心静止位姿与自由长度反推悬挂上挂点（沿静止位形的上方向）。
	for (int32 Index = 0; Index < WheelNodes.Num(); ++Index)
	{
		FCirrovaWheelRuntime& Wheel = Runtime.Wheels[Index];
		Wheel.ShaftPort = PortOf(Runtime.Handle, WheelNodes[Index], 0);
		Wheel.Radius = WheelConfigs[Index].Radius * 0.01f;
		Wheel.bSteerable = WheelConfigs[Index].bSteerable;
		Wheel.CurrentLength = Wheel.RestLength;

		const FTransform& RestPose = RestPoses[Index];
		Wheel.MountLocation =
			RestPose.GetTranslation() + RestPose.GetUnitAxis(EAxis::Z) * (Wheel.RestLength * 100.0f);
	}
	return true;
}

void UCirrovaVehicleComponent::DrawWheelDebug(
	const FCirrovaWheelState& WheelState,
	const FVector& MountWorld,
	const FVector& ChassisUp
) const
{
	UWorld* World = GetWorld();
	if (!World) { return; }

	const float TraceLength = (WheelState.MaxLength + WheelState.Radius) * 100.0f;
	const FVector TraceEnd = MountWorld - ChassisUp * TraceLength;
	DrawDebugLine(
		World,
		MountWorld,
		TraceEnd,
		WheelState.bInContact ? FColor::Green : FColor::Red,
		false,
		-1.0f,
		0,
		1.0f
	);

	if (WheelState.bInContact)
	{
		DrawDebugPoint(World, WheelState.ContactPoint, 8.0f, FColor::Yellow, false, -1.0f);
		DrawDebugDirectionalArrow(
			World,
			WheelState.ContactPoint,
			WheelState.ContactPoint + WheelState.ContactNormal * WheelState.NormalLoad * 0.01f,
			100.0f,
			FColor::Cyan,
			false,
			-1.0f
		);
		const FVector TireForce =
			WheelState.LongitudinalAxis * WheelState.TireForces.Longitudinal
			+ WheelState.LateralAxis * WheelState.TireForces.Lateral;
		DrawDebugDirectionalArrow(
			World,
			WheelState.ContactPoint,
			WheelState.ContactPoint + TireForce * 0.01f,
			100.0f,
			FColor::Magenta,
			false,
			-1.0f
		);
	}
}

uint64 UCirrovaVehicleComponent::PortOf(
	FLiminalVehicleHandle Handle,
	uint64 Node,
	uint32 Index
) const
{
	uint64 Port = 0;
	if (liminal_vehicle_port(Handle, Node, Index, &Port) != ELiminalStatus::Ok)
	{
		UE_LOG(
			LogTemp,
			Error,
			TEXT("Cirrova 载具组件读取端口失败：节点 %llu 索引 %u。"),
			Node,
			Index
		);
		return 0;
	}
	return Port;
}

bool UCirrovaVehicleComponent::Connect(
	FLiminalVehicleHandle Handle,
	uint64 SourcePort,
	uint64 TargetPort
) const
{
	const ELiminalStatus Status = liminal_vehicle_connect(Handle, SourcePort, TargetPort, nullptr);
	if (Status != ELiminalStatus::Ok)
	{
		UE_LOG(
			LogTemp,
			Error,
			TEXT("Cirrova 载具组件拓扑连接失败，状态码 %d。"),
			static_cast<int32>(Status)
		);
		return false;
	}
	return true;
}
