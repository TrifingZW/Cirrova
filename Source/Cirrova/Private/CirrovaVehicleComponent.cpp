#include "CirrovaVehicleComponent.h"

#include "CirrovaLog.h"
#include "CirrovaSchedulerSubsystem.h"
#include "CirrovaVehicleRuntime.h"
#include "CollisionQueryParams.h"
#include "CollisionShape.h"
#include "DrawDebugHelpers.h"
#include "PBDRigidsSolver.h"
#include "Chaos/ChaosScene.h"
#include "Chaos/PBDRigidsEvolution.h"
#include "Chaos/PBDSuspensionConstraintData.h"
#include "Components/PrimitiveComponent.h"
#include "Engine/OverlapResult.h"
#include "Engine/World.h"
#include "GameFramework/Actor.h"
#include "Physics/GenericPhysicsInterface.h"
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

	/** 在游戏线程收集车轮射线扫掠盒内的宽相候选。 */
	void GatherWheelCandidates(
		const UWorld* World,
		const UPrimitiveComponent* Chassis,
		const AActor* Owner,
		ECollisionChannel TraceChannel,
		const TArray<FCirrovaWheelState>& WheelStates,
		const TArray<FVector>& WheelMountLocations,
		TArray<Chaos::FConstPhysicsObjectHandle>& OutObjects
	)
	{
		if (!World || !Chassis || WheelStates.IsEmpty()) { return; }

		const FTransform ChassisToWorld = Chassis->GetComponentTransform();
		const FVector ChassisUp = ChassisToWorld.GetUnitAxis(EAxis::Z);
		FBox TraceBox(ForceInit);
		for (int32 Index = 0; Index < WheelStates.Num(); ++Index)
		{
			if (!WheelMountLocations.IsValidIndex(Index)) { continue; }

			const FCirrovaWheelState& State = WheelStates[Index];
			const FVector Mount = ChassisToWorld.TransformPosition(WheelMountLocations[Index]);
			FVector Axis = ChassisToWorld.TransformVectorNoScale(State.SuspensionAxis).GetSafeNormal();
			if (Axis.IsNearlyZero()) { Axis = ChassisUp; }

			// 沿悬挂轴下探；轴越倾斜，轮心沿射线方向的半径补偿越长。
			const float AxisVertical = FMath::Max(FMath::Abs(FVector::DotProduct(Axis, ChassisUp)), 0.25f);
			const float TraceLength = (State.MaxLength + State.Radius / AxisVertical) * 100.0f;
			TraceBox += Mount;
			TraceBox += Mount - Axis * TraceLength;
		}
		if (!TraceBox.IsValid || TraceBox.GetSize().Length() <= KINDA_SMALL_NUMBER) { return; }

		// 候选集允许一帧的位姿滞后：按车速扩张包围盒，避免高速时漏掉新进入的对象。
		const float Speed = Chassis->GetComponentVelocity().Size();
		const float Margin = FMath::IsFinite(Speed) ? 50.0f + Speed * 0.1f : 200.0f;
		TraceBox = TraceBox.ExpandBy(Margin);

		TArray<FOverlapResult> Overlaps;
		FCollisionQueryParams TraceParams(
			SCENE_QUERY_STAT(CirrovaWheelCandidates),
			false,
			Owner
		);
		FGenericPhysicsInterface::GeomOverlapMulti(
			World,
			FCollisionShape::MakeBox(TraceBox.GetExtent()),
			TraceBox.GetCenter(),
			FQuat::Identity,
			Overlaps,
			TraceChannel,
			TraceParams,
			FCollisionResponseParams::DefaultResponseParam,
			FCollisionObjectQueryParams::DefaultObjectQueryParam
		);

		TSet<Chaos::FConstPhysicsObjectHandle> Seen;
		Seen.Reserve(Overlaps.Num());
		for (const FOverlapResult& Overlap : Overlaps)
		{
			if (!Overlap.bBlockingHit || !Overlap.PhysicsObject) { continue; }
			if (Seen.Contains(Overlap.PhysicsObject)) { continue; }
			Seen.Add(Overlap.PhysicsObject);
			OutObjects.Add(Overlap.PhysicsObject);
		}
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
			LogCirrova,
			Warning,
			TEXT("Cirrova 载具组件需要拥有者根组件为模拟物理的 UPrimitiveComponent，组件已停用。")
		);
		ChassisPrimitive = nullptr;
		SetComponentTickEnabled(false);
		return;
	}

	// 双重支撑诊断：其它带碰撞的挂载体会与解析悬挂争夺接地来源。
	TArray<UPrimitiveComponent*> PrimitiveComponents;
	GetOwner()->GetComponents(PrimitiveComponents);
	for (const UPrimitiveComponent* Primitive : PrimitiveComponents)
	{
		if (!Primitive || Primitive == ChassisPrimitive.Get()) { continue; }
		if (Primitive->GetCollisionEnabled() == ECollisionEnabled::NoCollision) { continue; }

		UE_LOG(
			LogCirrova,
			Log,
			TEXT("Cirrova 载具组件：检测到额外碰撞组件 %s（配置 %s）。若为车轮视觉件，请设为 NoCollision，避免与解析悬挂构成双重支撑。"),
			*Primitive->GetName(),
			*Primitive->GetCollisionProfileName().ToString()
		);
	}

	FString ConfigurationError;
	if (!ValidateConfiguration(ConfigurationError))
	{
		UE_LOG(
			LogCirrova,
			Error,
			TEXT("Cirrova 载具组件配置无效：%s。组件已停用。"),
			*ConfigurationError
		);
		SetComponentTickEnabled(false);
		return;
	}

	const float ChassisMass = ChassisPrimitive->GetMass();
	UE_LOG(
		LogCirrova,
		Log,
		TEXT("Cirrova 载具组件初始化：底盘质量 %.1f kg，车轮 %d 个。"),
		ChassisMass,
		WheelConfigs.Num()
	);
	if (ChassisMass < 100.0f)
	{
		UE_LOG(
			LogCirrova,
			Warning,
			TEXT("Cirrova 载具组件：底盘质量 %.1f kg 远低于默认参数适用区间（约 1000 kg），请按质量缩放悬挂刚度与阻尼。"),
			ChassisMass
		);
	}

	// 悬挂静平衡与阻尼诊断：按整车质量均分到各轮估算静态压缩、固有频率与阻尼比。
	const float WheelCount = static_cast<float>(FMath::Max(WheelConfigs.Num(), 1));
	const float StaticLoadPerWheel = ChassisMass * 9.8f / WheelCount;
	const float QuarterMass = ChassisMass / WheelCount;
	for (int32 Index = 0; Index < WheelConfigs.Num(); ++Index)
	{
		const FCirrovaWheelConfig& Config = WheelConfigs[Index];
		const float Travel = Config.SuspensionRestLength - Config.SuspensionMinLength;
		const float StaticCompression = StaticLoadPerWheel
			/ FMath::Max(Config.SuspensionStiffness, KINDA_SMALL_NUMBER);
		const float NaturalFrequency = FMath::Sqrt(
			FMath::Max(Config.SuspensionStiffness / QuarterMass, 0.0f)
		) / (2.0f * PI);
		const float DampingRatio = Config.SuspensionDamping / (
			2.0f * FMath::Sqrt(
				FMath::Max(Config.SuspensionStiffness * QuarterMass, KINDA_SMALL_NUMBER)
			)
		);
		const float SagPercent = Travel > KINDA_SMALL_NUMBER
			? StaticCompression / Travel * 100.0f
			: 0.0f;

		UE_LOG(
			LogCirrova,
			Log,
			TEXT("Cirrova 载具组件：车轮 %d 悬挂诊断：静态压缩 %.3f m（占行程 %.1f%%），固有频率 %.2f Hz，阻尼比 %.2f。"),
			Index,
			StaticCompression,
			SagPercent,
			NaturalFrequency,
			DampingRatio
		);

		if (Travel > KINDA_SMALL_NUMBER && StaticCompression > Travel * 0.9f)
		{
			UE_LOG(
				LogCirrova,
				Warning,
				TEXT("Cirrova 载具组件：车轮 %d 的静平衡压缩超过行程 90%%，悬挂过软或底盘过重。"),
				Index
			);
		}
		if (DampingRatio < 0.2f)
		{
			UE_LOG(
				LogCirrova,
				Warning,
				TEXT("Cirrova 载具组件：车轮 %d 的阻尼比 %.2f 偏低，落地与冲击后会出现往复振荡。"),
				Index,
				DampingRatio
			);
		}
		else if (DampingRatio > 1.2f)
		{
			UE_LOG(
				LogCirrova,
				Warning,
				TEXT("Cirrova 载具组件：车轮 %d 的阻尼比 %.2f 偏高，路面跟随与吸收变差。"),
				Index,
				DampingRatio
			);
		}
	}

	UCirrovaSchedulerSubsystem* Scheduler = GetWorld()
		                                        ? GetWorld()->GetSubsystem<UCirrovaSchedulerSubsystem>()
		                                        : nullptr;
	if (!Scheduler)
	{
		UE_LOG(LogCirrova, Error, TEXT("Cirrova 载具组件：未找到调度器子系统，组件已停用。"));
		SetComponentTickEnabled(false);
		return;
	}

	TUniquePtr<FCirrovaVehicleRuntime> Runtime = MakeUnique<FCirrovaVehicleRuntime>();
	if (!BuildTopology(*Runtime))
	{
		SetComponentTickEnabled(false);
		return;
	}

	// 托底诊断：静平衡时轮胎最低点必须低于底盘碰撞体最低点，否则悬挂永远无法承担整车重量。
	const FTransform ChassisToWorld = ChassisPrimitive->GetComponentTransform();
	const FBoxSphereBounds ChassisBounds = ChassisPrimitive->CalcBounds(ChassisToWorld);
	const float ChassisBottom = ChassisBounds.Origin.Z - ChassisBounds.BoxExtent.Z;
	constexpr float GroundClearanceMargin = 2.0f;
	for (int32 Index = 0; Index < Runtime->Wheels.Num(); ++Index)
	{
		const FCirrovaWheelRuntime& Wheel = Runtime->Wheels[Index];
		FVector AxisWorld =
			ChassisToWorld.TransformVectorNoScale(Wheel.SuspensionAxis).GetSafeNormal();
		if (AxisWorld.IsNearlyZero()) { AxisWorld = ChassisToWorld.GetUnitAxis(EAxis::Z); }

		const float Travel = FMath::Max(Wheel.RestLength - Wheel.MinLength, 0.0f);
		const float RequiredCompression = StaticLoadPerWheel
			/ FMath::Max(Wheel.SuspensionStiffness, KINDA_SMALL_NUMBER);
		const float StaticCompression = FMath::Min(RequiredCompression, Travel);
		const FVector MountWorld = ChassisToWorld.TransformPosition(Wheel.MountLocation);
		const FVector StaticWheelBottom = MountWorld
			- AxisWorld * ((Wheel.RestLength - StaticCompression) * 100.0f)
			- FVector::UpVector * (Wheel.Radius * 100.0f);
		const float Clearance = ChassisBottom - StaticWheelBottom.Z;

		UE_LOG(
			LogCirrova,
			Log,
			TEXT("Cirrova 载具组件：车轮 %d 静平衡托底余量 %.1f cm（底盘底面 %.1f cm，轮胎底面 %.1f cm）。"),
			Index,
			Clearance,
			ChassisBottom,
			StaticWheelBottom.Z
		);

		if (RequiredCompression > Travel)
		{
			UE_LOG(
				LogCirrova,
				Warning,
				TEXT("Cirrova 载具组件：车轮 %d 的静平衡压缩 %.3f m 超过行程 %.3f m，悬挂会先触底。"),
				Index,
				RequiredCompression,
				Travel
			);
		}
		if (Clearance < GroundClearanceMargin)
		{
			UE_LOG(
				LogCirrova,
				Warning,
				TEXT("Cirrova 载具组件：车轮 %d 的底盘碰撞体不足静平衡轮胎接地面 %.1f cm（要求余量 %.0f cm），悬挂无法承载整车重量。请抬高底盘碰撞体、降低挂点或增大悬挂刚度。"),
				Index,
				GroundClearanceMargin - Clearance,
				GroundClearanceMargin
			);
		}
	}

	WheelStates.SetNum(Runtime->Wheels.Num());
	WheelMountLocations.SetNum(Runtime->Wheels.Num());
	for (int32 Index = 0; Index < Runtime->Wheels.Num(); ++Index)
	{
		const FCirrovaWheelRuntime& Wheel = Runtime->Wheels[Index];
		WheelStates[Index].MaxLength = Wheel.MaxLength;
		WheelStates[Index].Radius = Wheel.Radius;
		WheelStates[Index].SuspensionAxis = Wheel.SuspensionAxis;
		WheelMountLocations[Index] = Wheel.MountLocation;
	}

	if (!Scheduler->RegisterVehicle(this, MoveTemp(Runtime)))
	{
		UE_LOG(LogCirrova, Error, TEXT("Cirrova 载具组件：注册到调度器失败，组件已停用。"));
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
		for (int32 Index = 0; Index < WheelStates.Num(); ++Index)
		{
			const FVector MountWorld = WheelMountLocations.IsValidIndex(Index)
				                           ? ChassisToWorld.TransformPosition(WheelMountLocations[Index])
				                           : ChassisToWorld.GetLocation();
			DrawWheelDebug(WheelStates[Index], MountWorld, ChassisToWorld);
		}
}

void UCirrovaVehicleComponent::SetThrottle(float Value) { ThrottleInput = Value; }

void UCirrovaVehicleComponent::SetSteering(float Value) { SteeringInput = Value; }

void UCirrovaVehicleComponent::SetBrake(float Value) { BrakeInput = Value; }

void UCirrovaVehicleComponent::SetGear(int32 Gear) { GearInput = Gear; }

float UCirrovaVehicleComponent::GetEngineRPM() const { return EngineOmega * 60.0f / static_cast<float>(2.0 * PI); }

FString UCirrovaVehicleComponent::GetWheelDebugText(int32 WheelIndex) const
{
	if (!WheelStates.IsValidIndex(WheelIndex)) { return TEXT("无效车轮索引"); }

	const FCirrovaWheelState& State = WheelStates[WheelIndex];
	const TCHAR* ContactText = State.bInContact ? TEXT("接地") : TEXT("离地");
	const TCHAR* CouplingText = bDebugTireConstraintCoupling
		? (State.bTireConstraintApplied ? TEXT("约束/启用") : TEXT("约束/本步未启用"))
		: TEXT("外力回退");
	const TCHAR* SuspensionText = State.bNativeSuspensionActive
		? (State.bUsingConstraintLength ? TEXT("原生约束/约束长度") : TEXT("原生约束/射线长度"))
		: TEXT("弹簧力");
	const TCHAR* ClampText = State.bCompressionVelocityClamped ? TEXT("  [钳制]") : TEXT("");

	return FString::Printf(
		TEXT("轮 %d %s  耦合:%s  悬挂:%s\n")
		TEXT("弹簧 %.3f m  约束 %.3f m  压缩 %.3f m  压缩速度 %.2f m/s%s\n")
		TEXT("法向载荷 %.0f N  悬挂力 %.0f N  防倾 %.0f N  摩擦圆 %.0f N\n")
		TEXT("滑移率 %.3f  侧偏角 %.1f°  轮面 %.2f m/s  惯量 %.2f kg·m²\n")
		TEXT("轮胎力 纵 %.0f N 侧 %.0f N  力矩 %.0f N·m  滚动 %.0f N\n")
		TEXT("摩擦利用 %.2f  冲量 纵 %.2f / 侧 %.2f N·s（窗口 %.1f ms）"),
		WheelIndex,
		ContactText,
		CouplingText,
		SuspensionText,
		State.SuspensionLength,
		State.SuspensionConstraintLength,
		State.Compression,
		State.CompressionVelocity,
		ClampText,
		State.NormalLoad,
		State.SuspensionForce,
		State.AntiRollForce,
		State.FrictionForceLimit,
		State.SlipRatio,
		State.SlipAngleDegrees,
		State.SurfaceSpeed,
		State.EffectiveInertia,
		State.TireLongitudinalForce,
		State.TireLateralForce,
		State.WheelTorque,
		State.RollingResistance,
		State.FrictionUtilization,
		State.LongitudinalImpulse,
		State.LateralImpulse,
		State.ImpulseWindow * 1000.0f
	);
}

void UCirrovaVehicleComponent::LogDebugSnapshot() const
{
	UE_LOG(
		LogCirrova,
		Log,
		TEXT("Cirrova 遥测：耦合=%s，子步=%d，求解器步长=%.3f ms，接地=%s，发动机=%.0f RPM。"),
		bDebugTireConstraintCoupling ? TEXT("求解器约束") : TEXT("外力回退"),
		DebugAppliedSubsteps,
		DebugSolverDeltaTime * 1000.0f,
		bWasGrounded ? TEXT("是") : TEXT("否"),
		GetEngineRPM()
	);
	for (int32 Index = 0; Index < WheelStates.Num(); ++Index)
	{
		UE_LOG(LogCirrova, Log, TEXT("%s"), *GetWheelDebugText(Index));
	}
}

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
	Out.BlockingObjects.Reset();

	const UPrimitiveComponent* Chassis = ChassisPrimitive.Get();
	if (!Chassis) { return; }

	if (const FBodyInstance* BodyInstance = Chassis->GetBodyInstance()) { Out.Proxy = BodyInstance->GetPhysicsActor(); }
	GatherWheelCandidates(
		GetWorld(),
		Chassis,
		GetOwner(),
		WheelTraceChannel.GetValue(),
		WheelStates,
		WheelMountLocations,
		Out.BlockingObjects
	);
}

void UCirrovaVehicleComponent::InitializeSuspensionConstraints(FCirrovaVehicleRuntime& Runtime)
{
	if (!bUseNativeSuspension || !ChassisPrimitive.IsValid()) { return; }

	const FBodyInstance* BodyInstance = ChassisPrimitive->GetBodyInstance();
	IPhysicsProxyBase* Proxy = BodyInstance ? BodyInstance->GetPhysicsActor() : nullptr;
	if (!Proxy || Proxy->GetType() != EPhysicsProxyType::SingleParticleProxy)
	{
		UE_LOG(
			LogCirrova,
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
			LogCirrova,
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
			LogCirrova,
			Warning,
			TEXT("Cirrova 载具组件：无法获取物理场景，悬挂回退为弹簧力施加。")
		);
		return;
	}

	FLockedWritePhysicsObjectExternalInterface Interface = FPhysicsObjectExternalInterface::LockWrite(Scene);
	if (!Interface->GetParticle(PhysicsObject))
	{
		UE_LOG(
			LogCirrova,
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
				LogCirrova,
				Warning,
				TEXT("Cirrova 载具组件：车轮 %d 的自由长度不大于行程下限，悬挂回退为弹簧力施加。"),
				Index
			);
			continue;
		}

		const FVector Axis = Wheel.SuspensionAxis.IsNearlyZero()
			                     ? FVector::UpVector
			                     : Wheel.SuspensionAxis;
		// 约束的本地偏移是自由长度下的轮心（相对底盘原点），不是上挂点；
		// 求解长度以该偏移为静止参考，压缩为负、伸展为正。
		const FVector WheelRestLocation =
			Wheel.MountLocation - Axis * (Wheel.RestLength * 100.0f);

		const FPhysicsConstraintHandle Handle = FPhysicsInterface::CreateSuspension(
			PhysicsObject,
			WheelRestLocation
		);
		if (!Handle.IsValid())
		{
			UE_LOG(
				LogCirrova,
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
				LogCirrova,
				Warning,
				TEXT("Cirrova 载具组件：车轮 %d 悬挂约束类型无效，回退为弹簧力施加。"),
				Index
			);
			continue;
		}

		Constraint->SetHardstopStiffness(1.0f);
		Constraint->SetHardstopVelocityCompensation(1.0f);
		Constraint->SetSpringPreload(0.0f);
		// 弹簧在位置求解的每次迭代中都会完整施加一次，按实际迭代数折算设定值，
		// 使有效刚度/阻尼等于 liminal 的 k / c（官方 ChaosVehicles 固定取 1/4）。
		auto* Evolution = Solver->GetEvolution();
		const int32 PositionIterations = Evolution
			? FMath::Max(Evolution->GetNumPositionIterations(), 1)
			: 4;
		const float IterationScale = 1.0f / static_cast<float>(PositionIterations);
		Constraint->SetSpringStiffness(
			FMath::Max(Wheel.SuspensionStiffness, 0.0f) * IterationScale
		);
		Constraint->SetSpringDamping(
			FMath::Max(Wheel.SuspensionDamping, 0.0f) * SolverDeltaTime * IterationScale
		);
		// Chaos 悬挂约束的行程是相对自由长度的有符号量：压缩为负、伸展为正。
		// liminal 弹簧在自由长度以上不产生伸展力，因此上限取 0（自由长度），
		// 约束行程仅覆盖压缩区间 [−(rest−min), 0]。
		const float CompressionTravel =
			FMath::Max(Wheel.RestLength - Wheel.MinLength, 0.0f) * 100.0f;
		Constraint->SetMinLength(-CompressionTravel);
		Constraint->SetMaxLength(0.0f);
		Constraint->SetAxis(Chaos::FVec3(Axis.X, Axis.Y, Axis.Z));
		// 保持 Enabled=false：目标点与法向在首个物理步由 SetSuspensionTarget 写入。

		Wheel.SuspensionConstraint = Constraint;
		Wheel.bUseSuspensionConstraint = true;
		++NativeCount;

		UE_LOG(
			LogCirrova,
			Log,
			TEXT("Cirrova 载具组件：车轮 %d 静止轮心 (%.1f, %.1f, %.1f) cm，约束行程 压缩 %.1f cm（上限为自由长度），刚度/阻尼按位置迭代 %d 次折算。"),
			Index,
			WheelRestLocation.X,
			WheelRestLocation.Y,
			WheelRestLocation.Z,
			CompressionTravel,
			PositionIterations
		);
	}

	UE_LOG(
		LogCirrova,
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
		Target.SurfaceSpeed = Source.SurfaceSpeed;
		Target.EffectiveInertia = Source.EffectiveInertia;
		Target.SuspensionLength = Source.SuspensionLength;
		Target.SuspensionForce = Source.SuspensionForce;
		Target.AntiRollForce = Source.AntiRollForce;
		Target.NormalLoad = Source.NormalLoad;
		Target.Compression = Source.Compression;
		Target.CompressionVelocity = Source.CompressionVelocity;
		Target.bCompressionVelocityClamped = Source.bCompressionVelocityClamped;
		Target.bNativeSuspensionActive = Source.bNativeSuspensionActive;
		Target.bUsingConstraintLength = Source.bUsingConstraintLength;
		Target.SuspensionConstraintLength = Source.SuspensionConstraintLength;
		Target.TireLongitudinalForce = Source.TireLongitudinalForce;
		Target.TireLateralForce = Source.TireLateralForce;
		Target.WheelTorque = Source.WheelTorque;
		Target.RollingResistance = Source.RollingResistance;
		Target.FrictionForceLimit = Source.FrictionForceLimit;
		Target.LateralForceLimit = Source.LateralForceLimit;
		Target.FrictionUtilization = Source.FrictionUtilization;
		Target.bTireConstraintApplied = Source.bTireConstraintApplied;
		Target.LongitudinalImpulse = Source.LongitudinalImpulse;
		Target.LateralImpulse = Source.LateralImpulse;
		Target.ImpulseWindow = Source.ImpulseWindow;
		Target.ContactLongitudinalSpeed = Source.ContactLongitudinalSpeed;
		Target.ContactLateralSpeed = Source.ContactLateralSpeed;
		Target.SlipRatio = Source.SlipRatio;
		Target.SlipAngleDegrees = Source.SlipAngleDegrees;
		Target.SuspensionAxis = Source.SuspensionAxis;
		Target.ContactPoint = Source.ContactPoint;
		Target.ContactNormal = Source.ContactNormal;
		Target.LongitudinalAxis = Source.LongitudinalAxis;
		Target.LateralAxis = Source.LateralAxis;
		Target.ContactVelocity = Source.ContactVelocity;
		Target.GroundVelocity = Source.GroundVelocity;
	}

	bDebugTireConstraintCoupling = In.bTireConstraintCoupling;
	DebugAppliedSubsteps = In.AppliedSubsteps;
	DebugSolverDeltaTime = In.SolverDeltaTime;
	bDebugSubstepBudgetExceeded = In.bSubstepBudgetExceeded;

	// 双重支撑诊断：接地但四轮持续无载荷，说明悬挂未承载，车体由其它碰撞体支撑。
	bool bAnyWheelLoaded = false;
	for (const FCirrovaWheelOutput& Wheel : In.Wheels)
	{
		if (Wheel.bInContact && Wheel.NormalLoad > 100.0f) { bAnyWheelLoaded = true; break; }
	}
	if (In.bGrounded && !bAnyWheelLoaded)
	{
		NoSuspensionLoadSeconds += In.SolverDeltaTime;
		if (NoSuspensionLoadSeconds > 0.5f && !bWarnedNoSuspensionLoad)
		{
			bWarnedNoSuspensionLoad = true;
			UE_LOG(
				LogCirrova,
				Warning,
				TEXT("Cirrova 载具组件：车辆接地但四轮法向载荷持续低于 100 N（%.1f s），悬挂未承载。请检查是否有车轮/车身碰撞体在支撑车体（双重支撑）。"),
				NoSuspensionLoadSeconds
			);
		}
	}
	else
	{
		NoSuspensionLoadSeconds = 0.0f;
	}

	// 轮胎未进入求解器约束：记录一次回退警告，容器或求解器不可用。
	if (!In.bTireConstraintCoupling && !bWarnedCouplingFallback)
	{
		bWarnedCouplingFallback = true;
		UE_LOG(
			LogCirrova,
			Warning,
			TEXT("Cirrova 载具组件：轮胎未进入求解器约束，已回退外力耦合（子步 %d，求解器步长 %.2f ms）。请检查约束容器与调度器日志。"),
			In.AppliedSubsteps,
			In.SolverDeltaTime * 1000.0f
		);
	}

	if (In.bSubstepBudgetExceeded && !bWarnedSubstepBudget)
	{
		bWarnedSubstepBudget = true;
		UE_LOG(
			LogCirrova,
			Warning,
			TEXT("Cirrova 载具组件：求解器步长超过内部子步预算（2.5 ms × 32），轮胎显式耦合的数值稳定性会退化。")
		);
	}

	if (In.bGrounded != bWasGrounded)
	{
		bWasGrounded = In.bGrounded;
		OnVehicleGroundedEvent.Broadcast(bWasGrounded);
	}
}

bool UCirrovaVehicleComponent::ValidateConfiguration(FString& OutError) const
{
	auto Fail = [&OutError](const FString& Message)
	{
		OutError = Message;
		return false;
	};

	const uint32 RuntimeAbiVersion = liminal_abi_version();
	if (RuntimeAbiVersion != LiminalAbiVersion)
	{
		return Fail(
			FString::Printf(
				TEXT("liminal ABI 版本不匹配：头文件 %u，静态库 %u；请重新构建 ThirdParty/liminal"),
				LiminalAbiVersion,
				RuntimeAbiVersion
			)
		);
	}

	if (WheelConfigs.IsEmpty()) { return Fail(TEXT("未配置车轮")); }
	if (GearRatios.IsEmpty()) { return Fail(TEXT("未配置变速箱传动比")); }

	const float ChassisMass = ChassisPrimitive.IsValid() ? ChassisPrimitive->GetMass() : 0.0f;
	if (!FMath::IsFinite(ChassisMass) || ChassisMass <= 0.0f) { return Fail(TEXT("底盘质量必须为正有限值")); }

	if (!FMath::IsFinite(EngineMaxTorque) || EngineMaxTorque < 0.0f) { return Fail(TEXT("发动机最大扭矩必须为非负有限值")); }
	if (!FMath::IsFinite(EngineMaxAngularVelocity) || EngineMaxAngularVelocity <= 0.0f)
	{
		return Fail(TEXT("发动机最大角速度必须为正有限值"));
	}
	if (!FMath::IsFinite(EngineInertia) || EngineInertia <= 0.0f) { return Fail(TEXT("发动机转动惯量必须为正有限值")); }
	if (!FMath::IsFinite(EngineIdleRPM) || EngineIdleRPM < 0.0f
		|| !FMath::IsFinite(EngineIdleTorque) || EngineIdleTorque < 0.0f
		|| !FMath::IsFinite(EngineBrakingTorque) || EngineBrakingTorque < 0.0f)
	{
		return Fail(TEXT("发动机怠速与制动参数必须为非负有限值"));
	}
	for (int32 Index = 0; Index < EngineTorqueCurve.Num(); ++Index)
	{
		const FCirrovaEngineCurvePoint& Point = EngineTorqueCurve[Index];
		if (!FMath::IsFinite(Point.RPM) || Point.RPM < 0.0f
			|| !FMath::IsFinite(Point.TorqueScale) || Point.TorqueScale < 0.0f)
		{
			return Fail(FString::Printf(TEXT("扭矩曲线采样点 %d 无效"), Index));
		}
	}

	for (int32 Index = 0; Index < GearRatios.Num(); ++Index)
	{
		if (!FMath::IsFinite(GearRatios[Index]) || GearRatios[Index] <= 0.0f)
		{
			return Fail(FString::Printf(TEXT("变速箱传动比 %d 必须为正有限值"), Index));
		}
	}
	if (!FMath::IsFinite(GearboxInertia) || GearboxInertia < 0.0f
		|| !FMath::IsFinite(AxleInertia) || AxleInertia < 0.0f
		|| !FMath::IsFinite(DifferentialMaxTorque) || DifferentialMaxTorque < 0.0f
		|| !FMath::IsFinite(WheelInertia) || WheelInertia <= 0.0f
		|| !FMath::IsFinite(WheelFrictionTorque) || WheelFrictionTorque < 0.0f
		|| !FMath::IsFinite(MaxBrakeTorque) || MaxBrakeTorque < 0.0f)
	{
		return Fail(TEXT("传动系惯量、摩擦与制动参数必须为非负有限值，车轮惯量必须为正"));
	}

	if (!FMath::IsFinite(TireFriction) || TireFriction < 0.0f
		|| !FMath::IsFinite(TireLateralStiffness) || TireLateralStiffness < 0.0f
		|| !FMath::IsFinite(TireLoadSensitivity) || TireLoadSensitivity < 0.0f
		|| !FMath::IsFinite(TireReferenceLoad) || TireReferenceLoad <= 0.0f
		|| !FMath::IsFinite(TireRollingResistance) || TireRollingResistance < 0.0f)
	{
		return Fail(TEXT("轮胎附着、侧向刚度、载荷敏感、参考载荷与滚动阻力参数无效"));
	}

	if (!FMath::IsFinite(MaxSteeringAngle) || MaxSteeringAngle < 0.0f) { return Fail(TEXT("最大转向角必须为非负有限值")); }

	for (int32 Index = 0; Index < WheelConfigs.Num(); ++Index)
	{
		const FCirrovaWheelConfig& Config = WheelConfigs[Index];
		if (!FMath::IsFinite(Config.MountLocation.X)
			|| !FMath::IsFinite(Config.MountLocation.Y)
			|| !FMath::IsFinite(Config.MountLocation.Z))
		{
			return Fail(FString::Printf(TEXT("车轮 %d 的挂点位置必须为有限值"), Index));
		}
		if (!FMath::IsFinite(Config.Radius) || Config.Radius <= 0.0f)
		{
			return Fail(FString::Printf(TEXT("车轮 %d 的半径必须为正有限值"), Index));
		}
		if (!FMath::IsFinite(Config.SuspensionStiffness) || Config.SuspensionStiffness <= 0.0f
			|| !FMath::IsFinite(Config.SuspensionDamping) || Config.SuspensionDamping < 0.0f)
		{
			return Fail(FString::Printf(TEXT("车轮 %d 的悬挂刚度必须为正、阻尼必须为非负有限值"), Index));
		}
		if (!FMath::IsFinite(Config.SuspensionRestLength) || !FMath::IsFinite(Config.SuspensionMinLength)
			|| !FMath::IsFinite(Config.SuspensionMaxLength)
			|| Config.SuspensionMinLength <= 0.0f
			|| Config.SuspensionMaxLength < Config.SuspensionMinLength
			|| Config.SuspensionRestLength < Config.SuspensionMinLength
			|| Config.SuspensionRestLength > Config.SuspensionMaxLength)
		{
			return Fail(
				FString::Printf(
					TEXT("车轮 %d 的悬挂行程无效：须满足 0 < min ≤ rest ≤ max"),
					Index
				)
			);
		}
	}

	return true;
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
				LogCirrova,
				Warning,
				TEXT("Cirrova 载具组件：防倾杆 %d 的车轮索引无效，已跳过。"),
				Index
			);
			continue;
		}
		if (!FMath::IsFinite(Config.Stiffness) || Config.Stiffness < 0.0f)
		{
			UE_LOG(
				LogCirrova,
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
				LogCirrova,
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
		UE_LOG(LogCirrova, Error, TEXT("Cirrova 载具组件：liminal 句柄创建失败。"));
		return false;
	}

	const int32 WheelCount = WheelConfigs.Num();
	if (WheelCount == 0 || GearRatios.Num() == 0)
	{
		UE_LOG(LogCirrova, Warning, TEXT("Cirrova 载具组件缺少车轮配置或传动比，组件已停用。"));
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
	if (!Connect(Runtime.Handle, Runtime.ThrottleSignalPort, PortOf(Runtime.Handle, Engine, 0))
		|| !Connect(Runtime.Handle, Runtime.GearSignalPort, PortOf(Runtime.Handle, Gearbox, 0))
		|| !Connect(Runtime.Handle, Runtime.SteeringSignalPort, PortOf(Runtime.Handle, SteeringSplitter, 0))
		|| !Connect(Runtime.Handle, Runtime.BrakeSignalPort, PortOf(Runtime.Handle, BrakeSplitter, 0)))
	{
		return false;
	}

	// 动力链：发动机 → 变速箱 → 轮轴/差速器。
	if (!Connect(Runtime.Handle, PortOf(Runtime.Handle, Engine, 1), PortOf(Runtime.Handle, Gearbox, 1))
		|| !Connect(Runtime.Handle, PortOf(Runtime.Handle, Gearbox, 2), PortOf(Runtime.Handle, Distributor, 0)))
	{
		return false;
	}

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

		if (!Connect(Runtime.Handle, PortOf(Runtime.Handle, Chassis, Index), PortOf(Runtime.Handle, Suspension, 0))
			|| !Connect(Runtime.Handle, PortOf(Runtime.Handle, Suspension, 1), PortOf(Runtime.Handle, Knuckle, 0))
			|| !Connect(Runtime.Handle, PortOf(Runtime.Handle, Knuckle, 1), PortOf(Runtime.Handle, Wheel, 1))
			|| !Connect(
				Runtime.Handle,
				PortOf(Runtime.Handle, Distributor, static_cast<uint32>(Index) + 1),
				PortOf(Runtime.Handle, Wheel, 0)
			)
			|| !Connect(Runtime.Handle, PortOf(Runtime.Handle, Knuckle, 4), PortOf(Runtime.Handle, Wheel, 0)))
		{
			return false;
		}

		// 不可转向的车轮不接转向信号，端口保持零值。
		if (Config.bSteerable
			&& !Connect(
				Runtime.Handle,
				PortOf(Runtime.Handle, SteeringSplitter, static_cast<uint32>(Index) + 1),
				PortOf(Runtime.Handle, Knuckle, 2)
			)) { return false; }
		if (!Connect(
			Runtime.Handle,
			PortOf(Runtime.Handle, BrakeSplitter, static_cast<uint32>(Index) + 1),
			PortOf(Runtime.Handle, Knuckle, 3)
		)) { return false; }

		SuspensionNodes.Add(Suspension);
		KnuckleNodes.Add(Knuckle);
		WheelNodes.Add(Wheel);
	}

	if (liminal_vehicle_recompile(Runtime.Handle) != ELiminalStatus::Ok)
	{
		UE_LOG(LogCirrova, Error, TEXT("Cirrova 载具组件：liminal 拓扑编译失败。"));
		return false;
	}
	if (!ReadSpatialLinks(Runtime, SuspensionNodes, KnuckleNodes, WheelNodes)
		|| Runtime.Wheels.Num() != WheelCount)
	{
		UE_LOG(LogCirrova, Error, TEXT("Cirrova 载具组件：空间装配读取失败。"));
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
	if (WheelConfigs.Num() != WheelNodes.Num())
	{
		UE_LOG(LogCirrova, Error, TEXT("Cirrova 载具组件：车轮配置数量与拓扑车轮节点数量不一致。"));
		return false;
	}

	Runtime.Wheels.SetNum(WheelNodes.Num());

	TArray<bool> SuspensionSeen;
	TArray<bool> KnuckleSeen;
	TArray<bool> WheelSeen;
	SuspensionSeen.Init(false, WheelNodes.Num());
	KnuckleSeen.Init(false, WheelNodes.Num());
	WheelSeen.Init(false, WheelNodes.Num());

	size_t LinkCount = 0;
	if (liminal_vehicle_spatial_link_count(Runtime.Handle, &LinkCount) != ELiminalStatus::Ok) { return false; }

	for (size_t Index = 0; Index < LinkCount; ++Index)
	{
		FLiminalSpatialLink Link;
		if (liminal_vehicle_spatial_link(Runtime.Handle, Index, &Link) != ELiminalStatus::Ok) { continue; }

		// 悬挂链接（子 = 悬挂节点）：弹簧参数、上挂点位姿与悬挂轴。
		int32 WheelIndex = SuspensionNodes.IndexOfByKey(Link.Child);
		if (WheelIndex != INDEX_NONE)
		{
			FCirrovaWheelRuntime& Wheel = Runtime.Wheels[WheelIndex];
			const FTransform Pose = LiminalPoseToUE(Link.PoseInRoot);
			if (Pose.GetTranslation().ContainsNaN())
			{
				UE_LOG(LogCirrova, Error, TEXT("Cirrova 载具组件：车轮 %d 的悬挂挂点位姿无效。"), WheelIndex);
				return false;
			}
			Wheel.SuspensionStiffness = Link.Joint.Stiffness;
			Wheel.SuspensionDamping = Link.Joint.Damping;
			Wheel.RestLength = Link.Joint.RestLength;
			Wheel.MinLength = Link.Joint.MinLength;
			Wheel.MaxLength = Link.Joint.MaxLength;
			Wheel.MountLocation = Pose.GetTranslation();
			// 关节轴在子节点本地系（liminal）；Pose.GetRotation() 已含基变换与子到根旋转。
			const FVector AxisLiminal(
				Link.Joint.Axis.X,
				Link.Joint.Axis.Y,
				Link.Joint.Axis.Z
			);
			Wheel.SuspensionAxis =
				Pose.GetRotation().RotateVector(AxisLiminal).GetSafeNormal();
			if (Wheel.SuspensionAxis.ContainsNaN())
			{
				UE_LOG(LogCirrova, Error, TEXT("Cirrova 载具组件：车轮 %d 的悬挂轴无效。"), WheelIndex);
				return false;
			}
			if (Wheel.SuspensionAxis.IsNearlyZero()) { Wheel.SuspensionAxis = FVector::UpVector; }
			SuspensionSeen[WheelIndex] = true;
			continue;
		}

		// 转向节链接（子 = 转向节节点）：转向控制端口与转向轴。
		WheelIndex = KnuckleNodes.IndexOfByKey(Link.Child);
		if (WheelIndex != INDEX_NONE)
		{
			FCirrovaWheelRuntime& Wheel = Runtime.Wheels[WheelIndex];
			const FTransform Pose = LiminalPoseToUE(Link.PoseInRoot);
			Wheel.SteeringPort = Link.HasControl ? Link.ControlPort : 0;
			// 关节轴在子节点本地系（liminal）；Pose.GetRotation() 已含基变换与子到根旋转。
			const FVector AxisLiminal(
				Link.Joint.Axis.X,
				Link.Joint.Axis.Y,
				Link.Joint.Axis.Z
			);
			Wheel.SteeringAxis =
				Pose.GetRotation().RotateVector(AxisLiminal).GetSafeNormal();
			if (Wheel.SteeringAxis.ContainsNaN())
			{
				UE_LOG(LogCirrova, Error, TEXT("Cirrova 载具组件：车轮 %d 的转向轴无效。"), WheelIndex);
				return false;
			}
			if (Wheel.SteeringAxis.IsNearlyZero()) { Wheel.SteeringAxis = FVector::UpVector; }
			KnuckleSeen[WheelIndex] = true;
			continue;
		}

		// 车轮链接（子 = 车轮节点）：确认空间链完整，轮心由悬挂参数与射线求解。
		WheelIndex = WheelNodes.IndexOfByKey(Link.Child);
		if (WheelIndex != INDEX_NONE) { WheelSeen[WheelIndex] = true; }
	}

	for (int32 Index = 0; Index < WheelNodes.Num(); ++Index)
	{
		if (!SuspensionSeen[Index] || !KnuckleSeen[Index] || !WheelSeen[Index])
		{
			UE_LOG(
				LogCirrova,
				Error,
				TEXT("Cirrova 载具组件：车轮 %d 的空间链不完整，无法读取装配参数。"),
				Index
			);
			return false;
		}

		FCirrovaWheelRuntime& Wheel = Runtime.Wheels[Index];
		Wheel.ShaftPort = PortOf(Runtime.Handle, WheelNodes[Index], 0);
		if (Wheel.ShaftPort == 0)
		{
			UE_LOG(LogCirrova, Error, TEXT("Cirrova 载具组件：车轮 %d 的旋转端口读取失败。"), Index);
			return false;
		}
		Wheel.Radius = WheelConfigs[Index].Radius * 0.01f;
		Wheel.bSteerable = WheelConfigs[Index].bSteerable;
		Wheel.CurrentLength = Wheel.RestLength;
		Wheel.RaycastLength = Wheel.RestLength;
		Wheel.bConstraintLengthValid = false;
		Wheel.EffectiveInertia = Runtime.WheelInertia;
	}
	return true;
}

void UCirrovaVehicleComponent::DrawWheelDebug(
	const FCirrovaWheelState& WheelState,
	const FVector& MountWorld,
	const FTransform& ChassisToWorld
) const
{
	UWorld* World = GetWorld();
	if (!World) { return; }

	const FVector ChassisUp = ChassisToWorld.GetUnitAxis(EAxis::Z);
	FVector Axis = ChassisToWorld.TransformVectorNoScale(WheelState.SuspensionAxis).GetSafeNormal();
	if (Axis.IsNearlyZero()) { Axis = ChassisUp; }
	const float AxisVertical = FMath::Max(FMath::Abs(FVector::DotProduct(Axis, ChassisUp)), 0.25f);
	const float TraceLength = (WheelState.MaxLength + WheelState.Radius / AxisVertical) * 100.0f;
	const FVector TraceEnd = MountWorld - Axis * TraceLength;
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

	// 弹簧长度与射线行程同时绘制，便于对照约束求解值与射线估计。
	const FVector SpringEnd = MountWorld - Axis * (WheelState.SuspensionLength * 100.0f);
	DrawDebugLine(World, MountWorld, SpringEnd, FColor::White, false, -1.0f, 0, 1.5f);
	DrawDebugPoint(World, SpringEnd, 6.0f, FColor::Silver, false, -1.0f);

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
			WheelState.LongitudinalAxis * WheelState.TireLongitudinalForce
			+ WheelState.LateralAxis * WheelState.TireLateralForce;
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
			LogCirrova,
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
			LogCirrova,
			Error,
			TEXT("Cirrova 载具组件拓扑连接失败，状态码 %d。"),
			static_cast<int32>(Status)
		);
		return false;
	}
	return true;
}
