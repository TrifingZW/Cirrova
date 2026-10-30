#include "CirrovaVehicleComponent.h"

#include "DrawDebugHelpers.h"
#include "Components/PrimitiveComponent.h"
#include "Engine/World.h"

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

	// 滑移归一化参考速度（m/s），低于该值按该值归一，避免低速除零。
	constexpr float SlipReferenceSpeed = 1.0f;

	// 帧内子步上限：轮胎力与轮速为显式耦合，子步长须低于数值刚度阈值，
	// 否则轮速逐子步振荡放大（Jolt 以求解器内隐式约束消除该问题）。
	constexpr float MaxSubstepDeltaTime = 0.0025f;
	constexpr int32 MaxSubstepsPerTick = 32;

	// 阻尼压缩速度钳制（m/s），等效减振器泄压阀，防止落地帧产生过大冲量。
	constexpr float MaxCompressionVelocity = 2.0f;
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
	ChassisMass = ChassisPrimitive->GetMass();
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

	BuildTopology();
}

void UCirrovaVehicleComponent::EndPlay(const EEndPlayReason::Type EndPlayReason)
{
	if (VehicleHandle)
	{
		liminal_vehicle_free(VehicleHandle);
		VehicleHandle = nullptr;
	}
	Super::EndPlay(EndPlayReason);
}

void UCirrovaVehicleComponent::TickComponent(
	float DeltaTime,
	ELevelTick TickType,
	FActorComponentTickFunction* ThisTickFunction
)
{
	Super::TickComponent(DeltaTime, TickType, ThisTickFunction);

	if (!VehicleHandle || !ChassisPrimitive.IsValid() || Wheels.Num() == 0) { return; }
	if (DeltaTime <= KINDA_SMALL_NUMBER) { return; }

	// 1) 驾驶信号：转向写入目标角（弧度），其余为归一化标量。
	const float SteeringAngle =
		FMath::Clamp(SteeringInput, -1.0f, 1.0f) * FMath::DegreesToRadians(MaxSteeringAngle);
	liminal_vehicle_write_signal(VehicleHandle, ThrottleSignalPort, FMath::Clamp(ThrottleInput, 0.0f, 1.0f));
	liminal_vehicle_write_signal(VehicleHandle, SteeringSignalPort, SteeringAngle);
	liminal_vehicle_write_signal(VehicleHandle, GearSignalPort, static_cast<float>(GearInput));
	liminal_vehicle_write_signal(VehicleHandle, BrakeSignalPort, FMath::Clamp(BrakeInput, 0.0f, 1.0f));

	// 2) 逐轮接触解（帧内几何不变）：射线、接触坐标系、悬挂力与接触点速度。
	for (FCirrovaWheelRuntime& Wheel : Wheels) { SolveContact(Wheel, DeltaTime, SteeringAngle); }

	// 3) 帧内子步：轮胎力与 1D 动力流按子步耦合，
	//    将帧级刚度换算为子步刚度，消除轮速与轮胎力显式耦合的数值发散。
	for (FCirrovaWheelRuntime& Wheel : Wheels) { Wheel.AccumulatedForce = FVector::ZeroVector; }
	const int32 Substeps = FMath::Clamp(
		FMath::CeilToInt(DeltaTime / MaxSubstepDeltaTime),
		1,
		MaxSubstepsPerTick
	);
	const float SubDeltaTime = DeltaTime / static_cast<float>(Substeps);
	for (int32 Step = 0; Step < Substeps; ++Step)
	{
		for (FCirrovaWheelRuntime& Wheel : Wheels) { SolveTire(Wheel, SubDeltaTime); }
		liminal_vehicle_step(VehicleHandle, SubDeltaTime);
		for (FCirrovaWheelRuntime& Wheel : Wheels)
		{
			liminal_vehicle_read_omega(VehicleHandle, Wheel.ShaftPort, &Wheel.Omega);
		}
	}

	// 4) 施加帧平均合力（N → kg·cm/s²）并汇总接地状态。
	UPrimitiveComponent* Chassis = ChassisPrimitive.Get();
	bool bGrounded = false;
	for (FCirrovaWheelRuntime& Wheel : Wheels)
	{
		bGrounded |= Wheel.bInContact;
		if (Wheel.bInContact)
		{
			const FVector Force = Wheel.AccumulatedForce * (100.0f / static_cast<float>(Substeps));
			if (!Force.IsNearlyZero()) { Chassis->AddForceAtLocation(Force, Wheel.ContactPoint); }
		}
	}
	if (bGrounded != bWasGrounded)
	{
		bWasGrounded = bGrounded;
		OnVehicleGroundedEvent.Broadcast(bGrounded);
	}
}

void UCirrovaVehicleComponent::SetThrottle(float Value) { ThrottleInput = Value; }

void UCirrovaVehicleComponent::SetSteering(float Value) { SteeringInput = Value; }

void UCirrovaVehicleComponent::SetBrake(float Value) { BrakeInput = Value; }

void UCirrovaVehicleComponent::SetGear(int32 Gear) { GearInput = Gear; }

float UCirrovaVehicleComponent::GetEngineRPM() const
{
	if (!VehicleHandle || EngineShaftPort == 0) { return 0.0f; }
	float Omega = 0.0f;
	if (liminal_vehicle_read_omega(VehicleHandle, EngineShaftPort, &Omega) != ELiminalStatus::Ok) { return 0.0f; }
	return Omega * 60.0f / static_cast<float>(2.0 * PI);
}

float UCirrovaVehicleComponent::GetWheelAngularVelocity(int32 WheelIndex) const
{
	return Wheels.IsValidIndex(WheelIndex) ? Wheels[WheelIndex].Omega : 0.0f;
}

bool UCirrovaVehicleComponent::IsGrounded() const { return bWasGrounded; }

void UCirrovaVehicleComponent::BuildTopology()
{
	VehicleHandle = liminal_vehicle_new();
	if (!VehicleHandle)
	{
		SetComponentTickEnabled(false);
		return;
	}

	const int32 WheelCount = WheelConfigs.Num();
	if (WheelCount == 0 || GearRatios.Num() == 0)
	{
		UE_LOG(LogTemp, Warning, TEXT("Cirrova 载具组件缺少车轮配置或传动比，组件已停用。"));
		SetComponentTickEnabled(false);
		return;
	}

	uint64 Cockpit = 0;
	uint64 Chassis = 0;
	uint64 Engine = 0;
	uint64 Gearbox = 0;
	uint64 Distributor = 0;
	uint64 SteeringSplitter = 0;
	uint64 BrakeSplitter = 0;

	liminal_vehicle_add_cockpit(VehicleHandle, &Cockpit);

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
	liminal_vehicle_add_chassis(VehicleHandle, Mounts.GetData(), Mounts.Num(), &Chassis);

	liminal_vehicle_add_engine(
		VehicleHandle,
		EngineMaxTorque,
		EngineMaxAngularVelocity,
		EngineInertia,
		&Engine
	);
	liminal_vehicle_add_gearbox(
		VehicleHandle,
		GearRatios.GetData(),
		static_cast<size_t>(GearRatios.Num()),
		GearboxInertia,
		&Gearbox
	);
	if (bUseDifferential)
	{
		liminal_vehicle_add_differential(
			VehicleHandle,
			static_cast<size_t>(WheelCount),
			DifferentialCoupling,
			DifferentialMaxTorque,
			AxleInertia,
			&Distributor
		);
	}
	else
	{
		liminal_vehicle_add_axle(
			VehicleHandle,
			static_cast<size_t>(WheelCount),
			AxleInertia,
			&Distributor
		);
	}
	liminal_vehicle_add_splitter(VehicleHandle, static_cast<size_t>(WheelCount), &SteeringSplitter);
	liminal_vehicle_add_splitter(VehicleHandle, static_cast<size_t>(WheelCount), &BrakeSplitter);

	// 驾驶舱信号：油门/挡位直达，转向与制动经分发器到各轮。
	ThrottleSignalPort = PortOf(Cockpit, 0);
	SteeringSignalPort = PortOf(Cockpit, 1);
	GearSignalPort = PortOf(Cockpit, 2);
	BrakeSignalPort = PortOf(Cockpit, 3);
	EngineShaftPort = PortOf(Engine, 1);
	Connect(ThrottleSignalPort, PortOf(Engine, 0));
	Connect(GearSignalPort, PortOf(Gearbox, 0));
	Connect(SteeringSignalPort, PortOf(SteeringSplitter, 0));
	Connect(BrakeSignalPort, PortOf(BrakeSplitter, 0));

	// 动力链：发动机 → 变速箱 → 轮轴/差速器。
	Connect(PortOf(Engine, 1), PortOf(Gearbox, 1));
	Connect(PortOf(Gearbox, 2), PortOf(Distributor, 0));

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
			VehicleHandle,
			Config.SuspensionStiffness,
			Config.SuspensionDamping,
			Config.SuspensionRestLength,
			Config.SuspensionMinLength,
			Config.SuspensionMaxLength,
			&Suspension
		);
		liminal_vehicle_add_knuckle(VehicleHandle, MaxBrakeTorque, &Knuckle);
		liminal_vehicle_add_wheel(
			VehicleHandle,
			WheelInertia,
			WheelFrictionTorque,
			Config.Radius * 0.01f,
			&Wheel
		);

		Connect(PortOf(Chassis, Index), PortOf(Suspension, 0));
		Connect(PortOf(Suspension, 1), PortOf(Knuckle, 0));
		Connect(PortOf(Knuckle, 1), PortOf(Wheel, 1));

		Connect(PortOf(Distributor, static_cast<uint32>(Index) + 1), PortOf(Wheel, 0));
		Connect(PortOf(Knuckle, 4), PortOf(Wheel, 0));

		// 不可转向的车轮不接转向信号，端口保持零值。
		if (Config.bSteerable)
		{
			Connect(PortOf(SteeringSplitter, static_cast<uint32>(Index) + 1), PortOf(Knuckle, 2));
		}
		Connect(PortOf(BrakeSplitter, static_cast<uint32>(Index) + 1), PortOf(Knuckle, 3));

		SuspensionNodes.Add(Suspension);
		KnuckleNodes.Add(Knuckle);
		WheelNodes.Add(Wheel);
	}

	liminal_vehicle_recompile(VehicleHandle);
	ReadSpatialLinks(SuspensionNodes, KnuckleNodes, WheelNodes);
}

void UCirrovaVehicleComponent::ReadSpatialLinks(
	const TArray<uint64>& SuspensionNodes,
	const TArray<uint64>& KnuckleNodes,
	const TArray<uint64>& WheelNodes
)
{
	Wheels.SetNum(WheelNodes.Num());
	TArray<FTransform> RestPoses;
	RestPoses.Init(FTransform::Identity, WheelNodes.Num());

	size_t LinkCount = 0;
	if (liminal_vehicle_spatial_link_count(VehicleHandle, &LinkCount) != ELiminalStatus::Ok) { return; }

	for (size_t Index = 0; Index < LinkCount; ++Index)
	{
		FLiminalSpatialLink Link;
		if (liminal_vehicle_spatial_link(VehicleHandle, Index, &Link) != ELiminalStatus::Ok) { continue; }

		// 悬挂链接（子 = 悬挂节点）：弹簧参数。
		int32 WheelIndex = SuspensionNodes.IndexOfByKey(Link.Child);
		if (WheelIndex != INDEX_NONE)
		{
			FCirrovaWheelRuntime& Wheel = Wheels[WheelIndex];
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
			Wheels[WheelIndex].SteeringPort = Link.HasControl ? Link.ControlPort : 0;
			continue;
		}

		// 车轮链接（子 = 车轮节点）：轮心静止位姿。
		WheelIndex = WheelNodes.IndexOfByKey(Link.Child);
		if (WheelIndex != INDEX_NONE) { RestPoses[WheelIndex] = LiminalPoseToUE(Link.PoseInRoot); }
	}

	// 由轮心静止位姿与自由长度反推悬挂上挂点（沿静止位形的上方向）。
	for (int32 Index = 0; Index < WheelNodes.Num(); ++Index)
	{
		FCirrovaWheelRuntime& Wheel = Wheels[Index];
		Wheel.ShaftPort = PortOf(WheelNodes[Index], 0);
		Wheel.Radius = WheelConfigs[Index].Radius * 0.01f;
		Wheel.CurrentLength = Wheel.RestLength;

		const FTransform& RestPose = RestPoses[Index];
		Wheel.MountLocation =
			RestPose.GetTranslation() + RestPose.GetUnitAxis(EAxis::Z) * (Wheel.RestLength * 100.0f);
	}
}

void UCirrovaVehicleComponent::SolveContact(
	FCirrovaWheelRuntime& Wheel,
	float DeltaTime,
	float SteeringAngle
)
{
	UPrimitiveComponent* Chassis = ChassisPrimitive.Get();
	const FTransform ChassisToWorld = Chassis->GetComponentTransform();
	const FVector ChassisUp = ChassisToWorld.GetUnitAxis(EAxis::Z);
	const FVector MountWorld = ChassisToWorld.TransformPosition(Wheel.MountLocation);

	// 射线：挂点沿底盘向下，长度 = 行程上限 + 车轮半径。
	const float TraceLength = (Wheel.MaxLength + Wheel.Radius) * 100.0f;
	const FVector TraceEnd = MountWorld - ChassisUp * TraceLength;
	FCollisionQueryParams Params(SCENE_QUERY_STAT(CirrovaWheelTrace), false, GetOwner());
	FHitResult Hit;
	Wheel.bInContact = GetWorld()->LineTraceSingleByChannel(
		Hit,
		MountWorld,
		TraceEnd,
		WheelTraceChannel.GetValue(),
		Params
	);

	// 弹簧长度（m）：命中则按命中距离扣除半径，钳制在行程内。
	Wheel.CurrentLength = Wheel.MaxLength;
	if (Wheel.bInContact)
	{
		Wheel.CurrentLength = FMath::Clamp(
			Hit.Distance * 0.01f - Wheel.Radius,
			Wheel.MinLength,
			Wheel.MaxLength
		);
	}

	Wheel.ContactPoint = Wheel.bInContact ? Hit.ImpactPoint : TraceEnd;
	Wheel.ContactNormal = Wheel.bInContact ? Hit.ImpactNormal : ChassisUp;

	// 压缩量与压缩速度（SI），悬挂法向力只推不拉。
	// 首次求解跳过速度计算，避免出生位形的阶跃；
	// 速度钳制等效减振器泄压阀，消除落地帧的冲量尖峰。
	const float Compression = Wheel.RestLength - Wheel.CurrentLength;
	float CompressionVelocity = 0.0f;
	if (Wheel.bSolved)
	{
		CompressionVelocity = FMath::Clamp(
			(Compression - Wheel.PreviousCompression) / DeltaTime,
			-MaxCompressionVelocity,
			MaxCompressionVelocity
		);
	}
	Wheel.PreviousCompression = Compression;
	Wheel.bSolved = true;
	Wheel.SuspensionForce = Wheel.bInContact
		                        ? liminal_suspension_force(
			                        Wheel.SuspensionStiffness,
			                        Wheel.SuspensionDamping,
			                        Compression,
			                        CompressionVelocity
		                        )
		                        : 0.0f;

	// 接触坐标系：纵向 = 轮前向（含转向角）在接触平面上的投影，侧向 = 法线 × 纵向。
	const FQuat SteeringRotation(ChassisUp, SteeringAngle);
	FVector Forward = SteeringRotation.RotateVector(ChassisToWorld.GetUnitAxis(EAxis::X));
	Forward = (Forward - Wheel.ContactNormal * FVector::DotProduct(Forward, Wheel.ContactNormal))
		.GetSafeNormal();
	Wheel.LongitudinalAxis = Forward;
	Wheel.LateralAxis = FVector::CrossProduct(Wheel.ContactNormal, Forward).GetSafeNormal();

	// 接触点速度（cm/s），帧内缓存供各子步复用；地面假设为静态碰撞体。
	const FVector Linear = Chassis->GetPhysicsLinearVelocity();
	const FVector Angular = Chassis->GetPhysicsAngularVelocityInRadians();
	const FVector CenterOfMass = Chassis->GetCenterOfMass();
	Wheel.ContactVelocity =
		Linear + FVector::CrossProduct(Angular, Wheel.ContactPoint - CenterOfMass);

	if (bDrawDebug) { DrawWheelDebug(Wheel, MountWorld, ChassisUp); }
}

void UCirrovaVehicleComponent::SolveTire(FCirrovaWheelRuntime& Wheel, float SubDeltaTime)
{
	Wheel.TireForces = FLiminalTireForces{0.0f, 0.0f};
	if (!Wheel.bInContact || Wheel.SuspensionForce <= 0.0f) { return; }

	// 接触点相对地面的速度（m/s），地面假设为静态碰撞体。
	const float LongitudinalSpeed =
		FVector::DotProduct(Wheel.ContactVelocity, Wheel.LongitudinalAxis) * 0.01f;
	const float LateralSpeed =
		FVector::DotProduct(Wheel.ContactVelocity, Wheel.LateralAxis) * 0.01f;

	const float SlipAngle = FMath::Atan2(
		LateralSpeed,
		FMath::Max(FMath::Abs(LongitudinalSpeed), SlipReferenceSpeed)
	);

	// 冲量模式：纵向按本子步内对齐轮面与地面速度的冲量折算，侧向按侧偏刚度。
	Wheel.TireForces = liminal_tire_impulse_forces(
		TireFriction,
		TireLateralStiffness,
		Wheel.Omega,
		LongitudinalSpeed,
		Wheel.Radius,
		WheelInertia,
		SlipAngle,
		Wheel.SuspensionForce,
		SubDeltaTime
	);

	// 阻力侧零过冲：单轮阻力不得超过本步内使整车相对速度归零的分担量，
	// 防止静止/低速时轮胎力把车身反向推挤。
	const float StopForceShare = ChassisMass * FMath::Abs(LongitudinalSpeed)
		/ (FMath::Max(SubDeltaTime, KINDA_SMALL_NUMBER)
			* static_cast<float>(FMath::Max(Wheels.Num(), 1)));
	if (Wheel.TireForces.Longitudinal * LongitudinalSpeed < 0.0f
		&& FMath::Abs(Wheel.TireForces.Longitudinal) > StopForceShare)
	{
		Wheel.TireForces.Longitudinal =
			FMath::Sign(Wheel.TireForces.Longitudinal) * StopForceShare;
	}

	// 本子步合力累计（N），帧末取平均后施加。
	Wheel.AccumulatedForce += Wheel.ContactNormal * Wheel.SuspensionForce
		+ Wheel.LongitudinalAxis * Wheel.TireForces.Longitudinal
		+ Wheel.LateralAxis * Wheel.TireForces.Lateral;

	// 轮胎纵向力对车轮的反作用力矩回注 1D 域（N·m），由下一次子步消费。
	liminal_vehicle_write_torque(
		VehicleHandle,
		Wheel.ShaftPort,
		-Wheel.TireForces.Longitudinal * Wheel.Radius
	);
}

void UCirrovaVehicleComponent::DrawWheelDebug(
	const FCirrovaWheelRuntime& Wheel,
	const FVector& MountWorld,
	const FVector& ChassisUp
) const
{
	UWorld* World = GetWorld();
	if (!World) { return; }

	const float TraceLength = (Wheel.MaxLength + Wheel.Radius) * 100.0f;
	const FVector TraceEnd = MountWorld - ChassisUp * TraceLength;
	DrawDebugLine(
		World,
		MountWorld,
		TraceEnd,
		Wheel.bInContact ? FColor::Green : FColor::Red,
		false,
		-1.0f,
		0,
		1.0f
	);

	if (Wheel.bInContact)
	{
		DrawDebugPoint(World, Wheel.ContactPoint, 8.0f, FColor::Yellow, false, -1.0f);
		DrawDebugDirectionalArrow(
			World,
			Wheel.ContactPoint,
			Wheel.ContactPoint + Wheel.ContactNormal * Wheel.SuspensionForce * 0.01f,
			100.0f,
			FColor::Cyan,
			false,
			-1.0f
		);
		const FVector TireForce =
			Wheel.LongitudinalAxis * Wheel.TireForces.Longitudinal
			+ Wheel.LateralAxis * Wheel.TireForces.Lateral;
		DrawDebugDirectionalArrow(
			World,
			Wheel.ContactPoint,
			Wheel.ContactPoint + TireForce * 0.01f,
			100.0f,
			FColor::Magenta,
			false,
			-1.0f
		);
	}
}

uint64 UCirrovaVehicleComponent::PortOf(uint64 Node, uint32 Index) const
{
	uint64 Port = 0;
	if (liminal_vehicle_port(VehicleHandle, Node, Index, &Port) != ELiminalStatus::Ok)
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

bool UCirrovaVehicleComponent::Connect(uint64 SourcePort, uint64 TargetPort) const
{
	const ELiminalStatus Status =
		liminal_vehicle_connect(VehicleHandle, SourcePort, TargetPort, nullptr);
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
