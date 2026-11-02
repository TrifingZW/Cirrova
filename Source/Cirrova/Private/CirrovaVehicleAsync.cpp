#include "CirrovaVehicleAsync.h"

#include "Chaos/ParticleHandle.h"
#include "Chaos/PhysicsObjectCollisionInterface.h"
#include "Chaos/PhysicsObjectInternalInterface.h"
#include "Chaos/Particle/ParticleUtilities.h"
// FPhysicsObject 的完整定义位于 Chaos 私有头；物理对象查询需要其成员访问。
#include "CollisionShape.h"
#include "PBDRigidsSolver.h"
#include "PhysicsPublic.h"
#include "Engine/OverlapResult.h"
#include "Engine/World.h"
#include "Physics/GenericPhysicsInterface.h"
#include "PhysicsProxy/SingleParticlePhysicsProxy.h"
#include "Runtime/Experimental/Chaos/Private/Chaos/PhysicsObjectInternal.h"

using namespace Chaos;

namespace
{
	// 内部子步上限：轮胎力与轮速为显式耦合，子步长须低于数值刚度阈值。
	constexpr float MaxSubstepDeltaTime = 0.0025f;
	constexpr int32 MaxSubstepsPerTick = 32;

	// 阻尼压缩速度钳制（m/s），等效减振器泄压阀，防止落地帧产生过大冲量。
	constexpr float MaxCompressionVelocity = 2.0f;

	// 滑移归一化参考速度（m/s），低于该值按该值归一，避免低速除零。
	constexpr float SlipReferenceSpeed = 1.0f;

	constexpr float MeterToCentimeter = 100.0f;
	constexpr float CentimeterToMeter = 0.01f;

	/** 解析底盘粒子；非单粒子代理或已销毁时返回空。 */
	FPBDRigidParticleHandle* ResolveParticle(IPhysicsProxyBase* Proxy)
	{
		if (!Proxy || Proxy->GetType() != EPhysicsProxyType::SingleParticleProxy) { return nullptr; }
		auto* ParticleProxy = static_cast<FSingleParticlePhysicsProxy*>(Proxy);
		FGeometryParticleHandle* Handle = ParticleProxy->GetHandle_LowLevel();
		return Handle ? Handle->CastToRigidParticle() : nullptr;
	}

	/** 收集所有车轮射线扫掠盒内的可阻挡物理对象，供物理线程窄相查询使用。 */
	void GatherBlockingObjects(
		const UWorld* World,
		const FCollisionQueryParams& TraceParams,
		ECollisionChannel TraceChannel,
		const TArray<FCirrovaWheelRuntime>& Wheels,
		const FTransform& ChassisToWorld,
		TArray<FConstPhysicsObjectHandle>& OutObjects
	)
	{
		if (!World) { return; }

		const FVector Up = ChassisToWorld.GetUnitAxis(EAxis::Z);
		FBox TraceBox(ForceInit);
		for (const FCirrovaWheelRuntime& Wheel : Wheels)
		{
			const FVector Mount = ChassisToWorld.TransformPosition(Wheel.MountLocation);
			const FVector End =
				Mount - Up * ((Wheel.MaxLength + Wheel.Radius) * MeterToCentimeter);
			TraceBox += Mount;
			TraceBox += End;
		}
		if (!TraceBox.IsValid || TraceBox.GetSize().Length() <= KINDA_SMALL_NUMBER) { return; }

		TArray<FOverlapResult> Overlaps;
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

		for (const FOverlapResult& Overlap : Overlaps)
		{
			if (!Overlap.bBlockingHit || !Overlap.PhysicsObject) { continue; }
			if (!Overlap.PhysicsObject->GetParticle<EThreadContext::Internal>()) { continue; }
			OutObjects.Add(Overlap.PhysicsObject);
		}
	}

	/** 单轮接触解：射线、悬挂力、接触坐标系与接触点速度。 */
	void SolveContact(
		FPhysicsObjectCollisionInterface_Internal& CollisionInterface,
		const TArray<FConstPhysicsObjectHandle>& BlockingObjects,
		const FGeometryParticleHandle* ChassisParticle,
		const FTransform& ChassisToWorld,
		const FVector& ChassisUp,
		const FVector& LinearVelocity,
		const FVector& AngularVelocity,
		const FVector& CenterOfMass,
		float DeltaTime,
		float SteeringAngle,
		FCirrovaWheelRuntime& Wheel
	)
	{
		const FVector MountWorld = ChassisToWorld.TransformPosition(Wheel.MountLocation);
		const float TraceLength = (Wheel.MaxLength + Wheel.Radius) * MeterToCentimeter;
		const FVector TraceEnd = MountWorld - ChassisUp * TraceLength;

		Wheel.bInContact = false;
		Wheel.GroundProxy = nullptr;
		Wheel.CurrentLength = Wheel.MaxLength;
		FVector GroundVelocity = FVector::ZeroVector;
		if (!BlockingObjects.IsEmpty())
		{
			ChaosInterface::FPTRaycastHit Hit;
			Wheel.bInContact = CollisionInterface.LineTrace(
				BlockingObjects,
				MountWorld,
				TraceEnd,
				false,
				Hit
			);

			// 自命中防护：射线忽略拥有者组件，但仍排除命中底盘自身粒子的情况。
			if (Wheel.bInContact && Hit.Actor == ChassisParticle) { Wheel.bInContact = false; }

			if (Wheel.bInContact)
			{
				Wheel.ContactPoint = Hit.WorldPosition;
				Wheel.ContactNormal = Hit.WorldNormal.GetSafeNormal();
				if (Wheel.ContactNormal.IsNearlyZero()) { Wheel.ContactNormal = ChassisUp; }
				Wheel.CurrentLength = FMath::Clamp(
					Hit.Distance * CentimeterToMeter - Wheel.Radius,
					Wheel.MinLength,
					Wheel.MaxLength
				);
				Wheel.GroundProxy = Hit.Actor ? Hit.Actor->PhysicsProxy() : nullptr;

				// 动态地面：接触点速度叠加地面刚体的平动与转动贡献。
				if (auto* GroundParticle = Hit.Actor ? Hit.Actor->CastToRigidParticle() : nullptr)
				{
					const FVector GroundArm = Wheel.ContactPoint - FVector(GroundParticle->GetX());
					GroundVelocity =
						FVector(GroundParticle->GetV())
						+ FVector::CrossProduct(FVector(GroundParticle->GetW()), GroundArm);
					if (GroundVelocity.ContainsNaN()
						|| !FMath::IsFinite(GroundVelocity.X)
						|| !FMath::IsFinite(GroundVelocity.Y)
						|| !FMath::IsFinite(GroundVelocity.Z)) { GroundVelocity = FVector::ZeroVector; }
				}
			}
		}
		if (!Wheel.bInContact)
		{
			Wheel.ContactPoint = TraceEnd;
			Wheel.ContactNormal = ChassisUp;
		}

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
		Wheel.Compression = Compression;
		Wheel.bSolved = true;
		Wheel.SuspensionForce = Wheel.bInContact
			                        ? liminal_suspension_force(
				                        Wheel.SuspensionStiffness,
				                        Wheel.SuspensionDamping,
				                        Compression,
				                        CompressionVelocity
			                        )
			                        : 0.0f;

		const FQuat SteeringRotation(ChassisUp, Wheel.bSteerable ? SteeringAngle : 0.0f);
		FVector Forward =
			SteeringRotation.RotateVector(ChassisToWorld.GetUnitAxis(EAxis::X));
		Forward = (Forward - Wheel.ContactNormal * FVector::DotProduct(Forward, Wheel.ContactNormal))
			.GetSafeNormal();
		if (Forward.IsNearlyZero()) { Forward = ChassisToWorld.GetUnitAxis(EAxis::X); }
		Wheel.LongitudinalAxis = Forward;
		Wheel.LateralAxis = FVector::CrossProduct(Wheel.ContactNormal, Forward).GetSafeNormal();

		Wheel.ContactVelocity = LinearVelocity
			+ FVector::CrossProduct(AngularVelocity, Wheel.ContactPoint - CenterOfMass)
			- GroundVelocity;
	}

	/** 成对求解防倾杆：两侧均接地时产生等量反向的附加法向力。 */
	void SolveAntiRollBars(FCirrovaVehicleRuntime& Runtime)
	{
		for (FCirrovaWheelRuntime& Wheel : Runtime.Wheels) { Wheel.AntiRollForce = 0.0f; }

		for (const FCirrovaAntiRollBarRuntime& Bar : Runtime.AntiRollBars)
		{
			if (!Runtime.Wheels.IsValidIndex(Bar.WheelA) || !Runtime.Wheels.IsValidIndex(Bar.WheelB)) { continue; }

			FCirrovaWheelRuntime& WheelA = Runtime.Wheels[Bar.WheelA];
			FCirrovaWheelRuntime& WheelB = Runtime.Wheels[Bar.WheelB];
			if (!WheelA.bInContact || !WheelB.bInContact) { continue; }

			const float Force = liminal_anti_roll_force(
				Bar.Stiffness,
				WheelA.Compression,
				WheelB.Compression
			);
			if (!FMath::IsFinite(Force)) { continue; }

			WheelA.AntiRollForce += Force;
			WheelB.AntiRollForce -= Force;
		}
	}

	/** 单轮轮胎冲量解与车轮反作用力矩回注。 */
	void SolveTire(FCirrovaVehicleRuntime& Runtime, FCirrovaWheelRuntime& Wheel, float SubDeltaTime)
	{
		Wheel.TireForces = FLiminalTireForces{};
		if (!Wheel.bInContact) { return; }

		// 未启用原生悬挂约束时，弹簧力由适配层施加；
		// 防倾杆不属于弹簧约束，两种方式下都由适配层施加。
		if (!Wheel.bUseSuspensionConstraint) { Wheel.AccumulatedForce += Wheel.ContactNormal * Wheel.SuspensionForce; }
		if (Wheel.AntiRollForce != 0.0f) { Wheel.AccumulatedForce += Wheel.ContactNormal * Wheel.AntiRollForce; }

		// 法向载荷非正时轮胎无法传递力，但悬挂与防倾杆力已计入累加器。
		if (Wheel.NormalLoad <= 0.0f) { return; }

		const float LongitudinalSpeed =
			FVector::DotProduct(Wheel.ContactVelocity, Wheel.LongitudinalAxis) * CentimeterToMeter;
		const float LateralSpeed =
			FVector::DotProduct(Wheel.ContactVelocity, Wheel.LateralAxis) * CentimeterToMeter;

		const float SlipAngle = FMath::Atan2(
			LateralSpeed,
			FMath::Max(FMath::Abs(LongitudinalSpeed), SlipReferenceSpeed)
		);

		const FLiminalTireModel Model{
			Runtime.TireFriction,
			Runtime.TireLateralStiffness,
			Runtime.TireLoadSensitivity,
			Runtime.TireReferenceLoad,
			Runtime.TireRollingResistance,
		};
		Wheel.TireForces = liminal_tire_impulse_forces(
			&Model,
			Wheel.Omega,
			LongitudinalSpeed,
			Wheel.Radius,
			Runtime.WheelInertia,
			SlipAngle,
			Wheel.NormalLoad,
			SubDeltaTime
		);

		// 阻力侧零过冲：单轮阻力不得超过本步内使整车相对速度归零的分担量。
		const float UnclampedLongitudinal = Wheel.TireForces.Longitudinal;
		const float StopForceShare = Runtime.ChassisMass * FMath::Abs(LongitudinalSpeed) / (
			FMath::Max(SubDeltaTime, KINDA_SMALL_NUMBER)
			* static_cast<float>(FMath::Max(Runtime.Wheels.Num(), 1))
		);
		if (Wheel.TireForces.Longitudinal * LongitudinalSpeed < 0.0f
			&& FMath::Abs(Wheel.TireForces.Longitudinal) > StopForceShare)
		{
			Wheel.TireForces.Longitudinal =
				FMath::Sign(Wheel.TireForces.Longitudinal) * StopForceShare;
		}
		// 保持车轮反作用力矩与钳制后的施加力一致。
		Wheel.TireForces.WheelTorque -=
			(Wheel.TireForces.Longitudinal - UnclampedLongitudinal) * Wheel.Radius;

		Wheel.AccumulatedForce += Wheel.LongitudinalAxis * Wheel.TireForces.Longitudinal
			+ Wheel.LateralAxis * Wheel.TireForces.Lateral;

		liminal_vehicle_write_torque(
			Runtime.Handle,
			Wheel.ShaftPort,
			Wheel.TireForces.WheelTorque
		);
	}

	/** 推进单载具：接触解、子步动力流与力施加。 */
	void SimulateVehicle(
		const UWorld* World,
		float DeltaTime,
		const FCirrovaVehicleAsyncInput& Input,
		FCirrovaVehicleRuntime& Runtime,
		FPBDRigidParticleHandle& Particle,
		FCirrovaVehicleOutput& Output
	)
	{
		const FVec3 ParticlePosition = Particle.GetX();
		const FRotation3 ParticleRotation = Particle.GetR();
		const FVec3 ParticleLinearVelocity = Particle.GetV();
		const FVec3 ParticleAngularVelocity = Particle.GetW();

		const FQuat ChassisRotation(ParticleRotation);
		const FVector ChassisLocation(ParticlePosition);
		const FTransform ChassisToWorld(ChassisRotation, ChassisLocation);
		const FVector ChassisUp = ChassisToWorld.GetUnitAxis(EAxis::Z);
		const FVector LinearVelocity(ParticleLinearVelocity);
		const FVector AngularVelocity(ParticleAngularVelocity);

		const FVec3 CenterOfMassChaos = FParticleUtilitiesGT::GetCoMWorldPosition(&Particle);
		const FVector CenterOfMass(CenterOfMassChaos);

		const float SteeringAngle =
			FMath::Clamp(Input.Steering, -1.0f, 1.0f) * Runtime.MaxSteeringAngle;
		liminal_vehicle_write_signal(
			Runtime.Handle,
			Runtime.ThrottleSignalPort,
			FMath::Clamp(Input.Throttle, 0.0f, 1.0f)
		);
		liminal_vehicle_write_signal(Runtime.Handle, Runtime.SteeringSignalPort, SteeringAngle);
		liminal_vehicle_write_signal(
			Runtime.Handle,
			Runtime.GearSignalPort,
			static_cast<float>(Input.Gear)
		);
		liminal_vehicle_write_signal(
			Runtime.Handle,
			Runtime.BrakeSignalPort,
			FMath::Clamp(Input.Brake, 0.0f, 1.0f)
		);

		TArray<FConstPhysicsObjectHandle> BlockingObjects;
		GatherBlockingObjects(
			World,
			Input.TraceParams,
			Runtime.WheelTraceChannel,
			Runtime.Wheels,
			ChassisToWorld,
			BlockingObjects
		);

		FReadPhysicsObjectInterface_Internal ReadInterface = FPhysicsObjectInternalInterface::GetRead();
		FPhysicsObjectCollisionInterface_Internal CollisionInterface{ReadInterface};
		FPhysicsSolver* Solver = Input.Proxy
			                         ? Input.Proxy->GetSolver<FPhysicsSolver>()
			                         : nullptr;
		for (FCirrovaWheelRuntime& Wheel : Runtime.Wheels)
		{
			Wheel.AccumulatedForce = FVector::ZeroVector;
			SolveContact(
				CollisionInterface,
				BlockingObjects,
				&Particle,
				ChassisToWorld,
				ChassisUp,
				LinearVelocity,
				AngularVelocity,
				CenterOfMass,
				DeltaTime,
				SteeringAngle,
				Wheel
			);

			// 原生悬挂：目标点为轮心（地面接触点沿底盘上方向偏移半径），
			// 与约束的 Distance = 弹簧长度 语义对齐。
			if (Wheel.bUseSuspensionConstraint && Wheel.SuspensionConstraint && Solver)
			{
				const FVector TargetPosition = Wheel.bInContact
					                               ? Wheel.ContactPoint + ChassisUp * (Wheel.Radius * MeterToCentimeter)
					                               : Wheel.ContactPoint;
				Solver->SetSuspensionTarget(
					Wheel.SuspensionConstraint,
					TargetPosition,
					Wheel.ContactNormal,
					Wheel.bInContact,
					Wheel.GroundProxy
				);
			}
		}

		// 防倾杆成对求解后，重算各轮用于摩擦圆的总法向载荷。
		SolveAntiRollBars(Runtime);
		for (FCirrovaWheelRuntime& Wheel : Runtime.Wheels)
		{
			Wheel.NormalLoad = FMath::Max(Wheel.SuspensionForce + Wheel.AntiRollForce, 0.0f);
		}

		const int32 Substeps = FMath::Clamp(
			FMath::CeilToInt(DeltaTime / MaxSubstepDeltaTime),
			1,
			MaxSubstepsPerTick
		);
		const float SubDeltaTime = DeltaTime / static_cast<float>(Substeps);
		for (int32 Step = 0; Step < Substeps; ++Step)
		{
			for (FCirrovaWheelRuntime& Wheel : Runtime.Wheels) { SolveTire(Runtime, Wheel, SubDeltaTime); }
			liminal_vehicle_step(Runtime.Handle, SubDeltaTime);
			for (FCirrovaWheelRuntime& Wheel : Runtime.Wheels)
			{
				liminal_vehicle_read_omega(Runtime.Handle, Wheel.ShaftPort, &Wheel.Omega);
			}
		}

		// 力在求解器子步内直接写入粒子累加器；每个内部子步的合力取平均后施加一次。
		for (FCirrovaWheelRuntime& Wheel : Runtime.Wheels)
		{
			if (!Wheel.bInContact || Wheel.AccumulatedForce.IsNearlyZero()) { continue; }

			const FVector Force =
				Wheel.AccumulatedForce * (MeterToCentimeter / static_cast<float>(Substeps));
			FParticleUtilitiesGT::AddForceAtPositionWorld(
				&Particle,
				FVec3(Force),
				FVec3(Wheel.ContactPoint)
			);
			Wheel.AccumulatedForce = FVector::ZeroVector;
		}

		Output.bGrounded = false;
		Output.Wheels.SetNum(Runtime.Wheels.Num());
		for (int32 Index = 0; Index < Runtime.Wheels.Num(); ++Index)
		{
			const FCirrovaWheelRuntime& Wheel = Runtime.Wheels[Index];
			FCirrovaWheelOutput& WheelOutput = Output.Wheels[Index];
			WheelOutput.bInContact = Wheel.bInContact;
			WheelOutput.Omega = Wheel.Omega;
			WheelOutput.SuspensionLength = Wheel.CurrentLength;
			WheelOutput.SuspensionForce = Wheel.SuspensionForce;
			WheelOutput.AntiRollForce = Wheel.AntiRollForce;
			WheelOutput.NormalLoad = Wheel.NormalLoad;
			WheelOutput.TireForces = Wheel.TireForces;
			WheelOutput.ContactPoint = Wheel.ContactPoint;
			WheelOutput.ContactNormal = Wheel.ContactNormal;
			WheelOutput.LongitudinalAxis = Wheel.LongitudinalAxis;
			WheelOutput.LateralAxis = Wheel.LateralAxis;
			Output.bGrounded |= Wheel.bInContact;
		}

		float EngineOmega = 0.0f;
		if (Runtime.EngineShaftPort != 0
			&& liminal_vehicle_read_omega(Runtime.Handle, Runtime.EngineShaftPort, &EngineOmega)
			== ELiminalStatus::Ok) { Output.EngineOmega = EngineOmega; }
	}
}

void FCirrovaAsyncCallback::OnPreSimulate_Internal()
{
	const FCirrovaAsyncInput* Input = GetConsumerInput_Internal();
	UWorld* World = Input ? Input->World.Get() : nullptr;
	if (!Input || !World) { return; }

	const float DeltaTime = GetDeltaTime_Internal();
	if (DeltaTime <= 0.0f) { return; }

	FCirrovaAsyncOutput& Output = GetProducerOutputData_Internal();
	Output.Reset();
	Output.Vehicles.Reserve(Input->Vehicles.Num());

	for (const FCirrovaVehicleAsyncInput& VehicleInput : Input->Vehicles)
	{
		FCirrovaVehicleRuntime* Runtime = VehicleInput.Runtime;
		if (!Runtime || !Runtime->Handle) { continue; }

		FPBDRigidParticleHandle* Particle = ResolveParticle(VehicleInput.Proxy);
		if (!Particle) { continue; }

		FCirrovaVehicleOutput& VehicleOutput = Output.Vehicles.AddDefaulted_GetRef();
		VehicleOutput.VehicleId = VehicleInput.VehicleId;

		// 休眠粒子不响应力累加器：驾驶员有输入时显式唤醒。
		const bool bHasDriverInput = VehicleInput.Throttle > 0.0f
			|| VehicleInput.Brake > 0.0f
			|| FMath::Abs(VehicleInput.Steering) > 0.0f
			|| VehicleInput.Gear != 0;
		if (bHasDriverInput && Particle->IsSleeping()) { Particle->SetSleeping(false); }

		SimulateVehicle(
			World,
			DeltaTime,
			VehicleInput,
			*Runtime,
			*Particle,
			VehicleOutput
		);
	}
}

void FCirrovaAsyncCallback::OnContactModification_Internal(FCollisionContactModifier& Modifier)
{
	// 扩展点：接触求解阶段的自定义修正（当前未使用）。
}
