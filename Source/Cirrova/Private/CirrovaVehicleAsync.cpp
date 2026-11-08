#include "CirrovaVehicleAsync.h"

#include "CirrovaWheelFrictionConstraints.h"
#include "Chaos/PBDRigidsEvolutionGBF.h"
#include "Chaos/PBDSuspensionConstraintData.h"
#include "Chaos/PBDSuspensionConstraints.h"
#include "Chaos/ParticleHandle.h"
#include "Chaos/PhysicsObjectCollisionInterface.h"
#include "Chaos/PhysicsObjectInternalInterface.h"
#include "Chaos/Particle/ParticleUtilities.h"
// FPhysicsObject 的完整定义位于 Chaos 私有头；物理对象查询需要其成员访问。
#include "PBDRigidsSolver.h"
#include "PhysicsPublic.h"
#include "PhysicsProxy/SingleParticlePhysicsProxy.h"
#include "PhysicsProxy/SuspensionConstraintProxy.h"
#include "Runtime/Experimental/Chaos/Private/Chaos/PhysicsObjectInternal.h"

using namespace Chaos;

namespace
{
	// 外力回退模式的内部子步上限：轮胎力与轮速为显式耦合，子步长须低于数值刚度阈值。
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

	Chaos::FVec3 ToChaosVector(const FVector& Vector)
	{
		return Chaos::FVec3(Vector.X, Vector.Y, Vector.Z);
	}

	bool IsFiniteVector(const FVector& Vector)
	{
		return FMath::IsFinite(Vector.X) && FMath::IsFinite(Vector.Y) && FMath::IsFinite(Vector.Z);
	}

	/** 单轮接触解：射线、接触坐标系、地面速度与接触点相对速度。 */
	void SolveContact(
		FPhysicsObjectCollisionInterface_Internal& CollisionInterface,
		const TArray<Chaos::FConstPhysicsObjectHandle>& BlockingObjects,
		const FGeometryParticleHandle* ChassisParticle,
		const FTransform& ChassisToWorld,
		const FVector& ChassisUp,
		const FVector& LinearVelocity,
		const FVector& AngularVelocity,
		const FVector& CenterOfMass,
		float SteeringAngle,
		FCirrovaWheelRuntime& Wheel
	)
	{
		const FVector MountWorld = ChassisToWorld.TransformPosition(Wheel.MountLocation);
		FVector SuspensionAxis =
			ChassisToWorld.TransformVectorNoScale(Wheel.SuspensionAxis).GetSafeNormal();
		if (SuspensionAxis.IsNearlyZero()) { SuspensionAxis = ChassisUp; }

		// 沿悬挂轴下探；轴越倾斜，轮心沿射线方向的半径补偿越长。
		const float AxisVertical = FMath::Max(
			FMath::Abs(FVector::DotProduct(SuspensionAxis, ChassisUp)),
			0.25f
		);
		const float TraceLength =
			(Wheel.MaxLength + Wheel.Radius / AxisVertical) * MeterToCentimeter;
		const FVector TraceEnd = MountWorld - SuspensionAxis * TraceLength;

		Wheel.bInContact = false;
		Wheel.GroundProxy = nullptr;
		Wheel.RaycastLength = Wheel.MaxLength;
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

				// 轮心沿接触法线偏移半径；弹簧长度取挂点到轮心的轴向距离。
				Wheel.WheelCenter =
					Wheel.ContactPoint + Wheel.ContactNormal * (Wheel.Radius * MeterToCentimeter);
				Wheel.RaycastLength = FMath::Clamp(
					FVector::DotProduct(MountWorld - Wheel.WheelCenter, SuspensionAxis)
					* CentimeterToMeter,
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
					if (!IsFiniteVector(GroundVelocity)) { GroundVelocity = FVector::ZeroVector; }
				}
			}
		}
		if (!Wheel.bInContact)
		{
			Wheel.ContactPoint = TraceEnd;
			Wheel.ContactNormal = ChassisUp;
			Wheel.WheelCenter = MountWorld - SuspensionAxis * (Wheel.MaxLength * MeterToCentimeter);
			Wheel.RaycastLength = Wheel.MaxLength;
		}

		FVector SteeringAxis =
			ChassisToWorld.TransformVectorNoScale(Wheel.SteeringAxis).GetSafeNormal();
		if (SteeringAxis.IsNearlyZero()) { SteeringAxis = ChassisUp; }
		const FQuat SteeringRotation(
			SteeringAxis,
			Wheel.bSteerable ? SteeringAngle : 0.0f
		);
		FVector Forward =
			SteeringRotation.RotateVector(ChassisToWorld.GetUnitAxis(EAxis::X));
		Forward = (Forward - Wheel.ContactNormal * FVector::DotProduct(Forward, Wheel.ContactNormal))
			.GetSafeNormal();
		if (Forward.IsNearlyZero()) { Forward = ChassisToWorld.GetUnitAxis(EAxis::X); }
		Wheel.LongitudinalAxis = Forward;
		Wheel.LateralAxis = FVector::CrossProduct(Wheel.ContactNormal, Forward).GetSafeNormal();
		if (Wheel.LateralAxis.IsNearlyZero()) { Wheel.LateralAxis = ChassisToWorld.GetUnitAxis(EAxis::Y); }

		Wheel.GroundVelocity = GroundVelocity;
		Wheel.ContactVelocity = LinearVelocity
			+ FVector::CrossProduct(AngularVelocity, Wheel.ContactPoint - CenterOfMass)
			- GroundVelocity;

		Wheel.ContactLongitudinalSpeed =
			FVector::DotProduct(Wheel.ContactVelocity, Wheel.LongitudinalAxis) * CentimeterToMeter;
		Wheel.ContactLateralSpeed =
			FVector::DotProduct(Wheel.ContactVelocity, Wheel.LateralAxis) * CentimeterToMeter;
		Wheel.SlipAngleRad = FMath::Atan2(
			Wheel.ContactLateralSpeed,
			FMath::Max(FMath::Abs(Wheel.ContactLongitudinalSpeed), SlipReferenceSpeed)
		);

		// 非有限值归零，避免 NaN 沿遥测与轮胎模型传播。
		if (!FMath::IsFinite(Wheel.ContactLongitudinalSpeed)) { Wheel.ContactLongitudinalSpeed = 0.0f; }
		if (!FMath::IsFinite(Wheel.ContactLateralSpeed)) { Wheel.ContactLateralSpeed = 0.0f; }
		if (!FMath::IsFinite(Wheel.SlipAngleRad)) { Wheel.SlipAngleRad = 0.0f; }
	}

	/** 读回原生悬挂约束的求解长度，并按求解器步长与位置迭代数刷新刚度/阻尼。 */
	void RefreshSuspensionConstraints(
		FCirrovaVehicleRuntime& Runtime,
		FPhysicsSolver& Solver,
		float SolverDeltaTime
	)
	{
		auto* Evolution = Solver.GetEvolution();
		if (!Evolution) { return; }

		const int32 PositionIterations = FMath::Max(Evolution->GetNumPositionIterations(), 1);
		const float IterationScale = 1.0f / static_cast<float>(PositionIterations);
		FPBDSuspensionConstraints& Container = Evolution->GetSuspensionConstraints();

		for (FCirrovaWheelRuntime& Wheel : Runtime.Wheels)
		{
			if (!Wheel.bUseSuspensionConstraint || !Wheel.SuspensionConstraint) { continue; }

			FSuspensionConstraintPhysicsProxy* Proxy =
				Wheel.SuspensionConstraint->GetProxy<FSuspensionConstraintPhysicsProxy>();
			FPBDSuspensionConstraintHandle* Handle = Proxy ? Proxy->GetHandle() : nullptr;
			if (!Handle) { continue; }

			const int32 Index = Handle->GetConstraintIndex();
			if (Index == INDEX_NONE || Index >= Container.NumConstraints()) { continue; }

			// 弹簧在每次位置迭代中完整施加，按迭代数折算，保持有效刚度/阻尼与 liminal 一致。
			FPBDSuspensionSettings Settings = Container.GetSettings(Index);
			Settings.SpringStiffness = FMath::Max(Wheel.SuspensionStiffness, 0.0f) * IterationScale;
			Settings.SpringDamping = FMath::Max(Wheel.SuspensionDamping, 0.0f)
				* SolverDeltaTime * IterationScale;
			Container.SetSettings(Index, Settings);

			const FPBDSuspensionResults& Results = Container.GetResults(Index);
			if (Wheel.bInContact && FMath::IsFinite(Results.Length))
			{
				// Results.Length 是相对静止轮心的有符号位移（cm）：压缩为负、伸展为正；
				// 还原为绝对弹簧长度 rest_length + 位移。
				Wheel.ConstraintLength =
					Wheel.RestLength + static_cast<float>(Results.Length) * CentimeterToMeter;
				Wheel.bConstraintLengthValid = true;
			}
			else
			{
				// 约束未求解（离地或禁用）：退回射线估计，避免使用陈旧的弹簧长度。
				Wheel.ConstraintLength = 0.0f;
				Wheel.bConstraintLengthValid = false;
			}
		}
	}

	/** 由压缩量与压缩速度得到悬挂法向力；原生约束模式下长度取约束求解值。 */
	void ResolveSuspensionLoad(FCirrovaWheelRuntime& Wheel, float DeltaTime)
	{
		const bool bUseConstraintLength =
			Wheel.bUseSuspensionConstraint && Wheel.bConstraintLengthValid && Wheel.bInContact;
		const float Length = bUseConstraintLength
			                     ? FMath::Clamp(Wheel.ConstraintLength, Wheel.MinLength, Wheel.MaxLength)
			                     : Wheel.RaycastLength;
		Wheel.CurrentLength = Length;

		const float Compression = FMath::Max(Wheel.RestLength - Length, 0.0f);
		float CompressionVelocity = 0.0f;
		Wheel.bCompressionVelocityClamped = false;
		if (Wheel.bSolved && DeltaTime > KINDA_SMALL_NUMBER)
		{
			const float RawCompressionVelocity = (Compression - Wheel.PreviousCompression) / DeltaTime;
			Wheel.bCompressionVelocityClamped = FMath::Abs(RawCompressionVelocity) > MaxCompressionVelocity;
			CompressionVelocity = FMath::Clamp(
				RawCompressionVelocity,
				-MaxCompressionVelocity,
				MaxCompressionVelocity
			);
		}
		Wheel.CompressionVelocity = CompressionVelocity;
		Wheel.bUsingConstraintLength = bUseConstraintLength;
		Wheel.PreviousCompression = Compression;
		Wheel.Compression = Compression;
		Wheel.bSolved = true;

		const float Force = Wheel.bInContact
			                    ? liminal_suspension_force(
				                    Wheel.SuspensionStiffness,
				                    Wheel.SuspensionDamping,
				                    Compression,
				                    CompressionVelocity
			                    )
			                    : 0.0f;
		Wheel.SuspensionForce = FMath::IsFinite(Force) ? Force : 0.0f;
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

	/** 由悬挂与防倾杆支撑力得到法向载荷与待施加支撑力。 */
	void ResolveWheelSupport(
		const FTransform& ChassisToWorld,
		const FVector& ChassisUp,
		FCirrovaWheelRuntime& Wheel
	)
	{
		if (!Wheel.bInContact)
		{
			Wheel.NormalLoad = 0.0f;
			Wheel.SupportForce = FVector::ZeroVector;
			return;
		}

		FVector Axis = ChassisToWorld.TransformVectorNoScale(Wheel.SuspensionAxis).GetSafeNormal();
		if (Axis.IsNearlyZero()) { Axis = ChassisUp; }

		const float CalculatedSupport = Wheel.SuspensionForce + Wheel.AntiRollForce;
		Wheel.NormalLoad = FMath::Max(
			FVector::DotProduct(Axis * CalculatedSupport, Wheel.ContactNormal),
			0.0f
		);

		// 原生悬挂约束已承担弹簧力，适配层只施加防倾杆力。
		const float AppliedSuspension = Wheel.bUseSuspensionConstraint ? 0.0f : Wheel.SuspensionForce;
		Wheel.SupportForce = Axis * (AppliedSuspension + Wheel.AntiRollForce);
	}

	/** 外力回退的单轮轮胎冲量解与车轮反作用力矩回注。 */
	void SolveTire(
		FCirrovaVehicleRuntime& Runtime,
		FCirrovaWheelRuntime& Wheel,
		float SubDeltaTime,
		int32 GroundedWheelCount
	)
	{
		Wheel.TireForces = FLiminalTireForces{};
		Wheel.TireFrictionLongitudinal = 0.0f;
		Wheel.TireFrictionLateral = 0.0f;
		Wheel.WheelTorque = 0.0f;
		Wheel.RollingResistance = 0.0f;
		Wheel.FrictionForceLimit = 0.0f;
		Wheel.PendingFrictionForceLimit = 0.0f;
		Wheel.LateralForceLimit = 0.0f;
		Wheel.FrictionUtilization = 0.0f;
		Wheel.SlipRatio = 0.0f;
		Wheel.SurfaceSpeed = Wheel.Omega * Wheel.Radius;
		Wheel.bTireConstraintApplied = false;
		if (!Wheel.bInContact || Wheel.NormalLoad <= 0.0f) { return; }

		const float LongitudinalSpeed = Wheel.ContactLongitudinalSpeed;
		const float LateralSpeed = Wheel.ContactLateralSpeed;
		const float SlipAngle = Wheel.SlipAngleRad;
		Wheel.SlipRatio = (Wheel.SurfaceSpeed - LongitudinalSpeed) / FMath::Max(
			FMath::Abs(LongitudinalSpeed),
			SlipReferenceSpeed
		);

		const FLiminalTireModel Model{
			Runtime.TireFriction,
			Runtime.TireLateralStiffness,
			Runtime.TireLoadSensitivity,
			Runtime.TireReferenceLoad,
			Runtime.TireRollingResistance,
		};
		const float EffectiveInertia = Wheel.EffectiveInertia > 0.0f
			                               ? Wheel.EffectiveInertia
			                               : Runtime.WheelInertia;
		Wheel.TireForces = liminal_tire_impulse_forces(
			&Model,
			Wheel.Omega,
			LongitudinalSpeed,
			Wheel.Radius,
			EffectiveInertia,
			SlipAngle,
			LateralSpeed,
			Wheel.NormalLoad,
			SubDeltaTime
		);

		if (!FMath::IsFinite(Wheel.TireForces.Longitudinal)
			|| !FMath::IsFinite(Wheel.TireForces.Lateral)
			|| !FMath::IsFinite(Wheel.TireForces.WheelTorque))
		{
			Wheel.TireForces = FLiminalTireForces{};
			return;
		}

		// 遥测：摩擦圆界、侧向界与模型滚动阻力；纯函数求值，不参与本步力解。
		const FLiminalTireConstraint Constraint = liminal_tire_constraint(
			&Model,
			LongitudinalSpeed,
			SlipAngle,
			LateralSpeed,
			Wheel.NormalLoad
		);
		if (FMath::IsFinite(Constraint.FrictionForce)
			&& FMath::IsFinite(Constraint.LateralForce)
			&& FMath::IsFinite(Constraint.RollingForce))
		{
			Wheel.FrictionForceLimit = FMath::Max(Constraint.FrictionForce, 0.0f);
			Wheel.PendingFrictionForceLimit = Wheel.FrictionForceLimit;
			Wheel.LateralForceLimit = FMath::Abs(Constraint.LateralForce);
			Wheel.RollingResistance = Constraint.RollingForce;
		}

		// 阻力侧零过冲：单轮阻力不得超过本步内使整车相对速度归零的分担量。
		const float UnclampedLongitudinal = Wheel.TireForces.Longitudinal;
		const float StopForceShare = Runtime.ChassisMass * FMath::Abs(LongitudinalSpeed) / (
			FMath::Max(SubDeltaTime, KINDA_SMALL_NUMBER)
			* static_cast<float>(FMath::Max(GroundedWheelCount, 1))
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

		Wheel.WheelTorque = Wheel.TireForces.WheelTorque;
		Wheel.TireFrictionLongitudinal = Wheel.TireForces.Longitudinal - Wheel.RollingResistance;
		Wheel.TireFrictionLateral = Wheel.TireForces.Lateral;
		Wheel.FrictionUtilization = Wheel.FrictionForceLimit > KINDA_SMALL_NUMBER
			? FMath::Sqrt(
				FMath::Square(Wheel.TireFrictionLongitudinal)
				+ FMath::Square(Wheel.TireFrictionLateral)
			) / Wheel.FrictionForceLimit
			: 0.0f;

		Wheel.TireForce += Wheel.LongitudinalAxis * Wheel.TireForces.Longitudinal
			+ Wheel.LateralAxis * Wheel.TireForces.Lateral;

		liminal_vehicle_write_torque(
			Runtime.Handle,
			Wheel.ShaftPort,
			Wheel.TireForces.WheelTorque
		);
	}

	/** 为当前载具的车轮分配共享容器中的约束槽位。 */
	void BindTireConstraints(
		FCirrovaWheelFrictionConstraints& Container,
		FCirrovaVehicleRuntime& Runtime,
		int32 VehicleId,
		FGeometryParticleHandle* Particle
	)
	{
		for (FCirrovaWheelRuntime& Wheel : Runtime.Wheels)
		{
			if (!Container.IsBoundToOwner(Wheel.TireConstraintIndex, VehicleId))
			{
				Wheel.TireConstraintIndex = Container.AcquireConstraint(Particle, VehicleId);
			}
		}
	}

	/** 读取上一物理帧求解的轮胎冲量，换算为力与轮端反作用力矩。 */
	void ReadTireConstraint(
		FCirrovaWheelFrictionConstraints& Container,
		FCirrovaWheelRuntime& Wheel
	)
	{
		Wheel.TireForces = FLiminalTireForces{};
		Wheel.RollingResistance = 0.0f;
		Wheel.TireFrictionLongitudinal = 0.0f;
		Wheel.TireFrictionLateral = 0.0f;
		Wheel.WheelTorque = 0.0f;
		Wheel.FrictionUtilization = 0.0f;
		Wheel.LongitudinalImpulse = 0.0f;
		Wheel.LateralImpulse = 0.0f;
		Wheel.ImpulseWindow = 0.0f;
		Wheel.bTireConstraintApplied = false;
		if (Wheel.TireConstraintIndex == INDEX_NONE) { return; }

		const FCirrovaWheelFrictionResults Results =
			Container.ConsumeFrameResults(Wheel.TireConstraintIndex);
		if (Results.FrameDeltaTime <= KINDA_SMALL_NUMBER) { return; }

		// 冲量 kg·cm/s → N·s（÷100）。
		Wheel.LongitudinalImpulse = Results.LongitudinalImpulse * CentimeterToMeter;
		Wheel.LateralImpulse = Results.LateralImpulse * CentimeterToMeter;
		Wheel.ImpulseWindow = Results.FrameDeltaTime;

		const float LongitudinalForce =
			Results.LongitudinalImpulse / Results.FrameDeltaTime / MeterToCentimeter;
		const float LateralForce =
			Results.LateralImpulse / Results.FrameDeltaTime / MeterToCentimeter;
		if (!FMath::IsFinite(LongitudinalForce) || !FMath::IsFinite(LateralForce)) { return; }

		Wheel.TireForces.Longitudinal = LongitudinalForce;
		Wheel.TireForces.Lateral = LateralForce;
		Wheel.TireForces.WheelTorque = -LongitudinalForce * Wheel.Radius;

		Wheel.TireFrictionLongitudinal = LongitudinalForce;
		Wheel.TireFrictionLateral = LateralForce;
		Wheel.WheelTorque = Wheel.TireForces.WheelTorque;
		Wheel.FrictionForceLimit = Wheel.PendingFrictionForceLimit;
		Wheel.FrictionUtilization = Wheel.FrictionForceLimit > KINDA_SMALL_NUMBER
			? FMath::Sqrt(FMath::Square(LongitudinalForce) + FMath::Square(LateralForce))
				/ Wheel.FrictionForceLimit
			: 0.0f;
	}

	/** 写入本物理步的轮胎约束输入；滚动阻力作为外部力累积。 */
	void WriteTireConstraint(
		FCirrovaWheelFrictionConstraints& Container,
		const FCirrovaVehicleRuntime& Runtime,
		FCirrovaWheelRuntime& Wheel
	)
	{
		FCirrovaWheelFrictionSettings Settings;
		Wheel.RollingResistance = 0.0f;
		Wheel.TireForce = FVector::ZeroVector;
		Wheel.SurfaceSpeed = Wheel.Omega * Wheel.Radius;
		Wheel.SlipRatio = 0.0f;
		Wheel.bTireConstraintApplied = false;
		Wheel.PendingFrictionForceLimit = 0.0f;
		Wheel.LateralForceLimit = 0.0f;

		if (Wheel.bInContact && Wheel.NormalLoad > 0.0f
			&& Wheel.TireConstraintIndex != INDEX_NONE
			&& IsFiniteVector(Wheel.ContactPoint)
			&& IsFiniteVector(Wheel.LongitudinalAxis)
			&& IsFiniteVector(Wheel.LateralAxis)
			&& IsFiniteVector(Wheel.GroundVelocity))
		{
			const float LongitudinalSpeed = Wheel.ContactLongitudinalSpeed;
			const float LateralSpeed = Wheel.ContactLateralSpeed;
			const float SlipAngle = Wheel.SlipAngleRad;
			Wheel.SlipRatio = (Wheel.SurfaceSpeed - LongitudinalSpeed) / FMath::Max(
				FMath::Abs(LongitudinalSpeed),
				SlipReferenceSpeed
			);

			const FLiminalTireModel Model{
				Runtime.TireFriction,
				Runtime.TireLateralStiffness,
				Runtime.TireLoadSensitivity,
				Runtime.TireReferenceLoad,
				Runtime.TireRollingResistance,
			};
			const FLiminalTireConstraint Constraint = liminal_tire_constraint(
				&Model,
				LongitudinalSpeed,
				SlipAngle,
				LateralSpeed,
				Wheel.NormalLoad
			);

			const float WheelSurfaceSpeed = Wheel.Omega * Wheel.Radius * MeterToCentimeter;
			const bool bValid = FMath::IsFinite(Constraint.FrictionForce)
				&& FMath::IsFinite(Constraint.LateralForce)
				&& FMath::IsFinite(Constraint.RollingForce)
				&& FMath::IsFinite(WheelSurfaceSpeed)
				&& Constraint.FrictionForce > 0.0f;

			if (bValid)
			{
				Settings.bEnabled = true;
				Settings.ContactPoint = ToChaosVector(Wheel.ContactPoint);
				Settings.LongitudinalAxis = ToChaosVector(Wheel.LongitudinalAxis);
				Settings.LateralAxis = ToChaosVector(Wheel.LateralAxis);
				Settings.GroundVelocity = ToChaosVector(Wheel.GroundVelocity);
				Settings.WheelSurfaceSpeed = WheelSurfaceSpeed;
				Settings.FrictionForceLimit = Constraint.FrictionForce * MeterToCentimeter;
				Settings.LateralForceLimit = FMath::Abs(Constraint.LateralForce)
					* MeterToCentimeter;

				Wheel.PendingFrictionForceLimit = FMath::Max(Constraint.FrictionForce, 0.0f);
				Wheel.LateralForceLimit = FMath::Abs(Constraint.LateralForce);
				Wheel.RollingResistance = Constraint.RollingForce;
				Wheel.TireForce = Wheel.LongitudinalAxis * Wheel.RollingResistance;
				Wheel.bTireConstraintApplied = true;
			}
		}

		Container.SetSettings(Wheel.TireConstraintIndex, Settings);
	}

	/** 推进单载具：接触解、悬挂载荷、轮胎耦合与力施加。 */
	void SimulateVehicle(
		float DeltaTime,
		const FCirrovaVehicleAsyncInput& Input,
		FCirrovaWheelFrictionConstraints* WheelConstraints,
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

		const FVec3 CenterOfMassChaos =
			Particle.GetX() + Particle.GetR().RotateVector(Particle.CenterOfMass());
		const FVector CenterOfMass(CenterOfMassChaos);

		// 输入约定正值为左转；UE 绕 +Z 的正向旋转为右转，故取反。
		const float SteeringAngle =
			-FMath::Clamp(Input.Steering, -1.0f, 1.0f) * Runtime.MaxSteeringAngle;
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

		FReadPhysicsObjectInterface_Internal ReadInterface = FPhysicsObjectInternalInterface::GetRead();
		FPhysicsObjectCollisionInterface_Internal CollisionInterface{ReadInterface};
		FPhysicsSolver* Solver = Input.Proxy
			                         ? Input.Proxy->GetSolver<FPhysicsSolver>()
			                         : nullptr;

		for (FCirrovaWheelRuntime& Wheel : Runtime.Wheels)
		{
			Wheel.SupportForce = FVector::ZeroVector;
			Wheel.TireForce = FVector::ZeroVector;
			SolveContact(
				CollisionInterface,
				Input.BlockingObjects,
				&Particle,
				ChassisToWorld,
				ChassisUp,
				LinearVelocity,
				AngularVelocity,
				CenterOfMass,
				SteeringAngle,
				Wheel
			);
		}

		// 原生悬挂约束：以轮心为目标点、接触法线为法向，接地状态决定启用。
		if (Solver)
		{
			for (FCirrovaWheelRuntime& Wheel : Runtime.Wheels)
			{
				if (!Wheel.bUseSuspensionConstraint || !Wheel.SuspensionConstraint) { continue; }

				const FVector TargetPosition = Wheel.bInContact
					                               ? Wheel.WheelCenter
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

		// 原生悬挂约束：读回求解长度并保持阻尼项与求解器步长同步。
		float SolverDeltaTime = Solver ? static_cast<float>(Solver->GetAsyncDeltaTime()) : DeltaTime;
		if (SolverDeltaTime <= KINDA_SMALL_NUMBER) { SolverDeltaTime = DeltaTime; }
		SolverDeltaTime = FMath::Clamp(SolverDeltaTime, 1.0f / 1000.0f, 0.1f);
		if (Solver) { RefreshSuspensionConstraints(Runtime, *Solver, SolverDeltaTime); }

		int32 GroundedWheelCount = 0;
		for (FCirrovaWheelRuntime& Wheel : Runtime.Wheels)
		{
			ResolveSuspensionLoad(Wheel, DeltaTime);
			if (Wheel.bInContact) { ++GroundedWheelCount; }
		}

		SolveAntiRollBars(Runtime);
		for (FCirrovaWheelRuntime& Wheel : Runtime.Wheels)
		{
			ResolveWheelSupport(ChassisToWorld, ChassisUp, Wheel);
		}

		const bool bConstraintCoupling =
			Runtime.bUseTireConstraint && WheelConstraints != nullptr && Solver != nullptr;
		int32 AppliedSubsteps = 1;

		if (bConstraintCoupling)
		{
			FCirrovaWheelFrictionConstraints& Container = *WheelConstraints;
			BindTireConstraints(Container, Runtime, Input.VehicleId, &Particle);

			// 1) 上一物理帧的轮胎冲量换算为轮端反作用力矩。
			for (FCirrovaWheelRuntime& Wheel : Runtime.Wheels)
			{
				ReadTireConstraint(Container, Wheel);
				if (Wheel.TireConstraintIndex != INDEX_NONE)
				{
					liminal_vehicle_write_torque(
						Runtime.Handle,
						Wheel.ShaftPort,
						Wheel.TireForces.WheelTorque
					);
				}
			}

			// 2) 动力流按物理步单步推进；车轮不再需要内部子步。
			liminal_vehicle_step(Runtime.Handle, DeltaTime);
			for (FCirrovaWheelRuntime& Wheel : Runtime.Wheels)
			{
				liminal_vehicle_read_omega(Runtime.Handle, Wheel.ShaftPort, &Wheel.Omega);

				// 轮端等效惯量随转速连通组变化，逐步刷新供遥测与后续回注使用。
				float EffectiveInertia = 0.0f;
				if (liminal_vehicle_effective_inertia(
						Runtime.Handle,
						Wheel.ShaftPort,
						&EffectiveInertia
					) == ELiminalStatus::Ok
					&& EffectiveInertia > 0.0f) { Wheel.EffectiveInertia = EffectiveInertia; }
				else { Wheel.EffectiveInertia = Runtime.WheelInertia; }

				// 3) 以更新后的轮速写入本步约束输入。
				WriteTireConstraint(Container, Runtime, Wheel);
			}

			Output.bSubstepBudgetExceeded = false;
		}
		else
		{
			// 约束不可用的帧：丢弃未消费的冲量，避免模式恢复后出现过期的回注力矩。
			if (WheelConstraints)
			{
				for (FCirrovaWheelRuntime& Wheel : Runtime.Wheels)
				{
					if (Wheel.TireConstraintIndex != INDEX_NONE)
					{
						WheelConstraints->ConsumeFrameResults(Wheel.TireConstraintIndex);
					}
				}
			}

			// 外力回退：轮胎力与轮速显式交替，子步化压回稳定区。
			AppliedSubsteps = FMath::Clamp(
				FMath::CeilToInt(DeltaTime / MaxSubstepDeltaTime),
				1,
				MaxSubstepsPerTick
			);
			Output.bSubstepBudgetExceeded =
				DeltaTime > MaxSubstepDeltaTime * static_cast<float>(MaxSubstepsPerTick);
			const float SubDeltaTime = DeltaTime / static_cast<float>(AppliedSubsteps);
			for (int32 Step = 0; Step < AppliedSubsteps; ++Step)
			{
				for (FCirrovaWheelRuntime& Wheel : Runtime.Wheels)
				{
					SolveTire(Runtime, Wheel, SubDeltaTime, GroundedWheelCount);
				}
				liminal_vehicle_step(Runtime.Handle, SubDeltaTime);
				for (FCirrovaWheelRuntime& Wheel : Runtime.Wheels)
				{
					liminal_vehicle_read_omega(Runtime.Handle, Wheel.ShaftPort, &Wheel.Omega);
					float EffectiveInertia = 0.0f;
					if (liminal_vehicle_effective_inertia(
							Runtime.Handle,
							Wheel.ShaftPort,
							&EffectiveInertia
						) == ELiminalStatus::Ok
						&& EffectiveInertia > 0.0f) { Wheel.EffectiveInertia = EffectiveInertia; }
					else { Wheel.EffectiveInertia = Runtime.WheelInertia; }
				}
			}
		}

		Runtime.AppliedSubsteps = AppliedSubsteps;
		Runtime.SolverDeltaTime = SolverDeltaTime;
		Runtime.bTireConstraintCoupling = bConstraintCoupling;

		// 支撑力为整步常量；轮胎力在回退模式下为内部子步平均，约束模式下只剩滚动阻力。
		const float TireForceScale = bConstraintCoupling
			                             ? 1.0f
			                             : 1.0f / static_cast<float>(AppliedSubsteps);
		for (FCirrovaWheelRuntime& Wheel : Runtime.Wheels)
		{
			auto ApplyForceAtPosition = [&Particle, &CenterOfMassChaos](
				const FVector& Force,
				const FVector& Position
			)
			{
				if (Force.IsNearlyZero()) { return; }
				const FVec3 ForceChaos(Force);
				const FVec3 TorqueChaos = FVec3::CrossProduct(
					FVec3(Position) - CenterOfMassChaos,
					ForceChaos
				);
				Particle.AddForce(ForceChaos);
				Particle.AddTorque(TorqueChaos);
			};

			ApplyForceAtPosition(
				Wheel.SupportForce * MeterToCentimeter,
				Wheel.WheelCenter
			);
			ApplyForceAtPosition(
				Wheel.TireForce * (MeterToCentimeter * TireForceScale),
				Wheel.ContactPoint
			);
			Wheel.SupportForce = FVector::ZeroVector;
			Wheel.TireForce = FVector::ZeroVector;
		}

		Output.bGrounded = false;
		Output.bTireConstraintCoupling = Runtime.bTireConstraintCoupling;
		Output.AppliedSubsteps = Runtime.AppliedSubsteps;
		Output.SolverDeltaTime = Runtime.SolverDeltaTime;
		Output.Wheels.SetNum(Runtime.Wheels.Num());
		for (int32 Index = 0; Index < Runtime.Wheels.Num(); ++Index)
		{
			const FCirrovaWheelRuntime& Wheel = Runtime.Wheels[Index];
			FCirrovaWheelOutput& WheelOutput = Output.Wheels[Index];
			WheelOutput.bInContact = Wheel.bInContact;
			WheelOutput.Omega = Wheel.Omega;
			WheelOutput.SurfaceSpeed = Wheel.SurfaceSpeed;
			WheelOutput.EffectiveInertia = Wheel.EffectiveInertia;
			WheelOutput.SuspensionLength = Wheel.CurrentLength;
			WheelOutput.SuspensionForce = Wheel.SuspensionForce;
			WheelOutput.AntiRollForce = Wheel.AntiRollForce;
			WheelOutput.NormalLoad = Wheel.NormalLoad;
			WheelOutput.Compression = Wheel.Compression;
			WheelOutput.CompressionVelocity = Wheel.CompressionVelocity;
			WheelOutput.bCompressionVelocityClamped = Wheel.bCompressionVelocityClamped;
			WheelOutput.bNativeSuspensionActive = Wheel.bUseSuspensionConstraint;
			WheelOutput.bUsingConstraintLength = Wheel.bUsingConstraintLength;
			WheelOutput.SuspensionConstraintLength = Wheel.ConstraintLength;
			WheelOutput.TireLongitudinalForce = Wheel.TireFrictionLongitudinal;
			WheelOutput.TireLateralForce = Wheel.TireFrictionLateral;
			WheelOutput.WheelTorque = Wheel.WheelTorque;
			WheelOutput.RollingResistance = Wheel.RollingResistance;
			WheelOutput.FrictionForceLimit = Wheel.FrictionForceLimit;
			WheelOutput.LateralForceLimit = Wheel.LateralForceLimit;
			WheelOutput.FrictionUtilization = Wheel.FrictionUtilization;
			WheelOutput.bTireConstraintApplied = Wheel.bTireConstraintApplied;
			WheelOutput.LongitudinalImpulse = Wheel.LongitudinalImpulse;
			WheelOutput.LateralImpulse = Wheel.LateralImpulse;
			WheelOutput.ImpulseWindow = Wheel.ImpulseWindow;
			WheelOutput.ContactLongitudinalSpeed = Wheel.ContactLongitudinalSpeed;
			WheelOutput.ContactLateralSpeed = Wheel.ContactLateralSpeed;
			WheelOutput.SlipRatio = Wheel.SlipRatio;
			WheelOutput.SlipAngleDegrees = FMath::RadiansToDegrees(Wheel.SlipAngleRad);
			WheelOutput.SuspensionAxis = Wheel.SuspensionAxis;
			WheelOutput.ContactPoint = Wheel.ContactPoint;
			WheelOutput.ContactNormal = Wheel.ContactNormal;
			WheelOutput.LongitudinalAxis = Wheel.LongitudinalAxis;
			WheelOutput.LateralAxis = Wheel.LateralAxis;
			WheelOutput.ContactVelocity = Wheel.ContactVelocity;
			WheelOutput.GroundVelocity = Wheel.GroundVelocity;
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
	if (!Input) { return; }

	const float DeltaTime = GetDeltaTime_Internal();
	if (DeltaTime <= 0.0f) { return; }

	FCirrovaAsyncOutput& Output = GetProducerOutputData_Internal();
	Output.Reset();
	Output.Vehicles.Reserve(Input->Vehicles.Num());

	FCirrovaWheelFrictionConstraints* WheelConstraints = Input->WheelConstraints;
	if (WheelConstraints) { WheelConstraints->BeginFrame(); }

	for (const FCirrovaVehicleAsyncInput& VehicleInput : Input->Vehicles)
	{
		FCirrovaVehicleRuntime* Runtime = VehicleInput.Runtime;
		if (!Runtime || !Runtime->Handle) { continue; }

		FPBDRigidParticleHandle* Particle = ResolveParticle(VehicleInput.Proxy);
		if (!Particle) { continue; }

		if (WheelConstraints) { WheelConstraints->MarkOwnerActive(VehicleInput.VehicleId); }

		FCirrovaVehicleOutput& VehicleOutput = Output.Vehicles.AddDefaulted_GetRef();
		VehicleOutput.VehicleId = VehicleInput.VehicleId;

		// 休眠粒子不响应力累加器：驾驶员有输入时显式唤醒。
		const bool bHasDriverInput = VehicleInput.Throttle > 0.0f
			|| VehicleInput.Brake > 0.0f
			|| FMath::Abs(VehicleInput.Steering) > 0.0f
			|| VehicleInput.Gear != 0;
		if (bHasDriverInput && Particle->IsSleeping()) { Particle->SetSleeping(false); }

		SimulateVehicle(
			DeltaTime,
			VehicleInput,
			WheelConstraints,
			*Runtime,
			*Particle,
			VehicleOutput
		);
	}

	// 未在本步出现的载具所拥有的约束槽位在此释放。
	if (WheelConstraints) { WheelConstraints->EndFrame(); }
}

void FCirrovaAsyncCallback::OnContactModification_Internal(FCollisionContactModifier& Modifier)
{
	// 扩展点：接触求解阶段的自定义修正（当前未使用）。
}
