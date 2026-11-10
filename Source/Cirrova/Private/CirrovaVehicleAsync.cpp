#include "CirrovaVehicleAsync.h"

#include "CirrovaWheelFrictionConstraints.h"
#include "Chaos/PBDRigidsEvolutionGBF.h"
#include "Chaos/PhysicalMaterials.h"
#include "Chaos/ShapeInstance.h"
#include "Chaos/PBDSuspensionConstraintData.h"
#include "Chaos/PBDSuspensionConstraints.h"
#include "Chaos/ParticleHandle.h"
#include "Chaos/PhysicsObjectCollisionInterface.h"
#include "Chaos/Sphere.h"
#include "HAL/PlatformTime.h"
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

	// 地面材质摩擦系数上限，避免异常物理材质产生非物理抓地。
	constexpr float MaxSurfaceFriction = 2.0f;

	// 车轮接触的最大坡面角（80°），避免把垂直墙面当作可支撑地面。
	constexpr float MaxContactSlopeCos = 0.173648f;

	/** 有限且非负的摩擦系数；无效时回退。 */
	float SanitizeFriction(const float Value, const float Fallback)
	{
		if (!FMath::IsFinite(Value) || Value < 0.0f) { return Fallback; }
		return FMath::Min(Value, MaxSurfaceFriction);
	}

	/** 从接触形状解析地面动/静摩擦系数；无材质时回退 1。 */
	void ResolveSurfaceFriction(
		const Chaos::FPerShapeData* Shape,
		float& OutFriction,
		float& OutStaticFriction
	)
	{
		OutFriction = 1.0f;
		OutStaticFriction = 1.0f;
		if (!Shape) { return; }

		const Chaos::FMaterialHandle& Handle = Shape->GetMaterial(0);
		const Chaos::FChaosPhysicsMaterial* Material = Handle.Get();
		if (!Material) { return; }

		OutFriction = SanitizeFriction(Material->Friction, 1.0f);
		OutStaticFriction = Material->StaticFriction > 0.0f
			? SanitizeFriction(Material->StaticFriction, OutFriction)
			: OutFriction;
		OutStaticFriction = FMath::Max(OutStaticFriction, OutFriction);
	}

	/**
	 * 构造单轮轮胎模型：轮胎参数与地面材质按几何平均合并。
	 *
	 * 静摩擦倍率同时包含地面静/动摩擦比，供适配层在低滑移速度下使用。
	 */
	FLiminalTireModel MakeTireModel(
		const FCirrovaVehicleRuntime& Runtime,
		const FCirrovaWheelRuntime& Wheel
	)
	{
		const float GroundFriction = SanitizeFriction(Wheel.GroundFriction, 1.0f);
		const float GroundStatic = FMath::Max(
			GroundFriction,
			SanitizeFriction(Wheel.GroundStaticFriction, GroundFriction)
		);
		const float SurfaceScale = FMath::Sqrt(GroundFriction);
		const float StaticRatio = GroundFriction > KINDA_SMALL_NUMBER
			? FMath::Sqrt(GroundStatic / GroundFriction)
			: 1.0f;

		FLiminalTireModel Model{};
		Model.FrictionCoefficient = Runtime.TireFriction * SurfaceScale;
		Model.LateralFrictionCoefficient = Runtime.TireFriction
			* FMath::Clamp(Runtime.TireLateralFrictionScale, 0.0f, 2.0f)
			* SurfaceScale;
		Model.LateralStiffness = Runtime.TireLateralStiffness;
		Model.LoadSensitivity = Runtime.TireLoadSensitivity;
		Model.ReferenceLoad = Runtime.TireReferenceLoad;
		Model.RollingResistance = Runtime.TireRollingResistance;
		Model.StaticFrictionScale = Runtime.TireStaticFrictionScale * StaticRatio;
		return Model;
	}

	/** 低速静摩擦权重：1 为静止，0 为完全滑动。 */
	float ComputeStictionWeight(const FCirrovaWheelRuntime& Wheel)
	{
		if (!Wheel.bInContact) { return 0.0f; }

		const float LongitudinalSlip = Wheel.SurfaceSpeed - Wheel.ContactLongitudinalSpeed;
		const float CombinedSlip = FMath::Sqrt(
			FMath::Square(LongitudinalSlip)
			+ FMath::Square(Wheel.ContactLateralSpeed)
		);
		if (!FMath::IsFinite(CombinedSlip)) { return 0.0f; }
		return 1.0f - FMath::Clamp(CombinedSlip / SlipReferenceSpeed, 0.0f, 1.0f);
	}

	/** 按静摩擦权重在动/静界之间线性插值。 */
	float EffectiveFrictionLimit(
		const float KineticLimit,
		const float StaticLimit,
		const float StictionWeight
	)
	{
		return FMath::Lerp(
			FMath::Max(KineticLimit, 0.0f),
			FMath::Max(StaticLimit, 0.0f),
			FMath::Clamp(StictionWeight, 0.0f, 1.0f)
		);
	}

	/** 更新纵向松弛后的轮面速度；制动或离地时重置为真实轮面速度。 */
	void UpdateLongitudinalRelaxation(
		const FCirrovaVehicleRuntime& Runtime,
		FCirrovaWheelRuntime& Wheel,
		const float DeltaTime,
		const bool bBraking
	)
	{
		const float SurfaceSpeed = Wheel.Omega * Wheel.Radius;
		Wheel.SurfaceSpeed = SurfaceSpeed;
		if (!FMath::IsFinite(SurfaceSpeed)) { Wheel.RelaxedSurfaceSpeed = 0.0f; return; }

		const float RelaxationLength = FMath::Max(Runtime.TireRelaxationLength, 0.0f);
		if (!Wheel.bInContact || bBraking || RelaxationLength <= KINDA_SMALL_NUMBER)
		{
			Wheel.RelaxedSurfaceSpeed = SurfaceSpeed;
			Wheel.bLongitudinalRelaxationInitialized = Wheel.bInContact;
			return;
		}
		if (!Wheel.bLongitudinalRelaxationInitialized)
		{
			Wheel.RelaxedSurfaceSpeed = SurfaceSpeed;
			Wheel.bLongitudinalRelaxationInitialized = true;
			return;
		}

		const float Speed = FMath::Max(
			FMath::Abs(Wheel.ContactLongitudinalSpeed),
			SlipReferenceSpeed
		);
		const float Blend = 1.0f - FMath::Exp(-Speed * DeltaTime / RelaxationLength);
		Wheel.RelaxedSurfaceSpeed += (SurfaceSpeed - Wheel.RelaxedSurfaceSpeed) * Blend;
		if (!FMath::IsFinite(Wheel.RelaxedSurfaceSpeed))
		{
			Wheel.RelaxedSurfaceSpeed = SurfaceSpeed;
		}
	}

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

	/** 接触法线是否可作为车轮支撑面；过滤垂直墙面。 */
	bool IsGroundContactNormal(const FVector& Normal, const FVector& ChassisUp)
	{
		const FVector UnitNormal = Normal.GetSafeNormal();
		return !UnitNormal.IsNearlyZero()
			&& FVector::DotProduct(UnitNormal, ChassisUp) >= MaxContactSlopeCos;
	}

	/** 单轮接触解：射线/球体/圆柱包络、接触坐标系、地面材质与相对速度。 */
	void SolveContact(
		FPhysicsObjectCollisionInterface_Internal& CollisionInterface,
		const TArray<Chaos::FConstPhysicsObjectHandle>& BlockingObjects,
		const FGeometryParticleHandle* ChassisParticle,
		const FTransform& ChassisToWorld,
		const FVector& ChassisUp,
		const FVector& LinearVelocity,
		const FVector& AngularVelocity,
		const FVector& CenterOfMass,
		const float SteeringAngle,
		const ECirrovaWheelProbeMode ProbeMode,
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

		FVector SteeringAxis =
			ChassisToWorld.TransformVectorNoScale(Wheel.SteeringAxis).GetSafeNormal();
		if (SteeringAxis.IsNearlyZero()) { SteeringAxis = ChassisUp; }
		const FQuat SteeringRotation(
			SteeringAxis,
			Wheel.bSteerable ? SteeringAngle : 0.0f
		);
		const FVector WheelSpinAxis =
			SteeringRotation.RotateVector(ChassisToWorld.GetUnitAxis(EAxis::Y)).GetSafeNormal();

		Wheel.bInContact = false;
		Wheel.GroundProxy = nullptr;
		Wheel.bGroundSupportsConstraint = false;
		Wheel.RaycastLength = Wheel.MaxLength;
		Wheel.GroundFriction = 1.0f;
		Wheel.GroundStaticFriction = 1.0f;
		FVector GroundVelocity = FVector::ZeroVector;
		if (!BlockingObjects.IsEmpty())
		{
			bool bHit = false;
			FVector HitPosition = FVector::ZeroVector;
			FVector HitNormal = ChassisUp;
			FGeometryParticleHandle* HitActor = nullptr;
			const Chaos::FPerShapeData* HitShape = nullptr;

			if (ProbeMode == ECirrovaWheelProbeMode::Cylinder)
			{
				// 圆柱包络：Chaos 的 ShapeSweep 不支持 FCylinder，沿轮宽布置多条射线，
				// 取沿悬挂轴最先命中的接触作为包络接触。
				const int32 EnvelopeSamples = 3;
				const float HalfWidthCm = FMath::Max(Wheel.WheelWidth, 0.01f) * 50.0f;
				float BestAxialDistance = TNumericLimits<float>::Max();
				for (int32 Sample = 0; Sample < EnvelopeSamples; ++Sample)
				{
					const float Alpha = EnvelopeSamples > 1
						? 2.0f * static_cast<float>(Sample) / static_cast<float>(EnvelopeSamples - 1) - 1.0f
						: 0.0f;
					const FVector SampleOrigin = MountWorld + WheelSpinAxis * (Alpha * HalfWidthCm);
					const FVector SampleEnd = SampleOrigin - SuspensionAxis * TraceLength;
					ChaosInterface::FPTRaycastHit Hit;
					if (!CollisionInterface.LineTrace(
						BlockingObjects,
						SampleOrigin,
						SampleEnd,
						false,
						Hit
					)) { continue; }
					if (!IsFiniteVector(Hit.WorldPosition)
						|| !IsFiniteVector(Hit.WorldNormal)
						|| !IsGroundContactNormal(Hit.WorldNormal, ChassisUp)) { continue; }

					const float AxialDistance = FVector::DotProduct(
						SampleOrigin - Hit.WorldPosition,
						SuspensionAxis
					);
					if (AxialDistance >= BestAxialDistance) { continue; }

					BestAxialDistance = AxialDistance;
					bHit = true;
					HitPosition = Hit.WorldPosition;
					HitNormal = Hit.WorldNormal;
					HitActor = Hit.Actor;
					HitShape = Hit.Shape;
				}
			}
			else if (ProbeMode == ECirrovaWheelProbeMode::Sphere)
			{
				// 球体扫掠：从轮心最高位置上方开始，覆盖整个悬挂行程。
				const float RadiusCm = Wheel.Radius * MeterToCentimeter;
				constexpr float ProbeMarginCm = 10.0f;
				const FVector SweepEnd = MountWorld
					- SuspensionAxis * (Wheel.MaxLength * MeterToCentimeter + ProbeMarginCm);
				const Chaos::FSphere Probe(Chaos::FVec3(0.0f, 0.0f, 0.0f), RadiusCm);
				Chaos::FSweepParameters SweepParameters;
				ChaosInterface::FPTSweepHit SweepHit;
				bHit = CollisionInterface.ShapeSweep(
					BlockingObjects,
					Probe,
					FTransform(FQuat::Identity, MountWorld),
					SweepEnd,
					SweepParameters,
					SweepHit
				);
				if (bHit)
				{
					// 初始重叠时位置/法线未初始化；此时回退射线。
					if (!IsFiniteVector(SweepHit.WorldPosition)
						|| !IsFiniteVector(SweepHit.WorldNormal)
						|| SweepHit.WorldNormal.IsNearlyZero()
						|| !IsGroundContactNormal(SweepHit.WorldNormal, ChassisUp))
					{
						bHit = false;
					}
					else
					{
						HitPosition = SweepHit.WorldPosition;
						HitNormal = SweepHit.WorldNormal;
						HitActor = SweepHit.Actor;
						HitShape = SweepHit.Shape;
					}
				}
			}

			// 扫掠未命中（深度压缩/陡峭边缘）时回退到零厚度射线。
			if (!bHit)
			{
				ChaosInterface::FPTRaycastHit Hit;
				bHit = CollisionInterface.LineTrace(
					BlockingObjects,
					MountWorld,
					TraceEnd,
					false,
					Hit
				);
				if (bHit)
				{
					if (!IsFiniteVector(Hit.WorldNormal)
						|| !IsGroundContactNormal(Hit.WorldNormal, ChassisUp))
					{
						bHit = false;
					}
					else
					{
						HitPosition = Hit.WorldPosition;
						HitNormal = Hit.WorldNormal;
						HitActor = Hit.Actor;
						HitShape = Hit.Shape;
					}
				}
			}

			// 自命中防护：忽略拥有者组件，并排除命中底盘自身粒子的情况。
			if (bHit && HitActor == ChassisParticle) { bHit = false; }

			Wheel.bInContact = bHit;
			if (Wheel.bInContact)
			{
				Wheel.ContactPoint = HitPosition;
				Wheel.ContactNormal = HitNormal.GetSafeNormal();
				if (Wheel.ContactNormal.IsNearlyZero()) { Wheel.ContactNormal = ChassisUp; }

				ResolveSurfaceFriction(
					HitShape,
					Wheel.GroundFriction,
					Wheel.GroundStaticFriction
				);

				// 轮心由接触点沿法线偏移半径，再投影到悬挂轴得到弹簧长度。
				Wheel.WheelCenter =
					Wheel.ContactPoint + Wheel.ContactNormal * (Wheel.Radius * MeterToCentimeter);
				Wheel.RaycastLength = FMath::Clamp(
					FVector::DotProduct(MountWorld - Wheel.WheelCenter, SuspensionAxis)
					* CentimeterToMeter,
					Wheel.MinLength,
					Wheel.MaxLength
				);
				Wheel.GroundProxy = HitActor ? HitActor->PhysicsProxy() : nullptr;
				Wheel.bGroundSupportsConstraint = Wheel.GroundProxy
					&& Wheel.GroundProxy->GetType() == EPhysicsProxyType::SingleParticleProxy;

				// 动态地面：接触点速度叠加地面刚体的平动与转动贡献。
				if (auto* GroundParticle = HitActor ? HitActor->CastToRigidParticle() : nullptr)
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

			// 弹簧在每次位置迭代中完整施加，按迭代数折算；刚度按行程曲线与渐进限位刷新，
			// 阻尼按压缩/回弹方向选择，保持有效刚度/阻尼与适配层模型一致。
			const float Compression = FMath::Max(Wheel.Compression, 0.0f);
			const float EffectiveStiffness = EvaluateSuspensionStiffness(Wheel, Compression);
			const float EffectiveDamping = SelectSuspensionDamping(Wheel, Wheel.CompressionVelocity);
			FPBDSuspensionSettings Settings = Container.GetSettings(Index);
			Settings.SpringStiffness = EffectiveStiffness * IterationScale;
			Settings.SpringDamping = FMath::Max(EffectiveDamping, 0.0f)
				* SolverDeltaTime * IterationScale;
			Container.SetSettings(Index, Settings);

			const FPBDSuspensionResults& Results = Container.GetResults(Index);
			if (Wheel.bInContact && Wheel.bGroundSupportsConstraint
				&& FMath::IsFinite(Results.Length))
			{
				// Results.Length 是相对静止轮心的有符号位移（cm）：压缩为负、伸展为正；
				// 还原为绝对弹簧长度 rest_length + 位移。
				Wheel.ConstraintLength =
					Wheel.RestLength + static_cast<float>(Results.Length) * CentimeterToMeter;
				Wheel.bConstraintLengthValid = true;
				Wheel.ConstraintPushOut = FVector(
					static_cast<float>(Results.NetPushOut.X),
					static_cast<float>(Results.NetPushOut.Y),
					static_cast<float>(Results.NetPushOut.Z)
				);
				Wheel.ConstraintHardStopPushOut = FVector(
					static_cast<float>(Results.HardStopNetPushOut.X),
					static_cast<float>(Results.HardStopNetPushOut.Y),
					static_cast<float>(Results.HardStopNetPushOut.Z)
				);
				Wheel.bConstraintPushOutValid = true;
			}
			else
			{
				// 约束未求解（离地或禁用）：退回射线估计，避免使用陈旧的弹簧长度。
				Wheel.ConstraintLength = 0.0f;
				Wheel.bConstraintLengthValid = false;
				Wheel.ConstraintPushOut = FVector::ZeroVector;
				Wheel.ConstraintHardStopPushOut = FVector::ZeroVector;
				Wheel.bConstraintPushOutValid = false;
			}
		}
	}

	/** 由压缩量与压缩速度得到悬挂法向力；原生约束模式下长度取约束求解值。 */
	void ResolveSuspensionLoad(FCirrovaWheelRuntime& Wheel, float DeltaTime)
	{
		const bool bUseConstraintLength =
			Wheel.bUseSuspensionConstraint
			&& Wheel.bGroundSupportsConstraint
			&& Wheel.bConstraintLengthValid
			&& Wheel.bInContact;
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

		float Force = 0.0f;
		if (bUseConstraintLength && Wheel.bConstraintPushOutValid && DeltaTime > KINDA_SMALL_NUMBER)
		{
			// NetPushOut 是位置求解累积的 kg·cm 修正量；等价法向力 = 修正量 / dt²，换算为 N 乘以 0.01。
			// 该值包含弹簧、阻尼、行程曲线与硬限位；预载由适配层显式叠加。
			const FVector PushOut = Wheel.ConstraintPushOut + Wheel.ConstraintHardStopPushOut;
			const float NormalPushOut = FVector::DotProduct(PushOut, Wheel.ContactNormal);
			Force = NormalPushOut / (DeltaTime * DeltaTime) * 0.01f + Wheel.SuspensionPreload;
			if (!FMath::IsFinite(Force) || Force < 0.0f) { Force = 0.0f; }
		}
		else if (Wheel.bInContact)
		{
			// 外力回退：与原生约束相同的曲线、渐进限位、分向阻尼与预载。
			const float StiffnessScale = EvaluateSuspensionStiffnessScale(
				Wheel.SuspensionCurve,
				Compression
			);
			const float EffectiveStiffness = Wheel.SuspensionStiffness * StiffnessScale;
			const float EffectiveDamping = SelectSuspensionDamping(Wheel, CompressionVelocity);
			Force = liminal_suspension_force(
				EffectiveStiffness,
				EffectiveDamping,
				Compression,
				CompressionVelocity
			);
			if (Wheel.BumpStopStiffness > 0.0f && Compression > Wheel.BumpStopStart)
			{
				Force += Wheel.BumpStopStiffness * (Compression - Wheel.BumpStopStart);
			}
			Force += Wheel.SuspensionPreload;
			if (!FMath::IsFinite(Force) || Force < 0.0f) { Force = 0.0f; }
		}

		Wheel.SuspensionForce = Force;
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

		// 原生悬挂约束已承担弹簧/阻尼/曲线/渐进限位；预载由适配层沿悬挂轴外部施加。
		const bool bConstraintSupportsLoad =
			Wheel.bUseSuspensionConstraint && Wheel.bGroundSupportsConstraint;
		const float AppliedSuspension = bConstraintSupportsLoad
			? Wheel.SuspensionPreload
			: Wheel.SuspensionForce;
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
		Wheel.StaticFrictionForceLimit = 0.0f;
		Wheel.StaticLateralForceLimit = 0.0f;
		Wheel.KineticFrictionForceLimit = 0.0f;
		Wheel.KineticLateralForceLimit = 0.0f;
		Wheel.FrictionUtilization = 0.0f;
		Wheel.SlipRatio = 0.0f;
		Wheel.SurfaceSpeed = Wheel.Omega * Wheel.Radius;
		Wheel.bTireConstraintApplied = false;
		if (!Wheel.bInContact || Wheel.NormalLoad <= 0.0f) { return; }

		const float LongitudinalSpeed = Wheel.ContactLongitudinalSpeed;
		const float LateralSpeed = Wheel.ContactLateralSpeed;
		const float SlipAngle = Wheel.RelaxedSlipAngleRad;
		Wheel.SlipRatio = (Wheel.SurfaceSpeed - LongitudinalSpeed) / FMath::Max(
			FMath::Abs(LongitudinalSpeed),
			SlipReferenceSpeed
		);

		const FLiminalTireModel Model = MakeTireModel(Runtime, Wheel);
		const float EffectiveInertia = Wheel.EffectiveInertia > 0.0f
			                               ? Wheel.EffectiveInertia
			                               : Runtime.WheelInertia;
		const float RelaxedOmega = Wheel.Radius > KINDA_SMALL_NUMBER
			? Wheel.RelaxedSurfaceSpeed / Wheel.Radius
			: Wheel.Omega;
		Wheel.TireForces = liminal_tire_impulse_forces(
			&Model,
			RelaxedOmega,
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

		// 遥测：摩擦椭圆界、侧向界与模型滚动阻力；纯函数求值，不参与本步力解。
		const FLiminalTireConstraint Constraint = liminal_tire_constraint(
			&Model,
			LongitudinalSpeed,
			SlipAngle,
			LateralSpeed,
			Wheel.NormalLoad
		);
		if (FMath::IsFinite(Constraint.LongitudinalLimit)
			&& FMath::IsFinite(Constraint.LateralLimit)
			&& FMath::IsFinite(Constraint.LateralForce)
			&& FMath::IsFinite(Constraint.RollingForce))
		{
			Wheel.FrictionForceLimit = FMath::Max(Constraint.LongitudinalLimit, 0.0f);
			Wheel.PendingFrictionForceLimit = Wheel.FrictionForceLimit;
			Wheel.LateralForceLimit = FMath::Max(Constraint.LateralLimit, 0.0f);
			Wheel.StaticFrictionForceLimit =
				FMath::Max(Constraint.StaticLongitudinalLimit, 0.0f);
			Wheel.StaticLateralForceLimit = FMath::Max(Constraint.StaticLateralLimit, 0.0f);
			Wheel.KineticFrictionForceLimit = FMath::Max(Constraint.LongitudinalLimit, 0.0f);
			Wheel.KineticLateralForceLimit = FMath::Max(Constraint.LateralLimit, 0.0f);
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
		const float NormalizedLongitudinal = Wheel.FrictionForceLimit > KINDA_SMALL_NUMBER
			? Wheel.TireFrictionLongitudinal / Wheel.FrictionForceLimit
			: 0.0f;
		const float NormalizedLateral = Wheel.LateralForceLimit > KINDA_SMALL_NUMBER
			? Wheel.TireFrictionLateral / Wheel.LateralForceLimit
			: 0.0f;
		Wheel.FrictionUtilization = FMath::Sqrt(
			FMath::Square(NormalizedLongitudinal)
			+ FMath::Square(NormalizedLateral)
		);

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
		Wheel.TireFrictionLongitudinal = 0.0f;
		Wheel.TireFrictionLateral = 0.0f;
		Wheel.WheelTorque = 0.0f;
		Wheel.FrictionForceLimit = Wheel.PendingFrictionForceLimit;
		Wheel.FrictionUtilization = 0.0f;
		Wheel.LongitudinalImpulse = 0.0f;
		Wheel.LateralImpulse = 0.0f;
		Wheel.ImpulseWindow = 0.0f;
		Wheel.ConstraintWheelInverseMass = 0.0f;
		if (Wheel.TireConstraintIndex == INDEX_NONE) { return; }

		Wheel.ConstraintWheelInverseMass = Container.IsConstraintEnabled(Wheel.TireConstraintIndex)
			? Container.GetWheelInverseMass(Wheel.TireConstraintIndex)
			: 0.0f;

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
		const float NormalizedLongitudinal = Wheel.FrictionForceLimit > KINDA_SMALL_NUMBER
			? LongitudinalForce / Wheel.FrictionForceLimit
			: 0.0f;
		const float NormalizedLateral = Wheel.LateralForceLimit > KINDA_SMALL_NUMBER
			? LateralForce / Wheel.LateralForceLimit
			: 0.0f;
		Wheel.FrictionUtilization = FMath::Sqrt(
			FMath::Square(NormalizedLongitudinal)
			+ FMath::Square(NormalizedLateral)
		);
	}

	/** 写入本物理步的轮胎约束输入；滚动阻力作为外部力累积。 */
	void WriteTireConstraint(
		FCirrovaWheelFrictionConstraints& Container,
		const FCirrovaVehicleRuntime& Runtime,
		FCirrovaWheelRuntime& Wheel,
		float BrakeInput
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
		Wheel.StaticFrictionForceLimit = 0.0f;
		Wheel.StaticLateralForceLimit = 0.0f;
		Wheel.KineticFrictionForceLimit = 0.0f;
		Wheel.KineticLateralForceLimit = 0.0f;

		if (Wheel.bInContact && Wheel.NormalLoad > 0.0f
			&& Wheel.TireConstraintIndex != INDEX_NONE
			&& IsFiniteVector(Wheel.ContactPoint)
			&& IsFiniteVector(Wheel.LongitudinalAxis)
			&& IsFiniteVector(Wheel.LateralAxis)
			&& IsFiniteVector(Wheel.GroundVelocity))
		{
			const float LongitudinalSpeed = Wheel.ContactLongitudinalSpeed;
			const float LateralSpeed = Wheel.ContactLateralSpeed;
			const float SlipAngle = Wheel.RelaxedSlipAngleRad;
			Wheel.SlipRatio = (Wheel.SurfaceSpeed - LongitudinalSpeed) / FMath::Max(
				FMath::Abs(LongitudinalSpeed),
				SlipReferenceSpeed
			);

			const FLiminalTireModel Model = MakeTireModel(Runtime, Wheel);
			const FLiminalTireConstraint Constraint = liminal_tire_constraint(
				&Model,
				LongitudinalSpeed,
				SlipAngle,
				LateralSpeed,
				Wheel.NormalLoad
			);

			const float WheelSurfaceSpeed = Wheel.RelaxedSurfaceSpeed * MeterToCentimeter;
			const bool bValid = FMath::IsFinite(Constraint.LongitudinalLimit)
				&& FMath::IsFinite(Constraint.LateralLimit)
				&& FMath::IsFinite(Constraint.StaticLongitudinalLimit)
				&& FMath::IsFinite(Constraint.StaticLateralLimit)
				&& FMath::IsFinite(Constraint.LateralForce)
				&& FMath::IsFinite(Constraint.RollingForce)
				&& FMath::IsFinite(WheelSurfaceSpeed)
				&& (Constraint.LongitudinalLimit > 0.0f || Constraint.LateralLimit > 0.0f);

			if (bValid)
			{
				// 低速静摩擦：静止附近提高附着，进入滑移后平滑回到动摩擦。
				const float StictionWeight = ComputeStictionWeight(Wheel);
				const float EffectiveLongitudinalLimit = EffectiveFrictionLimit(
					Constraint.LongitudinalLimit,
					Constraint.StaticLongitudinalLimit,
					StictionWeight
				);
				const float EffectiveLateralLimit = EffectiveFrictionLimit(
					Constraint.LateralLimit,
					Constraint.StaticLateralLimit,
					StictionWeight
				);

				Settings.bEnabled = true;
				Settings.ContactPoint = ToChaosVector(Wheel.ContactPoint);
				Settings.LongitudinalAxis = ToChaosVector(Wheel.LongitudinalAxis);
				Settings.LateralAxis = ToChaosVector(Wheel.LateralAxis);
				Settings.GroundVelocity = ToChaosVector(Wheel.GroundVelocity);
				Settings.WheelSurfaceSpeed = WheelSurfaceSpeed;
				Settings.WheelRadiusCm = Wheel.Radius * MeterToCentimeter;
				const float EffectiveInertia = Wheel.EffectiveInertia > KINDA_SMALL_NUMBER
					? Wheel.EffectiveInertia
					: Runtime.WheelInertia;
				Settings.WheelInertiaKgCm2 = FMath::Max(EffectiveInertia, 0.0f)
					* MeterToCentimeter * MeterToCentimeter;
				// 制动器扭矩覆盖轮胎扭矩上限时视为锁止；轮端自由度由一维域制动器约束，
				// 纵向冲量按纯底盘求解，避免制动工况下把车轮当成自由轮而低估制动力。
				const float BrakeTorqueCapacity = Runtime.MaxBrakeTorque
					* FMath::Clamp(BrakeInput, 0.0f, 1.0f);
				Settings.bWheelLocked = BrakeTorqueCapacity
					>= Constraint.LongitudinalLimit * Wheel.Radius;
				Settings.LongitudinalForceLimit = EffectiveLongitudinalLimit * MeterToCentimeter;
				Settings.LateralForceLimit = EffectiveLateralLimit * MeterToCentimeter;

				Wheel.PendingFrictionForceLimit = EffectiveLongitudinalLimit;
				Wheel.LateralForceLimit = EffectiveLateralLimit;
				Wheel.StaticFrictionForceLimit = FMath::Max(Constraint.StaticLongitudinalLimit, 0.0f);
				Wheel.StaticLateralForceLimit = FMath::Max(Constraint.StaticLateralLimit, 0.0f);
				Wheel.KineticFrictionForceLimit = FMath::Max(Constraint.LongitudinalLimit, 0.0f);
				Wheel.KineticLateralForceLimit = FMath::Max(Constraint.LateralLimit, 0.0f);
				Wheel.RollingResistance = Constraint.RollingForce;
				Wheel.TireForce = Wheel.LongitudinalAxis * Wheel.RollingResistance;
				Wheel.bTireConstraintApplied = true;
			}
		}

		Container.SetSettings(Wheel.TireConstraintIndex, Settings);
	}

	/** 物理步准备：接触解、悬挂载荷、轮胎约束输入与外力施加。 */
	void PrepareVehicle(
		float DeltaTime,
		const FCirrovaVehicleAsyncInput& Input,
		FCirrovaWheelFrictionConstraints* WheelConstraints,
		FCirrovaVehicleRuntime& Runtime,
		FPBDRigidParticleHandle& Particle
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
				Runtime.WheelProbeMode,
				Wheel
			);
		}

		// 侧向松弛：一阶过滤侧偏角，模拟胎体变形/松弛长度造成的瞬态滞后。
		const float RelaxationLength = FMath::Max(Runtime.TireRelaxationLength, 0.0f);
		for (FCirrovaWheelRuntime& Wheel : Runtime.Wheels)
		{
			if (!Wheel.bInContact)
			{
				Wheel.bRelaxationInitialized = false;
				continue;
			}
			if (RelaxationLength <= KINDA_SMALL_NUMBER || !Wheel.bRelaxationInitialized)
			{
				Wheel.RelaxedSlipAngleRad = Wheel.SlipAngleRad;
				Wheel.bRelaxationInitialized = true;
				continue;
			}

			const float Speed = FMath::Max(
				FMath::Abs(Wheel.ContactLongitudinalSpeed),
				SlipReferenceSpeed
			);
			const float Blend = 1.0f - FMath::Exp(-Speed * DeltaTime / RelaxationLength);
			Wheel.RelaxedSlipAngleRad +=
				(Wheel.SlipAngleRad - Wheel.RelaxedSlipAngleRad) * Blend;
		}

		// 纵向松弛：对轮面速度做一阶低通，保留稳态抓地同时给出瞬态滞后。
		for (FCirrovaWheelRuntime& Wheel : Runtime.Wheels)
		{
			UpdateLongitudinalRelaxation(Runtime, Wheel, DeltaTime, Input.Brake > 0.0f);
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
				// 弹簧沿真实悬挂轴作用，而不是地面法线；斜坡与台阶上轮胎摩擦才参与支撑。
				FVector SuspensionNormal =
					ChassisToWorld.TransformVectorNoScale(Wheel.SuspensionAxis).GetSafeNormal();
				if (SuspensionNormal.IsNearlyZero()) { SuspensionNormal = ChassisUp; }
				// 引擎的 SetSuspensionTarget 只接受 SingleParticleProxy 地面体；
				// 不兼容时禁用本步约束，由射线弹簧力接管。
				const bool bConstraintEnabled =
					Wheel.bInContact && Wheel.bGroundSupportsConstraint;
				Solver->SetSuspensionTarget(
					Wheel.SuspensionConstraint,
					TargetPosition,
					SuspensionNormal,
					bConstraintEnabled,
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

		// 空气动力学：阻力作用于质心，下压力沿底盘向下作用于压力中心。
		if (Runtime.bEnableAerodynamics)
		{
			const FVector VelocityMs = LinearVelocity * CentimeterToMeter;
			const float Speed = VelocityMs.Size();
			if (FMath::IsFinite(Speed) && Speed > 1.0f)
			{
				const float DynamicPressure =
					0.5f * Runtime.AeroAirDensity * Speed * Speed;
				const FVector DragForce = -VelocityMs.GetSafeNormal()
					* (DynamicPressure * Runtime.AeroDragCoefficient * Runtime.AeroFrontalArea);
				const FVector Downforce = -ChassisToWorld.GetUnitAxis(EAxis::Z)
					* (DynamicPressure * Runtime.AeroDownforceCoefficient * Runtime.AeroFrontalArea);
				const FVector CenterOfPressureWorld =
					ChassisToWorld.TransformPosition(Runtime.AeroCenterOfPressure);

				auto ApplyAeroForce = [&Particle, &CenterOfMassChaos](
					const FVector& Force,
					const FVector& Position
				)
				{
					if (Force.IsNearlyZero()) { return; }
					const FVec3 ForceChaos(Force * MeterToCentimeter);
					const FVec3 TorqueChaos = FVec3::CrossProduct(
						FVec3(Position) - CenterOfMassChaos,
						ForceChaos
					);
					Particle.AddForce(ForceChaos);
					Particle.AddTorque(TorqueChaos);
				};

				ApplyAeroForce(DragForce, FVector(CenterOfMassChaos));
				ApplyAeroForce(Downforce, CenterOfPressureWorld);
			}
		}

		const bool bConstraintCoupling =
			Runtime.bUseTireConstraint && WheelConstraints != nullptr && Solver != nullptr;
		int32 AppliedSubsteps = 1;

		if (bConstraintCoupling)
		{
			FCirrovaWheelFrictionConstraints& Container = *WheelConstraints;
			BindTireConstraints(Container, Runtime, Input.VehicleId, &Particle);

			// 以本步开始时的轮速写入约束输入；冲量与轮端反作用在 PostSolve 收尾。
			for (FCirrovaWheelRuntime& Wheel : Runtime.Wheels)
			{
				WriteTireConstraint(Container, Runtime, Wheel, Input.Brake);
			}

			Runtime.bSubstepBudgetExceeded = false;
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
			Runtime.bSubstepBudgetExceeded =
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

	}

	/** 物理步收尾：消费本步轮胎冲量、回注轮端力矩、推进 1D 域并产出快照。 */
	void FinishVehicle(
		float DeltaTime,
		FCirrovaWheelFrictionConstraints* WheelConstraints,
		FCirrovaVehicleRuntime& Runtime,
		FCirrovaVehicleOutput& Output
	)
	{
		// 约束形态在求解器步结束后消费同一物理步的冲量；外力回退已在准备阶段推进。
		if (Runtime.bTireConstraintCoupling && WheelConstraints)
		{
			FCirrovaWheelFrictionConstraints& Container = *WheelConstraints;

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

			liminal_vehicle_step(Runtime.Handle, DeltaTime);
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

		Output.bGrounded = false;
		Output.bTireConstraintCoupling = Runtime.bTireConstraintCoupling;
		Output.AppliedSubsteps = Runtime.AppliedSubsteps;
		Output.SolverDeltaTime = Runtime.SolverDeltaTime;
		Output.bSubstepBudgetExceeded = Runtime.bSubstepBudgetExceeded;

		// 以收尾后的轮速刷新轮面速度与滑移率，保证快照内部一致。
		for (FCirrovaWheelRuntime& Wheel : Runtime.Wheels)
		{
			Wheel.SurfaceSpeed = Wheel.Omega * Wheel.Radius;
			Wheel.SlipRatio = Wheel.bInContact
				? (Wheel.SurfaceSpeed - Wheel.ContactLongitudinalSpeed) / FMath::Max(
					FMath::Abs(Wheel.ContactLongitudinalSpeed),
					SlipReferenceSpeed
				)
				: 0.0f;
		}

		Output.Wheels.SetNum(Runtime.Wheels.Num());
		for (int32 Index = 0; Index < Runtime.Wheels.Num(); ++Index)
		{
			const FCirrovaWheelRuntime& Wheel = Runtime.Wheels[Index];
			FCirrovaWheelOutput& WheelOutput = Output.Wheels[Index];
			WheelOutput.bInContact = Wheel.bInContact;
			WheelOutput.Omega = Wheel.Omega;
			WheelOutput.SurfaceSpeed = Wheel.SurfaceSpeed;
			WheelOutput.EffectiveInertia = Wheel.EffectiveInertia;
			WheelOutput.ConstraintWheelInverseMass = Wheel.ConstraintWheelInverseMass;
			WheelOutput.SuspensionLength = Wheel.CurrentLength;
			WheelOutput.SuspensionForce = Wheel.SuspensionForce;
			WheelOutput.AntiRollForce = Wheel.AntiRollForce;
			WheelOutput.NormalLoad = Wheel.NormalLoad;
			WheelOutput.Compression = Wheel.Compression;
			WheelOutput.CompressionVelocity = Wheel.CompressionVelocity;
			WheelOutput.bCompressionVelocityClamped = Wheel.bCompressionVelocityClamped;
			WheelOutput.bNativeSuspensionActive =
				Wheel.bUseSuspensionConstraint && Wheel.bGroundSupportsConstraint;
			WheelOutput.bGroundProxyUnsupported = Wheel.bInContact
				&& Wheel.bUseSuspensionConstraint
				&& !Wheel.bGroundSupportsConstraint;
			WheelOutput.bUsingConstraintLength = Wheel.bUsingConstraintLength;
			WheelOutput.SuspensionConstraintLength = Wheel.ConstraintLength;
			WheelOutput.TireLongitudinalForce = Wheel.TireFrictionLongitudinal;
			WheelOutput.TireLateralForce = Wheel.TireFrictionLateral;
			WheelOutput.WheelTorque = Wheel.WheelTorque;
			WheelOutput.RollingResistance = Wheel.RollingResistance;
			WheelOutput.FrictionForceLimit = Wheel.FrictionForceLimit;
			WheelOutput.LateralForceLimit = Wheel.LateralForceLimit;
			WheelOutput.StaticFrictionForceLimit = Wheel.StaticFrictionForceLimit;
			WheelOutput.StaticLateralForceLimit = Wheel.StaticLateralForceLimit;
			WheelOutput.KineticFrictionForceLimit = Wheel.KineticFrictionForceLimit;
			WheelOutput.KineticLateralForceLimit = Wheel.KineticLateralForceLimit;
			WheelOutput.GroundFriction = Wheel.GroundFriction;
			WheelOutput.GroundStaticFriction = Wheel.GroundStaticFriction;
			WheelOutput.RelaxedSurfaceSpeed = Wheel.RelaxedSurfaceSpeed;
			WheelOutput.FrictionUtilization = Wheel.FrictionUtilization;
			WheelOutput.bTireConstraintApplied = Wheel.bTireConstraintApplied;
			WheelOutput.LongitudinalImpulse = Wheel.LongitudinalImpulse;
			WheelOutput.LateralImpulse = Wheel.LateralImpulse;
			WheelOutput.ImpulseWindow = Wheel.ImpulseWindow;
			WheelOutput.ContactLongitudinalSpeed = Wheel.ContactLongitudinalSpeed;
			WheelOutput.ContactLateralSpeed = Wheel.ContactLateralSpeed;
			WheelOutput.SlipRatio = Wheel.SlipRatio;
			WheelOutput.SlipAngleDegrees = FMath::RadiansToDegrees(Wheel.SlipAngleRad);
			WheelOutput.RelaxedSlipAngleDegrees = FMath::RadiansToDegrees(Wheel.RelaxedSlipAngleRad);
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
	PendingStepStartSeconds = FPlatformTime::Seconds();
	PendingPreSimulateMs = 0.0f;
	PendingVehicles.Reset();
	PendingWheelConstraints = nullptr;
	PendingDeltaTime = 0.0f;

	const FCirrovaAsyncInput* Input = GetConsumerInput_Internal();
	if (!Input) { return; }

	const float DeltaTime = GetDeltaTime_Internal();
	if (DeltaTime <= 0.0f) { return; }

	// 输出统一在 PostSolve 产出；此处清空避免求解未收尾时残留上一物理步数据。
	FCirrovaAsyncOutput& Output = GetProducerOutputData_Internal();
	Output.Reset();

	PendingWheelConstraints = Input->WheelConstraints;
	PendingDeltaTime = DeltaTime;
	PendingVehicles.Reserve(Input->Vehicles.Num());

	FCirrovaWheelFrictionConstraints* WheelConstraints = Input->WheelConstraints;
	if (WheelConstraints) { WheelConstraints->BeginFrame(); }

	for (const FCirrovaVehicleAsyncInput& VehicleInput : Input->Vehicles)
	{
		FCirrovaVehicleRuntime* Runtime = VehicleInput.Runtime;
		if (!Runtime || !Runtime->Handle) { continue; }

		FPBDRigidParticleHandle* Particle = ResolveParticle(VehicleInput.Proxy);
		if (!Particle) { continue; }

		if (WheelConstraints) { WheelConstraints->MarkOwnerActive(VehicleInput.VehicleId); }

		// 休眠粒子不响应力累加器：测试载具禁用休眠；驾驶员输入时也显式唤醒并清零计数。
		const bool bHasDriverInput = VehicleInput.Throttle > 0.0f
			|| VehicleInput.Brake > 0.0f
			|| FMath::Abs(VehicleInput.Steering) > 0.0f
			|| VehicleInput.Gear != 0;
		if (bHasDriverInput)
		{
			Particle->SetSleepCounter(0);
			Particle->SetSleeping(false);
		}
		if (!Runtime->bAllowPhysicsSleep)
		{
			Particle->SetSleepThresholdMultiplier(0.0f);
			Particle->SetSleepCounter(0);
			Particle->SetSleeping(false);
		}

		PrepareVehicle(
			DeltaTime,
			VehicleInput,
			WheelConstraints,
			*Runtime,
			*Particle
		);

		FPendingVehicle& Pending = PendingVehicles.AddDefaulted_GetRef();
		Pending.Runtime = Runtime;
		Pending.Proxy = VehicleInput.Proxy;
		Pending.VehicleId = VehicleInput.VehicleId;
	}

	PendingPreSimulateMs =
		static_cast<float>((FPlatformTime::Seconds() - PendingStepStartSeconds) * 1000.0);
}

void FCirrovaAsyncCallback::OnPostSolve_Internal()
{
	const double PostSolveStartSeconds = FPlatformTime::Seconds();

	FCirrovaAsyncOutput& Output = GetProducerOutputData_Internal();
	Output.Reset();
	Output.PreSimulateMs = PendingPreSimulateMs;
	Output.StepWallMs = static_cast<float>(
		(PostSolveStartSeconds - PendingStepStartSeconds) * 1000.0
	);
	Output.VehicleCount = PendingVehicles.Num();
	Output.Vehicles.Reserve(PendingVehicles.Num());

	for (const FPendingVehicle& Pending : PendingVehicles)
	{
		if (!Pending.Runtime || !Pending.Runtime->Handle) { continue; }
		if (!ResolveParticle(Pending.Proxy)) { continue; }

		FCirrovaVehicleOutput& VehicleOutput = Output.Vehicles.AddDefaulted_GetRef();
		VehicleOutput.VehicleId = Pending.VehicleId;
		FinishVehicle(
			PendingDeltaTime,
			PendingWheelConstraints,
			*Pending.Runtime,
			VehicleOutput
		);
	}

	// 未在本步出现的载具所拥有的约束槽位在此释放。
	if (PendingWheelConstraints) { PendingWheelConstraints->EndFrame(); }

	PendingVehicles.Reset();
	PendingWheelConstraints = nullptr;
	PendingDeltaTime = 0.0f;

	Output.PostSolveMs =
		static_cast<float>((FPlatformTime::Seconds() - PostSolveStartSeconds) * 1000.0);
}

void FCirrovaAsyncCallback::OnContactModification_Internal(FCollisionContactModifier& Modifier)
{
	// 扩展点：接触求解阶段的自定义修正（当前未使用）。
}
