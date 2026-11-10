#include "CirrovaWheelFrictionConstraints.h"

#include "Chaos/Evolution/SolverBody.h"
#include "Chaos/Island/IslandManager.h"

namespace
{
	Chaos::FSolverVec3 ToSolverVector(const Chaos::FVec3& Vector)
	{
		return Chaos::FSolverVec3(
			static_cast<Chaos::FSolverReal>(Vector.X),
			static_cast<Chaos::FSolverReal>(Vector.Y),
			static_cast<Chaos::FSolverReal>(Vector.Z)
		);
	}

	/**
	 * 取得粒子在本组求解体容器中的实例。
	 *
	 * Chaos 的查找入口未从模块导出，插件无法直接链接；动态粒子把求解体索引保存在
	 * 粒子上，同组的悬挂约束先于本容器加入该粒子，因此可直接按索引取用。
	 */
	Chaos::FSolverBody* ResolveSolverBody(
		Chaos::FSolverBodyContainer& SolverBodyContainer,
		Chaos::FGeometryParticleHandle* Particle
	)
	{
		if (!Particle) { return nullptr; }

		const Chaos::FGenericParticleHandle Generic(Particle);
		const int32 Index = Generic->SolverBodyIndex();
		if (Index == INDEX_NONE || Index >= SolverBodyContainer.Num()) { return nullptr; }
		if (SolverBodyContainer.GetParticle(Index) != Particle) { return nullptr; }
		return &SolverBodyContainer.GetSolverBody(Index);
	}

	/**
	 * 单轴冲量求解：把接触点沿轴的速度驱动到目标速度，冲量按累计值钳制。
	 *
	 * 目标速度来自本步约束设置，冲量界在 GatherInput 中按步长折算；累计值跨
	 * 速度迭代保留，保证多迭代下的收敛与不越界。AdditionalInverseMass 为轮端
	 * 自由度消元项 R²/J（1/kg），侧向约束传 0。
	 */
	void SolveAxis(
		Chaos::FSolverBody& Body,
		const Chaos::FSolverVec3& ContactPoint,
		const Chaos::FSolverVec3& Axis,
		Chaos::FSolverReal TargetSpeed,
		Chaos::FSolverReal MaxImpulse,
		Chaos::FSolverReal& Accumulated,
		Chaos::FSolverReal AdditionalInverseMass = Chaos::FSolverReal(0)
	)
	{
		if (!FMath::IsFinite(TargetSpeed)
			|| !FMath::IsFinite(MaxImpulse)
			|| MaxImpulse < Chaos::FSolverReal(0))
		{
			return;
		}

		const Chaos::FSolverVec3 Arm = ContactPoint - ToSolverVector(Body.CorrectedP());
		const Chaos::FSolverVec3 ArmCrossAxis = Chaos::FSolverVec3::CrossProduct(Arm, Axis);
		const Chaos::FSolverReal InverseMass =
			Body.InvM()
			+ Chaos::FSolverVec3::DotProduct(ArmCrossAxis, Body.InvI() * ArmCrossAxis)
			+ AdditionalInverseMass;
		if (!FMath::IsFinite(InverseMass)
			|| InverseMass <= Chaos::FSolverReal(KINDA_SMALL_NUMBER)) { return; }

		const Chaos::FSolverVec3 PointVelocity =
			Body.V() + Chaos::FSolverVec3::CrossProduct(Body.W(), Arm);
		const Chaos::FSolverReal CurrentSpeed = Chaos::FSolverVec3::DotProduct(PointVelocity, Axis);
		if (!FMath::IsFinite(CurrentSpeed)) { return; }

		const Chaos::FSolverReal Desired = Accumulated + (TargetSpeed - CurrentSpeed) / InverseMass;
		const Chaos::FSolverReal Clamped = FMath::Clamp(Desired, -MaxImpulse, MaxImpulse);
		const Chaos::FSolverReal Applied = Clamped - Accumulated;
		Accumulated = Clamped;
		if (Applied == Chaos::FSolverReal(0)) { return; }

		const Chaos::FSolverVec3 Impulse = Axis * Applied;
		Body.ApplyLinearVelocityDelta(Impulse * Body.InvM());
		Body.ApplyAngularVelocityDelta(Body.InvI() * Chaos::FSolverVec3::CrossProduct(Arm, Impulse));
	}

	/**
	 * 摩擦椭圆单轴可用冲量：另一轴冲量占用椭圆后，本轴剩余的半轴长度。
	 *
	 * 椭圆方程 `(x/LimitX)² + (y/LimitY)² = 1`；任一轴界非正时退化为单轴约束。
	 */
	Chaos::FSolverReal EllipseAxisBound(
		const Chaos::FSolverReal AxisLimit,
		const Chaos::FSolverReal OtherLimit,
		const Chaos::FSolverReal OtherImpulse
	)
	{
		if (!FMath::IsFinite(AxisLimit) || AxisLimit <= Chaos::FSolverReal(0))
		{
			return Chaos::FSolverReal(0);
		}
		if (!FMath::IsFinite(OtherLimit) || OtherLimit <= Chaos::FSolverReal(0))
		{
			return AxisLimit;
		}
		if (!FMath::IsFinite(OtherImpulse)) { return AxisLimit; }

		const Chaos::FSolverReal Ratio = FMath::Clamp(
			FMath::Abs(OtherImpulse) / OtherLimit,
			Chaos::FSolverReal(0),
			Chaos::FSolverReal(1)
		);
		return AxisLimit * FMath::Sqrt(Chaos::FSolverReal(1) - Ratio * Ratio);
	}

	// 位置级静摩擦：仅消除速度级 PGS 在静止附近留下的残余蠕滑。
	// 速度超过阈值视为滚动/滑动，不修正；上一物理步任一轴触及椭圆界也视为滑动。
	constexpr float StaticFrictionSpeedThresholdCm = 15.0f;
	constexpr float StaticPositionRelaxation = 0.5f;
	constexpr float MaxStaticPositionCorrectionCm = 0.5f;

	// 侧向 warm start 保留系数：上一步累计侧向冲量乘此系数预施加，
	// 降低横向 PGS 残差；纵向不做 warm start（避免保留发动机/制动力）。
	constexpr float LateralWarmStartFactor = 0.5f;

	/** 在接触点沿轴施加线性冲量（kg·cm/s）。 */
	void ApplyImpulseAtPoint(
		Chaos::FSolverBody& Body,
		const Chaos::FSolverVec3& ContactPoint,
		const Chaos::FSolverVec3& Axis,
		const Chaos::FSolverReal Impulse
	)
	{
		if (Impulse == Chaos::FSolverReal(0)) { return; }

		const Chaos::FSolverVec3 Arm = ContactPoint - ToSolverVector(Body.CorrectedP());
		const Chaos::FSolverVec3 ImpulseVector = Axis * Impulse;
		Body.ApplyLinearVelocityDelta(ImpulseVector * Body.InvM());
		Body.ApplyAngularVelocityDelta(
			Body.InvI() * Chaos::FSolverVec3::CrossProduct(Arm, ImpulseVector)
		);
	}
}

const Chaos::FConstraintHandleTypeID& FCirrovaWheelFrictionConstraintHandle::StaticType()
{
	static const Chaos::FConstraintHandleTypeID STypeID(
		TEXT("FCirrovaWheelFrictionConstraintHandle"),
		&Chaos::FIndexedConstraintHandle::StaticType()
	);
	return STypeID;
}

Chaos::TVec2<Chaos::FGeometryParticleHandle*> FCirrovaWheelFrictionConstraintHandle::
	GetConstrainedParticles() const
{
	return ConcreteContainer()->GetConstrainedParticles(GetConstraintIndex());
}

FCirrovaWheelFrictionConstraints::FCirrovaWheelFrictionConstraints()
	: Base(FCirrovaWheelFrictionConstraintHandle::StaticType())
{
}

FCirrovaWheelFrictionConstraints::~FCirrovaWheelFrictionConstraints()
{
	for (int32 Index = 0; Index < Handles.Num(); ++Index)
	{
		FConstraintContainerHandle* Handle = Handles[Index];
		if (!Handle) { continue; }

		// 容器通常先于求解器销毁：摘除残留图边，避免句柄析构校验失败。
		if (Handle->IsInConstraintGraph() && IslandManager) { IslandManager->RemoveConstraint(Handle); }
		if (Slots.IsValidIndex(Index) && Slots[Index].Particle)
		{
			Slots[Index].Particle->RemoveConstraintHandle(Handle);
		}
		HandleAllocator.FreeHandle(Handle);
	}
}

void FCirrovaWheelFrictionConstraints::SetIslandManager(
	Chaos::Private::FPBDIslandManager* InIslandManager
)
{
	IslandManager = InIslandManager;
}

int32 FCirrovaWheelFrictionConstraints::AcquireConstraint(
	Chaos::FGeometryParticleHandle* Particle,
	int32 OwnerId
)
{
	int32 Index = INDEX_NONE;
	for (int32 Candidate = 0; Candidate < Slots.Num(); ++Candidate)
	{
		if (!Slots[Candidate].bBound)
		{
			Index = Candidate;
			break;
		}
	}
	if (Index == INDEX_NONE)
	{
		Index = Slots.AddDefaulted();
		Handles.Add(HandleAllocator.AllocHandle(this, Index));
	}

	FSlot& Slot = Slots[Index];
	Slot.Settings = FCirrovaWheelFrictionSettings();
	ResetFrameOutput(Slot);
	Slot.Particle = Particle;
	Slot.SolverBody = nullptr;
	Slot.OwnerId = OwnerId;
	Slot.LastSeenFrame = FrameCounter;
	Slot.bBound = true;
	Slot.bGraphDirty = true;

	if (Particle && Handles.IsValidIndex(Index)) { Particle->AddConstraintHandle(Handles[Index]); }
	return Index;
}

bool FCirrovaWheelFrictionConstraints::IsBoundToOwner(int32 ConstraintIndex, int32 OwnerId) const
{
	return Slots.IsValidIndex(ConstraintIndex)
		&& Slots[ConstraintIndex].bBound
		&& Slots[ConstraintIndex].OwnerId == OwnerId;
}

int32 FCirrovaWheelFrictionConstraints::NumActiveConstraints() const
{
	int32 Count = 0;
	for (const FSlot& Slot : Slots) { if (Slot.bBound) { ++Count; } }
	return Count;
}

float FCirrovaWheelFrictionConstraints::GetWheelInverseMass(const int32 ConstraintIndex) const
{
	return Slots.IsValidIndex(ConstraintIndex) ? Slots[ConstraintIndex].WheelInverseMass : 0.0f;
}

bool FCirrovaWheelFrictionConstraints::IsWheelLocked(const int32 ConstraintIndex) const
{
	if (!Slots.IsValidIndex(ConstraintIndex)) { return false; }

	const FSlot& Slot = Slots[ConstraintIndex];
	return Slot.Settings.bEnabled
		&& Slot.MaxBrakeAngularImpulse > 0.0f
		&& FMath::Abs(Slot.LocalOmega) < 0.5f;
}

void FCirrovaWheelFrictionConstraints::ReleaseConstraint(int32 ConstraintIndex)
{
	if (!Slots.IsValidIndex(ConstraintIndex)) { return; }

	FSlot& Slot = Slots[ConstraintIndex];
	if (Slot.Particle && Handles.IsValidIndex(ConstraintIndex))
	{
		Slot.Particle->RemoveConstraintHandle(Handles[ConstraintIndex]);
	}

	Slot.Settings = FCirrovaWheelFrictionSettings();
	ResetFrameOutput(Slot);
	Slot.Particle = nullptr;
	Slot.SolverBody = nullptr;
	Slot.OwnerId = INDEX_NONE;
	Slot.LastSeenFrame = -1;
	Slot.bBound = false;
	Slot.bGraphDirty = true;

	// 图边由下一次 AddConstraintsToGraph 依据 IsConstraintEnabled 移除。
}

void FCirrovaWheelFrictionConstraints::BeginFrame()
{
	++FrameCounter;
}

void FCirrovaWheelFrictionConstraints::MarkOwnerActive(int32 OwnerId)
{
	for (FSlot& Slot : Slots)
	{
		if (Slot.bBound && Slot.OwnerId == OwnerId) { Slot.LastSeenFrame = FrameCounter; }
	}
}

void FCirrovaWheelFrictionConstraints::EndFrame()
{
	for (int32 Index = 0; Index < Slots.Num(); ++Index)
	{
		const FSlot& Slot = Slots[Index];
		if (Slot.bBound && Slot.LastSeenFrame != FrameCounter) { ReleaseConstraint(Index); }
	}
}

void FCirrovaWheelFrictionConstraints::DetachFromSolver()
{
	// 调用方保证求解器已停止步进；此时摘除图边不会与建图/求解并发。
	for (int32 Index = 0; Index < Handles.Num(); ++Index)
	{
		FConstraintContainerHandle* Handle = Handles[Index];
		if (Handle && Handle->IsInConstraintGraph() && IslandManager)
		{
			IslandManager->RemoveConstraint(Handle);
		}

		if (Slots.IsValidIndex(Index))
		{
			FSlot& Slot = Slots[Index];
			if (Slot.Particle && Handle) { Slot.Particle->RemoveConstraintHandle(Handle); }
			Slot.Particle = nullptr;
			Slot.SolverBody = nullptr;
			Slot.Settings.bEnabled = false;
			Slot.bBound = false;
			Slot.OwnerId = INDEX_NONE;
			Slot.bGraphDirty = false;
			ResetFrameOutput(Slot);
		}
	}

	IslandManager = nullptr;
}

void FCirrovaWheelFrictionConstraints::SetSettings(
	int32 ConstraintIndex,
	const FCirrovaWheelFrictionSettings& Settings
)
{
	if (!Slots.IsValidIndex(ConstraintIndex)) { return; }

	FSlot& Slot = Slots[ConstraintIndex];
	const Chaos::FGenericParticleHandle Particle(Slot.Particle);
	const bool bHasParticle = Particle->Handle() != nullptr && !Particle->Disabled();
	const bool bWasEnabled = Slot.Settings.bEnabled;
	Slot.Settings = Settings;
	Slot.Settings.bEnabled = Settings.bEnabled && Slot.bBound && bHasParticle;
	if (Slot.Settings.bEnabled != bWasEnabled) { Slot.bGraphDirty = true; }
	if (!Slot.Settings.bEnabled)
	{
		// 接触中断：清除 warm start 与滑动门控状态，避免恢复时用过期的冲量。
		Slot.WarmStartLateralImpulse = 0.0f;
		Slot.LastAccumulatedLongitudinal = 0.0f;
		Slot.LastAccumulatedLateral = 0.0f;
		Slot.LastMaxLongitudinalImpulse = 0.0f;
		Slot.LastMaxLateralImpulse = 0.0f;
		ResetFrameOutput(Slot);
	}
}

FCirrovaWheelFrictionResults FCirrovaWheelFrictionConstraints::ConsumeFrameResults(
	int32 ConstraintIndex
)
{
	if (!Slots.IsValidIndex(ConstraintIndex)) { return FCirrovaWheelFrictionResults(); }

	FSlot& Slot = Slots[ConstraintIndex];
	const FCirrovaWheelFrictionResults Results = Slot.FrameResults;
	Slot.FrameResults = FCirrovaWheelFrictionResults();
	return Results;
}

void FCirrovaWheelFrictionConstraints::ResetFrameOutput(FSlot& Slot)
{
	Slot.FrameResults = FCirrovaWheelFrictionResults();
	Slot.MaxLongitudinalImpulse = 0.0f;
	Slot.MaxLateralImpulse = 0.0f;
	Slot.AccumulatedLongitudinal = 0.0f;
	Slot.AccumulatedLateral = 0.0f;
	Slot.AccumulatedTireAngularImpulse = 0.0f;
	Slot.AccumulatedBrakeAngularImpulse = 0.0f;
	Slot.MaxBrakeAngularImpulse = 0.0f;
	Slot.LocalOmega = 0.0f;
	Slot.WheelInverseMass = 0.0f;
	Slot.WarmStartLateralImpulse = 0.0f;
	Slot.LastAccumulatedLongitudinal = 0.0f;
	Slot.LastAccumulatedLateral = 0.0f;
	Slot.LastMaxLongitudinalImpulse = 0.0f;
	Slot.LastMaxLateralImpulse = 0.0f;
}

void FCirrovaWheelFrictionConstraints::SetConstraintEnabled(int32 ConstraintIndex, bool bEnabled)
{
	if (!Slots.IsValidIndex(ConstraintIndex)) { return; }

	FSlot& Slot = Slots[ConstraintIndex];
	const Chaos::FGenericParticleHandle Particle(Slot.Particle);
	const bool bHasParticle = Particle->Handle() != nullptr && !Particle->Disabled();
	Slot.Settings.bEnabled = bEnabled && Slot.bBound && bHasParticle;
	if (!Slot.Settings.bEnabled) { ResetFrameOutput(Slot); }
	Slot.bGraphDirty = true;
}

bool FCirrovaWheelFrictionConstraints::IsConstraintEnabled(int32 ConstraintIndex) const
{
	return Slots.IsValidIndex(ConstraintIndex) && Slots[ConstraintIndex].Settings.bEnabled;
}

Chaos::TVec2<Chaos::FGeometryParticleHandle*> FCirrovaWheelFrictionConstraints::
	GetConstrainedParticles(int32 ConstraintIndex) const
{
	if (!Slots.IsValidIndex(ConstraintIndex)) { return {nullptr, nullptr}; }
	return {Slots[ConstraintIndex].Particle, nullptr};
}

void FCirrovaWheelFrictionConstraints::DisconnectConstraints(
	const TSet<Chaos::FGeometryParticleHandle*>& RemovedParticles
)
{
	for (Chaos::FGeometryParticleHandle* RemovedParticle : RemovedParticles)
	{
		for (Chaos::FConstraintHandle* ConstraintHandle : RemovedParticle->ParticleConstraints())
		{
			auto* Handle = ConstraintHandle->As<FCirrovaWheelFrictionConstraintHandle>();
			if (!Handle) { continue; }

			const int32 Index = Handle->GetConstraintIndex();
			if (!Slots.IsValidIndex(Index)) { continue; }

			// 粒子正处于销毁过程：只在本容器状态中摘除，不再访问粒子。
			FSlot& Slot = Slots[Index];
			if (Slot.Particle == RemovedParticle) { Slot.Particle = nullptr; }
			Slot.Settings.bEnabled = false;
			Slot.bBound = false;
			Slot.OwnerId = INDEX_NONE;
			Slot.bGraphDirty = true;
			ResetFrameOutput(Slot);
		}
	}
}

void FCirrovaWheelFrictionConstraints::OnDisableParticle(
	Chaos::FGeometryParticleHandle* DisabledParticle
)
{
	if (!DisabledParticle) { return; }

	for (Chaos::FConstraintHandle* Handle : DisabledParticle->ParticleConstraints())
	{
		if (Handle && Handle->GetContainerId() == ContainerId && Handle->IsEnabled())
		{
			Handle->SetEnabled(false);
		}
	}
}

void FCirrovaWheelFrictionConstraints::OnEnableParticle(
	Chaos::FGeometryParticleHandle* EnabledParticle
)
{
	if (!EnabledParticle) { return; }

	for (Chaos::FConstraintHandle* Handle : EnabledParticle->ParticleConstraints())
	{
		if (Handle && Handle->GetContainerId() == ContainerId && !Handle->IsEnabled())
		{
			Handle->SetEnabled(true);
		}
	}
}

void FCirrovaWheelFrictionConstraints::AddConstraintsToGraph(
	Chaos::Private::FPBDIslandManager& InIslandManager
)
{
	for (int32 Index = 0; Index < Slots.Num(); ++Index)
	{
		FConstraintContainerHandle* Handle = Handles.IsValidIndex(Index) ? Handles[Index] : nullptr;
		if (!Handle) { continue; }

		bool bInGraph = Handle->IsInConstraintGraph();
		const bool bShouldBeInGraph = IsConstraintEnabled(Index);
		if (bInGraph && (Slots[Index].bGraphDirty || !bShouldBeInGraph))
		{
			InIslandManager.RemoveConstraint(Handle);
			bInGraph = false;
		}
		if (bShouldBeInGraph && !bInGraph)
		{
			InIslandManager.AddConstraint(ContainerId, Handle, Handle->GetConstrainedParticles());
		}
		Slots[Index].bGraphDirty = false;
	}
}

void FCirrovaWheelFrictionConstraints::AddBodies(
	const TArrayView<int32>& ConstraintIndices,
	Chaos::FSolverBodyContainer& SolverBodyContainer
)
{
	for (const int32 ConstraintIndex : ConstraintIndices)
	{
		AddBody(ConstraintIndex, SolverBodyContainer);
	}
}

void FCirrovaWheelFrictionConstraints::AddBodies(Chaos::FSolverBodyContainer& SolverBodyContainer)
{
	for (int32 ConstraintIndex = 0; ConstraintIndex < Slots.Num(); ++ConstraintIndex)
	{
		if (IsConstraintEnabled(ConstraintIndex)) { AddBody(ConstraintIndex, SolverBodyContainer); }
	}
}

void FCirrovaWheelFrictionConstraints::AddBody(
	int32 ConstraintIndex,
	Chaos::FSolverBodyContainer& SolverBodyContainer
)
{
	if (!Slots.IsValidIndex(ConstraintIndex)) { return; }

	FSlot& Slot = Slots[ConstraintIndex];
	Slot.SolverBody = (Slot.Settings.bEnabled && Slot.Particle)
		                  ? ResolveSolverBody(SolverBodyContainer, Slot.Particle)
		                  : nullptr;
}

void FCirrovaWheelFrictionConstraints::GatherInput(
	const TArrayView<int32>& ConstraintIndices,
	const Chaos::FReal Dt
)
{
	for (const int32 ConstraintIndex : ConstraintIndices) { GatherSingle(ConstraintIndex, static_cast<float>(Dt)); }
}

void FCirrovaWheelFrictionConstraints::GatherInput(const Chaos::FReal Dt)
{
	for (int32 ConstraintIndex = 0; ConstraintIndex < Slots.Num(); ++ConstraintIndex)
	{
		if (IsConstraintEnabled(ConstraintIndex)) { GatherSingle(ConstraintIndex, static_cast<float>(Dt)); }
	}
}

void FCirrovaWheelFrictionConstraints::GatherSingle(int32 ConstraintIndex, float Dt)
{
	if (!Slots.IsValidIndex(ConstraintIndex)) { return; }

	FSlot& Slot = Slots[ConstraintIndex];
	Slot.AccumulatedLongitudinal = 0.0f;
	Slot.AccumulatedLateral = 0.0f;
	Slot.AccumulatedTireAngularImpulse = 0.0f;
	Slot.AccumulatedBrakeAngularImpulse = 0.0f;
	const float SafeDt = FMath::IsFinite(Dt) && Dt > 0.0f ? Dt : 0.0f;
	Slot.MaxLongitudinalImpulse = Slot.Settings.LongitudinalForceLimit * SafeDt;
	Slot.MaxLateralImpulse = Slot.Settings.LateralForceLimit * SafeDt;
	// N·m → kg·cm²/s²（力 ×10⁴ × cm²）再乘 dt 得角冲量界。
	const float SafeCapacity = FMath::Max(Slot.Settings.BrakeTorqueCapacity, 0.0f);
	Slot.MaxBrakeAngularImpulse = FMath::IsFinite(SafeCapacity)
		? SafeCapacity * SafeDt * 10000.0f
		: 0.0f;

	// 轮端自由度消元：R²/J（cm² / (kg·cm²) = 1/kg）。制动不再切换该值——
	// 锁止由制动约束行与轮胎约束行在同一迭代内的竞争涌现。
	const float RadiusCm = Slot.Settings.WheelRadiusCm;
	const float Inertia = Slot.Settings.WheelInertiaKgCm2;
	Slot.WheelInverseMass = RadiusCm > KINDA_SMALL_NUMBER && Inertia > KINDA_SMALL_NUMBER
		? (RadiusCm * RadiusCm) / Inertia
		: 0.0f;
	if (!FMath::IsFinite(Slot.WheelInverseMass)) { Slot.WheelInverseMass = 0.0f; }

	// 本地轮速副本：迭代内被制动行与轮胎反作用实时更新；步初值来自 liminal。
	Slot.LocalOmega = FMath::IsFinite(Slot.Settings.WheelAngularVelocity)
		? Slot.Settings.WheelAngularVelocity
		: 0.0f;

	// 侧向 warm start：上一步累计侧向冲量衰减后预施加，降低动态 PGS 残差。
	// 仅在上一步未达椭圆界（静摩擦区间）且底盘有速度时启用：静止/缓蠕工况下
	// 旧冲量会与位置级静摩擦相互叠加而放大漂移（T4 回归验证）。
	Slot.AccumulatedLateral = 0.0f;
	Chaos::FSolverBody* Body = Slot.SolverBody;
	const bool bWarmStartAllowed = Body && Body->IsDynamic()
		&& Slot.LastMaxLateralImpulse > 0.0f
		&& FMath::Abs(Slot.LastAccumulatedLateral)
			< Slot.LastMaxLateralImpulse * 0.98f
		&& Body->V().Size() > Chaos::FSolverReal(50.0);
	if (bWarmStartAllowed
		&& Slot.WarmStartLateralImpulse != 0.0f
		&& Slot.MaxLateralImpulse > 0.0f)
	{
		const float WarmStart = FMath::Clamp(
			Slot.WarmStartLateralImpulse * LateralWarmStartFactor,
			-Slot.MaxLateralImpulse,
			Slot.MaxLateralImpulse
		);
		ApplyImpulseAtPoint(
			*Body,
			ToSolverVector(Slot.Settings.ContactPoint),
			ToSolverVector(Slot.Settings.LateralAxis),
			static_cast<Chaos::FSolverReal>(WarmStart)
		);
		Slot.AccumulatedLateral = WarmStart;
	}
}

void FCirrovaWheelFrictionConstraints::ScatterOutput(
	const TArrayView<int32>& ConstraintIndices,
	const Chaos::FReal Dt
)
{
	for (const int32 ConstraintIndex : ConstraintIndices)
	{
		ScatterSingle(ConstraintIndex, static_cast<float>(Dt));
	}
}

void FCirrovaWheelFrictionConstraints::ScatterOutput(const Chaos::FReal Dt)
{
	for (int32 ConstraintIndex = 0; ConstraintIndex < Slots.Num(); ++ConstraintIndex)
	{
		ScatterSingle(ConstraintIndex, static_cast<float>(Dt));
	}
}

void FCirrovaWheelFrictionConstraints::ScatterSingle(int32 ConstraintIndex, float Dt)
{
	if (!Slots.IsValidIndex(ConstraintIndex)) { return; }

	FSlot& Slot = Slots[ConstraintIndex];
	Slot.FrameResults.LongitudinalImpulse += Slot.AccumulatedLongitudinal;
	Slot.FrameResults.LateralImpulse += Slot.AccumulatedLateral;
	Slot.FrameResults.WheelAngularImpulse += Slot.AccumulatedTireAngularImpulse
		+ Slot.AccumulatedBrakeAngularImpulse;
	if (FMath::IsFinite(Dt) && Dt > 0.0f) { Slot.FrameResults.FrameDeltaTime += Dt; }
	Slot.SolverBody = nullptr;

	// 位置级静摩擦的滑动门控：本步任一轴触及界即视为滑动。
	Slot.LastAccumulatedLongitudinal = Slot.AccumulatedLongitudinal;
	Slot.LastAccumulatedLateral = Slot.AccumulatedLateral;
	Slot.LastMaxLongitudinalImpulse = Slot.MaxLongitudinalImpulse;
	Slot.LastMaxLateralImpulse = Slot.MaxLateralImpulse;

	// 侧向 warm start：本步累计量衰减后留给下一步预施加。
	Slot.WarmStartLateralImpulse = Slot.Settings.bEnabled ? Slot.AccumulatedLateral : 0.0f;
}

void FCirrovaWheelFrictionConstraints::ApplyPositionConstraints(
	const TArrayView<int32>& ConstraintIndices,
	const Chaos::FReal Dt,
	const int32 Iteration,
	const int32 NumIterations
)
{
	const float SafeDt = static_cast<float>(Dt);
	for (const int32 ConstraintIndex : ConstraintIndices)
	{
		ApplyPositionSingle(ConstraintIndex, SafeDt, Iteration, NumIterations);
	}
}

void FCirrovaWheelFrictionConstraints::ApplyPositionConstraints(
	const Chaos::FReal Dt,
	const int32 Iteration,
	const int32 NumIterations
)
{
	const float SafeDt = static_cast<float>(Dt);
	for (int32 ConstraintIndex = 0; ConstraintIndex < Slots.Num(); ++ConstraintIndex)
	{
		if (IsConstraintEnabled(ConstraintIndex))
		{
			ApplyPositionSingle(ConstraintIndex, SafeDt, Iteration, NumIterations);
		}
	}
}

void FCirrovaWheelFrictionConstraints::ApplyPositionSingle(
	const int32 ConstraintIndex,
	const float Dt,
	const int32,
	const int32 NumIterations
)
{
	if (!Slots.IsValidIndex(ConstraintIndex)) { return; }

	FSlot& Slot = Slots[ConstraintIndex];
	Chaos::FSolverBody* Body = Slot.SolverBody;
	if (!Slot.Settings.bEnabled || !Body || !Body->IsDynamic()) { return; }
	if (!FMath::IsFinite(Dt) || Dt <= 0.0f || NumIterations <= 0) { return; }

	// 滑动门控：上一物理步任一轴触及椭圆界即视为滑动，不做位置修正；
	// 否则静摩擦区间内的 PGS 残余会被逐步累积成 cm/s 级蠕滑。
	const bool bLongitudinalSaturated = Slot.LastMaxLongitudinalImpulse > 0.0f
		&& FMath::Abs(Slot.LastAccumulatedLongitudinal)
		>= Slot.LastMaxLongitudinalImpulse * 0.98f;
	const bool bLateralSaturated = Slot.LastMaxLateralImpulse > 0.0f
		&& FMath::Abs(Slot.LastAccumulatedLateral)
		>= Slot.LastMaxLateralImpulse * 0.98f;
	if (bLongitudinalSaturated || bLateralSaturated) { return; }

	const Chaos::FSolverVec3 ContactPoint = ToSolverVector(Slot.Settings.ContactPoint);
	const Chaos::FSolverVec3 LongitudinalAxis = ToSolverVector(Slot.Settings.LongitudinalAxis);
	const Chaos::FSolverVec3 LateralAxis = ToSolverVector(Slot.Settings.LateralAxis);
	const Chaos::FSolverVec3 GroundVelocity = ToSolverVector(Slot.Settings.GroundVelocity);

	// 位置迭代早于速度迭代，此处速度是积分后的预测速度（含重力）。
	const Chaos::FSolverVec3 Arm = ContactPoint - ToSolverVector(Body->CorrectedP());
	const Chaos::FSolverVec3 PointVelocity = Body->V()
		+ Chaos::FSolverVec3::CrossProduct(Body->W(), Arm);
	const Chaos::FSolverVec3 RelativeVelocity = PointVelocity - GroundVelocity;
	const Chaos::FSolverReal LongitudinalSpeed =
		Chaos::FSolverVec3::DotProduct(RelativeVelocity, LongitudinalAxis);
	const Chaos::FSolverReal LateralSpeed =
		Chaos::FSolverVec3::DotProduct(RelativeVelocity, LateralAxis);
	if (!FMath::IsFinite(LongitudinalSpeed) || !FMath::IsFinite(LateralSpeed)) { return; }

	const Chaos::FSolverReal SpeedSquared =
		LongitudinalSpeed * LongitudinalSpeed + LateralSpeed * LateralSpeed;
	if (SpeedSquared > Chaos::FSolverReal(StaticFrictionSpeedThresholdCm * StaticFrictionSpeedThresholdCm))
	{
		return;
	}

	// 残余蠕滑位移的一阶修正，每位置迭代均摊，整体上界 MaxStaticPositionCorrectionCm。
	// 位置修正不引入速度，只抵消本步的预测漂移。
	const Chaos::FSolverReal Scale = Chaos::FSolverReal(StaticPositionRelaxation)
		/ static_cast<Chaos::FSolverReal>(NumIterations);
	Chaos::FSolverVec3 Correction = (LongitudinalAxis * LongitudinalSpeed
		+ LateralAxis * LateralSpeed) * (-static_cast<Chaos::FSolverReal>(Dt) * Scale);
	const Chaos::FSolverReal CorrectionLength = Correction.Size();
	const Chaos::FSolverReal MaxCorrection = Chaos::FSolverReal(MaxStaticPositionCorrectionCm)
		/ static_cast<Chaos::FSolverReal>(NumIterations);
	if (CorrectionLength > MaxCorrection && CorrectionLength > Chaos::FSolverReal(0))
	{
		Correction *= MaxCorrection / CorrectionLength;
	}
	if (Correction.IsNearlyZero(Chaos::FSolverReal(1e-6))) { return; }

	Body->ApplyPositionCorrectionDelta(Correction);
}

void FCirrovaWheelFrictionConstraints::ApplyVelocityConstraints(
	const TArrayView<int32>& ConstraintIndices,
	const Chaos::FReal,
	const int32,
	const int32
)
{
	for (const int32 ConstraintIndex : ConstraintIndices) { ApplySingle(ConstraintIndex); }
}

void FCirrovaWheelFrictionConstraints::ApplyVelocityConstraints(
	const Chaos::FReal,
	const int32,
	const int32
)
{
	for (int32 ConstraintIndex = 0; ConstraintIndex < Slots.Num(); ++ConstraintIndex)
	{
		if (IsConstraintEnabled(ConstraintIndex)) { ApplySingle(ConstraintIndex); }
	}
}

void FCirrovaWheelFrictionConstraints::ApplySingle(int32 ConstraintIndex)
{
	if (!Slots.IsValidIndex(ConstraintIndex)) { return; }

	FSlot& Slot = Slots[ConstraintIndex];
	Chaos::FSolverBody* Body = Slot.SolverBody;
	if (!Slot.Settings.bEnabled || !Body || !Body->IsDynamic()) { return; }

	const Chaos::FSolverVec3 ContactPoint = ToSolverVector(Slot.Settings.ContactPoint);
	const Chaos::FSolverVec3 LongitudinalAxis = ToSolverVector(Slot.Settings.LongitudinalAxis);
	const Chaos::FSolverVec3 LateralAxis = ToSolverVector(Slot.Settings.LateralAxis);
	const Chaos::FSolverVec3 GroundVelocity = ToSolverVector(Slot.Settings.GroundVelocity);

	const Chaos::FSolverReal GroundLongitudinal =
		Chaos::FSolverVec3::DotProduct(GroundVelocity, LongitudinalAxis);
	const Chaos::FSolverReal LateralTarget =
		Chaos::FSolverVec3::DotProduct(GroundVelocity, LateralAxis);
	// 纵向目标：地面速度 + 松弛轮面速度（P1.3 语义，轮胎力建立滞后）。
	const Chaos::FSolverReal RelaxedLongitudinalTarget = GroundLongitudinal
		+ static_cast<Chaos::FSolverReal>(Slot.Settings.RelaxedWheelSurfaceSpeed);

	const Chaos::FSolverReal LimitX = FMath::Max(
		static_cast<Chaos::FSolverReal>(Slot.MaxLongitudinalImpulse),
		Chaos::FSolverReal(0)
	);
	const Chaos::FSolverReal LimitY = FMath::Max(
		static_cast<Chaos::FSolverReal>(Slot.MaxLateralImpulse),
		Chaos::FSolverReal(0)
	);
	const Chaos::FSolverReal BrakeImpulseBound = FMath::Max(
		static_cast<Chaos::FSolverReal>(Slot.MaxBrakeAngularImpulse),
		Chaos::FSolverReal(0)
	);
	const Chaos::FSolverReal WheelRadius = static_cast<Chaos::FSolverReal>(Slot.Settings.WheelRadiusCm);
	// 轮端惯量（kg·cm²），由 GatherInput 消元项反解；无效时制动行退化。
	const Chaos::FSolverReal WheelInertia =
		Slot.WheelInverseMass > Chaos::FSolverReal(0)
			&& WheelRadius > Chaos::FSolverReal(0)
		? (WheelRadius * WheelRadius) / static_cast<Chaos::FSolverReal>(Slot.WheelInverseMass)
		: Chaos::FSolverReal(0);
	Chaos::FSolverReal AccumulatedLongitudinal =
		static_cast<Chaos::FSolverReal>(Slot.AccumulatedLongitudinal);
	Chaos::FSolverReal AccumulatedLateral =
		static_cast<Chaos::FSolverReal>(Slot.AccumulatedLateral);
	Chaos::FSolverReal AccumulatedBrake =
		static_cast<Chaos::FSolverReal>(Slot.AccumulatedBrakeAngularImpulse);
	Chaos::FSolverReal AccumulatedTireAngular =
		static_cast<Chaos::FSolverReal>(Slot.AccumulatedTireAngularImpulse);
	Chaos::FSolverReal LocalOmega = static_cast<Chaos::FSolverReal>(Slot.LocalOmega);

	// 摩擦椭圆：坐标块迭代求交。每轴可用界由另一轴当前冲量在椭圆上的剩余量决定，
	// 纵向先求解，侧向跟随，制动行在每轮末尾收尾。
	//
	// 制动行在末尾执行，保证迭代结束时轮端状态已经过制动修正：
	// 末尾的轮胎反作用不会残留到 Scatter（否则净角冲量会把轮速重新推起）。
	// 锁止判定：制动能把轮端角速度在步内预算内维持为零时，纵向行退化为纯底盘
	// 约束（目标为地面速度、无轮端消元项），即旧锁止语义的连续化；预算不足时
	// 保持 P1.3 自由轮语义，由制动累计冲量按容量缓慢减速。
	const auto SolveBrakeRow = [&]() -> bool
	{
		if (BrakeImpulseBound <= Chaos::FSolverReal(0) || WheelInertia <= Chaos::FSolverReal(0))
		{
			return false;
		}

		// 冲量变化量钳制在剩余预算内：累计制动冲量不得超出 ±T_b·dt。
		const Chaos::FSolverReal BrakeDelta = FMath::Clamp(
			-LocalOmega * WheelInertia,
			-BrakeImpulseBound - AccumulatedBrake,
			BrakeImpulseBound - AccumulatedBrake
		);
		if (BrakeDelta != Chaos::FSolverReal(0))
		{
			AccumulatedBrake += BrakeDelta;
			LocalOmega += BrakeDelta / WheelInertia;
		}
		// 轮端被制动持有：角速度已归零且累计制动冲量未触及预算却。
		return FMath::Abs(LocalOmega) < Chaos::FSolverReal(0.01)
			&& FMath::Abs(AccumulatedBrake) < BrakeImpulseBound * Chaos::FSolverReal(0.999);
	};

	bool bWheelHeld = FMath::Abs(LocalOmega) < Chaos::FSolverReal(0.01)
		&& BrakeImpulseBound > Chaos::FSolverReal(0)
		&& WheelInertia > Chaos::FSolverReal(0);
	constexpr int32 AxisIterations = 3;
	for (int32 Iteration = 0; Iteration < AxisIterations; ++Iteration)
	{
		const Chaos::FSolverReal LongitudinalTarget = bWheelHeld
			? GroundLongitudinal
			: RelaxedLongitudinalTarget;
		// 制动持有时轮端不参与冲量分配；否则保留 R²/J 消元项（自由轮）。
		const Chaos::FSolverReal AdditionalInverseMass = bWheelHeld
			? Chaos::FSolverReal(0)
			: static_cast<Chaos::FSolverReal>(Slot.WheelInverseMass);

		const Chaos::FSolverReal LongitudinalBound = EllipseAxisBound(
			LimitX,
			LimitY,
			AccumulatedLateral
		);
		SolveAxis(
			*Body,
			ContactPoint,
			LongitudinalAxis,
			LongitudinalTarget,
			LongitudinalBound,
			AccumulatedLongitudinal,
			AdditionalInverseMass
		);

		// 轮胎反作用实时进入轮端：ΔL = −λ·R（角冲量，kg·cm²/s）。
		const Chaos::FSolverReal TireAngular = -AccumulatedLongitudinal * WheelRadius;
		if (WheelInertia > Chaos::FSolverReal(0))
		{
			LocalOmega += (TireAngular - AccumulatedTireAngular) / WheelInertia;
		}
		AccumulatedTireAngular = TireAngular;

		const Chaos::FSolverReal LateralBound = EllipseAxisBound(
			LimitY,
			LimitX,
			AccumulatedLongitudinal
		);
		SolveAxis(
			*Body,
			ContactPoint,
			LateralAxis,
			LateralTarget,
			LateralBound,
			AccumulatedLateral,
			Chaos::FSolverReal(0)
		);

		// 制动行收尾：吸收本轮的轮胎反作用，并更新下一轮的锁止判定。
		bWheelHeld = SolveBrakeRow();
	}
	Slot.AccumulatedLongitudinal = static_cast<float>(AccumulatedLongitudinal);
	Slot.AccumulatedLateral = static_cast<float>(AccumulatedLateral);
	Slot.AccumulatedBrakeAngularImpulse = static_cast<float>(AccumulatedBrake);
	Slot.AccumulatedTireAngularImpulse = static_cast<float>(AccumulatedTireAngular);
	Slot.LocalOmega = static_cast<float>(LocalOmega);
}
