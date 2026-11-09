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
	 * 速度迭代保留，保证多迭代下的收敛与不越界。
	 */
	void SolveAxis(
		Chaos::FSolverBody& Body,
		const Chaos::FSolverVec3& ContactPoint,
		const Chaos::FSolverVec3& Axis,
		Chaos::FSolverReal TargetSpeed,
		Chaos::FSolverReal MaxImpulse,
		Chaos::FSolverReal& Accumulated
	)
	{
		const Chaos::FSolverVec3 Arm = ContactPoint - ToSolverVector(Body.CorrectedP());
		const Chaos::FSolverVec3 ArmCrossAxis = Chaos::FSolverVec3::CrossProduct(Arm, Axis);
		const Chaos::FSolverReal InverseMass =
			Body.InvM() + Chaos::FSolverVec3::DotProduct(ArmCrossAxis, Body.InvI() * ArmCrossAxis);
		if (!FMath::IsFinite(InverseMass)
			|| InverseMass <= Chaos::FSolverReal(KINDA_SMALL_NUMBER)) { return; }

		const Chaos::FSolverVec3 PointVelocity =
			Body.V() + Chaos::FSolverVec3::CrossProduct(Body.W(), Arm);
		const Chaos::FSolverReal CurrentSpeed = Chaos::FSolverVec3::DotProduct(PointVelocity, Axis);

		const Chaos::FSolverReal Desired = Accumulated + (TargetSpeed - CurrentSpeed) / InverseMass;
		const Chaos::FSolverReal Clamped = FMath::Clamp(Desired, -MaxImpulse, MaxImpulse);
		const Chaos::FSolverReal Applied = Clamped - Accumulated;
		Accumulated = Clamped;
		if (Applied == Chaos::FSolverReal(0)) { return; }

		const Chaos::FSolverVec3 Impulse = Axis * Applied;
		Body.ApplyLinearVelocityDelta(Impulse * Body.InvM());
		Body.ApplyAngularVelocityDelta(Body.InvI() * Chaos::FSolverVec3::CrossProduct(Arm, Impulse));
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
	Slot.FrameResults = FCirrovaWheelFrictionResults();
	Slot.Particle = Particle;
	Slot.SolverBody = nullptr;
	Slot.MaxLongitudinalImpulse = 0.0f;
	Slot.MaxLateralImpulse = 0.0f;
	Slot.AccumulatedLongitudinal = 0.0f;
	Slot.AccumulatedLateral = 0.0f;
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

void FCirrovaWheelFrictionConstraints::ReleaseConstraint(int32 ConstraintIndex)
{
	if (!Slots.IsValidIndex(ConstraintIndex)) { return; }

	FSlot& Slot = Slots[ConstraintIndex];
	if (Slot.Particle && Handles.IsValidIndex(ConstraintIndex))
	{
		Slot.Particle->RemoveConstraintHandle(Handles[ConstraintIndex]);
	}

	Slot.Settings = FCirrovaWheelFrictionSettings();
	Slot.FrameResults = FCirrovaWheelFrictionResults();
	Slot.Particle = nullptr;
	Slot.SolverBody = nullptr;
	Slot.MaxLongitudinalImpulse = 0.0f;
	Slot.MaxLateralImpulse = 0.0f;
	Slot.AccumulatedLongitudinal = 0.0f;
	Slot.AccumulatedLateral = 0.0f;
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
	if (!Slot.Settings.bEnabled) { ResetFrameOutput(Slot); }
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
	Slot.AccumulatedLongitudinal = 0.0f;
	Slot.AccumulatedLateral = 0.0f;
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
	Slot.MaxLongitudinalImpulse = Slot.Settings.FrictionForceLimit * Dt;
	Slot.MaxLateralImpulse = Slot.Settings.LateralForceLimit * Dt;
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
	Slot.FrameResults.FrameDeltaTime += Dt;
	Slot.SolverBody = nullptr;
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

	const Chaos::FSolverReal LongitudinalTarget =
		Chaos::FSolverVec3::DotProduct(GroundVelocity, LongitudinalAxis)
		+ static_cast<Chaos::FSolverReal>(Slot.Settings.WheelSurfaceSpeed);
	const Chaos::FSolverReal LateralTarget =
		Chaos::FSolverVec3::DotProduct(GroundVelocity, LateralAxis);

	Chaos::FSolverReal AccumulatedLongitudinal =
		static_cast<Chaos::FSolverReal>(Slot.AccumulatedLongitudinal);
	SolveAxis(
		*Body,
		ContactPoint,
		LongitudinalAxis,
		LongitudinalTarget,
		static_cast<Chaos::FSolverReal>(Slot.MaxLongitudinalImpulse),
		AccumulatedLongitudinal
	);
	Slot.AccumulatedLongitudinal = static_cast<float>(AccumulatedLongitudinal);

	// 摩擦圆：侧向冲量只能使用纵向求解后剩余的圆界。
	const Chaos::FSolverReal Circle = static_cast<Chaos::FSolverReal>(Slot.MaxLongitudinalImpulse);
	const Chaos::FSolverReal Used = FMath::Abs(AccumulatedLongitudinal);
	const Chaos::FSolverReal Remaining = FMath::Sqrt(FMath::Max(Chaos::FSolverReal(0), Circle * Circle - Used * Used));
	const Chaos::FSolverReal LateralBound = FMath::Min(
		static_cast<Chaos::FSolverReal>(Slot.MaxLateralImpulse),
		Remaining
	);

	Chaos::FSolverReal AccumulatedLateral = static_cast<Chaos::FSolverReal>(Slot.AccumulatedLateral);
	SolveAxis(
		*Body,
		ContactPoint,
		LateralAxis,
		LateralTarget,
		LateralBound,
		AccumulatedLateral
	);
	Slot.AccumulatedLateral = static_cast<float>(AccumulatedLateral);
}
