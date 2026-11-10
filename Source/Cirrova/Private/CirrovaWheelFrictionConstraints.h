#pragma once

#include "CoreMinimal.h"
#include "Containers/ArrayView.h"
#include "Containers/Set.h"
#include "Chaos/ConstraintHandle.h"
#include "Chaos/PBDConstraintContainer.h"
#include "Chaos/Evolution/IndexedConstraintContainer.h"
#include "Chaos/Evolution/SolverConstraintContainer.h"
#include "Chaos/Evolution/SolverBodyContainer.h"
#include "Chaos/ParticleHandle.h"

class FCirrovaWheelFrictionConstraints;

/**
 * 单轮轮胎摩擦约束的求解输入。
 *
 * 世界坐标系，长度与位置为厘米，力为 kg·cm/s²。纵向与侧向共用摩擦椭圆界：
 * 坐标块迭代先求解纵向，再按剩余半轴长度求解侧向；制动行在每轮末尾收尾。
 */
struct FCirrovaWheelFrictionSettings
{
	// 接触点（世界坐标，cm）。
	Chaos::FVec3 ContactPoint{0.0f, 0.0f, 0.0f};

	// 接触坐标系（世界单位向量）。
	Chaos::FVec3 LongitudinalAxis{1.0f, 0.0f, 0.0f};
	Chaos::FVec3 LateralAxis{0.0f, 1.0f, 0.0f};

	// 接触点地面速度（cm/s），含地面刚体的平动与转动贡献。
	Chaos::FVec3 GroundVelocity{0.0f, 0.0f, 0.0f};

	// 本物理步开始时的轮端角速度（rad/s），来自 liminal 上一步回读；
	// 约束在迭代内维护本地副本，制动与轮胎反作用实时更新它。
	float WheelAngularVelocity = 0.0f;

	// 车轮半径（cm）与轮端等效转动惯量（kg·cm²）。
	// 纵向约束用 J_eff 把轮端自由度消元进有效质量：R²/J 与底盘接触点逆质量相加。
	float WheelRadiusCm = 0.0f;
	float WheelInertiaKgCm2 = 0.0f;

	// 本步制动扭矩容量（N·m）：逐轮容量 × 调制后的制动信号；0 为无制动。
	// 制动作为轮端 DOF 的速度约束行进入求解器，锁止由容量对比涌现。
	float BrakeTorqueCapacity = 0.0f;

	// 纵向松弛后的轮面线速度目标（cm/s）。
	// 松弛语义与轮胎模型一致：过滤的是轮胎力建立目标，不是轮端 DOF 本身。
	float RelaxedWheelSurfaceSpeed = 0.0f;

	// 纵向摩擦椭圆长轴界（kg·cm/s²）。
	float LongitudinalForceLimit = 0.0f;

	// 侧向摩擦椭圆短轴界（kg·cm/s²）。
	float LateralForceLimit = 0.0f;

	// 是否参与求解；离地或载荷为零时为 false。
	bool bEnabled = false;
};

/** 单轮轮胎摩擦约束的求解输出（当前物理帧累计）。 */
struct FCirrovaWheelFrictionResults
{
	// 对底盘施加的纵向与侧向冲量（kg·cm/s）。
	float LongitudinalImpulse = 0.0f;
	float LateralImpulse = 0.0f;

	// 轮端角冲量（kg·cm²/s）：轮胎反作用（-λ·R）与制动之和，PostSolve
	// 换算为净力矩回注一维域。
	float WheelAngularImpulse = 0.0f;

	// 累计的求解步长（s）。
	float FrameDeltaTime = 0.0f;
};

/** 轮胎摩擦约束句柄。 */
class FCirrovaWheelFrictionConstraintHandle final
	: public Chaos::TIndexedContainerConstraintHandle<FCirrovaWheelFrictionConstraints>
{
public:
	using Base = Chaos::TIndexedContainerConstraintHandle<FCirrovaWheelFrictionConstraints>;
	using Base::Base;

	static const Chaos::FConstraintHandleTypeID& StaticType();

	virtual Chaos::TVec2<Chaos::FGeometryParticleHandle*> GetConstrainedParticles() const override final;

protected:
	using Base::ConstraintIndex;
	using Base::ConcreteContainer;
};

/**
 * 轮胎摩擦约束容器：每轮一条约束，在宿主求解器的速度迭代内求解。
 *
 * 纵向约束把轮端等效惯量 J_eff 消元进标量有效质量，冲量同时决定底盘速度增量
 * 与轮端反作用力矩；制动作为第二条作用于轮端的角冲量约束行进入同一迭代，
 * 锁止由制动行与轮胎行的容量对比涌现。侧向约束仍只作用于底盘。车轮角速度
 * 由一维动力流积分，净角冲量在同一物理步的 PostSolve 以力矩回注。容器必须
 * 随求解器生存期存在：注册后不可移除，销毁前须确保求解器已停止步进。
 *
 * 直接继承约束容器基类而不使用索引容器模板：模板的内联构造函数会把其中间基类
 * 虚表实例化到本模块，而虚表中未覆写的槽位指向引擎未导出的实现。
 */
class FCirrovaWheelFrictionConstraints final : public Chaos::FPBDConstraintContainer
{
public:
	using Base = Chaos::FPBDConstraintContainer;
	using FConstraintContainerHandle = FCirrovaWheelFrictionConstraintHandle;
	using FConstraintHandleAllocator = Chaos::TConstraintHandleAllocator<FCirrovaWheelFrictionConstraints>;

	FCirrovaWheelFrictionConstraints();
	virtual ~FCirrovaWheelFrictionConstraints() override;

	/** 记录孤岛管理器；容器销毁时用于摘除残留图边。 */
	void SetIslandManager(Chaos::Private::FPBDIslandManager* InIslandManager);

	/** 为载具车轮分配或复用一条约束，返回容器索引。 */
	int32 AcquireConstraint(Chaos::FGeometryParticleHandle* Particle, int32 OwnerId);

	/** 索引是否仍绑定到指定载具。 */
	bool IsBoundToOwner(int32 ConstraintIndex, int32 OwnerId) const;

	/** 解除粒子关联并禁用；图边在下一次建图时移除。 */
	void ReleaseConstraint(int32 ConstraintIndex);

	/** 帧边界：标记仍在场的载具，随后释放未出现载具的约束。 */
	void BeginFrame();
	void MarkOwnerActive(int32 OwnerId);
	void EndFrame();

	/**
	 * 在求解器停止步进后摘除全部图边并放弃粒子关联。
	 * 此后容器不再被求解器使用，可在任意时机销毁。
	 */
	void DetachFromSolver();

	/** 写入求解输入；禁用时会清零帧输出。 */
	void SetSettings(int32 ConstraintIndex, const FCirrovaWheelFrictionSettings& Settings);

	/** 取出并清零当前帧累计的求解冲量。 */
	FCirrovaWheelFrictionResults ConsumeFrameResults(int32 ConstraintIndex);

	/** 索引关联的约束粒子（第二个元素恒为空）。 */
	Chaos::TVec2<Chaos::FGeometryParticleHandle*> GetConstrainedParticles(int32 ConstraintIndex) const;

	TArray<FConstraintContainerHandle*>& GetConstraintHandles() { return Handles; }
	const TArray<FConstraintContainerHandle*>& GetConstraintHandles() const { return Handles; }

	int32 NumConstraints() const { return Slots.Num(); }

	// 仍绑定粒子的约束数量。
	int32 NumActiveConstraints() const;

	// 指定约束当前的轮端惯量消元项 R²/J（1/kg）；未启用时为 0。
	float GetWheelInverseMass(int32 ConstraintIndex) const;

	// 锁止观测：本步求解后轮端静止（|ω| < 0.5 rad/s）且制动容量非零；
	// 锁止由制动约束行与轮胎约束行竞争涌现，不再是输入判据。
	bool IsWheelLocked(int32 ConstraintIndex) const;

	// FPBDConstraintContainer 接口。
	virtual int32 GetNumConstraints() const override final { return NumConstraints(); }
	virtual void ResetConstraints() override final {}
	virtual void PrepareTick() override final
	{
		// 求解体指针只在当前物理步有效；步进前清空，避免休眠约束遗留悬垂指针。
		for (FSlot& Slot : Slots) { Slot.SolverBody = nullptr; }
	}
	virtual void UnprepareTick() override final {}
	virtual void DisconnectConstraints(const TSet<Chaos::FGeometryParticleHandle*>& RemovedParticles) override;
	virtual void OnDisableParticle(Chaos::FGeometryParticleHandle* DisabledParticle) override;
	virtual void OnEnableParticle(Chaos::FGeometryParticleHandle* EnabledParticle) override;
	virtual void AddConstraintsToGraph(Chaos::Private::FPBDIslandManager& IslandManager) override final;

	// 无向图演化与主场景求解器工厂。
	virtual TUniquePtr<Chaos::FConstraintContainerSolver> CreateSceneSolver(const int32 Priority) override
	{
		return MakeUnique<Chaos::TSimpleConstraintContainerSolver<FCirrovaWheelFrictionConstraints>>(
			*this,
			Priority
		);
	}
	virtual TUniquePtr<Chaos::FConstraintContainerSolver> CreateGroupSolver(const int32 Priority) override
	{
		return MakeUnique<Chaos::TIndexedConstraintContainerSolver<FCirrovaWheelFrictionConstraints>>(
			*this,
			Priority
		);
	}

	void SetConstraintEnabled(int32 ConstraintIndex, bool bEnabled);
	bool IsConstraintEnabled(int32 ConstraintIndex) const;

	// 求解器回调：容器自行完成求解，不持有逐约束求解器对象。
	void AddBodies(const TArrayView<int32>& ConstraintIndices, Chaos::FSolverBodyContainer& SolverBodyContainer);
	void GatherInput(const TArrayView<int32>& ConstraintIndices, const Chaos::FReal Dt);
	void ScatterOutput(const TArrayView<int32>& ConstraintIndices, const Chaos::FReal Dt);
	void ApplyPositionConstraints(
		const TArrayView<int32>& ConstraintIndices,
		const Chaos::FReal Dt,
		const int32 Iteration,
		const int32 NumIterations
	);
	void ApplyVelocityConstraints(
		const TArrayView<int32>& ConstraintIndices,
		const Chaos::FReal Dt,
		const int32 Iteration,
		const int32 NumIterations
	);
	void ApplyProjectionConstraints(
		const TArrayView<int32>&,
		const Chaos::FReal,
		const int32,
		const int32
	) {}

	// 无向图演化（RBAN）使用的整体求解接口。
	void AddBodies(Chaos::FSolverBodyContainer& SolverBodyContainer);
	void GatherInput(const Chaos::FReal Dt);
	void ScatterOutput(const Chaos::FReal Dt);
	void ApplyPositionConstraints(const Chaos::FReal Dt, const int32 Iteration, const int32 NumIterations);
	void ApplyVelocityConstraints(const Chaos::FReal Dt, const int32 Iteration, const int32 NumIterations);
	void ApplyProjectionConstraints(const Chaos::FReal, const int32, const int32) {}

private:
	struct FSlot
	{
		FCirrovaWheelFrictionSettings Settings;
		FCirrovaWheelFrictionResults FrameResults;
		Chaos::FGeometryParticleHandle* Particle = nullptr;
		Chaos::FSolverBody* SolverBody = nullptr;

		// 本物理子步的椭圆冲量界与累计冲量（kg·cm/s）。
		float MaxLongitudinalImpulse = 0.0f;
		float MaxLateralImpulse = 0.0f;
		float AccumulatedLongitudinal = 0.0f;
		float AccumulatedLateral = 0.0f;

		// 轮端惯量消元项 R²/J（1/kg）；半径或惯量无效时为 0。
		float WheelInverseMass = 0.0f;

		// 轮端本地角速度（rad/s）：GatherInput 时初始化为步初值，速度迭代内被
		// 制动与轮胎纵向反作用持续更新。
		float LocalOmega = 0.0f;

		// 本物理步累计的轮端角冲量（kg·cm²/s）：轮胎反作用（-λ·R）与制动之和，
		// PostSolve 换算为净力矩回注一维域。
		float AccumulatedTireAngularImpulse = 0.0f;
		float AccumulatedBrakeAngularImpulse = 0.0f;

		// 本物理步制动角冲量界（kg·cm²/s）= 容量（N·m）× dt × 10⁴。
		float MaxBrakeAngularImpulse = 0.0f;

		// 上一物理步的累计冲量与界：位置级静摩擦的滑动门控。
		float LastAccumulatedLongitudinal = 0.0f;
		float LastAccumulatedLateral = 0.0f;
		float LastMaxLongitudinalImpulse = 0.0f;
		float LastMaxLateralImpulse = 0.0f;

		// 侧向 warm start：上一步累计侧向冲量的衰减副本，下一步 Gather 时预施加。
		float WarmStartLateralImpulse = 0.0f;

		int32 OwnerId = INDEX_NONE;
		int32 LastSeenFrame = -1;
		bool bBound = false;

		// 粒子或启用状态变更后须重建图边。
		bool bGraphDirty = false;
	};

	void ApplySingle(int32 ConstraintIndex);
	void ApplyPositionSingle(int32 ConstraintIndex, float Dt, int32 Iteration, int32 NumIterations);
	void AddBody(int32 ConstraintIndex, Chaos::FSolverBodyContainer& SolverBodyContainer);
	void GatherSingle(int32 ConstraintIndex, float Dt);
	void ScatterSingle(int32 ConstraintIndex, float Dt);
	void ResetFrameOutput(FSlot& Slot);

	TArray<FSlot> Slots;
	TArray<FConstraintContainerHandle*> Handles;
	FConstraintHandleAllocator HandleAllocator;

	Chaos::Private::FPBDIslandManager* IslandManager = nullptr;
	int32 FrameCounter = 0;
};
