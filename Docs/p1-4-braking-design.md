# P1.4 制动与驱动分配设计稿

状态：已完成（T15–T17 已入库，T0–T17 全量 PASS）。本文保留为实施前的设计记录，当前回归基线与完整验收见 `P0-validation.md`。

本文是 P1.4（制动扭矩进入轮端 DOF、制动力分配、ABS/TCS、差速器限滑参数化）的详细设计。
范围限定为 Cirrova 适配层：**不修改 liminal 的 Rust 代码与 C ABI 版本**——所需能力
（逐转向节 `max_brake_torque`、逐差速器 `max_torque`、按端口写信号、`J_eff` 查询、
扭矩回注）在当前 ABI 3 下均已具备。

对照代码：

- 约束容器：`Source/Cirrova/Private/CirrovaWheelFrictionConstraints.{h,cpp}`
- 物理线程闭环：`Source/Cirrova/Private/CirrovaVehicleAsync.cpp`
- 组件与拓扑构建：`Source/Cirrova/Public/CirrovaVehicleComponent.h`、
  `Source/Cirrova/Private/CirrovaVehicleComponent.cpp`
- 运行态结构：`Source/Cirrova/Private/CirrovaVehicleRuntime.h`

## 0. 边界决策

### 0.1 制动求解归属：约束容器，不进 liminal

制动要成为轮端 DOF 上的显式约束，必须在速度迭代内维护逐迭代状态（累计角冲量、
本地 ω）。liminal 的轮胎模型按设计是无状态纯函数（README：「库模型保持无状态纯函数，
瞬态松弛长度由 Cirrova 适配层实现」），迭代态只能活在宿主求解器容器中。因此：

- **轮端制动约束行**：`FCirrovaWheelFrictionConstraints` 内新增（本设计的核心）；
- **liminal 侧**：保持现有 `SteeringKnuckle` 摩擦约束不动，作为外力回退路径的
  制动权威；约束形态下通过信号门控避免双重施加（见 2.5）。

### 0.2 权威切换：按「本步该轮是否进入求解器」门控

制动扭矩的唯一施加者，按 `Wheel.bTireConstraintApplied`（已有字段）逐轮判定：

- **true（本轮以轮胎约束进入求解器）**：轮端 DOF 上的制动由容器求解；
  转向节制动信号写 0。
- **false（离地、载荷为零、容器不可用、外力回退）**：转向节制动信号写调制值，
  liminal 一维域摩擦约束承担制动。

切换在接触边缘自然连续：失去接触的瞬间轮胎本就无力可施加，ω 跨权威切换连续。

### 0.3 外力回退路径不动

`SolveTire`（外力回退）与子步化逻辑保持现状，仅制动信号来源改为 per-wheel 直写
（现经制动分发器）。回退路径的制动始终由 liminal 转向节约束承担。

## 1. 数据结构变更

### 1.1 `FCirrovaWheelFrictionSettings`（约束容器输入）

```cpp
struct FCirrovaWheelFrictionSettings
{
	// ... 现有字段不变（ContactPoint / 轴 / GroundVelocity / 半径 / 椭圆界 / bEnabled）...

	// 删除：bool bWheelLocked（二值锁止判据废除）
	// 删除：float WheelSurfaceSpeed（目标改由 WheelAngularVelocity 推导，见 2.2）

	// 本物理步开始时的轮端角速度（rad/s），来自 liminal 上一步回读。
	float WheelAngularVelocity = 0.0f;

	// 轮端等效转动惯量（kg·cm²），现有 WheelInertiaKgCm2 字段复用。

	// 本步制动扭矩容量（N·m）：MaxBrakeTorque_perWheel × 调制后的制动信号。
	// 0 表示无制动。
	float BrakeTorqueCapacity = 0.0f;

	// 纵向松弛后的轮面线速度目标（cm/s）。
	// 松弛语义与 P1.3 一致：过滤的是轮胎目标，不是轮端 DOF 本身。
	float RelaxedWheelSurfaceSpeed = 0.0f;
};
```

### 1.2 `FSlot`（约束容器槽位）

```cpp
struct FSlot
{
	// ... 现有字段不变 ...

	// 轮端本地角速度（rad/s）：GatherInput 时初始化为步初值，
	// 速度迭代内被轮胎反作用与制动角冲量持续更新。
	float LocalOmega = 0.0f;

	// 累计角冲量（kg·cm²/s）：轮胎纵向反作用（-λ·R）与制动（μ）之和。
	// 用于 ScatterOutput 换算回注扭矩。
	float AccumulatedTireAngularImpulse = 0.0f;
	float AccumulatedBrakeAngularImpulse = 0.0f;

	// 本子步制动角冲量界（kg·cm²/s）= BrakeTorqueCapacity·dt·10⁴。
	float MaxBrakeAngularImpulse = 0.0f;
};
```

### 1.3 `FCirrovaWheelRuntime` / 组件配置

- `float MaxBrakeTorque` → `TArray<float> MaxBrakeTorquePerWheel`；
- 新增 `TArray<uint64> BrakeSignalPorts`（各转向节制动端口，构建期捕获）；
- 新增（组件 UPROPERTY，P1.4b）：
  - `FrontBrakeTorqueScale` / `RearBrakeTorqueScale`（默认 1.0，按挂点
    `MountLocation.X > 0` 划分前后轴，与驱动布局同一约定）；
- 新增（组件 UPROPERTY，P1.4c）：
  - `bEnableABS`、`ABSSlipThreshold`（默认 0.30）、`ABSReleaseThreshold`（默认 0.15）、
    `ABSApplyRate` / `ABSReleaseRate`（默认 60.0 / 40.0 每秒）、`ABSMinSpeed`（默认 2.0 m/s）；
  - `bEnableTCS`、`TCSSlipThreshold`（默认 0.20）、`TCSReleaseRate`（默认 10.0 每秒）。

## 2. P1.4a 核心算法：制动作为轮端 DOF 的第二约束行

### 2.1 问题表述

轮端自由度 ω 与底盘接触点速度经轮胎纵向约束隐式联立（P1.1 已实现）。P1.4a 在同一
迭代框架内增加第二条作用于 ω 的约束行——制动摩擦行：

- **轮胎行**（现有，改造）：`v_c·l + ω·R − v_g·l → 目标`，未知量 λ（底盘线性冲量），
  底盘侧有效质量含 `R²/J_eff` 消元项，冲量界为摩擦椭圆剩余半轴；
- **制动行**（新增）：`ω → 0`，未知量 μ（角冲量），界 `|μ| ≤ T_b·dt·10⁴`
  （N·m → kg·cm²/s² 再乘 dt）。

两行共享 ω，按 PGS 交替求解。锁止成为涌现行为：`T_b·dt ≥ |λ_椭圆界·R|` 时制动行
每迭代把 ω 拉向 0，轮胎行退化为纯底盘滑动（目标中 ω 项消失），即现 `bWheelLocked`
判据的连续化——不再有二值切换的边界跳变。

### 2.2 求解序列（ApplySingle 内，替换现有纵向求解段）

```
输入：Slot（含 LocalOmega、累计量、界）、Body、Settings

每个速度迭代（宿主调度）× AxisIterations = 3 次：

  1) 轮胎纵向行：目标含松弛轮面速度（P1.3 语义不变）
     target   = ground_v·l + RelaxedWheelSurfaceSpeed
     invMass  = 底盘接触点逆质量 + R²/J_eff
     λ        = clamp(AccumulatedLongitudinal
                      + (target − 当前底盘点速·l)/invMass,
                      ±EllipseAxisBound(X 界, Y 界, 累计侧向))
     Δλ       = λ − AccumulatedLongitudinal
     AccumulatedLongitudinal = λ
     底盘 += Δλ（线性 + 角速度增量，现有 SolveAxis 路径）
     LocalOmega −= Δλ·R / J_eff                        // 轮胎反作用实时进入 ω
     AccumulatedTireAngularImpulse = −AccumulatedLongitudinal·R
     制动持有时：invMass 不含 R²/J、target 取地面速度（纯底盘滑动）

  2) 侧向行：不变（目标 = 地面侧向速度，无轮端耦合，无消元项）

  3) 制动行（每轮末尾收尾，吸收本轮轮胎反作用）：
     μ_raw    = -LocalOmega · J_eff                // 角动量归零所需
     μ        = clamp(AccumulatedBrake + μ_raw, ±MaxBrakeImpulse)
     Δμ       = μ - AccumulatedBrake
     AccumulatedBrake = μ
     LocalOmega += Δμ / J_eff
     锁止判定 = |ω| < 0.01 && |AccumulatedBrake| 未触预算却
```

实现时把制动行放在每轮末尾而非开头：末尾收尾保证轮胎反作用不会残留到 Scatter，
否则净角冲量会把轮速重新推起。PGS 的固定点与顺序无关。

与现实现的关键差异：

- 轮胎行的**目标与有效质量保持 P1.3 语义**（松弛目标 + `R²/J_eff` 消元），行为
  对驱动/滑行工况零变化；
- 轮端反作用**不再等到 PostSolve 一次性施加**，而是在迭代内实时更新 `LocalOmega`，
  使制动行看到的是本迭代内的真实轮速——这是「制动扭矩显式进入轮端 DOF」的全部含义；
- 顺序为制动先、轮胎后：边界工况下制动容量优先（与「制动覆盖轮胎上限即锁止」的
  旧语义方向一致）。

### 2.3 回注（PostSolve / FinishVehicle）

`ReadTireConstraint` 的回注公式从「仅轮胎反作用」改为「轮胎 + 制动净角冲量」：

```
τ_write[N·m] = (AccumulatedTireAngularImpulse + AccumulatedBrakeAngularImpulse)
             × 10⁻⁴ / dt        // kg·cm²/s → N·m·s → N·m
```

- 与现值的关系：`AccumulatedTireAngularImpulse = −λ_total·R` 恰为现
  `−F_long·R` 的冲量形式；新增项即制动贡献；
- 等价表述：`τ_write = (ω_final − ω_init)·J_eff / dt`（若内部一致则两式相等，
  取求和式保留遥测来源可分性）；
- `J_eff` 一致性由构造保证：容器使用的 `WheelInertiaKgCm2` 与 liminal
  `port_effective_inertia` 出自同一查询，`solve_island` 与该查询共用
  `build_bodies / collect_inertia`。

**注意**：在挡工况下该扭矩作用于整个转速连通组（含发动机/变速箱折算），这正是
物理正确的轮端耦合，与 P1.1 语义一致。

### 2.4 锁止遥测（测试契约迁移）

`bWheelLocked` 从输入判据变为输出观测量，由容器在 `IsWheelLocked` 中给出：

```
bWheelLocked = 本步制动行启用（容量 > 0）
            && |ω_final| < 0.5 rad/s
```

实现时去掉了设计初稿中的“纵向滑移 > 0.5”条件：停车驻坡（T5）时车轮静止且滑移约零，
滑移条件会误判为非锁止；制动容量与轮端静止已足以表达“制动持有轮端”。

`FCirrovaWheelOutput` 新增 `bWheelLocked` 与 `BrakeTorqueCapacity`；导出 CSV
列随之新增。**T2/T5 中 `MeanConstraintWheelInverseMass < 0.005`（制动锁止）断言
迁移为 `bWheelLocked` 均值 > 0.9**；T3/T7 的 `> 0`（自由驱动）断言不变且恒成立。
详见第 5 节。

### 2.5 权威门控（双重施加防护）

`PrepareVehicle` 写转向节制动信号处（P1.4b 的直写路由，见 3.2）：

```
对每轮：
  modulated = ABS 调制后的本轮制动信号 ∈ [0,1]
  signal    = Wheel.bTireConstraintApplied（本步将进入求解器） ? 0 : modulated
  liminal_vehicle_write_signal(Handle, BrakeSignalPorts[i], signal)
```

同时删除 `WriteTireConstraint` 中的 `bWheelLocked` 判据与
`Settings.WheelSurfaceSpeed` 的构造（目标值改由 1.1 的新字段承载）。

诊断：约束形态下若检测到 `bTireConstraintApplied && 转向节制动信号 > 0`，输出一次
`LogCirrova Warning`（权威分裂，理论上不可达，作为不变量守护）。

### 2.6 单位表

| 量 | 容器内单位 | 换算 |
| --- | --- | --- |
| λ（线性冲量） | kg·cm/s | ×10⁻² → N·s |
| R | cm | ×10⁻² → m |
| J_eff | kg·cm² | ×10⁻⁴ → kg·m² |
| T_b·dt（角冲量界） | kg·cm²/s | N·m·s ×10⁴ |
| ω | rad/s | 不换算 |
| τ_write | N·m | 角冲量 kg·cm²/s ×10⁻⁴ / dt |

## 3. P1.4b 制动力分配

### 3.1 逐轮容量

`BuildTopology` 中：

```
perWheelMaxBrake[i] = MaxBrakeTorque
    × (前轴 ? FrontBrakeTorqueScale : RearBrakeTorqueScale)
liminal_vehicle_add_knuckle(Handle, perWheelMaxBrake[i], &Knuckle)
Runtime.MaxBrakeTorquePerWheel[i] = perWheelMaxBrake[i]
```

外力回退路径自动获得同一分配（转向节容量即 liminal 摩擦约束上限）。
可选动态模式（按 `NormalLoad` 比例分配）留作后续，不进本期。

### 3.2 信号路由改造（移除制动分发器）

已核实：`sync_signals` 仅沿边拷贝，信号汇点无上游不构成编译错误。因此：

- `BuildTopology` 删除 `BrakeSplitter` 节点及 `驾驶舱制动端口 → 分发器`、
  `分发器 → 各转向节` 的全部边；
- 逐轮捕获 `BrakeSignalPorts[i] = PortOf(Handle, Knuckle_i, 3)`（制动端口索引 3，
  现由 `PortOf(Handle, Knuckle, 3)` 的既有调用模式覆盖，仅需把端口值存入 Runtime）；
- `PrepareVehicle` 不再写 `Runtime.BrakeSignalPort`，改为 2.5 节的逐轮门控直写；
- 转向路由在 P1.4 阶段保持 `SteeringSplitter`；P2 阿克曼转向时也改为逐轮直写（分发器已移除）。

该路由同时是 P1.4c（ABS per-wheel 调制）与 P2 手刹（后轴专属信号）的前提。

### 3.3 松弛重置跟随调制信号

`UpdateLongitudinalRelaxation` 的 `bBraking` 参数从整车 `Input.Brake > 0` 改为
该轮调制后信号 > 0：ABS 释压中的车轮不应重置纵向松弛。

## 4. P1.4c ABS / TCS

物理线程控制器，状态存 `FCirrovaVehicleRuntime`（`TArray<float> ABSModulation`，
初始化 1），在 `PrepareVehicle` 写信号前执行，输入为上一步遥测
（`Wheel.SlipRatio / Omega`，`FinishVehicle` 已更新）。

```
ABS（逐轮）：
  speed = 底盘速度（m/s）
  if !bEnableABS || speed < ABSMinSpeed || 该轮未接地：
      modulation → 1（速率 ABSApplyRate 回升）
  else if SlipRatio > ABSSlipThreshold：      // 进入深滑移
      modulation −= ABSReleaseRate · dt       // 快速泄压
  else if SlipRatio < ABSReleaseThreshold：   // 回抓
      modulation += ABSApplyRate · dt         // 缓慢建压
  modulation = clamp(modulation, 0, 1)
  轮制动信号 = 驾驶制动 × modulation

TCS（整车，调制油门写入）：
  if bEnableTCS && 任一驱动轮 SlipRatio > TCSSlipThreshold：
      tcsScale −= TCSReleaseRate · dt
  else：
      tcsScale += TCSReleaseRate · dt（回 1）
  写入驾驶舱油门端口 = clamp(Throttle × tcsScale, 0, 1)
```

迟滞（进入 0.30 / 退出 0.15）、泄压下限 0.1、泄压 40/s / 建压 60/s 是经典 ABS 形态；
不做轮速差预报（滑移率已含所需信息）。`FCirrovaWheelOutput` 新增 `ABSModulation` 遥测。

实测标定：初版 0.25/0.12、8/20 速率下建压过慢，低附着路面停止距离比无 ABS 长 16%；
40/40 速率下为 +5%；加入位置级静摩擦后重新标定为 40/60 + 泄压下限 0.1，最终差 1.1%
（T16 基线 33.35 / 33.72 m）。在无滑移率曲线的平坦摩擦模型下，ABS 不带来峰值附着收益，
距离略长属于预期。

## 5. 测试与基线迁移

### 5.1 既有断言迁移（破坏性，需显式重订基线）

| 场景 | 现断言 | 新断言 |
| --- | --- | --- |
| T2 | 制动期间 `constraint_wheel_inv_mass` 为 0（锁止） | 制动窗口 `bWheelLocked` 均值 > 0.9；末速/轮速、停止距离、符号震荡断言保留 |
| T5 | 制动期间 `MeanConstraintWheelInverseMass < 0.005` | 驻车窗口 `bWheelLocked` 均值 > 0.9；`MeanAbsOmega < 0.5` 保留 |
| T3/T7 | 自由驱动 `R²/J_eff > 0` | 不变（恒成立） |

### 5.2 新增场景

- **T15 制动力分配**：三车前/后制动比例 60/40、50/50、40/60，20 m/s、25% 制动减速。
  判据：前后轴平均制动力之比与配置比例一致（±25%）；三车均停止。
  实测力比 1.52 / 1.13 / 0.81。
- **T16 ABS**：地面动摩擦 0.35，20 m/s 全制动，ABS 开/关对照。判据：
  ABS 关锁止帧占比 > 0.5，开至少低 0.2；开出现泄压（最小调制 < 0.9）；
  停止距离开 ≤ 关 × 1.05 + 1 m；末速 < 0.8 m/s。
  基线：锁止占比 1.00 / 0.24，峰值滑移 0.77 / 0.42，停止距离 33.35 / 33.72 m。
- **T17 限滑参数化**：对分路面（左 μ=1.0、右 μ=0.12）静止起步，
  `前/后轴差速器限滑扭矩容量` = 0 / 50 / 100 N·m 三档。判据：高 μ 侧轮纵向力
  随限滑扭矩单调不减（容差 50 N）；行驶距离单调不减（容差 0.2 m）；无非有限值。
  基线：高侧驱动力 1701 / 1906 / 1926 N，距离 24.24 / 26.04 / 26.37 m。
  注：初稿用 0/200/600，200 以上已完全锁止输出轴，测得结果相同；降至 0/50/100 才处于敏感区间。

### 5.3 回归门禁

- `cargo test` 不动（liminal 零改动）；
- `VehicleEditor` / `Vehicle` 两目标构建；
- T0–T17 全量 18/18 PASS。

## 6. 风险与开放问题

1. **双重施加**是最大风险：门控（2.5）+ 不变量守护日志双保险；测试矩阵覆盖
   接触/离地切换（T1 落地、T6 台阶天然覆盖权威切换帧）。
2. **T2/T5 基线平移**：锁止边界从启发式切换变为连续涌现，制动距离可能变化
   ±10%；接受并重订，不做数值贴合旧基线的调参。
3. **PGS 预算**：每速度迭代增加 2 行 × 3 次往返的标量运算，可忽略；但制动行与
   悬挂约束的迭代顺序由容器注册顺序决定，T0/T1 静置与落地行为需确认无回归。
4. **松弛目标与真实 ω 的不一致**（既有近似，P1.3 遗留）：轮胎行目标用松弛值、
   制动行用真实 `LocalOmega`，两者 intentionally 不同——前者模拟胎体力建立滞后，
   后者是制动器的刚性作用。文档化为设计选择。
5. **开放**：换挡瞬态（无离合器节点，`J_eff` 突变）与手刹锁止（现由连续制动行
   自然涌现，P2 接入信号即可）不在本期。

## 7. 实施顺序

| 步骤 | 内容 | 依赖 | 验收 |
| --- | --- | --- | --- |
| a1 | 逐轮容量管线（UPROPERTY、per-knuckle add、Runtime 数组） | 无 | T0–T14 全绿（行为无变化） |
| a2 | 容器制动行 + LocalOmega + 净角冲量回注 + 门控 + 遥测迁移 | a1 | T2/T5 新断言，重订基线 |
| b | 分发器移除 + 直写路由 + 前后比例 | a2 | T15 |
| d | 限滑逐轴参数化 | 无（可与 a2 并行） | T17 |
| c | ABS/TCS 控制器 | b | T16 |

预估改动量：`CirrovaWheelFrictionConstraints.{h,cpp}` ~150 行、
`CirrovaVehicleAsync.cpp` ~120 行、`CirrovaVehicleComponent.{h,cpp}` ~100 行、
`CirrovaVehicleRuntime.h` ~30 行、测试 ~400 行、文档三处（roadmap / 本设计稿状态、
`adapter-contract.md` 第 4/6 节制动语义、`P0-validation.md` 基线）。
