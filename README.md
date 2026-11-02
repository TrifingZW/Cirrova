# Cirrova

UE5 载具仿真插件：liminal 仿真内核（Rust）的 Chaos 适配层。

## 架构

- 游戏线程：`UCirrovaVehicleComponent` 构建 liminal 拓扑；`UCirrovaSchedulerSubsystem`
  注册 Chaos 异步物理回调（`TSimCallbackObject`），编组驾驶输入与输出快照。
- 物理线程：`FCirrovaAsyncCallback` 在每个求解器子步内完成射线接触、轮胎冲量解、
  liminal 步进与力施加（底盘粒子力/力矩累加器）。
- 悬挂：默认映射为 Chaos 原生悬挂约束（`FPhysicsInterface::CreateSuspension`，
  `Chaos::FPBDSuspensionConstraints`）；创建失败或关闭开关时逐轮回退为弹簧力模型。
  防倾杆按成对车轮压缩差求附加法向力，与弹簧约束独立施加。
- 动力流：发动机、变速箱、差速器与车轮的 1D 域由 liminal 求解；差速器为开式差速约束
  + 摩擦限滑，制动/离合/车轮摩擦为库仑摩擦约束（静止不蠕行）；轮胎模型输出车轮反作用
  力矩，经端口回注。发动机支持扭矩曲线、怠速与发动机制动，默认关闭。
- 轮胎：载荷敏感摩擦圆与滚动阻力；接触点速度叠加地面刚体的平动与转动贡献。
- 输出：轮速、悬挂长度、接触状态与受力快照回传游戏线程，供 API 与调试绘制使用。

## 构建

1. `cd ThirdParty/liminal && cargo build --release`
2. 构建 UE 工程；`Cirrova.Build.cs` 链接 `target/release/liminal.lib`，缺失时构建报错。

## 参数

- `bUseNativeSuspension`：原生悬挂约束开关，仅初始化时读取；关闭后使用弹簧力模型。
- `AntiRollBars`：防倾杆成对配置，刚度为 0 时禁用。
- `DifferentialMaxTorque`：差速器限滑摩擦约束的扭矩容量；0 为纯开式。
- `TireLoadSensitivity` / `TireReferenceLoad` / `TireRollingResistance`：轮胎载荷敏感与滚动阻力。

## 已知限制

见 `ThirdParty/liminal/docs/adapter-contract.md` 第 6 节。
