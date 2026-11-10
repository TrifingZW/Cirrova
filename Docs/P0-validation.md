# Cirrova P0 验证与基线

本文是物理路线 P0 的固定验证协议：确认当前原生约束实现可编译、可运行、数值稳定，并形成后续回归基线。P0 不引入新物理功能。

## 0. 自动化验证（推荐）

P0 至 P1.3 的 T0–T14 已由 `CirrovaTests` 开发者模块自动化。测试在空关卡 `/Engine/Maps/Entry` 中以无头模式运行，自行创建静态地面、台阶与测试载具，逐帧采集遥测并输出 CSV/JSON。

构建：

```powershell
cd ThirdParty/liminal
cargo test
cargo build --release

cd ../..
& "<UE>/Engine/Build/BatchFiles/Build.bat" VehicleEditor Win64 Development `
  -Project="<Project>/Vehicle.uproject" -WaitMutex -FromMsBuild
```

运行全部场景：

```powershell
powershell -ExecutionPolicy Bypass -File Plugins/Cirrova/Tools/run-p0-tests.ps1
```

Git Bash：

```sh
Plugins/Cirrova/Tools/run-p0-tests.sh all
```

只运行指定场景：

```powershell
powershell -ExecutionPolicy Bypass -File Plugins/Cirrova/Tools/run-p0-tests.ps1 -Scenario T0,T1
```

直接调用引擎（脚本内部使用同一命令）：

```sh
MSYS_NO_PATHCONV=1 "<UE>/Engine/Binaries/Win64/UnrealEditor-Cmd.exe" \
  "<Project>/Vehicle.uproject" -game -nullrhi -unattended -nosplash -log \
  -Deterministic -FPS=60 -map=/Engine/Maps/Entry \
  -CirrovaTest=all \
  -CirrovaTestOut="<Project>/Saved/CirrovaTests/run" \
  -CirrovaTestExit=1
```

命令行：

| 参数 | 说明 |
| --- | --- |
| `-CirrovaTest=all` | 场景列表：`all`、`T0`、`T0,T1` … |
| `-CirrovaTestOut=<dir>` | 输出目录，省略时写入 `Saved/CirrovaTests/<时间戳>` |
| `-CirrovaTestExit=1` | 完成后退出，退出码 0=全部通过、1=存在失败；`0` 保持运行 |
| `-CirrovaTestMaxSeconds=<s>` | 总超时，默认 900 s |
| `-Deterministic -FPS=60` | 固定 16.667 ms 步长，使测试可重复且不依赖实时帧率 |

输出：

- `T0.csv` … `T14.csv`：逐帧逐轮遥测（位姿、载荷、弹簧、滑移、轮胎力、冲量、轮端惯量消元项、地面材质摩擦、静/动摩擦界、椭圆利用率、约束状态）。
- `report.json`：场景通过/失败、失败项与关键指标。
- `summary.txt`：可直接阅读的汇总。

当前基线（本机，全部 15 场景）：

```
总结果：PASS
T0 PASS 5.00s 四轮平均载荷 2447.2 N，弹簧 0.2899 m，水平漂移 0.000 m
T1 PASS 7.00s 落地冲击在容差内收敛
T2 PASS 8.00s 制动距离 12.8 m，最终速度 0.02 m/s
T3 PASS 6.00s 行驶 15.28 m，发动机 1858 RPM
T4 PASS 7.00s 侧向斜坡静态保持
T5 PASS 7.00s 下坡制动驻车
T6 PASS 9.50s 台阶通过（圆柱包络/球体扫掠判定，射线为对照）
T7 PASS 5.50s 三种驱动布局均只输出到驱动轴
T8 PASS 5.50s 松弛长度阶跃响应单调且无发散
T9 PASS 9.50s 下压力随速度平方增长，阻力方向正确
T10 PASS 10.00s 4 车 16 约束，8 车 32 约束，物理步无持续增长
T11 PASS 7.00s P1.2 分向阻尼/刚度曲线/渐进限位/预载行为符合预期
T12 PASS 6.00s 纵向松弛阶跃响应单调且无发散
T13 PASS 9.00s 地面材质摩擦影响制动距离与摩擦上限
T14 PASS 8.00s 摩擦椭圆联合界与静摩擦过渡有效
```

## 1. 构建基线

```sh
cd ThirdParty/liminal
cargo test --quiet
cargo build --release
```

```sh
# 编辑器目标
"<UE>/Engine/Build/BatchFiles/Build.bat" VehicleEditor Win64 Development \
  -Project="<Project>/Vehicle.uproject" -WaitMutex -FromMsBuild

# 游戏目标
"<UE>/Engine/Build/BatchFiles/Build.bat" Vehicle Win64 Development \
  -Project="<Project>/Vehicle.uproject" -WaitMutex -FromMsBuild
```

通过标准：两个目标 `Result: Succeeded`；liminal 单元测试全部通过。

## 2. 场景准备

以下为手动对照或蓝图场景的准备步骤；第 0 节的自动化场景使用 `CirrovaTests` 自建载具，不需要蓝图配置。

载具蓝图实例：

- [ ] 根组件为模拟物理的 `UPrimitiveComponent`，质量约 1000 kg
- [ ] `原生悬挂约束` 已勾选
- [ ] 所有车轮/轮胎视觉件碰撞为 `NoCollision`（不得与解析悬挂双重支撑）
- [ ] `驱动布局` 与预期一致（RWD / FWD / AWD）
- [ ] `球体接触扫掠` 开启（对照测试时可关闭）
- [ ] `轮胎松弛长度` 记录当前值（默认 0.3 m）
- [ ] 空气动力学参数记录当前值

控制台与日志：

```
Cirrova.LogPhysicsStats 1
Cirrova.LogPhysicsStatsInterval 1
Cirrova.LogTelemetry 1
Log LogCirrova Log
```

组件调试：

- [ ] `周期输出遥测` 开启，`遥测输出间隔` 设为 1.0 s
- [ ] 需要几何对照时开启 `绘制调试图元`

## 3. 启动日志检查

期望出现：

```
Cirrova 调度器：轮胎摩擦约束容器已注册。
Cirrova 载具组件：原生悬挂约束启用 N/N 轮。
Cirrova 载具组件：驱动布局 四驱，前轴 2 轮，后轴 2 轮。
Cirrova 载具组件：接触检测=圆柱包络，轮胎松弛长度 0.30 m，空气动力学=启用。
Cirrova 调度器：载具 0 的轮胎耦合为求解器约束，悬挂为原生约束。
```

不得出现：

- `轮胎未进入求解器约束`
- `地面不是单粒子物理代理`
- `车辆接地但四轮法向载荷持续低于 100 N`
- `原生悬挂未生效`
- `遥测出现非有限值`

## 4. 测试矩阵

### T0 静置

平地静置 3 s。记录最后两次周期遥测。

通过标准：

- 全部车轮接地
- 弹簧长度约 0.280 m（1000 kg、35000 N/m、0.35 m 自由长度）
- 压缩量约 0.070 m
- 法向载荷约 2450 N/轮
- 纵向/侧向摩擦椭圆界约 2450 N（默认等轴）
- 滑移率约 0，侧偏角约 0
- 无持续压缩速度钳制

说明：原生 Chaos 悬挂约束的迭代有效刚度略高于名义 `k`，实测静平衡弹簧长度约 0.290 m、压缩约 0.060 m；法向载荷由 `NetPushOut` 还原，仍约为 2440–2450 N/轮。自动化以载荷、接地/约束状态与零漂移为主判据，弹簧长度采用 0.27–0.295 m 的容差。

### T1 冲击

从 0.5 m、2 m 高度落地。

记录：峰值压缩量、峰值载荷、`压缩速度已钳制`、稳定时间。

通过标准：约 1.5–2 s 内收敛；不持续弹跳、不翻滚；无 NaN。

P1.2 渐进限位会把 2 m 落地峰值载荷提高到约 53 kN；自动化以 60 kN 为有界阈值，峰值压缩仍钳制在 0.150 m 行程。

### T2 制动锁止

60 km/h 直线全制动，随后松开。

记录：滑移率、纵向力、摩擦利用率、冲量窗口、轮速恢复时间。

通过标准：纵向力不超过摩擦椭圆；无持续符号震荡；无 NaN。

自动化在 3 s 施加全制动，6.5 s 松开，8 s 检查停止距离、最终速度/轮速与末段符号震荡；制动期间校验轮胎约束的轮端逆惯量为 0（制动锁止）。

### T3 直线驱动

静止全油门 1 挡，随后 2/3 挡。

记录：发动机转速、轮面速度、滑移率、四轮纵向力。

通过标准：左右轮力大致对称；滑移率在有界范围内；无空转失控。

自动化额外校验自由驱动时轮胎约束的轮端逆惯量 `R²/J_eff > 0`（P1.1 轮端 DOF 生效）。

### T4 侧向斜坡

选择 15°、25°、35° 斜面，车辆横向放置，无输入。

记录：侧向力、侧向冲量、摩擦利用率、滑移角、是否有蠕滑。

通过标准：`tanθ < μ` 时保持准静态；超过极限后连续滑移，无抖动。

自动化判据：测量窗 2 s 内面内漂移 ≤ 0.5 m、平均速度 ≤ 0.25 m/s、平均摩擦利用 < 0.99。原生悬挂约束沿真实悬挂轴作用，Chaos 低速斜坡防滑默认关闭（`Cirrova.SuspensionSlopeAssist=1` 可恢复对照），因此侧向轮胎力真实参与支撑；PGS 残余会产生 cm/s 级蠕滑。

### T5 制动驻坡

同一斜面，车头朝下，踩住制动。

通过标准：`tanθ < μ` 时保持准静态；制动力不导致车轮反向旋转。

自动化判据：测量窗 2 s 内下坡漂移 ≤ 0.4 m、平均速度 ≤ 0.25 m/s、车轮平均角速度 < 0.5 rad/s；制动期间校验轮端逆惯量为 0（制动锁止）。等扭矩制动在载荷转移下会使前轮先达到局部摩擦上限，整车纵向力仍与重力分量一致；局部饱和作为诊断记录，制动力分配优化列入 P1.4。

### T6 路肩/台阶

10 cm、15 cm 台阶，10 km/h 与 40 km/h 通过。

分别用轮宽圆柱包络、球体扫掠与射线各跑一次。

记录：峰值载荷、接触法线、是否被弹飞、是否穿透。

- 通过标准：圆柱包络与球体扫掠均不穿透边缘；射线组作为对照基线。

自动化在测试原点处生成 4 m × 8 m 台阶，车辆从台阶前 8 m 起步，2.5 s 后以目标速度滑行，9.5 s 时检查：圆柱包络与球体扫掠车辆到达台阶后沿（允许 0.1 m 容差）、底盘 AABB 穿透 ≤ 5 cm、垂直速度 < 6 m/s、无翻车；射线组仅记录。圆柱包络沿轮宽布置三条射线，取悬挂轴上最先命中的接触；Chaos `ShapeSweep` 不支持 `FCylinder`，因此不使用形状扫掠。

### T7 驱动布局

同一车辆分别切换 RWD / FWD / AWD，静止全油门。

记录：四轮轮速与纵向力、发动机转速、车辆轨迹。

通过标准：

- RWD：仅后轴有驱动力
- FWD：仅前轴有驱动力
- AWD：前后轴均输出；中央差速行为连续

自动化额外校验驱动时轮胎约束的轮端逆惯量 `R²/J_eff > 0`（P1.1 轮端 DOF 生效）。

### T8 松弛长度

`轮胎松弛长度` 分别设 0、0.3、1.0，做快速转向阶跃。

记录：侧偏角与侧向力的建立时间、峰值、回稳。

通过标准：0 为即时响应；较大值瞬态更慢但无发散。

### T9 空气动力学

平地保持 10/20/30/40 m/s（覆盖 120 km/h 以上），对比开启/关闭气动。

记录：法向载荷增量、减速度、俯仰趋势。

通过标准：下压力随速度平方增长；阻力方向正确；无数值异常。

### T10 多车与性能

4 辆、8 辆同场。开启 `Cirrova.LogPhysicsStats 1`。

记录：步墙钟、准备/收尾耗时、轮胎约束槽数量、帧率。

通过标准：物理线程无持续增长；约束槽数量与车辆轮数一致；无明显抖动。

### T11 悬挂模型（P1.2）

同一平地对比基线悬挂与 P1.2 配置：分向阻尼（3600 / 5200 N·s/m）、刚度曲线（1.0→1.5）、渐进限位（30000 N/m，0.12 m 起）、预载 200 N；从 0.5 m 与 2.0 m 落地。

记录：峰值压缩、峰值载荷、最大回弹速度、静态压缩与载荷、收敛时间。

通过标准：

- 0.5 m：P1.2 峰值压缩与静态压缩更低、峰值载荷更高、回弹速度不超过基线 1.3 倍 + 0.05 m/s。
- 2.0 m：两侧均收敛、无 NaN、无翻车，P1.2 峰值载荷 < 70 kN。
- 全部车辆最终载荷 2200–2700 N、收敛 < 3.5 s。

### T12 纵向松弛（P1.3）

平地静止全油门 1 挡，对比 `轮胎松弛长度` 0 / 0.3 / 1.0 m。记录平均纵向力的建立时间与峰值滑移率。

通过标准：

- 0/0.3/1.0 m 的 90% 力建立时间单调不减，且 1.0 m 相对 0 m 至少慢 0.03 s。
- 峰值纵向力 > 100 N，产生可测纵向滑移，无非有限值。

基线：0.05 / 0.25 / 0.70 s，峰值滑移率 0.36 / 1.66 / 3.65。

### T13 地面材质（P1.3）

三块平地的物理材质动/静摩擦分别设为 1.0 / 0.6 / 0.35；车辆以 20 m/s 进入，2.5 s 全制动。

通过标准：

- 遥测地面摩擦与设定值一致（±0.06）。
- 制动距离随地面摩擦降低而单调增加，且相邻档位至少相差 10%。
- 平均纵向摩擦上限随地面摩擦降低而单调下降。
- 三车最终速度 < 0.8 m/s。

基线：制动距离 18.3 / 23.3 / 30.6 m，平均摩擦上限约 2454 / 1901 / 1450 N。

### T14 摩擦椭圆与静摩擦（P1.3）

三车 `轮胎侧向附着倍率` 为 1.0 / 0.5 / 0.25，`轮胎静摩擦倍率` 1.2，地面动/静摩擦 1.0 / 1.2；20 m/s 带转向全制动至停止。

通过标准：

- 每轮椭圆利用率 `sqrt((Fx/FxMax)² + (Fy/FyMax)²) ≤ 1.1`。
- 平均侧向摩擦上限随倍率降低而单调下降，相邻档至少 1.4 倍。
- 地面静摩擦查询高于动摩擦，末帧静摩擦上限高于动摩擦上限至少 5%，有效上限已过渡到静摩擦界。

基线：侧向平均上限约 2835 / 1402 / 726 N；末帧静摩擦上限约为动摩擦的 1.31 倍，有效上限等于静摩擦界。

## 5. 数据回传

每个测试至少保留：

- 周期遥测日志（含 `LogDebugSnapshot` 输出）
- 一张调试绘制截图（接地/受力）；无头自动化以 CSV/JSON 替代，需要几何对照时在有渲染的 PIE 中开启 `bDrawDebug` 后截图
- 测试参数：坡度、路面类型、质量、四轮悬挂参数、轮胎参数、驱动布局、气动参数
- `Cirrova 物理统计` 行

## 6. 无头冒烟基线

Git Bash 下须禁用路径转换，否则 `/Game/...` 会被改写成本地路径：

```sh
MSYS_NO_PATHCONV=1 "<UE>/Engine/Binaries/Win64/UnrealEditor-Cmd.exe" \
  "<Project>/Vehicle.uproject" -map=/Game/Vehicle/VehicleLevel \
  -game -nullrhi -unattended -nosplash -log -stdout \
  -ExecCmds="Cirrova.LogTelemetry 1,Cirrova.LogPhysicsStats 1"
```

本机 P0 基线：

- `VehicleEditor Win64 Development`：Succeeded
- `Vehicle Win64 Development`：Succeeded
- `cargo test`：72 passed
- 自动化 T0–T14：15/15 PASS，无 `LogCirrova` Warning/Error
- 无头运行：4 车 16 个轮胎约束槽、8 车 32 个，物理步平均 0.08 ms、峰值 < 1 ms
- 启动：原生悬挂 4/4、AWD 前轴 2 / 后轴 2、圆柱包络、松弛 0.30 m、气动启用、位置迭代 8
- 默认关卡冒烟（旧）仍可用于初始化链路检查：2 辆车、约束槽 8、步墙钟 0.05–0.29 ms

注意：默认关卡不是受控静置场景，车辆可能位于斜坡或与另一车/地形相互作用；受控验证请使用第 0 节的自动化场景。

## 7. 已知限制

- 原生悬挂约束的地面体必须是 `SingleParticleProxy`；Landscape/高度场会逐轮回退射线弹簧力，轮胎仍为求解器约束。
- 车轮视觉件碰撞必须关闭，否则形成双重支撑。
- 前后轴按挂点 `MountLocation.X`（UE +X 前）划分；特殊布局需调整挂点。
- 轮胎模型仍非完整瞬态/热模型；纵/侧向松弛均为一阶近似；尚无 Jolt 式滑移率/滑移角摩擦曲线。
- P1.1 轮端 DOF 以 `R²/J_eff` 消元进纵向约束；自由驱动/滑行时生效，制动锁止时退化为纯底盘约束。制动扭矩本身尚未作为显式约束进入轮端 DOF，滑摩/ABS 与制动力分配留待 P1.4。
- 原生悬挂约束的静态法向力由 `FPBDSuspensionResults::NetPushOut` 还原，而非 `k·compression`；旧算法会低报约 15%，导致轮胎摩擦椭圆偏小。
- Chaos 原生悬挂默认带有低速斜坡防滑机制（把法线替换为世界上方）。Cirrova 默认关闭该机制，使悬挂沿真实悬挂轴作用、轮胎摩擦参与斜坡支撑；`Cirrova.SuspensionSlopeAssist 1` 可恢复对照。
- 轮胎摩擦约束仍是速度级 PGS：斜坡上会有 cm/s 级稳态蠕滑（T4/T5 以准静态判据验证）。P1.3 已用动/静摩擦过渡与椭圆坐标块迭代降低残差；严格零蠕滑仍需位置级静摩擦约束。
- 等扭矩制动在载荷转移下会使前轮先达到局部摩擦上限；整车力仍与重力分量一致，制动力分配/ABS 列入 P1。
- Chaos `ShapeSweep` 的类型分派只支持 Sphere/Box/Capsule/Convex，不支持 `FCylinder`；圆柱包络以轮宽多射线近似。
- 车轮接触会忽略与底盘上向量夹角超过 80° 的垂直墙面，避免把墙面当作支撑面；极陡坡（>80°）同样不会被车轮支撑。
- 地面物理材质摩擦由接触形状解析，并按 `sqrt(轮胎·地面)` 合并；无材质时回退 1。复合材质/地形图层混合尚未细分。
- 测试载具通过 `bAllowPhysicsSleep=false` 禁用物理休眠；量产载具默认允许休眠，驾驶员输入或制动会显式唤醒并清零休眠计数。
- 未包含网络同步、回放、燃料单元与游戏侧视觉/音频。
