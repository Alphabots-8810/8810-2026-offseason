# PowerManager — 省电模式 / 动态功耗预算

_Last updated: 2026-07-11_

## 这是什么

一套集中式功耗预算系统:每个周期汇总全车各电机的电池侧(supply)电流,估算电池"静息电压"(去掉负载压降后的真实电量信号),据此给**底盘**分配一个浮动的电流预算——机构限流保持固定,底盘是唯一的"缓冲垫"。电池好时底盘满功率;电池弱/机构抢电时底盘自动降载,保证不掉进 brownout;真发生 brownout 时立即砍预算、再以 50A/s 缓慢恢复。

## 调研背景(2026-07 实地考察结论)

| 队伍 | 做法 |
|------|------|
| **1678** (C2026-Public) | **没有**自动降载状态机。固定各机构 current limit + `setBrownoutVoltage(5.5V)` 硬扛 + `BatteryChecker` 纯报警(12.3V 警告 / 12.0V 换电池,防抖后弹 Elastic 通知)。功耗预算靠 PowerSlice(自制 .wpilog 赛后分析器)离线定 |
| **6328** (RobotCode2026Public) | `energy/FinanceDepartment`:Thevenin RC 电池模型(Kalman 修正)+ Peukert 库仑计数 + 主断路器热损伤模型,动态分配 drive budget(每模块 5–35A 浮动,auto 固定 50A),brownout 后砍预算 + 50A/s 恢复 |
| 社区共识 (Chief Delphi) | 2026 普遍反映 brownout 问题严重;PDP 2.0 无电流检测,总电流只能靠各 TalonFX supply current 信号加总 |

本实现 = 6328 的**架构**(浮动 drive 预算 + 非对称 brownout 响应)+ 大幅简化的电池模型(IR 补偿静息电压,不需要拟合电池参数)+ 1678 式换电池报警。

## 文件结构

- [`frc/lib/power/PowerManager.java`](../src/main/java/frc/lib/power/PowerManager.java) — 核心:电流汇总、静息电压估计、预算计算、tier 状态机、报警
- `MotorSubsystem.periodic()` — 每个机构自动上报 supply 电流(logKey 即上报名)
- `Module.periodic()` / `applyDrivePowerBudget()` — 上报底盘电流;把电池侧预算换算成 TorqueCurrentFOC 的定子电流上限并下发
- `ModuleIOTalonFX/FXS.applyDriveCurrentLimits()` — 值变化时异步写配置(timeout 0,量化去抖)
- `Robot.robotPeriodic()` — 在 `CommandScheduler.run()` **之后**调 `PowerManager.getInstance().update()`

## 关键实现细节(踩坑记录)

1. **TorqueCurrentFOC 不认 SupplyCurrentLimit**(Phoenix 6 著名陷阱)。本 repo 底盘 drive 用 TorqueCurrentFOC,所以浮动预算实际通过 `TorqueCurrent.PeakForward/ReverseTorqueCurrent` 生效,换算:`I_stator ≈ I_supply / duty`,duty 按 20% 分档向上取整,只在跨档时才写配置(控制 CAN 流量)。SupplyCurrentLimit 同时也写,覆盖 Voltage 模式(TalonFXS 变体)。
2. **配置写入是异步的**(`configurator.apply(cfg, 0.0)`),且带 1A 变化死区 + 预算 2A 量化,正常行驶中每秒最多几次写入。
3. **replay 安全**:电池电压 / brownout 标志走 `@AutoLog` inputs(`PowerManager/` 下),各机构电流来自各自 IO inputs,整条决策链可在 AdvantageKit replay 里复现。
4. **auto 不降载**:自动阶段固定 `DriveAutoModuleAmps`(默认 70A),保证自动路径可重复;降载只作用于 teleop。

## 怎么调(上机顺序)

1. **先只看日志不动行为**:AdvantageScope 里看 `PowerManager/RestVoltage`——猛加速时它应保持平稳。随负载下凹 → 调大 `BatteryResistanceOhms`(默认 0.020);随负载上凸 → 调小。这是唯一必须校的参数。
2. `PowerManager/Currents/*` 就是逐机构功耗档案(相当于免费版 PowerSlice),赛后直接出各机构耗电排名。
3. 好电池满电时 `DriveModuleLimitAmps` 应恒为 70(= `DriveModuleMaxAmps`,与 TunerConstants 的 supply limit 一致);拿块旧电池跑,看它随电压下降平滑回落。
4. 阈值:`ConserveRestVolts`(默认 12.1)/ `CriticalRestVolts`(默认 11.8)按你们电池群的实际衰减曲线微调。
5. 机构侧省电(可选):在 command 里用 `PowerManager.getInstance().shouldConserve()` 关掉"舒适负载",例如 CONSERVE 时停 Drum 空转怠速、降 LED 亮度。**不要**用它去限制 climber。

## 所有可调参数(NetworkTables 实时可调,`LoggedTunableNumber`)

| Key(前缀 `PowerManager/`) | 默认 | 说明 |
|---|---|---|
| `BatteryResistanceOhms` | 0.020 | 电池+线束等效内阻,唯一必校参数 |
| `OverheadAmps` | 3.0 | rio/radio 等未上报负载 |
| `BrownoutFloorVolts` | 7.0 | 预算不允许把母线拉到这以下(rio 触发点 6.75V) |
| `BudgetHeadroom` | 0.9 | 预算安全系数 |
| `MaxTotalBudgetAmps` | 240 | 总预算上限(主断路器短时可承受) |
| `ConserveRestVolts` / `CriticalRestVolts` | 12.1 / 11.8 | tier 阈值(对静息电压) |
| `DriveModuleMinAmps` / `MaxAmps` | 15 / 70 | 每模块浮动范围 |
| `DriveAutoModuleAmps` | 70 | auto 固定值 |
| `BrownoutRecoveryAmpsPerSec` | 50 | brownout 后预算恢复速率 |

## 参考

- 6328 FinanceDepartment/BatteryEstimator/BreakerModel: <https://github.com/Mechanical-Advantage/RobotCode2026Public>(energy 包;想要更精的电池模型可以整包抄,参数需按自家电池拟合)
- 1678 BatteryChecker: <https://github.com/frc1678/C2026-Public>
- CD 讨论: <https://www.chiefdelphi.com/t/methodologies-to-reduce-power-draw-without-impacting-robot-performance/516076>
