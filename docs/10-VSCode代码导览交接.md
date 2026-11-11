# 10 · VS Code 代码导览交接单（2026-11-09 刷新版）

> 本文件为2026-11-09历史交接；最新审计及保留问题以 `docs/13` 为准，现场教程读 `docs/12`，参数改动查 `docs/14`。

> **交接对象**：VS Code 里内置的 Claude Code（只读辅助）
> **为什么有这份文件**：用户在 Claude Code 里的对话历史丢失了，需要把上下文一次性补回来。
> **谁写的**：DeepSeek Harness（负责写代码、跑测试、维护文档）。
>
> **一句话背景**：用户在做一个 RoboMaster 舵轮步兵电控项目，快到考核截止，
> 很快要去基地调车。代码已经写完并验证过，现在缺的是"看得懂 + 现场会用"。

---

## 1. 现在的状态（2026-11-09）

| 项 | 状态 |
|---|---|
| 仓库 | `D:\ForTest\steered_skeleton\steered_skeleton`，分支 `master` |
| 最新提交 | `9347371`（文档自检修正） |
| 两板编译 | 通过（底盘 Flash 72656 B / 云台 61168 B） |
| 主机测试 | **692 项，0 失败**（分 7 组，见第 6 节） |
| 硬件 | 用户**没有**硬件，尚未上车 |
| 截止时间 | 很近；用户很忙、很焦虑 |

**最近两轮大改**（2026-10-27、11-08）：
- 控制与安全修复 12 项：限速舵角与最终方向分离、四轮统一缩放、积分限幅单位、
  IMU 坏样本整组校验、NaN/Inf 防护、CAN 发送失败锁存 + bus-off 检测、
  失败后停喂狗、对板重启撤销授权、旧帧重复保活拒绝、yaw 周期归一化。
  详见 `docs/11-控制与安全修复记录.md`。
- 现场手册重写：`docs/12-基地现场一页纸.md` 现在包含烧录命令、接线、
  上电判断、常见坑。

---

## 2. 用户是谁、怎么讲

- 刚开始学编程，**不懂术语**，需要生活类比 + 分层讲解（整体→逐行→底层）。
- 已经在对话里学完 6 站理论（见第 3 节），但**没在编辑器里走过代码**。
- 目标：看懂 + 最终能自己重写；眼下更急的是"现场出问题知道去哪看"。
- **必须遵守**（完整版见 `~/.dsh/AGENTS.md`）：
  1. 术语第一次出现给齐：全称 / 一句话定义 / 生活类比 / 在本项目哪里用。
     **中文简称也算术语**（"钳位""限幅""IMU""PID"都要解释）。
  2. 每个物理量说清是"转子端"还是"输出轴"、减速比在哪一层除过。
  3. 讲调试操作要能落地：什么硬件、哪个窗口、什么前提、没硬件怎么办。
  4. 讲"坑"要分类：参数配置错 / 接线硬件错 / 代码逻辑错。
  5. 不要用"时间计划/工期"式表达，用**关卡式**推进。
  6. 一次只推进一小块，讲完给 2~3 个自查问题。

---

## 3. 用户已经学过的（不要再从头讲）

理论六站（在 DeepSeek Harness 对话里讲的）：

| 站 | 内容 | 用户已经会 |
|---|---|---|
| 1 单电机 | CAN 帧、ID、原始值↔物理量、减速比 19.2 | 知道 `vel()` 已是输出轴 |
| 2 单轮组 | 两个环并列、PID、电压 vs 电流、对齐门 | 知道舵角没到位不给驱动 |
| 3 底盘 | `v = vc + ω×r`、δ 变换、等效解翻转、死区 | 能算自转时各轮方向 |
| 4 云台 | 关节角 vs 世界角、IMU、WrapPi、限位 | 知道 yaw 必须用 IMU |
| 5 双板 | 8 字节帧、三种模式、平移永远云台系 | 知道跟随是"车头追枪口" |
| 6 安全 | 五层保护、喂狗只在末尾、复位原因 | 知道急停在最外层 |

**他踩过、已纠正过的坑**（如果发现他又犯了，用一两句话提醒即可）：

1. 以为 `vel()` 还要再除一次 19.2（把 1 m/s 算成 0.05 m/s）。
2. 以为原地左转时左前轮往"左前"走（实际是"左后"）。
3. 以为"项目不需要位置环"（其实舵角环就是位置环）。
4. 以为分离模式"按车头方向走"（说明书 3.2：平移永远按**云台**方向）。
5. 以为可以"只接一个轮组"调车（固件要求 10 台电机全在线才进 Working）。

---

## 4. 代码地图（含 10-27 新增）

```
steered_skeleton/
├── RobotComponents/component/     ← 两板共用
│   ├── module_state.hpp           ← 枚举、WrapPi、Clamp
│   ├── pid.hpp                    ← PID（含积分限幅、抗饱和、微分低通）
│   ├── gimbal_chassis_comm.*      ← 8 字节双板协议 + CRC8 + 序号
│   ├── fault_injection.*          ← 故障注入开关（默认关）
│   ├── diagnostics.*              ← 控制周期耗时统计
│   └── can_tx_guard.hpp           ← ★新增：CAN 发送失败锁存/恢复
├── Chassis/
│   ├── Core/Src/main.c            ← 入口（只看 USER CODE）
│   ├── Task/component/            ← main_task / comm_task
│   ├── RobotModules/component/    ← robot / chassis / chassis_ik_solver
│   ├── Instance/component/        ← ins_motor / ins_pid / ins_imu / ins_chassis_iksolver
│   ├── src/                       ← HW_can / imu / DT7
│   └── inc/                       ← 骨架组件头文件（当黑盒）
├── Gimbal/                        ← 与 Chassis 镜像
├── tests/                         ← 5 个测试文件 + 4 个 run 脚本 + 3 个 stub 目录
└── docs/00~12                     ← 交付、逆解、协议、整定、安全、调车、审计、
                                     故障注入、待办、本交接、修复记录、现场一页纸
```

---

## 5. 导览路线（优先级从高到低）

> 用户时间紧。**先走 A→B→C→F 四站**（现场排障最用得上），
> D/E/G/H 有空再补。

### 站 A：入口（10 分钟）
- `Chassis/Core/Src/main.c`，只看 `USER CODE 2`。
- 看：`MainTaskInit()` 在 `HAL_TIM_Base_Start_IT(&htim6)` **之前**。
- 问：反过来会怎样？（中断会在对象没装配好时进来）

### 站 B：装配 + 心跳（15 分钟）
- `Chassis/Task/component/main_task.cpp`。
- `MainTaskInit()` 顺序、`MainTask()` 只有一句、TIM6 回调里先控制后通信。
- 问：为什么 `while(1)` 是空的？

### 站 C：决策层（20 分钟，最重要）
- `Chassis/RobotModules/component/robot.cpp`：
  - `update()` 五步；
  - `applyOutput()`：急停 → 非 Working → `stop()`；
  - `updateStateMachine()`：**四个条件**（遥控 / IMU / 8 台电机 / 对板 Working）；
  - `generateCommand()`：三模式的 w 来源、`UnitClamp`、Slew 限加速度、兜底 `Clamp`。
- 问：为什么急停放在状态机前面？
- 问：少接一台电机，车会怎样？（永远 Dead）

### 站 F：通信 + 安全（25 分钟）
- `comm_task.cpp`：DBUS 校验→解码→通知；CAN 发送；**唯一喂狗点**；
  新增的 `CanTxGuard`（连续失败 10ms 停喂狗、故障锁存、要急停才能解除）。
- `HW_can.cpp`：ID 分发、decode、bus-off 检测。
- `gimbal_chassis_comm.hpp`：8 字节字段、CRC8、序号防旧帧。
- 问：为什么喂狗不能放在中断里？

### 站 D：执行层（20 分钟）
- `chassis.cpp::update()`：读舵角 → 逆解 → 两个环 → `setInput`；
  `kSteerReadyAngle = 0.35f` 与 `cos(err)` 缩放；统一缩放四轮。

### 站 E：数学层（15 分钟）
- `chassis_ik_solver.cpp::solve()`：R(+δ)、`v = vc + ω×r`、等效解翻转、死区。
- 提醒：说明书 3.2 印的是 R(−δ)，与它自己的 δ 定义矛盾；代码用 R(+δ)，测试锁定。

### 站 G：云台（15 分钟）
- `gimbal.cpp`：yaw 世界角、pitch 关节角、限位钳位；
- `Gimbal/robot.cpp`：`gimbal_enable`、使能上升沿对齐、pitch 到边界的处理。

### 站 H：配置层（10 分钟）
- `ins_motor.cpp`：ID / 方向 / 零点（四台转向全 `kDirRev`）；
- `ins_pid.cpp`：两组增益与限幅。

### 站 I：测试当活文档（10 分钟）
- `tests/run_tests.ps1` 统一入口（PowerShell 5.1 / 7 都能跑）；
- 新套件：`test_control.cpp`、`test_imu.cpp`、`test_protocol_safety.cpp`、
  `test_robot_safety.cpp`。

---

## 6. 现场排障速查（讲课时可以直接用这张表）

| 现象 | 先看哪里 |
|---|---|
| 车完全不动、推杆没反应 | `Robot::updateStateMachine()` 的四个条件；`CanTxGuard` 是否锁存 |
| 一块板重启后车不动 | 对板重启会撤销授权，需要"有效急停"才恢复（`robot.cpp`） |
| 关遥控器不复位 | `CommTask` 末尾的喂狗条件；`kDebugAlwaysFeedWatchdog` 是否被改 |
| 某个轮子方向反 | `ins_motor.cpp` 的 `kWheelDir` / `kSteerDir` |
| 舵角差固定角度 | `kSteerAngleOffset`（四台分别是 2.61346 / −2.06844 / 0.00729 / 1.58518） |
| 云台不跟手 | `gimbal_enable`、IMU `isCalibrated()`、`setYawWorldTarget` |
| 车斜着窜 | `kSteerReadyAngle` / `align` 门 |

---

## 7. 讲的时候必须遵守的技术约定

| 项目 | 约定 |
|---|---|
| 坐标系 | x 前、y 左、z 上；逆时针为正；舵角 = 轮子朝向与车头夹角 |
| δ | 云台朝向 − 车头朝向；变换用 **R(+δ)**（与 PDF 3.2 不同，测试锁定） |
| 电机量 | `vel()` / `angle()` **已经是输出轴**，不要再除 19.2 |
| 几何 | 轮半径 0.066 m；轮组坐标 (±0.167, ±0.167) |
| 减速比 | 19.2（出厂） |
| 转向 | 4 台 GM6020 全 `kDirRev` + 四个零点偏移 |
| 云台 | yaw 正向偏移 1.60879；pitch 反向偏移 2.91772 |
| 控制周期 | 1ms（TIM6），先 `MainTask()` 后 `CommTask()` |
| 喂狗 | 每板一处，在 `CommTask` 末尾 |
| 模式 | 平移**永远**云台系；只有 `w` 的来源随模式变 |
| Working 条件 | 遥控 + IMU 健康 + 8 台电机在线 + 对板 Working（云台另有 2 台） |

---

## 8. 边界（重要）

- **只读讲解，不要改代码**。发现问题写进对话，由用户决定是否让 DeepSeek Harness 改。
- 一次只讲一个文件；讲完给 2~3 个问题，等回答再往下。
- 用户答错时先肯定对的部分，再指出错在哪。
- 他时间紧：**每次控制在 20~30 分钟能讲完的一小块**，别一次灌完。

---

## 9. 开场白（让用户直接复制给 Claude Code）

> 先读 `docs/10-VSCode代码导览交接.md`，按第 5 节的顺序带我。
> 今天只走站 A 和站 B，每讲完一段，问我 2 个问题。
> 我看不懂的地方会直接说。

**如果只剩 20 分钟**：

> 先读 `docs/10-VSCode代码导览交接.md`，只讲第 6 节"现场排障速查"对应的代码：
> 状态机四个条件、喂狗位置、CanTxGuard。讲完让我复述一遍。
