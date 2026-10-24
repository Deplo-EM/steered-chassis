# 10 · VS Code 代码导览交接单

> **交接对象**：VS Code 里内置的 Claude Code（只读辅助）
> **交接原因**：DeepSeek Harness 能读磁盘上的文件，但**看不到**用户在编辑器里
> 打开了哪个文件、光标停在哪一行，无法"你翻到哪我讲到哪"。
> 前期理论（第 1~4 站）已由它讲完，代码导览交给能在编辑器里看到上下文的你。
>
> **交接时用户的进度**：理论已学完"单电机 / 单轮组 / 底盘逆解 / 云台"，
> 还差"双板协议"和"安全验收"两站；现在要做的是**在 VS Code 里建立整体框架**。

---

## 1. 用户是谁、怎么讲

- 刚开始学编程，**不懂术语**，需要生活类比 + 分层讲解（整体→逐行→底层）。
- **暂时没有硬件**，只能读代码、问问题、跑主机测试。
- 目标：能看懂全部代码，最终能**自己重写**；考核以"调试任务"为核心。
- **必须遵守**（详见 `~/.dsh/AGENTS.md`）：
  1. 每个新术语第一次出现，给齐"全称 / 一句话定义 / 生活类比 / 在本项目哪里用"；
     **中文简称也算术语**（"钳位""限幅""IMU"都要解释）。
  2. 每个物理量说清是"转子端"还是"输出轴"、减速比在哪一层除过。
  3. 讲调试操作要能落地（什么硬件、哪个窗口、什么前提、没硬件怎么办）。
  4. 讲"坑"要分类：参数配置错 / 接线硬件错 / 代码逻辑错。
  5. 不要用"时间计划/工期"式表达，用**关卡式**推进。
  6. 一次只推进一小块，讲完给 2~3 个自查问题。

---

## 2. 用户现在的诉求

> "现在虽然能看到代码但是太散了。"

所以**不要按目录结构讲**，要按**一条指令的旅程**讲：遥控器 → 底盘 → 轮子。
每到一个文件，先说"它在整条链路的哪一环"，再看代码。

---

## 3. 代码结构：一页地图

```
steered_skeleton/            ← git 仓库根
├── RobotComponents/         ← 两块板共用的"零件"
│   ├── component/           ← module_state / pid / gimbal_chassis_comm /
│   │                          fault_injection / diagnostics
│   └── ...
├── Chassis/                 ← 底盘板（4×M3508 驱动 + 4×GM6020 转向）
│   ├── Core/Src/main.c      ← 程序入口（CubeMX 生成，只看 USER CODE 区）
│   ├── Task/component/      ← main_task / comm_task
│   ├── RobotModules/component/ ← robot / chassis / chassis_ik_solver
│   ├── Instance/component/  ← ins_motor / ins_pid / ins_imu / ins_chassis_iksolver
│   ├── src/                 ← HW_can / imu / DT7（DT7 是骨架给的）
│   └── inc/                 ← 骨架给的组件头文件（motor_* / BMI088 / mahony…）
├── Gimbal/                  ← 云台板（结构与底盘板镜像）
├── tests/                   ← 主机测试（不需要板子就能跑）
└── docs/00~09               ← 交付说明、逆解推导、协议、安全、调车、待办
```

**两板是镜像结构**：讲完底盘的某一层，云台那边只要说"同名文件、同样的分层"。

---

## 4. 导览路线（按这个顺序，每站"看到能回答 X 就过"）

### 站 A：入口（10 分钟）
- 打开 `Chassis/Core/Src/main.c`，只看 `USER CODE 2` 区。
- 看：`MainTaskInit()` 在 `HAL_TIM_Base_Start_IT(&htim6)` **之前**。
- 问：如果反过来会怎样？（答案：中断会在对象还没装配好时进来）
- **过关标准**：能说出"程序从 main 开始，1ms 定时器一启动就进入中断驱动"。

### 站 B：装配 + 心跳（15 分钟）
- 打开 `Chassis/Task/component/main_task.cpp`。
- `MainTaskInit()`：电机 → IMU → PID → 逆解 → Chassis → Robot → CommTaskInit。
- `MainTask()`：只有一句 `robot->update()`。
- `HAL_TIM_PeriodElapsedCallback()`：`MainTask()` → `CommTask()` → 记录耗时。
- 问：为什么 `while(1)` 是空的？为什么先控制后通信？

### 站 C：决策层（20 分钟）★最重要
- 打开 `Chassis/RobotModules/component/robot.cpp`。
- `update()` 五步：读传感器 → 输入 → 状态机 → 生成指令 → 回填通信。
- `applyOutput()`：**急停闸门在最外层**，其次 `state_ != Working` 就 `stop()`。
- 问：为什么急停要放在状态机**前面**？
- 问：不进 Working 时电机会怎样？

### 站 D：执行层（20 分钟）
- 打开 `Chassis/RobotModules/component/chassis.cpp` 的 `update()`。
- 四步：读当前舵角 → 逆解 → 两个环 → `setInput`。
- 重点看"对齐门" `kSteerReadyAngle = 0.35f` 和 `cos(err)` 缩放。
- 问：轮子斜着 90° 时为什么不出力？

### 站 E：数学层（15 分钟）
- 打开 `Chassis/RobotModules/component/chassis_ik_solver.cpp` 的 `solve()`。
- 三段：云台系→车体系（R(+δ)）→ 每个轮 `v = vc + ω×r` → 等效解翻转。
- **注意**：δ 的旋转用 **R(+δ)**，与考核说明 3.2 印的 R(−δ) 不同（已用测试锁定，见 docs/07）。
- 问：枪口朝左 90°、推前进，车往哪走？

### 站 F：通信层（20 分钟）
- `Chassis/Task/component/comm_task.cpp`：收 DBUS（校验→解码→通知）、发 CAN、**唯一喂狗点**。
- `Chassis/src/HW_can.cpp`：按 ID 分发、`decode`、通知。
- `RobotComponents/component/gimbal_chassis_comm.cpp`：8 字节帧 + CRC8。
- 问：为什么喂狗放在控制周期末尾，而不是中断回调里？

### 站 G：云台（15 分钟）
- `Gimbal/RobotModules/component/gimbal.cpp`：yaw 用**世界角**、pitch 用**关节角**。
- `Gimbal/RobotModules/component/robot.cpp`：`gimbal_enable`、使能上升沿对齐。
- 问：为什么 yaw 不能用电机自己的关节角？

### 站 H：配置层（10 分钟）
- `Chassis/Instance/component/ins_motor.cpp`：10 台电机的 ID / 方向 / 零点。
- `Chassis/Instance/component/ins_pid.cpp`：两组 PID 参数。
- 问：`setInput(2.0f)` 在 `kCurr` 和 `kRaw` 下分别是什么？

### 站 I：测试当活文档（10 分钟）
- `tests/test_main.cpp`：174 条断言，覆盖 PID、协议、DBUS、逆解、跟随仿真。
- 让用户跑 `./tests/run_tests.ps1`（PowerShell 5.1 或 7 都行）。
- 问：哪一条测试对应"枪口朝左 90°"？

---

## 5. 讲的时候必须遵守的技术约定（讲错会误导用户）

| 项目 | 约定 |
|---|---|
| 坐标系 | x 前、y 左、z 上；逆时针为正；舵角 = 轮子朝向与车头夹角 |
| δ | 云台朝向 − 车头朝向；变换用 **R(+δ)**（与 PDF 3.2 的 R(−δ) 不同，已用测试锁定） |
| 电机量 | `vel()` / `angle()` **已经是输出轴**，不要再除 19.2 |
| 几何 | 轮半径 0.066 m；四轮组坐标 (±0.167, ±0.167)（说明书 6.2） |
| 减速比 | 19.2（说明书 2.3） |
| 转向 | 4 台 GM6020 全 `kDirRev`，零点 2.61346 / −2.06844 / 0.00729 / 1.58518 rad |
| 云台 | yaw 正向、偏移 1.60879；pitch 反向、偏移 2.91772 |
| 控制周期 | 1ms（TIM6），中断里先 `MainTask()` 后 `CommTask()` |
| 喂狗 | 每板只有一处，在 `CommTask` 末尾 |

---

## 6. 当前代码状态

- 分支 `master`，最近提交：`e23d284`（docs/09 待办）、`99e9c44`（按补全版更新参数）。
- 两板编译通过；主机测试 **174 项全过**。
- 已知限制（讲的时候要如实说）：
  - IMU 读数"冻结在合理旧值"检测不到（SPI 驱动丢弃返回值）；
  - 台架故障注入 8 项**尚未执行**（工具已就绪，见 docs/08）；
  - pitch 软件限位 `[-0.9, +0.7]` 是经验值，需实测。

---

## 7. 边界（重要）

- **只读讲解，不要改代码**——用户明确要求这一轮不动代码。
- 发现代码问题：写进对话告诉用户，由用户决定是否让 DeepSeek Harness 改。
- 一次只讲一个文件；讲完给 2~3 个问题，等用户回答再往下。
- 用户答错时**先肯定对的部分**，再指出错在哪、为什么。

---

## 8. 建议的开场白

> "我先带你在 VS Code 里按'一条指令的旅程'走一遍：
> 遥控器 → 底盘板 → 决策 → 逆解 → 两个环 → CAN → 轮子。
> 你先把 `Chassis/Core/Src/main.c` 打开，滚到 `USER CODE 2` 那几行，
> 看完告诉我：程序是从哪一行开始'每 1 毫秒跑一次'的？"
