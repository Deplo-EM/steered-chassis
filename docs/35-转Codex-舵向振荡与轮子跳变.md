# 转 Codex：舵向高频振荡、轮子跳变与正常操作进 Dead

整理：2026-11-15 深夜。写给 Codex 接手排查，自包含，不需要读之前的对话。
现场由用户一人操作，没有队友协助，不要再安排"找队友/抓包/装上位机"的步骤。

> **2026-11-16 已接手。** 修复、离线证据和新候选见 [36 交接复核](36-编号35交接处理与复核.md)。B01 自带角度归一化，下面「驱动未归一化」是当时的待验假设。整车暂停通电。

## 一句话

底盘舵角环在**降低增益后仍然高频振荡**（输出 ±8k raw 翻符号），**正常操作就进 Dead**；
用户还观察到**轮子不是连续转，而是非常快地跳变**，即使左摇杆做圆周运动也一样。

## 当前固件（磁盘上的候选，未入库）

| 板 | 用途 | ELF | SHA-256 前缀 |
|---|---|---|---|
| 云台 | 最终调参候选 | `Gimbal/build/dsh-pid-B14/steered_gimbal.elf` | `EFC5ACC3DDA8` |
| 底盘 | 舵角降增益 + 调试结构体 | `Chassis/build/dsh-steer-B01/steered_chassis_steer_current_field.elf` | `B59C0AA1A3F1` |
| 底盘 | 上一版（对照） | `Chassis/build/imu-sample-fix/steered_chassis_steer_current_field.elf` | `50DC591F03DA` |

- 仓库 `D:/ForTest/steered_skeleton/steered_skeleton`，HEAD `c73c2b4`，**工作区有大量未提交改动**（你的 + 我的）。
- 两个 Git 构建开关：底盘 `-DROBOT_STEER_CURRENT_FIELD=ON`；探针 `-DROBOT_STEER_MODE_PROBE=ON`。
- 两个新调试结构体（只读、不参与控制）：
  - 云台 `robot::diag::g_gimbal_dbg`（`Gimbal/RobotModules/component/gimbal_debug.hpp`）
  - 底盘 `robot::diag::g_chassis_dbg`（`Chassis/RobotModules/component/chassis_debug.hpp`）
- Ozone：`debug/ozone/chassis_dsh_steer_B01.jdebug`、`debug/ozone/gimbal_dsh_B14.jdebug`。

## 现象与证据

### 1. 舵角高频振荡（降增益无效）

- `steer_output[0]` 在 **+8k ～ −12k raw** 之间高频翻符号（峰值周期几毫秒）；
- `steer_target / steer_desired / steer_feedback` 跟着一起跳；
- 已把电流版增益从 `Kp=10000 / Ki=1000 / Kd=600` 降到 **`6000 / 400 / 120`**，振荡依旧；
- 旧假设"D 项放大转速噪声"（Kd×kMaxDampingRate≈12000）**已被这次实验否定**。

### 2. 正常操作就进 Dead

- 右杆中位、只动左杆（平移）时也会进入 `Dead`；
- 尚未读到 `s_robot.last_dead_.causes` 与 `s_chassis.last_steer_stall_`（用户现场时间紧）。

### 3. 轮子"跳变"而不是连续转动（用户新观察）

- 左摇杆做**圆周运动**时，轮子不是平滑滚动，而是**非常快地一跳一跳**；
- 这与"舵角在 ±π 表示之间来回翻"或"对齐门高频开合"高度吻合：
  - 舵角翻 → 驱动门 `align_blocked_mask` 反复置位/清零 → 轮电流被反复清零/放行 → 轮子跳。

### 4. 一个硬异常

`g_chassis_dbg.steer_feedback[0] = 26.65 rad`。

- 舵角按代码应被归一到 `[−π, π)`（`ins_motor.cpp` 里 `opt.angle_range = AngleRange::kNegPiToPosPi`）；
- 26.65 rad ≈ 4.2 圈，**说明反馈没有归一化**；
- 若反馈是多圈连续角，逆解、等效解翻转、`desired_angle` 与伺服目标都会跟着漂/跳，能同时解释"振荡"和"轮子跳变"。

## 重点怀疑（按可能性排序）

1. **舵角反馈未归一化**：`steers_[i]->angle()` 在电流版/当前构建下返回多圈连续角；
   查 `Gimbal/Chassis` 两处 `motor_GM6020.cpp` 的 `angle()`、`AngleRange` 处理、`decode()`，
   以及 `Chassis/RobotModules/component/chassis.cpp::captureFeedback()` 里 `steer_feedback_[i] = steers_[i]->angle()`。
2. **等效解在 ±π 附近来回翻**：`chassis_ik_solver.cpp` 的 `desired_angle = reference + diff` 与 `angle += π` 分支，
   配合未归一化的反馈会每周期翻一次，导致舵角目标跳变。
3. **停滞保护误触发**：`checkSteerProgress()`（输出饱和 + 角度不进展）在振荡下反复触发 → `stop()` → Dead。
4. **对齐门高频开合**：`align_blocked_mask_` 与轮电流的联动，造成"轮子跳"。

## Codex 需要查的具体位置

| 位置 | 查什么 |
|---|---|
| `Chassis/Instance/component/ins_motor.cpp` | 舵机 `opt.angle_range`、`opt.dir`、`angle_offset` 是否真的生效 |
| `Chassis/src/motor_GM6020.cpp` | `decode()` 后 `angle()` 的取值范围；是否按 `angle_range` 归一 |
| `Chassis/RobotModules/component/chassis.cpp` | `captureFeedback()` 的 `steer_feedback_` 是否应再 `WrapPi` |
| `Chassis/RobotModules/component/chassis_ik_solver.cpp` | `reference`/`desired_angle`/`diff` 在 ±π 附近的连续性 |
| `Chassis/RobotModules/component/chassis.cpp` | `checkSteerProgress()` 的触发条件与 `steer_stall_mask_` |
| 运行时 | `s_robot.last_dead_.causes`、`s_chassis.last_steer_stall_`、`g_chassis_dbg.align_blocked_mask` |

## 现场安全

- 用户报告过**急停不能立刻停住**（另案，Codex 已在查）；**在该问题确认前不要再安排通电试车**。
- 如果必须复现：车架固定、四轮离地、能直接切动力、右下急停待命。
- 不再要求用户装 RoboMaster Assistant、抓 CAN、找队友。

## 我这边改过什么（便于 Codex 定位 diff）

- 云台：Pitch `6.6/2.0/0.30`、下极限 `−0.47`；Yaw `2.2/0.05/0.28` + 速率前馈 `0.30`；新增 `g_gimbal_dbg`。
- 底盘：舵角 `6000/400/120`（仅电流版）；新增 `g_chassis_dbg`。
- 测试：`tests/test_robot_safety.cpp` 的几处 golden 值随增益同步更新；安全测试 99+81 项 0 失败。
- 主机全量套件当前被 `ins_motor_safety.hpp` 的测试 include 路径挡住（Codex 未提交改动引入），与本轮 PID 无关。

## 建议顺序

1. 先回答"`steer_feedback_` 为什么是 26.65 rad"——归一化修好之前，调 PID 没有意义；
2. 再确认 `last_dead_.causes` 是不是 1（舵停滞）；
3. 最后才回到增益/前馈。
