# DSH PID 调参记录（Pitch 第一轮 A01→B01）

> **当前暂停通电。** 出现两次 Yaw 自转、遥控急停不能停止。以下 PID 记录仅供对照，恢复试车前先完成 [Yaw 停机事件复核](35-Yaw异常转动与急停失效.md)。
> **2026-11-16：** 保留 B14 参数并修复前馈后总力矩越界；最新候选和调试字段含义见 [36 交接复核](36-编号35交接处理与复核.md)。

日期：2026-11-15。底盘交接闸门（`kAccLow` 复测）尚未关闭，本轮只做 **云台 Pitch 一个量的 A/B**，不改底盘、不改 Yaw/舵向。

## 1. 基线 A01

- 来源：当前工作树（HEAD `c73c2b4` + 未提交改动），未改任何 PID。
- ELF：`Gimbal/build/dsh-pid-A01/steered_gimbal.elf`
- SHA-256：`5F01D68B548DC6EF6A06C36792B1FDD8735DE6225AC57C4A59C73CFAB8311F78`（含 `g_gimbal_dbg` 调试结构体，PID 仍为 3.6）
- 标记：`SAFE-BUILD`
- Ozone：`debug/ozone/gimbal_dsh_A01.jdebug`

## 2. 实车数据（A01，1 kHz）

条件：架空、右杆小幅往复、分离档；窗口约 12 s（另有一次 12 ms 的放大视图）。

| 观察 | 数值 |
|---|---|
| 目标/反馈摆幅 | 约 ±0.5 rad，反馈明显滞后、幅值略衰减 |
| 输出力矩 | 0.3–1.2 N·m（限幅 ±3.5 N·m），**未饱和** |
| 积分项 | 0.6–0.8 N·m（上限 1.5），占比大 |
| 大幅动作的极值 | 曾到 −3.27 N·m（93% 限幅），属于大误差段 |

结论：小中幅动作时力矩有余量、响应偏慢 → **可以单独加 P**；大误差段接近饱和，暂不动限幅。

## 3. 本轮唯一改动

`Gimbal/Instance/component/ins_pid.cpp`：

```diff
-constexpr float kPitchPidKp = 3.6f;
+constexpr float kPitchPidKp = 4.2f;   // +17%
```

`Ki=1.0 / Kd=0.20 / 输出 ±3.5 / 积分 1.5` 全部不变；Yaw、舵向、底盘一律不动。

## 4. 候选 B01

- ELF：`Gimbal/build/dsh-pid-B01/steered_gimbal.elf`
- SHA-256：`15FA04DDFEC4922D78BB098474A292666ADB5AB904373A2570BB501C19804D1D`（含 `g_gimbal_dbg`，仅 P=4.2 与 A01 不同）
- 标记：`SAFE-BUILD`
- Ozone：`debug/ozone/gimbal_dsh_B01.jdebug`
- 构建条件：与 A01 同源码、同选项，仅上述一个常量不同（构建前已确认 Gimbal 与 RobotComponents 无其它改动）。

## 5. 主机测试

`pwsh -NoProfile -File tests/run_tests.ps1`：全部通过，0 失败。

其中 `tests/test_robot_safety.cpp` 有一条按旧 P 写死的期望值（0.67–0.78），已同步为 0.80–0.88，测试意图（P 追角、D 不追指令跳变）不变。

## 6. 复测步骤（只换云台）

1. 烧 B01 云台 ELF，底盘保持不动；架空、右下急停、左上分离。
2. 与 A01 相同条件，重复同一组动作：
   - 冷启第一次上/下；
   - 同次第二次上/下；
   - 急停→回中→再解除后上/下。
3. 1 kHz 采样，窗口 ≥ 2 s，记录 `pitch_target_`、`pitch_feedback_`、`output_`、`integral_`。

## 7. 判定标准

| 指标 | 通过条件 |
|---|---|
| 滞后 | 同一动作下，反馈起动相对目标的延迟减少 ≥ 30% |
| 过冲 | 峰值越过不增加超过 5%（或仍 < 10%） |
| 饱和 | 小中幅动作输出仍有余量（< 3.0 N·m） |
| 稳定 | 无持续振荡、无异常噪声 |

通过 → 保留 B01，下一轮再考虑 Yaw（需先采 Yaw 数据）。
不通过 → 回退 A01（`gimbal_dsh_A01.jdebug`），下一轮只改 D 或改目标速率。

## 8. 回退

- 云台 A01：`Gimbal/build/dsh-pid-A01/steered_gimbal.elf`（`EBFBBEB4…`）
- Ozone：`debug/ozone/gimbal_dsh_A01.jdebug`
- 回退只烧云台，不动底盘。

## 9. 变量地图（Ozone Watch 已预置，不用再搜符号树）

| 想看什么 | 表达式 | 单位/含义 |
|---|---|---|
| 授权状态 | `s_robot.state_` | 0=Dead，2=Working |
| 急停 | `s_robot.estop_` | 1=急停中 |
| 输出许可上一拍 | `s_robot.output_enabled_prev_` | 授权跳变时刻 |
| Pitch 目标 | `s_gimbal.pitch_target_` | rad（关节角） |
| Pitch 反馈 | `s_gimbal.pitch_feedback_` | rad（关节角） |
| Pitch 力矩请求 | `s_pitch_pid.output_` | N·m，限幅 ±3.5 |
| Pitch 积分 | `s_pitch_pid.integral_` | N·m，限幅 ±1.5 |
| Pitch 驱动输入 | `s_pitch_motor.torq_input_` | N·m（基类成员，符号树里不展开） |
| Yaw 世界目标 | `s_gimbal.yaw_target_` | rad |
| Yaw 世界反馈 | `s_gimbal.yaw_world_feedback_` | rad |
| Yaw 关节反馈 | `s_gimbal.yaw_feedback_` | rad（不能直接对世界目标） |
| Yaw 力矩请求 | `s_yaw_pid.output_` | N·m |
| Yaw 积分 | `s_yaw_pid.integral_` | N·m |

两个 Ozone 工程（`gimbal_dsh_A01.jdebug` / `gimbal_dsh_B01.jdebug`）已把这些表达式写进 `Watch.Add`，**打开工程直接就能看到值**。

## 10. 调试结构体（2026-11-15 晚追加）

为免去每次翻符号树，固件新增只读快照：

```cpp
robot::diag::g_gimbal_dbg
```

字段：`state / estop / authorized`、`pitch_target / pitch_feedback / pitch_output / pitch_integral / pitch_torq_meas`、`yaw_target / yaw_world_feedback / yaw_output / yaw_integral`。

- 定义：`Gimbal/RobotModules/component/gimbal_debug.hpp`，由 `Gimbal::update()` 与云台 `Robot::update()` 每周期写入；
- 不参与控制，不影响 PID；
- A01（`5F01D68B…`）与 B01（`15FA04DD…`）都含此结构体，两者仍只差 `kPitchPidKp`；
- Ozone 里 Watch 一个 `robot::diag::g_gimbal_dbg`，展开即可；Timeline 选变量时也只在这一项下面找。
- 旧哈希 `EBFBBEB4…` / `F64260DC…` 是不含结构体的版本，已作废。

## 11. 第二轮（B02）：下极限修正（2026-11-15 晚）

B01（P=4.2）三组实车数据（1 kHz）：

| 工况 | 目标范围 rad | 反馈范围 rad | 输出 N·m | 积分 N·m |
|---|---|---|---|---|
| 小幅 | −0.73…+0.56 | **−0.40**…+0.60 | −0.88…+1.14 | 0…0.73 |
| 压底再抬 | −0.90…+0.69 | **−0.50**…+0.59 | **−3.18**…+1.44 | **−1.50**（顶限） |
| 大幅 | −0.90…+0.69 | **−0.50**…+0.59 | −3.18…+1.44 | −1.50…0.73 |

结论：目标压到 −0.9 时，反馈在 **−0.5016** 不再变化；输出顶到 −3.18（限幅 −3.5），
积分被灌到 −1.50 上限。说明**机械下止点约 −0.50，软件下限 −0.9 超出了行程**，
造成饱和 + 积分饱和，上抬时先卸积分 → 又慢又冲。不是 P 的问题。

**改动（唯一一项）**：`kPitchLowerLimit` −0.9 → **−0.47**（留 0.03 余量）。
P 保持 4.2，上极限 +0.7 暂不改（未测到机械上止点）。

- ELF：`Gimbal/build/dsh-pid-B02/steered_gimbal.elf`
- SHA-256：`5DFB271AD9570F12CC62C2BD7C080166B8912B8F9BF47B261A98415A20B0B54C`
- Ozone：`debug/ozone/gimbal_dsh_B02.jdebug`
- 主机测试：全部通过。

**复测重点**：压底保持时，反馈应停在约 −0.47，输出不再顶到 −3.2，积分不再到 −1.5；
上抬时应立即反向出力、无明显迟滞。若确认，再测上止点并决定 P 是否保留 4.2。

## 12. 第三轮（B03）：跟随 + 峰值一起改（2026-11-15 晚，用户要求合并）

时间紧，用户明确要求一次改多个量。B03 在 B02 基础上再改两个 Pitch 参数：

| 参数 | A01 | B01 | B02 | **B03** | 目的 |
|---|---|---|---|---|---|
| Kp | 3.6 | 4.2 | 4.2 | **4.8** | 跟紧目标 |
| Kd | 0.20 | 0.20 | 0.20 | **0.32** | 压峰值（D 走实测转速，无目标跳变冲击） |
| 下极限 | −0.9 | −0.9 | −0.47 | **−0.47** | 消除机械限位饱和与积分灌满 |

- ELF：`Gimbal/build/dsh-pid-B03/steered_gimbal.elf`
- SHA-256：`245FF3B95C883B4B359C9C1CC04C1CC3C2954CE2B66FD3AD570C75C12F75D198`
- Ozone：`debug/ozone/gimbal_dsh_B03.jdebug`
- 主机测试：全部通过（`test_robot_safety` 的 Pitch 期望值 0.80–0.88 已同步为 0.92–1.00）。

**注意**：本轮同时改了 P 和 D，若结果变差，先回 B02 单独判断；
若峰值仍大，下一杠杆是 I（1.0→0.8）或继续加 D；若出现高频抖动，先把 P 降回 4.2。

## 13. 第四轮（B04）：积分加快，削减稳态下垂

B03 实车数据（1 kHz）：

- 小幅动作：反馈紧贴目标，滞后和过冲都很小 → **动态跟随已基本到位**；
- 大幅阶跃：上升快、峰值小；但保持段目标 0.45、反馈约 0.42（差约 **0.08 rad**），
  输出平顶在 **1.3 N·m**，积分从 0.5 慢慢爬到 0.9。

判读：这是**静摩擦/重力托举**造成的稳态下垂，靠积分慢慢攒力矩。不是 P/D 不足。

**改动（唯一一项）**：`kPitchPidKi` **1.0 → 1.6**（积分充电加快）。P=4.8、D=0.32、下极限 −0.47 不变。

- ELF：`Gimbal/build/dsh-pid-B04/steered_gimbal.elf`
- SHA-256：`9F46DCBC155997D8DDEECA971537FB7AA3BEFF2157024ABD4681603B3D8C36A9`
- Ozone：`debug/ozone/gimbal_dsh_B04.jdebug`
- 主机测试：全部通过。

**复测重点**：阶跃后保持 1 秒，稳态误差应从 0.08 rad 降到 0.03 rad 以内；
若出现"粘一下再跳"的极限环或过冲变大，说明静摩擦占主导，下一步改用**重力前馈**（常量或 cos 形），而不是继续加 Ki。

## 14. 第五轮（B05）：按严格指标重新拉增益

用户指出验收标准严格，B04 仍不合格。按严格口径复读 B04 数据：

| 指标 | 严格值 | B04 实测 | 判定 |
|---|---|---|---|
| 保持段稳态误差 | ≤ 0.01 rad | 顶部约 **0.045 rad** | 不合格 |
| 输出余量 | — | 最大约 1.3 N·m（限幅 3.5） | 大量余量未用 |

**改动（三个量一起）**：`Kp 4.8→7.0`、`Ki 1.6→2.5`、`Kd 0.32→0.40`；下极限 −0.47 不变。

- ELF：`Gimbal/build/dsh-pid-B05/steered_gimbal.elf`
- SHA-256：`CF623BBDD2EFB47F78BAC1BF7432C2AAB61CBE7D9DFD259770D6EFEFBD313B5A`
- Ozone：`debug/ozone/gimbal_dsh_B05.jdebug`
- 主机测试：全部通过（Pitch 期望值同步为 1.36–1.44）。

**严格验收口径**：稳态误差 ≤0.01 rad、过冲 ≤3%、稳定时间 ≤150 ms、无持续振荡。

**风险与回退**：P 提高 46%，可能出现高频抖动或过冲变大；出现即回 B04。
若高增益仍压不下稳态偏差，说明静摩擦/重力占主导，下一版改用**重力前馈**（`kG*cos(pitch)`），而不是继续加增益。

## 15. 第六轮（B06）：先保稳定，再谈精度

B05 实车数据：反馈出现约 **20–40 Hz、±0.2 rad** 的抖动（−11.5～−10.5 s 段），
峰值附近也有尖刺。按严格标准，**持续振荡 = 不合格**；P=7.0 / D=0.40 推过头，
D 把实测转速噪声放大进了力矩。

**改动（四个量）**：

| 参数 | B05 | **B06** |
|---|---|---|
| Kp | 7.0 | **5.8** |
| Ki | 2.5 | **2.0** |
| Kd | 0.40 | **0.30** |
| Pitch D 低通 | 0.10 | **0.05**（更强平滑） |

- ELF：`Gimbal/build/dsh-pid-B06/steered_gimbal.elf`
- SHA-256：`CD4009F1779E9FCC411161A8F262719A9A954EA29E1C7D47BF1C0715933DBC3E`
- Ozone：`debug/ozone/gimbal_dsh_B06.jdebug`
- 主机测试：全部通过（Pitch 期望值同步为 1.12–1.20）。

**判定顺序**：先看有没有振荡（有 → 继续退 P/D），再看稳态误差 ≤0.01 rad，
最后看过冲 ≤3%、稳定 ≤150 ms。稳定优先于精度。

## 16. 第七轮（B07）：Pitch 最后一版

B06 已无高频抖动，但阶跃拐角仍有可见滞后。最后一次 Pitch 调整：

| 参数 | B06 | **B07** | 目的 |
|---|---|---|---|
| Kp | 5.8 | **6.6** | 提高带宽、减小拐角滞后 |
| Kd | 0.30 | 0.30 | 不变 |
| Ki | 2.0 | 2.0 | 不变 |
| Pitch D 低通 | 0.05 | **0.08** | 减少相位滞后（B06 已证明不会振荡） |

- ELF：`Gimbal/build/dsh-pid-B07/steered_gimbal.elf`
- SHA-256：`E60E5254FA4CAF84EDC3657BF501B779AE6FCC50DE5A94DAC963C9D4EF38579C`
- Ozone：`debug/ozone/gimbal_dsh_B07.jdebug`
- 主机测试：全部通过（Pitch 期望值同步为 1.28–1.36）。

判定：无振荡、稳态 ≤0.01 rad 即收；若又出现抖动，退回 B06 并接受少量滞后。
Pitch 到此为止，下一环转 Yaw。

## 17. Yaw 第一轮（B08）：先稳定

实车 Yaw 数据：世界目标/反馈在 **+0.7 ～ −2.6 rad** 之间振荡，周期约 **2.5 s**，
幅度持续增长；用户描述为“转着转着突然开始疯转”，且急停未能立即停止（由 Codex 另查）。

判读：这是**发散振荡**，不是跟踪滞后。两个候选源：

1. **Yaw 位置环**欠阻尼（分离档自己打杆也振荡）；
2. **底盘跟随外环**与云台世界稳向耦合（只有跟随档出现，2.5 s 正是外环量级）。

**改动（稳定优先）**：`Kp 2.5→1.8`、`Kd 0.24→0.30`；`Ki 0.2`、`LPF 0.15` 不变。

- ELF：`Gimbal/build/dsh-pid-B08/steered_gimbal.elf`
- SHA-256：`B023237CB3B98F2EC50CC9C8D3B78F10410A282FE49E67F633CE457E3CB31DB8`
- Ozone：`debug/ozone/gimbal_dsh_B08.jdebug`
- 主机测试：全部通过（Yaw 期望值同步为 0.32–0.40）。

**测试前置**：急停通路确认有效后才通电。
**分档判断**：先在分离档打杆——仍振荡则是 Yaw 环；只有跟随档振荡则改跟随外环
（`kFollowKp=4.0`、`kFollowKd=0.15`）。

## 18. Yaw 第二轮（B10→B11）：先压单向偏置，再收滞后

**B10（Kp=2.2、Ki=0.05、Kd=0.28、积分限幅 0.5）实车结果**：

- 单向"疯转"消失，目标/反馈在 −0.1～−2.3 rad 之间双向跟随；
- 稳态贴合（平台段红绿几乎重合）；
- 积分始终很小，没有再积累单向偏置；
- 剩余问题：**阶跃拐角有可见滞后**。

**B11（本版）**：`Kp 2.2→3.0` 收滞后、`Kd 0.28→0.34` 补阻尼；`Ki 0.05`、积分限幅 0.5 不变。

- ELF：`Gimbal/build/dsh-pid-B11/steered_gimbal.elf`
- SHA-256：`F5CA7CFC5529642AADBED76BAA6E1A93256DC607E5A26C625238E9C705DAA3DB`
- Ozone：`debug/ozone/gimbal_dsh_B11.jdebug`
- 主机安全测试：全部通过（99+81 项，0 失败；全量套件仍被 Codex 未提交的 `ins_motor_safety.hpp` include 路径问题挡住，与本轮 PID 无关）。

**判定**：无单向漂移、无持续振荡、阶跃过冲 ≤3%、稳态 ≤0.01 rad。
若又出现高频抖动 → 回 B10（B627DBA9…）。

## 19. Yaw 第三轮（B11→B12）：找稳定边界

**B11（Kp=3.0、Kd=0.34）实车结果**：平台段绿线在 ±0.5 rad 来回抖、青线同步尖刺，
出现高频振荡 → **否决**。B10（Kp=2.2）是稳定基线，稳定边界在 2.2～3.0 之间。

**B12（本版）**：`Kp 2.2→2.5`、`Kd 0.28→0.32`；`Ki 0.05`、积分限幅 0.5 不变。

- ELF：`Gimbal/build/dsh-pid-B12/steered_gimbal.elf`
- SHA-256：`078C0479D2A32787DB4085BF701C06642599557F83A6C4559E06B3C7593874EA`
- Ozone：`debug/ozone/gimbal_dsh_B12.jdebug`
- 主机安全测试：99+81 项，0 失败。

**判定顺序**：先看平台段有没有高频抖动（有 → 回 B10，不再加 P）；
再看阶跃过冲 ≤3%、稳态 ≤0.01 rad。
若 B12 稳定但滞后仍大，下一步不是继续加 P，而是**目标速率前馈**（用 `yaw_rate_cmd`）。

## 20. Yaw 第四轮（B13）：前馈替代加 P

**B12（Kp=2.5、Kd=0.32）实车结果**：平台段仍出现成片高频抖动 → **否决**。
结论：Yaw 的 P 上限就是 2.2（B10），再靠加 P 收滞后会牺牲稳定性。

**B13（本版）**：P 回到 B10 的 2.2，新增**速率前馈**：

```cpp
yaw_input += kYawRateFfGain * yaw_rate_ff_;   // kYawRateFfGain = 0.20 N·m/(rad/s)
```

- 前馈输入 = 底盘下发的 `command.yaw_rate`（满杆约 2 rad/s → 0.4 N·m）；
- 目的：用已知的期望转速直接补力矩，减小跟随滞后，**不再动 P**；
- 符号与 PID 输出一致（正速率 → 正力矩），已在主机测试中确认（同一用例命令从 −2.4 变为 −2.2）。

- ELF：`Gimbal/build/dsh-pid-B13/steered_gimbal.elf`
- SHA-256：`B31241190F81F49CD4BF0EAB64FFF966D93ABB50A40F727623396576C480C0F8`
- Ozone：`debug/ozone/gimbal_dsh_B13.jdebug`
- 主机安全测试：99+81 项，0 失败。

**判定**：平台段无抖动（保持 B10 的稳定）、阶跃滞后明显减小、无额外过冲。
若过冲变大 → 前馈 0.20→0.10；若滞后仍大且稳定 → 0.20→0.30。

## 21. Yaw 第五轮（B14）：前馈加到 0.30

B13（前馈 0.20）实车：平台段贴合、无单向漂移、无高频振荡，拐角仍有圆角滞后。

**B14（本版）**：只把 `kYawRateFfGain 0.20→0.30`（满杆约 2 rad/s → 0.6 N·m）；
Yaw PID（2.2/0.05/0.28、积分限幅 0.5）与 Pitch B07 全部不动。

- ELF：`Gimbal/build/dsh-pid-B14/steered_gimbal.elf`
- SHA-256：`EFC5ACC3DDA8E59AB4162E50E1D6058A1CCE01711C4E6C640C5F624B800B8393`
- Ozone：`debug/ozone/gimbal_dsh_B14.jdebug`
- 主机安全测试：99+81 项，0 失败。

**判定**：拐角滞后继续减小且不出现过冲即为收；若过冲 → 回 B13（前馈 0.20）。
P 仍封顶 2.2，这是 B11/B12 两次振荡换来的边界。

## 22. 下一步：底盘两个环（转向角、轮速）

Yaw 收尾后转底盘，两个环分别调：

| 环 | 参数 | 观察量 |
|---|---|---|
| 转向角 | `kSteerPidKp/Ki/Kd` = 10000/1000/600（电流版） | `s_steer_frame`、`s_chassis.steer_feedback_[i]`、`s_solver.command_[i].angle/desired_angle` |
| 轮速 | `kWheelPidKp/Ki/Kd` = 0.35/1.5/0 | `s_chassis.wheel_velocity_`、`s_wheel_pids[i].output_`、`align_blocked_mask_` |

顺序：先转向角（它决定对齐门何时放行），再轮速。
需要的数据：单次阶跃（转向给一个 0.3 rad 目标、轮速给 1 m/s 目标）的 1 kHz 曲线。













若某一行显示 not found（多半是 `s_pitch_motor.torq_input_` 的匿名空间问题），先展开兜底的 `s_gimbal` → `pitch_motor_` 指针；仍不行就告诉我，我加一个固件调试结构体，把所有量收进一个符号。

