# 转 Codex：摩擦特调进展与下地阻塞

2026-11-16 夜。接 [37-转Codex-下地前确认](37-转Codex-下地前确认.md)。
用户已按 37 的清单在台架试跑，结果与新增改动如下。请 Codex **复核配置一致性、评审摩擦补偿方案、给出下地结论**。
现场仍只有用户一人；不要安排队友/上位机/抓包。

## 一、当前候选与构建开关

### 1. 已下发的三个构建

| 用途 | ELF | SHA-256 |
|---|---|---|
| 四轮基准（无前馈、无排除、翻转开） | `Chassis/build/dsh-reorder-4w/...` | `3B3249200E1AA9DE129E7AC86D705EFE96A2A290022484C2221B8BEF4FB7248A` |
| 四轮 + 静摩擦前馈 0.25 A（堵转开） | `Chassis/build/dsh-friction-B01/...` | `3FBAD99A1502C24C363C6CBFD3D712AC1EBE64F435CD35A934386F3A5CD6BADF` |
| 台架专用：前馈 0.25 A + **关堵转** | `Chassis/build/dsh-friction-nostall-B02/...` | `8A7DC0974C89E2A9A50447517AE82496D3325952AE2265D480C1DCBDF060C880` |

对照：排除右后（索引 2）`4F6D3F1B…`；关闭翻转 `E95A22EB…`（用户已否决，不再使用）。

### 2. 新增的构建开关（默认全部关闭）

| 开关 | 作用 | 默认 |
|---|---|---|
| `ROBOT_EXCLUDED_WHEEL`（-1..3） | 排除某轮的角度判废/对齐门/堵转/输出 | -1 |
| `ROBOT_BLOCKED_WHEEL_FALLBACK` | 对齐超时后逐轮放行 | OFF |
| `ROBOT_STEER_NO_FLIP` | 关闭等效解翻转 | OFF |
| `ROBOT_STEER_FRICTION_COMP` | 舵向静摩擦前馈 | OFF |
| `ROBOT_STEER_BENCH_NO_STALL` | 关闭舵向堵转锁存（台架专用） | OFF |

## 二、已完成并实车验证的配置变更

### 1. 电机 ID 重绑（37 的核心问题）

两次失能试验确认实车映射：**ID1=右后、ID2=右前、ID3=左前、ID4=左后**，与解算器相差 180°。
`Chassis/Instance/component/ins_motor.cpp`：

```cpp
kWheelId = {3, 4, 1, 2};   // LF=ID3, LB=ID4, RB=ID1, RF=ID2
kSteerId = {3, 4, 1, 2};
```

`kSteerAngleOffset[4]` **内容未动**、`kSteerDir`/`kWheelDir` 未动。

**实车结果**：慢速圆周转左杆，舵角跟随平滑，不再 ±π 大摆 → 重绑方向正确。

### 2. 快转 ±180° 跳变

- 重绑后快速圆周转左杆，出现接近 ±180° 的突然跳转；
- 判定为等效解翻转（目标领先实测 >90° 时换支）；
- 用户明确要求**不能关闭翻转**，故该跳变作为已知限制保留。

### 3. 右后机械卡死（刮蹭）

- 右后（重绑后索引 2）在一个固定角位置被机械刮蹭卡住；
- 手动转离刮蹭点后可对齐；
- 四轮全开时它会把整组对齐门拦住（`drive_ready=0`，四轮电流全零）；
- 之前"转着转着就停了"= 堵转锁存（输出饱和 + 500 ms 不动），不是新故障。

### 4. 轮速环

`kWheelPid 0.35/1.50 → 0.50/0.30`；空载实车目标与实测基本重合、输出 ±2 A。**未做落地负载验证**。

## 三、本轮新增的摩擦补偿（待评审）

`Chassis/RobotModules/component/chassis.cpp`，在舵角 PID 之后叠加：

```cpp
当 |误差| > 0.06 rad 且 |舵速| < 1.0 rad/s：
  steer_request_raw += 0.25 A × (20000 / 1.2)   // 方向随误差符号
```

目的：轮子卡在刮蹭点、转速接近零时，额外给 0.25 A 帮它越过静摩擦。

**尚未取得有效数据**：第一次试跑被堵转锁存打断；台架关堵转版（B02）刚下发，用户还没回报结果。

## 四、未决问题（请 Codex 逐条确认）

| # | 问题 | 现状 |
|---|---|---|
| 1 | `kSteerAngleOffset[4]` 是按轮位还是按电机 ID？ | 未确认；若按 ID，重绑后四个零点全错 |
| 2 | 重绑后 `kWheelDir` 全 `kDirFwd` 是否成立？ | 未确认 |
| 3 | 逆解几何与重绑后的物理轮位是否一致？ | 慢转流畅是间接证据，未做方向实测 |
| 4 | 0.25 A 前馈是否安全、是否会与堵转/对齐门冲突？ | 未评审 |
| 5 | 台架关堵转版长时间顶住的热风险？ | 用户已知，单次不超过 10–15 s |
| 6 | 正式固件最终配置：翻转开、堵转开、前馈取值、是否排除右后？ | 未定 |
| 7 | 急停/独立断动力是否验证？ | 仍未闭环（上轮 Yaw 自转事件） |

## 五、请 Codex 做三件事

1. **复核第 1–3 条配置一致性**（零点、方向、几何）——这决定车往哪走；
2. **评审摩擦补偿方案**：0.25 A 常值前馈 + 0.06 rad 死区 + 1 rad/s 速度门限，是否合理；堵转锁存恢复后能否共存；给出建议取值或替代方案；
3. **给出下地结论**：可以 / 有条件下地（列前置项）/ 不可下地（列缺陷）。

## 六、安全红线

- 台架关堵转版（`8A7DC097…`）**绝不能上赛场**；正式固件必须 `ROBOT_STEER_BENCH_NO_STALL=OFF`；
- 急停未闭环前不落地；
- 右后刮蹭轮不解决，四轮全开会被对齐门拦住；排除右后则车会偏。

## 七、复现命令

```powershell
cmake -S Chassis -B Chassis/build/dsh-friction-nostall-B02 -G Ninja -DCMAKE_BUILD_TYPE=Debug `
  -DROBOT_STEER_CURRENT_FIELD=ON -DROBOT_STEER_MODE_PROBE=OFF -DROBOT_EXCLUDED_WHEEL=-1 `
  -DROBOT_STEER_FRICTION_COMP=ON -DROBOT_STEER_BENCH_NO_STALL=ON `
  "-DCMAKE_RUNTIME_OUTPUT_DIRECTORY:PATH=$PWD/Chassis/build/dsh-friction-nostall-B02"
cmake --build Chassis/build/dsh-friction-nostall-B02
```

Ozone：`debug/ozone/chassis_dsh_friction_nostall_B02.jdebug`、`debug/ozone/gimbal_steer_handoff_review.jdebug`。
