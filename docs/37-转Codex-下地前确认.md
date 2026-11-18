# 转 Codex：四轮版下地前确认

2026-11-16 晚。用户准备下地测试，请 Codex **判定当前四轮版是否具备下地条件**，并列出否决项或前置条件。
不需要复述历史，本文自包含；现场只有用户一人，不要再安排队友/上位机/抓包类步骤。

## 一、要判定的候选

| 板 | ELF | SHA-256 | 说明 |
|---|---|---|---|
| 底盘（四轮，待下地） | `Chassis/build/dsh-reorder-4w/steered_chassis_steer_current_field.elf` | `3B3249200E1AA9DE129E7AC86D705EFE96A2A290022484C2221B8BEF4FB7248A` | 本轮改动 + 四轮全开 |
| 云台（配套） | `Gimbal/build/steer-handoff-paired-review/steered_gimbal.elf` | `17BE54FC5BA38702588A8E8F363AEF427D5A2BE47EE71411E6568DFFF0B6896D` | 保持 B14 参数 + 你上轮的安全修补 |

对照版（不用于下地）：

| 用途 | ELF | SHA-256 |
|---|---|---|
| 排除右后（索引 2） | `Chassis/build/dsh-reorder-excl2/...` | `4F6D3F1B31F2DD4BDBAA346D8857F56A369115032AD911A696BE304A607B0642` |
| 关闭翻转（已否决） | `Chassis/build/dsh-noflip-excl2/...` | `E95A22EB8AA26CCAB35A9E16FFF70B9AFE4CFB1123783217CF601AB65594FC17` |

主机安全测试：底盘 107+107+95 项、0 失败；两份 ELF 均 `SAFE-BUILD`。

## 二、本轮实车结论与改动

### 1. 电机 ID 与解算器轮位相差 180°（已实车确认）

两次"失能试验"（`ROBOT_EXCLUDED_WHEEL` 逐位排除）结果：

| 排除索引 | 实际停住的物理轮 |
|---|---|
| 2 | 左前 |
| 1 | 右前 |

由此推出实车映射：**ID1=右后、ID2=右前、ID3=左前、ID4=左后**。
解算器假设 LF/LB/RB/RF 对应 ID1/2/3/4，两者相差 180°。

**本轮改动**（`Chassis/Instance/component/ins_motor.cpp`）：

```cpp
kWheelId = {3, 4, 1, 2};   // LF=ID3, LB=ID4, RB=ID1, RF=ID2
kSteerId = {3, 4, 1, 2};
```

`kSteerDir`、`kWheelDir` 不变（四轮同向）；`kSteerAngleOffset[4]` **数组内容未动**。

### 2. 实车表现（重绑后）

- 慢速圆周转左杆：舵角跟随平滑，不再 ±π 大摆；
- 快速圆周转左杆：出现接近 **±180° 的突然跳转**（等效解翻转，`ROBOT_STEER_NO_FLIP` 关闭即恢复）；
- 用户明确要求：**不能关闭翻转**，所以快转跳变作为已知限制保留。

### 3. 右后轮机械卡死（刮蹭）

- 右后舵轮在一个固定角位置被机械刮蹭卡住，无法对齐；
- 手动把它转离刮蹭点后可对齐；
- 四轮版下地时它必须能过那个点，否则会触发堵转/对齐保护。

### 4. 轮速环

`kWheelPidKp/Ki` 由 0.35/1.50 改为 **0.50/0.30**（空载 ±30 rad/s、0.5 Hz 极限环）。
改后实车曲线：目标与实测基本重合、输出 ±2 A，轮速环已稳定；但**未在四轮落地负载下验证**。

### 5. 之前的 Dead 记录

- 一次 `last_dead_.causes = 8`（IMU），发生在推左杆时；`last_imu_dead_.imu.reason` 未读；
- 另一次被用户更正：不是 Dead，是右后卡死导致对齐门不放行（Working）。

## 三、未验证项与风险（请逐条给结论）

| # | 问题 | 风险 |
|---|---|---|
| 1 | `kSteerAngleOffset[4]` 是按**轮位**还是按**电机 ID** 给的？ | 若按 ID，重绑后四个舵角零点全错，下地会跑偏/原地打转 |
| 2 | 重绑后 `kWheelDir` 全 `kDirFwd` 是否仍成立？ | 若某轮安装方向不同，前进指令会让该轮反转 |
| 3 | 逆解几何 `(±0.167, ±0.167)` 与重绑后的物理轮位是否一致？ | 几何错位会让平移/自转方向整体错 |
| 4 | 快转 ±180° 跳变在落地负载下是否会更严重？ | 可能触发堵转/过流/失控 |
| 5 | 右后刮蹭轮在四轮全开时会不会持续顶住、发热或触发保护？ | 烧舵机/停机 |
| 6 | 急停与独立断动力是否已验证？ | 上轮 Yaw 自转无法靠遥控急停停止，尚未闭环 |
| 7 | 轮速环 0.50/0.30 在落地负载下是否仍稳定？ | 空载调参不能替代负载验证 |
| 8 | 上次 `causes=8` 的 IMU 判废是否还会在驱动振动时出现？ | 行驶中突然停机 |

## 四、请 Codex 给出的结论（三选一）

1. **可以下地**：附前置条件清单与首跑限制（速度/模式/场地）；
2. **有条件下地**：列出必须先做的验证项（如零点标定、方向台架检查）；
3. **不可下地**：给出否决的具体缺陷与修复建议。

请特别确认第 1、2、3 条——这三条决定"车往哪走、轮子朝哪转"，是下地前必须闭环的配置问题。

## 五、现场已具备的条件

- 两板可烧录（Ozone + J-Link），四轮可架空；
- 平地约 2 米、可扶车；
- 用户一人操作，无队友；
- 底盘电流配置：CAN1 舵帧 `0x1FE`、±1.2 A；云台 B14 参数未动。

## 六、构建命令（复现用）

```powershell
cmake -S Chassis -B Chassis/build/dsh-reorder-4w -G Ninja -DCMAKE_BUILD_TYPE=Debug `
  -DROBOT_STEER_CURRENT_FIELD=ON -DROBOT_STEER_MODE_PROBE=OFF -DROBOT_EXCLUDED_WHEEL=-1 `
  "-DCMAKE_RUNTIME_OUTPUT_DIRECTORY:PATH=$PWD/Chassis/build/dsh-reorder-4w"
cmake --build Chassis/build/dsh-reorder-4w
```

Ozone 工程：`debug/ozone/chassis_dsh_reorder_4w.jdebug` 与 `debug/ozone/gimbal_steer_handoff_review.jdebug`。
