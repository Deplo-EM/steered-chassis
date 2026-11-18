# 给 DSH：云台 PID 整定与实车动态验收交接

整理日期：2026-11-15。项目是 RoboMaster 舵轮机器人电控考核。**交接状态：暂停。** 两次出现 Yaw 自转、拨急停不能停止，整车保持断电，先按 [Yaw 停机事件复核](35-Yaw异常转动与急停失效.md)处理。DSH 保留已有记录和候选；恢复条件满足后才使用下文调参步骤。底盘 `kAccLow` 另行记录，完整回归仍须在整车落地与最终验收前通过。

## 0. 先准备空载云台 PID

1. 核对工作树、实际烧录的两板 ELF 与 SHA-256（未知的写明）、原版参数和回退文件。底盘固件保持现用版；第一轮用原云台固件录基线。
2. **只测空载云台。** 通电短测前确认车体固定、四轮离地、轮组不碰支架、急停有效且能切断动力，条件不齐就先做离线部分。两板就绪后，在左上分离档、左摇杆与拨轮居中，只小幅试右摇杆。先采 Pitch 冷启首次、同次第二次及重新解除急停，再采 Yaw 松杆后的回摆。分清授权、目标和反馈，再决定改哪个环。
3. **一次只改云台的一个量。** 保留另一板和原 ELF；交付可复核的代码差异、构建与测试结果、候选 ELF 与 Ozone 工程路径和 SHA-256、复测和回退步骤。遇见新的停机、复位、非指令动作或明显晃动，立即停试留证，交回 Codex。由用户操作实车。
4. 云台整定后，按[现场总计划第 2 节](31-现场调试与考核收尾计划.md#2-稳定台架复测验证底盘停机问题)补底盘回归。正常操作期间应无异常 `Dead`、无被迫重新授权、无意外复位；记录实际两板版本、每项结果与重复次数。底盘舵角、轮速的通电整定和整车落地等这一步通过；跟随外环最后评估。

## 1. 仓库、考核、版本

- 仓库：`D:/ForTest/steered_skeleton/steered_skeleton`，Windows PowerShell。本机有 CMake、Ninja、`arm-none-eabi-gcc`、MinGW `g++` 和 PowerShell 7。烧录与调试用 SEGGER Ozone + J-Link SWD，目标 `STM32F407IG`。底盘、云台分别编译。
- 交接时分支 `master`、`HEAD c73c2b4`，**工作区有许多未提交和未跟踪的文件**。当前 IMU、控制、取证和调试配置都包含工作区改动，不能只用该提交重建最新候选。先看 `git status --short`、相关差异和现有文件；不要清工作区、重置或用旧树覆盖。若 DSH 在另一空间，需当前工作树与用于对照的 ELF。
- 正式依据：`D:/ForTest/Hello World 电控组步兵（舵轮）考核说明（已补全）.pdf`。第 6–7 页是三模式与坐标，第 8–9 页是 PID、两轴和验收。基础包括三档、云台两轴、方向与稳定性、急停/失联、双板断链和 IWDG。唯一选定的加分项是 **PID 整定记录文档**，做 Yaw 前后实测对比即可准备材料。截止和现场安排以考核方通知为准。
- 最新入口：[现场总计划](31-现场调试与考核收尾计划.md)、[PID 整定和动态验收](32-云台PID整定与三模式动态验收.md)、[底盘 IMU 采样修复](30-BMI088采样修复与现场回归.md)。`docs/10`、`docs/21` 的“云台不动”“单轮滚动”是历史排查记录，不是当前状态。

以下是**磁盘上**的文件，不能凭标题栏或哈希推断板上跑的是哪版。表内相对于仓库根目录，SHA-256 取前 12 位供快速核对。

| Ozone 工程或用途 | ELF | SHA-256 前缀 | 如何用 |
|---|---|---|---|
| [chassis_imu_sample_fix.jdebug](../debug/ozone/chassis_imu_sample_fix.jdebug) | `Chassis/build/imu-sample-fix/steered_chassis_steer_current_field.elf` | `50DC591F03DA` | 最新底盘电流候选，含采样修复与 `last_imu_dead_`，离线通过；实车捕获 `kAccLow`，排除台架问题后待重测。 |
| [chassis_imu_stop_trace.jdebug](../debug/ozone/chassis_imu_stop_trace.jdebug) | `Chassis/build/imu-stop-trace/steered_chassis_steer_current_field.elf` | `78EE5F512CA9` | 有历史 Ozone 截图的旧取证版；无独立 `last_imu_dead_`，不含最新采样修复。 |
| [chassis_separate_release.jdebug](../debug/ozone/chassis_separate_release.jdebug) | `Chassis/build/separate-release/steered_chassis_steer_current_field.elf` | `B9DFAC5C2313` | 含分离松杆方向修正，曾在现场偶发 `Dead`，仅作对照或有据回退。 |
| [chassis_steer_current_field.jdebug](../debug/ozone/chassis_steer_current_field.jdebug) | `Chassis/build/steer-current-field/steered_chassis_steer_current_field.elf` | `0CF05C0AEDB6` | 较早电流版，实际用过哪版以记录为准。 |
| [gimbal.jdebug](../debug/ozone/gimbal.jdebug) | `Gimbal/build/steered_gimbal.elf` | `C749BB340DC7` | 旧默认云台 ELF，与当前源码不能画等号。 |
| [gimbal_axis_tune.jdebug](../debug/ozone/gimbal_axis_tune.jdebug) | `Gimbal/build/axis-tune/steered_gimbal.elf` | `B00B10E58FB7` | 之前的整套轴调候选，增益和 D 的算法都不同于默认版，仅能整体比较。 |
| 云台构建验证 | `Gimbal/build/imu-sample-fix-verify/steered_gimbal.elf` | `66AA9EB2F48E` | 当前云台采样修复的离线构建产物，没有实车记录，不自动替换现用云台。 |

**云台第一轮录基线保留现用底盘和原云台。** 从烧录记录核对目前云台 ELF，未知写“未知”。若要证明是一个 PID 数值的影响，A/B 必须同源码、同构建选项，仅该数值不同；不能把旧云台 ELF 与当前工作树全量重编的差别归给一个 P。

## 2. 硬件和操作约定

- 两板各一颗 BMI088，TIM6 控制周期 1 ms，板间状态常规约 5 ms 更新，关键状态变化可提前发送。以**最后的修正拓扑**和 [board_wiring.hpp](../RobotComponents/component/board_wiring.hpp) 为准：底盘 CAN1 接 GM6020；底盘 CAN2 接 M3508、Yaw 和云台 CAN1；Pitch 接云台 CAN2。红叉删掉的是旧的云台 CAN1→Pitch 支路，板间 CAN1↔CAN2 保留。接收机接云台 USART3，原始遥控分帧经 `0x212–0x214` 转发；板间状态 `0x210/0x211`。使用转发来源时底盘本地 DBUS 计数为 0 可以正常。
- 四组轮按左前、左后、右后、右前，代码中两类电机均为设备 ID 1–4。已用于实车测试的底盘候选走 GM6020 电流模式：CAN1 控制帧 `0x1FE`，软件限制 ±1.2 A。默认构建可走电压模式，二者不同。不要看见橙灯就改总线或电机参数。底盘轮半径 0.066 m、轮组坐标 ±0.167 m，沿用补全版和当前程序。
- Yaw ID 1、Pitch ID 2，反馈 `0x11/0x12`。修正图标注 DMJ620，补全版和代码使用 DM-J4310；当前 PID 工作不靠猜测改型号、驱动、MIT 量程或扭矩限制。组文件已有的配置不反复向用户确认；只有与实测冲突才针对性核对。
- 左拨杆 **上分离／中跟随／下小陀螺**；右拨杆 **下急停／中试动**。左杆平移，右杆左右是 Yaw、上下是 Pitch，拨轮方向测试时先居中。分离不自动追云台，允许拨轮手动转底盘。跟随让车头朝云台靠近。小陀螺拨轮居中仍有非零自旋（当前最小目标 1.5 rad/s），靠急停或退出档位停自旋。**三档的平移都以云台朝向为前；模式互斥。** 自旋不要求车头同时对准云台。Yaw 保持相对开机姿态的世界朝向；Pitch 是关节角闭环，不保证车体颠簸时的世界俯仰不动。

## 3. 实车历史以及尚未解决的事

| 已看到的现象 | 证据和边界 |
|---|---|
| 早期云台不动／`Dead ↔ Resurrection`，舵轮不动或单轮转 | 先前处理过陀螺 DRDY 漏采与舵向帧/对齐问题，后来用户实见分离、跟随、小陀螺能动作。这些是历史基线，不能给新版本签字。 |
| 偶发底盘 `Dead` | 旧底盘 `separate-release`：分离档两杆同动、单动左杆、单动右杆均出现，跟随档单左也出现。对应快照均 `causes=8`、`uwTick` 稳定增长：底盘 IMU 条件失效，这几次不是复位。云台有保持 `Working` 但不能操作的情况，因为它仍需要底盘授权。采样数值拒绝 23、新鲜度拒绝 9 为累计计数，不能对到某个故障。 |
| 底盘 IMU 新候选 | 主机复现并修复了晚到加速度就绪位被旧驱动清掉、陀螺 FIFO 为空时未及时判废新坏加速度。20 ms 新鲜度、急停、重新授权和看门狗维持。2026-11-15 新版在 `98 017 ms` 记录了 `kAccLow`，实测样本模长约 `0.77 m/s²`。现场架空车体有明显晃动或碰擦，静置后数值拒绝计数未增；物理原因仍待区分。 |
| 交接后的调参任务 | **Yaw 及舵向过冲明显、首次上电后的 Pitch 竖直响应迟滞。** 尚缺与实际云台 ELF 配对的目标/反馈/力矩曲线；未确定原因。用户还要求小陀螺前后左右与反向时方向准确稳定、分离/自旋时云台保持、退出自旋进跟随后稳定收敛。 |

再遇 `Dead`：右下急停，仍主动出力时优先切驱动力。安全时保留控制板供电，用匹配 ELF 只读查看 `s_robot.last_dead_.causes`、`tick_ms` 和 `uwTick`。位值 1=舵停滞、2=CAN 发送、4=遥控、8=底盘 IMU、16=电机、32=对板、64=未授权、128=非法状态，可叠加。含 8 再看新版的 `s_robot.last_imu_dead_.imu.reason`；量级异常抄 `last_rejected.accel_mps2[0..2]`，过期抄各 `age_ms`。用 `tick_ms` 区分新旧；复位会清 RAM。`uwTick` 反复归零才另看 `g_reset_by_iwdg`。停机保护问题先排查，不通过调 PID 掩盖。

**调参中遇到意外，交回 Codex 排查。** 若底盘再次锁死，或出现新复位、动作不受控、急停异常，先按上面的方式停试，暂停继续改参数。保留最后一次正常和首次出问题的 ELF、SHA-256 与对应源码差异；写清是哪个档、拨杆和摇杆做了什么、哪块板先异常，以及可安全取得的状态和快照。把这些连同截图或 Ozone 采样交给用户转回 Codex，判断原因并修正，再恢复 PID 整定。安全优先，不能取到的记录留空。

## 4. 第一轮：取得 Pitch 首动和 Yaw 回摆的证据

固定车架、四轮离地、能直接切断驱动力；两板保持互联。遥控先开，右下急停，左上分离；左杆和拨轮回中，只小幅试右杆。冷上电时车与云台静止，等两板 `Working` 再把右拨杆移中。正常有约 2,000 个 IMU 静止样本，加上各板约 500 ms 的恢复自检；实际总时长受标定、反馈和对板就绪影响。**把上电→就绪与就绪后推杆→电机起动分开记录。**

烧录 C 板正常供逻辑电、隔离驱动力，探针连到对应板。用 `Download & Reset Program`，看 `Program & Verify`，然后确认 `Reset & Run / CPU running`。`Reset & Halt` 未继续运行时不会响应。已烧录只选 `Attach to Running Program`，先清掉遗留断点。Ozone 不必一直开，结束会话不能代替急停。动态看 Ozone Data Sampling/Timeline 或导出 CSV；5 Hz Watch 看状态即可，不能量过冲峰值或毫秒延迟。只有一个探针先采云台，必要时同条件另采底盘，不相减两块板的 `uwTick`。没法采样时用固定机位视频初筛。[SEGGER Ozone 手册](https://www.segger.com/downloads/jlink/UM08025_Ozone.pdf)

| 先做什么 | Ozone 看什么 | 按下列证据分流 |
|---|---|---|
| **Pitch**：从相近中间位置，比较冷启第一次、同次第二次及重新解除急停后的上、下小动作 | Watch 看云台 `s_robot.state_`、`estop_`；Data Sampling 录 `s_robot.output_enabled_prev_`、`s_gimbal.pitch_target_`、`s_gimbal.pitch_feedback_`、`s_pitch_motor.torq_input_`。必要时加 `s_pitch_pid.output_`、`integral_`。 | 记录授权时间、目标起变、反馈起动、过冲、稳定时间和是否顶限。角度误差按 `pitch_feedback_ - kPitchJointOffset`，当前源码偏移为 0。`torq_input_` 是驱动请求。 |
| 目标没变或目标缓慢爬 | 云台 `s_robot.peer_input_.pitch_target`；回查底盘 `s_robot.remote_input_.rv`、`gimbal_pitch_target_`，两板状态及软件端点。 | 底盘按竖杆积分后发 C2G：死区 2%，满杆目标速率 2.0 rad/s。尚未授权、在死区或朝限位外推，调 PID 不会解决；目标斜率不足和反馈过慢分开。 |
| 目标已变、首次难抬但低头相对快，或力矩请求一直很大 | 看上/下方向、急停重授权时重复性、积分是否缓慢累积、力矩请求是否顶限。 | 重新使能会清积分；当前无额外重力前馈。负载、配重、静摩擦、机械干涉和反馈都是候选。先排除它们；输出已经饱和时增 P 不能增加可用力矩。 |
| **Yaw**：只小幅横拨右杆后松开，左右分别录一次 | 云台 `s_gimbal.yaw_target_` 对 `yaw_world_feedback_`；旧 ELF 没后者时用 `s_imu.euler_[2]`。必要时加 `s_yaw_pid.output_`。 | 比较世界目标与世界角，跨 ±π 取最短角；`yaw_feedback_` 是关节角，不能直接对世界目标。只有反馈越过已稳定目标才算过冲。 |

当前达妙 MIT 力矩字段承载外部反馈 PID 输出，不另加速度或重力前馈；架空会改变轮胎负载，不能据此断言云台回摆成因。Pitch 松杆保持目标角、不自动回水平。

## 5. 参数在哪，何时才改

以下是交接时的**工作区源码**基线，不等于板上实际参数。

| 回路 | 文件、参数和候选条件 |
|---|---|
| **Yaw** | [Gimbal/Instance/component/ins_pid.cpp](../Gimbal/Instance/component/ins_pid.cpp)，`kYawPidKp/Ki/Kd` 为 **2.5 / 0.2 / 0.24**，D 用世界 Yaw 实测速率，输出 ±3.5 N·m。确认目标稳、反馈越过且控制推得猛，A 确认为这组值后试只减 P（例如 2.5→2.3）。若阻尼不足且速度反馈平顺，可另起一轮只微增 D；积分造成慢性越过才单独减 I。 |
| **Pitch** | 同文件 `kPitchPidKp/Ki/Kd` 为 **3.6 / 1.0 / 0.20**，D 用关节实测速率，输出 ±3.5 N·m。只有目标正常、授权持续、上下都跟不上且输出未顶限，才可试单独小增 P（例如 3.6→3.8）。示例未经本车实测，不是定论；不要为首次迟滞盲拉 I 或扭矩。 |
| **舵角** | [Chassis/Instance/component/ins_pid.cpp](../Chassis/Instance/component/ins_pid.cpp)，电流版 `kSteerPidKp/Ki/Kd` 为 **10000 / 1000 / 600**，内部 raw 最终映射到 ±1.2 A。对比 `s_solver.command_[i].angle`（限速目标）、`desired_angle`（最终目标）、`s_chassis.steer_feedback_[i]`。四轮共同过冲才调整；单轮异常先查反馈/零点/机构。对齐不全时四轮驱动暂为零是当前设计。 |
| **轮速** | 同底盘文件 `kWheelPidKp/Ki/Kd` 为 **0.35 / 1.50 / 0**，输出 ±8 A。舵均对齐、轮速跟踪确实振荡才改。看 `s_chassis.wheel_velocity_`、`s_wheel_pids[i].output_` 和 `align_blocked_mask_`。 |
| **跟随外环** | [Chassis/RobotModules/component/robot.cpp](../Chassis/RobotModules/component/robot.cpp)，`kFollowKp=4.0`、`kFollowKd=0.15`。Yaw/Pitch 摇杆目标速率也在此。云台稳向、轮组稳定且跟随仍反复过零才调。先核对物理对齐时 `peer_input_.yaw_joint` 接近 0、符号正确。 |

`Gimbal/RobotModules/component/gimbal.cpp` 做位置环与 Pitch 外向力矩限制；云台 `robot.cpp` 做授权/接管；`RobotComponents/component/pid.hpp` 是公共实现，正常先仅改本板 `ins_pid.cpp`。当前两板 Pitch 软件角限 −0.9～+0.7 rad，不能当作已验证的机械行程。保留急停、离线保护、对齐门、限幅、IMU 20 ms 门槛、看门狗；不改 CubeMX 生成区、组里标为勿改的接口、接线或电机设置。不在运行中通过 Watch 写 PID。**每次仅一项，变差回退到 A，再开始下一项。**

## 6. 构建和验证：A/B 必须互不覆盖

记录 A 的源码差异、ELF、Ozone 工程和 SHA-256。若现车版本未知，可先做行为基线；要验证单个增益，再在同一工作树先构建 A，然后改一个常量构建 B。下面是 B 的模板，只执行要更换的板，下一轮换目录名。

```powershell
Set-Location -LiteralPath 'D:/ForTest/steered_skeleton/steered_skeleton'
$out = Join-Path (Get-Location) 'Gimbal/build/dsh-pid-B01'
if (Test-Path -LiteralPath $out) { throw "Use a fresh directory: $out" }
cmake -S Gimbal -B $out -G Ninja -DCMAKE_BUILD_TYPE=Debug "-DCMAKE_RUNTIME_OUTPUT_DIRECTORY:PATH=$out"
if ($LASTEXITCODE -ne 0) { throw 'Gimbal configure failed' }
cmake --build $out --parallel
if ($LASTEXITCODE -ne 0) { throw 'Gimbal build failed' }
$elf = Join-Path $out 'steered_gimbal.elf'
Get-FileHash -LiteralPath $elf -Algorithm SHA256
arm-none-eabi-strings $elf | Select-String '^(SAFE-BUILD|FAULT-INJECTION-BUILD|STEER-PROBE-BUILD)$'
```

```powershell
Set-Location -LiteralPath 'D:/ForTest/steered_skeleton/steered_skeleton'
$out = Join-Path (Get-Location) 'Chassis/build/dsh-pid-B01'
if (Test-Path -LiteralPath $out) { throw "Use a fresh directory: $out" }
cmake -S Chassis -B $out -G Ninja -DCMAKE_BUILD_TYPE=Debug -DROBOT_STEER_CURRENT_FIELD=ON -DROBOT_STEER_MODE_PROBE=OFF "-DCMAKE_RUNTIME_OUTPUT_DIRECTORY:PATH=$out"
if ($LASTEXITCODE -ne 0) { throw 'Chassis configure failed' }
cmake --build $out --parallel
if ($LASTEXITCODE -ne 0) { throw 'Chassis build failed' }
$elf = Join-Path $out 'steered_chassis_steer_current_field.elf'
Get-FileHash -LiteralPath $elf -Algorithm SHA256
arm-none-eabi-strings $elf | Select-String '^(SAFE-BUILD|FAULT-INJECTION-BUILD|STEER-PROBE-BUILD)$'
```

**尤其云台：只换 `-B` 可能覆盖默认 ELF，必须显式给 `CMAKE_RUNTIME_OUTPUT_DIRECTORY`。** 给候选复制一份 `.jdebug`，只改 `File.Open(...)` 的路径；核对板号、目标和哈希，原工程保留。正常候选应为 `SAFE-BUILD`；另查 `kEnableFaultInjection=false`、两板 `kDebugAlwaysFeedWatchdog=false`。标签只证明关闭故障注入，不证明已烧录或已验收。

改代码后从根目录运行 `pwsh -NoProfile -File tests/run_tests.ps1`。迭代期可先跑 `tests/run_safety_tests.ps1`、`tests/run_control_tests.ps1`。上次整套 1,873 项、0 失败，双板构建成功，是旧有离线结果。本轮新改动要用当次输出作证。共用代码改动要双板构建；仍按需逐板更新实车。

## 7. 动态验收、记录与最终交付

1. **云台整定后在架空台架做底盘回归**：按[现场总计划第 2 节](31-现场调试与考核收尾计划.md#2-稳定台架复测验证底盘停机问题)核对急停、四轮方向、分离与跟随单杆和组合松杆。底盘回归及安全项通过、Yaw/Pitch 稳定后，才进落地步骤。明显动错、持续摆或停机时停止后续试车。
2. **低速落地**：固定参照看分离时云台是否随底盘摆，前后左右按云台方向走；跟随从左偏与右偏分别看是否收敛，带平移仍按云台方向。小陀螺先低速纯转看中心漂移和云台保持，再在不同车头方位测前后、左右、回中后反向，按车体中心地面轨迹判方向。分别验证分离↔小陀螺、跟随↔小陀螺；退出自旋不继续自旋，进跟随不反复找正。先急停停稳切档，通过后若需连续切档，再单独做受控低速试验。参见 [整套动态表](32-云台PID整定与三模式动态验收.md)。急停、失联、IWDG 和软行程按[总计划](31-现场调试与考核收尾计划.md)保留实测记录。
3. **定版与加分**：留实际烧入的两份 ELF、完整哈希、可追溯源码。唯一加分项用同条件的 Yaw 前后实测、旧值/新值、过冲、稳定时间和保留/回退原因，可附 Ozone 曲线或视频。结果分「通过／失败／未做」；没测的数据留空，正式演示和 GitHub 验收由用户安排。

```text
底盘/云台实际 ELF 和 SHA-256：____
工况（架空/落地）、起点/目标幅度、动作：____
Pitch 冷启/第二次/急停再解除：授权→目标____  目标→起动____  顶限____
Yaw/Pitch：峰值越过____  稳定时间____  稳态偏差____
本轮唯一改动（板/文件/旧值→新值）：____
候选 ELF / Ozone / SHA-256：____  回退 ELF / Ozone：____
构建/主机测试：____  实车通过/失败/未做：____
分离松杆和云台保持：____  跟随左右收敛及带平移：____
小陀螺前后左右/反向和云台保持：____  两类切档：____
停机时模式与操作____  tick_ms____  causes____  IMU reason（适用时）____
```

当前可把下句给 DSH 保留证据：

> 先保留实际烧录记录、对应 ELF 和 Ozone 已有数据。两次 Yaw 自转且遥控急停没有停止，暂停调参和上电验证；请读 `D:/ForTest/steered_skeleton/steered_skeleton/docs/35-Yaw异常转动与急停失效.md`。停机保护由 Codex 跟进，验证完成后再按本交接文档恢复空载整定。
