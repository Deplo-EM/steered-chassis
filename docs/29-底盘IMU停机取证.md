# 底盘 IMU 停机取证

最新候选见 [BMI088 采样修复与现场回归](30-BMI088采样修复与现场回归.md)。本页下文的 `imu-stop-trace` 为旧取证版，保留以便对照已有截图。

2026-11-14，`separate-release` 底盘：分离档分别在两杆同动、只动左杆、只动右杆时出现过 `Dead`；跟随档只动左杆也出现过。四次截图都显示 `last_dead_.causes = 8`。该位是底盘本地 IMU 条件失效；`uwTick` 持续增长，所以这几次是状态机停机。第三次的残余平移目标和现场操作描述有出入，旧版没有保存停机当拍的摇杆值，不能据此认定遥控串轴。正常静止截图中的五路输入都是 0。

底盘 IMU 截图显示 SPI 读取失败 0、时钟回退 0、数值拒绝 23、新鲜度拒绝 9，后者同时伴随加速度过期计数 9。它们都是开机以来的累计值，不能和某次停机一一对应；当前 `healthy_=1` 也不代表停机当时样本有效。代码在首个不合格样本就撤销该拍的输出资格，50 次连续坏样本才会另行标记长期不健康。

## 现场只换底盘诊断版

- 当前使用的 `separate-release` 底盘 ELF 保留。新工程：[chassis_imu_stop_trace.jdebug](../debug/ozone/chassis_imu_stop_trace.jdebug)。打开后标题中的 ELF 应来自 `Chassis/build/imu-stop-trace/`，文件名是 `steered_chassis_steer_current_field.elf`。它仍是舵向电流版（`0x1FE`），使用原来的 PID、验收模式和保护条件。云台板无需重烧。
- 空载架稳、四轮离地，能够直接切断动力。保持右拨杆下档，隔离电机动力时烧录底盘，使用 **Reset & Run**。Ozone 保持连接，在 `Watched Data 1` 中展开 `s_robot → last_dead_`。如果已经捕获故障，直接读记录，不必再复现。
- 需要取证时，按已出现过的分离档场景做一次小幅输入即可。进 `Dead` 后回右下急停，先不复位或断开板上逻辑电源。展开 `last_dead_ → imu`，先记 `reason`；若 `last_rejected.has_values=1`，再展开其中 `accel_mps2[0..2]`。如需区分遥控输入，展开同一快照里的 `remote`，读取 `lv/lh/rv/rh/wheel` 和 `remote_source`。快照保留到下一次进入 `Dead` 或复位。发生异常运动时优先断动力。

## 如何读 `last_dead_ → imu`

只有 `last_dead_.causes` 含位值 `8` 时才使用这组字段。各 `age_ms` 表示距上次观测到新数据的时间，`4294967295` 表示初始化或重新标定后尚未观测到。若多项同时过期，`reason` 给出优先项，并保留各自的时间。历史坏帧与当拍原因分开保存；新鲜度故障时不要把 `last_rejected` 里的旧样本当成当拍读取结果。

| `reason` | 含义与下一项证据 |
|---|---|
| 1 / `kSpi` | IMU 读操作返回失败；先查同一时段是否连续发生。 |
| 2 / `kClockBackward` | IMU 设备时钟异常回退，触发重标定。 |
| 3 / `kGyroFifo` | 陀螺仪帧队列失去连续性，触发重标定。 |
| 4 / `kAccLow`、5 / `kAccHigh` | 合加速度低于 2 或高于 40 m/s²；读取真实判废帧的三轴值和时间。 |
| 6 / `kAccInvalid`、7 / `kGyroInvalid`、8 / `kTempInvalid` | 数值非有限，或陀螺仪轴值越界；读取 `last_rejected`。 |
| 9 / `kAhrs` | 姿态计算结果无效，触发重标定。 |
| 10–13 | 设备时钟、加速度、陀螺仪或控制可用样本过期；分别看四个 `age_ms`。 |
| 14–16 | 还未标定、尚未恢复健康或未初始化；结合 `calibrated`、`healthy` 和此前判废记录。 |

这版只补证据。真实冲击、假读数和数据更新过期需要分开处理；在拿到对应停机的记录前，不修改阈值。主机测试覆盖 SPI 失败、时钟/FIFO 异常、加速度新鲜度及量级判废、停机后保留快照和急停再授权，仍需现场采到一次故障才能确定修复方向。

## 离线核对

- 底盘诊断 ELF：`78EE5F512CA90FB76D039E0F6DDB3DB37E3DD60BABCA95FEEC73A0D598AB2CA4`（SHA-256）。ELF 含 `SAFE-BUILD`，表示故障注入关闭；调试信息包含 `last_dead_`、`last_rejected`、各数据年龄和三轴读数。
- 两板编译成功，1,844 项主机检查通过。原 `separate-release` 底盘和 `axis-tune` 云台 ELF 的 SHA-256 不变；默认路径的原云台 ELF 在验证构建覆盖后按原 SHA-256 恢复。
- 单独验证云台时，除了 `-B`，还要显式传 `-DCMAKE_RUNTIME_OUTPUT_DIRECTORY=<验证构建目录>`，否则验证会覆盖 `Gimbal/build/steered_gimbal.elf`。云台验证产物与本轮台架取证无关。
