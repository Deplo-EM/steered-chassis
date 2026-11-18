# 给 DSH：舵轮电控进度同步（R3，2026-11-18）

**当前结论：R3 是最新的双板候选。两板编译、主机回归和固件标识已核对；没有 R3 烧录、实车停机或落地通过的记录。DSH 先接手核对和记录，底盘异常撤力在台架确认解决、停机验收通过后，再继续云台 PID。**

项目目录：`D:/ForTest/steered_skeleton/steered_skeleton`。本次核对 Git `HEAD=c73c2b4`；R3 连同之前 DSH 的改动仍在当前工作区。仅检出该提交得不到 R3 的源码和配置。先看实际工作区，保留已有改动和历史 ELF。考核以 [补全版说明](<../../../Hello World 电控组步兵（舵轮）考核说明（已补全）.pdf>) 和最后一张修正接线图为准。

## 先认准这对固件

| 板 | 本机 ELF | 对应 Ozone 工程 | `g_competition_build_tag` | SHA-256 前 12 位 |
|---|---|---|---|---|
| 底盘 | [R3 底盘](../Chassis/build/competition-r3/steered_chassis_competition_finish.elf) | [chassis_competition_r3.jdebug](../debug/ozone/chassis_competition_r3.jdebug) | `COMPETITION-FINISH-CHASSIS-R3` | `DA16620BD754` |
| 云台 | [R3 云台](../Gimbal/build/competition-r3/steered_gimbal_competition_finish.elf) | [gimbal_competition_r3.jdebug](../debug/ozone/gimbal_competition_r3.jdebug) | `COMPETITION-FINISH-GIMBAL-R3` | `3724BFBD4E79` |

两个 Ozone 工程默认 **Attach**，只用于观察已运行的固件，不会烧录。先分开记录「电脑上候选版本」和「两块板实际烧入版本」；板上版本不清楚就写未知。[44 号审计](44-底盘与云台撤力路径审计.md#候选产物与复核)有完整哈希。v22/B22、40/41 号首跑、42 号候选及 43 号 R2 都是历史对照，不能把其台架现象记成 R3 结果；尤其首跑固件不是三档整车版。

## 截至现在做了什么

| 问题 | 当前代码 | 证据和边界 |
|---|---|---|
| 分离、小陀螺偶发撤力 | 底盘自检通过并进入 Working 后，两档方向计算使用新鲜的云台 Yaw 关节角，不再因**底盘本板** IMU 不可用而停车。跟随档用底盘陀螺做阻尼，仍检查；云台仍检查自己的 IMU 与底盘明确授权。 | 主机回归覆盖两档继续响应、坏样本时切跟随撤力、云台不被对板 IMU 遥测位误停。旧版确有一次 `kAccLow` 现场快照，不能据此解释所有停机。 |
| 快速换向后，轮组实际上已对齐却不给驱动力 | 新目标进入现有 0.08 rad 全驱动误差范围时，从实测舵角重新起步并清该轮旧 PID 历史。实测仍歪，驱动轮继续等舵纠偏。 | 主机复现已修；真实负载下的对齐时间、直行和过冲还没测。正常 R3 对齐等到 300 ms 只记等待超时，持续大出力、没有角度进展才是单独的堵转判定。 |
| 一帧例行板间状态没挤进发送邮箱，整车锁存 | 例行状态下一周期重试，云台转发的真实遥控分片优先。关键授权和急停变化、Yaw 停止或反向的关键帧、真实发送错误仍走停机路径。 | 主机区分临时邮箱忙与真的断链、发送失败，覆盖重连后不能自动恢复出力；发送计数上涨本身不证明进了保护。 |
| 小陀螺合推时旋转变弱 | 保留 R2：轮速够时同时保留旋转和平移；轮速会超限时，先保留可实现的旋转，再用余量统一缩放平移。已删额外的 0.60 m/s 小陀螺平移上限。 | 逆解回归通过。实际偏航速度和地面平移轨迹仍未验收。 |
| Pitch 首动慢、受力时误掉力；Yaw 过冲 | 保留 B22 的重力补偿；Pitch 反馈与前馈合计限幅，正常饱和不再因为越界返回而停机。共同软件限位 −0.47～+0.70 rad，到边界只拦向外力矩。R3 没改 PID 系数。 | 停机分支有主机回归。首动延迟与过冲仍需测；用户此前报告两次 Yaw 自转且遥控急停无效，原事件根因、真实卸力仍未闭环。 |
| 两只轮刮擦 | 保留 R2 按两支等效舵角绕行；真实无路则归零、撤销待发旧帧。配置数组目前全空，`steer_keepout_configured_mask` 应为 0。 | 历史右后卡点见 38 号；两个物理轮位、受载干涉范围和角度都还缺完整记录。R3 没有启用未知位置的绕行。 |

构建证据：两份 ARM ELF 构建成功，2026-11-18 核对了文件哈希及 Ozone 指向。上轮全套主机测试 2,639 项零失败；补上真断链及 Pitch 重力补偿配置用例后，针对底盘两种、云台两种配置各通过 146、146、123、24 项，与全套有重叠，不相加。范围与复现见 [44 号](44-底盘与云台撤力路径审计.md)；当前没有实车放行记录。

## 下一步按这个顺序接手

1. **先核对机械和停机。** 依据 [38 号放行清单](38-四轮版下地放行清单.md) 与 [35 号 Yaw 事件](35-Yaw异常转动与急停失效.md)，隔离动力确认刮擦、轮位与零点；没有验证独立断动力与实际卸力前，暂停带动力 PID 和落地。恢复台架后先查本地急停、遥控失联、双板断链、重连不自动起动，以及 IWDG；要看到车轮和两轴真实停止出力，不能只看指令归零。R3 是完整三档候选，没带首跑版的独立中位／起步门；小陀螺档拨轮居中仍会给旋转请求，右下急停释放后不能按「一定不动」理解。
2. **先确认 R3 不再异常撤力，再调 PID。** 在满足台架条件后，从分离单独轻动左杆、右杆开始，再测组合动作、松杆和跟随切换；逐次记两板状态、是否进 Dead、是否需要重新授权。出现不受控运动、急停异常、复位或新的 Dead，停止该项并保存版本与当拍记录，由用户转回 Codex 复查。底盘这一关通过后，DSH 再沿 [33 号 PID 交接](33-DSH云台PID与动态验收交接.md) 的流程采基线；其中旧参数和旧限位不要抄回 R3，以当前 [云台 PID 实例](../Gimbal/Instance/component/ins_pid.cpp) 和 [限位头文件](../RobotComponents/component/gimbal_limits.hpp) 为准。
3. **单项比较，随后验三档。** 先看目标有没有变化、输出是否饱和、实测是否跟随，再定是否改 PID。一次只变一项，保留两个可对照的 ELF；Ozone 只读采样记录，避免带动力暂停／单步或在线改值。台架稳定后再按 38 号条件评估地面：分离直线与停车在前，跟随和小陀螺合推在后。基础验收优先；加分项仍只准备 PID 整定前后对比记录。

已知配置：底盘 CAN1→GM6020（R3 电流帧 `0x1FE`）；底盘 CAN2→M3508，并连接云台 CAN1（同总线有 Yaw、双板帧）；Pitch 在云台 CAN2；接收机在云台 UART3。R3 程序的驱动和舵向 ID 都是 `{1,2,3,4}`，索引依次左前、左后、右后、右前；38、39 号的 `{3,4,1,2}` 是早期候选。两类电机分别核对实际轮位，别把任何一版数组直接当实车证据。对照 [路由定义](../RobotComponents/component/board_wiring.hpp) 与 [电机实例](../Chassis/Instance/component/ins_motor.cpp)。遥控左拨杆上／中／下为分离／跟随／小陀螺；右下是急停。三档均以云台朝向为左摇杆平移参考；分离档不会自动让车头追云台。

## 有异常时，先留这些证据

| 看到什么 | 用匹配 ELF 看什么 |
|---|---|
| 底盘不走但还是 Working | `robot::diag::g_chassis_dbg` 的 `estop`、`authorized`、`drive_ready`、`align_blocked_mask`、`steer_keepout_blocked_mask`；再比 `steer_desired`、`steer_target`、`steer_feedback`。舵在纠偏、轮电流为零，和整板 Dead 要分别记。 |
| 底盘进 Dead | `s_robot.last_dead_` 的 `tick_ms`／`causes`。位值：1 堵转、2 CAN 发送、4 遥控、8 IMU、16 电机、32 对板、64 未授权、128 状态异常，可叠加。含 8 再读 `s_robot.last_imu_dead_.imu`。 |
| 云台不动作或 Pitch 到限位 | `robot::diag::g_gimbal_dbg` 的 `estop`、`authorized`、`last_dead_reason`、`last_dead_tick_ms` 以及对应轴的目标、反馈和输出。Working 但 `authorized=0` 先查授权，别先改 PID。 |
| 看着像重新启动 | `uwTick` 是否归零；与停机时间对照 `g_reset_by_iwdg`。只在设备已安全停止时取证，有异常出力先切动力。 |
| 小陀螺合推变成纯平移 | 对比 `ramped_w`、`drive_w_request`、`alignment_scale`。若命令仍有转速而实际不转，再核对实际偏航、四轮速度、零点和受载刮擦。 |

记录一行即可开始交接：`日期 / 两板实际版本 / 架空或落地 / 档位和拨杆动作 / 实际动作及先异常的板 / 状态、原因位与时间戳 / 是否复位 / 已重复次数`。不为了补记录重新带电复现，测不出写「未测」。

离线重建时，从当前仓库根目录另建目录以保留上表产物：

```powershell
cmake -S Chassis -B Chassis/build/competition-r3-check -G Ninja -DCMAKE_BUILD_TYPE=Debug -DROBOT_COMPETITION_FINISH=ON -DROBOT_STEER_CURRENT_FIELD=ON
cmake --build Chassis/build/competition-r3-check
cmake -S Gimbal -B Gimbal/build/competition-r3-check -G Ninja -DCMAKE_BUILD_TYPE=Debug -DROBOT_COMPETITION_FINISH=ON -DROBOT_PITCH_GRAVITY_FF=ON
cmake --build Gimbal/build/competition-r3-check
pwsh -NoProfile -File tests/run_tests.ps1
```

若另出候选，写明改了哪一项、A/B 两个 ELF 的路径和哈希、匹配的 Ozone 工程、复测记录和回退方式；已有 R3 文件留作基线。详细源码审计看 [44 号](44-底盘与云台撤力路径审计.md)，机械避让数据看 [43 号](43-两轮刮擦绕行与限速简化.md)，三模式测试步骤看 [42 号](42-三模式考核收尾与台架验收.md)；这些旧文档里的固件链接只对应各自版本。
