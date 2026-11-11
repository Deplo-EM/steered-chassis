# 18 · 现场无响应排查与Ozone调试信息兼容（2026-11-11）

> 2026-11-11后续运行代码修正见[现场固件适配记录](19-现场固件适配与复核.md)。本轮改变发送路由、遥控转发和接收恢复，必须配套重烧两板；此前“仅调试信息变化、无需重烧”只适用于更早的DWARF修正。

## 当前事实

用户确认两板均上电，双板CAN已连接；接收机DBUS接哪块板尚未确认。
提供的Console有Program & Verify完成记录，表明本次下载链路成功；Startup complete的PC=0x08000718对应本地云台main入口，
不代表IMU标定、反馈在线或Working已经满足。最后Debug.Stop/Disconnected表示调试连接结束，不能继续把窗口旧值当实时数据。
日志还包含ELF location list / directory index解析警告，不能据此直接认定CPU运行故障。

## 已进行的兼容性修正

GCC15原Debug构建输出DWARF5；两板工具链改为显式`-gdwarf-4 -gstrict-dwarf`，保留原-g3和-O0。
两板构建exit0，无新警告；编译单元为DWARF4，行表版本3/4，不再使用DWARF5 loclists/rnglists。
修改前后两板烧录BIN逐字节相同，LOAD布局也相同。因此本次只调整ELF调试元数据，不改变控制算法、参数、接线或保护门。
证据见`audit_evidence/ozone-dwarf4-2026-11-11.json`；旧报告中的ELF哈希作为历史值保留，当前哈希以此记录为准。

只因本次调试信息改变，不需要重烧运行代码；重新打开对应.jdebug使Ozone加载新ELF。
该结论以目标原来运行同一版代码为前提：如果目标固件本就不同，则仍需重新确认版本，不能拿不匹配的ELF读变量。
Ozone GUI与实板解析是否消除Warning96/106尚待用户确认，不能把离线DWARF检查等同于上板验证。

两份Ozone工程增加5个只读Watch表达式：uwTick、g_reset_by_iwdg、robot::diag::g_cpu（展开后看cycle_count）、
s_dbus_valid_frames、s_dbus_invalid_frames。Watch不执行目标函数、不修改目标内存，也不自动连接/运行。

## 下一步只查底盘遥控链路

1. 固定架空、保持遥控器右下急停。按现场规程隔离驱动力并断电后，接探针到**底盘板**；保留两板CAN。
2. 恢复供电，Ozone重新打开debug/ozone/chassis.jdebug，选择Attach to Running Program；确认Running。
3. 打开Watched Data，给预填5项开启Refresh Rate。不要为看计数反复Halt，也不改PID。
4. 记录合法/非法DBUS计数是否增长、uwTick/cycle_count是否增长或反复归零、g_reset_by_iwdg值。

| 观察 | 可以推断什么 | 还不能推断什么 |
|---|---|---|
| 合法计数持续增加 | 底盘确实在接收并通过DBUS帧校验 | 不证明电机/IMU/对板全部就绪或解除急停 |
| 非法计数增加 | 底盘收到数据，但帧合法性未通过 | 不能仅凭此断定某根线接错；需核对帧、接收配置与接收机 |
| 两计数0，控制计数正常增长 | 底盘未收到可处理的DBUS事件 | 可能接收机在云台、未供电/配对或接线/接收异常，不能先改线猜 |
| 计数反复归零、IWDG标志true | 存在看门狗复位循环 | 还需分辨遥控新鲜度、CAN持续失败、任务卡死等原因 |
| Running但cycle_count不增长 | 控制周期未完成或任务尚未启动 | 需定位PC、初始化/中断，不认为Running就等于Working |

当前底盘必须有本地合法DBUS才授权Working。云台收到遥控并经G2C报告dbus_online只可保活，不转发底盘摇杆值。
如果证实接收机只接云台，这属于固件控制来源与实际线束不匹配；先记录证据，再由团队决定适配代码还是接线，不擅自热插拔/改线。

## 自动回中状态

当前代码没有实现上电主动舵向回中：底盘停止时保持/重置到实际舵角，云台恢复控制时对齐当前姿态。
这是功能未闭合项，不应把“没有回中”解释为烧录失败，也不能在保护条件未知时强行下发回中力矩。
后续补回中需要限速、机械边界、失败超时、有效遥控许可与急停，并协调任务书中“非Working零输出”的约束；本次未新增运动行为。

遥控无响应的实板根因尚未确认，等待上述底盘计数/状态；不得将调试信息修正宣布为整车故障已解决。


## 用户首轮实测反馈

底盘`s_dbus_valid_frames=0`、`s_dbus_invalid_frames=0`、`g_reset_by_iwdg=1`；cycle_count未找到。
目前不能据此唯一归因于接收机接错：计数受复位影响，也可能UART接收尚未建立/出现其他长度事件。
复位标志为1只说明最近一次启动带IWDG复位标志，不独立证明现在仍在反复复位。
下一步先观察uwTick能否持续超过1000，以区分当前复位循环与持续运行。

cycle_count不是独立全局符号：它是`robot::diag::g_cpu`成员，当前底盘ELF中g_cpu=0x20000D2C，
类型有cycle_count/last_cycle_ticks/cycle_ticks_max/cycle_ticks_avg/overrun_count。
地址仅为本构建的离线核实证据，不要求用户手填固定地址。Ozone工程改为预填g_cpu整体，展开成员观察，避免把成员名当全局名搜。


## 第二轮实测反馈

用户观察底盘uwTick在0～220ms之间重复变化，且g_reset_by_iwdg=1；截图状态栏为CPU running，
Watch中合法/非法DBUS计数均0，g_cpu父结构已存在但未展开。
可确认存在看门狗复位循环，不能再归因于持续Halt；具体不喂狗原因仍待查。
截图中的g_cpu左侧加号可展开cycle_count；下一步在同一底盘连接读取s_robot.peer_online_、
peer_input_.dbus_online、can_tx_guard_.faulted_，分辨对板遥控信息/板间通信/CAN发送保护。
没有用“接上CAN”推定协议已在线，也没有把单次仲裁/串口信号问题凭空认定为根因。


## 第三轮实测与线束适配待确认

用户确认cycle_count增长后归零；新截图中peer_online_=0、remote_armed_=0、state_=kDead、estop_=0。
这说明控制周期确实执行后复位，并且该次观察中未取得有效对板在线与遥控恢复确认；
estop_=0不能证明用户已成功解除急停：没有合法遥控输入时，它也可能只是默认值。
此前合法/非法DBUS计数均0，仍需查实际接收源与UART接收情况，不能只凭计数断定硬件无信号。
peer_input_.dbus_online与can_tx_guard_.faulted_尚未取得，不能臆测为0或1。

重新比对用户最初提供的接线图发现：图中底盘CAN1接GM6020，CAN2接M3508、yaw和云台CAN1；
当前实现却发送M3508/板间帧到底盘CAN1、GM6020到底盘CAN2，且yaw由云台CAN2控制。
接收分发两路全收按ID路由，不代表发送可跨越物理上不相连的总线。
补全PDF允许CAN按实机接线分配，因此这应记录为实际线束尚未确认的适配风险，不能仅凭旧图宣布现场根因已唯一确定。
当前代码还要求底盘本地DBUS；补全PDF的双板概述将遥控接收列在云台板，代码来源假设需核实并适配。

下一步请用户只观察、不插拔：确认板间线在底盘端的CAN1/CAN2丝印，以及DBUS实际进入哪块板。
当前未修改运动行为、发送路由、看门狗或保护条件，也未要求用户重烧或改线。
