# 2026-11-10审计探针

这些程序只在电脑上运行，使用仓库真实源码，不连接硬件。模式探针独立运行；相应实际驱动断言已加入tests/run_tests.ps1，最新数量见docs/15。
`motor_feedback_probe`现用于复核修复后速度、故障锁存和节点校验；任一不符返回非零。
`mode_behavior_probe`以既有测试硬件替身为基础，调用真实Robot、CommTask和底盘模块，含8个模式断言。

在仓库根目录PowerShell执行：

```powershell
g++ -std=c++20 -O1 -Wno-unknown-pragmas '-DSTM32_HAL_FILENAME=<host_hal.hpp>' -Itests/control_stubs -IChassis/inc docs/audit_evidence/motor_feedback_probe.cpp Chassis/src/motor_DaMiao.cpp Chassis/src/motor_base.cpp Chassis/src/base.cpp -o tests/build/audit_motor_feedback.exe
if ($LASTEXITCODE -ne 0) { throw '编译失败，停止' }
& tests/build/audit_motor_feedback.exe

g++ -std=c++20 -O1 -Wall -DUSE_UNIX_TICK -DTEST_CHASSIS '-DSTM32_HAL_FILENAME=<host_hal.hpp>' -Itests/safety_stubs -IRobotComponents/component -IChassis/inc -IChassis/RobotModules/component -IChassis/Task/component docs/audit_evidence/mode_behavior_probe.cpp Chassis/RobotModules/component/robot.cpp Chassis/Task/component/comm_task.cpp Chassis/src/DT7.cpp Chassis/src/motor_base.cpp Chassis/src/base.cpp RobotComponents/component/gimbal_chassis_comm.cpp Chassis/RobotModules/component/chassis.cpp Chassis/RobotModules/component/chassis_ik_solver.cpp -o tests/build/audit_modes.exe
if ($LASTEXITCODE -ne 0) { throw '编译失败，停止' }
& tests/build/audit_modes.exe
```

模式探针断言失败返回非零。达妙探针修复后输出（原异常输出保留在docs/13）：

```text
velocity: expected=-17.892307 actual=-17.892307 (rad/s)
overtemp: decode=1 status=0xB next_command=0xFD
wrong_node_nibble: configured=1 payload=2 decode=0
```
