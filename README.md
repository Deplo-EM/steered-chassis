# 舵轮步兵电控（Hello World 电控组考核 · steered_skeleton）

> 一台「4 个驱动轮 + 4 个转向舵电机 + 两轴云台」的舵轮步兵，由两块 STM32F407 分别控制
> 底盘和云台，通过 CAN 交换信息。本仓库实现考核控制链路；台架故障注入与实车验收尚未完成。
>
> 一句话概括设计：**底盘板当大脑**（读遥控器、决定车怎么走、把云台指令发过去），
> **云台板当瞄准器**（yaw 用 IMU 做世界系稳定，pitch 做关节限位闭环），
> **失联时两板软件均撤销输出许可并请求零输出**。电机的实际停机时间需在台架测量。

---

## 1. 两块板分别负责什么

| | 底盘板（Chassis） | 云台板（Gimbal） |
|---|---|---|
| 执行器 | 4× M3508（驱动轮，CAN1）+ 4× GM6020（转向，CAN2） | 2× 达妙 DM-J4310（yaw / pitch，CAN2） |
| 传感器 | BMI088 IMU（SPI1） | BMI088 IMU（SPI1） |
| 遥控器 | USART3 DBUS | USART3 DBUS（可选，用于本地急停） |
| 算的东西 | 三种模式、云台系→车体系旋转、舵轮逆解、轮速环、舵角环 | yaw 世界角闭环、pitch 关节角闭环 |
| 发出的帧 | C2G `0x210`（yaw 转速设定 + pitch 目标 + 许可标志） | G2C `0x211`（yaw/pitch 实际关节角 + 状态） |

两块板之间**只有** CAN 帧这一条通路，不共享变量、不互相调用函数。

---

## 2. 快速开始

### 2.1 编译固件

```powershell
# 底盘板
cd Chassis
cmake -S . -B build/verify -G Ninja -DCMAKE_BUILD_TYPE=Debug
cmake --build build/verify

# 云台板
cd ../Gimbal
cmake -S . -B build/verify -G Ninja -DCMAKE_BUILD_TYPE=Debug
cmake --build build/verify
```

产物：`Chassis/build/steered_chassis.elf`、`Gimbal/build/steered_gimbal.elf`
（`.hex` / `.bin` 位于本次配置的构建目录，如 `build/verify`）。

> 为什么用 `build/verify` 而不是 `build/Debug`：骨架包里自带的 `build/Debug` 是在**别的电脑**上
> 生成的，里面的 CMakeCache 记着旧路径，直接用会报 "CMakeCache.txt directory is different"。
> 新建一个目录最省事（不要删旧目录，它对你有用）。

### 2.2 跑电脑端单元测试（不需要单片机）

```powershell
cd steered_skeleton      # 仓库根目录
./tests/run_tests.ps1
```

检查数量以脚本本次输出为准。覆盖范围与修复证据见 `docs/11`；主机测试使用模拟硬件，
不能证明电机实际卸力、CAN 总线送达、IMU 安装方向或 1ms 最坏执行时间。

### 2.3 烧录

用 ST-Link / Ozone / CubeProgrammer 烧对应的 `.elf` 即可。
两块板各烧各的，**顺序无所谓**，但都必须上电且 CAN 互联，否则状态机会停在 Dead。

---

## 3. ⚠️ 上电前必读

1. **遥控器必须先开机**。IWDG 一上电就开始计时（约 200ms），而喂狗条件是
   「控制周期跑完 + 合法遥控数据仍新鲜 + CAN 未持续发送失败」。遥控器没开 → 板子约每 200ms 复位一次。
   这是**故意的**（关遥控器 = 车必须停），不是故障。
   台架上需要不开遥控器调试时，把 `Task/component/comm_task.cpp` 里的
   `kDebugAlwaysFeedWatchdog` 改成 `true`（= 无条件喂狗，板子不再复位）；
   **验收前必须改回 `false`**，否则看门狗保护就失效了。
   > ⚠️ 方向别搞反：`false` 是正常模式（**条件**喂狗，关遥控器会复位）；
   > `true` 才是调试模式（**无条件**喂狗，所以板子不复位）。
2. **接收机必须接到底盘板 USART3，云台本地 DBUS 为可选**。底盘 Working 条件包含"本板收到合法 DBUS"，
   所以只把接收机接在云台板时，车**不会动**（云台帧里的 `dbus_online` 只用于喂狗保活）。
3. **第一次上车把轮子架空**。方向标定（电机正方向、舵角零点）没做完之前，不要落地。
4. **急停键位**：右拨杆打到最下面。上电及任何安全故障退回 Dead 后，需重新收到有效急停档确认；正常档保持不能自动恢复。解除前确认全体就绪并让摇杆回中。
5. **IMU 零漂标定需要累计 2000 个有效样本（1kHz 下至少 2 秒）**，期间车必须静止不动，
   标定完成前所有电机输出为 0。
6. **验收时怎么证明看门狗真的动作过**：关遥控器 → 板子复位 → 重新连调试器，
   读全局变量 `g_reset_by_iwdg`，应为 `true`（`MainTaskInit()` 开头会读并清除
   RCC 的 `IWDGRST` 标志）。这比"看 LED 闪不闪"可靠。

---

## 4. 目录结构

```
steered_skeleton/
├─ Chassis/                       底盘板
│  ├─ Core/                       CubeMX 底层（main / can / tim / usart / spi / iwdg）
│  ├─ inc/ + src/                 组件库 + 你自己写的东西（imu.hpp / pid 等）
│  ├─ Instance/component/         ★ 实例化层：电机 ID / 方向 / 减速比、IMU 引脚、单例
│  ├─ RobotModules/component/     ★ 应用层：chassis（逆解+闭环）、robot（状态机+模式）
│  └─ Task/component/             ★ 任务层：main_task（装配+1ms）、comm_task（DBUS+CAN+喂狗）
├─ Gimbal/                        云台板（结构与底盘板同构）
├─ RobotComponents/component/     两板共享：module_state（类型）、gimbal_chassis_comm（协议）、pid
├─ tests/                         电脑端单元测试（不影响固件编译）
└─ docs/                          原理讲解 / 调车手册 / 验收对照
```

标 ★ 的就是考核要求「你自己写」的部分。

---

## 5. 考核任务对照

| # | 考核要求 | 实现位置 |
|---|---|---|
| 1 | CAN 接收分发 + 电机配置 | `Chassis/src/HW_can.cpp`、`Gimbal/src/HW_can.cpp`、`Instance/component/ins_motor.cpp` |
| 2 | UART DBUS 接收（两板） | `Task/component/comm_task.cpp`（`HAL_UARTEx_RxEventCallback`） |
| 3 | PID 控制器类 | `RobotComponents/component/pid.hpp` |
| 4 | Imu 封装类 | `Chassis/inc/imu.hpp` + `src/imu.cpp`（云台板同构） |
| 5 | module_state 共享类型 | `RobotComponents/component/module_state.hpp` |
| 6 | 舵轮逆解 + 前进方向变换 | `Chassis/RobotModules/component/chassis_ik_solver.*` + `chassis.cpp` |
| 7 | 云台两轴位置环 | `Gimbal/RobotModules/component/gimbal.cpp` |
| 8 | 双板通信协议 | `RobotComponents/component/gimbal_chassis_comm.*` |
| 9 | 状态机 + 任务调度 | `RobotModules/component/robot.cpp`、`Task/component/main_task.cpp` |
| 10 | IWDG 看门狗 | 两板 `Task/component/comm_task.cpp` 末尾（唯一喂狗点，控制周期跑完才喂） |
| 加分 | 舵轮回摆抑制 | 逆解里的等效解翻转 + 速率限幅 + 「舵角没到位不驱动」 |
| 加分 | 跟随/小陀螺 | `robot.cpp` 的三模式分支 + 陀螺仪阻尼 |
| 加分 | 双板互保 | 两板状态机互相把「对方不在线」当停机条件 |
| 加分 | 小陀螺平移方向不变 | 每 1ms 用最新 δ 旋转指令（仿真用例已验证） |

---

## 6. 三个关键设计取舍

1. **云台 yaw 用「世界系 + 转速设定」，不用「关节角 + 角度目标」**
   yaw 电机装在底盘和云台之间，只做关节角闭环的话，底盘一转枪口就跟着转，
   跟随模式永远收敛不了。所以 yaw 必须用云台 IMU 钉住世界方向，而世界角只能由
   持有 IMU 的云台板自己积分 → 底盘板下发「转多快」。详见 `docs/03`。

2. **云台用「纯力矩 + 自写 PID」，不用达妙内置位置环**
   达妙 MIT 模式支持位置环（kp/kd），但我们把 kp、kd 都设为 0，只用力矩通道。
   这样整定逻辑全在自己手里，换电机也不用改控制结构；代价是参数要更保守。

3. **静止时不自动回中舵轮**
   进入 Working 时保留舵轮当前朝向，而不是「嗖」地转回 0°。
   少一次不必要的动作，就少一次夹手/扫到东西的机会。

---

## 7. 文档索引

| 文档 | 内容 |
|---|---|
| [`docs/00-交付说明与验收对照.md`](docs/00-交付说明与验收对照.md) | 交付清单、自测证据、验收时怎么演示 |
| [`docs/01-总体架构与坐标系.md`](docs/01-总体架构与坐标系.md) | 数据流、1ms 时序、坐标/符号约定、状态机 |
| [`docs/02-舵轮运动学逆解.md`](docs/02-舵轮运动学逆解.md) | 从零推导逆解、等效解翻转、限幅 |
| [`docs/03-双板通信协议.md`](docs/03-双板通信协议.md) | 8 字节帧格式、为什么这么设计、CRC/离线判定 |
| [`docs/04-控制环路与PID整定.md`](docs/04-控制环路与PID整定.md) | 三个模式、三个环、整定步骤、现象→原因对照表 |
| [`docs/05-IMU、状态机与安全机制.md`](docs/05-IMU、状态机与安全机制.md) | 零漂标定、三态迁移、五层安全保护 |
| [`docs/06-调车手册.md`](docs/06-调车手册.md) | 从空车到验收的完整步骤 + 故障排查 |
| [`docs/07-审计回应与修复记录.md`](docs/07-审计回应与修复记录.md) | 独立审计的逐条结论、修复与证据（含仍未验证项） |
| [`docs/08-台架故障注入清单.md`](docs/08-台架故障注入清单.md) | 8 项可执行的故障注入步骤：怎么注入、看哪个变量、什么算通过 |
| [`docs/09-现场调试待办.md`](docs/09-现场调试待办.md) | 现场核对清单与尚未实现的单电机台架流程边界 |
| [`docs/11-控制与安全修复记录.md`](docs/11-控制与安全修复记录.md) | 2026-10-27 那轮控制/安全修复的逐条记录与回归证据 |
| [`docs/12-基地现场一页纸.md`](docs/12-基地现场一页纸.md) | 到基地照着做的 8 步顺序 + 现象速查 + 验收演示话术 |

---

## 8. 说明

- `Drivers/`、`third_party/`（HAL、CMSIS-DSP、Eigen3）体积大，未纳入版本控制，
  请使用骨架包自带的这两份目录。
- 电机方向、舵角零点、减速比与几何尺寸已按补全版说明书预置；PID 为待实车整定的初值。
  参数来源与文件 SHA-256 见 `docs/00`，现场核对步骤见 `docs/06`。

本轮修改与验证：[`docs/11-控制与安全修复记录.md`](docs/11-控制与安全修复记录.md)。台架 T1–T8 尚未执行。
