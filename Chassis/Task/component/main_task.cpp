/**
 *******************************************************************************
 * @file      :main_task.cpp
 * @brief     : 底盘板任务入口（初始化装配 + 1ms 周期调度）
 * @history   :
 *  Version     Date            Author          Note
 *  V1.0.0      2026-10-19      <author>        1. 完成编写
 *******************************************************************************
 * @attention :
 *  【本文件是整台车的「装配车间」】
 *  Instance 层负责造零件（电机、IMU、PID、逆解器、通信对象），
 *  这里负责把它们按依赖顺序插到一起，然后把 1ms 的节拍交给 Robot。
 *
 *  【依赖顺序不能乱】
 *    电机 / IMU  →（它们依赖 SPI、GPIO，已由 CubeMX 在 main 里初始化好）
 *    底盘模块    →（需要电机、PID、逆解器）
 *    Robot       →（需要底盘模块、IMU、遥控器、通信对象）
 *    通信任务    →（最后启动，因为它一开就会开始收帧、喂狗）
 *******************************************************************************
 *  Copyright (c) 2024 Hello World Team, Zhejiang University.
 *  All Rights Reserved.
 *******************************************************************************
 */
/* Includes ------------------------------------------------------------------*/
#include "main_task.hpp"

#include "chassis.hpp"
#include "comm_task.hpp"
#include "diagnostics.hpp"
#include "fault_injection.hpp"
#include "gimbal_chassis_comm.hpp"
#include "imu.hpp"
#include "ins_all.hpp"
#include "main.h"
#include "motor.hpp"
#include "pid.hpp"
#include "robot.hpp"
#include "tim.h"
/* Private macro -------------------------------------------------------------*/
/* Private constants ---------------------------------------------------------*/
/* Private types -------------------------------------------------------------*/
/* Private variables ---------------------------------------------------------*/

/**
 * @brief 本次复位是否由独立看门狗触发（调试器里可直接观察）
 *
 * 验收时用它证明看门狗真的动作过：关掉遥控器 → 板子复位 → 重新连调试器，
 * 读这个变量应为 true。比"看 LED 闪不闪"可靠得多（审计 F2 的建议）。
 *
 * 不加 static，保证在调试器的全局变量窗口里能按名字找到。
 */
volatile bool g_reset_by_iwdg = false;
#if defined(ROBOT_COMPETITION_FINISH) && ROBOT_COMPETITION_FINISH
static_assert(!robot::kEnableFaultInjection, "Competition firmware must not contain fault injection");
const char* volatile g_competition_build_tag = nullptr;
#endif
#if defined(ROBOT_GROUND_TRIAL) && ROBOT_GROUND_TRIAL
static_assert(!robot::kEnableFaultInjection, "Ground trial must not contain fault injection");
const char* volatile g_ground_build_tag = nullptr;
#endif
/* External variables --------------------------------------------------------*/
/* Private function prototypes -----------------------------------------------*/
/* Exported function definitions ---------------------------------------------*/
/* ============================================================================
 * 上电初始化：顺序不能随便调
 * ---------------------------------------------------------------------------
 * 0.   记录复位原因（IWDG？）并清标志——否则下次复位识别不到。
 * 0.5  故障注入归零、保存构建标签——保证符号留在固件里，可离线核对。
 * 1.   构造 8 个电机对象（4 驱动 + 4 转向），配置 ID/方向/减速比。
 * 2.   IMU 只做芯片配置；零漂标定放到之后每 1 ms 的 update 里慢慢做。
 * 3.   把电机、PID、逆解器接进底盘模块（chassis->init）。
 * 4.   把遥控、IMU、底盘、通信接进 Robot（robot->init）。
 * 5.   最后启动通信任务：它一启动就开始收帧 + 喂狗，必须等前面都就绪。
 * ========================================================================== */
void MainTaskInit(void) {
  /* ---------- 0. 记录复位原因 ----------
   * IWDG 复位会置位 RCC 的 IWDGRST 标志。读完之后清掉，
   * 这样下一次复位还能被正确识别。 */
  g_reset_by_iwdg = (__HAL_RCC_GET_FLAG(RCC_FLAG_IWDGRST) != RESET);
  __HAL_RCC_CLEAR_RESET_FLAGS();

  /* ---------- 0.5 故障注入状态自检 ----------
   * 这里是**volatile 写**：编译器不能删除，链接器也就不会把这两个符号回收。
   * 保证任何构建里都能用调试器 / 符号表核实「烧的是不是安全固件」。
   * 同时把 g_fault_id 归零 = 上电默认不注入。 */
  robot::g_fault_id = static_cast<uint32_t>(robot::FaultId::kNone);
  robot::g_fault_injection_enabled = robot::kEnableFaultInjection;

  /* 保存构建标签地址：保证字符串留在 .rodata 里，
   * 可用 strings 在固件文件上离线核对是不是安全版。 */
  robot::g_build_tag_addr = robot::g_build_tag;
#if defined(ROBOT_COMPETITION_FINISH) && ROBOT_COMPETITION_FINISH
  g_competition_build_tag = "COMPETITION-FINISH-CHASSIS-R3";
#endif
#if defined(ROBOT_GROUND_TRIAL) && ROBOT_GROUND_TRIAL
  g_ground_build_tag = robot::ground::kMotionEnabled ? "GROUND-TRIAL-MOTION" : "GROUND-TRIAL-LOCKED";
#endif

  /* ---------- 1. 电机：构造并配置 ID / 方向 / 减速比 ---------- */
  for (int i = 0; i < 4; ++i) {
    (void)GetMotorWheel(i);
    (void)GetMotorSteer(i);
  }

  /* ---------- 2. IMU：配置芯片并启动零漂标定 ----------
   * 注意：这里只初始化，零漂标定是在之后每 1ms 的 update() 里慢慢做的。 */
  hello_world::imu::Imu* imu = GetImu();

  /* ---------- 3. 底盘模块接线 ---------- */
  robot::Chassis* chassis = GetChassis();
  if (chassis != nullptr) {
    hello_world::motor::Motor* wheels[4];
    hello_world::motor::Motor* steers[4];
    hello_world::pid::Pid* wheel_pids[4];
    hello_world::pid::Pid* steer_pids[4];
    for (int i = 0; i < 4; ++i) {
      wheels[i] = GetMotorWheel(i);
      steers[i] = GetMotorSteer(i);
      wheel_pids[i] = GetPidMotorWheel(i);
      steer_pids[i] = GetPidMotorSteer(i);
    }
    chassis->init(wheels, steers, wheel_pids, steer_pids, GetChassisIkSolver());
  }

  /* ---------- 4. Robot 注册 ---------- */
  robot::Robot* robot = GetRobot();
  if (robot != nullptr) {
    robot->setRemoteControl(GetRemoteControl());
    robot->setImu(imu);
    robot->setChassis(chassis);
    robot->setComm(GetGimbalChassisComm());
    robot->init();
  }

  /* ---------- 5. 最后启动通信（会立刻开始收帧 + 喂狗） ---------- */
  CommTaskInit();
}

/* ============================================================================
 * 控制主任务：由调度器每 1 ms 调用一次
 * ---------------------------------------------------------------------------
 * 只做两件事：
 *   1) 故障注入（审计用）：死循环模拟"控制任务卡死"，故意不关中断；
 *      因为喂狗在 CommTask 里，200 ms 内看门狗必然复位——证明喂狗没有
 *      藏在控制循环里。
 *   2) robot->update()：传感器 → 遥控 → 状态机 → 输出 → 通信帧。
 * ========================================================================== */
void MainTask(void) {
  /* ---------- 故障注入：模拟「控制任务卡死」 ----------
   * 注意这里**故意不关中断**：UART/CAN 中断还会继续跑。
   * 这正是审计 H1 描述的场景——如果喂狗写在中断里，板子就不会复位；
   * 现在喂狗只在 CommTask 末尾，所以这里死循环 → 200ms 后必然复位。 */
  if (robot::FaultActive(robot::FaultId::kHangControlLoop)) {
    while (true) {
    }
  }

  robot::Robot* robot = GetRobot();
  if (robot != nullptr) {
    robot->update();
  }
}

/* Private function definitions ----------------------------------------------*/

/**
 * @brief 定时器周期中断回调：整个系统的 1ms 心跳
 * @note  1. 必须 extern "C"（HAL 是 C 链接）；
 *        2. 必须先判断中断源是 TIM6——TIM2 也在跑，别把别的定时器当成控制节拍；
 *        3. 顺序是「先控制、后通信」：本周期算出来的指令，同一周期就发出去；
 *        4. 顺便用 TIM2 量一下这一整轮花了多少 tick（84 tick = 1us），
 *           结果放在 robot::diag::g_cpu 里，验收时读它证明没有超时。
 */
extern "C" void HAL_TIM_PeriodElapsedCallback(TIM_HandleTypeDef* htim) {
  if (htim->Instance == TIM6) {
    const uint32_t start_ticks = __HAL_TIM_GET_COUNTER(&htim2);

    MainTask();
    CommTask();

    const uint32_t elapsed_ticks = __HAL_TIM_GET_COUNTER(&htim2) - start_ticks;
    /* 预算：1ms @ 84MHz = 84000 tick */
    robot::diag::RecordCycle(elapsed_ticks, 84000u);
  }
}
