/**
 *******************************************************************************
 * @file      :main_task.cpp
 * @brief     : 云台板任务入口（初始化装配 + 1ms 周期调度）
 * @history   :
 *  Version     Date            Author          Note
 *  V1.0.0      2026-10-19      <author>        1. 完成编写
 *******************************************************************************
 * @attention :
 *  装配顺序：电机 → IMU → 云台模块 → Robot → 通信。
 *  通信必须最后启动：它一开就会开始收帧、喂狗。
 *******************************************************************************
 *  Copyright (c) 2024 Hello World Team, Zhejiang University.
 *  All Rights Reserved.
 *******************************************************************************
 */
/* Includes ------------------------------------------------------------------*/
#include "main_task.hpp"

#include "comm_task.hpp"
#include "diagnostics.hpp"
#include "fault_injection.hpp"
#include "gimbal.hpp"
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
 * 读这个变量应为 true。
 */
volatile bool g_reset_by_iwdg = false;

/* External variables --------------------------------------------------------*/
/* Private function prototypes -----------------------------------------------*/
/* Exported function definitions ---------------------------------------------*/
void MainTaskInit(void) {
  /* ---------- 0. 记录复位原因 ---------- */
  g_reset_by_iwdg = (__HAL_RCC_GET_FLAG(RCC_FLAG_IWDGRST) != RESET);
  __HAL_RCC_CLEAR_RESET_FLAGS();

  /* ---------- 0.5 故障注入状态自检 ----------
   * volatile 写：编译器不能删除，链接器不会回收符号，
   * 保证任何构建里都能核实「烧的是不是安全固件」；同时把注入项归零。 */
  robot::g_fault_id = static_cast<uint32_t>(robot::FaultId::kNone);
  robot::g_fault_injection_enabled = robot::kEnableFaultInjection;

  /* 保存构建标签地址：保证字符串留在 .rodata 里，
   * 可用 strings 在固件文件上离线核对是不是安全版。 */
  robot::g_build_tag_addr = robot::g_build_tag;

  /* ---------- 1. 两个达妙电机 ---------- */
  hello_world::motor::Motor* yaw_motor = GetMotorYaw();
  hello_world::motor::Motor* pitch_motor = GetMotorPitch();

  /* ---------- 2. IMU ---------- */
  hello_world::imu::Imu* imu = GetImu();

  /* ---------- 3. 云台模块接线 ---------- */
  robot::Gimbal* gimbal = GetGimbal();
  if (gimbal != nullptr) {
    gimbal->init(yaw_motor, pitch_motor, GetPidMotorYaw(), GetPidMotorPitch());
  }

  /* ---------- 4. Robot 注册 ---------- */
  robot::Robot* robot = GetRobot();
  if (robot != nullptr) {
    robot->setRemoteControl(GetRemoteControl());
    robot->setImu(imu);
    robot->setGimbal(gimbal);
    robot->setComm(GetGimbalChassisComm());
    robot->init();
  }

  /* ---------- 5. 通信 ---------- */
  CommTaskInit();
}

void MainTask(void) {
  /* ---------- 故障注入：模拟「控制任务卡死」（详见 docs/08） ----------
   * 故意不关中断，用来证明喂狗确实只发生在控制周期末尾。 */
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
 * @brief 定时器周期中断回调：1ms 心跳
 * @note  必须 extern "C"，且先确认中断源是 TIM6；
 *        顺便用 TIM2 量这一整轮的耗时（84 tick = 1us），结果在
 *        robot::diag::g_cpu 里，验收时读它证明没有超时。
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
