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
#include "gimbal.hpp"
#include "gimbal_chassis_comm.hpp"
#include "imu.hpp"
#include "ins_all.hpp"
#include "motor.hpp"
#include "pid.hpp"
#include "robot.hpp"
#include "tim.h"
/* Private macro -------------------------------------------------------------*/
/* Private constants ---------------------------------------------------------*/
/* Private types -------------------------------------------------------------*/
/* Private variables ---------------------------------------------------------*/
/* External variables --------------------------------------------------------*/
/* Private function prototypes -----------------------------------------------*/
/* Exported function definitions ---------------------------------------------*/
void MainTaskInit(void) {
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
  robot::Robot* robot = GetRobot();
  if (robot != nullptr) {
    robot->update();
  }
}

/* Private function definitions ----------------------------------------------*/

/**
 * @brief 定时器周期中断回调：1ms 心跳
 * @note  必须 extern "C"，且先确认中断源是 TIM6。
 */
extern "C" void HAL_TIM_PeriodElapsedCallback(TIM_HandleTypeDef* htim) {
  if (htim->Instance == TIM6) {
    MainTask();
    CommTask();
  }
}
