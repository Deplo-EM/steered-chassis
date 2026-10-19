/**
 *******************************************************************************
 * @file      :ins_robot.cpp
 * @brief     : 顶层状态机与底盘模块实例化
 * @history   :
 *  Version     Date            Author          Note
 *  V1.0.0      2026-10-19      <author>        1. 完成实例化
 *******************************************************************************
 * @attention :
 *  Robot 与 Chassis 都必须「函数内静态」定义：对象里含 PID、电机指针等成员，
 *  在 main() 之前构造可能碰到硬件尚未初始化的问题。
 *******************************************************************************
 *  Copyright (c) 2024 Hello World Team, Zhejiang University.
 *  All Rights Reserved.
 *******************************************************************************
 */
/* Includes ------------------------------------------------------------------*/
#include "ins_robot.hpp"

#include "chassis.hpp"
#include "robot.hpp"
/* Private macro -------------------------------------------------------------*/
/* Private constants ---------------------------------------------------------*/
/* Private types -------------------------------------------------------------*/
/* Private variables ---------------------------------------------------------*/
/* External variables --------------------------------------------------------*/
/* Private function prototypes -----------------------------------------------*/
/* Exported function definitions ---------------------------------------------*/
robot::Robot *GetRobot() {
  static robot::Robot s_robot;
  return &s_robot;
}

robot::Chassis *GetChassis() {
  static robot::Chassis s_chassis;
  return &s_chassis;
}

/* Private function definitions ----------------------------------------------*/
