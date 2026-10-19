/**
 *******************************************************************************
 * @file      :ins_chassis_gimbal_comm.cpp
 * @brief     : 云台板双板通信对象实例化
 * @history   :
 *  Version     Date            Author          Note
 *  V1.0.0      2026-10-19      <author>        1. 完成实例化
 *******************************************************************************
 * @attention :
 *  本板是云台板（CodePart::Gimbal）：发送 G2C 帧（0x211），接收 C2G 帧（0x210）。
 *  与底盘板那份必须严格互补，否则两板「各说各话」。
 *******************************************************************************
 *  Copyright (c) 2024 Hello World Team, Zhejiang University.
 *  All Rights Reserved.
 *******************************************************************************
 */
/* Includes ------------------------------------------------------------------*/
#include "ins_chassis_gimbal_comm.hpp"
#include "gimbal_chassis_comm.hpp"
/* Private macro -------------------------------------------------------------*/
/* Private constants ---------------------------------------------------------*/
/* Private types -------------------------------------------------------------*/
/* Private variables ---------------------------------------------------------*/
/* External variables --------------------------------------------------------*/
/* Private function prototypes -----------------------------------------------*/
/* Exported function definitions ---------------------------------------------*/
robot::GimbalChassisComm *GetGimbalChassisComm() {
  static robot::GimbalChassisComm s_comm(
      robot::GimbalChassisComm::CodePart::Gimbal,
      robot::kCommIdGimbalToChassis,   ///< 本板发送
      robot::kCommIdChassisToGimbal);  ///< 本板接收
  return &s_comm;
}

/* Private function definitions ----------------------------------------------*/
