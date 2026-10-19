/**
 *******************************************************************************
 * @file      :ins_chassis_gimbal_comm.cpp
 * @brief     : 底盘板双板通信对象实例化
 * @history   :
 *  Version     Date            Author          Note
 *  V1.0.0      2026-10-19      <author>        1. 完成实例化
 *******************************************************************************
 * @attention :
 *  本板是底盘板（CodePart::Chassis）：发送 C2G 帧（0x210），接收 G2C 帧（0x211）。
 *  云台板那份实例化文件必须与此严格互补，否则两板「各说各话」。
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
      robot::GimbalChassisComm::CodePart::Chassis,
      robot::kCommIdChassisToGimbal,   ///< 本板发送
      robot::kCommIdGimbalToChassis);  ///< 本板接收
  return &s_comm;
}

/* Private function definitions ----------------------------------------------*/
