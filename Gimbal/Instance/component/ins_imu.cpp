/**
 *******************************************************************************
 * @file      :ins_imu.cpp
 * @brief     : IMU 实例化与硬件接线配置（云台板）
 * @history   :
 *  Version     Date            Author          Note
 *  V1.0.0      2026-10-19      <author>        1. 完成实例化与引脚配置
 *******************************************************************************
 * @attention :
 *  ★ 两个片选引脚如果接反，现象是 imuInit() 返回「加速度计未找到」★
 *  SPI1 引脚：SCK=PB3 / MISO=PB4 / MOSI=PA7，两个片选是 PA4 与 PB0。
 *  errorState非0或姿态异常时先查供电、SPI和芯片ID；仅在原理图证实接反后修正CS。
 *
 *  云台板的 IMU 装在云台（枪管）上，所以它测到的 yaw 就是「枪口的世界朝向」，
 *  这正是 yaw 世界系稳定所依赖的量。
 *******************************************************************************
 *  Copyright (c) 2024 Hello World Team, Zhejiang University.
 *  All Rights Reserved.
 *******************************************************************************
 */
/* Includes ------------------------------------------------------------------*/
#include "ins_imu.hpp"

#include "imu.hpp"
#include "spi.h"
/* Private macro -------------------------------------------------------------*/
/* Private constants ---------------------------------------------------------*/
/* Private types -------------------------------------------------------------*/
/* Private variables ---------------------------------------------------------*/
/* External variables --------------------------------------------------------*/
/* Private function prototypes -----------------------------------------------*/
/* Exported function definitions ---------------------------------------------*/

hello_world::imu::Imu *GetImu() {
  static hello_world::imu::Imu s_imu;
  static bool s_inited = false;

  if (!s_inited) {
    hello_world::imu::BMI088HWConfig hw;
    hw.hspi = &hspi1;
    hw.acc_cs_port = GPIOA;   ///< 加速度计片选
    hw.acc_cs_pin = GPIO_PIN_4;
    hw.gyro_cs_port = GPIOB;  ///< 陀螺仪片选
    hw.gyro_cs_pin = GPIO_PIN_0;

    (void)s_imu.init(hw, hello_world::imu::kImuDefaultRotMat);
    s_inited = true;
  }

  return &s_imu;
}

/* Private function definitions ----------------------------------------------*/
