/**
 *******************************************************************************
 * @file      :ins_imu.cpp
 * @brief     : IMU 实例化与硬件接线配置
 * @history   :
 *  Version     Date            Author          Note
 *  V1.0.0      2026-10-19      <author>        1. 完成实例化与引脚配置
 *******************************************************************************
 * @attention :
 *  ★ 两个片选引脚如果接反，现象是 imuInit() 返回「加速度计未找到」★
 *  BMI088 有两个独立片选（加速度计 CS、陀螺仪 CS）。本车 SPI1 引脚：
 *  SCK=PB3 / MISO=PB4 / MOSI=PA7，两个片选是 PA4 与 PB0。
 *  errorState非0或姿态异常时先查供电、SPI和芯片ID；仅在原理图证实接反后修正CS。
 *
 *  安装方向：若 IMU 不是「x 朝前、y 朝左、z 朝上」安装的，改传给 init() 的
 *  旋转矩阵（默认单位矩阵 kImuDefaultRotMat）。
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

    /* 返回 false 表示芯片没找到 / 自检失败。此时 IMU 永远不会 isCalibrated()，
     * 状态机会一直停在 Dead、电机不会动——这是故意的「失效即安全」。 */
    (void)s_imu.init(hw, hello_world::imu::kImuDefaultRotMat);
    s_inited = true;
  }

  return &s_imu;
}

/* Private function definitions ----------------------------------------------*/
