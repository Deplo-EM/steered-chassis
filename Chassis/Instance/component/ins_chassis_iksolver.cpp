/**
 *******************************************************************************
 * @file      :ins_chassis_iksolver.cpp
 * @brief     : 底盘运动学逆解器实例化
 * @history   :
 *  Version     Date            Author          Note
 *  V1.0.0      2026-10-19      <author>        1. 完成实例化
 *******************************************************************************
 * @attention :
 *  几何参数写在 chassis_ik_solver.hpp 的 Geometry 结构体里（默认值已按
 *  「轮径 104mm、舵电机矩形 384×310mm」配置）。实车量出来不一样就改那里。
 *******************************************************************************
 *  Copyright (c) 2024 Hello World Team, Zhejiang University.
 *  All Rights Reserved.
 *******************************************************************************
 */
/* Includes ------------------------------------------------------------------*/
#include "ins_chassis_iksolver.hpp"

#include "chassis_ik_solver.hpp"
/* Private macro -------------------------------------------------------------*/
/* Private constants ---------------------------------------------------------*/
/* Private types -------------------------------------------------------------*/
/* Private variables ---------------------------------------------------------*/
/* External variables --------------------------------------------------------*/
/* Private function prototypes -----------------------------------------------*/
/* Exported function definitions ---------------------------------------------*/
hello_world::chassis_ik_solver::ChassisIkSolver *GetChassisIkSolver() {
  static hello_world::chassis_ik_solver::ChassisIkSolver s_solver;
  static bool s_inited = false;

  if (!s_inited) {
    hello_world::chassis_ik_solver::Geometry geo;
    /* 舵角最大变化率 10 rad/s：从 0° 转到 90° 约 0.16s，
     * 足够快又不会让舵机「啪」地一下甩过去。 */
    s_solver.init(geo, 10.0f, 3.0f);
    s_inited = true;
  }

  return &s_solver;
}

/* Private function definitions ----------------------------------------------*/
