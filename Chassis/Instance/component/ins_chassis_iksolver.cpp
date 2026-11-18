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
 *  考核说明书 6.2：轮半径 0.066m、四个轮组坐标 (±0.167, ±0.167)m）。实车量出来不一样就改那里。
 *******************************************************************************
 *  Copyright (c) 2024 Hello World Team, Zhejiang University.
 *  All Rights Reserved.
 *******************************************************************************
 */
/* Includes ------------------------------------------------------------------*/
#include "ins_chassis_iksolver.hpp"

#include "chassis_ik_solver.hpp"

namespace {

// 逐轮填写隔离动力后核实的干涉区。顺序 LF/LB/RB/RF，单位 rad。
// 每项为 {中心, 半宽（含实测裕量）}，半宽 0 表示没有启用该条路径约束。
// 不清楚轮位或角度时保留 0，且不要据此判定这台车已能绕开刮擦。
constexpr hello_world::chassis_ik_solver::Geometry kGeometry{
    .steer_keepout = { {}, {}, {}, {} },
};

}  // namespace
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
#if defined(ROBOT_COMPETITION_FINISH) && ROBOT_COMPETITION_FINISH
    // 只提高轨迹限速：90° 参考最短约 131 ms；实物速度还受负载、PID 和限流约束。
    constexpr float kSteerRate = 12.0f;
#else
    constexpr float kSteerRate = 10.0f;
#endif
    s_solver.init(kGeometry, kSteerRate, 3.0f);
    s_inited = true;
  }

  return &s_solver;
}

/* Private function definitions ----------------------------------------------*/
