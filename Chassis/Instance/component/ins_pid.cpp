/**
 *******************************************************************************
 * @file      :ins_pid.cpp
 * @brief     : 底盘板 PID 实例化与初始参数
 * @history   :
 *  Version     Date            Author          Note
 *  V1.0.0      2026-10-19      <author>        1. 完成实例化
 *******************************************************************************
 * @attention :
 *  ★ 下面的参数是「保守的初值」，必须在实车上按 tests/ 文档里的整定步骤调 ★
 *
 *  两组环的区别：
 *    - 轮速环：输入误差单位 rad/s（轮子转速），输出单位 A（电流）。
 *      1 m/s 车速 ≈ 15.2 rad/s 轮速（R=0.066m）。Kp 决定「差多少速度补多少电流」。
 *    - 舵角环：输入误差单位 rad，输出单位 raw（GM6020 电压，±25000）。
 *      GM6020 是电压驱动，本身带反电动势阻尼，所以纯 P 也能稳定，
 *      Kd 用来抑制大角度回摆时的超调（考核加分项「舵轮回摆抑制」）。
 *******************************************************************************
 *  Copyright (c) 2024 Hello World Team, Zhejiang University.
 *  All Rights Reserved.
 *******************************************************************************
 */
/* Includes ------------------------------------------------------------------*/
#include "ins_pid.hpp"

#include "pid.hpp"
/* Private macro -------------------------------------------------------------*/
/* Private constants ---------------------------------------------------------*/

namespace {

/* ---------------- 轮速环（4 个，参数相同） ---------------- */
constexpr float kWheelPidKp = 0.35f;   ///< A / (rad/s)
constexpr float kWheelPidKi = 1.50f;   ///< A / (rad/s·s)，消除稳态误差
constexpr float kWheelPidKd = 0.0f;    ///< 速度环一般不用微分（反馈本来就抖）
constexpr float kWheelPidOutLim = 8.0f;   ///< 输出限幅，A（与电机电流限幅一致）
constexpr float kWheelPidIntegralLim = 2.5f;  ///< 积分项限幅（等效最多贡献 3.75A）

/* ---------------- 舵角环（4 个，参数相同） ---------------- */
constexpr float kSteerPidKp = 10000.0f;  ///< raw / rad
constexpr float kSteerPidKi = 2000.0f;   ///< raw / (rad·s)
constexpr float kSteerPidKd = 150.0f;    ///< raw / (rad/s)
constexpr float kSteerPidOutLim = 20000.0f;    ///< 电压限幅
constexpr float kSteerPidIntegralLim = 2000.0f;  ///< 积分限幅（等效最多 2000 raw）

hello_world::pid::Pid s_wheel_pids[4];
hello_world::pid::Pid s_steer_pids[4];
bool s_inited = false;

void InitPids()
{
  if (s_inited) {
    return;
  }
  for (int i = 0; i < 4; ++i) {
    s_wheel_pids[i].init(kWheelPidKp, kWheelPidKi, kWheelPidKd,
                         kWheelPidOutLim, kWheelPidIntegralLim);
    s_steer_pids[i].init(kSteerPidKp, kSteerPidKi, kSteerPidKd,
                         kSteerPidOutLim, kSteerPidIntegralLim);
    /* 舵角反馈有量化噪声（8192 线），微分再做一次强低通 */
    s_steer_pids[i].setDerivativeLpf(0.08f);
  }
  s_inited = true;
}

}  // namespace

/* Exported function definitions ---------------------------------------------*/
hello_world::pid::Pid *GetPidMotorWheel(int idx) {
  InitPids();
  if (idx < 0 || idx >= 4) {
    return nullptr;
  }
  return &s_wheel_pids[idx];
}

hello_world::pid::Pid *GetPidMotorSteer(int idx) {
  InitPids();
  if (idx < 0 || idx >= 4) {
    return nullptr;
  }
  return &s_steer_pids[idx];
}

/* Private function definitions ----------------------------------------------*/
