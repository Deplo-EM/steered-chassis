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
 *    - 舵角环：输入误差单位 rad，输出沿用电压命令的 raw 数值标度。
 *      电流版在传给电机之前换到 A；现场初测的映射和上限见 steer_output_profile.hpp。
 *******************************************************************************
 *  Copyright (c) 2024 Hello World Team, Zhejiang University.
 *  All Rights Reserved.
 *******************************************************************************
 */
/* Includes ------------------------------------------------------------------*/
#include "ins_pid.hpp"

#include "pid.hpp"
#include "steer_output_profile.hpp"
#if defined(ROBOT_GROUND_TRIAL) && ROBOT_GROUND_TRIAL
#include "ground_trial.hpp"
#endif
/* Private macro -------------------------------------------------------------*/
/* Private constants ---------------------------------------------------------*/

namespace {

/* ---------------- 轮速环（4 个，参数相同） ---------------- */
/* 2026-11-16 实车：空载时轮速在 ±30 rad/s、约 0.5 Hz 来回摆，电流只有 ±2 A。
 * 2 A 就能把空载轮推到 ±30 rad/s，说明是积分主导的低频极限环，不是推力不足。
 * B02：Kp 0.35→0.50、Ki 1.50→0.30，改比例主导、积分只修稳态。 */
constexpr float kWheelPidKp = 0.50f;   ///< A / (rad/s)
constexpr float kWheelPidKi = 0.30f;   ///< A / (rad/s·s)，消除稳态误差
constexpr float kWheelPidKd = 0.0f;    ///< 速度环一般不用微分（反馈本来就抖）
#if defined(ROBOT_GROUND_TRIAL) && ROBOT_GROUND_TRIAL
constexpr float kWheelPidOutLim = robot::ground::kWheelCurrentA;
#else
constexpr float kWheelPidOutLim = 8.0f;   ///< 输出限幅，A（与电机电流限幅一致）
#endif
constexpr float kWheelPidIntegralLim = 2.5f;  ///< 积分状态限幅；Ki × 限值 = 0.75 A。

/* ---------------- 舵角环（4 个，参数相同） ---------------- */
/* 电流版沿用 B01 的参数，电压版保留原参数。降增益后仍有振荡，
 * 所以不能仅凭输出翻号归因于 D 项；轨迹与对齐门另行验证。 */
#if defined(ROBOT_COMPETITION_FINISH) && ROBOT_COMPETITION_FINISH
// A/B 候选 A：R3 基线 7200/800/150 → 12000/2000/240。
// 实车架空实测：R3 输出仅 0.2~0.3 A（上限 1.2 A），转向跟不上 2 rad/s 目标。
constexpr float kSteerPidKp = 12000.0f;
constexpr float kSteerPidKi = 2000.0f;
constexpr float kSteerPidKd = 240.0f;
#else
constexpr float kSteerPidKp = robot::steer_output::kUseCurrent ? 6000.0f : 10000.0f;
constexpr float kSteerPidKi = robot::steer_output::kUseCurrent ? 400.0f : 2000.0f;
constexpr float kSteerPidKd = robot::steer_output::kUseCurrent ? 120.0f : 150.0f;
#endif
constexpr float kSteerPidOutLim = robot::steer_output::kPidOutputLimit;  ///< raw
constexpr float kSteerPidIntegralLim = 1.0f;  ///< rad·s；输出积分上限等于 Ki 乘此值。

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
    /* 电流版使用转速反馈；电压版保留原有的角度差分与滤波。 */
    s_steer_pids[i].setDerivativeLpf(robot::steer_output::kUseCurrent ? 0.15f : 0.08f);
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
