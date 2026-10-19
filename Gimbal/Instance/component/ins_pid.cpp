/**
 *******************************************************************************
 * @file      :ins_pid.cpp
 * @brief     : 云台板 PID 实例化与初始参数
 * @history   :
 *  Version     Date            Author          Note
 *  V1.0.0      2026-10-19      <author>        1. 完成实例化
 *******************************************************************************
 * @attention :
 *  ★ 下面的参数是「保守的初值」，必须在实车上整定 ★
 *
 *  云台位置环的输入输出单位：
 *    输入：角度误差，单位 rad
 *    输出：力矩，单位 N·m（经达妙 MIT 接口下发，kp / kd 都设为 0）
 *
 *  yaw 用世界系角度（IMU），pitch 用关节角。两者都是「角度误差 → 力矩」，
 *  所以参数形式一样，只是数值不同（pitch 要克服重力，需要更大一些）。
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

/* ---------------- yaw 环 ---------------- */
constexpr float kYawPidKp = 2.5f;    ///< N·m / rad
constexpr float kYawPidKi = 0.6f;    ///< N·m / (rad·s)
constexpr float kYawPidKd = 0.12f;   ///< N·m / (rad/s)
constexpr float kYawPidOutLim = 3.5f;       ///< N·m
constexpr float kYawPidIntegralLim = 1.0f;  ///< 积分项限幅

/* ---------------- pitch 环 ---------------- */
constexpr float kPitchPidKp = 3.0f;
constexpr float kPitchPidKi = 1.0f;
constexpr float kPitchPidKd = 0.15f;
constexpr float kPitchPidOutLim = 3.5f;
constexpr float kPitchPidIntegralLim = 1.5f;  ///< 留出「顶住重力」的积分余量

hello_world::pid::Pid s_yaw_pid;
hello_world::pid::Pid s_pitch_pid;
bool s_inited = false;

void InitPids()
{
  if (s_inited) {
    return;
  }
  s_yaw_pid.init(kYawPidKp, kYawPidKi, kYawPidKd, kYawPidOutLim,
                 kYawPidIntegralLim);
  s_pitch_pid.init(kPitchPidKp, kPitchPidKi, kPitchPidKd, kPitchPidOutLim,
                   kPitchPidIntegralLim);
  /* 达妙的反馈角度量化较细，但位置环输出的微分仍建议低通一下 */
  s_yaw_pid.setDerivativeLpf(0.1f);
  s_pitch_pid.setDerivativeLpf(0.1f);
  s_inited = true;
}

}  // namespace

/* Exported function definitions ---------------------------------------------*/
hello_world::pid::Pid *GetPidMotorYaw()
{
  InitPids();
  return &s_yaw_pid;
}

hello_world::pid::Pid *GetPidMotorPitch()
{
  InitPids();
  return &s_pitch_pid;
}

/* Private function definitions ----------------------------------------------*/
