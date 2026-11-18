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
 *  yaw 使用世界系姿态与角速度，pitch 使用关节角与关节转速。
 *  D 只对实际运动加阻尼，扭矩来自反馈控制，没有单独的重力前馈。
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
/* 2026-11-15 调参线：
 * B10 2.2/0.05/0.28：稳定基线（单向疯转消失、无高频振荡）。
 * B11 3.0、B12 2.5：平台段均出现高频振荡 → P 不能再加。
 * B13（本版）：P 回到 B10 的 2.2，另加“速率前馈”（kYawRateFfGain=0.20），
 *     用前馈减小跟随滞后，而不是继续加大 P。 */
constexpr float kYawPidKp = 4.5f;    ///< N·m / rad（收尾要求：更硬）
constexpr float kYawPidKi = 0.05f;   ///< N·m / (rad·s)
constexpr float kYawPidKd = 0.70f;   ///< N·m / (rad/s)（配合 Kp 提高阻尼）
constexpr float kYawPidOutLim = 4.5f;       ///< N·m
constexpr float kYawPidIntegralLim = 0.5f;  ///< 积分项限幅（B10：1.0→0.5）

/* ---------------- pitch 环 ---------------- */
/* 2026-11-15 调参线：
 * A01 3.6/1.0/0.20 → B01 4.2/1.0/0.20（先单独加 P）。
 * B01 实车数据：跟随仍有滞后、反转有峰值；同时下极限饱和把积分灌到 −1.5。
 * B02 先把软件下极限从 −0.9 收到 −0.47（机械止点约 −0.50）。
 * B03 4.8/1.0/0.32：小型动作已跟紧，大阶跃峰值也小；但保持段下垂约 0.08 rad，
 *     输出平顶 1.3 N·m、积分从 0.5 慢爬到 0.9 —— 静摩擦/重力托举，积分充电太慢。
 * B04 4.8/1.6/0.32：保持段仍有约 0.045 rad 稳态偏差，严格指标（≤0.01 rad）不过关。
 * B05 7.0/2.5/0.40：B05 实车出现约 20~40 Hz、±0.2 rad 的抖动（严格标准不允许），
 *     说明 P/D 推过头、D 把反馈噪声放大了。
 * B06 5.8/2.0/0.30（D-LPF 0.05）：已无高频抖动，但阶跃拐角仍有可见滞后。
 * B07（本版，Pitch 最后一版）：P 5.8→6.6 提高带宽；D 低通 0.05→0.08 减少相位滞后；
 *     Ki 2.0、Kd 0.30、下极限 −0.47 不变。之后转 Yaw。 */
constexpr float kPitchPidKp = 11.0f;
constexpr float kPitchPidKi = 2.0f;
constexpr float kPitchPidKd = 0.70f;
constexpr float kPitchPidOutLim = 5.0f;  ///< 回路额定上限；Gimbal 还按实际电机的限值限幅。
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
  /* 实测角速度先低通，避免陀螺与电机转速反馈噪声直接进入力矩。 */
  s_yaw_pid.setDerivativeLpf(0.15f);
  /* B07：Pitch 的 D 低通 0.05→0.08，减少相位滞后；B06 已证明不会振荡。 */
  s_pitch_pid.setDerivativeLpf(0.08f);
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


