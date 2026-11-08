/**
 *******************************************************************************
 * @file      :gimbal.cpp
 * @brief     : 云台运动模块实现
 * @history   :
 *  Version     Date            Author          Note
 *  V1.0.0      2026-10-19      <author>        1. 完成编写
 *******************************************************************************
 * @attention :
 *  单位约定：
 *    角度 : rad       力矩 : N·m
 *    达妙 setInput(torque) → MIT 模式下 kp=kd=0 的纯力矩
 *******************************************************************************
 *  Copyright (c) 2024 Hello World Team, Zhejiang University.
 *  All Rights Reserved.
 *******************************************************************************
 */
/* Includes ------------------------------------------------------------------*/
#include "gimbal.hpp"

#include <cmath>

#include "motor_base.hpp"
#include "pid.hpp"
/* Private macro -------------------------------------------------------------*/
/* Private constants ---------------------------------------------------------*/
/** 判定「到限位」的余量，rad */
constexpr float kPitchLimitMargin = 0.02f;
/* Private types -------------------------------------------------------------*/
/* Private variables ---------------------------------------------------------*/
/* External variables --------------------------------------------------------*/
/* Private function prototypes -----------------------------------------------*/
/* Exported function definitions ---------------------------------------------*/

void robot::Gimbal::init(hello_world::motor::Motor* yaw_motor,
                         hello_world::motor::Motor* pitch_motor,
                         hello_world::pid::Pid* yaw_pid,
                         hello_world::pid::Pid* pitch_pid)
{
  yaw_motor_ = yaw_motor;
  pitch_motor_ = pitch_motor;
  yaw_pid_ = yaw_pid;
  pitch_pid_ = pitch_pid;
  ready_ = (yaw_motor_ != nullptr) && (pitch_motor_ != nullptr) &&
           (yaw_pid_ != nullptr) && (pitch_pid_ != nullptr);
  yaw_target_ = 0.0f;
  pitch_target_ = 0.0f;
  pitch_at_limit_ = false;
}

void robot::Gimbal::setPitchTarget(float pitch_joint)
{
  if (!std::isfinite(pitch_joint)) {
    pitch_target_ = pitch_joint;  // 保留无效标记，由 update 整组停机。
    return;
  }
  /* 先限制目标角，避免要求越过软件行程；update 还会按实测角阻止外向力矩。
   * 是否能安全保持姿态、机械边界需要多少制动余量，须通过台架确认。 */
  const float clamped =
      robot::Clamp(pitch_joint, kPitchLowerLimit, kPitchUpperLimit);
  pitch_target_ = clamped;
  pitch_at_limit_ =
      (pitch_joint >= kPitchUpperLimit - kPitchLimitMargin) ||
      (pitch_joint <= kPitchLowerLimit + kPitchLimitMargin);
}

void robot::Gimbal::update(float dt, float yaw_world_meas)
{
  if (!ready_) {
    return;
  }

  // 任一轴反馈或目标异常时整组停机，禁止 NaN 进入 PID 或电机的整数编码。
  // 目标保留异常标记，调用方必须给出新的有效目标才可恢复。
  if (!std::isfinite(dt) || dt <= 0.0f || !std::isfinite(yaw_world_meas) ||
      !std::isfinite(yaw_target_) || !std::isfinite(pitch_target_) ||
      !std::isfinite(yawJoint()) || !std::isfinite(pitchJoint())) {
    stop();
    return;
  }

  /* ---------- yaw：世界角误差 → 力矩 ----------
   * 误差必须先归一化到 ±π：云台可以连续转 360°，如果直接拿 359° 和 1° 相减，
   * 控制器会以为要倒转 358°，电机就会「啪」地反向甩一大圈。 */
  const float yaw_err = robot::WrapPi(yaw_target_ - yaw_world_meas);
  const float yaw_input = yaw_pid_->updateError(yaw_err, dt);

  /* ---------- pitch：关节角误差 → 力矩 ----------
   * pitch 行程小（±1 rad 以内），不存在回绕问题，直接相减。 */
  const float pitch_err = pitch_target_ - pitchJoint();
  float pitch_input = pitch_pid_->updateError(pitch_err, dt);
  if (!std::isfinite(yaw_input) || !std::isfinite(pitch_input)) {
    stop();
    return;
  }
  // 反馈到达软件边界后禁止向外顶住；清积分避免多周期反复施加外向力矩。
  // 允许向内脱离边界；机械限位必须仍留出制动距离并在台架标定。
  if ((pitchJoint() >= kPitchUpperLimit && pitch_input > 0.0f) ||
      (pitchJoint() <= kPitchLowerLimit && pitch_input < 0.0f)) {
    pitch_pid_->reset();
    pitch_input = 0.0f;
  }
  yaw_motor_->setInput(yaw_input);
  pitch_motor_->setInput(pitch_input);
}

void robot::Gimbal::stop()
{
  if (yaw_motor_ != nullptr) {
    yaw_motor_->setInput(0.0f);
  }
  if (pitch_motor_ != nullptr) {
    pitch_motor_->setInput(0.0f);
  }
  if (yaw_pid_ != nullptr && yaw_motor_ != nullptr) {
    yaw_pid_->reset(yaw_motor_->angle());
  }
  if (pitch_pid_ != nullptr && pitch_motor_ != nullptr) {
    pitch_pid_->reset(pitch_motor_->angle());
  }
  pitch_at_limit_ = false;
}

float robot::Gimbal::yawJoint() const
{
  if (yaw_motor_ == nullptr) {
    return 0.0f;
  }
  /* angle() 是 [-π, π) 的关节角；减掉零点偏移后就是 δ（云台相对车头） */
  return robot::WrapPi(yaw_motor_->angle() - kYawJointOffset);
}

float robot::Gimbal::pitchJoint() const
{
  if (pitch_motor_ == nullptr) {
    return 0.0f;
  }
  return pitch_motor_->angle() - kPitchJointOffset;
}

/* Private function definitions ----------------------------------------------*/
