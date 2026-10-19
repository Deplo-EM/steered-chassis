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
  /* 软件限位：先钳位再记录「是否顶到限位」。
   * 注意是「钳位目标」而不是「到限位就断电」——钳住之后电机仍然出力顶住，
   * 这样枪口停在限位上不会往下掉。 */
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

  /* ---------- yaw：世界角误差 → 力矩 ----------
   * 误差必须先归一化到 ±π：云台可以连续转 360°，如果直接拿 359° 和 1° 相减，
   * 控制器会以为要倒转 358°，电机就会「啪」地反向甩一大圈。 */
  const float yaw_err = robot::WrapPi(yaw_target_ - yaw_world_meas);
  yaw_motor_->setInput(yaw_pid_->updateError(yaw_err, dt));

  /* ---------- pitch：关节角误差 → 力矩 ----------
   * pitch 行程小（±1 rad 以内），不存在回绕问题，直接相减。 */
  const float pitch_err = pitch_target_ - pitchJoint();
  pitch_motor_->setInput(pitch_pid_->updateError(pitch_err, dt));
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
