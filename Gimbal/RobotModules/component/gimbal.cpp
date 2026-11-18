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
#include "critical_section.hpp"
#include "gimbal_debug.hpp"

namespace robot::diag {
volatile GimbalDebug g_gimbal_dbg = {};
}  // namespace robot::diag
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
  yaw_rate_ff_ = 0.0f;
  pitch_target_ = 0.0f;
  pitch_at_limit_ = false;
  feedback_valid_=feedback_pending_=feedback_healthy_=false;
  feedback_finite_ = control_valid_ = false;
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

void robot::Gimbal::captureFeedback()
{
  CriticalSection lock;
  yaw_feedback_ = yaw_motor_ ? yaw_motor_->angle() : 0;
  pitch_feedback_ = pitch_motor_ ? pitch_motor_->angle() : 0;
  pitch_joint_rate_ = pitch_motor_ ? pitch_motor_->vel() : 0.0f;
  feedback_finite_ = ready_ && std::isfinite(yaw_feedback_) &&
                     std::isfinite(pitch_feedback_) && std::isfinite(pitch_joint_rate_);
  feedback_healthy_ = feedback_finite_ && yaw_motor_->isHealthy() && pitch_motor_->isHealthy();
  feedback_pending_=true;
  feedback_valid_=true;
}

void robot::Gimbal::update(float dt, float yaw_world_meas, float yaw_world_rate)
{
  if (!ready_) {
    return;
  }

  if (!feedback_pending_) captureFeedback();
  feedback_pending_=false;
  if (!feedback_healthy_) { stop(); return; }
  const float yaw_feedforward = kYawRateFfGain * yaw_rate_ff_;
  const float yaw_torque_limit = yaw_motor_->motor_info().torq_input_lim;
  const float pitch_torque_limit = pitch_motor_->motor_info().torq_input_lim;
  const float pitch_joint = pitchJoint();
  // 任一轴反馈或目标异常时整组停机，禁止 NaN 进入 PID 或电机的整数编码。
  // 目标保留异常标记，调用方必须给出新的有效目标才可恢复。
  if (!std::isfinite(dt) || dt <= 0.0f || !std::isfinite(yaw_world_meas) ||
      !std::isfinite(yaw_world_rate) || !std::isfinite(pitch_joint_rate_) ||
      !std::isfinite(yaw_target_) || !std::isfinite(pitch_target_) ||
      !std::isfinite(yaw_feedforward) ||
      !std::isfinite(yaw_torque_limit) || yaw_torque_limit <= 0.0f ||
      !std::isfinite(pitch_torque_limit) || pitch_torque_limit <= 0.0f ||
      !std::isfinite(yawJoint()) || !std::isfinite(pitch_joint)) {
    stop();
    return;
  }

  /* ---------- yaw：世界角误差 → 力矩 ----------
   * 误差必须先归一化到 ±π：云台可以连续转 360°，如果直接拿 359° 和 1° 相减，
   * 控制器会以为要倒转 358°，电机就会「啪」地反向甩一大圈。 */
  yaw_world_feedback_ = yaw_world_meas;
  const float yaw_err = robot::WrapPi(yaw_target_ - yaw_world_feedback_);
  // 合计力矩先限幅并做积分抗饱和，避免正常大角差触发驱动的越界返回。
  const float yaw_input = yaw_pid_->updateWithFeedforward(
      yaw_err, yaw_world_rate, dt, yaw_feedforward, yaw_torque_limit);

  /* ---------- pitch：关节角误差 → 力矩 ----------
   * pitch 行程小（±1 rad 以内），不存在回绕问题，直接相减。 */
  const float pitch_err = pitch_target_ - pitch_joint;
  float pitch_feedforward = 0.0f;
#if defined(ROBOT_PITCH_GRAVITY_FF) && ROBOT_PITCH_GRAVITY_FF
  /* 重力前馈（构建开关）：抬平/保持时直接补住重力力矩，不再只靠积分慢慢攒。 */
  constexpr float kPitchGravityFf = 1.4f;
  pitch_feedforward = kPitchGravityFf * std::cos(pitch_joint);
#endif
  // 重力项和反馈一起受执行器限幅，正常饱和不再触发 setInput 越界。
  float pitch_input = pitch_pid_->updateWithFeedforward(
      pitch_err, pitch_joint_rate_, dt, pitch_feedforward, pitch_torque_limit);
  if (!std::isfinite(yaw_input) || !std::isfinite(pitch_input)) {
    stop();
    return;
  }
  // 反馈到达软件边界后禁止向外顶住；清积分避免多周期反复施加外向力矩。
  // 允许向内脱离边界；机械限位必须仍留出制动距离并在台架标定。
  pitch_at_limit_ = pitch_target_ >= kPitchUpperLimit - kPitchLimitMargin ||
                    pitch_target_ <= kPitchLowerLimit + kPitchLimitMargin ||
                    pitch_joint >= kPitchUpperLimit - kPitchLimitMargin ||
                    pitch_joint <= kPitchLowerLimit + kPitchLimitMargin;
  if ((pitch_joint >= kPitchUpperLimit && pitch_input > 0.0f) ||
      (pitch_joint <= kPitchLowerLimit && pitch_input < 0.0f)) {
    pitch_pid_->reset();
    pitch_input = 0.0f;
  }
  using hello_world::motor::Status;
  if (yaw_motor_->setInput(yaw_input) != Status::kOk ||
      pitch_motor_->setInput(pitch_input) != Status::kOk) {
    stop();
    return;
  }
  control_valid_ = true;
  diag::g_gimbal_dbg.yaw_err = yaw_err;
  diag::g_gimbal_dbg.yaw_ff_torque = yaw_feedforward;
  updateDebug();
}

void robot::Gimbal::updateDebug()
{
  auto& d = diag::g_gimbal_dbg;
  d.pitch_target = pitch_target_;
  d.pitch_feedback = pitchJoint();
  d.pitch_output = pitch_pid_ ? pitch_pid_->output() : 0.0f;
  d.pitch_integral = pitch_pid_ ? pitch_pid_->integral() : 0.0f;
  d.pitch_torq_meas = pitch_motor_ ? pitch_motor_->torq() : 0.0f;
  d.yaw_target = yaw_target_;
  d.yaw_world_feedback = yaw_world_feedback_;
  d.yaw_output = yaw_pid_ ? yaw_pid_->output() : 0.0f;
  d.yaw_integral = yaw_pid_ ? yaw_pid_->integral() : 0.0f;
}

void robot::Gimbal::stop()
{
  control_valid_ = false;
  feedback_pending_=false;
  yaw_rate_ff_ = 0.0f;
  diag::g_gimbal_dbg.yaw_err = 0.0f;
  diag::g_gimbal_dbg.yaw_ff_torque = 0.0f;
  if (yaw_motor_ != nullptr) {
    yaw_motor_->setInput(0.0f);
  }
  if (pitch_motor_ != nullptr) {
    pitch_motor_->setInput(0.0f);
  }
  resetControllers();
  pitch_at_limit_ = false;
  updateDebug();
}

void robot::Gimbal::resetControllers()
{
  if (yaw_pid_ != nullptr) yaw_pid_->reset();
  if (pitch_pid_ != nullptr) pitch_pid_->reset();
}

float robot::Gimbal::yawJoint() const
{
  if (yaw_motor_ == nullptr) {
    return 0.0f;
  }
  /* angle() 是 [-π, π) 的关节角；减掉零点偏移后就是 δ（云台相对车头） */
  if (feedback_valid_) return robot::WrapPi(yaw_feedback_ - kYawJointOffset);
  CriticalSection lock;
  return robot::WrapPi(yaw_motor_->angle() - kYawJointOffset);
}

float robot::Gimbal::pitchJoint() const
{
  if (pitch_motor_ == nullptr) {
    return 0.0f;
  }
  if (feedback_valid_) return pitch_feedback_ - kPitchJointOffset;
  CriticalSection lock;
  return pitch_motor_->angle() - kPitchJointOffset;
}

/* Private function definitions ----------------------------------------------*/
