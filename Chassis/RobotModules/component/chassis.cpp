/**
 *******************************************************************************
 * @file      :chassis.cpp
 * @brief     : 底盘运动模块实现
 * @history   :
 *  Version     Date            Author          Note
 *  V1.0.0      2026-10-19      <author>        1. 完成编写
 *******************************************************************************
 * @attention :
 *  轮速环和舵角环的单位约定（很容易搞错，写在这里随时对照）：
 *    - 逆解输出 speed  : 轮子线速度 m/s
 *    - 轮子转速 ω      : speed / R，单位 rad/s（电机输出轴转速，因为轮子直接
 *                        装在减速箱输出轴上，中间没有别的传动）
 *    - 轮速环反馈      : motor->vel()，单位 rad/s（已按减速比折算到输出轴）
 *    - 轮速环输出      : 电流 A，交给 motor->setInput()
 *    - 舵角环误差      : rad（已归一化到 ±π）
 *    - 舵角环输出      : GM6020 电压 raw（±25000）
 *******************************************************************************
 *  Copyright (c) 2024 Hello World Team, Zhejiang University.
 *  All Rights Reserved.
 *******************************************************************************
 */
/* Includes ------------------------------------------------------------------*/
#include "chassis.hpp"

#include <cmath>

#include "motor_base.hpp"
#include "critical_section.hpp"
/* Private macro -------------------------------------------------------------*/
/* Private constants ---------------------------------------------------------*/
/* Private types -------------------------------------------------------------*/
/* Private variables ---------------------------------------------------------*/
/* External variables --------------------------------------------------------*/
/* Private function prototypes -----------------------------------------------*/
/* Exported function definitions ---------------------------------------------*/

void robot::Chassis::init(hello_world::motor::Motor* wheels[kWheelNum],
                          hello_world::motor::Motor* steers[kWheelNum],
                          hello_world::pid::Pid* wheel_pids[kWheelNum],
                          hello_world::pid::Pid* steer_pids[kWheelNum],
                          hello_world::chassis_ik_solver::ChassisIkSolver* solver)
{
  ready_ = true;
  for (int i = 0; i < kWheelNum; ++i) {
    wheels_[i] = wheels[i];
    steers_[i] = steers[i];
    wheel_pids_[i] = wheel_pids[i];
    steer_pids_[i] = steer_pids[i];
    if (wheels_[i] == nullptr || steers_[i] == nullptr ||
        wheel_pids_[i] == nullptr || steer_pids_[i] == nullptr) {
      ready_ = false;
    }
  }
  solver_ = solver;
  if (solver_ == nullptr) {
    ready_ = false;
  }
  motion_.reset();
  delta_ = 0.0f;
}

void robot::Chassis::setMotion(const ChassisMotion& motion, float delta)
{
  motion_ = motion;
  delta_ = delta;
}

const hello_world::chassis_ik_solver::WheelCommand& robot::Chassis::wheelCommand(
    int index) const
{
  static const hello_world::chassis_ik_solver::WheelCommand kZero;
  if (solver_ == nullptr) {
    return kZero;
  }
  return solver_->command(index);
}

void robot::Chassis::captureFeedback()
{
  CriticalSection lock;
  feedback_healthy_ = ready_;
  for (int i=0;i<kWheelNum;++i) {
    if (!wheels_[i] || !steers_[i]) { feedback_healthy_=false; continue; }
    steer_feedback_[i]=steers_[i]->angle();
    wheel_velocity_[i]=wheels_[i]->vel();
    feedback_healthy_ = feedback_healthy_ && wheels_[i]->isHealthy() && steers_[i]->isHealthy();
  }
  feedback_pending_=true;
}

void robot::Chassis::update(float dt)
{
  if (!ready_) {
    return;
  }

  if (!feedback_pending_) captureFeedback();
  feedback_pending_=false;
  if (!feedback_healthy_) { stop(); return; }
  // 先读取并检查整组反馈，再算全部输出；任一数值异常都不能只停一只轮。
  float steer_now[kWheelNum], wheel_velocity[kWheelNum];
  for (int i = 0; i < kWheelNum; ++i) {
    steer_now[i] = steer_feedback_[i];
    wheel_velocity[i] = wheel_velocity_[i];
    if (!std::isfinite(steer_now[i]) || !std::isfinite(wheel_velocity[i])) {
      stop(); return;
    }
  }
  hello_world::chassis_ik_solver::WheelCommand cmd[kWheelNum];
  solver_->solve(motion_, delta_, steer_now, dt, cmd);
  if (!solver_->lastSolveValid()) { stop(); return; }

  const float radius = solver_->geometry().wheel_radius;
  float current[kWheelNum], voltage[kWheelNum];
  for (int i = 0; i < kWheelNum; ++i) {
    const float steer_err = robot::WrapPi(cmd[i].angle - steer_now[i]);
    const float drive_err = robot::WrapPi(cmd[i].desired_angle - steer_now[i]);
    if (std::fabs(drive_err) > kSteerReadyAngle) {
      // 未到最终方向时直接撤销驱动力和旧积分；只把轮速目标设零仍会残留积分出力。
      wheel_pids_[i]->reset();
      current[i] = 0.0f;
    } else {
      // 目标沿实测滚动方向投影，减轻小角度未对齐时的轮胎侧向拖拽。
      const float omega_target = cmd[i].speed / radius * std::cos(drive_err);
      current[i] = wheel_pids_[i]->update(omega_target, wheel_velocity[i], dt);
    }
    voltage[i] = steer_pids_[i]->updateError(steer_err, dt);
    if (!std::isfinite(current[i]) || !std::isfinite(voltage[i])) { stop(); return; }
  }
  for (int i = 0; i < kWheelNum; ++i) {
    wheels_[i]->setInput(current[i]);
    steers_[i]->setInput(voltage[i]);
  }
}

void robot::Chassis::stop()
{
  feedback_pending_=false;
  motion_.reset();
  delta_ = 0.0f;
  for (int i = 0; i < kWheelNum; ++i) {
    if (wheels_[i]) wheels_[i]->setInput(0.0f);
    if (steers_[i]) steers_[i]->setInput(0.0f);
    // reset 后首帧跳过微分；不把失效反馈存进控制历史。
    if (wheel_pids_[i]) wheel_pids_[i]->reset();
    if (steer_pids_[i]) steer_pids_[i]->reset();
  }
  // 清历史，下一次 solve 从当时实测舵角开始，允许停机期间手动转动舵轮。
  if (solver_) solver_->reset();
}

/* Private function definitions ----------------------------------------------*/
