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

void robot::Chassis::update(float dt)
{
  if (!ready_) {
    return;
  }

  /* ---------- 1. 逆解：速度指令 → 每个轮组的 (舵角, 轮速) ---------- */
  float steer_now[kWheelNum];
  for (int i = 0; i < kWheelNum; ++i) {
    steer_now[i] = steers_[i]->angle();
  }

  hello_world::chassis_ik_solver::WheelCommand cmd[kWheelNum];
  solver_->solve(motion_, delta_, steer_now, dt, cmd);

  /* ---------- 2. 逐个轮组闭环 ---------- */
  const float radius = solver_->geometry().wheel_radius;

  for (int i = 0; i < kWheelNum; ++i) {
    /* 舵角误差：目标角是「连续角」，反馈是 [−π, π)，所以必须归一化差值 */
    const float steer_err = robot::WrapPi(cmd[i].angle - steer_now[i]);

    /* 舵角没到位就先不给轮子出力（详见 hpp 里的说明）。
     * 到位后按 cos(误差) 缩放：轮子斜着走时，它对「车身前进」的有效分量只有
     * cos 那么多，提前缩一点能让起步更平顺。 */
    float align = 1.0f;
    const float abs_err = std::fabs(steer_err);
    if (abs_err > kSteerReadyAngle) {
      align = 0.0f;
    } else {
      align = std::cos(steer_err);
    }

    /* 轮速环：目标角速度 = 线速度 / 轮半径 */
    const float omega_target = (cmd[i].speed / radius) * align;
    const float current =
        wheel_pids_[i]->update(omega_target, wheels_[i]->vel(), dt);
    wheels_[i]->setInput(current);

    /* 舵角环：角度误差 → 电压。用 updateError 是为了把我们算好的
     * （已经归一化过的）误差直接送进去，避免 PID 内部再做一次减法。 */
    const float voltage = steer_pids_[i]->updateError(steer_err, dt);
    steers_[i]->setInput(voltage);
  }
}

void robot::Chassis::stop()
{
  if (!ready_) {
    return;
  }
  motion_.reset();
  delta_ = 0.0f;

  float steer_now[kWheelNum];
  for (int i = 0; i < kWheelNum; ++i) {
    wheels_[i]->setInput(0.0f);
    steers_[i]->setInput(0.0f);
    /* reset 时把「上次反馈」设成当前值：否则恢复的瞬间会因误差突变产生
     * 一个巨大的微分尖峰（电机「抽」一下）。 */
    wheel_pids_[i]->reset(wheels_[i]->vel());
    steer_pids_[i]->reset(steers_[i]->angle());
    steer_now[i] = steers_[i]->angle();
  }
  solver_->reset(steer_now);
}

void robot::Chassis::centerSteer(float dt)
{
  if (!ready_) {
    return;
  }

  for (int i = 0; i < kWheelNum; ++i) {
    /* 轮子不出力 */
    wheels_[i]->setInput(0.0f);
    wheel_pids_[i]->reset(wheels_[i]->vel());

    /* 舵角朝 0 转：目标就是 0，误差归一化到 ±π */
    const float steer_err = robot::WrapPi(0.0f - steers_[i]->angle());
    steers_[i]->setInput(steer_pids_[i]->updateError(steer_err, dt));
  }
}

/* Private function definitions ----------------------------------------------*/
