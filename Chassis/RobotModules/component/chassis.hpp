/**
 *******************************************************************************
 * @file      :chassis.hpp
 * @brief     : 底盘运动模块（逆解 + 轮速环 + 舵角环）
 * @history   :
 *  Version     Date            Author          Note
 *  V1.0.0      2026-10-19      <author>        1. 完成编写
 *******************************************************************************
 * @attention :
 *  【它在整条链路里的位置】
 *     遥控器 → Robot（模式逻辑、算 vx/vy/w）→ Chassis（本类）→ 8 个电机
 *
 *  Chassis 只做三件事：
 *    1. 把「云台系速度指令」交给逆解器，得到每个轮组的 (目标舵角, 目标轮速)；
 *    2. 用 PID 把「目标舵角 → 舵电机电压」「目标轮速 → 轮电机电流」闭环；
 *    3. 保证任何时刻都能一键停机（stop()）。
 *
 *  【为什么舵角没到位时不给轮子出力】
 *  舵轮如果一边转向一边驱动，轮子会「斜着蹭地」：既费电、又打滑、还会让车
 *  走出一条弧线。所以这里加了一条工程规则：舵角误差超过约 20° 时，轮速目标
 *  直接置 0，先把轮子摆正再走。代价是起步慢几十毫秒，换来的是不打滑、不抖。
 *******************************************************************************
 *  Copyright (c) 2024 Hello World Team, Zhejiang University.
 *  All Rights Reserved.
 *******************************************************************************
 */
/* Define to prevent recursive inclusion -------------------------------------*/
#ifndef CHASSIS_ROBOT_MODULES_CHASSIS_HPP_
#define CHASSIS_ROBOT_MODULES_CHASSIS_HPP_

/* Includes ------------------------------------------------------------------*/
#include "chassis_ik_solver.hpp"
#include "module_state.hpp"
#include "pid.hpp"

namespace hello_world {
namespace motor {
class Motor;
}  // namespace motor
}  // namespace hello_world

namespace robot {

/* Exported types ------------------------------------------------------------*/

/**
 * @brief 底盘运动模块
 */
class Chassis {
 public:
  /** 轮组数量（与逆解器一致） */
  static constexpr int kWheelNum = hello_world::chassis_ik_solver::ChassisIkSolver::kWheelNum;

  Chassis() = default;
  ~Chassis() = default;

  /**
   * @brief 接线：把电机、PID、逆解器接进来
   * @note  只保存指针，不拥有对象；对象生命周期由 Instance 层保证（全程存在）。
   */
  void init(hello_world::motor::Motor* wheels[kWheelNum],
            hello_world::motor::Motor* steers[kWheelNum],
            hello_world::pid::Pid* wheel_pids[kWheelNum],
            hello_world::pid::Pid* steer_pids[kWheelNum],
            hello_world::chassis_ik_solver::ChassisIkSolver* solver);

  /**
   * @brief 下发速度指令（云台坐标系）并给出云台相对车头的偏航角
   * @param motion 云台系速度指令 (vx, vy, w)
   * @param delta  云台相对车头的偏航角 δ，单位 rad（取 G2C 帧里的 yaw_joint）
   */
  void setMotion(const ChassisMotion& motion, float delta);

  /**
   * @brief 周期更新：逆解 → 闭环 → 下发（1kHz 调用一次）
   * @param dt 控制周期，单位 s
   */
  void update(float dt);

  /**
   * @brief 所有电机输出归零（急停 / 未就绪 / 掉线时调用）
   * @note  同时清空 PID 积分和逆解历史，避免恢复时出现「攒了很久的积分突然释放」
   */
  void stop();

  /* 状态查询 */
  bool isReady() const { return ready_; }
  const ChassisMotion& motion() const { return motion_; }         ///< 云台系指令
  const ChassisMotion& chassisMotion() const { return solver_ ? solver_->chassis_motion() : zero_motion_; }
  const hello_world::chassis_ik_solver::WheelCommand& wheelCommand(int index) const;
  float delta() const { return delta_; }

 private:
  /** 舵角误差超过这个值就不驱动轮子（rad） */
  static constexpr float kSteerReadyAngle = 0.35f;  // ≈20°

  hello_world::motor::Motor* wheels_[kWheelNum] = {nullptr};
  hello_world::motor::Motor* steers_[kWheelNum] = {nullptr};
  hello_world::pid::Pid* wheel_pids_[kWheelNum] = {nullptr};
  hello_world::pid::Pid* steer_pids_[kWheelNum] = {nullptr};
  hello_world::chassis_ik_solver::ChassisIkSolver* solver_ = nullptr;

  ChassisMotion motion_;  ///< 云台系目标速度
  ChassisMotion zero_motion_;  ///< 空对象，solver_ 为空时返回它
  float delta_ = 0.0f;    ///< 云台相对车头偏航角
  bool ready_ = false;
};

}  // namespace robot

#endif /* CHASSIS_ROBOT_MODULES_CHASSIS_HPP_ */
