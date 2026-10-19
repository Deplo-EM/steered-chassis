/**
 *******************************************************************************
 * @file      :gimbal.hpp
 * @brief     : 云台运动模块（yaw 世界系稳定 + pitch 关节限位）
 * @history   :
 *  Version     Date            Author          Note
 *  V1.0.0      2026-10-19      <author>        1. 完成编写
 *******************************************************************************
 * @attention :
 *  【为什么 yaw 和 pitch 用两套不同的控制方式】
 *  这是本设计里最需要讲清楚的一点（也正是考核资料里「混用控制」那一页）。
 *
 *  yaw 轴的问题：yaw 电机的定子固定在底盘上，转子带着云台。如果只做关节角
 *  闭环（「把云台相对底盘的夹角保持在 X」），那么底盘一转，整个云台就跟着转，
 *  枪口方向也跟着变。于是：
 *      - 操作手松开摇杆时，枪口会随车头一起乱转；
 *      - 「跟随模式」里底盘要去追云台，可云台又被底盘带着转，永远追不上。
 *  解决办法：yaw 用云台自己的 IMU 做「世界角」闭环——不管底盘怎么转，
 *  都把枪口钉在同一个世界方向上（相当于给云台加了「防抖云台」）。
 *  底盘转多少，yaw 电机就反向补多少，云台相对底盘的夹角自然就变了，
 *  这也正是底盘板计算 δ 所需的信息。
 *
 *  pitch 轴没有这个问题：俯仰不随车体偏航转动。所以直接用关节角闭环最直观，
 *  也方便加软件限位（超出范围就钳住，防止顶到机械结构堵转）。
 *
 *  【怎么下发】
 *  达妙 DM-J4310 的 MIT 接口可以「位置 + 速度 + kp + kd + 前馈力矩」。这里把
 *  kp、kd 都设为 0，只用最后那个「前馈力矩」通道，也就是纯力矩控制：
 *  角度误差由我们自己写的 PID 变成力矩，再交给电机执行。
 *  好处：整定逻辑全在我们手里，换电机型号也不用改控制结构；
 *  代价：电机的力矩环带宽有限，参数要比内置位置环更保守。
 *******************************************************************************
 *  Copyright (c) 2024 Hello World Team, Zhejiang University.
 *  All Rights Reserved.
 *******************************************************************************
 */
/* Define to prevent recursive inclusion -------------------------------------*/
#ifndef GIMBAL_ROBOT_MODULES_GIMBAL_HPP_
#define GIMBAL_ROBOT_MODULES_GIMBAL_HPP_

/* Includes ------------------------------------------------------------------*/
#include "module_state.hpp"
#include "pid.hpp"

namespace hello_world {
namespace motor {
class Motor;
}  // namespace motor
namespace pid {
class Pid;
}  // namespace pid
}  // namespace hello_world

namespace robot {

/* Exported types ------------------------------------------------------------*/

/**
 * @brief 云台运动模块
 */
class Gimbal {
 public:
  /* ---------------- 机械限位（实车按结构改） ---------------- */
  /** pitch 下限，rad：约 −51°（枪口压到最低） */
  static constexpr float kPitchLowerLimit = -0.9f;
  /** pitch 上限，rad：约 +40°（枪口抬到最高） */
  static constexpr float kPitchUpperLimit = 0.7f;

  /* ---------------- 零点偏移（标定用） ---------------- */
  /** yaw 关节零位偏移：把云台与车头对齐时读到的角度填在这里，δ 就归零了 */
  static constexpr float kYawJointOffset = 0.0f;
  /** pitch 关节零位偏移（一般不用改） */
  static constexpr float kPitchJointOffset = 0.0f;

  Gimbal() = default;
  ~Gimbal() = default;

  /**
   * @brief 接线
   * @param yaw_motor        yaw 电机
   * @param pitch_motor      pitch 电机
   * @param yaw_pid          yaw 位置环
   * @param pitch_pid        pitch 位置环
   */
  void init(hello_world::motor::Motor* yaw_motor,
            hello_world::motor::Motor* pitch_motor,
            hello_world::pid::Pid* yaw_pid,
            hello_world::pid::Pid* pitch_pid);

  /** 设定 yaw 的世界系目标角（rad，可超出 ±π，表示连续旋转） */
  void setYawWorldTarget(float yaw_world) { yaw_target_ = yaw_world; }

  /** 设定 pitch 的关节目标角（rad），内部会做软件限位钳位 */
  void setPitchTarget(float pitch_joint);

  /**
   * @brief 周期更新：算误差 → PID → 力矩下发
   * @param dt             控制周期，s
   * @param yaw_world_meas 云台当前的世界系 yaw（来自云台 IMU），rad
   */
  void update(float dt, float yaw_world_meas);

  /** 两个电机输出归零（急停 / 未就绪） */
  void stop();

  /* ---------------- 状态查询 ---------------- */
  /** yaw 关节角（云台相对底盘的偏航角 δ），rad，已去零点偏移并归一化 */
  float yawJoint() const;
  /** pitch 关节角，rad，已去零点偏移 */
  float pitchJoint() const;
  /** pitch 是否已经顶到软件限位 */
  bool pitchAtLimit() const { return pitch_at_limit_; }
  /** yaw 电机反馈是否有效（收到过数据） */
  bool isReady() const { return ready_; }

  float yawTarget() const { return yaw_target_; }
  float pitchTarget() const { return pitch_target_; }

 private:
  hello_world::motor::Motor* yaw_motor_ = nullptr;
  hello_world::motor::Motor* pitch_motor_ = nullptr;
  hello_world::pid::Pid* yaw_pid_ = nullptr;
  hello_world::pid::Pid* pitch_pid_ = nullptr;

  float yaw_target_ = 0.0f;    ///< 世界系目标角
  float pitch_target_ = 0.0f;  ///< 关节目标角
  bool pitch_at_limit_ = false;
  bool ready_ = false;
};

}  // namespace robot

#endif /* GIMBAL_ROBOT_MODULES_GIMBAL_HPP_ */
