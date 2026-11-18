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
#include "gimbal_limits.hpp"
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
  /** pitch 下限，rad。
   *  2026-11-15 实车实测：目标压到 −0.9 时反馈停在 −0.5016 不动，
   *  输出已顶到 −3.18（限幅 −3.5）、积分灌满 −1.50 —— 说明机械行程在约 −0.50。
   *  故把软件下限收到 −0.47（留 0.03 余量），避免顶住机械限位后饱和+积分饱和。 */
  static constexpr float kPitchLowerLimit = gimbal_limits::kPitchLowerRad;
  /** pitch 上限，rad：约 +40°（枪口抬到最高）。尚未测到机械上止点，暂不改。 */
  static constexpr float kPitchUpperLimit = gimbal_limits::kPitchUpperRad;

  /** yaw 速率前馈增益，N·m / (rad/s)。
   *  B13 0.20 已明显减小滞后；B14 提到 0.30（满杆约 2 rad/s → 0.6 N·m），
   *  用前馈继续收拐角滞后，不再动 P（P 上限 2.2，2.5 会振荡）。 */
  static constexpr float kYawRateFfGain = 0.30f;

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

  /** 设定 yaw 速率前馈（rad/s）：底盘下发的期望转速，用于减小跟随滞后。
   *  符号约定与 PID 输出一致：正速率 → 正力矩（使世界角增大）。 */
  void setYawRateFeedforward(float rate) { yaw_rate_ff_ = rate; }

  /**
   * @brief 周期更新：算误差 → PID → 力矩下发
   * @param dt             控制周期，s
   * @param yaw_world_meas 云台当前的世界系 yaw（来自云台 IMU），rad
   * @param yaw_world_rate 世界系 yaw 角的变化率，rad/s
   */
  void update(float dt, float yaw_world_meas, float yaw_world_rate);
  /** 在控制周期输入快照阶段调用，避免关节位置/健康来自不同反馈。 */
  void captureFeedback();
  bool feedbackHealthy() const { return feedback_healthy_; }
  bool feedbackFinite() const { return feedback_finite_; }
  bool controlValid() const { return control_valid_; }

  /** 两个电机输出归零（急停 / 未就绪） */
  /** 只清PID历史，恢复出力时保留本周期已经捕获的反馈。 */
  void resetControllers();
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
  /** 把常用量写进 robot::diag::g_gimbal_dbg，供 Ozone 只读观察。 */
  void updateDebug();

  hello_world::motor::Motor* yaw_motor_ = nullptr;
  hello_world::motor::Motor* pitch_motor_ = nullptr;
  hello_world::pid::Pid* yaw_pid_ = nullptr;
  hello_world::pid::Pid* pitch_pid_ = nullptr;

  float yaw_target_ = 0.0f;    ///< 世界系目标角
  float yaw_rate_ff_ = 0.0f;   ///< yaw 速率前馈输入 rad/s
  float pitch_target_ = 0.0f;  ///< 关节目标角
  bool pitch_at_limit_ = false;
  /** 到限位后的“保持”状态：为 true 时不再每周期重算 PID，直接出 0，
   *  避免 1 kHz 反复“算出力矩→清零”形成方波把电机抖响。 */
  bool pitch_limit_hold_ = false;
  bool pitch_limit_hold_high_ = false;  ///< 保持的是上限(true)还是下限(false)
  bool ready_ = false;
  bool feedback_pending_ = false;
  bool feedback_valid_ = false;
  bool feedback_healthy_ = false;
  bool feedback_finite_ = false;
  bool control_valid_ = false;
  float yaw_feedback_ = 0, pitch_feedback_ = 0;
  float pitch_joint_rate_ = 0.0f;  ///< 与俯仰关节角同方向，rad/s
  float yaw_world_feedback_ = 0.0f;  ///< 世界系角反馈，供 Ozone 和目标角并排对照。
};

}  // namespace robot

#endif /* GIMBAL_ROBOT_MODULES_GIMBAL_HPP_ */
