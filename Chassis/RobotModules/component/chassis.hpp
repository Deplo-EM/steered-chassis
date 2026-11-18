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
#include "chassis_debug.hpp"
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
   * @param keep_rotation 小陀螺使用旋转优先的轮速分配。
   */
  void setMotion(const ChassisMotion& motion, float delta, bool keep_rotation = false);

  /**
   * @brief 周期更新：逆解 → 闭环 → 下发（1kHz 调用一次）
   * @param dt 控制周期，单位 s
   */
  void update(float dt);
  /** 与遥控/对板在同一短临界区采集；纯模块调用时update会自行采集。 */
  void captureFeedback();
  bool feedbackHealthy() const { return feedback_healthy_; }
  /** 有驱动需求且全组对齐门已放行（诊断用）。 */
  bool driveReady() const { return drive_ready_; }
  /** 无法绕过已配置的干涉区时，通信层撤销旧电机帧。 */
  uint8_t steerKeepoutBlockedMask() const { return steer_keepout_blocked_mask_; }
  diag::SteerAngleFault steerAngleFault() const;
  /** 对齐门已等待的时长，ms（诊断用） */
  float alignPendingMs() const { return align_pending_ms_; }
  /** 实时对齐超时。首跑锁存后的原因在 lastSteerStall() 保留。 */
  bool alignTimedOut() const { return align_timeout_; }
  uint8_t alignBlockedMask() const { return align_blocked_mask_; }
  uint8_t steerStallMask() const { return steer_stall_mask_; }
  uint8_t alignmentAtStall() const { return alignment_at_stall_; }
  /** 首个触发停机的周期。急停确认只清锁存，保留此记录直到下一次停机。 */
  struct SteerStallSnapshot {
    uint32_t sequence = 0;
    uint8_t stall_mask = 0;
    uint8_t alignment_mask = 0;
    uint8_t alignment_timeout_mask = 0;  ///< 首跑：对齐时间用尽，而非舵 PID 大出力判据。
    float measured_rad[kWheelNum] = {};
    float target_rad[kWheelNum] = {};
    float requested_raw[kWheelNum] = {};  ///< PID 标度；电流版不是 CAN 数据槽的值。
    float feedback_amp[kWheelNum] = {};
  };
  const SteerStallSnapshot& lastSteerStall() const { return last_steer_stall_; }
  /** 只由 Robot 在收到有效急停确认后调用。stop() 不解除故障。 */
  void acknowledgeSteerStall() { steer_stall_mask_ = 0; }

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
  /** 舵角最大误差超过此值时撤销整组驱动；重新放行需回到更小的角差。 */
  static constexpr float kSteerReadyAngle = 0.20f;
  static constexpr float kSteerReentryAngle = 0.12f;
  /** 整组等待超过此时长就报警（ms），不放宽驱动条件。 */
  static constexpr float kAlignTimeoutMs = 300.0f;
  // 保守初值：有明显误差且持续大输出，但编码器没有可辨别的进展。
  static constexpr float kStallOutputRaw = 8000.0f;
  static constexpr float kStallErrorRad = 0.35f;
  static constexpr float kStallProgressRad = 0.04f;
  static constexpr float kStallTimeMs = 500.0f;
  struct SteerProgress {
    float angle = 0.0f;
    float elapsed_ms = 0.0f;
    bool positive = false;
  };
  void checkSteerProgress(int index, float angle, float error, float request_raw, float dt);

  hello_world::motor::Motor* wheels_[kWheelNum] = {nullptr};
  hello_world::motor::Motor* steers_[kWheelNum] = {nullptr};
  hello_world::pid::Pid* wheel_pids_[kWheelNum] = {nullptr};
  hello_world::pid::Pid* steer_pids_[kWheelNum] = {nullptr};
  hello_world::chassis_ik_solver::ChassisIkSolver* solver_ = nullptr;

  ChassisMotion motion_;  ///< 云台系目标速度
  ChassisMotion zero_motion_;  ///< 空对象，solver_ 为空时返回它
  float delta_ = 0.0f;    ///< 云台相对车头偏航角
  bool ready_ = false;
  bool feedback_pending_ = false;
  bool feedback_healthy_ = false;
  uint8_t steer_angle_fault_mask_ = 0;  ///< 本次采样中超出单圈范围或非有限的舵角。
  uint8_t steer_keepout_blocked_mask_ = 0;
  bool drive_ready_ = false;      ///< 最近一次全组对齐结果
  bool keep_rotation_ = false;   ///< 对应当拍的小陀螺轮速分配请求。
  bool align_timeout_ = false;    ///< 已超时，仍禁止驱动
  float align_pending_ms_ = 0.0f; ///< 对不齐已持续的时间
  uint8_t align_blocked_mask_ = 0; ///< bit0–3 = 左前、左后、右后、右前
  uint8_t steer_stall_mask_ = 0;   ///< 堵转锁存，急停确认后才清除
  uint8_t alignment_at_stall_ = 0; ///< 最近一次停机时的对齐阻塞位，复位前供诊断
  SteerStallSnapshot last_steer_stall_;
  SteerProgress steer_progress_[kWheelNum];
  float steer_feedback_[kWheelNum] = {};
  float steer_velocity_[kWheelNum] = {};  ///< 舵轴实测转速，rad/s；与舵角同方向。
  float steer_current_[kWheelNum] = {};
  float wheel_velocity_[kWheelNum] = {};
};

}  // namespace robot

#endif /* CHASSIS_ROBOT_MODULES_CHASSIS_HPP_ */
