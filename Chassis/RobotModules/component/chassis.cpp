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
 *    - 舵角环输出      : 沿用电压 raw 标度；传给电机时按固件配置使用 raw 或 A。
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
#include "steer_output_profile.hpp"
#if defined(ROBOT_GROUND_TRIAL) && ROBOT_GROUND_TRIAL
#include "ground_trial.hpp"
static_assert(robot::steer_output::kUseCurrent, "Ground trial uses the verified CAN current frame profile");
#endif

namespace robot::diag {
volatile ChassisDebug g_chassis_dbg = {};
volatile KeepoutDebug g_keepout_dbg = {};
}  // namespace robot::diag

namespace {
/* 现场让步（构建开关）：机械卡死、无法参与控制的轮位（0–3）。
 * -1 = 不排除；保留电机接线，只在角度判废/对齐门/堵转/输出里忽略它。 */
#if defined(ROBOT_EXCLUDED_WHEEL) && ROBOT_EXCLUDED_WHEEL >= 0
constexpr int kExcludedWheel = ROBOT_EXCLUDED_WHEEL;
#else
constexpr int kExcludedWheel = -1;
#endif

float s_steer_vel_filt[4] = {};  ///< 前馈门限用的滤波舵速，抑制原始速度尖刺

#if defined(ROBOT_STEER_BREAKAWAY) && ROBOT_STEER_BREAKAWAY
/* 脉冲式静摩擦补偿的每轮状态（历史试验构建）。 */
float s_stuck_ms[4] = {};
float s_pulse_ms[4] = {};
float s_cooldown_ms[4] = {};
#endif
}  // namespace
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
  steer_stall_mask_ = 0;
  alignment_at_stall_ = 0;
  last_steer_stall_ = {};
  uint8_t configured_mask = 0u;
  if (solver_) {
    for (int i = 0; i < kWheelNum; ++i) {
      if (solver_->geometry().steer_keepout[i].half_width_rad != 0.0f)
        configured_mask |= static_cast<uint8_t>(1u << i);
    }
  }
  diag::g_chassis_dbg.steer_keepout_configured_mask = configured_mask;
  diag::g_chassis_dbg.last_steer_keepout_blocked_mask = 0u;
  stop();
}

void robot::Chassis::setMotion(const ChassisMotion& motion, float delta, bool keep_rotation)
{
  motion_ = motion;
  delta_ = delta;
  keep_rotation_ = keep_rotation;
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

/* ============================================================================
 * 采集一轮电机反馈（在临界区里做，防止和 CAN 中断抢数据）
 * ---------------------------------------------------------------------------
 * 读取：四个舵角、舵速、舵电流，四个轮速。
 * 健康判定：任一轮反馈非有限，或任一电机 isHealthy 为 false，整组标记不健康。
 * 注意：舵角保留原始值不做 Wrap——异常值要留给上层判废，不能"折一下当真"。
 * ========================================================================== */
void robot::Chassis::captureFeedback()
{
  CriticalSection lock;
  auto& dbg = diag::g_chassis_dbg;
  feedback_healthy_ = ready_;
  steer_angle_fault_mask_ = 0u;
  for (int i = 0; i < kWheelNum; ++i) {
    if (!wheels_[i] || !steers_[i]) {
      feedback_healthy_ = false;
      continue;
    }
    steer_feedback_[i] = steers_[i]->angle();
    if constexpr (steer_output::kUseCurrent) steer_velocity_[i] = steers_[i]->vel();
    steer_current_[i] = steers_[i]->curr();
    wheel_velocity_[i] = wheels_[i]->vel();

    // 实例约定单圈角；保留异常原值，不能 Wrap 后将坏数据当真。允许等价端点 +π。
    const float angle = steer_feedback_[i];
    const bool excluded = (i == kExcludedWheel);
#if defined(ROBOT_SKIP_ANGLE_FAULT) && ROBOT_SKIP_ANGLE_FAULT
    /* 地面让步（单独开关）：跳过舵角越界判废，仅用于定位问题，不用于交付。 */
    const bool skip_angle_fault = true;
#else
    const bool skip_angle_fault = excluded;
#endif
    const bool angle_ok = std::isfinite(angle) && std::fabs(angle) <= robot::kPi;
    if (!angle_ok && !skip_angle_fault) steer_angle_fault_mask_ |= static_cast<uint8_t>(1u << i);
    feedback_healthy_ = feedback_healthy_ && (skip_angle_fault ||
        (angle_ok && std::isfinite(wheel_velocity_[i]) &&
         std::isfinite(steer_velocity_[i]) &&
         wheels_[i]->isHealthy() && steers_[i]->isHealthy()));
    dbg.steer_feedback[i] = angle;
    dbg.wheel_velocity[i] = wheel_velocity_[i];
  }
  dbg.steer_angle_fault_mask = steer_angle_fault_mask_;
  feedback_pending_ = true;
}

robot::diag::SteerAngleFault robot::Chassis::steerAngleFault() const
{
  diag::SteerAngleFault fault;
  fault.mask = steer_angle_fault_mask_;
  for (int i = 0; i < kWheelNum; ++i) {
    if (fault.mask & (1u << i)) fault.measured_rad[i] = steer_feedback_[i];
  }
  return fault;
}

/* ============================================================================
 * 底盘控制主流程（每 1 ms 一次）
 * ---------------------------------------------------------------------------
 * 步骤：
 *   1) 采反馈 + 健康检查；不健康直接 stop；
 *   2) 逆解 solve()；解算失败直接 stop；
 *   3) 对齐判定（drive_ready_ / align_blocked_mask_）；
 *   4) 逐轮轮速 PID：m/s → rad/s，乘 cos(舵角误差) 减小侧向拖胎；
 *   5) 逐轮舵角 PID：D 用实测转速，另加静摩擦前馈/脉冲；
 *   6) 堵转与禁入区检查；
 *   7) 下发 8 个电机，并写调试快照。
 * 不变量：任一轮异常都整组停车，不做"只停一个轮"的降级（现场让步开关除外）。
 * ========================================================================== */
void robot::Chassis::update(float dt)
{
  if (!ready_) {
    return;
  }

  if (!feedback_pending_) captureFeedback();
  feedback_pending_ = false;
  if (!feedback_healthy_ || steer_stall_mask_ != 0) { stop(); return; }
  hello_world::chassis_ik_solver::WheelCommand cmd[kWheelNum];
  solver_->solve(motion_, delta_, steer_feedback_, dt, cmd, keep_rotation_);
  if (!solver_->lastSolveValid()) {
    const uint8_t blocked = solver_->keepoutBlockedMask();
    stop();
    steer_keepout_blocked_mask_ = blocked;
    diag::g_chassis_dbg.steer_keepout_blocked_mask = blocked;
    if (blocked != 0u) diag::g_chassis_dbg.last_steer_keepout_blocked_mask = blocked;
    return;
  }
  steer_keepout_blocked_mask_ = 0u;
  diag::g_chassis_dbg.steer_keepout_blocked_mask = 0u;

  const float radius = solver_->geometry().wheel_radius;
  float drive_error[kWheelNum];
  align_blocked_mask_ = 0;
#if defined(ROBOT_GROUND_TRIAL) && ROBOT_GROUND_TRIAL
  const float align_limit = drive_ready_ ? ground::kAlignExitRad : ground::kAlignEnterRad;
  constexpr float kAlignWaitMs = ground::kAlignCutoffMs;
#else
  const float align_limit = drive_ready_ ? kSteerReadyAngle : kSteerReentryAngle;
  constexpr float kAlignWaitMs = kAlignTimeoutMs;
#endif
  bool has_drive_demand = false;
  float worst_alignment_error = 0.0f;
  for (int i = 0; i < kWheelNum; ++i) {
    drive_error[i] = robot::WrapPi(cmd[i].desired_angle - steer_feedback_[i]);
    if (cmd[i].speed == 0.0f) continue;
    has_drive_demand = true;
    const float trajectory_error = robot::WrapPi(cmd[i].desired_angle - cmd[i].angle);
    const float error = std::fmax(std::fabs(drive_error[i]), std::fabs(trajectory_error));
    worst_alignment_error = std::fmax(worst_alignment_error, error);
    if (error > align_limit) {
      align_blocked_mask_ |= static_cast<uint8_t>(1u << i);
    }
  }
  drive_ready_ = has_drive_demand && align_blocked_mask_ == 0;
#if defined(ROBOT_ACCEPT_STUCK_STEER) && ROBOT_ACCEPT_STUCK_STEER
  /* 地面让步（构建开关）：接受某个舵轮被机械刮擦卡住、无法对齐，
   * 四轮照常驱动，不再要求整组对齐。代价是卡住的轮子会拖胎/刮擦。 */
  drive_ready_ = has_drive_demand;
#endif
  if constexpr (kExcludedWheel >= 0) {
    // 排除轮不参与整组对齐判定，避免一个卡死轮把其余三轮一起拦住。
    align_blocked_mask_ &= static_cast<uint8_t>(~(1u << kExcludedWheel));
    drive_ready_ = has_drive_demand && align_blocked_mask_ == 0;
  }
  // 对齐边缘四轮共同降速，保持平移/旋转比例，避免某个轮位先吃满速度。
#if (defined(ROBOT_GROUND_TRIAL) && ROBOT_GROUND_TRIAL) || \
    (defined(ROBOT_ACCEPT_STUCK_STEER) && ROBOT_ACCEPT_STUCK_STEER) || \
    (defined(ROBOT_BLOCKED_WHEEL_FALLBACK) && ROBOT_BLOCKED_WHEEL_FALLBACK)
  const float group_scale = drive_ready_ ? 1.0f : 0.0f;
#else
  constexpr float kFullDriveAngle = hello_world::chassis_ik_solver::kDriveAlignedRad;
  const float group_scale = drive_ready_ ?
      Clamp((kSteerReadyAngle - worst_alignment_error) / (kSteerReadyAngle - kFullDriveAngle), 0.0f, 1.0f)
      : 0.0f;
#endif
  if (align_blocked_mask_ == 0) {
    align_pending_ms_ = 0.0f;
    align_timeout_ = false;
  } else {
    align_pending_ms_ = std::fmin(kAlignWaitMs, align_pending_ms_ + dt * 1000.0f);
    if (align_pending_ms_ >= kAlignWaitMs) align_timeout_ = true;
  }

  float current[kWheelNum], steer_request_raw[kWheelNum], steer_error[kWheelNum];
  float wheel_target[kWheelNum] = {};
  for (int i = 0; i < kWheelNum; ++i) {
    const float steer_err = robot::WrapPi(cmd[i].angle - steer_feedback_[i]);
    steer_error[i] = steer_err;
    // 整组到位才放行。无需求的轮清空旧积分，避免停车仍被驱动。
#if defined(ROBOT_BLOCKED_WHEEL_FALLBACK) && ROBOT_BLOCKED_WHEEL_FALLBACK
    /* 现场让步（构建开关）：某个舵轮被机械卡死、永远到不了对齐范围时，
     * 整组门会随它的误差在回差带内反复开合，造成驱动轮“一跳一跳”。
     * 超时后改为逐轮放行：已对齐的轮子连续驱动，卡死轮保持零输出。 */
    const bool wheel_ok = (align_blocked_mask_ & (1u << i)) == 0;
    const bool release = drive_ready_ || (align_timeout_ && wheel_ok);
#else
    const bool release = drive_ready_ && group_scale > 0.0f;
#endif
    float omega_target = 0.0f;
    if (!release || cmd[i].speed == 0.0f) {
      wheel_pids_[i]->reset();
      current[i] = 0.0f;
    } else {
      // 历史让步/首跑保持原投影；完整四轮使用同一比例。
#if (defined(ROBOT_GROUND_TRIAL) && ROBOT_GROUND_TRIAL) || \
    (defined(ROBOT_ACCEPT_STUCK_STEER) && ROBOT_ACCEPT_STUCK_STEER) || \
    (defined(ROBOT_BLOCKED_WHEEL_FALLBACK) && ROBOT_BLOCKED_WHEEL_FALLBACK)
      omega_target = cmd[i].speed / radius * std::cos(drive_error[i]);
#else
      omega_target = cmd[i].speed / radius * group_scale;
#endif
      wheel_target[i] = omega_target;
      current[i] = wheel_pids_[i]->update(omega_target, wheel_velocity_[i], dt);
    }
    if (cmd[i].reanchored) steer_pids_[i]->reset();
    if constexpr (steer_output::kUseCurrent) {
      // D 只给实际舵向转动加阻尼，逆解目标快速变化时不额外推舵机。
      constexpr float kMaxDampingRate = 20.0f;  // 约为舵角目标限速的两倍，限制异常速率造成的 D 出力。
      const float rate = Clamp(steer_velocity_[i], -kMaxDampingRate, kMaxDampingRate);
      steer_request_raw[i] = steer_pids_[i]->updateWithMeasuredRate(steer_err, rate, dt);
    } else {
      steer_request_raw[i] = steer_pids_[i]->updateError(steer_err, dt);
    }
#if defined(ROBOT_STEER_BREAKAWAY) && ROBOT_STEER_BREAKAWAY
    {
      /* 脉冲式静摩擦补偿：平时不推；卡住（误差大、几乎不转）持续 100 ms 后
       * 给 1.0 A / 60 ms 的短脉冲帮助越过刮蹭点，然后冷却 200 ms。 */
      constexpr float kPulseA = 1.0f;
      constexpr float kPulseRaw =
          kPulseA * steer_output::kPidOutputLimit / steer_output::kFieldCurrentLimitA;
      constexpr float kStuckEnterRad = 0.12f;
      constexpr float kStuckExitRad = 0.05f;
      constexpr float kStuckMaxRate = 0.30f;
      constexpr float kStuckMs = 100.0f;
      constexpr float kPulseMs = 60.0f;
      constexpr float kCooldownMs = 200.0f;

      if (std::fabs(steer_err) < kStuckExitRad || std::fabs(steer_velocity_[i]) > kStuckMaxRate) {
        s_stuck_ms[i] = 0.0f;
        s_pulse_ms[i] = 0.0f;
        s_cooldown_ms[i] = 0.0f;
      } else if (s_pulse_ms[i] > 0.0f) {
        steer_request_raw[i] = (steer_err > 0.0f ? kPulseRaw : -kPulseRaw);
        s_pulse_ms[i] -= dt * 1000.0f;
      } else if (s_cooldown_ms[i] > 0.0f) {
        s_cooldown_ms[i] -= dt * 1000.0f;
      } else if (std::fabs(steer_err) > kStuckEnterRad) {
        s_stuck_ms[i] += dt * 1000.0f;
        if (s_stuck_ms[i] >= kStuckMs) {
          s_stuck_ms[i] = 0.0f;
          s_pulse_ms[i] = kPulseMs;
          s_cooldown_ms[i] = kCooldownMs;
        }
      } else {
        s_stuck_ms[i] = 0.0f;
      }
    }
#endif
#if defined(ROBOT_STEER_FRICTION_COMP) && ROBOT_STEER_FRICTION_COMP
    /* 台架静摩擦特调（构建开关）：误差较大且舵轴几乎不转（卡在刮蹭点）时，
     * 叠加一个固定方向的前馈，帮助越过静摩擦。
     * 单位：kFrictionCompA 为安培，按 raw = A × 20000 / 1.2 换算。 */
    {
      constexpr float kFrictionCompA = 0.70f;
      /* 死区 0.15 rad：目标附近的小误差不触发，避免静止抖动。 */
      constexpr float kFrictionDeadbandRad = 0.15f;
      constexpr float kFrictionMaxRate = 0.6f;
      /* 原始舵速噪声可达 ±20~30 rad/s，直接用会把前馈永远挡掉；
       * 这里对速度做一阶低通（0.1/拍），用滤波值判断“是否几乎不转”。 */
      s_steer_vel_filt[i] += 0.10f * (steer_velocity_[i] - s_steer_vel_filt[i]);
      if (std::fabs(steer_err) > kFrictionDeadbandRad &&
          std::fabs(s_steer_vel_filt[i]) < kFrictionMaxRate) {
        constexpr float kCompRaw = kFrictionCompA * steer_output::kPidOutputLimit /
                                   steer_output::kFieldCurrentLimitA;
        steer_request_raw[i] += (steer_err > 0.0f ? kCompRaw : -kCompRaw);
      }
    }
#endif
    if (!std::isfinite(current[i]) || !std::isfinite(steer_request_raw[i])) { stop(); return; }
#if defined(ROBOT_GROUND_TRIAL) && ROBOT_GROUND_TRIAL
    current[i] = Clamp(current[i], -ground::kWheelCurrentA, ground::kWheelCurrentA);
#endif
    if (i == kExcludedWheel) {
      // 排除轮既不驱动也不推舵，避免顶着机械卡点发热/堵转。
      wheel_pids_[i]->reset();
      current[i] = 0.0f;
      steer_request_raw[i] = 0.0f;
    }
    checkSteerProgress(i, steer_feedback_[i], steer_err, steer_request_raw[i], dt);
  }
  uint8_t alignment_timeout_mask = 0;
#if defined(ROBOT_GROUND_TRIAL) && ROBOT_GROUND_TRIAL
  // 中等误差卡住时，舵 PID 可能低于原堵转阈值。仍限制整组等待及持续顶住的时间。
  if (align_timeout_) {
    alignment_timeout_mask = align_blocked_mask_;
    steer_stall_mask_ |= alignment_timeout_mask;
  }
#endif
  if (steer_stall_mask_ != 0) {
    alignment_at_stall_ = align_blocked_mask_;
    auto& event = last_steer_stall_;
    event.stall_mask = steer_stall_mask_;
    event.alignment_mask = align_blocked_mask_;
    event.alignment_timeout_mask = alignment_timeout_mask;
    for (int i = 0; i < kWheelNum; ++i) {
      event.measured_rad[i] = steer_feedback_[i];
      event.target_rad[i] = cmd[i].angle;
      event.requested_raw[i] = steer_request_raw[i];
      event.feedback_amp[i] = steer_current_[i];
    }
    ++event.sequence;
    stop();
    return;
  }
  auto& dbg = diag::g_chassis_dbg;
  auto& kd = diag::g_keepout_dbg;
  dbg.align_blocked_mask = align_blocked_mask_;
  dbg.drive_ready = drive_ready_;
  dbg.alignment_scale = group_scale;
  dbg.drive_w_request = solver_->chassis_motion().w * group_scale;
  kd.configured_mask = 0u;
  kd.blocked_mask = solver_->keepoutBlockedMask();
  kd.in_zone_mask = 0u;
  for (int i = 0; i < kWheelNum; ++i) {
    wheels_[i]->setInput(current[i]);
    steers_[i]->setInput(steer_output::MotorInput(steer_request_raw[i]));
    dbg.steer_target[i] = cmd[i].angle;
    dbg.steer_desired[i] = cmd[i].desired_angle;
    dbg.steer_output[i] = steer_request_raw[i];
    dbg.steer_error[i] = steer_error[i];
    dbg.wheel_target[i] = wheel_target[i];
    dbg.wheel_output[i] = current[i];
    /* 卡点测量视图：目标/反馈/转速/输出/误差 + 已配置的干涉区。 */
    const auto& zone = solver_->geometry().steer_keepout[i];
    kd.target[i] = cmd[i].angle;
    kd.feedback[i] = steer_feedback_[i];
    kd.velocity[i] = steer_velocity_[i];
    kd.output[i] = steer_request_raw[i];
    kd.error[i] = steer_error[i];
    kd.center_rad[i] = zone.center_rad;
    kd.half_width_rad[i] = zone.half_width_rad;
    if (zone.half_width_rad != 0.0f) kd.configured_mask |= static_cast<uint8_t>(1u << i);
    if (zone.half_width_rad > 0.0f &&
        std::fabs(robot::WrapPi(steer_feedback_[i] - zone.center_rad)) < zone.half_width_rad) {
      kd.in_zone_mask |= static_cast<uint8_t>(1u << i);
    }
  }
}

/* ============================================================================
 * 整组停车
 * ---------------------------------------------------------------------------
 * 八个电机输入清零；清轮速/舵角 PID 历史；清逆解历史（下次从实测角重新起步）；
 * 清对齐计时与禁入区阻塞位。
 * 不清堵转锁存：那是故障证据，要留给上层读。
 * ========================================================================== */
void robot::Chassis::stop()
{
  feedback_pending_ = false;
  motion_.reset();
  delta_ = 0.0f;
  keep_rotation_ = false;
  auto& dbg = diag::g_chassis_dbg;
  dbg.align_blocked_mask = 0u;
  dbg.drive_ready = 0u;
  steer_keepout_blocked_mask_ = 0u;
  dbg.steer_keepout_blocked_mask = 0u;
  dbg.alignment_scale = 0.0f;
  dbg.drive_w_request = 0.0f;
  for (int i = 0; i < kWheelNum; ++i) {
    if (wheels_[i]) wheels_[i]->setInput(0.0f);
    if (steers_[i]) steers_[i]->setInput(0.0f);
    // reset 后首帧跳过微分；不把失效反馈存进控制历史。
    if (wheel_pids_[i]) wheel_pids_[i]->reset();
    if (steer_pids_[i]) steer_pids_[i]->reset();
    // 停机后保留实测反馈；实时请求同步归零，历史原因看故障快照。
    dbg.steer_target[i] = 0.0f;
    dbg.steer_desired[i] = 0.0f;
    dbg.steer_output[i] = 0.0f;
    dbg.steer_error[i] = 0.0f;
    dbg.wheel_target[i] = 0.0f;
    dbg.wheel_output[i] = 0.0f;
  }
  // 清历史，下一次 solve 从当时实测舵角开始，允许停机期间手动转动舵轮。
  if (solver_) solver_->reset();
  // 清临时计时和角度历史，但保留堵转锁存供 Ozone 与状态机读取。
  align_pending_ms_ = 0.0f;
  align_timeout_ = false;
  align_blocked_mask_ = 0;
  for (auto& progress : steer_progress_) progress = {};
  drive_ready_ = false;
}

/* ============================================================================
 * 舵向堵转判定（单轮）
 * ---------------------------------------------------------------------------
 * 三个条件同时满足才锁存：
 *   1) |角度误差| > kSteerReadyAngle（20°）；
 *   2) |输出请求| >= kStallOutputRaw（8000 raw，约 0.48 A）；
 *   3) 500 ms 内角度进展 < 0.04 rad。
 * 这样"输出大但正常在转"不会被误判；锁存后由上层撤销整组输出。
 * 本函数只负责置位，不直接停车。
 * ========================================================================== */
void robot::Chassis::checkSteerProgress(int index, float angle, float error,
                                       float request_raw, float dt)
{
#if defined(ROBOT_STEER_BENCH_NO_STALL) && ROBOT_STEER_BENCH_NO_STALL
  /* 台架特调专用：关闭堵转锁存，允许长时间顶住刮蹭点观察摩擦行为。
   * 绝不能用于赛场固件。 */
  (void)index; (void)angle; (void)error; (void)request_raw; (void)dt;
  return;
#endif
#if defined(ROBOT_ACCEPT_STUCK_STEER) && ROBOT_ACCEPT_STUCK_STEER
  /* 地面让步：卡住的舵轮不参与堵转锁存，避免它把整车停掉。 */
  (void)index; (void)angle; (void)error; (void)request_raw; (void)dt;
  return;
#endif
  if (index == kExcludedWheel) return;  // 现场让步：排除轮不参与堵转锁存
  auto& progress = steer_progress_[index];
  if (std::fabs(error) <= kStallErrorRad || std::fabs(request_raw) < kStallOutputRaw) {
    progress = {};
    return;
  }
  const bool positive = request_raw > 0.0f;
  const float advance = robot::WrapPi(angle - progress.angle) * (positive ? 1.0f : -1.0f);
  if (progress.elapsed_ms == 0.0f || positive != progress.positive ||
      advance >= kStallProgressRad) {
    progress = {angle, 0.0f, positive};
  }
  progress.elapsed_ms += dt * 1000.0f;
  if (progress.elapsed_ms >= kStallTimeMs) {
    steer_stall_mask_ |= static_cast<uint8_t>(1u << index);
  }
}

/* Private function definitions ----------------------------------------------*/
