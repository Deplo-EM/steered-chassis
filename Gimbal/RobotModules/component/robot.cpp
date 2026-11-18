/**
 *******************************************************************************
 * @file      :robot.cpp
 * @brief     : 云台板顶层状态机实现
 * @history   :
 *  Version     Date            Author          Note
 *  V1.0.0      2026-10-19      <author>        1. 完成编写
 *******************************************************************************
 * @attention :
 *  【使能上升沿为什么要重新对齐目标】
 *  平时 yaw 的目标角是「积分」出来的（转速 × 时间）。如果停止出力一段时间
 *  （急停、掉线恢复），这段时间里操作手可能已经把云台用手掰走了。恢复的瞬间
 *  如果直接沿用旧目标，云台会「突然发现自己在几百毫秒前的角度上」然后猛地
 *  甩回去。所以每次从「禁止出力」变成「允许出力」时，把目标重新对齐到当前
 *  实际角度，实现无扰恢复。
 *******************************************************************************
 *  Copyright (c) 2024 Hello World Team, Zhejiang University.
 *  All Rights Reserved.
 *******************************************************************************
 */
/* Includes ------------------------------------------------------------------*/
#include "robot.hpp"

#include <cmath>
#include <limits>

#include "DT7.hpp"
#include "fault_injection.hpp"
#include "gimbal.hpp"
#include "gimbal_chassis_comm.hpp"
#include "imu.hpp"
#include "tick.hpp"
#include "critical_section.hpp"
#include "basic_mode.hpp"
#include "gimbal_debug.hpp"
/* Private macro -------------------------------------------------------------*/
/* Private constants ---------------------------------------------------------*/

namespace {

/** 控制周期，s */
constexpr float kDt = 0.001f;

/** 遥控器掉线阈值，ms */
constexpr uint32_t kRemoteOfflineMs = 100;
/** 电机掉线阈值，ms */
constexpr uint32_t kMotorOfflineMs = 100;
/** 恢复自检时长，ms */
constexpr uint32_t kResurrectionMs = 500;

#if !defined(ROBOT_GROUND_GIMBAL_LOCK) || !ROBOT_GROUND_GIMBAL_LOCK
float WorldYawRate(const hello_world::imu::Imu& imu)
{
  const float roll = imu.roll(), pitch = imu.pitch();
  const float gyro_y = imu.gyroY(), gyro_z = imu.gyroZ();
  if (!std::isfinite(roll) || !std::isfinite(pitch) ||
      !std::isfinite(gyro_y) || !std::isfinite(gyro_z)) {
    return std::numeric_limits<float>::quiet_NaN();
  }
  // ZYX 欧拉角的 yaw 变化率。身体系 gyroZ 在云台有俯仰时不等于 yaw 变化率。
  // 远离欧拉角奇异点时直接换算；接近竖直时限制分母增益，以免放大噪声。
  const float denominator = std::fmax(0.35f, std::cos(pitch));
  const float rate = (gyro_y * std::sin(roll) + gyro_z * std::cos(roll)) / denominator;
  return robot::Clamp(rate, -15.0f, 15.0f);
}
#endif

}  // namespace

/* Private variables ---------------------------------------------------------*/
/* Private function prototypes -----------------------------------------------*/
/* Exported function definitions ---------------------------------------------*/

void robot::Robot::init()
{
  rc_checker_.init(kRemoteOfflineMs);
  for (int i = 0; i < kMotorSlotNum; ++i) {
    motor_checkers_[i].init(kMotorOfflineMs);
  }

  state_ = LifeState::kDead;
  state_enter_tick_ = hello_world::tick::GetTickMs();
  estop_ = false;
  can_tx_guard_ = CanTxGuard{};
  imu_control_state_ = ImuControlState::kUnavailable;
  diag::g_gimbal_dbg.dead_reason = 0u;
  diag::g_gimbal_dbg.last_dead_reason = 0u;
  diag::g_gimbal_dbg.last_dead_tick_ms = 0u;
  yaw_target_world_ = 0.0f;
  output_enabled_prev_ = false;
  drive_feedback_ = {};
  drive_permitted_ = estop_ack_seen_ = disabled_feedback_seen_ = false;
  arm_started_tick_ = 0;
  initialized_ = (rc_ != nullptr) && (imu_ != nullptr) && (gimbal_ != nullptr) &&
                 (comm_ != nullptr);
}

/* ============================================================================
 * 云台主流程（每 1 ms 一次）
 * ---------------------------------------------------------------------------
 *   1) IMU 采样（故障注入时跳过）；
 *   2) 读遥控 + 板间帧；
 *   3) 状态机（Dead/Resurrection/Working）；
 *   4) 生成并下发两轴力矩；
 *   5) 写通信帧（含转发的遥控数据）。
 * ========================================================================== */
void robot::Robot::update()
{
  if (!initialized_) {
    return;
  }

  /* 1. 传感器（故障注入 kImuFrozen 时跳过采样，见 docs/08） */
  if (!robot::FaultActive(robot::FaultId::kImuFrozen)) {
    imu_->update();
  }

  /* 2. 输入（急停判据） */
  updateInput();

  /* 3. 状态迁移 */
  updateStateMachine();

  /* 4. 生成并下发指令 */
  applyOutput(kDt);

  /* 4.5 调试快照：状态/授权（运动量由 Gimbal::update 填） */
  {
    auto& d = diag::g_gimbal_dbg;
    d.state = static_cast<uint8_t>(state_);
    d.estop = estop_ ? 1u : 0u;
    d.authorized = output_enabled_prev_ ? 1u : 0u;
    d.stick_rh = remote_input_.rh;
    d.yaw_rate_cmd = output_enabled_prev_ && imu_control_state_ == ImuControlState::kFresh ?
        peer_input_.yaw_rate : 0.0f;
  }

  /* 5. 回填通信帧 */
  updateCommData();
}

void robot::Robot::notifyRemoteRx()
{
  rc_checker_.update();
}

void robot::Robot::notifyCanTxCycle(bool success)
{
  if (!initialized_) return;
  const bool reset_ack = operatorEstopConfirmed();
  can_tx_guard_.update(success, reset_ack, hello_world::tick::GetTickMs());
  if (!can_tx_guard_.healthy()) {
    diag::g_gimbal_dbg.dead_reason = static_cast<uint8_t>(diag::g_gimbal_dbg.dead_reason | 32u);
    enterState(LifeState::kDead);
    stopOutputs();
    updateCommData();
  }
}

void robot::Robot::notifyMotorRx(int slot)
{
  if (slot >= 0 && slot < kMotorSlotNum) {
    motor_checkers_[slot].update();
  }
}

bool robot::Robot::isRemoteOnline()
{
  CriticalSection lock;
  return !rc_checker_.isOffline();
}

bool robot::Robot::areAllMotorsOnline()
{
  CriticalSection lock;
  for (int i = 0; i < kMotorSlotNum; ++i) {
    if (motor_checkers_[i].isOffline()) {
      return false;
    }
  }
  return true;
}

/* Private function definitions ----------------------------------------------*/

void robot::Robot::updateInput()
{
  {
    CriticalSection lock;
    remote_input_.capture(rc_, !rc_checker_.isOffline());
    peer_input_ = comm_->chassis_data();
    peer_online_ = !comm_->isOffline();
    motors_online_ = areAllMotorsOnline();
    gimbal_->captureFeedback();
    drive_feedback_ = GetGimbalDriveFeedback();
  }
  remote_input_.normalize();
  /* 本地急停：只有「本板确实有遥控器且在线」时才采信它的拨杆，
   * 否则云台板没接收机时会被误判成一直急停。 */
  const bool local_estop =
      remote_input_.online &&
      (remote_input_.right == hello_world::remote_control::kSwitchStateDown);

  /* 底盘板下发的急停（底盘板那边已经把自己遥控器的急停算进去了） */
  const bool peer_estop =
      peer_online_ ? peer_input_.estop : true;

  estop_ = local_estop || peer_estop;
}

void robot::Robot::updateStateMachine()
{
  const bool comm_ok = peer_online_;
  const uint32_t now = hello_world::tick::GetTickMs();
  const bool allow_imu_hold = state_ == LifeState::kWorking && !estop_ && output_enabled_prev_;
  imu_control_state_ = EvaluateImuForControl(imu_, now, allow_imu_hold);
  const bool imu_ok = imu_control_state_ != ImuControlState::kUnavailable;
  // 正常失能仍有反馈，允许双板自检。已经在出力时意外失能必须撤销授权。
  const bool motors_ok = motors_online_ && drive_feedback_.fault_free && gimbal_->feedbackFinite();
  const bool lost_enable = output_enabled_prev_ && !estop_ && !gimbal_->feedbackHealthy();
  const bool arm_timeout = drive_permitted_ && !output_enabled_prev_ && !estop_ &&
                           !gimbal_->feedbackHealthy() && now - arm_started_tick_ >= kMotorOfflineMs;
  const bool all_ok = comm_ok && imu_ok && motors_ok && !lost_enable && !arm_timeout &&
                      can_tx_guard_.healthy();

  /* 记录 all_ok 的失败位，便于实车一次复现就定位 Dead 原因。 */
  {
    uint8_t reason = 0;
    if (!comm_ok) reason |= 1u;
    if (!imu_ok) reason |= 2u;
    if (!motors_ok) reason |= 4u;
    if (lost_enable) reason |= 8u;
    if (arm_timeout) reason |= 16u;
    if (!can_tx_guard_.healthy()) reason |= 32u;
    diag::g_gimbal_dbg.dead_reason = reason;
  }

  switch (state_) {
    case LifeState::kDead:
      if (all_ok) {
        enterState(LifeState::kResurrection);
      }
      break;

    case LifeState::kResurrection:
      if (!all_ok) {
        enterState(LifeState::kDead);
      } else if ((now - state_enter_tick_) >= kResurrectionMs) {
        enterState(LifeState::kWorking);
      }
      break;

    case LifeState::kWorking:
      if (!all_ok) {
        enterState(LifeState::kDead);
      }
      break;

    default:
      enterState(LifeState::kDead);
      break;
  }
}

void robot::Robot::enterState(LifeState next)
{
  if (next == state_) {
    return;
  }
  state_ = next;
  state_enter_tick_ = hello_world::tick::GetTickMs();

  /* 离开 Working 就停机；进入 Working 时的目标对齐在 applyOutput 的
   * 使能上升沿里完成（因为那时才知道「从这一刻开始真的允许出力」）。 */
  if (next != LifeState::kWorking) {
    if (next == LifeState::kDead) {
      estop_ack_seen_ = false;
      diag::g_gimbal_dbg.last_dead_reason = diag::g_gimbal_dbg.dead_reason;
      diag::g_gimbal_dbg.last_dead_tick_ms = state_enter_tick_;
    }
    stopOutputs();
  }
}

bool robot::Robot::operatorEstopConfirmed() const
{
  return (remote_input_.online && remote_input_.right == hello_world::remote_control::kSwitchStateDown) ||
         (peer_online_ && peer_input_.alive && peer_input_.dbus_online && peer_input_.estop);
}

void robot::Robot::stopOutputs()
{
  if (drive_permitted_) disabled_feedback_seen_ = false;
  drive_permitted_ = output_enabled_prev_ = false;
  diag::g_gimbal_dbg.authorized = 0u;
  diag::g_gimbal_dbg.yaw_rate_cmd = 0.0f;
  gimbal_->stop();
}

/* ============================================================================
 * 云台输出（授权链 + 目标 + 闭环）
 * ---------------------------------------------------------------------------
 * 授权条件同时满足才出力：
 *   Working 状态、未急停、已确认过急停、对板在线、对板帧 alive/working/
 *   dbus_online、对板允许 gimbal_enable、对板未急停。
 * 另外要求两轴电机反馈为"正常失能"（both_disabled）才允许恢复，避免
 * 电机掉线后假装使能。
 * 通过后：yaw 世界目标积分、pitch 目标（底盘下发 - IMU 俯仰做水平补偿）、
 * 调 gimbal_->update() 完成两轴闭环。
 * ========================================================================== */
void robot::Robot::applyOutput(float dt)
{
#if defined(ROBOT_GROUND_GIMBAL_LOCK) && ROBOT_GROUND_GIMBAL_LOCK
  (void)dt;
  stopOutputs();  // 通信层始终发两轴失能；保持 DBUS 转发与反馈接收。
#else
  const auto& command = peer_input_;
  const bool confirmed_estop = operatorEstopConfirmed();
  if (confirmed_estop) estop_ack_seen_ = true;
  // 世界系闭环用本板 IMU，自检已在状态机完成。对板 IMU 仅作遥测；
  // 真正的对板授权取 gimbal_enable，不能用 imu_ready 再否决一次。
  const bool allowed = state_ == LifeState::kWorking && !estop_ && estop_ack_seen_ && peer_online_ &&
                       command.alive && command.working && command.dbus_online &&
                       command.gimbal_enable && !command.estop;
  const bool both_disabled = motors_online_ && drive_feedback_.fault_free &&
      drive_feedback_.disabled_mask == GimbalDriveFeedback::kBothDisabled;
  if (!allowed || (!drive_permitted_ && (!disabled_feedback_seen_ || !both_disabled))) {
    if (!allowed && drive_permitted_ && !confirmed_estop) estop_ack_seen_ = false;
    stopOutputs();
    // 停机时双轴失能曾被确认还不够；最新反馈改变即作废，继续发失能而不发使能。
    disabled_feedback_seen_ = both_disabled;
    return;
  }

  if (!drive_permitted_) arm_started_tick_ = hello_world::tick::GetTickMs();
  drive_permitted_ = true;
  /* 允许通信任务尝试使能；两轴尚未确认使能时，闭环仍不运行。 */
  if (!gimbal_->feedbackHealthy()) {
    gimbal_->stop();
    output_enabled_prev_ = false;
    return;
  }

  /* ---------- 使能上升沿：把目标对齐到当前实际角度 ---------- */
  if (!output_enabled_prev_) {
    yaw_target_world_ = imu_->yaw();          ///< 世界系目标 = 当前朝向
    gimbal_->setPitchTarget(gimbal_->pitchJoint());  ///< pitch 目标 = 当前角度
    gimbal_->resetControllers();  ///< 清历史但保留输入阶段的反馈快照
    output_enabled_prev_ = true;
  }

  /* ---------- yaw：积分离散转速设定 ----------
   * 底盘板下发的是「转多快」（rad/s），这里积分成世界系目标角。
   * 为什么不直接让它下发角度：世界角只有持有 IMU 的本板才能定义，
   * 详见 gimbal_chassis_comm.hpp 的说明。 */
  // 短暂保留上一份姿态期间，冻结 Yaw 目标并去掉速率前馈，不在旧角度上继续积累运动。
  const float yaw_rate =
      imu_control_state_ == ImuControlState::kFresh ? command.yaw_rate : 0.0f;
  // 目标与反馈都是周期角；每周期归一化，避免长时间连续自旋丢失 float 小增量。
  yaw_target_world_ = robot::WrapPi(yaw_target_world_ + yaw_rate * dt);
  gimbal_->setYawWorldTarget(yaw_target_world_);

  /* ---------- pitch：底盘下发的关节角目标，减去车体俯仰做世界水平补偿 ----------
   * 直行加减速时悬挂让车体俯仰，若只控关节角，炮管会跟着后仰/前倾。
   * IMU 装在云台底座，其 pitch 就是车体俯仰；关节目标减去它即可保持水平。
   * 若实车方向相反（越补越仰），把这里的减号翻成加号。 */
  gimbal_->setPitchTarget(command.pitch_target - imu_->pitch());

  /* ---------- 闭环 ---------- */
  gimbal_->setYawRateFeedforward(yaw_rate);
  gimbal_->update(dt, imu_->yaw(), WorldYawRate(*imu_));
  if (!gimbal_->controlValid()) {
    diag::g_gimbal_dbg.dead_reason = 64u;  // 状态前置条件通过；本拍执行控制失败。
    enterState(LifeState::kDead);
  }
#endif
}

void robot::Robot::updateCommData()
{
  if (comm_ == nullptr) {
    return;
  }
  GimbalChassisComm::GimbalData& d = comm_->gimbal_data();
  d.working = (state_ == LifeState::kWorking);
  // 只回传本地急停；C2G 的组合急停不能回显，否则两板会互相锁住急停。
  d.estop = remote_input_.online &&
            remote_input_.right == hello_world::remote_control::kSwitchStateDown;
  d.dbus_online = remote_input_.online;
  d.imu_ready = imu_control_state_ != ImuControlState::kUnavailable;
  d.pitch_at_limit = gimbal_->pitchAtLimit();
#if defined(ROBOT_GROUND_GIMBAL_LOCK) && ROBOT_GROUND_GIMBAL_LOCK
  d.ground_disarmed = d.working && motors_online_ && drive_feedback_.fault_free &&
      drive_feedback_.disabled_mask == GimbalDriveFeedback::kBothDisabled;
#endif
  d.yaw_joint = gimbal_->yawJoint();
  d.pitch_joint = gimbal_->pitchJoint();
}
