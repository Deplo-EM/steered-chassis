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

#include "DT7.hpp"
#include "fault_injection.hpp"
#include "gimbal.hpp"
#include "gimbal_chassis_comm.hpp"
#include "imu.hpp"
#include "tick.hpp"
#include "critical_section.hpp"
#include "basic_mode.hpp"
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
  yaw_target_world_ = 0.0f;
  output_enabled_prev_ = false;
  initialized_ = (rc_ != nullptr) && (imu_ != nullptr) && (gimbal_ != nullptr) &&
                 (comm_ != nullptr);
}

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
  const bool reset_ack = peer_online_ && peer_input_.dbus_online && peer_input_.estop;
  can_tx_guard_.update(success, reset_ack, hello_world::tick::GetTickMs());
  if (!can_tx_guard_.healthy()) {
    enterState(LifeState::kDead);
    gimbal_->stop();
    output_enabled_prev_ = false;
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
#if ROBOT_BASIC_MODE
  const bool imu_ok = (imu_ != nullptr) && imu_->isCalibrated();
#else
  const bool imu_ok =
      (imu_ != nullptr) && imu_->isCalibrated() && imu_->isHealthy() && imu_->isSampleValid();
#endif
  const bool motors_ok = motors_online_ && gimbal_->feedbackHealthy();
  const bool all_ok = comm_ok && imu_ok && motors_ok && can_tx_guard_.healthy();

  const uint32_t now = hello_world::tick::GetTickMs();

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
    gimbal_->stop();
    output_enabled_prev_ = false;
  }
}

void robot::Robot::applyOutput(float dt)
{
  if (state_ != LifeState::kWorking) {
    gimbal_->stop();
    output_enabled_prev_ = false;
    return;
  }

  /* 急停闸门（最外层） */
  if (estop_) {
    gimbal_->stop();
    output_enabled_prev_ = false;
    return;
  }

  /* 接收端独立核对授权条件，不能只信任对板的一个 enable 位。 */
  const auto& command = peer_input_;
  const bool enabled = command.alive && command.working && command.dbus_online &&
                       command.imu_ready && command.gimbal_enable && !command.estop;
  if (!enabled) {
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
  const float yaw_rate = command.yaw_rate;
  // 目标与反馈都是周期角；每周期归一化，避免长时间连续自旋丢失 float 小增量。
  yaw_target_world_ = robot::WrapPi(yaw_target_world_ + yaw_rate * dt);
  gimbal_->setYawWorldTarget(yaw_target_world_);

  /* ---------- pitch：直接采用底盘下发的关节角目标（模块内部再限位） ---------- */
  gimbal_->setPitchTarget(command.pitch_target);

  /* ---------- 闭环 ---------- */
  gimbal_->update(dt, imu_->yaw());
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
  d.imu_ready = (imu_ != nullptr) && imu_->isCalibrated() && imu_->isHealthy() && imu_->isSampleValid();
  d.pitch_at_limit = gimbal_->pitchAtLimit();
  d.yaw_joint = gimbal_->yawJoint();
  d.pitch_joint = gimbal_->pitchJoint();
}
