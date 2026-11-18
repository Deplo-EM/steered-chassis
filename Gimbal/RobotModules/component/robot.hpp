/**
 *******************************************************************************
 * @file      :robot.hpp
 * @brief     : 云台板顶层状态机
 * @history   :
 *  Version     Date            Author          Note
 *  V1.0.0      2026-10-19      <author>        1. 完成编写
 *******************************************************************************
 * @attention :
 *  【云台板的职责边界】
 *  云台板不决定「车往哪走」，它只负责「枪口指向哪」。所以它收到的 C2G 帧里
 *  只有两件事：yaw 转速设定、pitch 角度目标。其余信息（模式、急停、使能）
 *  都只是「许可 / 禁止」的信号。
 *
 *  【两板互保】
 *  - 底盘板离线 → 云台板 100ms 后判定掉线 → 停止出力；200ms 后看门狗直接把
 *    板子复位（因为喂狗依赖底盘帧里的「遥控器在线」标志）。
 *  - 云台板不 Working → 底盘板不允许进入 Working（车不会在枪口乱晃时开动）。
 *  两条合起来就是考核加分项里的「双板互保」。
 *******************************************************************************
 *  Copyright (c) 2024 Hello World Team, Zhejiang University.
 *  All Rights Reserved.
 *******************************************************************************
 */
/* Define to prevent recursive inclusion -------------------------------------*/
#ifndef GIMBAL_ROBOT_MODULES_ROBOT_HPP_
#define GIMBAL_ROBOT_MODULES_ROBOT_HPP_

/* Includes ------------------------------------------------------------------*/
#include <cstdint>

#include "module_state.hpp"
#include "can_tx_guard.hpp"
#include "remote_input.hpp"
#include "gimbal_chassis_comm.hpp"
#include "offline_checker.hpp"
#include "gimbal_drive_feedback.hpp"
#include "imu_control_state.hpp"

namespace hello_world {
namespace imu {
class Imu;
}  // namespace imu
namespace remote_control {
class DT7;
}  // namespace remote_control
}  // namespace hello_world

namespace robot {

class Gimbal;
class GimbalChassisComm;

/* Exported types ------------------------------------------------------------*/

/**
 * @brief 云台板顶层状态机
 */
class Robot {
 public:
  /** 电机在线检测槽位：0 = yaw，1 = pitch */
  static constexpr int kMotorSlotNum = 2;

  Robot() = default;
  ~Robot() = default;

  /* ---------------- 注册 ---------------- */
  void setRemoteControl(hello_world::remote_control::DT7* rc) { rc_ = rc; }
  void setImu(hello_world::imu::Imu* imu) { imu_ = imu; }
  void setGimbal(Gimbal* gimbal) { gimbal_ = gimbal; }
  void setComm(GimbalChassisComm* comm) { comm_ = comm; }

  void init();
  void update();

  /* ---------------- 中断侧通知 ---------------- */
  void notifyRemoteRx();
  void notifyMotorRx(int slot);

  /** CommTask 完成全部关键帧后报告；失败立即清零，恢复需有效急停确认。 */
  void notifyCanTxCycle(bool success);
  bool canTxHealthy() const { return can_tx_guard_.healthy(); }
  bool canFeedWatchdog() const { return can_tx_guard_.canFeedWatchdog(); }
  uint32_t canTxFailedCycles() const { return can_tx_guard_.failedCycles(); }

  /* ---------------- 状态查询 ---------------- */
  LifeState lifeState() const { return state_; }
  bool isEstopped() const { return estop_; }
  bool isRemoteOnline();
  bool areAllMotorsOnline();
  /** 本周期可发控制/使能帧；false 时通信层只能发失能。 */
  bool outputPermitted() const {
#if defined(ROBOT_GROUND_GIMBAL_LOCK) && ROBOT_GROUND_GIMBAL_LOCK
    return false;
#else
    return drive_permitted_;
#endif
  }
  /** 只在已授权且尚未给力矩时，给仍报告正常失能的轴送使能帧。 */
  bool motorEnableRequested(int slot) const {
    return outputPermitted() && !output_enabled_prev_ && slot >= 0 && slot < kMotorSlotNum &&
           (drive_feedback_.disabled_mask & (1u << slot)) != 0u;
  }

 private:
  void updateInput();
  void updateStateMachine();
  void applyOutput(float dt);
  void updateCommData();
  void stopOutputs();
  bool operatorEstopConfirmed() const;

  void enterState(LifeState next);

  hello_world::remote_control::DT7* rc_ = nullptr;
  hello_world::imu::Imu* imu_ = nullptr;
  Gimbal* gimbal_ = nullptr;
  GimbalChassisComm* comm_ = nullptr;

  hello_world::OfflineChecker rc_checker_;
  hello_world::OfflineChecker motor_checkers_[kMotorSlotNum];

  LifeState state_ = LifeState::kDead;
  uint32_t state_enter_tick_ = 0;
  bool initialized_ = false;
  CanTxGuard can_tx_guard_;
  RemoteInput remote_input_;
  GimbalChassisComm::ChassisData peer_input_;
  bool peer_online_ = false;
  bool motors_online_ = false;
  GimbalDriveFeedback drive_feedback_;
  ImuControlState imu_control_state_ = ImuControlState::kUnavailable;

  bool estop_ = false;
  bool drive_permitted_ = false;       ///< 综合授权，可开始使能握手。
  bool estop_ack_seen_ = false;        ///< 上电、故障或丢授权后需观察到有效急停档。
  bool disabled_feedback_seen_ = false; ///< 禁止输出时，最近两轴均报告正常失能。
  uint32_t arm_started_tick_ = 0;
  /** yaw 世界系目标（每周期归一化至 [-pi, pi)，误差也按最短角计算） */
  float yaw_target_world_ = 0.0f;
  /** 上一周期是否处于「允许出力」状态，用于检测使能上升沿 */
  bool output_enabled_prev_ = false;
};

}  // namespace robot

#endif /* GIMBAL_ROBOT_MODULES_ROBOT_HPP_ */
