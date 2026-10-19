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
#include "offline_checker.hpp"

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

  /* ---------------- 状态查询 ---------------- */
  LifeState lifeState() const { return state_; }
  bool isEstopped() const { return estop_; }
  bool isRemoteOnline();
  bool areAllMotorsOnline();

 private:
  void updateInput();
  void updateStateMachine();
  void applyOutput(float dt);
  void updateCommData();

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

  bool estop_ = false;
  /** yaw 世界系目标（连续累加，不做归一化） */
  float yaw_target_world_ = 0.0f;
  /** 上一周期是否处于「允许出力」状态，用于检测使能上升沿 */
  bool output_enabled_prev_ = false;
};

}  // namespace robot

#endif /* GIMBAL_ROBOT_MODULES_ROBOT_HPP_ */
