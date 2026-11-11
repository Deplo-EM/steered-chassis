/**
 *******************************************************************************
 * @file      :robot.hpp
 * @brief     : 底盘板顶层状态机（本板「大脑」）
 * @history   :
 *  Version     Date            Author          Note
 *  V1.0.0      2026-10-19      <author>        1. 完成编写
 *******************************************************************************
 * @attention :
 *  【职责】
 *  1ms 进来一次，按「更新数据 → 状态机迁移 → 生成指令 → 下发」四步走：
 *    - 更新数据：读遥控器、读双板通信帧、更新 IMU；
 *    - 状态迁移：Dead / Resurrection / Working 三态；
 *    - 生成指令：按工作模式算出 (vx, vy, w) 和云台目标；
 *    - 下发：交给 Chassis 逆解 + 闭环，最后写进电机对象。
 *
 *  【急停为什么放在最外层】
 *  如果只在某个分支里判断急停，那么新增一个模式就可能忘掉它。
 *  这里在「下发」之前统一判一次：只要急停生效，无论什么模式、什么状态，
 *  电机立刻归零。这样安全性不依赖分支写得全不全。
 *
 *  【与其他模块的关系】
 *      Task ──调用──> Robot ──调用──> Chassis ──写──> Motor
 *                        │
 *                        └──读写──> GimbalChassisComm（两块板之间唯一的联系）
 *  Robot 不直接碰 CAN / 串口：收在中断回调里（CommTask / HW_can.cpp），
 *  发在 CommTask 里。这样 Robot 只关心「控制」，通信细节不污染控制逻辑。
 *******************************************************************************
 *  Copyright (c) 2024 Hello World Team, Zhejiang University.
 *  All Rights Reserved.
 *******************************************************************************
 */
/* Define to prevent recursive inclusion -------------------------------------*/
#ifndef CHASSIS_ROBOT_MODULES_ROBOT_HPP_
#define CHASSIS_ROBOT_MODULES_ROBOT_HPP_

/* Includes ------------------------------------------------------------------*/
#include <cstdint>

#include "module_state.hpp"
#include "can_tx_guard.hpp"
#include "remote_input.hpp"
#include "gimbal_chassis_comm.hpp"
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

class Chassis;
class GimbalChassisComm;

/* Exported types ------------------------------------------------------------*/

/**
 * @brief 底盘板顶层状态机
 */
class Robot {
 public:
  /** 电机在线检测槽位数：0~3 = 驱动轮，4~7 = 舵电机 */
  static constexpr int kMotorSlotNum = 8;

  Robot() = default;
  ~Robot() = default;

  /* ---------------- 注册（由 MainTaskInit 按依赖顺序调用） ---------------- */

  void setRemoteControl(hello_world::remote_control::DT7* rc) { rc_ = rc; }
  void setImu(hello_world::imu::Imu* imu) { imu_ = imu; }
  void setChassis(Chassis* chassis) { chassis_ = chassis; }
  void setComm(GimbalChassisComm* comm) { comm_ = comm; }

  /** 一次性初始化：状态清零、离线检测器设定阈值 */
  void init();

  /** 1ms 周期入口（在 TIM6 中断里调用） */
  void update();

  /* ---------------- 中断侧通知（由通信层调用） ---------------- */

  /** 收到遥控器数据时调用（HAL_UARTEx_RxEventCallback 里） */
  void notifyRemoteRx();

  /** 收到某号电机的反馈帧时调用（HW_can.cpp 的分发里） */
  void notifyMotorRx(int slot);

  /** CommTask 完成全部关键帧后报告；失败立即清零，恢复需有效急停确认。 */
  void notifyCanTxCycle(bool success);
  bool canTxHealthy() const { return can_tx_guard_.healthy(); }
  bool canFeedWatchdog() const { return can_tx_guard_.canFeedWatchdog(); }
  uint32_t canTxFailedCycles() const { return can_tx_guard_.failedCycles(); }

  /* ---------------- 状态查询（调试 / 上层判断） ---------------- */

  LifeState lifeState() const { return state_; }
  ChassisMode chassisMode() const { return mode_; }
  bool isEstopped() const { return estop_; }
  bool isRemoteOnline();
  bool areAllMotorsOnline();
  /** 本周期下发资格，用于通信层停机沿撤销旧队列。 */
  bool outputPermitted() const { return state_ == LifeState::kWorking && !estop_; }

  /** 当前（已做加速度限幅的）云台系速度指令 */
  const ChassisMotion& motion() const { return ramped_; }

  /** 云台相对车头的偏航角 δ（来自 G2C 帧，单位 rad） */
  float delta() const;

 private:
  /* 四步流程 */
  void updateInput();
  void updateStateMachine();
  void generateCommand();
  void applyOutput();
  void updateCommData();

  /* 内部工具 */
  void enterState(LifeState next);
  void initWorkingTargets();
  ChassisMode readModeSwitch() const;
  bool readEstopSwitch() const;
  void updateGimbalCommand();

  /* 外部模块（只借用，不拥有；生命周期由 Instance 层保证） */
  hello_world::remote_control::DT7* rc_ = nullptr;
  hello_world::imu::Imu* imu_ = nullptr;
  Chassis* chassis_ = nullptr;
  GimbalChassisComm* comm_ = nullptr;

  /* 在线检测 */
  hello_world::OfflineChecker rc_checker_;
  hello_world::OfflineChecker motor_checkers_[kMotorSlotNum];

  /* 状态机 */
  LifeState state_ = LifeState::kDead;
  uint32_t state_enter_tick_ = 0;
  bool initialized_ = false;
  CanTxGuard can_tx_guard_;
  RemoteInput remote_input_;
  GimbalChassisComm::GimbalData peer_input_;
  bool peer_online_ = false;
  bool motors_online_ = false;

  /* 指令 */
  ChassisMode mode_ = ChassisMode::kSeparate;
  bool estop_ = false;
  bool remote_armed_ = false;  ///< 上电/遥控失联/发送失败后需当前来源急停确认
  enum class RemoteSource : uint8_t { kNone, kLocal, kGimbal };
  RemoteSource remote_source_ = RemoteSource::kNone; ///< 来源只能通过新来源急停确认选定
  bool local_estop_ = false; ///< 即使当前使用云台遥控，本地有效急停仍有最高优先级
  ChassisMotion ramped_;  ///< 加速度限幅后的云台系速度指令
  float gimbal_yaw_rate_ = 0.0f;
  float gimbal_pitch_target_ = 0.0f;
};

}  // namespace robot

#endif /* CHASSIS_ROBOT_MODULES_ROBOT_HPP_ */
