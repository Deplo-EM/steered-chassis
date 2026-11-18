/**
 * @file gimbal_debug.hpp
 * @brief 云台调试快照：把 Ozone 常用量收进一个符号，免去翻符号树。
 *
 * 只读观察，不参与控制。Ozone 里 Watch 一个 `robot::diag::g_gimbal_dbg`，
 * 展开即可看到授权、Pitch/Yaw 的目标、反馈、PID 输出与积分。
 *
 * 单位：角度 rad，力矩 N·m，积分状态 rad·s。
 */
#ifndef ROBOT_GIMBAL_DEBUG_HPP_
#define ROBOT_GIMBAL_DEBUG_HPP_

#include <cstdint>

namespace robot::diag {

struct GimbalDebug {
  volatile uint8_t state;       ///< 0=Dead, 1=Resurrection, 2=Working
  volatile uint8_t estop;       ///< 1=急停中
  volatile uint8_t authorized;  ///< 1=两轴反馈已使能，且本周期控制计算有效。
  volatile uint8_t dead_reason; ///< 当拍失败位：1=通信 2=IMU 4=电机 8=掉使能 16=使能超时 32=CAN 64=控制拒绝
  volatile uint8_t last_dead_reason;  ///< 最近一次进入 Dead 的原因，恢复后保留。
  volatile uint32_t last_dead_tick_ms;

  volatile float pitch_target;      ///< 关节目标角 rad
  volatile float pitch_feedback;    ///< 关节反馈角 rad
  volatile float pitch_output;      ///< Pitch PID 输出 N·m（力矩请求）
  volatile float pitch_integral;    ///< Pitch 积分状态 rad·s
  volatile float pitch_torq_meas;   ///< Pitch 实测力矩反馈 N·m

  volatile float yaw_target;        ///< 世界系目标角 rad
  volatile float yaw_world_feedback;///< 世界系反馈角 rad
  volatile float yaw_output;        ///< Yaw 反馈与前馈合成、限幅后的力矩请求 N·m
  volatile float yaw_integral;      ///< Yaw 积分状态 rad·s
  volatile float yaw_err;           ///< 世界系误差（已 WrapPi）rad
  volatile float yaw_rate_cmd;      ///< 本周期采用的底盘 yaw 速率 rad/s；停机时为 0。
  volatile float stick_rh;          ///< 本地右杆水平值（已归一化）。
  volatile float yaw_ff_torque;     ///< 限幅前的前馈分量 N·m；停机时为 0。
};

extern volatile GimbalDebug g_gimbal_dbg;

}  // namespace robot::diag

#endif  // ROBOT_GIMBAL_DEBUG_HPP_
