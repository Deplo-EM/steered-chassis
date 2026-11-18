/**
 * @file board_wiring.hpp
 * @brief 双板 CAN 路由，以最后提供的修正拓扑图为准。
 * @note 底盘CAN2与云台CAN1是同一条物理总线，yaw也在该总线上。
 *       底盘 CAN1 接 GM6020，CAN2 接 M3508、yaw 和云台板；pitch 在云台 CAN2。
 */
#ifndef ROBOT_BOARD_WIRING_HPP_
#define ROBOT_BOARD_WIRING_HPP_
#include "can.h"
namespace robot::wiring {
enum class CanPort { kCan1, kCan2 };
constexpr CanPort kChassisWheel = CanPort::kCan2;
constexpr CanPort kChassisSteer = CanPort::kCan1;
constexpr CanPort kChassisPeer = CanPort::kCan2;
constexpr CanPort kGimbalYaw = CanPort::kCan1;
constexpr CanPort kGimbalPitch = CanPort::kCan2;
constexpr CanPort kGimbalPeer = CanPort::kCan1;
inline CAN_HandleTypeDef* Can(CanPort port) {
  return port==CanPort::kCan1 ? &hcan1 : &hcan2;
}
}  // namespace robot::wiring
#endif
