/**
 * @file board_wiring.hpp
 * @brief CAN发送路由；依据用户提供的双C板接线图集中配置。
 * @note 底盘CAN2与云台CAN1是同一条物理总线，yaw也在该总线上。
 *       接收按ID全收并不改变发送端口；若队伍调整线束，只在此核对整组路由。
 */
#ifndef ROBOT_BOARD_WIRING_HPP_
#define ROBOT_BOARD_WIRING_HPP_
#include "can.h"
namespace robot::wiring {
enum class CanPort { kCan1, kCan2 };
constexpr CanPort kChassisWheel = CanPort::kCan1;
constexpr CanPort kChassisSteer = CanPort::kCan2;
constexpr CanPort kChassisPeer = CanPort::kCan2;
constexpr CanPort kGimbalYaw = CanPort::kCan1;
constexpr CanPort kGimbalPitch = CanPort::kCan2;
constexpr CanPort kGimbalPeer = CanPort::kCan1;
inline CAN_HandleTypeDef* Can(CanPort port) {
  return port==CanPort::kCan1 ? &hcan1 : &hcan2;
}
}  // namespace robot::wiring
#endif
