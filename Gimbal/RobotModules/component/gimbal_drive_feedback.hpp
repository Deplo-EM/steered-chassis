/**
 * @file gimbal_drive_feedback.hpp
 * @brief 云台两轴的驱动反馈；正常失能允许自检就绪，但不能出力。
 */
#ifndef GIMBAL_DRIVE_FEEDBACK_HPP_
#define GIMBAL_DRIVE_FEEDBACK_HPP_

#include <cstdint>

namespace robot {

struct GimbalDriveFeedback {
  static constexpr uint8_t kBothDisabled = 0x03u;
  bool fault_free = false;
  uint8_t disabled_mask = 0;  ///< bit0=Yaw，bit1=Pitch；只包含正常失能的驱动。
};

}  // namespace robot

/** 与电机角度和在线状态一起在临界区读取；本函数不发送命令。 */
robot::GimbalDriveFeedback GetGimbalDriveFeedback();

#endif  // GIMBAL_DRIVE_FEEDBACK_HPP_
