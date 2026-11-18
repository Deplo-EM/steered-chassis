/**
 * @file gimbal_limits.hpp
 * @brief 底盘目标生成与云台反馈限位共用的 Pitch 关节工作范围。
 */
#ifndef ROBOT_GIMBAL_LIMITS_HPP_
#define ROBOT_GIMBAL_LIMITS_HPP_

namespace robot::gimbal_limits {
inline constexpr float kPitchLowerRad = -0.47f;
inline constexpr float kPitchUpperRad = 0.70f;
}  // namespace robot::gimbal_limits

#endif  // ROBOT_GIMBAL_LIMITS_HPP_
