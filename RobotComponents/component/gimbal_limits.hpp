/**
 * @file gimbal_limits.hpp
 * @brief 底盘目标生成与云台反馈限位共用的 Pitch 关节工作范围。
 */
#ifndef ROBOT_GIMBAL_LIMITS_HPP_
#define ROBOT_GIMBAL_LIMITS_HPP_

namespace robot::gimbal_limits {
/** 下限：机械止点约 −0.50 rad；−0.44 实车仍会顶到，改留 0.10 余量（约 5.7°）。
 *  若仍顶到，说明机械止点比 −0.50 更高，需要实测后重定。 */
inline constexpr float kPitchLowerRad = -0.40f;
inline constexpr float kPitchUpperRad = 0.70f;
}  // namespace robot::gimbal_limits

#endif  // ROBOT_GIMBAL_LIMITS_HPP_
