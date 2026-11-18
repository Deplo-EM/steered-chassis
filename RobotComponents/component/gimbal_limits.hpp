/**
 * @file gimbal_limits.hpp
 * @brief 底盘目标生成与云台反馈限位共用的 Pitch 关节工作范围。
 */
#ifndef ROBOT_GIMBAL_LIMITS_HPP_
#define ROBOT_GIMBAL_LIMITS_HPP_

namespace robot::gimbal_limits {
/** 下限：实测机械止点约 −0.50 rad，软件限位留 0.06 余量（约 3.4°）。
 *  原 −0.47 只有 0.03 余量，实车底部仍会顶到机械止点。 */
inline constexpr float kPitchLowerRad = -0.44f;
inline constexpr float kPitchUpperRad = 0.70f;
}  // namespace robot::gimbal_limits

#endif  // ROBOT_GIMBAL_LIMITS_HPP_
