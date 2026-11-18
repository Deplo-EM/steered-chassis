/**
 * @file gimbal_limits.hpp
 * @brief 底盘目标生成与云台反馈限位共用的 Pitch 关节工作范围。
 */
#ifndef ROBOT_GIMBAL_LIMITS_HPP_
#define ROBOT_GIMBAL_LIMITS_HPP_

namespace robot::gimbal_limits {
/** 下限：机械止点约 −0.50 rad；−0.40 实测已不顶，留 0.10 余量（约 5.7°）。 */
inline constexpr float kPitchLowerRad = -0.40f;
/** 上限：原 +0.70 实车顶死；+0.62 仍顶，收到 +0.56，留 0.14 余量（约 8°）。 */
inline constexpr float kPitchUpperRad = 0.56f;
}  // namespace robot::gimbal_limits

#endif  // ROBOT_GIMBAL_LIMITS_HPP_
