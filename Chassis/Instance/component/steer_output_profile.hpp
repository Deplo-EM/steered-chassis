#ifndef CHASSIS_STEER_OUTPUT_PROFILE_HPP_
#define CHASSIS_STEER_OUTPUT_PROFILE_HPP_

namespace robot::steer_output {

// 仅用于已开启电流环的舵机。默认仍按考核资料使用电压模式。
#if defined(ROBOT_STEER_CURRENT_FIELD) && ROBOT_STEER_CURRENT_FIELD
inline constexpr bool kUseCurrent = true;
#else
inline constexpr bool kUseCurrent = false;
#endif

inline constexpr float kPidOutputLimit = 20000.0f;
inline constexpr float kFieldCurrentLimitA = 2.0f;  ///< 实验：1.2→2.0 A，GM6020 余量内
// 以现有 PID 输出上限映射现场初测电流；这是初测增益，不是电压和电流的物理换算。
constexpr float MotorInput(float pid_output)
{
  if constexpr (kUseCurrent) return pid_output * (kFieldCurrentLimitA / kPidOutputLimit);
  return pid_output;
}

}  // namespace robot::steer_output

#endif  // CHASSIS_STEER_OUTPUT_PROFILE_HPP_
