/**
 * @file imu_control_state.hpp
 * @brief 控制层如何使用 IMU 最新的合格结果；不改传感器判废或融合逻辑。
 */
#ifndef ROBOT_IMU_CONTROL_STATE_HPP_
#define ROBOT_IMU_CONTROL_STATE_HPP_

#include <cstdint>

#include "imu_fault_detail.hpp"

namespace robot {

enum class ImuControlState : uint8_t { kUnavailable, kFresh, kHeldAfterAccLow };
inline constexpr uint32_t kKeepLastImuMs = 8u;

/** 只有已授权时允许短暂保留上一个解算值。时钟、传输等故障仍须立即撤销授权。 */
template <class Imu>
ImuControlState EvaluateImuForControl(const Imu* imu, uint32_t now, bool allow_hold)
{
  if (imu == nullptr || !imu->isCalibrated() || !imu->isHealthy()) return ImuControlState::kUnavailable;
  if (imu->isSampleValidAt(now)) return ImuControlState::kFresh;
  if (!allow_hold) return ImuControlState::kUnavailable;

  const auto sample = imu->faultDetail(now);
  using hello_world::imu::ImuFaultKind;
  if (sample.reason == ImuFaultKind::kAccLow &&
      sample.last_rejected.kind == ImuFaultKind::kAccLow && sample.last_rejected.has_values &&
      now - sample.last_rejected.tick_ms <= kKeepLastImuMs &&
      sample.sample_age_ms <= kKeepLastImuMs && sample.clock_age_ms <= kKeepLastImuMs &&
      sample.gyro_age_ms <= kKeepLastImuMs && sample.accel_age_ms <= kKeepLastImuMs) {
    return ImuControlState::kHeldAfterAccLow;
  }
  return ImuControlState::kUnavailable;
}
}  // namespace robot

#endif  // ROBOT_IMU_CONTROL_STATE_HPP_
