/**
 * @file imu_control_state.hpp
 * @brief 控制层如何使用 IMU 最新的合格结果；不改传感器判废或融合逻辑。
 */
#ifndef ROBOT_IMU_CONTROL_STATE_HPP_
#define ROBOT_IMU_CONTROL_STATE_HPP_

#include <cstdint>

#include "imu_fault_detail.hpp"

namespace robot {

/**
 * @brief 区分最新有效解、短时保留的旧解，以及不能用于控制的状态。
 * @note  kHeldAfterAccLow 仅允许复用上一份合格姿态；云台会冻结 yaw 目标积分，
 *        启动与故障恢复仍要经过标定和有效样本检查。
 */
enum class ImuControlState : uint8_t { kUnavailable, kFresh, kHeldAfterAccLow };
inline constexpr uint32_t kKeepLastImuMs = 8u;  ///< 保留窗口，单位 ms。

/** 只有已授权时允许短暂保留上一个解算值。时钟、传输等故障仍须立即撤销授权。 */
template <class Imu>
ImuControlState EvaluateImuForControl(const Imu* imu, uint32_t now, bool allow_hold)
{
  if (imu == nullptr || !imu->isCalibrated() || !imu->isHealthy()) return ImuControlState::kUnavailable;
  if (imu->isSampleValidAt(now)) return ImuControlState::kFresh;
  if (!allow_hold) return ImuControlState::kUnavailable;

  // 当前原因和最近拒绝样本必须共同指向低加速度，同时检查最后好样本
  // 以及时钟、加速度、陀螺三条通路的年龄，避免把传输断流当作短时扰动。
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
