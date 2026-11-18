#pragma once
#include "imu_fault_detail.hpp"

// 仅替代物理 IMU；状态机、通信任务、PID 和运动模块均编译生产源码。
namespace hello_world::imu {
class Imu {
 public:
  bool healthy = true;
  bool sample_valid = true;
  ImuRejectedSample rejected;
  uint32_t sample_age_ms = UINT32_MAX;
  float yaw_value = 0.0f, roll_value = 0.0f, pitch_value = 0.0f;
  float gyro_y = 0.0f, gyro_z = 0.0f;
  void update() {}
  bool isCalibrated() const { return true; }
  bool isHealthy() const { return healthy; }
  bool isSampleValid() const { return sample_valid; }
  bool isSampleValidAt(uint32_t) const { return sample_valid; }
  ImuFaultDetail faultDetail(uint32_t) const {
    ImuFaultDetail detail;
    detail.reason = isSampleValid() ? ImuFaultKind::kNone : rejected.kind;
    detail.calibrated = isCalibrated();
    detail.healthy = isHealthy();
    detail.sample_valid = isSampleValid();
    detail.clock_age_ms = detail.accel_age_ms = detail.gyro_age_ms = sample_age_ms;
    detail.sample_age_ms = sample_age_ms;
    detail.last_rejected = rejected;
    return detail;
  }
  float yaw() const { return yaw_value; }
  float roll() const { return roll_value; }
  float pitch() const { return pitch_value; }
  float gyroY() const { return gyro_y; }
  float gyroZ() const { return gyro_z; }
};
}
