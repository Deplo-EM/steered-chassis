#pragma once
// 仅替代物理 IMU；状态机、通信任务、PID 和运动模块均编译生产源码。
namespace hello_world::imu {
class Imu {
 public:
  bool healthy = true;
  bool sample_valid = true;
  float yaw_value = 0.0f, gyro_z = 0.0f;
  void update() {}
  bool isCalibrated() const { return true; }
  bool isHealthy() const { return healthy; }
  bool isSampleValid() const { return sample_valid; }
  float yaw() const { return yaw_value; }
  float gyroZ() const { return gyro_z; }
};
}
