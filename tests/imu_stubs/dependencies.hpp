#pragma once
// 只替代芯片访问和姿态解算依赖，imu.hpp / imu.cpp 使用每块板的实际源码。
// 提前声明相同 include guard，避免测试依赖 STM32 HAL / CMSIS。
#define HW_COMPONENTS_DEVICES_IMU_BMI088_HPP_
#define HW_COMPONENTS_ALGORITHMS_AHRS_MAHONY_HPP_
#include <cstdint>
#include <cstring>
namespace imu_test {
inline float acc[3] = {0.0f, 0.0f, 9.8f};
inline float gyro[3] = {0.1f, -0.2f, 0.3f};
inline float temperature = 25.0f;
inline bool init_ok = true;
inline int ahrs_updates = 0;
inline uint32_t now=0, clock=100;
}
namespace hello_world::tick {
inline uint32_t GetTickMs() { return imu_test::now; }
}
namespace hello_world::imu {
struct BMI088HWConfig {};
struct BMI088Config { uint8_t gyro_odr_fbw=2; };
enum BMI088ErrState : uint8_t { kBMI088ErrStateNoErr = 0, kBMI088ErrStateAccNotFound = 1 };
struct BMI088SampleInfo { uint32_t sensor_time=0; bool accel_ready=false; uint8_t gyro_count=0; bool gyro_discontinuity=false; float gyro_batch[4][3]={}; };
class BMI088 {
 public:
  void init(const BMI088HWConfig&, const float[9], const BMI088Config&) {}
  BMI088ErrState imuInit(bool) { return imu_test::init_ok ? kBMI088ErrStateNoErr : kBMI088ErrStateAccNotFound; }
  bool getData(float acc[3], float gyro[3], float* temperature, BMI088SampleInfo* info=nullptr) const {
    ++imu_test::now; imu_test::clock=(imu_test::clock+26)&0xFFFFFFu;
    if(info) { *info={};info->sensor_time=imu_test::clock;info->accel_ready=true;info->gyro_count=1;std::memcpy(info->gyro_batch[0],imu_test::gyro,sizeof(imu_test::gyro)); }
    std::memcpy(acc, imu_test::acc, sizeof(imu_test::acc));
    std::memcpy(gyro, imu_test::gyro, sizeof(imu_test::gyro));
    *temperature = imu_test::temperature;
    return true;
  }
};
}
namespace hello_world::ahrs {
class Mahony {
 public:
  void init(float, float, float) { yaw_ = 0.0f; }
  void update(const float[3], const float gyro[3]) { ++imu_test::ahrs_updates; yaw_ += gyro[2] * 0.001f; }
  void getEulerAngle(float euler[3]) const { euler[0] = euler[1] = 0.0f; euler[2] = yaw_; }
 private:
  float yaw_ = 0.0f;
};
}
