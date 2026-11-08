// 故障回归：编译真实 Imu，仅替代 BMI088 读取和 Mahony 依赖。
#include <cmath>
#include <cstdio>
#include <limits>
#include "imu.hpp"

namespace {
int checks = 0, failures = 0;
#define CHECK(expr) do { ++checks; if (!(expr)) { ++failures; std::printf("FAIL line %d: %s\n", __LINE__, #expr); } } while (0)
bool Near(float actual, float expected) { return std::isfinite(actual) && std::fabs(actual - expected) < 0.0001f; }

// 单帧有效性与长期健康迟滞独立验证。
bool SampleValid(const hello_world::imu::Imu& imu) { return imu.isSampleValid(); }

void GoodSample() {
  imu_test::acc[0] = imu_test::acc[1] = 0.0f; imu_test::acc[2] = 9.8f;
  imu_test::gyro[0] = 0.1f; imu_test::gyro[1] = -0.2f; imu_test::gyro[2] = 0.3f;
  imu_test::temperature = 25.0f; imu_test::init_ok = true;
}

void Calibrate(hello_world::imu::Imu& imu) {
  GoodSample(); CHECK(imu.init({}));
  for (int i = 0; i < hello_world::imu::kImuCalibSamples; ++i) imu.update();
  CHECK(imu.isCalibrated()); CHECK(imu.isHealthy()); CHECK(Near(imu.gyroZ(), 0.0f));
}
}

int main() {
  using hello_world::imu::Imu;
  Imu imu;
  CHECK(!SampleValid(imu));
  Calibrate(imu);
  imu_test::gyro[2] = 0.8f; imu.update();
  CHECK(Near(imu.gyroZ(), 0.5f));
  const float last_yaw = imu.yaw();
  const int ahrs_before = imu_test::ahrs_updates;
  imu_test::gyro[2] = std::numeric_limits<float>::quiet_NaN();
  imu_test::acc[0] = 1.0f; imu_test::temperature = 30.0f;
  imu.update();
  CHECK(!SampleValid(imu)); CHECK(imu.isHealthy());
  CHECK(Near(imu.gyroZ(), 0.5f)); CHECK(Near(imu.accX(), 0.0f));
  CHECK(Near(imu.temperature(), 25.0f)); CHECK(Near(imu.yaw(), last_yaw));
  CHECK(imu_test::ahrs_updates == ahrs_before); CHECK(imu.badSamples() == 1);

  // Inf、超范围和温度非有限：整个采样不能部分发布。
  GoodSample(); imu_test::acc[2] = std::numeric_limits<float>::infinity(); imu.update();
  CHECK(!SampleValid(imu)); CHECK(Near(imu.accZ(), 9.8f)); CHECK(Near(imu.gyroZ(), 0.5f));
  GoodSample(); imu_test::gyro[1] = 51.0f; imu.update();
  CHECK(!SampleValid(imu)); CHECK(Near(imu.gyroY(), 0.0f));
  GoodSample(); imu_test::temperature = std::numeric_limits<float>::quiet_NaN(); imu.update();
  CHECK(!SampleValid(imu)); CHECK(Near(imu.temperature(), 25.0f));
  CHECK(imu_test::ahrs_updates == ahrs_before);

  // 50 个连续坏样本触发长期失效，99 个好样本不能恢复，第100个才可以。
  GoodSample(); imu.update();
  imu_test::acc[2] = 0.0f;
  for (int i = 0; i < 49; ++i) imu.update();
  CHECK(imu.isHealthy()); CHECK(!SampleValid(imu));
  imu.update(); CHECK(!imu.isHealthy());
  GoodSample();
  for (int i = 0; i < 99; ++i) imu.update();
  CHECK(!imu.isHealthy()); CHECK(SampleValid(imu));
  imu.update(); CHECK(imu.isHealthy()); CHECK(Near(imu.gyroZ(), 0.0f));

  // 标定只累计有效样本；遇到坏帧不会对外泄漏未去bias的原始角速度。
  Imu calibrating; GoodSample(); CHECK(calibrating.init({}));
  for (int i = 0; i < 1000; ++i) calibrating.update();
  const float progress = calibrating.calibProgress();
  imu_test::gyro[0] = std::numeric_limits<float>::infinity(); calibrating.update();
  CHECK(!SampleValid(calibrating)); CHECK(Near(calibrating.gyroX(), 0.0f));
  CHECK(Near(calibrating.calibProgress(), progress)); CHECK(!calibrating.isCalibrated());
  GoodSample();
  for (int i = 0; i < 999; ++i) calibrating.update();
  CHECK(!calibrating.isCalibrated()); calibrating.update(); CHECK(calibrating.isCalibrated());
  CHECK(Near(calibrating.gyroX(), 0.0f));

  // 再标定立即撤销原有效资格，重新初始化失败也不能保留旧的已标定状态。
  imu.restartCalibration();
  CHECK(!imu.isCalibrated()); CHECK(!SampleValid(imu)); CHECK(!imu.isHealthy());
  Calibrate(imu); imu_test::init_ok = false; CHECK(!imu.init({}));
  CHECK(!imu.isCalibrated()); CHECK(!SampleValid(imu)); CHECK(!imu.isHealthy());
  const float yaw_before = imu.yaw(); imu.update(); CHECK(Near(imu.yaw(), yaw_before));
  std::printf("IMU: %d checks, %d failures\n", checks, failures);
  return failures ? 1 : 0;
}
