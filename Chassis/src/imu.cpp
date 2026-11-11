/**
 *******************************************************************************
 * @file      : imu.cpp
 * @brief     : IMU 封装实现
 * @history   :
 *  Version     Date            Author          Note
 *  V1.0.0      2026-10-19      <author>        1. 完成 Imu 封装
 *******************************************************************************
 * @attention :
 *  本文件与 Gimbal/src/imu.cpp 内容完全一致（两块板各编一份）。
 *  修改时请两边同步——这是骨架工程「每块板各自一份组件」的结构决定的。
 *******************************************************************************
 */
/* Includes ------------------------------------------------------------------*/
#include "imu.hpp"

#include <cmath>
#include <limits>

/* Private constants ---------------------------------------------------------*/
/* Exported function definitions ---------------------------------------------*/

bool hello_world::imu::Imu::init(const BMI088HWConfig& hw_config,
                                 const float rot_mat_flatten[9],
                                 const BMI088Config& config)
{
  inited_ = false;
  restartCalibration();
  bmi_.init(hw_config, rot_mat_flatten, config);

  /* imuInit 内部会做「读芯片 ID + 配置寄存器 + 可选自检」，成功返回 0。
   * 返回非0时按错误位排查供电、SPI、芯片ID和配置；只在核对原理图确认接反后修正片选。 */
  error_state_ = bmi_.imuInit(false);
  if (error_state_ != kBMI088ErrStateNoErr) {
    inited_ = false;
    return false;
  }

  ahrs_.init(kImuSampleFreq, kImuMahonyKp, kImuMahonyKi);
  restartCalibration();
  inited_ = true;
  return true;
}

void hello_world::imu::Imu::restartCalibration()
{
  calibrated_ = false;
  sample_valid_ = false;
  clock_seen_=accel_seen_=gyro_seen_=false;
  healthy_ = false;
  consecutive_bad_ = consecutive_good_ = 0;
  gyro_bias_removed_[0] = gyro_bias_removed_[1] = gyro_bias_removed_[2] = 0.0f;
  samples_ = 0;
  gyro_sum_[0] = gyro_sum_[1] = gyro_sum_[2] = 0.0f;
  gyro_bias_[0] = gyro_bias_[1] = gyro_bias_[2] = 0.0f;
  euler_[0] = euler_[1] = euler_[2] = 0.0f;
  ahrs_.init(kImuSampleFreq, kImuMahonyKp, kImuMahonyKi);
}

void hello_world::imu::Imu::update()
{
  if (!inited_) {
    sample_valid_=false;
    return;
  }

  /* ---------- 1. 读芯片 ---------- */
  // 候选样本通过整帧校验后才能发布；坏样本不污染控制可见的上一帧数据。
  constexpr float invalid = std::numeric_limits<float>::quiet_NaN();
  float sample_acc[3] = {invalid, invalid, invalid};
  float sample_gyro[3] = {invalid, invalid, invalid};
  float sample_temperature = invalid;
  BMI088SampleInfo info;
  if (!bmi_.getData(sample_acc, sample_gyro, &sample_temperature, &info)) {
    RejectSample(); return;
  }
  const uint32_t now=hello_world::tick::GetTickMs();
  if (!clock_seen_) {
    clock_seen_=true; last_clock_=info.sensor_time; last_clock_tick_=now;
  } else {
    const uint32_t advance=(info.sensor_time-last_clock_)&0xFFFFFFu;
    if (advance>=0x800000u) {
      // 时钟回退/设备重启：原偏置与姿态不再可信，重新标定后再授权。
      restartCalibration(); RejectSample(); return;
    }
    if (advance>0) { last_clock_=info.sensor_time; last_clock_tick_=now; }
  }
  if (info.accel_ready) { accel_seen_=true; last_accel_tick_=now; }
  if (info.gyro_ready) { gyro_seen_=true; last_gyro_tick_=now; }
  if (!FreshnessOk(now)) { RejectSample(); return; }
  // 轮询相位导致暂时没有新陀螺样本时，保留有时限的上一有效状态，不重复积分/标定。
  if (!info.gyro_ready) return;

  /* ---------- 1.5 运行期健康检查（审计 M3 + F3） ----------
   * 初始化成功不代表运行中一直正常。SPI 接触不良时读回来的可能是全 0、
   * 全 0xFF、NaN，或者短暂的乱码。
   *
   * ★ 关键：坏样本**根本不进入**标定和 Mahony。如果先喂给滤波器再判"不健康"，
   *   一次坏数据就会把姿态拉歪，恢复起来要好几百毫秒（甚至永久偏掉）。 */
  if (!IsSampleOk(sample_acc, sample_gyro, sample_temperature)) {
    RejectSample(); return;
  }

  sample_valid_ = true;
  last_sample_tick_=now;
  for (int i = 0; i < 3; ++i) acc_[i] = sample_acc[i];
  temperature_ = sample_temperature;

  /* 好样本：要连续好够久才恢复健康，避免"一次正常采样就痊愈"（审计 F3） */
  consecutive_bad_ = 0;
  if (consecutive_good_ < kImuGoodSampleLimit) {
    ++consecutive_good_;
  }
  if (consecutive_good_ >= kImuGoodSampleLimit) {
    healthy_ = true;
  }

  /* ---------- 2. 零漂标定 ---------- */
  if (!calibrated_) {
    /* 先判断「是否静止」：与当前均值偏差过大 → 车在动，重新采样。
     * 这样即使上电时有人碰了车，也会自动重来，而不是把运动数据当成偏置。 */
    if (samples_ > 100u) {
      const float inv = 1.0f / static_cast<float>(samples_);
      for (int i = 0; i < 3; ++i) {
        const float mean = gyro_sum_[i] * inv;
        if (std::fabs(sample_gyro[i] - mean) > kImuCalibMotionThres) {
          samples_ = 0;
          gyro_sum_[0] = gyro_sum_[1] = gyro_sum_[2] = 0.0f;
          break;
        }
      }
    }

    gyro_sum_[0] += sample_gyro[0];
    gyro_sum_[1] += sample_gyro[1];
    gyro_sum_[2] += sample_gyro[2];
    ++samples_;

    if (samples_ >= kImuCalibSamples) {
      const float inv = 1.0f / static_cast<float>(samples_);
      gyro_bias_[0] = gyro_sum_[0] * inv;
      gyro_bias_[1] = gyro_sum_[1] * inv;
      gyro_bias_[2] = gyro_sum_[2] * inv;
      calibrated_ = true;
      /* 标定完成这一刻就是姿态零点，Mahony 从单位四元数重新开始，
       * 加速度计会在随后几百毫秒内把 roll / pitch 收敛到正确值。 */
      ahrs_.init(kImuSampleFreq, kImuMahonyKp, kImuMahonyKi);
    } else {
      /* 标定未完成：输出保持 0，绝不对外提供「看起来像真值」的角度 */
      gyro_bias_removed_[0] = gyro_bias_removed_[1] = gyro_bias_removed_[2] = 0.0f;
      return;
    }
  }

  /* ---------- 3. 扣零漂 ---------- */
  gyro_bias_removed_[0] = sample_gyro[0] - gyro_bias_[0];
  gyro_bias_removed_[1] = sample_gyro[1] - gyro_bias_[1];
  gyro_bias_removed_[2] = sample_gyro[2] - gyro_bias_[2];

  /* ---------- 4. 姿态解算 ---------- */
  ahrs_.update(acc_, gyro_bias_removed_);
  float candidate_euler[3];
  ahrs_.getEulerAngle(candidate_euler);
  for (float angle : candidate_euler) {
    if (!std::isfinite(angle)) {
      ++bad_samples_total_;
      restartCalibration();  // 解算状态异常需重建，不能把坏姿态交给控制层。
      return;
    }
  }
  for (int i = 0; i < 3; ++i) euler_[i] = candidate_euler[i];
}

/* Private function definitions ----------------------------------------------*/

void hello_world::imu::Imu::RejectSample()
{
  sample_valid_=false;
  ++bad_samples_total_;
  if (consecutive_bad_<kImuBadSampleLimit) ++consecutive_bad_;
  if (consecutive_bad_>=kImuBadSampleLimit) healthy_=false;
  consecutive_good_=0;
}

bool hello_world::imu::Imu::IsSampleOk(
    const float acc[3], const float gyro[3], float temperature) const
{
  /* 判据（全部满足才认为这一个采样可信）：
   *   1. 所有采样都是有限数（不是 NaN / Inf）；
   *   2. 加速度模长在 [2, 40] m/s²：静止时约 9.79，剧烈运动也落在这个区间；
   *      读回全 0 或全 0xFF 时会明显落在区间外；
   *   3. 角速度绝对值 < 50 rad/s：任何正常的车都到不了这个量级。
   *
   * 传输状态、设备时钟和DRDY新鲜度在本函数前单独检查，不能用量级校验替代。
   * 设备内部若持续伪造合理数值且时钟/DRDY仍正常，仍需独立硬件证据识别。 */
  constexpr float kAccNormMin = 2.0f;
  constexpr float kAccNormMax = 40.0f;
  constexpr float kGyroAbsMax = 50.0f;

  if (!std::isfinite(temperature)) return false;

  for (int i = 0; i < 3; ++i) {
    if (!std::isfinite(acc[i]) || !std::isfinite(gyro[i])) {
      return false;
    }
  }

  const float acc_norm = std::sqrt(acc[0] * acc[0] + acc[1] * acc[1] +
                                   acc[2] * acc[2]);
  if (acc_norm < kAccNormMin || acc_norm > kAccNormMax) {
    return false;
  }

  for (int i = 0; i < 3; ++i) {
    if (std::fabs(gyro[i]) > kGyroAbsMax) {
      return false;
    }
  }

  return true;
}
