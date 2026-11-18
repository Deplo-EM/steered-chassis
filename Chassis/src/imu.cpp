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
  // 每个FIFO帧独立积分，步长由陀螺仪ODR决定，不是TIM6周期。
  constexpr float kGyroOdrHz[] = {2000.0f, 2000.0f, 1000.0f, 400.0f,
                                   200.0f, 100.0f, 200.0f, 100.0f};
  const auto odr = static_cast<uint8_t>(config.gyro_odr_fbw);
  if (odr >= sizeof(kGyroOdrHz) / sizeof(kGyroOdrHz[0])) return false;
  gyro_sample_freq_ = kGyroOdrHz[odr];
  restartCalibration();
  bmi_.init(hw_config, rot_mat_flatten, config);

  /* imuInit 内部会做「读芯片 ID + 配置寄存器 + 可选自检」，成功返回 0。
   * 返回非0时按错误位排查供电、SPI、芯片ID和配置；只在核对原理图确认接反后修正片选。 */
  error_state_ = bmi_.imuInit(false);
  if (error_state_ != kBMI088ErrStateNoErr) {
    inited_ = false;
    return false;
  }

  restartCalibration();
  inited_ = true;
  return true;
}

void hello_world::imu::Imu::restartCalibration()
{
  calibrated_ = false;
  sample_valid_ = false;
  accel_cache_valid_ = false;
  published_sample_ = false;
  current_reject_ = ImuFaultKind::kNone;
  clock_seen_=accel_seen_=gyro_seen_=false;
  healthy_ = false;
  consecutive_bad_ = consecutive_good_ = 0;
  gyro_bias_removed_[0] = gyro_bias_removed_[1] = gyro_bias_removed_[2] = 0.0f;
  samples_ = 0;
  gyro_sum_[0] = gyro_sum_[1] = gyro_sum_[2] = 0.0f;
  gyro_bias_[0] = gyro_bias_[1] = gyro_bias_[2] = 0.0f;
  euler_[0] = euler_[1] = euler_[2] = 0.0f;
  ahrs_.init(gyro_sample_freq_, kImuMahonyKp, kImuMahonyKi);
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
    ++rej_transport_;
    RejectSample(ImuFaultKind::kSpi, hello_world::tick::GetTickMs());
    return;
  }
  const uint32_t now=hello_world::tick::GetTickMs();
  if (!clock_seen_) {
    clock_seen_=true; last_clock_=info.sensor_time; last_clock_tick_=now;
  } else {
    const uint32_t advance=(info.sensor_time-last_clock_)&0xFFFFFFu;
    if (advance>=0x800000u) {
      // 时钟回退/设备重启：原偏置与姿态不再可信，重新标定后再授权。
      ++rej_clock_;
      restartCalibration();
      RejectSample(ImuFaultKind::kClockBackward, now);
      return;
    }
    if (advance>0) { last_clock_=info.sensor_time; last_clock_tick_=now; }
  }
  if (info.gyro_discontinuity) {
    // 样本队列失去连续性，旧姿态不再可信，重新标定后才能恢复出力。
    restartCalibration();
    RejectSample(ImuFaultKind::kGyroFifo, now);
    return;
  }
  if (info.gyro_count != 0u) { gyro_seen_=true; last_gyro_tick_=now; }
  if (info.accel_ready) {
    accel_seen_ = true;
    last_accel_tick_ = now;
    // 坏加速度的判废独立于陀螺 FIFO，不能等到以后两者恰好同时出样。
    const auto reason = ValidateAccel(sample_acc);
    if (reason != ImuFaultKind::kNone) {
      accel_cache_valid_ = false;
      ++rej_sample_;
      const float no_gyro[3] = {invalid, invalid, invalid};
      RejectSample(reason, now, sample_acc,
                   info.gyro_count != 0u ? info.gyro_batch[0] : no_gyro, sample_temperature);
      return;
    }
    for (int i = 0; i < 3; ++i) accel_cache_[i] = sample_acc[i];
    accel_cache_valid_ = true;
  }
  if (!FreshnessOk(now)) {
    ++rej_fresh_;
    if (clock_seen_ && (now - last_clock_tick_) > kImuFreshnessMs) ++stale_clock_;
    if (accel_seen_ && (now - last_accel_tick_) > kImuFreshnessMs) ++stale_accel_;
    if (gyro_seen_ && (now - last_gyro_tick_) > kImuFreshnessMs) ++stale_gyro_;
    return;
  }
  // 空队列不积分；加速度判废后，必须等新的合格加速度才能恢复。
  if (info.gyro_count == 0u || !accel_cache_valid_) return;

  // 先校验整个批次，再按顺序消费；坏帧不能部分污染标定或姿态。
  if (!std::isfinite(sample_temperature)) {
    ++rej_sample_;
    RejectSample(ImuFaultKind::kTempInvalid, now, accel_cache_, info.gyro_batch[0], sample_temperature);
    return;
  }
  for (uint8_t i = 0; i < info.gyro_count; ++i) {
    if (!GyroValid(info.gyro_batch[i])) {
      ++rej_sample_;
      RejectSample(ImuFaultKind::kGyroInvalid, now, accel_cache_, info.gyro_batch[i], sample_temperature);
      return;
    }
  }
  for (uint8_t i = 0; i < info.gyro_count; ++i) {
    if (!ApplySample(accel_cache_, info.gyro_batch[i], sample_temperature, now)) break;
  }
}

bool hello_world::imu::Imu::ApplySample(const float sample_acc[3], const float sample_gyro[3],
                                      float sample_temperature, uint32_t now)
{
  sample_valid_ = true;
  published_sample_ = true;
  current_reject_ = ImuFaultKind::kNone;
  last_sample_tick_=now;
  for (int i = 0; i < 3; ++i) {
    acc_[i] = sample_acc[i];
    gyro_raw_[i] = sample_gyro[i];
  }
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
      ahrs_.init(gyro_sample_freq_, kImuMahonyKp, kImuMahonyKi);
    } else {
      /* 标定未完成：输出保持 0，绝不对外提供「看起来像真值」的角度 */
      gyro_bias_removed_[0] = gyro_bias_removed_[1] = gyro_bias_removed_[2] = 0.0f;
      return true;
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
      RecordRejected(ImuFaultKind::kAhrs, now, sample_acc, sample_gyro, sample_temperature);
      return false;
    }
  }
  for (int i = 0; i < 3; ++i) euler_[i] = candidate_euler[i];
  return true;
}

hello_world::imu::ImuFaultDetail hello_world::imu::Imu::faultDetail(uint32_t now) const
{
  ImuFaultDetail detail;
  detail.calibrated = calibrated_;
  detail.healthy = healthy_;
  detail.sample_valid = isSampleValidAt(now);
  if (clock_seen_) detail.clock_age_ms = now - last_clock_tick_;
  if (accel_seen_) detail.accel_age_ms = now - last_accel_tick_;
  if (gyro_seen_) detail.gyro_age_ms = now - last_gyro_tick_;
  if (published_sample_) detail.sample_age_ms = now - last_sample_tick_;
  detail.last_rejected = last_rejected_;

  // 明确的判废优先；成功后只看当拍的状态与新鲜度，历史坏帧不冒充新故障。
  if (!inited_) detail.reason = ImuFaultKind::kNotInitialized;
  else if (current_reject_ != ImuFaultKind::kNone) detail.reason = current_reject_;
  else if (clock_seen_ && detail.clock_age_ms > kImuFreshnessMs)
    detail.reason = ImuFaultKind::kClockStale;
  else if (accel_seen_ && detail.accel_age_ms > kImuFreshnessMs)
    detail.reason = ImuFaultKind::kAccStale;
  else if (gyro_seen_ && detail.gyro_age_ms > kImuFreshnessMs)
    detail.reason = ImuFaultKind::kGyroStale;
  else if (published_sample_ && detail.sample_age_ms > kImuFreshnessMs)
    detail.reason = ImuFaultKind::kSampleStale;
  else if (!calibrated_) detail.reason = ImuFaultKind::kNotCalibrated;
  else if (!healthy_) detail.reason = ImuFaultKind::kUnhealthy;
  return detail;
}

/* Private function definitions ----------------------------------------------*/

void hello_world::imu::Imu::RecordRejected(
    ImuFaultKind kind, uint32_t now, const float* acc, const float* gyro, float temperature)
{
  current_reject_ = kind;
  last_rejected_ = {};
  last_rejected_.kind = kind;
  last_rejected_.tick_ms = now;
  if (acc != nullptr && gyro != nullptr) {
    last_rejected_.has_values = true;
    for (int i = 0; i < 3; ++i) {
      last_rejected_.accel_mps2[i] = acc[i];
      last_rejected_.gyro_radps[i] = gyro[i];
    }
    last_rejected_.temperature_c = temperature;
  }
}

void hello_world::imu::Imu::RejectSample(
    ImuFaultKind kind, uint32_t now, const float* acc, const float* gyro, float temperature)
{
  sample_valid_=false;
  RecordRejected(kind, now, acc, gyro, temperature);
  ++bad_samples_total_;
  if (consecutive_bad_<kImuBadSampleLimit) ++consecutive_bad_;
  if (consecutive_bad_>=kImuBadSampleLimit) healthy_=false;
  consecutive_good_=0;
}

hello_world::imu::ImuFaultKind hello_world::imu::Imu::ValidateAccel(const float acc[3])
{
  // 保持原判据；量级判据无法替代传输和数据新鲜度检查。
  constexpr float kAccNormMin = 2.0f;
  constexpr float kAccNormMax = 40.0f;

  for (int i = 0; i < 3; ++i) {
    if (!std::isfinite(acc[i])) return ImuFaultKind::kAccInvalid;
  }

  const float acc_norm = std::sqrt(acc[0] * acc[0] + acc[1] * acc[1] +
                                   acc[2] * acc[2]);
  if (acc_norm < kAccNormMin) return ImuFaultKind::kAccLow;
  if (acc_norm > kAccNormMax) return ImuFaultKind::kAccHigh;
  return ImuFaultKind::kNone;
}

bool hello_world::imu::Imu::GyroValid(const float gyro[3])
{
  constexpr float kGyroAbsMax = 50.0f;
  for (int i = 0; i < 3; ++i) {
    if (!std::isfinite(gyro[i]) || std::fabs(gyro[i]) > kGyroAbsMax) return false;
  }
  return true;
}
