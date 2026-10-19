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

/* Private constants ---------------------------------------------------------*/
/* Exported function definitions ---------------------------------------------*/

bool hello_world::imu::Imu::init(const BMI088HWConfig& hw_config,
                                 const float rot_mat_flatten[9],
                                 const BMI088Config& config)
{
  bmi_.init(hw_config, rot_mat_flatten, config);

  /* imuInit 内部会做「读芯片 ID + 配置寄存器 + 可选自检」，成功返回 0。
   * 返回非 0 时最常见的原因是片选引脚接反了：把 acc_cs / gyro_cs 对调即可。 */
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
  samples_ = 0;
  gyro_sum_[0] = gyro_sum_[1] = gyro_sum_[2] = 0.0f;
  gyro_bias_[0] = gyro_bias_[1] = gyro_bias_[2] = 0.0f;
  euler_[0] = euler_[1] = euler_[2] = 0.0f;
  ahrs_.init(kImuSampleFreq, kImuMahonyKp, kImuMahonyKi);
}

void hello_world::imu::Imu::update()
{
  if (!inited_) {
    return;
  }

  /* ---------- 1. 读芯片 ---------- */
  bmi_.getData(acc_, gyro_bias_removed_, &temperature_);

  /* ---------- 2. 零漂标定 ---------- */
  if (!calibrated_) {
    /* 先判断「是否静止」：与当前均值偏差过大 → 车在动，重新采样。
     * 这样即使上电时有人碰了车，也会自动重来，而不是把运动数据当成偏置。 */
    if (samples_ > 100u) {
      const float inv = 1.0f / static_cast<float>(samples_);
      for (int i = 0; i < 3; ++i) {
        const float mean = gyro_sum_[i] * inv;
        if (std::fabs(gyro_bias_removed_[i] - mean) > kImuCalibMotionThres) {
          samples_ = 0;
          gyro_sum_[0] = gyro_sum_[1] = gyro_sum_[2] = 0.0f;
          break;
        }
      }
    }

    gyro_sum_[0] += gyro_bias_removed_[0];
    gyro_sum_[1] += gyro_bias_removed_[1];
    gyro_sum_[2] += gyro_bias_removed_[2];
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
  gyro_bias_removed_[0] -= gyro_bias_[0];
  gyro_bias_removed_[1] -= gyro_bias_[1];
  gyro_bias_removed_[2] -= gyro_bias_[2];

  /* ---------- 4. 姿态解算 ---------- */
  ahrs_.update(acc_, gyro_bias_removed_);
  ahrs_.getEulerAngle(euler_);
}

/* Private function definitions ----------------------------------------------*/
