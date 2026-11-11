/**
 *******************************************************************************
 * @file      : imu.hpp
 * @brief     : IMU 设备封装（BMI088 原始读取 + 零漂标定 + Mahony 姿态解算）
 * @history   :
 *  Version     Date            Author          Note
 *  V1.0.0      2026-10-19      <author>        1. 完成 Imu 封装类
 *******************************************************************************
 * @attention :
 *  【这一层在干什么】
 *  BMI088 只会吐出「三轴加速度 + 三轴角速度」的原始数字，Mahony 只会把
 *  「角速度 + 加速度」积分成姿态。中间还差三件事，都由本类完成：
 *    1. 开机零漂标定：陀螺仪静止时输出不是精确的 0（几十 mrad/s 的偏置），
 *       不扣掉的话积分 10 秒就能漂出好几度。做法是静止采样求平均值当偏置。
 *    2. 坐标系换算：把传感器安装方向折算成「x 前、y 左、z 上」。
 *    3. 姿态角输出：roll / pitch / yaw（rad，相对标定完成那一刻）。
 *
 *  【为什么零漂没收敛前不许进入 Working】
 *  未标定完就控制，等于拿一个带偏置的陀螺去算 δ 和跟随误差，车会慢慢「跑偏」。
 *  更重要的是，标定期间电机不能动（一动就采不到静止数据），所以状态机必须
 *  在标定完成前把所有电机输出按在 0 上（考核说明「注意事项」第 4 条）。
 *
 *  【调用方式】
 *    Imu imu;
 *    imu.init(硬件配置, 安装矩阵);   // 只调用一次，内部会阻塞几百毫秒做芯片自检
 *    // 之后每 1ms 调用一次：
 *    imu.update();
 *    if (imu.isCalibrated()) { ... imu.yaw() ... }
 *******************************************************************************
 *  Copyright (c) 2025 Hello World Team, Zhejiang University.
 *  All Rights Reserved.
 *******************************************************************************
 */
/* Define to prevent recursive inclusion -------------------------------------*/
#ifndef HW_COMPONENTS_DEVICES_IMU_IMU_HPP_
#define HW_COMPONENTS_DEVICES_IMU_IMU_HPP_

/* Includes ------------------------------------------------------------------*/
#include "BMI088.hpp"
#include "mahony.hpp"
#include "tick.hpp"

namespace hello_world {
namespace imu {

/* Exported constants --------------------------------------------------------*/

/** 零漂标定采样数：1kHz 采样 2000 次 = 2 秒。
 *  时间太短偏置估计不准；太长则上电等待久。2 秒是常用折中。 */
constexpr uint16_t kImuCalibSamples = 2000u;

/** 标定期间允许的「静止」阈值，单位 rad/s。
 *  若采样值与当前均值之差超过它，说明车被人碰了 / 在动，重新开始采样。 */
constexpr float kImuCalibMotionThres = 0.5f;

/** 姿态解算采样频率，单位 Hz（必须与控制周期一致） */
constexpr float kImuSampleFreq = 1000.0f;

/** Mahony 增益（经验值：kp 决定收敛快慢，ki 补偿残余陀螺漂移） */
constexpr float kImuMahonyKp = 1.0f;
constexpr float kImuMahonyKi = 0.01f;

/** 连续多少个异常采样判定为「不健康」（1kHz → 50ms） */
constexpr uint16_t kImuBadSampleLimit = 50u;

/** 恢复健康需要连续多少个正常采样（1kHz → 100ms）。
 *  要求"连续好够久"才恢复，避免一次正常采样就宣布痊愈（审计 F3）。 */
constexpr uint16_t kImuGoodSampleLimit = 100u;
/** 新样本/硬件时钟最大允许停滞时间(ms)，兼容1kHz轮询相位抖动。 */
constexpr uint32_t kImuFreshnessMs = 20u;

/** 默认安装矩阵：传感器坐标 = 车体坐标（x 前、y 左、z 上）。
 *  若 IMU 是倒装 / 转 90° 装的，改这个矩阵即可，不用动解算代码。 */
constexpr float kImuDefaultRotMat[9] = {1.0f, 0.0f, 0.0f,
                                        0.0f, 1.0f, 0.0f,
                                        0.0f, 0.0f, 1.0f};

/* Exported types ------------------------------------------------------------*/

/**
 * @brief IMU 封装类
 */
class Imu {
 public:
  Imu() = default;
  ~Imu() = default;

  /**
   * @brief 初始化：配置芯片、设定安装方向、启动零漂标定
   * @param hw_config        SPI 句柄与两个片选引脚
   * @param rot_mat_flatten  安装旋转矩阵（展平 3x3），把「传感器坐标」转成
   *                         「车体坐标（x 前 y 左 z 上）」。默认单位矩阵。
   * @param config           量程 / 输出频率等芯片配置
   * @retval true 芯片初始化成功；false 芯片未找到或自检失败
   * @note  内部有阻塞（芯片自检、几百毫秒），只能在 main 的初始化阶段调用，
   *        绝对不能放进中断。
   */
  bool init(const BMI088HWConfig& hw_config,
            const float rot_mat_flatten[9] = kImuDefaultRotMat,
            const BMI088Config& config = BMI088Config());

  /**
   * @brief 周期更新：读芯片 → （标定中则累计偏置）→ Mahony 解算
   * @note  1kHz 调用一次。标定未完成时只累计、不解算。
   */
  void update();

  /**
   * @brief 强制重新开始零漂标定
   * @note  只在车静止时调用，否则会采到运动数据。
   */
  void restartCalibration();

  /* ---------------- 数据查询 ---------------- */

  /** 零漂标定是否完成（完成前不得进入 Working） */
  bool isCalibrated() const { return calibrated_; }

  /** 标定进度 0~1（调试用） */
  float calibProgress() const
  {
    return static_cast<float>(samples_) / static_cast<float>(kImuCalibSamples);
  }

  /** 姿态角，单位 rad，相对「标定完成那一刻」的姿态 */
  float roll() const { return euler_[0]; }
  float pitch() const { return euler_[1]; }
  float yaw() const { return euler_[2]; }

  /** 去除零漂后的角速度，单位 rad/s（车体系） */
  float gyroX() const { return gyro_bias_removed_[0]; }
  float gyroY() const { return gyro_bias_removed_[1]; }
  float gyroZ() const { return gyro_bias_removed_[2]; }

  /** 加速度，单位 m/s²（车体系，含重力） */
  float accX() const { return acc_[0]; }
  float accY() const { return acc_[1]; }
  float accZ() const { return acc_[2]; }

  /** 温度，单位 ℃ */
  float temperature() const { return temperature_; }

  /** 芯片初始化错误码（BMI088ErrState 的或运算结果），0 表示正常 */
  uint8_t errorState() const { return static_cast<uint8_t>(error_state_); }

  /**
   * @brief 运行期数据是否合理
   * @note  初始化成功 ≠ 运行期一直正常：SPI 接触不良、芯片被干扰时，
   *        读回来的是一堆无意义的值（全 0、全 0xFF、NaN）。
   *        判据：所有采样都是有限数、加速度模长在 [2, 40] m/s²
   *        （静止时约 9.79，剧烈运动也在范围内）、角速度绝对值 < 50 rad/s。
   *        单帧异常立即撤销 isSampleValid；连续 50 坏 / 100 好控制长期健康迟滞。
   */
  bool isHealthy() const { return healthy_; }

  /** 本次采样可用于控制；单帧异常立即 false，不等待长期健康迟滞。 */
  bool isSampleValid() const {
    const uint32_t now=hello_world::tick::GetTickMs();
    return sample_valid_ && FreshnessOk(now) && (now-last_sample_tick_)<=kImuFreshnessMs;
  }

  /** 累计异常采样数（调试用） */
  uint32_t badSamples() const { return bad_samples_total_; }

 private:
  /** 本周期采样是否可信（有限值、加速度/角速度量级正常） */
  bool IsSampleOk(const float acc[3], const float gyro[3], float temperature) const;

  void RejectSample();
  bool FreshnessOk(uint32_t now) const {
    return clock_seen_ && accel_seen_ && gyro_seen_ &&
      (now-last_clock_tick_)<=kImuFreshnessMs &&
      (now-last_accel_tick_)<=kImuFreshnessMs && (now-last_gyro_tick_)<=kImuFreshnessMs;
  }
  bool clock_seen_=false, accel_seen_=false, gyro_seen_=false;
  uint32_t last_clock_=0, last_clock_tick_=0, last_accel_tick_=0, last_gyro_tick_=0, last_sample_tick_=0;
  BMI088 bmi_;                ///< 芯片驱动
  ahrs::Mahony ahrs_;         ///< 姿态解算
  BMI088ErrState error_state_ = kBMI088ErrStateNoErr;

  bool inited_ = false;
  bool calibrated_ = false;

  /* 运行期健康状态 */
  bool sample_valid_ = false;
  bool healthy_ = false;          ///< 上电后要先采到合理数据才为 true
  uint16_t consecutive_bad_ = 0;  ///< 连续异常采样数
  uint16_t consecutive_good_ = 0; ///< 连续正常采样数（用于恢复判定）
  uint32_t bad_samples_total_ = 0;

  /* 零漂标定累加器 */
  uint16_t samples_ = 0;
  float gyro_sum_[3] = {0.0f, 0.0f, 0.0f};
  float gyro_bias_[3] = {0.0f, 0.0f, 0.0f};

  /* 最新数据 */
  float acc_[3] = {0.0f, 0.0f, 0.0f};
  float gyro_bias_removed_[3] = {0.0f, 0.0f, 0.0f};
  float euler_[3] = {0.0f, 0.0f, 0.0f};
  float temperature_ = 0.0f;
};

}  // namespace imu
}  // namespace hello_world

#endif /* HW_COMPONENTS_DEVICES_IMU_IMU_HPP_ */
