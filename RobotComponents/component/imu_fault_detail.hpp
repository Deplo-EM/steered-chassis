#pragma once
#include <cstdint>

namespace hello_world::imu {

/** 底盘停机取证使用。取值稳定，供 Ozone 十进制显示和文档对照。 */
enum class ImuFaultKind : uint8_t {
  kNone = 0,
  kSpi = 1,
  kClockBackward = 2,
  kGyroFifo = 3,
  kAccLow = 4,
  kAccHigh = 5,
  kAccInvalid = 6,
  kGyroInvalid = 7,
  kTempInvalid = 8,
  kAhrs = 9,
  kClockStale = 10,
  kAccStale = 11,
  kGyroStale = 12,
  kSampleStale = 13,
  kNotCalibrated = 14,
  kUnhealthy = 15,
  kNotInitialized = 16,
};

/** 被拒绝的数据。数值拒绝时保存失败批次中的实际三轴读数，而非上次通过的缓存。 */
struct ImuRejectedSample {
  ImuFaultKind kind = ImuFaultKind::kNone;
  bool has_values = false;  ///< 传输/时钟/FIFO 故障可能没有完整三轴值。
  uint32_t tick_ms = 0;  ///< 这份拒绝样本发生的本板毫秒时刻。
  float accel_mps2[3] = {};  ///< 原始候选加速度，m/s²；可能含导致判废的非有限值。
  float gyro_radps[3] = {};  ///< 同批候选角速度，rad/s。
  float temperature_c = 0.0f;
};

/**
 * @brief 调用时刻的可用状态，以及保留下来的最近坏样本。
 * @note  reason 描述当前不可用原因；last_rejected 可在恢复后保留历史，
 *        判断是否同一次故障时需同时对照其中的时间戳。
 */
struct ImuFaultDetail {
  ImuFaultKind reason = ImuFaultKind::kNone;
  bool calibrated = false;
  bool healthy = false;
  bool sample_valid = false;  ///< 已包含 20 ms 新鲜度判定。
  // UINT32_MAX 表示本次初始化或重新标定后还未观察到对应数据。
  uint32_t clock_age_ms = UINT32_MAX;  ///< 芯片时钟上次前进距今多久。
  uint32_t accel_age_ms = UINT32_MAX;  ///< 加速度硬件上次报告新数据距今多久。
  uint32_t gyro_age_ms = UINT32_MAX;  ///< 陀螺 FIFO 上次给出新样本距今多久。
  uint32_t sample_age_ms = UINT32_MAX;  ///< 最后一份通过处理的样本距今多久。
  ImuRejectedSample last_rejected;
};

}  // namespace hello_world::imu
