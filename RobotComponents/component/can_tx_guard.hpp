/**
 * @file can_tx_guard.hpp
 * @brief 关键 CAN 帧提交结果的故障锁存。两板共用，无 HAL 依赖。
 * @note success 仅表示编码成功且 HAL 接受入邮箱，不代表电机收到或执行。
 *       每控制周期调用一次；失败后重试零指令。物理断线时仍需电机端失联保护。
 */
#ifndef ROBOT_CAN_TX_GUARD_HPP_
#define ROBOT_CAN_TX_GUARD_HPP_
#include <cstdint>

namespace robot {
class CanTxGuard {
 public:
  static constexpr uint32_t kStopFeedingAfterMs = 10u;
  static constexpr uint32_t kRecoveryMs = 100u;

  /** 仅在有效遥控急停持续成立时接受复位确认，避免故障消失后自动起动。 */
  void update(bool success, bool reset_ack, uint32_t now) {
    if (!success) {
      ++failed_cycles_;
      faulted_ = true;
      recovering_ = false;
      if (!failing_) bad_since_ = now;
      failing_ = true;
      watchdog_allowed_ = (now - bad_since_) < kStopFeedingAfterMs;
      return;
    }
    failing_ = false;
    watchdog_allowed_ = true;
    if (!faulted_ || !reset_ack) {
      recovering_ = false;
      return;
    }
    if (!recovering_) good_since_ = now;
    recovering_ = true;
    if ((now - good_since_) >= kRecoveryMs) {
      faulted_ = false;
      recovering_ = false;
    }
  }

  bool healthy() const { return !faulted_; }
  bool canFeedWatchdog() const { return watchdog_allowed_; }
  uint32_t failedCycles() const { return failed_cycles_; }

 private:
  bool faulted_ = false;
  bool failing_ = false;
  bool recovering_ = false;
  bool watchdog_allowed_ = true;
  uint32_t bad_since_ = 0, good_since_ = 0, failed_cycles_ = 0;
};
}  // namespace robot
#endif
