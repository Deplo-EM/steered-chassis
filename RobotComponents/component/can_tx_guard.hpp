/**
 * @file can_tx_guard.hpp
 * @brief CAN提交与上一批发送完成监督结果的故障锁存。两板共用，无 HAL 依赖。
 * @note success汇总编码/入邮箱、已有邮箱TXOK/超时、停机撤销结果；不代表目标电机已执行。
 *       正常每控制周期调用一次；故障需编码前处理，周期末不得重复累计同一失败；失败后重试零指令。物理断线时仍需电机端失联保护。
 */
#ifndef ROBOT_CAN_TX_GUARD_HPP_
#define ROBOT_CAN_TX_GUARD_HPP_
#include <cstdint>
#include "basic_mode.hpp"

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

  bool healthy() const { return ROBOT_BASIC_MODE ? true : !faulted_; }
  bool canFeedWatchdog() const { return ROBOT_BASIC_MODE ? true : watchdog_allowed_; }
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
