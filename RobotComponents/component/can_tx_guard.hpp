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
/**
 * @brief 汇总通信任务的成功/失败，并分别管理出力故障和喂狗资格。
 * @note  一次失败即锁存故障；持续失败达到阈值才撤销喂狗。
 *        通信恢复后仍须保持有效急停确认，才能解除故障锁存。
 */
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
      // 只在连续失败段的起点记时；反复失败不能把计时起点不断后移。
      if (!failing_) bad_since_ = now;
      failing_ = true;
      watchdog_allowed_ = (now - bad_since_) < kStopFeedingAfterMs;
      return;
    }
    // 一次成功结束连续失败段，恢复喂狗资格；出力故障另走下方确认流程。
    failing_ = false;
    watchdog_allowed_ = true;
    if (!faulted_ || !reset_ack) {
      recovering_ = false;
      return;
    }
    // 仅成功且急停确认持续成立时累计恢复时间；中途任一条件丢失便重计。
    if (!recovering_) good_since_ = now;
    recovering_ = true;
    if ((now - good_since_) >= kRecoveryMs) {
      faulted_ = false;
      recovering_ = false;
    }
  }

  /** 返回锁存是否解除；与 canFeedWatchdog 的延时策略分别使用。 */
  bool healthy() const { return !faulted_; }
  bool canFeedWatchdog() const { return watchdog_allowed_; }
  uint32_t failedCycles() const { return failed_cycles_; }

 private:
  bool faulted_ = false;
  bool failing_ = false;
  bool recovering_ = false;
  bool watchdog_allowed_ = true;
  uint32_t bad_since_ = 0, good_since_ = 0, failed_cycles_ = 0;  ///< 两段起点均为 ms，失败数为累计周期数。
};
}  // namespace robot
#endif
