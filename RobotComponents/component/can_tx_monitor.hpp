/** @file can_tx_monitor.hpp
 *  @brief 区分入邮箱与完成发送；2ms内未完成或TXOK缺失均判失败。
 *  @note ACK只证明总线上有接收端应答，不证明目标电机执行；仍需反馈和物理验收。
 */
#ifndef ROBOT_CAN_TX_MONITOR_HPP_
#define ROBOT_CAN_TX_MONITOR_HPP_
#include <cstdint>
#include "can_tx_receipt.hpp"
namespace robot {
/**
 * @brief 跟踪单个 CAN 控制器的三个发送邮箱及可选回执。
 * @note  pending/tx_ok 的低三位一一对应邮箱；发送入队与硬件 TXOK 分别确认。
 */
class CanTxMonitor {
 public:
  static constexpr uint32_t kDeadlineMs=2u;
  uint8_t tracked() const { return tracked_; }
  bool cancelling() const { return cancelling_; }
  /** 先把挂起回执标为未确认；邮箱跟踪保留到硬件撤销完成。 */
  void beginCancel() {
    cancelling_ = true;
    for (int i = 0; i < 3; ++i) finish(i, CanTxStatus::kUnconfirmed);
  }
  /** 入邮箱后开始计时；receipt 指向的对象须一直存活到完成或撤销。 */
  void submitted(uint8_t mask,uint32_t now,CanTxReceipt* receipt=nullptr) {
    for(int i=0;i<3;++i) if(mask&(1u<<i)) {
      since_[i]=now;
      receipts_[i]=receipt;
    }
    if (receipt) receipt->status = CanTxStatus::kPending;
    tracked_|=mask;
  }
  /** 检查硬件快照；清空邮箱同时带 TXOK 才确认为已发送。 */
  bool poll(uint8_t pending,uint8_t tx_ok,uint32_t now) {
    bool healthy=true;
    for(int i=0;i<3;++i) if(tracked_&(1u<<i)) {
      if(!(pending&(1u<<i))) {
        const bool sent = (tx_ok & (1u << i)) != 0;
        if (!sent) healthy=false;
        finish(i, sent ? CanTxStatus::kSent : CanTxStatus::kUnconfirmed);
        tracked_&=uint8_t(~(1u<<i));
      } else if(now-since_[i]>=kDeadlineMs) {
        // 超时只报告故障。邮箱可能仍在发，须等控制器确认撤销。
        healthy=false;
      }
    }
    return healthy;
  }
  /** HAL 层已确认邮箱清空后结束撤销；超时本身不能代替硬件确认。 */
  void confirmEmpty() { tracked_=0; cancelling_=false; }
 private:
  /** 只完成一次回执并释放引用，避免后续轮询重复写入已消费的回执。 */
  void finish(int i, CanTxStatus result) {
    if (receipts_[i] != nullptr) {
      receipts_[i]->status = result;
      receipts_[i] = nullptr;
    }
  }
  bool cancelling_=false;
  // tracked_ 是尚未完成确认的邮箱位图；since_ 保存各邮箱入队时刻，单位 ms。
  uint8_t tracked_=0;
  uint32_t since_[3]={};
  CanTxReceipt* receipts_[3]={};
};
}
#endif
