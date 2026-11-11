/** @file can_tx_monitor.hpp
 *  @brief 区分入邮箱与完成发送；2ms内未完成或TXOK缺失均判失败。
 *  @note ACK只证明总线上有接收端应答，不证明目标电机执行；仍需反馈和物理验收。
 */
#ifndef ROBOT_CAN_TX_MONITOR_HPP_
#define ROBOT_CAN_TX_MONITOR_HPP_
#include <cstdint>
namespace robot {
class CanTxMonitor {
 public:
  static constexpr uint32_t kDeadlineMs=2u;
  uint8_t tracked() const { return tracked_; }
  void submitted(uint8_t mask,uint32_t now) {
    for(int i=0;i<3;++i) if(mask&(1u<<i)) since_[i]=now;
    tracked_|=mask;
  }
  bool poll(uint8_t pending,uint8_t tx_ok,uint32_t now) {
    bool healthy=true;
    for(int i=0;i<3;++i) if(tracked_&(1u<<i)) {
      if(!(pending&(1u<<i))) {
        if(!(tx_ok&(1u<<i))) healthy=false;
        tracked_&=uint8_t(~(1u<<i));
      } else if(now-since_[i]>=kDeadlineMs) healthy=false;
    }
    return healthy;
  }
  void reset() { tracked_=0; }
 private:
  uint8_t tracked_=0;
  uint32_t since_[3]={};
};
}
#endif
