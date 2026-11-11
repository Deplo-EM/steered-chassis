/**
 * @file dbus_relay.hpp
 * @brief 原始DBUS跨板转发：18字节分成3帧，完整校验后一次提交。
 * @note 0x212/213/214各携带6字节原文、1字节同组序号、1字节CRC8。
 *       CRC包含标准帧ID，防止片号互换；不改DT7精度或协议宽度。
 *       publish只允许在本地合法UART帧到达时调用，不能用周期心跳制造新序号。
 *       调用方在UART/CAN中断与控制任务之间使用短临界区保护本对象。
 */
#ifndef ROBOT_DBUS_RELAY_HPP_
#define ROBOT_DBUS_RELAY_HPP_
#include <cstdint>
#include <cstring>
#include "crc8.hpp"
#include "dbus_check.hpp"
namespace robot {
class DbusRelay {
 public:
  static constexpr uint32_t kFirstId=0x212u;
  static constexpr uint32_t kOfflineMs=100u;
  static constexpr uint32_t kAssemblyMs=20u;
  static bool ownsId(uint32_t id) { return id>=kFirstId && id<kFirstId+3u; }

  /** 本地合法DBUS是唯一发送源；每次真实接收产生新序号。 */
  bool publish(const uint8_t raw[18],uint32_t now) {
    if(!raw || !IsDbusFrameValid(raw)) return false;
    std::memcpy(published_frame_,raw,18);
    ++generation_; published_tick_=now; published_=true;
    return true;
  }
  /** 每周期最多发一片，给共享CAN上的yaw/状态帧保留邮箱。 */
  bool peekTx(uint32_t now,uint32_t& id,uint8_t out[8]) {
    if(!out || !published_ || now-published_tick_>kOfflineMs) return false;
    if(tx_active_ && now-tx_tick_>kAssemblyMs) tx_active_=false;
    if(!tx_active_) {
      if(sent_ && generation_==sent_generation_) return false;
      std::memcpy(tx_frame_,published_frame_,18);
      tx_generation_=generation_;tx_tick_=published_tick_;tx_part_=0;tx_active_=true;
    }
    id=kFirstId+tx_part_;
    std::memcpy(out,tx_frame_+6*tx_part_,6);
    out[6]=uint8_t(tx_generation_);out[7]=crc(id,out);
    return true;
  }
  /** 仅HAL接受当前片后推进；提交成功仍由CAN邮箱完成监督另行确认。 */
  void txAccepted() {
    if(!tx_active_) return;
    if(++tx_part_==3) {
      tx_active_=false;sent_=true;sent_generation_=tx_generation_;
    }
  }
  /** 只在完整、同序号、20ms内、DBUS合法且序号新鲜时提交。 */
  bool accept(uint32_t id,const uint8_t data[8],uint32_t now) {
    if(!ownsId(id) || !data || data[7]!=crc(id,data)) return reject();
    const uint8_t seq=data[6];
    if(online(now)) {
      const uint8_t delta=uint8_t(seq-rx_seq_);
      if(delta==0 || delta>=128) return reject();
    }
    if(assembling_ && now-assembly_tick_>kAssemblyMs) assembling_=false;
    if(!assembling_ || assembly_seq_!=seq) {
      if(assembling_ && uint8_t(seq-assembly_seq_)>=128) return reject();
      assembling_=true;assembly_tick_=now;assembly_seq_=seq;parts_=0;
    }
    const uint8_t part=uint8_t(id-kFirstId), bit=uint8_t(1u<<part);
    if(parts_&bit) return reject(); // 重复片不刷新拼包时间或在线时间
    std::memcpy(assembly_frame_+6*part,data,6);parts_|=bit;
    if(parts_!=7) return false;
    assembling_=false;
    if(!IsDbusFrameValid(assembly_frame_)) return reject();
    if(received_ && !online(now)) {
      const uint8_t progress=uint8_t(seq-resync_seq_);
      const bool resynced=resync_pending_ && progress>0u && progress<128u &&
                          now-resync_tick_<=kAssemblyMs;
      resync_pending_=!resynced;resync_seq_=seq;resync_tick_=now;
      if(!resynced) return false;
    }
    std::memcpy(rx_frame_,assembly_frame_,18);
    rx_seq_=seq;last_rx_tick_=assembly_tick_;received_=true;resync_pending_=false;
    ++rx_frames_;return true;
  }
  bool online(uint32_t now) const { return received_ && now-last_rx_tick_<=kOfflineMs; }
  const uint8_t* frame() const { return rx_frame_; }
  uint32_t rxFrames() const { return rx_frames_; }
  uint32_t rejectedFrames() const { return rejected_frames_; }
 private:
  static uint8_t crc(uint32_t id,const uint8_t* data) {
    hello_world::tools::crc8_check check;
    const uint8_t tag[2]={uint8_t(id>>8),uint8_t(id)};
    return check.getCrc8CheckNum(data,7,check.getCrc8CheckNum(tag,2,0));
  }
  bool reject() { ++rejected_frames_;return false; }
  uint8_t published_frame_[18]={},tx_frame_[18]={},assembly_frame_[18]={},rx_frame_[18]={};
  uint32_t generation_=0,published_tick_=0,tx_generation_=0,tx_tick_=0,sent_generation_=0;
  uint32_t assembly_tick_=0,last_rx_tick_=0,resync_tick_=0,rx_frames_=0,rejected_frames_=0;
  uint8_t tx_part_=0,assembly_seq_=0,parts_=0,rx_seq_=0,resync_seq_=0;
  bool published_=false,tx_active_=false,sent_=false,assembling_=false,received_=false,resync_pending_=false;
};
}  // namespace robot
#endif
