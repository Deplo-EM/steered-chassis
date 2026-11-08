// 同一字节流同时经过实际编解码器；可控时钟检验过期、重发和序号回绕。
#include <cmath>
#include <cstdio>
#include <cstring>
#include <limits>
#include <initializer_list>
#include "gimbal_chassis_comm.hpp"
#include "crc8.hpp"
#include "can_tx_guard.hpp"
uint32_t protocol_now = 0;
namespace hello_world::tick {
uint64_t GetTickUs() { return uint64_t(protocol_now) * 1000u; }
uint32_t GetTickMs() { return protocol_now; }
uint32_t GetTickS() { return protocol_now / 1000u; }
void DelayUs(uint32_t) {}
}
int failures = 0, checks = 0;
#define CHECK(expr) do { ++checks; if (!(expr)) { ++failures; std::printf("FAIL %d: %s\n", __LINE__, #expr); } } while (0)
void Crc(uint8_t* f) { hello_world::tools::crc8_check c; f[7] = c.getCrc8CheckNum(f, 7, 0); }
int main() {
  using Comm = robot::GimbalChassisComm;
  Comm tx(Comm::CodePart::Chassis, 0x210, 0x211), rx(Comm::CodePart::Gimbal, 0x211, 0x210);
  uint8_t frame[8], old[8];
  tx.chassis_data().working = tx.chassis_data().gimbal_enable = true;
  tx.chassis_data().dbus_online = true;
  tx.encode(frame); CHECK(rx.decode(frame));
  std::memcpy(old, frame, 8);
  protocol_now = 80; CHECK(!rx.decode(frame));
  protocol_now = 101; CHECK(rx.isOffline());
  CHECK(!rx.decode(old)); CHECK(rx.isOffline()); // 过期之后同一帧也不能重新授权。
  tx.encode(frame); CHECK(rx.decode(frame)); CHECK(!rx.isOffline());
  // 持续流允许 255 -> 0，允许有限丢帧；相同序号与逆序拒绝。
  for (int i = 0; i < 260; ++i) { ++protocol_now; tx.encode(frame); CHECK(rx.decode(frame)); }
  CHECK(!rx.decode(old));
  tx.encode(frame); tx.encode(frame); CHECK(rx.decode(frame));
  // CRC 正确也不代表字段合法。
  tx.encode(frame); frame[5] = 1; Crc(frame); CHECK(!rx.decode(frame));
  tx.encode(frame); frame[0] |= 0x30u; Crc(frame); CHECK(!rx.decode(frame));
  // 对方重启后序号归零：离线后先同步一帧，后续递增帧才恢复在线。
  protocol_now += 101;
  Comm reboot(Comm::CodePart::Chassis, 0x210, 0x211);
  reboot.encode(frame); CHECK(!rx.decode(frame)); CHECK(rx.isOffline());
  ++protocol_now; reboot.encode(frame); CHECK(rx.decode(frame));
  // 发送端的 NaN/Inf 不能变成可执行的云台目标。
  for (float v : {std::numeric_limits<float>::quiet_NaN(), std::numeric_limits<float>::infinity()}) {
    tx.chassis_data().yaw_rate = v;
    tx.encode(frame);
    Comm clean(Comm::CodePart::Gimbal, 0, 0);
    CHECK(clean.decode(frame));
    CHECK(!clean.chassis_data().working); CHECK(!clean.chassis_data().gimbal_enable);
    CHECK(clean.chassis_data().estop); CHECK(clean.chassis_data().yaw_rate == 0);
  }
  // 锁存时序、确认中断及毫秒计数回绕。
  robot::CanTxGuard guard;
  guard.update(false, false, 0xfffffffau); CHECK(!guard.healthy()); CHECK(guard.canFeedWatchdog());
  guard.update(false, false, 3u); CHECK(guard.canFeedWatchdog());
  guard.update(false, false, 4u); CHECK(!guard.canFeedWatchdog());
  guard.update(true, true, 5u); guard.update(true, true, 104u); CHECK(!guard.healthy());
  guard.update(true, false, 105u); guard.update(true, true, 106u);
  guard.update(true, true, 205u); CHECK(!guard.healthy());
  guard.update(true, true, 206u); CHECK(guard.healthy()); CHECK(guard.canFeedWatchdog());
  std::printf("Protocol safety: %d checks, %d failures\n", checks, failures);
  return failures ? 1 : 0;
}
