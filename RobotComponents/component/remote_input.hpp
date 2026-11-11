/** @file remote_input.hpp
 *  @brief 每周期遥控快照与连续死区。只处理应用输入，不改变DBUS协议校验。
 */
#ifndef ROBOT_REMOTE_INPUT_HPP_
#define ROBOT_REMOTE_INPUT_HPP_
#include <cmath>
#include <cstdint>
#include "DT7.hpp"
namespace robot {
/** 2%归一化范围；阈值内为0，外侧重新缩放，满杆仍为1。 */
constexpr float kRemoteDeadband = 0.02f;
inline float RemoteAxis(float value) {
  if (!std::isfinite(value)) return 0.0f;
  const float magnitude = std::fmin(std::fabs(value), 1.0f);
  if (magnitude <= kRemoteDeadband) return 0.0f;
  return std::copysign((magnitude-kRemoteDeadband)/(1.0f-kRemoteDeadband), value);
}
struct RemoteInput {
  float lv=0, lh=0, rv=0, rh=0, wheel=0;
  uint8_t left=0, right=0;
  bool online=false;
  void capture(const hello_world::remote_control::DT7* rc, bool alive) {
    *this = RemoteInput{};
    online = alive && rc != nullptr;
    if (!online) return;
    lv=rc->rc_lv(); lh=rc->rc_lh();
    rv=rc->rc_rv(); rh=rc->rc_rh();
    wheel=rc->rc_wheel();
    left=rc->rc_l_switch(); right=rc->rc_r_switch();
  }
  /** 数据复制结束、恢复中断后再做浮点变换。 */
  void normalize() {
    lv=RemoteAxis(lv);lh=RemoteAxis(lh);rv=RemoteAxis(rv);rh=RemoteAxis(rh);wheel=RemoteAxis(wheel);
  }
};
}
#endif
