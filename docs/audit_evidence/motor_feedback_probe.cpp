// 实际驱动复核探针；只在主机运行。失败返回非零，不连接真实电机。
#include <cmath>
#include <cstdio>
#include "motor_DaMiao.hpp"
int main() {
  using namespace hello_world::motor;
  DM_J4310 motor;
  OptionalParams opt;
  opt.input_type = InputType::kTorq;
  motor.init(1, opt, true, false);
  // Enabled id=1, p=32768, v=0x12F, torque=0x800, normal temperatures.
  uint8_t frame[8] = {0x11,0x80,0x00,0x12,0xF8,0x00,25,25};
  motor.decode(frame);
  const float expected = (float(0x12F) / 4095.0f) * 42.0f - 21.0f;
  std::printf("velocity: expected=%.6f actual=%.6f (rad/s)\n",expected,motor.vel());
  const bool velocity_ok=std::fabs(expected-motor.vel())<0.00001f;
  frame[0] = 0xB1; frame[6] = 100;
  const bool accepted = motor.decode(frame);
  uint8_t tx[8] = {};
  motor.encode(tx);
  std::printf("overtemp: decode=%d status=0x%X next_command=0x%X\n",accepted,motor.status_code(),tx[7]);
  frame[0] = 0x12;
  const bool wrong_node=motor.decode(frame);
  std::printf("wrong_node_nibble: configured=1 payload=2 decode=%d\n",wrong_node);
  return velocity_ok && accepted && tx[7]==0xFD && motor.faultLatched() && !wrong_node ? 0 : 1;
}
