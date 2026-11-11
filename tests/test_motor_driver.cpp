// 使用实际达妙编解码器：字节边界、节点隔离、故障锁存，禁止自动清错/重新出力。
#include <cmath>
#include <cstdio>
#include <cstring>
#include "motor_DaMiao.hpp"
namespace {
int checks=0, failures=0;
#define CHECK(x) do { ++checks; if (!(x)) { ++failures; std::printf("FAIL %d: %s\n",__LINE__,#x); } } while(0)
using namespace hello_world::motor;
void Frame(uint8_t f[8], uint8_t status=1, uint8_t id=1, uint16_t velocity=0x800) {
  const uint8_t raw[8]={uint8_t((status<<4)|id),0x80,0x00,uint8_t(velocity>>4),uint8_t((velocity<<4)|0x8),0,25,25};
  std::memcpy(f,raw,8);
}
}
int main() {
  using namespace hello_world::motor;
  DM_J4310 motor; OptionalParams opt; opt.input_type=InputType::kTorq;
  motor.init(1,opt,true,false);
  uint8_t frame[8],tx[8];
  // 扫描速度低四位，恰好覆盖旧代码重复取data[3]的所有情况。
  for (int low=0;low<16;++low) {
    const uint16_t velocity=0x120|low;
    Frame(frame,1,1,velocity); CHECK(motor.decode(frame));
    const float expected=float(velocity)*42.0f/4095.0f-21.0f;
    CHECK(std::fabs(motor.vel()-expected)<0.0001f);
  }
  const float last_angle=motor.angle();
  CHECK(motor.encode(tx)); // 固定MIT模式不能在init末尾被重置成Unknown。
  Frame(frame,1,2); CHECK(!motor.decode(frame)); CHECK(motor.angle()==last_angle);
  Frame(frame,2); CHECK(!motor.decode(frame));
  for (int fault=8;fault<=14;++fault) {
    motor.init(1,opt,true,false); Frame(frame); motor.decode(frame);
    CHECK(motor.is_enabled()); motor.setInput(2.0f);
    Frame(frame,uint8_t(fault)); CHECK(motor.decode(frame));
    CHECK(!motor.is_enabled()); CHECK(motor.getInput()==0.0f);
    motor.setInput(2.0f); CHECK(motor.getInput()==0.0f);
    for (int t=0;t<4;++t) {
      CHECK(motor.encode(tx)); CHECK(tx[7]==DaMiao::kDisable);
    }
    // 后来收到正常帧也不能悄悄解除锁存。
    Frame(frame); motor.decode(frame); CHECK(!motor.is_enabled());
    CHECK(motor.encode(tx)); CHECK(tx[7]==DaMiao::kDisable);
  }
  // 复位/重新初始化是明确的恢复边界；仍需上层重新授权。
  motor.init(1,opt,true,false); Frame(frame); motor.decode(frame);
  CHECK(motor.is_enabled()); motor.setInput(1.0f); CHECK(motor.getInput()==1.0f);
  std::printf("Motor driver: %d checks, %d failures\n",checks,failures);
  return failures?1:0;
}
