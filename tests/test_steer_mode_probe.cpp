#include <cstdio>
#include <limits>
#include "steer_mode_probe.hpp"

using namespace robot::probe;
int failures = 0, checks = 0;
#define CHECK(x) do { ++checks; if (!(x)) { ++failures; std::printf("FAIL %d: %s\n", __LINE__, #x); } } while (0)

int main() {
  SteerModeProbe probe;
  Input in;
  in.healthy = true;
  in.left = 1;
  in.right = 3;
  in.estop = false;
  in.forward = 0.3f;
  CHECK(probe.step(in, 0).raw == 0);  // 上电时解除急停或带杆均无效。
  auto ready = [&](uint32_t t, uint8_t mode) {
    in = {};
    in.healthy = true;
    in.left = mode;
    in.right = 2;
    in.estop = true;
    CHECK(probe.step(in, t).raw == 0);
    CHECK(probe.step(in, t + 150).raw == 0);
    in.right = 3;
    in.estop = false;
    CHECK(probe.step(in, t + 151).raw == 0);
    CHECK(probe.step(in, t + 231).raw == 0);
    in.forward = 0.3f;
    return t + 232;
  };
  auto t = ready(1000, 1);
  auto cmd = probe.step(in, t);
  CHECK(cmd.id == 0x1FF && cmd.raw > 0 && cmd.raw <= 3000);
  CHECK(probe.step(in, t + 159).raw == 3000);
  CHECK(probe.step(in, t + 160).raw == 0);
  CHECK(probe.step(in, t + 1000).raw == 0); // 一次急停确认只允许一次。
  uint8_t frame[8];
  cmd.encode(frame);
  CHECK(frame[0] == 0 && frame[1] == 0 && frame[2] == (cmd.raw >> 8) &&
        frame[3] == (cmd.raw & 0xff) && frame[4] == 0 && frame[5] == 0 && frame[6] == 0 && frame[7] == 0);

  t = ready(2500, 3);
  cmd = probe.step(in, t);
  CHECK(cmd.id == 0x1FE && cmd.raw > 0 && cmd.raw <= 2457);
  CHECK(probe.step(in, t + 100).raw == 2457);
  in.angle = 0.13f;
  in.current = 0.28f;
  CHECK(probe.step(in, t + 101).raw == 0);
  CHECK(probe.current_result.peak_angle_rad >= 0.13f && probe.current_result.peak_delta_a >= 0.28f);
  CHECK(probe.current_result.completed == 1);

  t = ready(3500, 3);
  CHECK(probe.step(in, t).raw > 0);
  in.healthy = false;
  CHECK(probe.step(in, t + 1).raw == 0);
  in.healthy = true;
  CHECK(probe.step(in, t + 2).raw == 0);

  t = ready(4500, 1);
  CHECK(probe.step(in, t).raw > 0);
  in.estop = true;
  CHECK(probe.step(in, t + 1).raw == 0);

  t = ready(5500, 1);
  CHECK(probe.step(in, t).raw > 0);
  in.forward = 0.0f;
  CHECK(probe.step(in, t + 1).raw == 0);
  in.forward = 0.3f;
  CHECK(probe.step(in, t + 2).raw == 0);

  t = ready(6500, 2);  // 左下不选任何协议。
  CHECK(probe.step(in, t).raw == 0);
  t = ready(7500, 1);
  in.right = 1;  // 只接受右中档。
  CHECK(probe.step(in, t).raw == 0);
  t = ready(8500, 1);
  in.angle = std::numeric_limits<float>::quiet_NaN();
  CHECK(probe.step(in, t).raw == 0);
  t = ready(9500, 1);
  in.other_neutral = false;
  CHECK(probe.step(in, t).raw == 0);

  std::printf("steer mode probe: %d checks, %d failures\n", checks, failures);
  return failures != 0;
}
