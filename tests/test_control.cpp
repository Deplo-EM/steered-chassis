// 实际 Chassis、逆解、PID、实例参数的回归；仅电机反馈/写入接口为主机替身。
#include <cmath>
#include <cstdio>
#include <limits>
#include "chassis.hpp"
#include "ins_chassis_iksolver.hpp"
#include "chassis_debug.hpp"
#include "ins_motor.hpp"
#include "ins_pid.hpp"
#include "motor_base.hpp"
#include "pid.hpp"
#include "critical_section.hpp"
#include "remote_input.hpp"

namespace {
int checks = 0, failures = 0;
const char* current_case = "";
#define CHECK(expr) do { ++checks; if (!(expr)) { ++failures; std::printf("FAIL %s:%d: %s\n", current_case, __LINE__, #expr); } } while (0)
bool Near(float a, float b, float tol = 0.0001f) { return std::fabs(a-b) <= tol; }
constexpr float dt = 0.001f;
constexpr float pi = 3.14159265358979f;
const float nan = std::numeric_limits<float>::quiet_NaN();
const float inf = std::numeric_limits<float>::infinity();
using hello_world::pid::Pid;
using hello_world::chassis_ik_solver::ChassisIkSolver;
using hello_world::chassis_ik_solver::Geometry;
using hello_world::chassis_ik_solver::WheelCommand;

struct FakeMotor : hello_world::motor::Motor {
  float command = 0;
  bool decode(const uint8_t[8]) override { return true; }
  bool encode(uint8_t[8]) override { return true; }
  hello_world::motor::Status setInput(float input) override { command = input; return hello_world::motor::Status::kOk; }
  void feedback(float a, float v = 0) { angle_ = a; vel_ = v; }
};
struct Rig {
  ChassisIkSolver solver;
  FakeMotor wheel[4], steer[4];
  Pid wp[4], sp[4];
  robot::Chassis chassis;
  explicit Rig(const Geometry& geo = {}) {
    hello_world::motor::Motor *w[4], *s[4]; Pid *wpp[4], *spp[4];
    solver.init(geo, 10, 3);
    for (int i=0;i<4;++i) {
      w[i]=&wheel[i]; s[i]=&steer[i]; wpp[i]=&wp[i]; spp[i]=&sp[i];
      wp[i].init(0.35f,1.5f,0,8,2.5f); sp[i].init(10000,2000,150,20000,1);
    }
    chassis.init(w,s,wpp,spp,&solver);
  }
  void update(float vx, float vy, float w=0, float period=dt) {
    robot::ChassisMotion m; m.vx=vx; m.vy=vy; m.w=w;
    chassis.setMotion(m,0); chassis.update(period);
  }
  bool allZero() const {
    for (int i=0;i<4;++i) if (wheel[i].command != 0 || steer[i].command != 0) return false;
    return true;
  }
};

void TestAlignment() {
  current_case="90-degree start uses final heading";
  Rig r;
  r.update(0,1);
  CHECK(Near(r.chassis.wheelCommand(0).angle,0.01f));
  CHECK(r.steer[0].command > 0);
  for (const auto& m:r.wheel) CHECK(m.command == 0);

  current_case="direction change discards old drive integral";
  r.chassis.stop();
  for (int t=0;t<100;++t) r.update(1,0);
  CHECK(r.wp[0].integral()>0);
  r.update(0,1);
  for (int i=0;i<4;++i) { CHECK(r.wheel[i].command==0); CHECK(r.wp[i].integral()==0); }

  current_case="170-degree equivalent flip drives backwards";
  r.chassis.stop();
  r.update(-0.98480775f,0.17364818f);
  CHECK(r.wheel[0].command == 0);
  CHECK(r.steer[0].command < 0);
  for (auto& s : r.steer) s.feedback(-pi / 18.0f);
  for (int t=0;t<30;++t) r.update(-0.98480775f,0.17364818f);
  CHECK(r.wheel[0].command < 0);

  current_case="stop then restart does not retain steering history";
  r.chassis.stop(); CHECK(r.allZero());
  for (auto& s:r.steer) s.feedback(pi/2);
  r.update(0,1); CHECK(r.wheel[0].command>0);
  CHECK(Near(r.chassis.wheelCommand(0).angle,pi/2));
  r.update(1,0); CHECK(r.wheel[0].command==0);

  current_case="rotation waits for every driven wheel to align";
  Rig turning;
  // 纯旋转时，左前轮已在切向，另外三个轮组仍指向车头。
  turning.steer[0].feedback(3*pi/4);
  turning.update(0,0,1);
  CHECK(turning.chassis.wheelCommand(0).speed > 0);
  CHECK(turning.steer[1].command != 0);
  for (const auto& m:turning.wheel) CHECK(m.command == 0);

  for (int i=0;i<4;++i) {
    turning.steer[i].feedback(turning.chassis.wheelCommand(i).desired_angle);
  }
  turning.update(0,0,1);
  // A stale internal target must not hold the drive after all four measurements align.
  for (const auto& m:turning.wheel) CHECK(m.command != 0);

  // 任一受驱轮失去对齐时，整组撤销驱动并清旧积分，舵角环继续纠偏。
  turning.steer[2].feedback(turning.chassis.wheelCommand(2).desired_angle+0.7f);
  turning.update(0,0,1);
  CHECK(turning.steer[2].command != 0);
  for (int i=0;i<4;++i) {
    CHECK(turning.wheel[i].command == 0);
    CHECK(turning.wp[i].integral() == 0);
  }

  // 超时只报警：一个轮组持续偏离时，其余轮也不得自行恢复驱动。
  for (int t=0;t<400;++t) {
    for (int i=0;i<4;++i) {
      const float target=turning.chassis.wheelCommand(i).desired_angle;
      turning.steer[i].feedback(i==2 ? target+0.7f : target);
    }
    turning.update(0,0,1);
  }
  CHECK(turning.chassis.alignTimedOut());
  CHECK(turning.chassis.alignBlockedMask() == (1u<<2));
  CHECK(turning.wheel[2].command == 0);
  for (const auto& m:turning.wheel) CHECK(m.command == 0);

  current_case="equivalent direction uses measured angle despite old trajectory";
  Rig stuck;
  const auto move = [&](float a) { stuck.update(0.5f*std::cos(a), 0.5f*std::sin(a)); };
  for (int t=0; t<200; ++t) move(80*pi/180);
  move(160*pi/180);
  const auto& command = stuck.chassis.wheelCommand(0);
  CHECK(command.speed < 0);
  CHECK(Near(robot::WrapPi(command.desired_angle), -20*pi/180));
  CHECK(stuck.wheel[0].command == 0); // 限速目标还在回程，驱动先等舵角。

  current_case="stationary steer with sustained large output stops entire chassis";
  Rig stalled;
  for (int t=0;t<1000;++t) stalled.update(0,1);
  CHECK(stalled.allZero());
  CHECK(stalled.chassis.steerStallMask() == 0x0Fu);
  const auto before_ack = stalled.chassis.lastSteerStall();
  CHECK(before_ack.sequence == 1 && before_ack.stall_mask == 0x0Fu);
  CHECK(before_ack.requested_raw[0] >= 8000 && Near(before_ack.measured_rad[0], 0));
  stalled.chassis.stop();
  stalled.update(1,0);
  CHECK(stalled.allZero()); // 清 PID 历史不能同时解除堵转。
  stalled.chassis.acknowledgeSteerStall();
  stalled.update(1,0);
  CHECK(stalled.chassis.steerStallMask() == 0);
  CHECK(stalled.chassis.lastSteerStall().stall_mask == before_ack.stall_mask);
  CHECK(stalled.chassis.lastSteerStall().sequence == before_ack.sequence);
  for(const auto& m:stalled.wheel) CHECK(m.command > 0);
}

void TestDriveGateChatter() {
  current_case="alignment gate must stay closed until errors have cleared the reentry margin";
  Rig r;
  r.update(1, 0);
  r.steer[0].feedback(0.199f);
  r.update(1, 0);
  CHECK(r.chassis.driveReady() && r.wheel[0].command > 0);
  r.steer[0].feedback(0.201f);
  r.update(1, 0);
  CHECK(!r.chassis.driveReady() && r.chassis.alignBlockedMask() == 1);
  bool stayed_stopped = true;
  for (int t = 0; t < 30; ++t) {
    r.steer[0].feedback((t % 2 == 0) ? 0.199f : 0.201f);
    r.update(1, 0);
    for (int i = 0; i < 4; ++i) {
      stayed_stopped &= r.wheel[i].command == 0.0f && r.wp[i].integral() == 0.0f;
    }
  }
  CHECK(stayed_stopped);
  r.steer[0].feedback(0.11f);
  r.update(1, 0);
  CHECK(r.chassis.driveReady());
  for (const auto& m : r.wheel) CHECK(m.command > 0);

  current_case="zero wheel demand clears accumulated propulsion";
  for (int t = 0; t < 50; ++t) r.update(1, 0);
  CHECK(r.wp[0].integral() > 0);
  r.update(0, 0);
  for (int i = 0; i < 4; ++i) {
    CHECK(r.wheel[i].command == 0 && r.wp[i].integral() == 0);
  }
}

void TestStraightAlignment() {
  current_case="a visibly skewed steering module must not propel the chassis";
  Rig r;
  r.steer[2].feedback(0.25f);
  r.update(1.0f, 0.0f);
  CHECK(!r.chassis.driveReady());
  CHECK(r.chassis.alignBlockedMask() == 0x04u);
  for (const auto& wheel : r.wheel) CHECK(wheel.command == 0.0f);
  CHECK(r.steer[2].command < 0.0f);
  r.steer[2].feedback(0.09f);
  for (int t=0;t<30;++t) r.update(1.0f, 0.0f);
  CHECK(r.chassis.driveReady());
  CHECK(robot::diag::g_chassis_dbg.alignment_scale < 1.0f);
  for (const auto& wheel : r.wheel) CHECK(Near(wheel.command, r.wheel[0].command));
}

void TestReturnToMeasuredHeading() {
  current_case="returning to the measured heading must not wait on a stale steering target";
  Rig r;
  for (int i = 0; i < 100; ++i) r.update(0.0f, 1.0f);
  CHECK(!r.chassis.driveReady() && std::fabs(r.chassis.wheelCommand(0).angle) > 0.9f);
  r.update(1.0f, 0.0f);
  CHECK(r.chassis.driveReady() && r.chassis.alignBlockedMask() == 0u);
  CHECK(std::fabs(robot::WrapPi(r.chassis.wheelCommand(0).angle)) < 0.001f);
  CHECK(r.wheel[0].command > 0.0f && std::fabs(r.steer[0].command) < 0.001f);
}

void TestSteerTrajectory() {
  ChassisIkSolver solver;
  solver.init({}, 10, 3);
  WheelCommand out[4];
  float measured[4] = {};
  const auto solveHeading = [&](float degrees, float speed = 1.0f) {
    const float a = degrees * pi / 180.0f;
    solver.solve({speed * std::cos(a), speed * std::sin(a), 0.0f}, 0, measured, dt, out);
  };

  current_case="servo trajectory between equivalent targets does not cross behind stalled feedback";
  solveHeading(89);
  for (int t = 0; t < 180; ++t) solveHeading(92);
  CHECK(out[0].speed > 0 && Near(out[0].angle, 92*pi/180));
  bool stayed_on_valid_arc = true, stayed_limited = true;
  float previous = out[0].angle;
  for (int t = 0; t < 340; ++t) {
    solveHeading(-92);
    stayed_on_valid_arc &= std::fabs(robot::WrapPi(out[0].angle)) <= 92.01f*pi/180;
    stayed_limited &= std::fabs(robot::WrapPi(out[0].angle - previous)) <= 0.0101f;
    previous = out[0].angle;
  }
  CHECK(stayed_on_valid_arc && stayed_limited);
  CHECK(out[0].speed > 0 && Near(robot::WrapPi(out[0].angle), -92*pi/180));

  current_case="steering direction hysteresis survives a zero speed crossing";
  solver.reset();
  solveHeading(89);
  solveHeading(92, 0.01f);
  CHECK(out[0].speed == 0);
  solveHeading(92);
  CHECK(out[0].speed > 0);
  solver.reset();
  solveHeading(95);
  solveHeading(88, 0.01f);
  CHECK(out[0].speed == 0);
  solveHeading(88);
  CHECK(out[0].speed < 0);

  current_case="circular input crosses the encoder seam with a continuous physical target";
  solver.reset();
  bool smooth = true, correct = true;
  previous = 0.0f;
  for (int t = 0; t <= 6400; ++t) {
    const float theta = t * 0.004f;
    solver.solve({std::cos(theta), std::sin(theta), 0.0f}, 0, measured, dt, out);
    smooth &= std::fabs(robot::WrapPi(out[0].angle - previous)) <= 0.0101f;
    correct &= out[0].speed > 0 && std::fabs(robot::WrapPi(out[0].angle - theta)) < 0.001f;
    previous = out[0].angle;
    for (float& a : measured) a = robot::WrapPi(previous);
  }
  CHECK(smooth && correct);
}

void TestConfiguredSteerAngleRange() {
  current_case="configured GM6020 feedback stays single-turn through repeated encoder wraps";
  bool all_angles_in_range = true;
  for (int i = 0; i < 4; ++i) {
    auto* steer = GetMotorSteer(i);
    CHECK(steer->angle_range() == hello_world::motor::AngleRange::kNegPiToPosPi);
    for (unsigned n = 0; n < 8u * 8192u; ++n) {
      const uint16_t raw = static_cast<uint16_t>((n + 4000u) % 8192u);
      const uint8_t frame[8] = {uint8_t(raw >> 8), uint8_t(raw), 0, 0, 0, 0, 25, 0};
      all_angles_in_range &= steer->decode(frame) && std::fabs(steer->angle()) <= pi;
    }
  }
  CHECK(all_angles_in_range);
}

void TestConfiguredSteerFrames() {
  using namespace hello_world::motor;
  current_case="configured GM6020 instances encode the steering response and stop frame";
  Motor *w[4], *s[4];
  Pid *wp[4], *sp[4];
  ChassisIkSolver solver;
  robot::Chassis chassis;
  solver.init({},10,3);
  // 方向和零点来自配置；这组编码器读数把四个舵角设到接近 0。
  const uint16_t centered[4] = {4784,2696,8181,6124};
  for (int i=0;i<4;++i) {
    w[i]=GetMotorWheel(i); s[i]=GetMotorSteer(i);
    wp[i]=GetPidMotorWheel(i); sp[i]=GetPidMotorSteer(i);
    const uint8_t feedback[8]={uint8_t(centered[i]>>8),uint8_t(centered[i]),0,0,0,0,25,0};
    CHECK(s[i]->decode(feedback));
    CHECK(std::fabs(s[i]->angle())<0.003f);
  }
  chassis.init(w,s,wp,sp,&solver);
  const auto frame = [&](uint8_t bytes[8]) {
    for (const auto* motor : s) {
#if defined(ROBOT_STEER_CURRENT_FIELD) && ROBOT_STEER_CURRENT_FIELD
      CHECK(GM6020::GetTxId(motor->id(),motor->get_input_type())==0x1FE);
#else
      CHECK(GM6020::GetTxId(motor->id(),motor->get_input_type())==0x1FF);
#endif
    }
    for (auto* motor : s) CHECK(motor->encode(bytes));
  };
  robot::ChassisMotion m; m.vx=1; m.vy=1;
  for (int i=0;i<25;++i) { chassis.setMotion(m,0); chassis.update(dt); }
  uint8_t bytes[8]={}; frame(bytes);
  for (int i=0;i<4;++i) {
    const auto request=static_cast<int16_t>((bytes[2*i]<<8)|bytes[2*i+1]);
#if defined(ROBOT_COMPETITION_FINISH) && ROBOT_COMPETITION_FINISH
    // 收尾加强后（Kp 12000、电流上限 2.0 A）实测约 -1641。
    CHECK(request<=-1620 && request>=-1660);
#elif defined(ROBOT_STEER_CURRENT_FIELD) && ROBOT_STEER_CURRENT_FIELD
    // 电流上限 2.0 A 后的实测值约 -819。
    CHECK(request<=-800 && request>=-840);
#else
    CHECK(request<=-3650 && request>=-3950);
#endif
    CHECK(w[i]->getInput()==0);
  }
  chassis.stop(); frame(bytes);
  for (uint8_t byte : bytes) CHECK(byte==0);
#if defined(ROBOT_STEER_CURRENT_FIELD) && ROBOT_STEER_CURRENT_FIELD
#if defined(ROBOT_COMPETITION_FINISH) && ROBOT_COMPETITION_FINISH
  // A 增益下 45° 卡住会越过堵转门槛（实测 mask=15），锁存并撤销输出。
  for (int i=0;i<1000;++i) { chassis.setMotion(m,0); chassis.update(dt); }
  CHECK(chassis.steerStallMask()!=0);
#else
  // B01 电流档较低的 P：卡在 45° 时低于大出力堵转线。只能等待对齐，不应恢复其他轮。
  for (int i=0;i<1000;++i) { chassis.setMotion(m,0); chassis.update(dt); }
  CHECK(chassis.steerStallMask()==0 && chassis.alignTimedOut());
#endif
  for (auto* wheel : w) CHECK(wheel->getInput()==0);
  chassis.stop();
#endif
  // 90° 静止误差在两个配置下都能越过大出力门限。
  m.vx=0;
  for (int i=0;i<1000;++i) { chassis.setMotion(m,0); chassis.update(dt); }
  // 编码器零点误差会错开触发周期；任一舵停滞就要撤销整组输出。
  CHECK(chassis.steerStallMask()!=0 && chassis.alignmentAtStall()==15);
  frame(bytes);
  for (uint8_t byte : bytes) CHECK(byte==0);
  for (auto* wheel : w) CHECK(wheel->getInput()==0);
}

void TestSteerRateDamping() {
  current_case="configured steering trajectory rate";
  auto* configured = GetChassisIkSolver();
  configured->reset();
  const float measured[4]={};
  WheelCommand out[4];
  configured->solve({0.0f,1.0f,0.0f},0.0f,measured,dt,out);
#if defined(ROBOT_COMPETITION_FINISH) && ROBOT_COMPETITION_FINISH
  CHECK(Near(out[0].angle, 0.012f));
#else
  CHECK(Near(out[0].angle, 0.010f));
#endif
#if defined(ROBOT_STEER_CURRENT_FIELD) && ROBOT_STEER_CURRENT_FIELD
  current_case="current-profile steering brakes real motion without a setpoint change";
  Rig still, advancing, retreating;
  for (int t=0;t<25;++t) {
    for (int i=0;i<4;++i) {
      advancing.steer[i].feedback(0, 2.0f);
      retreating.steer[i].feedback(0, -2.0f);
    }
    still.update(1,1); advancing.update(1,1); retreating.update(1,1);
  }
  CHECK(advancing.steer[0].command < still.steer[0].command);
  CHECK(retreating.steer[0].command > still.steer[0].command);
  advancing.steer[2].feedback(0,nan);
  advancing.update(1,1); CHECK(advancing.allZero());
#endif
}

void TestUniformSaturation() {
  current_case="global wheel limit preserves twist proportions";
  ChassisIkSolver s; s.init({},10,3);
  robot::ChassisMotion m; m.vx=2; m.vy=1; m.w=6;
  float measured[4]={0,0,0,0}; WheelCommand out[4];
  for (int t=0;t<1000;++t) s.solve(m,0,measured,dt,out);
  // (vx-w*y, vy+w*x), r=(+-0.167,+-0.167); RF has greatest speed.
  const float x[4]={0.998f,0.998f,3.002f,3.002f};
  const float y[4]={2.002f,-0.002f,-0.002f,2.002f};
  const float scale=3.0f/std::hypot(3.002f,2.002f);
  for (int i=0;i<4;++i) {
    CHECK(std::fabs(out[i].speed)<=3.00001f);
    CHECK(Near(out[i].speed*std::cos(out[i].angle),x[i]*scale));
    CHECK(Near(out[i].speed*std::sin(out[i].angle),y[i]*scale));
  }
  current_case="spin desaturation keeps yaw; translation follows the same heading";
  s.reset();
  for (int t=0;t<1000;++t) s.solve(m, pi / 2.0f, measured, dt, out, true);
  float vx = 0.0f, vy = 0.0f, wz = 0.0f;
  const float px[4]={0.167f,-0.167f,-0.167f,0.167f};
  const float py[4]={0.167f,0.167f,-0.167f,-0.167f};
  for (int i=0;i<4;++i) {
    CHECK(std::fabs(out[i].speed)<=3.00001f);
    const float wx = out[i].speed * std::cos(out[i].angle);
    const float wy = out[i].speed * std::sin(out[i].angle);
    vx += 0.25f * wx;
    vy += 0.25f * wy;
    wz += -py[i] * wx + px[i] * wy;
  }
  CHECK(vx < 0.0f && vy > 0.0f && std::hypot(vx, vy) < std::hypot(m.vx,m.vy));
  CHECK(Near(vy,-2.0f*vx));
  CHECK(Near(wz/(8.0f*0.167f*0.167f), 6.0f));

  current_case="spin does not derate translation if each actual wheel fits";
  s.reset();
  s.solve({2.0f, 0.0f, 5.0f}, 0.0f, measured, dt, out, true);
  // 最快轮约 2.955 m/s；用模长相加的上界会误判超出 3 m/s。
  CHECK(Near(s.chassis_motion().vx, 2.0f));
  CHECK(Near(s.chassis_motion().w, 5.0f));
  for (const auto& c : out) CHECK(std::fabs(c.speed) <= 3.00001f);
}

void TestSteerKeepout() {
  current_case="two modules avoid different parts of their steering paths";
  Geometry geo;
  geo.steer_keepout[0] = {0.30f, 0.10f};
  geo.steer_keepout[2] = {-2.60f, 0.025f};
  ChassisIkSolver s;
  s.init(geo, 10.0f, 3.0f);
  float measured[4] = {0.0f, 0.0f, -2.65f, 0.0f};
  WheelCommand out[4];
  const robot::ChassisMotion request{std::cos(0.60f), std::sin(0.60f), 0.0f};
  bool stayed_clear = true, slew_ok = true;
  for (int t = 0; t < 400; ++t) {
    s.solve(request, 0.0f, measured, dt, out);
    for (const int i : {0, 2}) {
      const float increment = robot::WrapPi(out[i].angle - measured[i]);
      for (const float fraction : {0.0f, 0.5f, 1.0f}) {
        const float sample = measured[i] + fraction * increment;
        const auto& zone = geo.steer_keepout[i];
        stayed_clear &= std::fabs(robot::WrapPi(sample - zone.center_rad)) > zone.half_width_rad;
      }
      slew_ok &= std::fabs(increment) <= 0.01001f;
    }
    for (int i = 0; i < 4; ++i) measured[i] = robot::WrapPi(out[i].angle);
  }
  CHECK(s.lastSolveValid() && stayed_clear && slew_ok);
  CHECK(out[0].speed < 0.0f && out[2].speed > 0.0f);
  for (const auto& c : out) {
    CHECK(Near(c.speed * std::cos(c.angle), request.vx));
    CHECK(Near(c.speed * std::sin(c.angle), request.vy));
  }

  current_case="a keepout wraps across the encoder seam";
  Geometry seam;
  seam.steer_keepout[2] = {3.10f, 0.18f};
  Rig rig(seam);
  rig.steer[2].feedback(-3.10f);
  rig.update(1.0f, 0.0f);
  CHECK(rig.allZero() && !rig.chassis.driveReady());
  CHECK(rig.chassis.steerKeepoutBlockedMask() == 0x04u);
  CHECK(robot::diag::g_chassis_dbg.steer_keepout_configured_mask == 0x04u);

  current_case="both targets blocked while measured angle is outside";
  Geometry blocked;
  blocked.steer_keepout[0] = {0.0f, 2.0f};
  s.init(blocked, 10.0f, 3.0f);
  measured[0] = pi;
  s.solve({0.0f, 1.0f, 0.0f}, 0.0f, measured, dt, out);
  CHECK(!s.lastSolveValid() && s.keepoutBlockedMask() == 1u);
  for (const auto& c : out) CHECK(c.speed == 0.0f);

  current_case="configured paths require measured angles";
  s.solve({1.0f, 0.0f, 0.0f}, 0.0f, nullptr, dt, out);
  CHECK(!s.lastSolveValid() && s.keepoutBlockedMask() == 1u);
}

void TestSteerKeepoutCoverage() {
  current_case="two local keepouts admit every heading from clear starting angles";
  constexpr int kStartingPositions = 7, kHeadings = 360;
  unsigned attempted = 0, blocked = 0;
  for (const float half_width : {0.20f, 0.5f * pi - 0.015f}) {
    Geometry geometry;
    geometry.steer_keepout[0] = {0.37f, half_width};
    geometry.steer_keepout[2] = {3.10f, half_width};
    ChassisIkSolver solver;
    solver.init(geometry, 10.0f, 3.0f);
    // 以两段畅通圆弧的中心为基准，独立选两轮起点；包含接近两端的起点。
    const float clearance = pi - half_width - 0.004f;
    for (int a = 0; a < kStartingPositions; ++a) {
      for (int b = 0; b < kStartingPositions; ++b) {
        const auto starting = [&](int i, int sample) {
          const float frac = 2.0f * sample / (kStartingPositions - 1) - 1.0f;
          return robot::WrapPi(geometry.steer_keepout[i].center_rad + pi + frac * clearance);
        };
        const float measured[4] = {starting(0, a), -0.20f, starting(2, b), 0.15f};
        for (int heading = 0; heading < kHeadings; ++heading) {
          const float angle = 2.0f * pi * heading / kHeadings;
          for (const float w : {0.0f, 1.5f, -6.0f}) {
            const robot::ChassisMotion motion{2.0f * std::cos(angle), 2.0f * std::sin(angle), w};
            WheelCommand out[4];
            solver.reset(measured);
            solver.solve(motion, 2.83f, measured, dt, out, w != 0.0f);
            ++attempted;
            if (!solver.lastSolveValid() || solver.keepoutBlockedMask() != 0u) ++blocked;
          }
        }
      }
    }
  }
  std::printf("Steer route coverage: %u requests, %u unexpected blocks\n", attempted, blocked);
  CHECK(blocked == 0u);
}

void TestPid() {
  current_case="antiwindup restores actual previous integral after clamp";
  Pid p(10,1,0,1,0.1f);
  CHECK(Near(p.updateError(1,1),1)); CHECK(p.integral()==0);
  CHECK(Near(p.updateError(-1,1),-1)); CHECK(p.integral()==0);

  current_case="steering configured integral output is bounded to 2000 raw";
  Pid* steering=GetPidMotorSteer(0); steering->reset();
  for (int t=0;t<6000;++t) steering->updateError(1,dt);
  CHECK(std::fabs(steering->integral()*steering->ki())<=2000.01f);
  for (int t=0;t<100;++t) steering->updateError(0,dt);
  CHECK(std::fabs(steering->output())<=2000.01f);

  current_case="invalid PID input clears accumulated state";
  for (float bad : {nan,inf,-inf}) {
    Pid q(1,1,1,10,1); q.updateError(1,dt);
    CHECK(q.updateError(bad,dt)==0); CHECK(q.integral()==0); CHECK(q.output()==0);
    CHECK(Near(q.updateError(0,dt),0));
    q.updateError(1,dt); CHECK(q.updateError(1,bad)==0); CHECK(q.integral()==0);
  }
  p.init(1,1,1,10,1); p.updateError(1,dt);
  CHECK(p.updateError(1,0)==0); CHECK(p.integral()==0);
  p.setGains(inf,1,1); CHECK(p.updateError(1,dt)==0);
  current_case="online integral limit decrease cannot roll back above new limit";
  Pid tuning(10,1,0,100,10); tuning.updateError(1,5);
  CHECK(Near(tuning.integral(),5));
  tuning.setOutputLimit(1); tuning.setIntegralLimit(0.1f);
  tuning.updateError(-1,dt); CHECK(std::fabs(tuning.integral()) <= 0.10001f);
}

void TestFiniteBoundary() {
  current_case="one invalid command rejects all actuator outputs";
  for (float bad : {nan,inf,-inf}) {
    Rig r; r.update(1,0); r.update(bad,0); CHECK(r.allZero());
    r.update(0,1); CHECK(r.wheel[0].command==0);
    r.steer[2].feedback(bad); r.update(1,0); CHECK(r.allZero());
  }
  current_case="invalid velocity or timing rejects whole chassis";
  Rig r; r.update(1,0); r.wheel[2].feedback(0,nan); r.update(1,0); CHECK(r.allZero());
  r.wheel[2].feedback(0,0); r.update(1,0); r.update(1,0,0,0); CHECK(r.allZero());
  r.update(1,0); r.update(1,0,0,nan); CHECK(r.allZero());

  current_case="out of range steering feedback fails closed and stays visible after output zero";
  Rig feedback;
  feedback.update(1,0);
  CHECK(robot::diag::g_chassis_dbg.wheel_output[0] > 0);
  feedback.steer[2].feedback(26.65f);
  feedback.update(1,0);
  CHECK(!feedback.chassis.feedbackHealthy() && feedback.allZero());
  CHECK(Near(robot::diag::g_chassis_dbg.steer_feedback[2], 26.65f));
  for (int i = 0; i < 4; ++i) {
    CHECK(robot::diag::g_chassis_dbg.wheel_output[i] == 0);
    CHECK(robot::diag::g_chassis_dbg.steer_output[i] == 0);
  }

  current_case="solver overflow clears result instead of narrowing NaN";
  ChassisIkSolver solver; solver.init({},10,3);
  robot::ChassisMotion m; m.vx=std::numeric_limits<float>::max(); m.vy=m.vx; m.w=m.vx;
  float measured[4]={}; WheelCommand out[4]; solver.solve(m,0,measured,dt,out);
  for (auto& c:out) { CHECK(c.speed==0); CHECK(std::isfinite(c.angle)); }
  Geometry bad; bad.wheel_radius=0;
  r.solver.init(bad,10,3); r.update(1,0); CHECK(r.allZero());
}
}
int main() {
  TestAlignment(); TestDriveGateChatter(); TestStraightAlignment(); TestReturnToMeasuredHeading();
  TestSteerTrajectory();
  TestConfiguredSteerAngleRange(); TestConfiguredSteerFrames(); TestSteerRateDamping();
  TestUniformSaturation(); TestSteerKeepout(); TestSteerKeepoutCoverage(); TestPid(); TestFiniteBoundary();
  current_case="nested critical sections restore the previous IRQ mask";
  __set_PRIMASK(0);
  { robot::CriticalSection outer; CHECK(__get_PRIMASK()==1);
    { robot::CriticalSection inner; CHECK(__get_PRIMASK()==1); }
    CHECK(__get_PRIMASK()==1); }
  CHECK(__get_PRIMASK()==0);
  __set_PRIMASK(1); { robot::CriticalSection locked; }
  CHECK(__get_PRIMASK()==1); __set_PRIMASK(0);
  current_case="remote deadband is continuous bounded and finite";
  CHECK(robot::RemoteAxis(0.001515f)==0); CHECK(robot::RemoteAxis(-0.02f)==0);
  CHECK(robot::RemoteAxis(1)==1); CHECK(robot::RemoteAxis(-2)==-1);
  CHECK(robot::RemoteAxis(std::numeric_limits<float>::quiet_NaN())==0);
  CHECK(robot::RemoteAxis(0.02001f)>0 && robot::RemoteAxis(0.02001f)<0.00002f);
  std::printf("Control regression: %d checks, %d failures\n", checks, failures);
  return failures ? 1 : 0;
}

