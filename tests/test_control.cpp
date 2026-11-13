// 实际 Chassis、逆解、PID、实例参数的回归；仅电机反馈/写入接口为主机替身。
#include <cmath>
#include <cstdio>
#include <limits>
#include "chassis.hpp"
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
  Rig() {
    hello_world::motor::Motor *w[4], *s[4]; Pid *wpp[4], *spp[4];
    solver.init({}, 10, 3);
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
  CHECK(r.wheel[0].command < 0);
  CHECK(r.steer[0].command < 0);

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
  for (const auto& m:turning.wheel) CHECK(m.command != 0);

  // 任一受驱轮失去对齐时，整组撤销驱动并清旧积分，舵角环继续纠偏。
  turning.steer[2].feedback(turning.chassis.wheelCommand(2).desired_angle+0.7f);
  turning.update(0,0,1);
  CHECK(turning.steer[2].command != 0);
  for (int i=0;i<4;++i) {
    CHECK(turning.wheel[i].command == 0);
    CHECK(turning.wp[i].integral() == 0);
  }

  // 超时兜底：持续 300ms 仍对不齐时降级为逐轮放行，已对齐的轮子恢复驱动。
  for (int t=0;t<400;++t) {
    for (int i=0;i<4;++i) {
      const float target=turning.chassis.wheelCommand(i).desired_angle;
      turning.steer[i].feedback(i==2 ? target+0.7f : target);
    }
    turning.update(0,0,1);
  }
  CHECK(turning.chassis.alignTimedOut());
  CHECK(turning.wheel[2].command == 0);
  CHECK(turning.wheel[0].command != 0);
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
  TestAlignment(); TestUniformSaturation(); TestPid(); TestFiniteBoundary();
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
