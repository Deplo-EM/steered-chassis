// 主机集成测试：真实 Robot/CommTask/协议/运动模块；只替代硬件和时间。
#include <cmath>
#include <cstdio>
#include <cstring>
#include <limits>
#include "host_hal.hpp"
#include "imu.hpp"
#include "ins_all.hpp"
#include "comm_task.hpp"
#include "pid.hpp"
#ifdef TEST_CHASSIS
#include "chassis.hpp"
#else
#include "gimbal.hpp"
#endif

uint32_t now_ms = 0;
namespace hello_world::tick {
uint64_t GetTickUs() { return uint64_t(now_ms) * 1000; }
uint32_t GetTickMs() { return now_ms; }
uint32_t GetTickS() { return now_ms / 1000; }
void DelayUs(uint32_t) {}
}
CAN_HandleTypeDef hcan1, hcan2;
UART_HandleTypeDef huart3;
IWDG_HandleTypeDef hiwdg;
bool send_ok = true, bus_off = false;
bool peer_working = true;
int feeds = 0, failures = 0, checks = 0;
uint8_t* dbus_buffer = nullptr;
int HAL_CAN_Start(CAN_HandleTypeDef*) { return HAL_OK; }
int HAL_CAN_ActivateNotification(CAN_HandleTypeDef*, int) { return HAL_OK; }
uint32_t HAL_CAN_GetError(const CAN_HandleTypeDef*) { return bus_off ? HAL_CAN_ERROR_BOF : 0; }
int HAL_UARTEx_ReceiveToIdle_DMA(UART_HandleTypeDef*, uint8_t* p, uint16_t) { dbus_buffer = p; return HAL_OK; }
int HAL_IWDG_Refresh(IWDG_HandleTypeDef*) { ++feeds; return HAL_OK; }
void CanFilter_Init(CAN_HandleTypeDef*) {}
bool CAN_Is_BusOff(const CAN_HandleTypeDef*) { return bus_off; }
bool CAN_Send_Msg(CAN_HandleTypeDef*, uint8_t*, uint32_t, uint8_t) { return send_ok; }
extern "C" void HAL_UARTEx_RxEventCallback(UART_HandleTypeDef*, uint16_t);
#define CHECK(expr) do { ++checks; if (!(expr)) { ++failures; std::printf("FAIL line %d: %s\n", __LINE__, #expr); } } while (0)

struct FakeMotor : hello_world::motor::Motor {
  float command = 0;
  FakeMotor() { motor_info_.id = 1; }
  bool decode(const uint8_t[8]) override { return true; }
  bool encode(uint8_t[8]) override { return true; }
  hello_world::motor::Status setInput(float v) override { command = v; return hello_world::motor::Status::kOk; }
  void angle(float v) { angle_ = v; }
};
FakeMotor motors[8];
hello_world::remote_control::DT7 rc;
hello_world::imu::Imu imu;
robot::Robot controller;
#ifdef TEST_CHASSIS
constexpr auto role = robot::GimbalChassisComm::CodePart::Chassis;
constexpr auto peer_role = robot::GimbalChassisComm::CodePart::Gimbal;
robot::Chassis module;
hello_world::chassis_ik_solver::ChassisIkSolver solver;
#else
constexpr auto role = robot::GimbalChassisComm::CodePart::Gimbal;
constexpr auto peer_role = robot::GimbalChassisComm::CodePart::Chassis;
robot::Gimbal module;
#endif
robot::GimbalChassisComm comm(role, 0x210, 0x211), peer(peer_role, 0x211, 0x210);
hello_world::motor::Motor* GetMotorWheel(int i) { return &motors[i]; }
hello_world::motor::Motor* GetMotorSteer(int i) { return &motors[i + 4]; }
hello_world::motor::Motor* GetMotorYaw() { return &motors[0]; }
hello_world::motor::Motor* GetMotorPitch() { return &motors[1]; }
hello_world::remote_control::DT7* GetRemoteControl() { return &rc; }
robot::Robot* GetRobot() { return &controller; }
robot::GimbalChassisComm* GetGimbalChassisComm() { return &comm; }

void Remote(bool estop, bool diagonal = false) {
  std::memset(dbus_buffer, 0, 18);
  for (int i = 0; i < 4; ++i) {
    const uint16_t v = diagonal && i >= 2 ? 1684 : (i == 3 ? 1354 : 1024);
    for (int b = 0; b < 11; ++b) if (v & (1 << b)) dbus_buffer[(11 * i + b) / 8] |= 1 << ((11 * i + b) % 8);
  }
  dbus_buffer[5] |= (1u << 6) | ((estop ? 2u : 1u) << 4);
  dbus_buffer[16] = 0; dbus_buffer[17] = 4;
  HAL_UARTEx_RxEventCallback(&huart3, 18);
}
void Step(bool estop = false, bool diagonal = false) {
  ++now_ms;
  Remote(estop, diagonal);
  for (int i = 0; i < controller.kMotorSlotNum; ++i) controller.notifyMotorRx(i);
  peer.chassis_data().working = peer.chassis_data().gimbal_enable = peer.chassis_data().dbus_online = true;
  peer.chassis_data().imu_ready = true;
  peer.chassis_data().estop = estop;
  peer.chassis_data().yaw_rate = 1.0f;
  peer.gimbal_data().working = peer_working;
  peer.gimbal_data().imu_ready = true;
  uint8_t frame[8]; peer.encode(frame); comm.decode(frame);
  controller.update(); CommTask();
}
bool OutputsZero() {
  for (int i = 0; i < controller.kMotorSlotNum; ++i) if (motors[i].command != 0) return false;
  return true;
}
int main() {
  hello_world::pid::Pid pids[8];
  for (auto& p : pids) p.init(1.0f, 0.1f, 0.0f, 8.0f, 1.0f);
#ifdef TEST_CHASSIS
  hello_world::motor::Motor *wp[4], *sp[4]; hello_world::pid::Pid *wpp[4], *spp[4];
  for (int i = 0; i < 4; ++i) { wp[i] = &motors[i]; sp[i] = &motors[i+4]; wpp[i] = &pids[i]; spp[i] = &pids[i+4]; }
  solver.init({}, 10.0f, 3.0f); module.init(wp, sp, wpp, spp, &solver); controller.setChassis(&module);
#else
  module.init(&motors[0], &motors[1], &pids[0], &pids[1]); controller.setGimbal(&module);
#endif
  controller.setImu(&imu); controller.setRemoteControl(&rc); controller.setComm(&comm); controller.init(); CommTaskInit();
#ifdef TEST_CHASSIS
  for (int i = 0; i < 650; ++i) Step();
  CHECK(OutputsZero()); CHECK(controller.lifeState() == robot::LifeState::kDead);
  Step(true); // 上电必须由有效遥控急停确认，防止看门狗重启后自动带指令起动。
#endif
  for (int i = 0; i < 650; ++i) Step();
  CHECK(controller.lifeState() == robot::LifeState::kWorking); CHECK(!OutputsZero()); CHECK(feeds > 0);
  send_ok = false;
  Step(); CHECK(OutputsZero()); CHECK(controller.lifeState() == robot::LifeState::kDead);
  for (int i = 0; i < 12; ++i) Step();
  const int feeds_before = feeds;
  Step(); CHECK(feeds == feeds_before);
  send_ok = true;
  for (int i = 0; i < 700; ++i) Step();
  CHECK(OutputsZero()); CHECK(controller.lifeState() == robot::LifeState::kDead);
  // 仅有效急停状态下持续成功发送，才允许解除锁存。
  for (int i = 0; i < 110; ++i) Step(true);
  for (int i = 0; i < 650; ++i) Step();
  CHECK(controller.lifeState() == robot::LifeState::kWorking); CHECK(!OutputsZero());
  imu.sample_valid = false; Step();
  CHECK(OutputsZero()); CHECK(controller.lifeState() == robot::LifeState::kDead);
  imu.sample_valid = true;
  for (int i = 0; i < 650; ++i) Step();
#ifdef TEST_CHASSIS
  CHECK(OutputsZero()); CHECK(controller.lifeState() == robot::LifeState::kDead);
  Step(true); for (int i = 0; i < 650; ++i) Step();
#endif
  CHECK(controller.lifeState() == robot::LifeState::kWorking);
#ifdef TEST_CHASSIS
  // 云台单独复位/掉线后返回，底盘仍不能保留先前授权自行起动。
  peer_working = false; Step(); CHECK(OutputsZero());
  peer_working = true;
  for (int i = 0; i < 650; ++i) Step();
  CHECK(OutputsZero()); CHECK(controller.lifeState() == robot::LifeState::kDead);
  Step(true); for (int i = 0; i < 650; ++i) Step();
  CHECK(controller.lifeState() == robot::LifeState::kWorking);
  for (int i = 0; i < 500; ++i) Step(false, true);
  CHECK(std::hypot(controller.motion().vx, controller.motion().vy) <= 2.00001f);
#else
  // C2G 急停不能原样回显到 G2C，否则双方 OR 后无法解除急停。
  for (int i = 0; i < 4000; ++i) Step();
  CHECK(std::fabs(module.yawTarget()) <= robot::kPi);
  Step(true); CHECK(comm.gimbal_data().estop); // 本地急停仍必须上报。
  peer.chassis_data().estop = true;
  Remote(false); uint8_t f[8]; peer.encode(f); comm.decode(f); controller.update();
  CHECK(controller.isEstopped()); CHECK(!comm.gimbal_data().estop);
  // 单个轴的坏反馈必须使整个云台输出清零。
  module.setYawWorldTarget(1.0f); module.setPitchTarget(0.2f);
  module.update(0.001f, std::numeric_limits<float>::quiet_NaN());
  CHECK(OutputsZero());
  module.setPitchTarget(std::numeric_limits<float>::infinity());
  module.update(0.001f, 0.0f); CHECK(OutputsZero());
  // 角度到达软件边界后，旧积分不得继续向边界外施加力矩。
  module.stop(); module.setYawWorldTarget(0); module.setPitchTarget(0.7f);
  for (int i = 0; i < 2000; ++i) module.update(0.001f, 0.0f);
  motors[1].angle(0.7f); module.update(0.001f, 0.0f);
  CHECK(motors[1].command <= 0);
  // 不一致的对板使能不能绕过 DBUS/Working 资格。
  peer.chassis_data().estop = false;
  peer.chassis_data().dbus_online = false;
  peer.chassis_data().gimbal_enable = true;
  peer.encode(f); comm.decode(f); controller.update(); CHECK(OutputsZero());
#endif
  bus_off = true; Step(); CHECK(OutputsZero());
  std::printf("Robot safety: %d checks, %d failures\n", checks, failures);
  return failures ? 1 : 0;
}
