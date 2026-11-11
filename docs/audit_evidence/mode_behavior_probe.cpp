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
unsigned requested_left_switch=1;
float peer_yaw=0.4f;
int feeds = 0, failures = 0, checks = 0;
uint8_t* dbus_buffer = nullptr;
int HAL_CAN_Start(CAN_HandleTypeDef*) { return HAL_OK; }
int HAL_CAN_ActivateNotification(CAN_HandleTypeDef*, int) { return HAL_OK; }
uint32_t HAL_CAN_GetError(const CAN_HandleTypeDef*) { return bus_off ? HAL_CAN_ERROR_BOF : 0; }
int HAL_UARTEx_ReceiveToIdle_DMA(UART_HandleTypeDef*, uint8_t* p, uint16_t) { dbus_buffer = p; return HAL_OK; }
int HAL_IWDG_Refresh(IWDG_HandleTypeDef*) { ++feeds; return HAL_OK; }
void CanFilter_Init(CAN_HandleTypeDef*) {}
bool CAN_Is_BusOff(const CAN_HandleTypeDef*) { return bus_off; }
bool CAN_TxHealthy(CAN_HandleTypeDef*) { return !bus_off; }
bool CAN_AbortPendingTx(CAN_HandleTypeDef*) { return true; }
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
    const uint16_t v = 1024;
    (void)diagonal;
    for (int b = 0; b < 11; ++b) if (v & (1 << b)) dbus_buffer[(11 * i + b) / 8] |= 1 << ((11 * i + b) % 8);
  }
  dbus_buffer[5] |= (requested_left_switch << 6) | ((estop ? 2u : 1u) << 4);
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
  peer.gimbal_data().yaw_joint = peer_yaw;
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
  for (int i=0;i<650;++i) Step(true);
  struct Case { unsigned sw; float delta; robot::ChassisMode mode; float w; };
  const Case cases[] = {
    {1,0.4f,robot::ChassisMode::kSeparate,0.0f},
    {3,0.4f,robot::ChassisMode::kFollow,1.6f},
    {3,-0.4f,robot::ChassisMode::kFollow,-1.6f},
    {2,0.0f,robot::ChassisMode::kSpin,1.5f}
  };
  for (const auto& c:cases) {
    requested_left_switch=c.sw; peer_yaw=c.delta;
    for (int i=0;i<500;++i) Step();
    std::printf("switch=%u mode=%u delta=%.3f w=%.3f expected=%.3f\n",c.sw,unsigned(controller.chassisMode()),c.delta,controller.motion().w,c.w);
    CHECK(controller.chassisMode()==c.mode);
    CHECK(std::fabs(controller.motion().w-c.w)<0.01f);
  }
  std::printf("Mode audit: %d checks, %d failures\n",checks,failures);
  return failures ? 1 : 0;
}
