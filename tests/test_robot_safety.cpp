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
#include "board_wiring.hpp"
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
bool completion_ok = true, cancel_ok = true;
int cancel_requests = 0;
CAN_HandleTypeDef* sent_bus[0x800]={};
int sent_count[0x800]={};
uint32_t failed_id=0x800;
robot::DbusRelay wire_relay;
int right_bias = 0;
bool forwarded_remote=false,relay_enabled=true;
uint8_t left_switch=1;
void (*interrupt_hook)() = nullptr;
int feeds = 0, failures = 0, checks = 0;
uint8_t* dbus_buffer = nullptr;
bool rx_start_ok=true;
int rx_requests=0,rx_capacity=0;
int HAL_CAN_Start(CAN_HandleTypeDef*) { return HAL_OK; }
int HAL_CAN_ActivateNotification(CAN_HandleTypeDef*, int) { return HAL_OK; }
uint32_t HAL_CAN_GetError(const CAN_HandleTypeDef*) { return bus_off ? HAL_CAN_ERROR_BOF : 0; }
int HAL_UARTEx_ReceiveToIdle_DMA(UART_HandleTypeDef*, uint8_t* p, uint16_t size) { ++rx_requests;rx_capacity=size;dbus_buffer=p;return rx_start_ok?HAL_OK:HAL_ERROR; }
int HAL_IWDG_Refresh(IWDG_HandleTypeDef*) { ++feeds; return HAL_OK; }
void CanFilter_Init(CAN_HandleTypeDef*) {}
bool CAN_Is_BusOff(const CAN_HandleTypeDef*) { return bus_off; }
bool CAN_TxHealthy(CAN_HandleTypeDef*) { return completion_ok && !bus_off; }
bool CAN_AbortPendingTx(CAN_HandleTypeDef*) { ++cancel_requests; return cancel_ok; }
bool CAN_Send_Msg(CAN_HandleTypeDef* h, uint8_t* data, uint32_t id, uint8_t) {
  if(id<0x800) {sent_bus[id]=h;++sent_count[id];}
  const bool ok=send_ok && id!=failed_id;
  if(ok && robot::DbusRelay::ownsId(id)) wire_relay.accept(id,data,now_ms);
  return ok;
}
extern "C" void HAL_UARTEx_RxEventCallback(UART_HandleTypeDef*, uint16_t);
extern "C" void HAL_UART_ErrorCallback(UART_HandleTypeDef*);
#define CHECK(expr) do { ++checks; if (!(expr)) { ++failures; std::printf("FAIL line %d: %s\n", __LINE__, #expr); } } while (0)

struct FakeMotor : hello_world::motor::Motor {
  float command = 0;
  bool healthy = true;
  bool isHealthy() const override { return healthy; }
  FakeMotor() { motor_info_.id = 1; }
  bool decode(const uint8_t[8]) override { return true; }
  bool encode(uint8_t[8]) override { return true; }
  hello_world::motor::Status setInput(float v) override { command = v; if (interrupt_hook) { auto hook=interrupt_hook; interrupt_hook=nullptr; hook(); } return hello_world::motor::Status::kOk; }
  void angle(float v) { angle_ = v; }
  void setId(uint8_t value) { motor_info_.id=value; }
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
    const uint16_t v = i < 2 ? uint16_t(1024 + right_bias) : (diagonal ? 1684 : (i == 3 ? 1354 : 1024));
    for (int b = 0; b < 11; ++b) if (v & (1 << b)) dbus_buffer[(11 * i + b) / 8] |= 1 << ((11 * i + b) % 8);
  }
  dbus_buffer[5] |= (left_switch << 6) | ((estop ? 2u : 1u) << 4);
  dbus_buffer[16] = 0; dbus_buffer[17] = 4;
#ifdef TEST_CHASSIS
  if(forwarded_remote) {
    if(relay_enabled) peer.remote_relay().publish(dbus_buffer,now_ms);
    return;
  }
#endif
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
#ifdef TEST_CHASSIS
  peer.gimbal_data().dbus_online=forwarded_remote;
  peer.gimbal_data().estop=forwarded_remote && estop;
  uint32_t relay_id;uint8_t relay_frame[8];
  if(forwarded_remote && relay_enabled && peer.remote_relay().peekTx(now_ms,relay_id,relay_frame)) {
    comm.remote_relay().accept(relay_id,relay_frame,now_ms);peer.remote_relay().txAccepted();
  }
#endif
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
  motors[1].setId(2);
  module.init(&motors[0], &motors[1], &pids[0], &pids[1]); controller.setGimbal(&module);
#endif
  controller.setImu(&imu); controller.setRemoteControl(&rc); controller.setComm(&comm); controller.init();
  rx_start_ok=false;CommTaskInit();
  const int failed_start_requests=rx_requests;
  rx_start_ok=true;CommTask();
  CHECK(rx_requests>failed_start_requests); // 初始化DMA启动失败后必须在控制周期重试
  CHECK(rx_capacity==36); // 给IDLE边界留出余量，18字节不等于DMA传输完成
  const int recovered_requests=rx_requests;
  HAL_UART_ErrorCallback(&huart3);CommTask();CHECK(rx_requests>recovered_requests);
  UART_HandleTypeDef unrelated_uart;
  const int unrelated_requests=rx_requests;
  HAL_UARTEx_RxEventCallback(&unrelated_uart,18);CHECK(rx_requests==unrelated_requests);
  fake_rx_event=HAL_UART_RXEVENT_HT;
  HAL_UARTEx_RxEventCallback(&huart3,18);CHECK(rx_requests==unrelated_requests);
  fake_rx_event=HAL_UART_RXEVENT_IDLE;
  CommTask();
  ++now_ms;CommTask();
#ifdef TEST_CHASSIS
  CHECK(sent_bus[0x200]==robot::wiring::Can(robot::wiring::kChassisWheel));
  CHECK(sent_bus[0x1FF]==robot::wiring::Can(robot::wiring::kChassisSteer));
  CHECK(sent_bus[comm.tx_id()]==robot::wiring::Can(robot::wiring::kChassisPeer));
#else
  CHECK(sent_bus[1]==robot::wiring::Can(robot::wiring::kGimbalYaw));
#endif
  const int status_before=sent_count[comm.tx_id()];
#ifdef TEST_CHASSIS
  const int motor_before=sent_count[0x200];
#else
  const int motor_before=sent_count[1];
#endif
  for(int i=0;i<10;++i) {++now_ms;CommTask();}
  CHECK(sent_count[comm.tx_id()]-status_before==2); // 状态200Hz，不占用1kHz电机总线预算
#ifdef TEST_CHASSIS
  CHECK(sent_count[0x200]-motor_before==5);
#else
  CHECK(sent_count[1]-motor_before==5);
#endif
#ifdef TEST_CHASSIS
  // 任务书的实际架构：只有云台接收机，底盘也必须能完成三模式控制。
  forwarded_remote=true;
  for(int i=0;i<15;++i) Step(true);
  for(int i=0;i<650;++i) Step();
  CHECK(controller.lifeState()==robot::LifeState::kWorking);
  CHECK(controller.isRemoteOnline());CHECK(!OutputsZero());
  left_switch=3;for(int i=0;i<6;++i) Step();CHECK(controller.chassisMode()==robot::ChassisMode::kFollow);
  left_switch=2;for(int i=0;i<6;++i) Step();CHECK(controller.chassisMode()==robot::ChassisMode::kSpin);
  Step(true);CHECK(OutputsZero());
  for(int i=0;i<650;++i) Step();
  relay_enabled=false;for(int i=0;i<110;++i) Step();
  CHECK(controller.lifeState()==robot::LifeState::kDead);CHECK(OutputsZero());
  const int stale_feeds=feeds;Step();CHECK(feeds==stale_feeds); // G2C在线位不能为冻结摇杆续命
  relay_enabled=true;for(int i=0;i<650;++i) Step();CHECK(OutputsZero());
  for(int i=0;i<15;++i) { Step(true); }
  for(int i=0;i<650;++i) { Step(); }
  CHECK(controller.lifeState()==robot::LifeState::kWorking);
  // 更换遥控来源必须重新急停确认，不能让本地新出现的正常档接管。
  forwarded_remote=false;left_switch=1;
  for(int i=0;i<110;++i) { Step(); }
  CHECK(OutputsZero());
  Step(true);for(int i=0;i<650;++i) Step();CHECK(controller.lifeState()==robot::LifeState::kWorking);
  comm=robot::GimbalChassisComm(role,0x210,0x211);peer=robot::GimbalChassisComm(peer_role,0x211,0x210);
  controller.init();
  for (int i = 0; i < 650; ++i) Step();
  CHECK(OutputsZero()); CHECK(controller.lifeState() == robot::LifeState::kDead);
  Step(true); // 上电必须由有效遥控急停确认，防止看门狗重启后自动带指令起动。
#endif
  for (int i = 0; i < 650; ++i) Step();
  CHECK(controller.lifeState() == robot::LifeState::kWorking); CHECK(!OutputsZero()); CHECK(feeds > 0);
#ifndef TEST_CHASSIS
  CHECK(wire_relay.online(now_ms));CHECK(std::memcmp(wire_relay.frame(),dbus_buffer,18)==0);
#endif
#ifdef TEST_CHASSIS
  while(now_ms%2u!=0u) {Step();}
  const int stopped_count=sent_count[0x200];
#else
  while(now_ms%2u!=1u) {Step();}
  const int stopped_count=sent_count[1];
#endif
  Step(true);CHECK(OutputsZero()); // 停机沿必须立即发送，不等待500Hz的下一时隙
#ifdef TEST_CHASSIS
  CHECK(sent_count[0x200]>stopped_count);
#else
  CHECK(sent_count[1]>stopped_count);
#endif
  Step();
#ifdef TEST_CHASSIS
  failed_id=0x1FF; // 只让舵电机提交失败，轮电机和板间帧仍成功
  for(int i=0;i<14;++i) {Step();}
  const int steer_fault_feeds=feeds;Step();CHECK(feeds==steer_fault_feeds);
  failed_id=0x800;
  for(int i=0;i<110;++i) {Step(true);}
  for(int i=0;i<650;++i) {Step();}
  CHECK(controller.lifeState()==robot::LifeState::kWorking);
#endif
  send_ok = false;
  Step();Step(); // HAL拒绝提交最迟在下一个2ms电机时隙被发现
  CHECK(OutputsZero()); CHECK(controller.lifeState() == robot::LifeState::kDead);
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
#ifdef TEST_CHASSIS
  right_bias = 1;
  Step(); const float pitch_before = comm.chassis_data().pitch_target;
  for(int i=0;i<100;++i) Step();
  CHECK(comm.chassis_data().yaw_rate == 0.0f);
  CHECK(std::fabs(comm.chassis_data().pitch_target-pitch_before)<0.000001f);
  right_bias = 0;
#else
  Step();
  // 模拟控制下发过程中UART抢占：本周期必须保持旧快照，下周期处理新急停。
  interrupt_hook = []() { Remote(true); };
  Step(); CHECK(!comm.gimbal_data().estop);
  controller.update(); CommTask(); CHECK(OutputsZero()); CHECK(controller.isEstopped());
#endif
#ifndef TEST_CHASSIS
  // 首次恢复输出时清PID历史也必须保留本周期反馈快照。
  motors[1].angle(0); Step(true);
  interrupt_hook=[]() { motors[1].angle(0.4f); };
  Step(); CHECK(std::fabs(motors[1].command)<0.000001f);
  motors[1].angle(0);
#endif
  Step(); motors[0].healthy = false; Step();
  CHECK(controller.lifeState() == robot::LifeState::kDead); CHECK(OutputsZero());
  motors[0].healthy = true;
  for(int i=0;i<110;++i) Step(true);
  for(int i=0;i<650;++i) Step();
  const int canceled_before=cancel_requests;
  Step(true); CHECK(OutputsZero()); CHECK(cancel_requests>canceled_before);
  Step();
  cancel_ok=false; Step(true); CHECK(OutputsZero());
  CHECK(controller.lifeState()==robot::LifeState::kDead);
  const int retry_before=cancel_requests; Step(true); CHECK(cancel_requests>retry_before);
  cancel_ok=true; Step(true); const int finished=cancel_requests; Step(true);
  CHECK(cancel_requests==finished); // 撤销完成后不能每周期取消新的零帧
  for(int i=0;i<110;++i) Step(true);
  for(int i=0;i<650;++i) { Step(); }
  CHECK(controller.lifeState()==robot::LifeState::kWorking);
  completion_ok=false; Step(); CHECK(OutputsZero()); CHECK(controller.lifeState()==robot::LifeState::kDead);
  completion_ok=true;
  for(int i=0;i<650;++i) { Step(); }
  CHECK(OutputsZero());
  bus_off = true; Step(); CHECK(OutputsZero());
  std::printf("Robot safety: %d checks, %d failures\n", checks, failures);
  return failures ? 1 : 0;
}
