// 主机集成测试：真实 Robot/CommTask/协议/运动模块；只替代硬件和时间。
#include <cmath>
#include <cstdio>
#include <cstring>
#include <limits>
#include "host_hal.hpp"
#include "imu.hpp"
#include "ins_all.hpp"
#include "comm_task.hpp"
#include "can_tx_receipt.hpp"
#include "pid.hpp"
#include "board_wiring.hpp"
#if defined(ROBOT_GROUND_TRIAL)
#include "crc8.hpp"
bool ground_peer_disarmed = false;
uint8_t ground_run_switch = 3;
int ground_wheel_live_frames = 0;
int ground_steer_live_frames = 0;
#endif
#if defined(ROBOT_STEER_MODE_PROBE) && ROBOT_STEER_MODE_PROBE
#include "steer_mode_probe.hpp"
namespace robot::probe {
extern volatile Result g_steer_probe_voltage, g_steer_probe_current;
}
int probe_nonzero[0x800]={};
int probe_zero[0x800]={};
bool probe_bad_slots=false;
bool probe_reject=false;
bool neutral_left=true;
#endif
#ifdef TEST_CHASSIS
robot::CanTxReceipt* pending_receipt=nullptr;
#endif
#ifdef TEST_CHASSIS
#include "chassis.hpp"
#include "steer_output_profile.hpp"
#if !defined(ROBOT_STEER_MODE_PROBE) || !ROBOT_STEER_MODE_PROBE
#include "steer_stall_tx.hpp"
#endif
#else
#include "gimbal.hpp"
#include "gimbal_debug.hpp"
#include "ins_pid.hpp"
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
#ifdef TEST_CHASSIS
bool last_wheel_frame_zero = true, last_steer_frame_zero = true;
#endif
#ifndef TEST_CHASSIS
// 模拟驱动收到使能/失能命令后的反馈；按 CAN 帧验收，不能只看 PID 归零。
int dm_enable_frames = 0, dm_disable_frames = 0;
uint8_t dm_last_cmd[2] = {}, dm_pending_reply[2] = {};
bool dm_ack_disable = true;
bool dm_ack_enable[2] = {true, true};
CAN_HandleTypeDef* dm_cancel_fail_bus = nullptr;
int dm_cancel_calls[2] = {};
#endif
uint32_t failed_id=0x800;
uint32_t busy_status_id=0x800;
bool peer_imu_ready = true;
bool deliver_peer_status = true;
#ifndef TEST_CHASSIS
bool limit_peer_slot = false;
int peer_slot_attempts = 0;
#endif
robot::DbusRelay wire_relay;
int right_bias = 0;
bool forwarded_remote=false,relay_enabled=true;
uint8_t left_switch=1;
#ifdef TEST_CHASSIS
uint16_t left_h = 1024, left_v = 1354, wheel_raw = 1024;
#endif
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
bool CAN_TxHealthy(CAN_HandleTypeDef* bus) {
#ifdef TEST_CHASSIS
  if (bus == &hcan1 && pending_receipt) {
    pending_receipt->status = completion_ok && !bus_off ? robot::CanTxStatus::kSent
                                                       : robot::CanTxStatus::kUnconfirmed;
    pending_receipt = nullptr;
  }
#else
  (void)bus;
#endif
  return completion_ok && !bus_off;
}
bool CAN_AbortPendingTx(CAN_HandleTypeDef* bus) {
  ++cancel_requests;
#ifndef TEST_CHASSIS
  ++dm_cancel_calls[bus == &hcan1 ? 0 : 1];
  if (bus == dm_cancel_fail_bus) return false;
#else
  (void)bus;
#endif
  return cancel_ok;
}
bool CAN_Send_Msg(CAN_HandleTypeDef* h, uint8_t* data, uint32_t id, uint8_t) {
  if(id<0x800) {sent_bus[id]=h;++sent_count[id];}
#ifdef TEST_CHASSIS
  if (id == 0x200u || id == 0x1FEu || id == 0x1FFu) {
    bool zero = true;
    for (int i = 0; i < 8; ++i) zero &= data[i] == 0;
    if (id == 0x200u) last_wheel_frame_zero = zero;
    else last_steer_frame_zero = zero;
  }
#endif
#if defined(ROBOT_GROUND_TRIAL)
  if (id == 0x200u) {
    for (int i = 0; i < 8; ++i) {
      if (data[i]) { ++ground_wheel_live_frames; break; }
    }
  }
  if (id == 0x1FEu) {
    for (int i = 0; i < 8; ++i) {
      if (data[i]) { ++ground_steer_live_frames; break; }
    }
  }
#endif
#if defined(ROBOT_STEER_MODE_PROBE) && ROBOT_STEER_MODE_PROBE
  if (id==0x1FF || id==0x1FE || id==0x200) {
    bool nonzero=false;
    for (int i=0;i<8;++i) if (data[i]!=0) {
      nonzero=true;
      if (id==0x200 || (i!=2 && i!=3)) probe_bad_slots=true;
    }
    ++(nonzero ? probe_nonzero[id] : probe_zero[id]);
  }
#endif
  bool ok=send_ok && id!=failed_id && id!=busy_status_id;
#ifndef TEST_CHASSIS
  if (limit_peer_slot && (id == robot::kCommIdGimbalToChassis || robot::DbusRelay::ownsId(id)))
    ok = ++peer_slot_attempts == 1 && ok;
  if (ok && (id == 1u || id == 2u)) {
    bool special = true;
    for (int i = 0; i < 7; ++i) special = special && data[i] == 0xFFu;
    const uint8_t cmd = special ? data[7] : 0u;
    dm_last_cmd[id - 1u] = cmd;
    if (cmd == 0xFCu) ++dm_enable_frames;
    if (cmd == 0xFDu) ++dm_disable_frames;
    if ((cmd == 0xFCu && dm_ack_enable[id - 1u]) || (cmd == 0xFDu && dm_ack_disable)) {
      dm_pending_reply[id - 1u] = cmd;
    }
  }
#endif
  if(ok && robot::DbusRelay::ownsId(id)) wire_relay.accept(id,data,now_ms);
  return ok;
}

robot::CanTxSubmitResult CAN_TrySend(CAN_HandleTypeDef* h, const uint8_t* data, uint32_t id,
                                   uint8_t len)
{
  using robot::CanTxSubmitResult;
  uint8_t frame[8];
  std::memcpy(frame, data, len);
  if (CAN_Send_Msg(h, frame, id, len)) return CanTxSubmitResult::kQueued;
  bool busy = id == busy_status_id;
#ifndef TEST_CHASSIS
  busy = busy || (limit_peer_slot && id == robot::kCommIdGimbalToChassis && peer_slot_attempts > 1);
#endif
  return send_ok && id != failed_id && busy
      ? CanTxSubmitResult::kBusy : CanTxSubmitResult::kFault;
}
#ifdef TEST_CHASSIS
bool CAN_SendTracked(CAN_HandleTypeDef* h, const uint8_t* data, uint32_t id, uint8_t len,
                     robot::CanTxReceipt& receipt) {
#if defined(ROBOT_STEER_MODE_PROBE) && ROBOT_STEER_MODE_PROBE
  if (probe_reject) return false;
#endif
  uint8_t frame[8];
  std::memcpy(frame,data,8);
  if (!CAN_Send_Msg(h,frame,id,len)) return false;
  receipt.status=robot::CanTxStatus::kPending;
  pending_receipt=&receipt;
  return true;
}
#endif
extern "C" void HAL_UARTEx_RxEventCallback(UART_HandleTypeDef*, uint16_t);
extern "C" void HAL_UART_ErrorCallback(UART_HandleTypeDef*);
#define CHECK(expr) do { ++checks; if (!(expr)) { ++failures; std::printf("FAIL line %d: %s\n", __LINE__, #expr); } } while (0)

struct FakeMotor : hello_world::motor::Motor {
  float command = 0;
  bool healthy = true;
  bool drive_enabled = true;
#ifndef TEST_CHASSIS
  bool enforce_torque_limit = false;
#endif
  bool isHealthy() const override { return healthy && drive_enabled; }
  FakeMotor() {
    motor_info_.id = 1;
#ifndef TEST_CHASSIS
    motor_info_.torq_input_lim = 3.5f;
#endif
  }
  bool decode(const uint8_t[8]) override { return true; }
  bool encode(uint8_t data[8]) override {
#ifndef TEST_CHASSIS
    if (!drive_enabled && healthy) {
      std::memset(data, 0xFF, 7);
      data[7] = 0xFC;
      return true;
    }
#endif
    float wire_command = command;
#if defined(ROBOT_GROUND_TRIAL)
  if (!steer_slot) wire_command *= 16384.0f / 20.0f;
#endif
#if defined(TEST_CHASSIS) && defined(ROBOT_STEER_CURRENT_FIELD) && ROBOT_STEER_CURRENT_FIELD
    if (this->steer_slot) wire_command *= 16384.0f / 3.0f;
#endif
    const auto raw = static_cast<uint16_t>(static_cast<int16_t>(wire_command));
    const int i = (motor_info_.id - 1) % 4;
    data[2*i] = static_cast<uint8_t>(raw >> 8);
    data[2*i+1] = static_cast<uint8_t>(raw);
    return true;
  }
  hello_world::motor::Status setInput(float v) override {
#ifndef TEST_CHASSIS
    if (enforce_torque_limit && std::fabs(v) > motor_info_.torq_input_lim) {
      command = std::copysign(motor_info_.torq_input_lim, v);
      return hello_world::motor::Status::kInputValueOverflow;
    }
#endif
    command = v;
    if (interrupt_hook) {
      auto hook=interrupt_hook; interrupt_hook=nullptr; hook();
    }
    return hello_world::motor::Status::kOk;
  }
  void angle(float v) { angle_ = v; }
  void velocity(float v) { vel_ = v; }
  void setId(uint8_t value) { motor_info_.id=value; }
#ifdef TEST_CHASSIS
  bool steer_slot = false;
#endif
};
FakeMotor motors[8];
hello_world::remote_control::DT7 rc;
hello_world::imu::Imu imu;
robot::Robot controller;
#ifdef TEST_CHASSIS
constexpr uint16_t kSteerTxId = robot::steer_output::kUseCurrent ? 0x1FEu : 0x1FFu;
constexpr auto role = robot::GimbalChassisComm::CodePart::Chassis;
constexpr auto peer_role = robot::GimbalChassisComm::CodePart::Gimbal;
robot::Chassis module;
hello_world::chassis_ik_solver::ChassisIkSolver solver;
robot::Chassis* GetChassis() { return &module; }
#else
constexpr auto role = robot::GimbalChassisComm::CodePart::Gimbal;
constexpr auto peer_role = robot::GimbalChassisComm::CodePart::Chassis;
robot::Gimbal module;
robot::GimbalDriveFeedback GetGimbalDriveFeedback() {
  return {motors[0].healthy && motors[1].healthy,
          static_cast<uint8_t>(unsigned(!motors[0].drive_enabled) |
                               (unsigned(!motors[1].drive_enabled) << 1u))};
}
#endif
constexpr uint32_t kStatusTxId = role == robot::GimbalChassisComm::CodePart::Chassis
    ? robot::kCommIdChassisToGimbal : robot::kCommIdGimbalToChassis;
constexpr uint32_t kStatusRxId = peer_role == robot::GimbalChassisComm::CodePart::Chassis
    ? robot::kCommIdChassisToGimbal : robot::kCommIdGimbalToChassis;
robot::GimbalChassisComm comm(role, kStatusTxId, kStatusRxId),
    peer(peer_role, kStatusRxId, kStatusTxId);
hello_world::motor::Motor* GetMotorWheel(int i) { return &motors[i]; }
hello_world::motor::Motor* GetMotorSteer(int i) {
#ifdef TEST_CHASSIS
  motors[i + 4].steer_slot = true;
  if constexpr (robot::steer_output::kUseCurrent)
    motors[i + 4].set_input_type(hello_world::motor::InputType::kCurr);
#endif
  return &motors[i + 4];
}
hello_world::motor::Motor* GetMotorYaw() { return &motors[0]; }
hello_world::motor::Motor* GetMotorPitch() { return &motors[1]; }
hello_world::remote_control::DT7* GetRemoteControl() { return &rc; }
robot::Robot* GetRobot() { return &controller; }
robot::GimbalChassisComm* GetGimbalChassisComm() { return &comm; }

void Remote(bool estop, bool diagonal = false) {
  std::memset(dbus_buffer, 0, 18);
  for (int i = 0; i < 4; ++i) {
    uint16_t v = i < 2 ? uint16_t(1024 + right_bias) : (diagonal ? 1684 : (i == 3 ? 1354 : 1024));
#ifdef TEST_CHASSIS
    if (!diagonal && i >= 2) v = i == 2 ? left_h : left_v;
#endif
#if defined(ROBOT_STEER_MODE_PROBE) && ROBOT_STEER_MODE_PROBE
    if (i == 3 && neutral_left) v=1024;
#endif
    for (int b = 0; b < 11; ++b) if (v & (1 << b)) dbus_buffer[(11 * i + b) / 8] |= 1 << ((11 * i + b) % 8);
  }
#if defined(ROBOT_STEER_MODE_PROBE) && ROBOT_STEER_MODE_PROBE
  dbus_buffer[5] |= (left_switch << 6) | ((estop ? 2u : 3u) << 4);
#elif defined(ROBOT_GROUND_TRIAL)
  dbus_buffer[5] |= (left_switch << 6) | ((estop ? 2u : ground_run_switch) << 4);
#else
  dbus_buffer[5] |= (left_switch << 6) | ((estop ? 2u : 1u) << 4);
#endif
  dbus_buffer[16] = 0; dbus_buffer[17] = 4;
#ifdef TEST_CHASSIS
  dbus_buffer[16] = static_cast<uint8_t>(wheel_raw);
  dbus_buffer[17] = static_cast<uint8_t>(wheel_raw >> 8);
  if(forwarded_remote) {
    if(relay_enabled) peer.remote_relay().publish(dbus_buffer,now_ms);
    return;
  }
#endif
  HAL_UARTEx_RxEventCallback(&huart3, 18);
}
void Step(bool estop = false, bool diagonal = false, bool peer_knows_estop = true) {
  ++now_ms;
#ifndef TEST_CHASSIS
  for (int i = 0; i < 2; ++i) {
    if (dm_pending_reply[i]) motors[i].drive_enabled = dm_pending_reply[i] == 0xFCu;
    dm_pending_reply[i] = 0;
  }
#endif
  Remote(estop, diagonal);
  for (int i = 0; i < controller.kMotorSlotNum; ++i) controller.notifyMotorRx(i);
  peer.chassis_data().working = peer.chassis_data().gimbal_enable = peer.chassis_data().dbus_online = true;
  peer.chassis_data().imu_ready = peer_imu_ready;
  peer.chassis_data().estop = estop && peer_knows_estop;
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
  uint8_t frame[8]; peer.encode(frame);
#if defined(ROBOT_GROUND_TRIAL)
  // 独立构造配对云台发来的字节，不用底盘解码器生成预期位。
  if (ground_peer_disarmed) {
    frame[0] |= 0x40u;
    hello_world::tools::crc8_check crc;
    frame[7] = crc.getCrc8CheckNum(frame, 7, 0);
  }
#endif
  if (deliver_peer_status) comm.decode(frame);
  controller.update(); CommTask();
}
bool OutputsZero() {
  for (int i = 0; i < controller.kMotorSlotNum; ++i) if (motors[i].command != 0) return false;
  return true;
}

void TestRoutinePeerSendBusy()
{
#ifdef TEST_CHASSIS
  constexpr uint32_t phase = 0;
#else
  constexpr uint32_t phase = 2;
#endif
  while ((now_ms + 1u) % 5u != phase) Step();
  const auto before = sent_count[comm.tx_id()];
  const int before_abort = cancel_requests;
  busy_status_id = comm.tx_id();
  Step();
  CHECK(sent_count[comm.tx_id()] == before + 1);
  CHECK(controller.lifeState() == robot::LifeState::kWorking && controller.outputPermitted());
  CHECK(!OutputsZero() && cancel_requests == before_abort);
  busy_status_id = 0x800;
  Step();
  CHECK(sent_count[comm.tx_id()] == before + 2);
  CHECK(controller.lifeState() == robot::LifeState::kWorking && controller.outputPermitted());

#ifndef TEST_CHASSIS
  // Gimbal 的实时 DBUS 分片要先拿到最后一个邮箱，并允许状态帧随后重试。
  while ((now_ms + 1u) % 5u != phase) Step();
  limit_peer_slot = true;
  peer_slot_attempts = 0;
  Step();
  CHECK(peer_slot_attempts == 2 && controller.outputPermitted());
  limit_peer_slot = false;
  Step();
#endif

  // 状态带急停时不能按例行帧跳过，即使只是邮箱暂时满。
  busy_status_id = comm.tx_id();
  Step(true);
  CHECK(controller.lifeState() == robot::LifeState::kDead && OutputsZero());
  CHECK(cancel_requests > before_abort);
  busy_status_id = 0x800;
  for (int i = 0; i < 110; ++i) Step(true);
  for (int i = 0; i < 650; ++i) Step();
  CHECK(controller.lifeState() == robot::LifeState::kWorking && !OutputsZero());
#ifdef TEST_CHASSIS
  // 只动右摇杆后回中。即使授权位未变化，也不能容许停转帧被当作例行帧。
  right_bias = 330;
  for (int i = 0; i < 12; ++i) Step();
  CHECK(comm.chassis_data().yaw_rate != 0.0f);
  busy_status_id = comm.tx_id();
  right_bias = 0;
  Step();
  CHECK(controller.lifeState() == robot::LifeState::kDead && OutputsZero());
  busy_status_id = 0x800;
  for (int i = 0; i < 110; ++i) Step(true);
  for (int i = 0; i < 650; ++i) Step();
  CHECK(controller.lifeState() == robot::LifeState::kWorking && !OutputsZero());
#endif
}

void TestIndependentImu()
{
#ifdef TEST_CHASSIS
  // 分离、小陀螺已有板间关节角，底盘本地 IMU 未参与这两档的控制。
  imu.sample_valid = false;
  imu.sample_age_ms = 50;
  imu.rejected.kind = hello_world::imu::ImuFaultKind::kGyroStale;
  imu.rejected.tick_ms = now_ms + 1;
  Step();
  CHECK(controller.lifeState() == robot::LifeState::kWorking && !OutputsZero());
  CHECK(comm.chassis_data().gimbal_enable && !comm.chassis_data().imu_ready);
  left_switch = 2;
  for (int i = 0; i < 100; ++i) Step();
  CHECK(controller.lifeState() == robot::LifeState::kWorking && controller.outputPermitted());
  CHECK(controller.motion().w > 0.0f);
  left_switch = 3;
  Step();
  CHECK(controller.lastDead().causes == robot::kDeadImu && OutputsZero());
  CHECK(!comm.chassis_data().gimbal_enable);
  imu.sample_valid = true;
  imu.sample_age_ms = 0;
  imu.rejected = {};
  left_switch = 1;
  Step(true);
  for (int i = 0; i < 650; ++i) Step();
  CHECK(controller.lifeState() == robot::LifeState::kWorking && !OutputsZero());
#else
  // 对板仍明确授权，仅其本地 IMU 遥测变化；云台依赖的是自己的反馈。
  peer_imu_ready = false;
  for (int i = 0; i < 8; ++i) Step();
  CHECK(controller.lifeState() == robot::LifeState::kWorking && controller.outputPermitted());
  CHECK(!OutputsZero());
  peer_imu_ready = true;
  Step(true);
  for (int i = 0; i < 12; ++i) Step();
  CHECK(controller.outputPermitted());
#endif
}

void TestActualPeerStatusLoss()
{
  // 一侧仍持续收到本地遥控和电机反馈，只有对板状态帧真正停止。
  deliver_peer_status = false;
  for (uint32_t i = 0; i < robot::kCommOfflineThresholdMs + 3u; ++i) Step();
  CHECK(controller.lifeState() == robot::LifeState::kDead && OutputsZero());
#ifdef TEST_CHASSIS
  CHECK(controller.lastDead().causes == robot::kDeadPeer && last_wheel_frame_zero);
#else
  CHECK(robot::diag::g_gimbal_dbg.last_dead_reason == 1u);
  CHECK(dm_last_cmd[0] == 0xFDu && dm_last_cmd[1] == 0xFDu);
#endif
  deliver_peer_status = true;
  for (int i = 0; i < 650; ++i) Step();
  CHECK(!controller.outputPermitted() && OutputsZero());
  for (int i = 0; i < 110; ++i) Step(true);
  for (int i = 0; i < 650; ++i) Step();
  CHECK(controller.lifeState() == robot::LifeState::kWorking && !OutputsZero());
}
#if defined(ROBOT_GROUND_TRIAL)
void TestGroundChassis() {
  CommTaskInit();
  forwarded_remote = true;
  peer.gimbal_data().yaw_joint = 1.2f;  // 首跑方向只能跟车身前轴。
  for (int i = 0; i < 600; ++i) Step(true);
  for (int i = 0; i < 100; ++i) Step();
  CHECK(!controller.outputPermitted() && OutputsZero() && !comm.chassis_data().gimbal_enable);
  CHECK(ground_wheel_live_frames == 0);

  ground_peer_disarmed = true;
  for (int i = 0; i < 700; ++i) Step(true);
  CHECK(controller.lifeState() == robot::LifeState::kWorking);
  for (int i = 0; i < 700; ++i) Step();  // 带杆解除，恢复多久也不能走。
  CHECK(!controller.outputPermitted() && OutputsZero());
  left_v = 1024;
  for (int i = 0; i < 100; ++i) Step();
  left_v = 1684;
  for (int i = 0; i < 260; ++i) Step();  // 回中过短，需要重计。
  CHECK(!controller.outputPermitted() && OutputsZero());
  left_v = 1024;
  for (int i = 0; i < 300; ++i) Step();
  CHECK(!controller.outputPermitted() && OutputsZero());
  left_v = 1684;
  for (int i = 0; i < 650; ++i) Step();
#if defined(ROBOT_GROUND_TRIAL_MOTION)
  CHECK(controller.outputPermitted() && module.driveReady() && !OutputsZero());
  CHECK(ground_wheel_live_frames > 0);
  CHECK(std::fabs(module.chassisMotion().vx - 0.20f) < 0.0001f);
  CHECK(module.chassisMotion().vy == 0.0f && module.chassisMotion().w == 0.0f);
  for (int i = 0; i < 4; ++i) CHECK(motors[i].command > 0.0f && motors[i].command <= 2.0f);
  left_v = 364;  // 反打也要先完成停机，不能直接换向。
  Step();
  CHECK(module.chassisMotion().vx > 0.0f && module.chassisMotion().vx < 0.20f);
  for (int i = 0; i < 900; ++i) Step();
  CHECK(!controller.outputPermitted() && OutputsZero());
  left_v = 1024;
  for (int i = 0; i < 300; ++i) Step();
  left_v = 364;
  for (int i = 0; i < 650; ++i) Step();
  CHECK(controller.outputPermitted() && module.chassisMotion().vx < -0.19f);
  const int cancels_before = cancel_requests;
  Step(true);
  CHECK(!controller.outputPermitted() && OutputsZero() && cancel_requests >= cancels_before + 2);
  left_v = 1024;
  // DBUS 分片转发不是每个 1ms 采样都完成一整帧。持续下档再放开。
  for (int i = 0; i < 30; ++i) Step(true);
  for (int i = 0; i < 300; ++i) Step();
  motors[6].angle(0.45f);  // 低出力舵角卡住，尚未达到原堵转出力阈值。
  left_v = 1684;
  const int live_frames_before = ground_wheel_live_frames;
  for (int i = 0; i < 450; ++i) Step();
  CHECK(controller.outputPermitted() && !module.driveReady() && module.alignBlockedMask() == 0x04u);
  CHECK(ground_wheel_live_frames == live_frames_before && ground_steer_live_frames > 0);
  for (int i = 0; i < 450; ++i) Step();
  CHECK(!controller.outputPermitted() && OutputsZero() && module.steerStallMask() == 0x04u);
  CHECK(controller.lifeState() == robot::LifeState::kDead && ground_wheel_live_frames == live_frames_before);
  CHECK(module.lastSteerStall().alignment_timeout_mask == 0x04u);

  motors[6].angle(0.0f);
  for (int i = 0; i < 600; ++i) Step();
  CHECK(!controller.outputPermitted() && OutputsZero() && module.steerStallMask() == 0x04u);
  left_v = 1024;
  for (int i = 0; i < 700; ++i) Step(true);
  for (int i = 0; i < 300; ++i) Step();
  left_v = 1684;
  for (int i = 0; i < 150; ++i) Step();
  CHECK(controller.outputPermitted() && !OutputsZero());
  left_switch = 3;  // 任一考核运动档都不能继承首跑授权。
  for (int i = 0; i < 30; ++i) Step();
  CHECK(!controller.outputPermitted() && OutputsZero() && !comm.chassis_data().gimbal_enable);
  left_switch = 1;
  for (int i = 0; i < 300; ++i) Step();
  CHECK(!controller.outputPermitted() && OutputsZero());
  left_v = 1024;
  for (int i = 0; i < 30; ++i) Step(true);
  for (int i = 0; i < 300; ++i) Step();
  left_v = 1684;
  for (int i = 0; i < 150; ++i) Step();
  CHECK(controller.outputPermitted() && !OutputsZero());
  ground_peer_disarmed = false;
  Step();
  CHECK(controller.lifeState() == robot::LifeState::kDead && !controller.outputPermitted() && OutputsZero());
  ground_peer_disarmed = true;
  for (int i = 0; i < 700; ++i) Step();
  CHECK(!controller.outputPermitted() && OutputsZero());
#else
  CHECK(!controller.outputPermitted() && OutputsZero() &&
        ground_wheel_live_frames == 0 && ground_steer_live_frames == 0);
#endif
  CHECK(!comm.chassis_data().gimbal_enable);
}
#elif defined(ROBOT_GROUND_GIMBAL_LOCK)
void TestGroundGimbal() {
  CommTaskInit();
  for (int i = 0; i < 800; ++i) Step(true);
  for (int i = 0; i < 350; ++i) Step();
  uint8_t frame[8];
  comm.encode(frame);
  CHECK(!controller.outputPermitted() && OutputsZero() && dm_enable_frames == 0);
  CHECK(dm_last_cmd[0] == 0xFD && dm_last_cmd[1] == 0xFD);
  CHECK((frame[0] & 0x40u) != 0 && !motors[0].drive_enabled && !motors[1].drive_enabled);
  dm_ack_disable = false;
  motors[1].drive_enabled = true;
  for (int i = 0; i < 12; ++i) Step();
  comm.encode(frame);
  CHECK((frame[0] & 0x40u) == 0 && !controller.outputPermitted());
  CHECK(dm_last_cmd[0] == 0xFD && dm_last_cmd[1] == 0xFD && dm_enable_frames == 0);
}
#endif
#ifdef TEST_CHASSIS
void TestSeparateRelease()
{
  // 用板间 Yaw 反馈和转发的真实 DBUS，左移/前进交替后每次松杆。
  const auto compare_release = [&](uint16_t h, uint16_t v) {
    left_h=h; left_v=v;
    const float yaw_start = peer.gimbal_data().yaw_joint;
    for (int i=0;i<280;++i) {
      peer.gimbal_data().yaw_joint = robot::WrapPi(yaw_start + 0.003f * (i+1));
      Step();
    }
    const float stop_yaw = peer.gimbal_data().yaw_joint;
    const auto before = module.chassisMotion();
    const float before_speed = std::hypot(before.vx,before.vy);
    const auto before_direction = std::atan2(before.vy,before.vx);
    CHECK(before_speed>0.95f && before.w==0.0f);
    CHECK(std::fabs(robot::WrapPi(before_direction - stop_yaw - robot::kPi - (h==1024 ? 0.0f : robot::kPi/2))) < 0.002f);

    left_h=left_v=1024;
    bool fixed_direction=true, smooth_deceleration=true, held_at_rest=true;
    float previous_speed=before_speed;
    bool captured_rest=false;
    float parked_target=0.0f;
    for (int i=0;i<180;++i) {
      peer.gimbal_data().yaw_joint = robot::WrapPi(stop_yaw + 0.003f * (i+1));
      Step();
      const auto cur=module.chassisMotion();
      const float speed=std::hypot(cur.vx,cur.vy);
      if (speed>0.05f && speed<before_speed-0.05f) {
        const float direction_error=robot::WrapPi(std::atan2(cur.vy,cur.vx)-before_direction);
        // 遥控中位经三片转发接收，允许前几拍参考角直接跟随最新云台。
        fixed_direction &= std::fabs(direction_error)<0.05f;
      }
      smooth_deceleration &= speed<=previous_speed+0.0001f && previous_speed-speed<=0.0082f;
      previous_speed=speed;
      if (speed==0.0f) {
        if (!captured_rest) { parked_target=module.wheelCommand(0).angle; captured_rest=true; }
        held_at_rest &= module.wheelCommand(0).speed==0.0f && module.wheelCommand(0).angle==parked_target;
      }
    }
    CHECK(fixed_direction && smooth_deceleration && captured_rest && held_at_rest);
    CHECK(controller.lifeState()==robot::LifeState::kWorking);
  };
  peer.gimbal_data().yaw_joint=2.8f; // 包含一次 ±π 越界。
  compare_release(694,1024); compare_release(1024,1354);

  // 松杆后刹车还没结束就改向，恢复实时云台参考时不得旋转原有的车体系速度。
  const auto turn_and_step = []() {
    auto& yaw = peer.gimbal_data().yaw_joint;
    yaw = robot::WrapPi(yaw + 0.003f);
    Step();
  };
  left_v=1354;
  for (int i=0;i<170;++i) turn_and_step();
  left_v=1024;
  for (int i=0;i<75;++i) turn_and_step();
  auto previous=module.chassisMotion();
  CHECK(std::hypot(previous.vx,previous.vy)>0.25f);
  left_h=694;
  bool continuous_regrab=true;
  for (int i=0;i<14;++i) {
    turn_and_step();
    const auto current=module.chassisMotion();
    continuous_regrab &= std::hypot(current.vx-previous.vx,current.vy-previous.vy)<0.014f;
    previous=current;
  }
  CHECK(continuous_regrab);
  left_h=1024;
  for (int i=0;i<150;++i) turn_and_step();

  // 分离档的拨轮仍可独立指令车身旋转；此时松开左杆不应清掉 w。
  wheel_raw=1160;
  for (int i=0;i<100;++i) Step();
  CHECK(controller.motion().w>0.9f && std::hypot(controller.motion().vx,controller.motion().vy)<0.001f);
  wheel_raw=1024; left_h=1024; left_v=1354; peer.gimbal_data().yaw_joint=0;
  for (int i=0;i<180;++i) Step();
}
#endif
#ifndef TEST_CHASSIS
void TestPitchTorqueLimit()
{
  FakeMotor yaw_axis, pitch_axis;
  pitch_axis.enforce_torque_limit = true;
  robot::Gimbal position_loop;
  auto* pitch_pid = GetPidMotorPitch();
  position_loop.init(&yaw_axis, &pitch_axis, GetPidMotorYaw(), pitch_pid);
  for (const float sign : {1.0f, -1.0f}) {
    position_loop.stop();
    position_loop.setPitchTarget(sign > 0.0f ? 0.6f : -0.35f);
    pitch_axis.angle(sign > 0.0f ? -0.6f : 0.85f);
    for (int i = 0; i < 5; ++i) position_loop.update(0.001f, 0.0f, 0.0f);
    CHECK(position_loop.controlValid());
    CHECK(pitch_axis.command == sign * 3.5f && pitch_pid->integral() == 0.0f);
    CHECK(position_loop.pitchAtLimit());
  }
  position_loop.stop();
}
#endif
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
#if defined(ROBOT_GROUND_TRIAL) || defined(ROBOT_GROUND_GIMBAL_LOCK)
#if defined(ROBOT_GROUND_TRIAL)
  for (int i = 0; i < 4; ++i) {
    // 沿用电流版舵角环；否则假电机输出可能全部舍入成 CAN 槽零值。
    pids[i + 4].init(6000.0f, 400.0f, 120.0f, 20000.0f, 1.0f);
    motors[i].setId(i + 1);
    motors[i + 4].setId(i + 1);
    (void)GetMotorSteer(i);
  }
  TestGroundChassis();
#else
  TestGroundGimbal();
#endif
  std::printf("Ground trial integration: %d checks, %d failures\n", checks, failures);
  return failures ? 1 : 0;
#endif
#if defined(ROBOT_PITCH_GRAVITY_FF) && ROBOT_PITCH_GRAVITY_FF
  TestPitchTorqueLimit();
  CommTaskInit();
  for (int i = 0; i < 550; ++i) Step(true);
  for (int i = 0; i < 12; ++i) Step();
  CHECK(controller.lifeState() == robot::LifeState::kWorking && !OutputsZero());
  TestRoutinePeerSendBusy();
  TestIndependentImu();
  TestActualPeerStatusLoss();
  std::printf("Pitch feedforward and safety integration: %d checks, %d failures\n", checks, failures);
  return failures ? 1 : 0;
#endif
#if defined(ROBOT_STEER_MODE_PROBE) && ROBOT_STEER_MODE_PROBE
  motors[5].setId(2);
  CommTaskInit();
  for (int i=0;i<800;++i) Step(true);
  CHECK(controller.lifeState()==robot::LifeState::kWorking);
  for (int i=0;i<100;++i) Step();
  neutral_left=false;
  const int voltage_zero_before=probe_zero[0x1FF];
  const int current_zero_before=probe_zero[0x1FE];
  for (int i=0;i<50;++i) Step();
  CHECK(probe_nonzero[0x1FF]>0 && probe_nonzero[0x1FE]==0);
  CHECK(probe_zero[0x1FF]==voltage_zero_before && probe_zero[0x1FE]==current_zero_before);
  CHECK(!comm.chassis_data().gimbal_enable && comm.chassis_data().estop && OutputsZero());
  CHECK(!probe_bad_slots);
  neutral_left=true;
  left_switch=3;
  for (int i=0;i<220;++i) Step(true);
  for (int i=0;i<100;++i) Step();
  neutral_left=false;
  const int voltage_count_before=sent_count[0x1FF];
  for (int i=0;i<50;++i) Step();
  CHECK(sent_count[0x1FF]==voltage_count_before); // 电流试探期间也不能夹杂普通 0x1FF。
  CHECK(probe_nonzero[0x1FE]>0 && probe_nonzero[0x200]==0 && !probe_bad_slots);
  CHECK(robot::probe::g_steer_probe_voltage.sent>0 && robot::probe::g_steer_probe_current.sent>0);
  neutral_left=true; left_switch=1;
  for (int i=0;i<220;++i) Step(true);
  for (int i=0;i<100;++i) Step();
  neutral_left=false; probe_reject=true;
  for (int i=0;i<10;++i) Step();
  CHECK(robot::probe::g_steer_probe_voltage.attempted>0);
  CHECK(robot::probe::g_steer_probe_voltage.queued==0 && robot::probe::g_steer_probe_voltage.sent==0);
  CHECK(OutputsZero() && controller.lifeState()==robot::LifeState::kDead);
  std::printf("Steer probe integration: %d checks, %d failures\n",checks,failures);
  return failures?1:0;
#endif
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
  CHECK(sent_bus[kSteerTxId]==robot::wiring::Can(robot::wiring::kChassisSteer));
  CHECK(sent_bus[comm.tx_id()]==robot::wiring::Can(robot::wiring::kChassisPeer));
#else
  CHECK(sent_bus[1]==robot::wiring::Can(robot::wiring::kGimbalYaw));
  CHECK(dm_last_cmd[0]==0xFDu && dm_last_cmd[1]==0xFDu); // 上电即失能，包括电机还保留旧使能的情况
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
  TestSeparateRelease();
  // 自动抬平期间，明显的向下指令立即接管；两板共用 -0.40 rad 下限。
  peer.gimbal_data().pitch_joint = -0.30f;
  right_bias = -330;
  for (int i=0;i<20;++i) Step(true);
  for (int i=0;i<50;++i) Step();
  CHECK(comm.chassis_data().pitch_target < -0.31f);
  for (int i=0;i<400;++i) Step();
  CHECK(std::fabs(comm.chassis_data().pitch_target + 0.40f) < 0.00001f);
  right_bias = 330;
  for (int i=0;i<20;++i) Step();
  CHECK(comm.chassis_data().pitch_target > -0.39f);
  right_bias = 0;
  left_switch=3;for(int i=0;i<6;++i) Step();CHECK(controller.chassisMode()==robot::ChassisMode::kFollow);
  left_switch=2;for(int i=0;i<6;++i) Step();CHECK(controller.chassisMode()==robot::ChassisMode::kSpin);
  left_v = 1684;
  // 只模拟舵实际追上限速目标；通过真实 DBUS 与两板控制链路合推。
  for (int t = 0; t < 600; ++t) {
    for (int i = 0; i < 4; ++i) motors[i + 4].angle(robot::WrapPi(module.wheelCommand(i).angle));
    Step();
  }
  CHECK(controller.motion().w > 1.49f && controller.motion().w < 1.51f);
  CHECK(std::fabs(std::hypot(controller.motion().vx, controller.motion().vy) - 2.0f) < 0.0001f);
  CHECK(module.driveReady());
  CHECK(robot::diag::g_chassis_dbg.drive_w_request > 1.49f);
  left_v = 1354;
  for (int i = 0; i < 4; ++i) motors[i + 4].angle(0.0f);
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
  comm = robot::GimbalChassisComm(role, kStatusTxId, kStatusRxId);
  peer = robot::GimbalChassisComm(peer_role, kStatusRxId, kStatusTxId);
  controller.init();
  CHECK(controller.lastImuDead().causes==robot::kDeadNone);
  for (int i = 0; i < 650; ++i) Step();
  CHECK(OutputsZero()); CHECK(controller.lifeState() == robot::LifeState::kDead);
  Step(true); // 上电必须由有效遥控急停确认，防止看门狗重启后自动带指令起动。
#else
  for (int i = 0; i < 550; ++i) Step();
  CHECK(OutputsZero() && dm_enable_frames == 0);
  CHECK(!motors[0].drive_enabled && !motors[1].drive_enabled);
  CHECK(controller.lifeState() == robot::LifeState::kWorking); // 两板自检就绪不依赖电机先使能
  Step(true);
#endif
  for (int i = 0; i < 650; ++i) Step();
  CHECK(controller.lifeState() == robot::LifeState::kWorking); CHECK(!OutputsZero()); CHECK(feeds > 0);
  TestRoutinePeerSendBusy();
  TestIndependentImu();
  TestActualPeerStatusLoss();
#ifdef TEST_CHASSIS
#if !defined(ROBOT_STEER_MODE_PROBE) || !ROBOT_STEER_MODE_PROBE
  // 切到跟随时只给角差，四舵保持不动；恢复后仍能看到触发轮和真实 TXOK。
  for (int i=4; i<8; ++i) {
    motors[i].setId(static_cast<uint8_t>(i-3));
    pids[i].init(20000, 0, 0, 20000, 1.0f);
  }
  peer.gimbal_data().yaw_joint = 0.3f;
  left_switch = 3;
  for (int i=0; i<900; ++i) Step();
  const auto& event = robot::diag::g_steer_stall_tx;
  CHECK(controller.lifeState() == robot::LifeState::kDead && OutputsZero());
  CHECK(event.sequence == 1 && event.has_recent_txok);
  CHECK(event.frame_id == kSteerTxId && event.command_raw[0] != 0);
  CHECK(event.confirmed_nonzero > 0 && event.attempted_nonzero >= event.confirmed_nonzero);
  const auto stall_dead = controller.lastDead();
  CHECK(stall_dead.causes==robot::kDeadSteerStall && stall_dead.steer_stall_sequence==event.sequence);
  CHECK(stall_dead.mode==robot::ChassisMode::kFollow && stall_dead.tick_ms==event.tick_ms);
  const auto tick_at_stall = event.tick_ms;
  left_switch=1;
  peer.gimbal_data().yaw_joint = 0;
  for (int i=4; i<8; ++i) pids[i].init(1.0f, 0.1f, 0.0f, 8.0f, 1.0f);
  for (int i=0; i<650; ++i) Step(true);
  CHECK(controller.lifeState() == robot::LifeState::kWorking && OutputsZero());
  CHECK(module.steerStallMask() == 0 && module.lastSteerStall().stall_mask != 0);
  CHECK(event.tick_ms == tick_at_stall && event.has_recent_txok);
  CHECK(controller.lastDead().tick_ms==stall_dead.tick_ms && controller.lastDead().causes==stall_dead.causes);
  Step();
#endif
#endif
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
  Step(true, false, false); CHECK(OutputsZero()); // 本地急停；对板急停信息尚未往返
#ifdef TEST_CHASSIS
  CHECK(sent_count[0x200]>stopped_count);
#else
  CHECK(sent_count[1]>stopped_count);
  CHECK(dm_last_cmd[0]==0xFDu && dm_last_cmd[1]==0xFDu);
  const int enable_before_hold = dm_enable_frames;
  const int disable_before_hold = dm_disable_frames;
  for (int i = 0; i < 12; ++i) Step(true, false, false);
  CHECK(dm_enable_frames == enable_before_hold && dm_disable_frames > disable_before_hold);
  CHECK(!motors[0].drive_enabled && !motors[1].drive_enabled);
#endif
  Step();
#ifdef TEST_CHASSIS
  failed_id=kSteerTxId; // 只让舵电机提交失败，轮电机和板间帧仍成功
  for(int i=0;i<14;++i) {Step();}
  const int steer_fault_feeds=feeds;Step();CHECK(feeds==steer_fault_feeds);
  CHECK(controller.lastDead().causes==robot::kDeadCanTx && controller.lastDead().can_failed_cycles>0);
  CHECK(controller.lastDead().steer_stall_sequence==module.lastSteerStall().sequence);
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
  // 架空颠动后的瞬时低模长：上一份可靠 IMU 数据仍然新鲜。
  imu.rejected.kind = hello_world::imu::ImuFaultKind::kAccLow;
  imu.rejected.has_values = true;
  imu.rejected.tick_ms = now_ms + 1;
  imu.sample_age_ms = 1;
  imu.sample_valid = false;
  Step();
  CHECK(controller.lifeState() == robot::LifeState::kWorking && controller.outputPermitted());
#ifdef TEST_CHASSIS
  CHECK(comm.chassis_data().imu_ready);
#else
  CHECK(comm.gimbal_data().imu_ready);
  CHECK(robot::diag::g_gimbal_dbg.yaw_rate_cmd == 0.0f);
#endif
  imu.sample_valid = true;
  imu.rejected = {};
  Step();
  imu.sample_age_ms = 9;
#ifdef TEST_CHASSIS
  // 跟随档只有右杆输入时判废。后续遥控归中、IMU 恢复，停机快照仍应保留当拍输入。
  left_switch = 3;
  left_h = left_v = 1024;
  right_bias = 330;
  imu.rejected.kind = hello_world::imu::ImuFaultKind::kAccLow;
  imu.rejected.has_values = true;
  imu.rejected.tick_ms = now_ms + 1;
#else
  // 真正的传输错误即使上一拍有效，也不能享受 AccLow 的短窗口。
  imu.sample_age_ms = 1;
  imu.rejected.kind = hello_world::imu::ImuFaultKind::kSpi;
  imu.rejected.tick_ms = now_ms + 1;
#endif
  imu.sample_valid = false; Step();
  CHECK(OutputsZero()); CHECK(controller.lifeState() == robot::LifeState::kDead);
#ifdef TEST_CHASSIS
  CHECK(controller.lastDead().causes==robot::kDeadImu);
  const auto imu_dead_tick = now_ms;
  const auto& dead = controller.lastDead();
  CHECK(dead.mode==robot::ChassisMode::kFollow && dead.left_centered);
  CHECK(dead.remote_source==robot::RemoteSource::kLocal && dead.remote.online);
  CHECK(dead.remote.lv==0.0f && dead.remote.lh==0.0f);
  CHECK(dead.remote.rh>0.48f && dead.remote.rv>0.48f && dead.remote.wheel==0.0f);
  CHECK(dead.imu.reason==hello_world::imu::ImuFaultKind::kAccLow && !dead.imu.sample_valid);
  CHECK(dead.imu.last_rejected.has_values && dead.imu.last_rejected.tick_ms==imu_dead_tick);
  right_bias = 0;
  left_switch = 1;
  left_v = 1354;
  imu.rejected = {};
#else
  CHECK(robot::diag::g_gimbal_dbg.last_dead_reason == 2u);
  CHECK(robot::diag::g_gimbal_dbg.last_dead_tick_ms == now_ms);
  imu.rejected = {};
#endif
  imu.sample_valid = true;
  for (int i = 0; i < 650; ++i) Step();
#ifdef TEST_CHASSIS
  CHECK(OutputsZero()); CHECK(controller.lifeState() == robot::LifeState::kDead);
  CHECK(dead.tick_ms==imu_dead_tick && dead.remote.rh>0.48f &&
        dead.imu.reason==hello_world::imu::ImuFaultKind::kAccLow);
  Step(true); for (int i = 0; i < 650; ++i) Step();
#else
  CHECK(OutputsZero() && !controller.outputPermitted()); // IMU 恢复不自动续上旧授权
  Step(true);
  for (int i = 0; i < 12; ++i) Step();
  // Yaw 总线仍在撤销旧帧时，不能反复撤销另一条总线新排入的 Pitch 失能帧。
  dm_cancel_fail_bus = &hcan1;
  Step(true);
  const int yaw_cancels = dm_cancel_calls[0];
  const int pitch_cancels = dm_cancel_calls[1];
  for (int i = 0; i < 4; ++i) Step(true);
  CHECK(dm_cancel_calls[0] > yaw_cancels && dm_cancel_calls[1] == pitch_cancels);
  CHECK(dm_last_cmd[1] == 0xFDu && OutputsZero());
  dm_cancel_fail_bus = nullptr;
  for (int i = 0; i < 110; ++i) Step(true);
  for (int i = 0; i < 650; ++i) Step();
#endif
  CHECK(controller.lifeState() == robot::LifeState::kWorking);
#ifdef TEST_CHASSIS
  // 云台单独复位/掉线后返回，底盘仍不能保留先前授权自行起动。
  peer_working = false; Step(); CHECK(OutputsZero());
  CHECK(controller.lastDead().causes==robot::kDeadPeer);
  // 后续非 IMU 故障可以更新通用记录，但不能覆盖最近一次 IMU 停机的证据。
  const auto& imu_dead = controller.lastImuDead();
  CHECK(imu_dead.tick_ms==imu_dead_tick && imu_dead.causes==robot::kDeadImu);
  CHECK(imu_dead.imu.reason==hello_world::imu::ImuFaultKind::kAccLow &&
        imu_dead.remote.rh>0.48f);
  peer_working = true;
  for (int i = 0; i < 650; ++i) Step();
  CHECK(OutputsZero()); CHECK(controller.lifeState() == robot::LifeState::kDead);
  Step(true); for (int i = 0; i < 650; ++i) Step();
  CHECK(controller.lifeState() == robot::LifeState::kWorking);
  // 舵反馈越界时，在线计数仍更新；必须从运动状态退出且清零，不能只做一次静默归零。
  motors[6].angle(26.65f);
  Step();
  CHECK(controller.lifeState()==robot::LifeState::kDead && OutputsZero());
  CHECK(controller.lastDead().causes==robot::kDeadMotor);
  CHECK(controller.lastDead().steer_angle.mask==(1u<<2));
  CHECK(controller.lastDead().steer_angle.measured_rad[2]==26.65f);
  const uint32_t invalid_angle_tick = now_ms;
  motors[6].angle(0.0f);
  for (int i = 0; i < 650; ++i) Step();
  CHECK(controller.lifeState()==robot::LifeState::kDead && OutputsZero());
  CHECK(controller.lastDead().tick_ms==invalid_angle_tick);
  CHECK(controller.lastDead().steer_angle.mask==(1u<<2));
  Step(true); for (int i = 0; i < 650; ++i) Step();
  CHECK(controller.lifeState()==robot::LifeState::kWorking && !OutputsZero());
  // 在原本不发电机帧的奇数周期同时发现两轮位无路，仍撤销两路邮箱并立即发零。
  if ((now_ms & 1u) != 0u) Step();
  CHECK(controller.outputPermitted());
  const int abort_before_keepout = cancel_requests;
  const int wheel_frames_before_keepout = sent_count[0x200u];
  const int steer_frames_before_keepout = sent_count[kSteerTxId];
  hello_world::chassis_ik_solver::Geometry restricted;
  restricted.steer_keepout[0] = {0.30f, 0.10f};
  restricted.steer_keepout[2] = {-2.60f, 0.03f};
  solver.init(restricted, 10.0f, 3.0f);
  motors[4].angle(0.30f);
  motors[6].angle(-2.60f);
  Step();
  CHECK(controller.lifeState() == robot::LifeState::kWorking && !controller.outputPermitted());
  CHECK(OutputsZero() && module.steerKeepoutBlockedMask() == 5u);
  CHECK(cancel_requests >= abort_before_keepout + 2);
  CHECK(sent_count[0x200u] == wheel_frames_before_keepout + 1 && last_wheel_frame_zero);
  CHECK(sent_count[kSteerTxId] == steer_frames_before_keepout + 1 && last_steer_frame_zero);
  motors[4].angle(0.0f);
  motors[6].angle(0.0f);
  Step();
  CHECK(controller.outputPermitted() && module.steerKeepoutBlockedMask() == 0u);
  CHECK(robot::diag::g_chassis_dbg.last_steer_keepout_blocked_mask == 5u);
  solver.init({}, 10.0f, 3.0f);
  for (int i = 0; i < 500; ++i) Step(false, true);
  CHECK(std::hypot(controller.motion().vx, controller.motion().vy) <= 2.00001f);
#else
  // C2G 急停不能原样回显到 G2C，否则双方 OR 后无法解除急停。
  for (int i = 0; i < 4000; ++i) Step();
  CHECK(std::fabs(module.yawTarget()) <= robot::kPi);
  TestPitchTorqueLimit();
  // 云台模块的横转和俯仰目标跳变：实物转速为零，输出不应被目标微分顶到限幅。
  FakeMotor yaw_axis, pitch_axis;
  auto* yaw_pid = GetPidMotorYaw();
  auto* pitch_pid = GetPidMotorPitch();
  robot::Gimbal position_loop;
  position_loop.init(&yaw_axis, &pitch_axis, yaw_pid, pitch_pid);
  position_loop.resetControllers();
  // 固定初始位置，从小目标到更大目标；P 追角，D 不追指令跳变。
  position_loop.update(0.001f,0.0f,0.0f);
  position_loop.setYawWorldTarget(0.2f); position_loop.setPitchTarget(0.2f);
  position_loop.update(0.001f,0.0f,0.0f);
  CHECK(yaw_axis.command>0.86f && yaw_axis.command<0.94f);  // 加硬：yaw Kp 4.5
  CHECK(pitch_axis.command>2.16f && pitch_axis.command<2.24f);  // 加硬：pitch Kp 11
  // 实测 Pitch 同方向运动时应该减力，回到固定目标附近时应该反向刹车。
  pitch_axis.angle(0.2f); pitch_axis.velocity(1.0f);
  for (int i=0;i<80;++i) position_loop.update(0.001f,0.0f,0.0f);
  CHECK(pitch_axis.command<-0.1f);
  position_loop.stop(); CHECK(yaw_axis.command==0 && pitch_axis.command==0);
  pitch_axis.angle(robot::Gimbal::kPitchUpperLimit);
  pitch_axis.velocity(-1.0f);
  position_loop.setPitchTarget(2.0f);
  for (int i=0;i<80;++i) position_loop.update(0.001f,0.0f,0.0f);
  CHECK(pitch_axis.command==0);
  pitch_axis.velocity(0.0f); position_loop.setPitchTarget(0.0f);
  position_loop.update(0.001f,0.0f,0.0f); CHECK(pitch_axis.command<0);
  // 模拟达妙真实的限幅返回：大角差 + 同向前馈，不得因正常饱和撤销控制。
  yaw_axis.enforce_torque_limit = true;
  for (float dir : {1.0f, -1.0f}) {
    position_loop.stop();
    position_loop.setYawWorldTarget(dir * 2.0f);
    position_loop.setYawRateFeedforward(dir * 2.0f);
    position_loop.update(0.001f,0.0f,0.0f);
    CHECK(position_loop.controlValid() && yaw_axis.command == dir * 3.5f);
    CHECK(yaw_pid->integral() == 0.0f);
    CHECK(robot::diag::g_gimbal_dbg.yaw_output == yaw_axis.command);
    CHECK(std::fabs(robot::diag::g_gimbal_dbg.yaw_ff_torque - dir * 0.6f) < 0.0001f);
  }
  position_loop.stop();
  CHECK(robot::diag::g_gimbal_dbg.yaw_output==0.0f && robot::diag::g_gimbal_dbg.yaw_ff_torque==0.0f);
  position_loop.setYawWorldTarget(0.0f);
  position_loop.update(0.001f,0.0f,0.0f);
  CHECK(position_loop.controlValid() && yaw_axis.command == 0.0f);
  position_loop.stop();
  // 当最短角误差跨过 ±π 分支，不能把近 2π 的表示跳变当作速度。
  hello_world::pid::Pid yaw_d_only(0,0,1,3.5f,0);
  position_loop.init(&yaw_axis, &pitch_axis, &yaw_d_only, pitch_pid);
  position_loop.update(0.001f,robot::kPi-0.005f,0.0f);
  position_loop.update(0.001f,-robot::kPi+0.005f,0.0f);
  CHECK(std::fabs(yaw_axis.command)<0.01f);
  // 纯 IMU 转动不能被误当成关节变化；用投影后的世界 Yaw 转速给出阻尼。
  pids[0].setGains(0,0,1);
  imu.roll_value=0.3f; imu.pitch_value=0.4f;
  imu.gyro_y=1.0f; imu.gyro_z=2.0f;
  motors[0].velocity(-10.0f);
  for (int i=0;i<100;++i) Step();
  CHECK(motors[0].command < -2.05f && motors[0].command > -2.14f);  // 含 1 rad/s × 0.30 前馈
  imu.roll_value=0.0f; imu.pitch_value=0.0f; imu.gyro_y=0.0f; imu.gyro_z=0.0f;
  imu.gyro_z=std::numeric_limits<float>::quiet_NaN(); Step(); CHECK(OutputsZero());
  imu.yaw_value=0.0f; imu.gyro_z=0.0f; motors[0].velocity(0.0f);
  pids[0].setGains(1.0f,0.1f,0.0f);
  Step(true); CHECK(comm.gimbal_data().estop); // 本地急停仍必须上报。
  peer.chassis_data().estop = true;
  Remote(false); uint8_t f[8]; peer.encode(f); comm.decode(f); controller.update();
  CHECK(controller.isEstopped()); CHECK(!comm.gimbal_data().estop);
  // 单个轴的坏反馈必须使整个云台输出清零。
  module.setYawWorldTarget(1.0f); module.setPitchTarget(0.2f);
  module.update(0.001f, std::numeric_limits<float>::quiet_NaN(), 0.0f);
  CHECK(OutputsZero());
  module.setPitchTarget(std::numeric_limits<float>::infinity());
  module.update(0.001f, 0.0f, 0.0f); CHECK(OutputsZero());
  // 角度到达软件边界后，旧积分不得继续向边界外施加力矩。
  module.stop(); module.setYawWorldTarget(0); module.setPitchTarget(0.7f);
  for (int i = 0; i < 2000; ++i) module.update(0.001f, 0.0f, 0.0f);
  motors[1].angle(0.7f); module.update(0.001f, 0.0f, 0.0f);
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
  motors[1].angle(0);
  for (int i = 0; i < 520; ++i) Step(true);
  for (int i = 0; i < 8 && dm_pending_reply[1] != 0xFCu; ++i) Step();
  CHECK(dm_pending_reply[0]==0xFCu && dm_pending_reply[1]==0xFCu && OutputsZero());
  interrupt_hook=[]() { motors[1].angle(0.4f); };
  Step(); CHECK(module.controlValid() && std::fabs(motors[1].command)<0.000001f);
  motors[1].angle(0);
  // 轴在运行中意外返回失能且无明确急停确认：不得自动尝试重新使能。
  for (int i = 0; i < 10; ++i) Step();
  CHECK(motors[0].drive_enabled && motors[1].drive_enabled && !OutputsZero());
  const int enable_before_drop = dm_enable_frames;
  motors[0].drive_enabled = false;
  for (int i = 0; i < 650; ++i) Step();
  CHECK(OutputsZero() && dm_enable_frames == enable_before_drop);
  CHECK(dm_last_cmd[0]==0xFDu && dm_last_cmd[1]==0xFDu);
  Step(true);
  for (int i = 0; i < 12; ++i) Step();
  CHECK(motors[0].drive_enabled && motors[1].drive_enabled);

  // 急停后尚无两轴失能反馈：释放拨杆也不能直接恢复旧扭矩。
  dm_ack_disable = false;
  Step(true);
  for (int i = 0; i < 8; ++i) Step();
  CHECK(OutputsZero() && dm_last_cmd[0]==0xFDu && dm_last_cmd[1]==0xFDu);
  dm_ack_disable = true;
  for (int i = 0; i < 14; ++i) Step();
  CHECK(motors[0].drive_enabled && motors[1].drive_enabled);

  // 已确认失能后，Yaw 在急停期间意外报告使能；不能拿旧确认打开新一轮输出。
  for (int i = 0; i < 8; ++i) Step(true);
  CHECK(!motors[0].drive_enabled && !motors[1].drive_enabled);
  dm_ack_disable = false;
  Step(true);  // 收完上一拍已排队的失能应答。
  motors[0].drive_enabled = true;
  Step(true);
  const int enable_before_stale_ack = dm_enable_frames;
  for (int i = 0; i < 8; ++i) Step();
  CHECK(!controller.outputPermitted() && dm_enable_frames == enable_before_stale_ack);
  CHECK(OutputsZero() && dm_last_cmd[0]==0xFDu && dm_last_cmd[1]==0xFDu);
  dm_ack_disable = true;
  for (int i = 0; i < 14; ++i) Step();
  CHECK(motors[0].drive_enabled && motors[1].drive_enabled);

  // 一轴应答未使能，另一轴也不能带力矩等它；握手超时后关两轴，不能自行重试。
  for (int i = 0; i < 8; ++i) Step(true);
  dm_ack_enable[1] = false;
  for (int i = 0; i < 8; ++i) Step();
  CHECK(motors[0].drive_enabled && !motors[1].drive_enabled && OutputsZero());
  for (int i = 0; i < 110; ++i) Step();
  CHECK(!motors[0].drive_enabled && !motors[1].drive_enabled && OutputsZero());
  CHECK(dm_last_cmd[0]==0xFDu && dm_last_cmd[1]==0xFDu);
  const int enable_after_timeout = dm_enable_frames;
  dm_ack_enable[1] = true;
  for (int i = 0; i < 550; ++i) Step();
  CHECK(dm_enable_frames == enable_after_timeout && OutputsZero());
  Step(true);
  for (int i = 0; i < 12; ++i) Step();
#endif
  Step(); motors[0].healthy = false; Step();
  CHECK(controller.lifeState() == robot::LifeState::kDead); CHECK(OutputsZero());
  motors[0].healthy = true;
  for(int i=0;i<110;++i) Step(true);
  for(int i=0;i<650;++i) Step();
  const int canceled_before=cancel_requests;
  Step(true); CHECK(OutputsZero()); CHECK(cancel_requests>canceled_before);
#ifdef TEST_CHASSIS
  Step();
#else
  for (int i = 0; i < 12; ++i) Step();
  CHECK(motors[0].drive_enabled && motors[1].drive_enabled && controller.outputPermitted());
#endif
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
