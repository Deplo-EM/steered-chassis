/**
 *******************************************************************************
 * @file      :comm_task.cpp
 * @brief     : 底盘板通信任务（DBUS 接收 + CAN 发送 + 看门狗）
 * @history   :
 *  Version     Date            Author          Note
 *  V1.0.0      2026-10-19      <author>        1. 完成编写
 *  V1.1.0      2026-10-20      <author>        2. 审计修复：喂狗改为「控制循环跑完才喂」，
 *                                                且非法 DBUS 帧不再视为遥控在线
 *******************************************************************************
 * @attention :
 *  【看门狗到底在监督什么（V1.1 修正）】
 *  最初的实现把喂狗放在 UART 接收中断里。问题是：UART 中断优先级高于 TIM6，
 *  所以哪怕控制循环（TIM6）因为 SPI 卡死、死循环而完全停住，UART 中断照样
 *  会进来喂狗 —— 看门狗就永远等不到复位，考核要求的「任务卡死 200ms 复位」
 *  形同虚设。
 *
 *  现在的做法：
 *    1. 中断里**只记录**「收到了一帧合法 DBUS」这件事（时间戳 + 计数），不喂狗；
 *    2. 唯一的喂狗点在 CommTask 的最后一行；
 *    3. 而 CommTask 是由 TIM6 中断在 MainTask() **之后**调用的。
 *  于是：只要控制循环没跑完这一整轮，就不会喂狗 → 200ms 后复位。
 *
 *  【为什么要校验 DBUS 帧】
 *  串口错位、噪声、接收机掉电时的全零数据，都可能凑够 18 字节。不校验就把
 *  它们当有效指令，等于「谁都能让这台车继续动」。校验规则见 dbus_check.hpp。
 *  非法帧：不解码、不刷新在线时间戳、不喂狗。
 *
 *  ⚠️ 副作用：上电时遥控器没开机 → 板子每 200ms 复位一次（预期行为）。
 *  台架上需要不开遥控器调试时，把 kDebugAlwaysFeedWatchdog 改成 true
 *  （= 无条件喂狗，板子不再复位）；验收前必须改回 false。
 *******************************************************************************
 *  Copyright (c) 2024 Hello World Team, Zhejiang University.
 *  All Rights Reserved.
 *******************************************************************************
 */
/* Includes ------------------------------------------------------------------*/
#include "comm_task.hpp"

#include <cmath>

#include "critical_section.hpp"

#include "DT7.hpp"
#include "HW_can.hpp"
#include "chassis.hpp"
#include "can.h"
#include "board_wiring.hpp"
#include "dbus_check.hpp"
#include "fault_injection.hpp"
#include "gimbal_chassis_comm.hpp"
#include "ins_all.hpp"
#include "iwdg.h"
#include "motor.hpp"
#include "robot.hpp"
#include "tick.hpp"
#include "usart.h"
#if defined(ROBOT_STEER_MODE_PROBE) && ROBOT_STEER_MODE_PROBE
#include "steer_mode_probe.hpp"

namespace robot::probe {
// Ozone 只读结果，急停后仍保留。每一组只显示该模式最近一次试探。
volatile Result g_steer_probe_voltage;
volatile Result g_steer_probe_current;
volatile uint8_t g_steer_probe_phase = 0;
}
#else
#include "steer_stall_tx.hpp"
namespace robot::diag {
volatile SteerStallTx g_steer_stall_tx;
}
#endif
/* Private macro -------------------------------------------------------------*/
/* Private constants ---------------------------------------------------------*/

namespace {

/**
 * @brief 调试模式：无条件喂狗
 * @note  **正常情况下必须是 false**（只有控制周期跑完 + 遥控数据新鲜才喂）。
 *        台架上需要连着调试器、又不接遥控器时，可以临时改成 true —— 此时
 *        看门狗不再起保护作用，**绝不能用于实车控制**，验收前必须改回来。
 *        注意：改成 true 的效果是「板子不再复位」；改成 false（默认）才是
 *        「条件喂狗、关遥控器会复位」。审计 F2 指出的正是原文档把这两个方向说反了。
 */
constexpr bool kDebugAlwaysFeedWatchdog = false;
#if (defined(ROBOT_GROUND_TRIAL) && ROBOT_GROUND_TRIAL) || \
    (defined(ROBOT_COMPETITION_FINISH) && ROBOT_COMPETITION_FINISH)
static_assert(!kDebugAlwaysFeedWatchdog, "Keep watchdog supervision active");
#endif

/** 遥控数据超过这个时间没更新，就认为「主人失联」，不再喂狗（ms） */
constexpr uint32_t kDbusTimeoutMs = 100u;

/** DBUS缓冲区36字节；仅接受IDLE边界18字节 */
uint8_t s_dbus_buf[2u * hello_world::remote_control::kRcRxDataLen];

/** 最近一帧「合法」DBUS 的时间戳（只由 UART 中断写） */
volatile uint32_t s_last_valid_dbus_tick = 0;
/** 是否收到过至少一帧合法数据（防止上电瞬间被误判为在线） */
volatile bool s_dbus_received = false;
/** 统计：合法 / 非法帧数（调试用） */
volatile uint32_t s_dbus_valid_frames = 0;
volatile uint32_t s_dbus_invalid_frames = 0;
volatile uint32_t s_dbus_rx_start_failures = 0;
volatile uint32_t s_dbus_uart_errors = 0;
volatile uint32_t s_dbus_partial_frames = 0;
bool s_dbus_restart_pending = true;

/** 普通DMA配合36字节缓冲，以IDLE收18字节帧；半满中断不能当成一帧。 */
void StartDbusReceive()
{
  robot::CriticalSection lock;
  if (!s_dbus_restart_pending) return;
  if (HAL_UARTEx_ReceiveToIdle_DMA(&huart3, s_dbus_buf, sizeof(s_dbus_buf)) == HAL_OK) {
    __HAL_DMA_DISABLE_IT(huart3.hdmarx, DMA_IT_HT);
    s_dbus_restart_pending = false;
  } else {
    s_dbus_rx_start_failures = s_dbus_rx_start_failures + 1u;
  }
}


/** CAN 发送失败计数（邮箱满 / 总线错误时会增长） */
uint32_t s_can_tx_fail_count = 0;

/** CAN 发送缓冲（每个控制周期复用，避免频繁压栈） */
uint8_t s_wheel_frame[8];
uint8_t s_steer_frame[8];
uint8_t s_comm_frame[8];

/**
 * @brief 本地合法帧或云台完整转发帧足够新，才允许本周期喂狗。
 * @note 云台在线标志本身不能保活，必须同时有完整18字节的新序号遥控帧。
 */
bool RemoteAliveForWatchdog()
{
  robot::CriticalSection lock;
  const bool local =
      s_dbus_received &&
      ((hello_world::tick::GetTickMs() - s_last_valid_dbus_tick) <= kDbusTimeoutMs);

  robot::GimbalChassisComm* comm = GetGimbalChassisComm();
  const bool peer = (comm != nullptr) && !comm->isOffline() &&
                    comm->gimbal_data().dbus_online &&
                    comm->remote_relay().online(hello_world::tick::GetTickMs());

  return local || peer;
}

#if !defined(ROBOT_STEER_MODE_PROBE) || !ROBOT_STEER_MODE_PROBE
bool EncodeMotor(hello_world::motor::Motor* motor, uint8_t* frame)
{
  robot::CriticalSection lock;
  return motor != nullptr && motor->encode(frame);
}
#endif

bool s_previous_output=false;
bool s_cancel_pending=false;
#if !defined(ROBOT_STEER_MODE_PROBE) || !ROBOT_STEER_MODE_PROBE
struct SteerTxSample {
  uint16_t id = 0;
  int16_t raw[4] = {};
};
robot::CanTxReceipt s_steer_receipt;
SteerTxSample s_candidate, s_last_txok;
uint32_t s_steer_attempted = 0, s_steer_queued = 0;
uint32_t s_steer_confirmed = 0, s_steer_unconfirmed = 0, s_steer_txok_tick = 0;
constexpr uint32_t kRecentSteerTxMs = 10u;

/** 仅在有可用回执时跟踪非零帧，不为诊断改变正常的发送节拍。 */
bool SendSteerFrame(CAN_HandleTypeDef* bus, uint32_t id)
{
  SteerTxSample sample;
  sample.id = static_cast<uint16_t>(id);
  bool nonzero = false;
  for (int i = 0; i < 4; ++i) {
    const int value = (int(s_steer_frame[2*i]) << 8) | s_steer_frame[2*i+1];
    sample.raw[i] = static_cast<int16_t>(value < 32768 ? value : value - 65536);
    nonzero |= value != 0;
  }
  if (nonzero) {
    ++s_steer_attempted;
    if (s_steer_receipt.status == robot::CanTxStatus::kIdle) {
      if (!CAN_SendTracked(bus, s_steer_frame, id, 8, s_steer_receipt)) return false;
      s_candidate = sample;
      ++s_steer_queued;
      return true;
    }
  }
  return CAN_Send_Msg(bus, s_steer_frame, id, 8);
}

void CollectSteerReceipt(uint32_t now)
{
  if (s_steer_receipt.status == robot::CanTxStatus::kSent) {
    s_last_txok = s_candidate;
    s_steer_txok_tick = now;
    ++s_steer_confirmed;
    s_steer_receipt.status = robot::CanTxStatus::kIdle;
  } else if (s_steer_receipt.status == robot::CanTxStatus::kUnconfirmed) {
    ++s_steer_unconfirmed;
    s_steer_receipt.status = robot::CanTxStatus::kIdle;
  }
}

void RecordSteerStall(uint32_t now)
{
  const auto* chassis = GetChassis();
  if (chassis == nullptr) return;
  const uint32_t sequence = chassis->lastSteerStall().sequence;
  auto& event = robot::diag::g_steer_stall_tx;
  if (event.sequence == sequence) return;
  const bool recent = s_steer_confirmed != 0 && now - s_steer_txok_tick <= kRecentSteerTxMs;
  event.tick_ms = now;
  event.attempted_nonzero = s_steer_attempted;
  event.queued_nonzero = s_steer_queued;
  event.confirmed_nonzero = s_steer_confirmed;
  event.unconfirmed_nonzero = s_steer_unconfirmed;
  event.has_recent_txok = recent;
  event.frame_id = recent ? s_last_txok.id : 0;
  for (int i = 0; i < 4; ++i) event.command_raw[i] = recent ? s_last_txok.raw[i] : 0;
  event.sequence = sequence; // 最后更新编号，便于核对两份记录属于同一次停机。
}
#endif
#if defined(ROBOT_STEER_MODE_PROBE) && ROBOT_STEER_MODE_PROBE
static_assert(!robot::kEnableFaultInjection, "Do not combine the stand probe with fault injection");
robot::probe::SteerModeProbe s_probe;
bool s_probe_output = false;
bool s_probe_cancel = false;
robot::CanTxReceipt s_probe_receipt;
uint16_t s_probe_receipt_id = 0;

void CollectSteerProbeReceipt()
{
  const auto status = s_probe_receipt.status;
  if (status != robot::CanTxStatus::kSent &&
      status != robot::CanTxStatus::kUnconfirmed) return;
  auto& result = s_probe.resultFor(s_probe_receipt_id);
  if (status == robot::CanTxStatus::kSent) ++result.sent;
  else ++result.unconfirmed;
  s_probe_receipt.status = robot::CanTxStatus::kIdle;
}

/** 复制实时反馈；仅在全部正常且指定舵电机仍为 ID 2 时允许单轮试探。 */
robot::probe::Command SteerProbeStep(robot::Robot* controller, bool can_ok, uint32_t now)
{
  robot::probe::Input input;
  if (controller != nullptr && can_ok && !s_probe_cancel && controller->canTxHealthy() &&
      controller->lifeState() == robot::LifeState::kWorking && RemoteAliveForWatchdog()) {
    auto* motor = GetMotorSteer(1);
    const auto& remote = controller->probeRemote();
    if (motor != nullptr && motor->id() == 2u && motor->isHealthy()) {
      robot::CriticalSection lock;
      input.healthy = remote.online;
      input.estop = controller->isEstopped();
      input.left = remote.left;
      input.right = remote.right;
      input.forward = remote.lv;
      input.other_neutral = remote.lh == 0 && remote.rh == 0 && remote.rv == 0 && remote.wheel == 0;
      input.angle = motor->angle();
      input.current = motor->curr();
    }
  }
  return s_probe.step(input, now);
}

/** 发布完一整个控制周期的只读诊断，急停后仍保留本次结果。 */
void PublishSteerProbe()
{
  robot::probe::g_steer_probe_phase = static_cast<uint8_t>(s_probe.phase);
  for (const bool voltage : {true, false}) {
    const auto& src = voltage ? s_probe.voltage_result : s_probe.current_result;
    auto& dst = voltage ? robot::probe::g_steer_probe_voltage : robot::probe::g_steer_probe_current;
    dst.peak_angle_rad = src.peak_angle_rad;
    dst.peak_delta_a = src.peak_delta_a;
    dst.completed = src.completed;
    dst.attempted = src.attempted;
    dst.queued = src.queued;
    dst.sent = src.sent;
    dst.unconfirmed = src.unconfirmed;
  }
}
#endif
/** 只在输出撤销沿撤销旧队列；取消失败保留请求，下周期继续重试。 */
bool CancelOnStop(robot::Robot* controller)
{
  const bool allowed=controller!=nullptr && controller->outputPermitted();
  if(s_previous_output && !allowed) s_cancel_pending=true;
  s_previous_output=allowed;
  if(!s_cancel_pending) return true;
  const bool first=CAN_AbortPendingTx(&hcan1);
  const bool second=CAN_AbortPendingTx(&hcan2);
  if(first && second) s_cancel_pending=false;
  return first && second;
}

}  // namespace

/* Exported function definitions ---------------------------------------------*/

void CommTaskInit(void)
{
  /* ---------- 1. CAN 滤波器 + 启动 CAN ----------
   * 滤波器「全收」，具体要哪一帧由接收回调里按 ID 分发决定。 */
  CanFilter_Init(&hcan1);
  CanFilter_Init(&hcan2);

  HAL_CAN_Start(&hcan1);
  HAL_CAN_Start(&hcan2);

  /* 打开「收到新帧」中断：CAN1 → FIFO0，CAN2 → FIFO1（与滤波器配置对应） */
  HAL_CAN_ActivateNotification(&hcan1, CAN_IT_RX_FIFO0_MSG_PENDING);
  HAL_CAN_ActivateNotification(&hcan2, CAN_IT_RX_FIFO1_MSG_PENDING);

  /* ---------- 2. 启动 DBUS 接收 ---------- */
  StartDbusReceive();

  /* ---------- 3. 看门狗 ----------
   * IWDG 已经在 MX_IWDG_Init() 里启动（约 200ms 超时）。这里不喂，
   * 第一帧合法 DBUS 到达后、由 CommTask 每 1ms 喂一次。 */
}

/* ============================================================================
 * 通信任务：每个周期做四件事
 * ---------------------------------------------------------------------------
 *   1) 检查 DBUS 是否需要重启接收；
 *   2) 组帧、发 CAN（舵帧、轮帧、双板帧）；
 *   3) 收发送回执、统计失败、处理堵转上报；
 *   4) 最后一行才喂狗——且要求 canFeedWatchdog() 和遥控在线同时成立。
 * 全板只有这一个喂狗点：控制任务卡死时走不到这里，200 ms 后看门狗复位。
 * ========================================================================== */
void CommTask(void)
{
  StartDbusReceive();
  // 不依赖错误中断是否开启，直接读取控制器的 bus-off 状态。
  robot::Robot* controller=GetRobot();
  // 两路都轮询，不能用短路表达式让第二条总线的旧邮箱无人监督。
  const bool tx_first=CAN_TxHealthy(&hcan1);
  const bool tx_second=CAN_TxHealthy(&hcan2);
  bool cycle_ok=tx_first && tx_second;
  const bool failure_reported=controller && !cycle_ok;
  if(failure_reported) controller->notifyCanTxCycle(false); // 编码前先清零
  // 控制仍为1kHz；电机500Hz（两板各用本地时钟，不保证物理错相）、状态200Hz，避免共享总线过载。
  const uint32_t now = hello_world::tick::GetTickMs();
  const bool stop_edge = s_previous_output && (controller == nullptr || !controller->outputPermitted());
#if defined(ROBOT_STEER_MODE_PROBE) && ROBOT_STEER_MODE_PROBE
  CollectSteerProbeReceipt();
  const auto probe_command = SteerProbeStep(controller, cycle_ok, now);
  const bool probe_stop = s_probe_output && probe_command.raw == 0u;
  s_probe_output = probe_command.raw != 0u;
  if (probe_stop) s_probe_cancel = true;
  if (s_probe_cancel) {
    s_probe_cancel = !CAN_AbortPendingTx(robot::wiring::Can(robot::wiring::kChassisSteer));
    if (s_probe_cancel) cycle_ok = false;
  }
  const bool motor_due = (now % 2u) == 0u || stop_edge || probe_stop;
#else
  // 在撤销邮箱和写入零帧之前，保存停机前的发送证据。
  CollectSteerReceipt(now);
  RecordSteerStall(now);
  const bool motor_due = (now % 2u) == 0u || stop_edge;
#endif
  if(!CancelOnStop(controller)) cycle_ok=false;
  /* ---------- 故障注入：占满 CAN 发送邮箱 ----------
   * 连发 3 帧把 3 个邮箱占满，本周期真正的控制帧就会发送失败，
   * s_can_tx_fail_count 会增长。用来验证审计 H4 的"发送失败可检测"。 */
  if (robot::FaultActive(robot::FaultId::kCanTxStarve)) {
    static uint8_t dummy[8] = {0};
    for (int i = 0; i < 3; ++i) {
      (void)CAN_Send_Msg(&hcan1, dummy, 0x7F0u, 8);
    }
  }

  static bool last_motor_submit_ok = true;
  if (motor_due) {
    bool motor_cycle_ok = true;
    // 停机或旧帧撤销失败时，直接发送零帧，不编码尚可见的旧电机命令。
    [[maybe_unused]] const bool pack_motor_commands =
        cycle_ok && controller && controller->outputPermitted();
    /* ---------- 1. 下发驱动电机（4 台共用 0x200） ----------
     * encode() 只会往缓冲区里「自己那 2 个字节」写数据（索引 = (id-1)%4），
     * 所以 4 台电机依次编码到同一块缓冲区，最后发一帧即可。 */
    for (uint8_t i = 0; i < 8; ++i) {
      s_wheel_frame[i] = 0;
    }
    hello_world::motor::Motor* wheel0 = GetMotorWheel(0);
    if (wheel0 == nullptr) motor_cycle_ok = false;
    if (wheel0 != nullptr) {
#if !defined(ROBOT_STEER_MODE_PROBE) || !ROBOT_STEER_MODE_PROBE
      for (int i = 0; i < 4; ++i) {
        hello_world::motor::Motor* motor = GetMotorWheel(i);
        if (motor != nullptr) {
          if (pack_motor_commands && !EncodeMotor(motor, s_wheel_frame)) motor_cycle_ok = false;
        } else { motor_cycle_ok = false; }
      }
#endif
      if (!CAN_Send_Msg(robot::wiring::Can(robot::wiring::kChassisWheel), s_wheel_frame,
                        hello_world::motor::M3508::GetTxId(wheel0->id()), 8)) {
        ++s_can_tx_fail_count;
        motor_cycle_ok = false;
      }
    }

    /* ---------- 2. 下发转向电机（4 台共用配置对应的电压帧或电流帧） ---------- */
#if defined(ROBOT_STEER_MODE_PROBE) && ROBOT_STEER_MODE_PROBE
    // 只发当前选择的协议。空闲时交替发两种零帧；脉冲结束当轮立刻发两种零帧。
    const auto send_probe = [&](const robot::probe::Command& command) {
      command.encode(s_steer_frame);
      auto* bus = robot::wiring::Can(robot::wiring::kChassisSteer);
      bool ok;
      if (command.raw == 0) {
        ok = CAN_Send_Msg(bus, s_steer_frame, command.id, 8);
      } else {
        auto& result = s_probe.resultFor(command.id);
        ++result.attempted;
        ok = CAN_SendTracked(bus, s_steer_frame, command.id, 8, s_probe_receipt);
        if (ok) {
          ++result.queued;
          s_probe_receipt_id = command.id;
        }
      }
      if (!ok) {
        ++s_can_tx_fail_count;
        motor_cycle_ok = false;
      }
    };
    if (s_probe_output && motor_cycle_ok && cycle_ok &&
        s_probe_receipt.status != robot::CanTxStatus::kPending) {
      send_probe(probe_command);
    } else if (probe_stop) {
      send_probe({0x1FF, 0});
      send_probe({0x1FE, 0});
    } else if (!s_probe_output) {
      send_probe({static_cast<uint16_t>((now & 2u) ? 0x1FF : 0x1FE), 0});
    }
#else
    for (uint8_t i = 0; i < 8; ++i) {
      s_steer_frame[i] = 0;
    }
    hello_world::motor::Motor* steer0 = GetMotorSteer(0);
    if (steer0 == nullptr) motor_cycle_ok = false;
    if (steer0 != nullptr) {
      for (int i = 0; i < 4; ++i) {
        hello_world::motor::Motor* motor = GetMotorSteer(i);
        if (motor != nullptr) {
          if (pack_motor_commands && !EncodeMotor(motor, s_steer_frame)) motor_cycle_ok = false;
        } else { motor_cycle_ok = false; }
      }
      if (!SendSteerFrame(robot::wiring::Can(robot::wiring::kChassisSteer),
                          hello_world::motor::GM6020::GetTxId(steer0->id(),
                                                             steer0->get_input_type()))) {
        ++s_can_tx_fail_count;
        motor_cycle_ok = false;
      }
    }
#endif

    last_motor_submit_ok = motor_cycle_ok;
  }
  cycle_ok = cycle_ok && last_motor_submit_ok;

  /* ---------- 3. 双板通信帧 ---------- */
  robot::GimbalChassisComm* comm = GetGimbalChassisComm();
  if (comm == nullptr) cycle_ok = false;
  static bool retry_status = false;
  static uint8_t last_queued_status = 0;
  static int8_t last_queued_yaw_direction = 0;
  const auto* state = comm != nullptr ? &comm->chassis_data() : nullptr;
  const uint8_t status = state == nullptr ? 0u : uint8_t(state->working | (state->estop<<1) |
      (state->dbus_online<<2) | (state->gimbal_enable<<3));
  const float yaw_rate = state == nullptr ? 0.0f : state->yaw_rate;
  const int8_t yaw_direction = yaw_rate > 0.0f ? 1 : (yaw_rate < 0.0f ? -1 : 0);
  // 授权变化和 Yaw 停止/反向指令不能延后；例行状态碰到邮箱满则下一拍重试。
  const bool required = state != nullptr &&
      (status != last_queued_status ||
       (last_queued_yaw_direction != 0 && yaw_direction != last_queued_yaw_direction) ||
       !std::isfinite(yaw_rate) || !std::isfinite(state->pitch_target));
  if (comm != nullptr && ((now % 5u) == 0u || required || retry_status)) {
    comm->encode(s_comm_frame);
    using robot::CanTxSubmitResult;
    const auto result = CAN_TrySend(robot::wiring::Can(robot::wiring::kChassisPeer),
                                    s_comm_frame, comm->tx_id(), sizeof(s_comm_frame));
    retry_status = result != CanTxSubmitResult::kQueued;
    if (retry_status) {
      ++s_can_tx_fail_count;
      if (required || result != CanTxSubmitResult::kBusy) cycle_ok = false;
    } else {
      last_queued_status = status;
      last_queued_yaw_direction = yaw_direction;
    }
  }

  /* ---------- 4. 看门狗：整个周期都跑完了才喂 ----------
   * 这一行在 MainTask()（控制、逆解、闭环、下发）之后执行。
   * 控制循环卡住 → 走不到这里 → 不喂 → 200ms 后复位。
   * 全板**只有这一个**喂狗调用点。 */
  if(controller && !failure_reported) controller->notifyCanTxCycle(cycle_ok);
  if(!CancelOnStop(controller) && controller && !failure_reported && cycle_ok)
    controller->notifyCanTxCycle(false);
#if defined(ROBOT_STEER_MODE_PROBE) && ROBOT_STEER_MODE_PROBE
  CollectSteerProbeReceipt();
  PublishSteerProbe();
#endif
  // 连续 10 ms 失败停止喂狗；恢复前仅尝试零输出。HAL_OK 不等于送达。
  const bool should_feed = kDebugAlwaysFeedWatchdog ||
      (controller != nullptr && controller->canFeedWatchdog() && RemoteAliveForWatchdog());
  if (should_feed) {
    HAL_IWDG_Refresh(&hiwdg);
  }
}

/* Private function definitions ----------------------------------------------*/

/**
 * @brief UART 空闲中断回调：DBUS 一帧收完时进入
 * @note  必须用 extern "C"，否则中断向量表找不到这个符号。
 *        这里**不喂狗**，只记录「收到了一帧合法数据」。
 */
extern "C" void HAL_UARTEx_RxEventCallback(UART_HandleTypeDef* huart,
                                           uint16_t Size)
{
  if (huart != &huart3 || HAL_UARTEx_GetRxEventType(huart) == HAL_UART_RXEVENT_HT) return;
  if (huart == &huart3 && Size == hello_world::remote_control::kRcRxDataLen) {
    /* ---------- 故障注入：把拨轮两个字节改成 0xFC00 ----------
     * 低 11 位仍是 1024（正常中位），高 5 位是脏的。
     * 期望：校验不通过 → 不解码、不刷新在线时间 → 100ms 后回 Dead，
     *       并且看门狗在 200ms 左右复位（因为没喂狗）。
     * 这项专门验证审计 F1：校验器与解码器位宽不一致时会被绕过。 */
    if (robot::FaultActive(robot::FaultId::kInvalidDbusWheel)) {
      s_dbus_buf[16] = 0x00;
      s_dbus_buf[17] = 0xFC;
    }

    /* 先校验，再解码。非法帧一律当作「没收到」。 */
    if (robot::IsDbusFrameValid(s_dbus_buf)) {
      hello_world::remote_control::DT7* rc = GetRemoteControl();
      if (rc != nullptr) {
        rc->decode(s_dbus_buf);
      }
      robot::Robot* robot_ptr = GetRobot();
      if (robot_ptr != nullptr) {
        robot_ptr->notifyRemoteRx();
      }
      s_last_valid_dbus_tick = hello_world::tick::GetTickMs();
      s_dbus_received = true;
      s_dbus_valid_frames = s_dbus_valid_frames + 1u;
    } else {
      s_dbus_invalid_frames = s_dbus_invalid_frames + 1u;
    }
  }

  /* 重新启动下一轮接收。不重启的话只能收到第一帧。 */
  if (Size != hello_world::remote_control::kRcRxDataLen)
    s_dbus_partial_frames = s_dbus_partial_frames + 1u;
  s_dbus_restart_pending = true;
  StartDbusReceive();
}

/** UART/DMA错误不能永久关闭接收；HAL结束错误传输后由下一控制周期重试。 */
extern "C" void HAL_UART_ErrorCallback(UART_HandleTypeDef* huart)
{
  if (huart != &huart3) return;
  s_dbus_uart_errors = s_dbus_uart_errors + 1u;
  s_dbus_received = false;
  s_dbus_restart_pending = true;
}
