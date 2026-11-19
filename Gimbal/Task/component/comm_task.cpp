/**
 *******************************************************************************
 * @file      :comm_task.cpp
 * @brief     : 云台板通信任务（DBUS 接收 + CAN 发送 + 看门狗）
 * @history   :
 *  Version     Date            Author          Note
 *  V1.0.0      2026-10-19      <author>        1. 完成编写
 *  V1.1.0      2026-10-20      <author>        2. 审计修复：喂狗改为「控制循环跑完才喂」；
 *                                                非法 DBUS 帧不再视为遥控在线
 *******************************************************************************
 * @attention :
 *  【喂狗点只有一个：CommTask 的最后一行】
 *  V1.0 曾在 UART 中断和 CAN 解帧处各喂一次，问题是这两处都不代表「控制循环
 *  真的跑完了」——控制卡死时它们照样能喂，看门狗就失效了。
 *  现在中断只记录「收到过合法数据」的时间戳，喂狗统一放在 CommTask 末尾，
 *  而 CommTask 由 TIM6 在 MainTask() 之后调用。
 *
 *  【本板能不能喂狗，取决于两件事之一】
 *    1. 本板自己的遥控器在线（如果接收机接在云台板上）；
 *    2. 底盘板的 C2G 帧说它那边的遥控器在线。
 *  第 2 条就是考核资料里「上板在解包下板信息处调用看门狗刷新函数」的落地方式：
 *  底盘板一死，云台板收不到「遥控器在线」的帧，200ms 后跟着复位，不会出现
 *  「一半活着」的僵尸车。
 *
 *  ⚠️ 副作用：遥控器没开机时，两板都会每 200ms 复位一次（预期行为）。
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
/* Private macro -------------------------------------------------------------*/
/* Private constants ---------------------------------------------------------*/

namespace {

/**
 * @brief 调试模式：无条件喂狗
 * @note  **正常情况下必须是 false**（只有控制周期跑完 + 遥控数据新鲜才喂）。
 *        台架上需要连着调试器、又不接遥控器时，可以临时改成 true —— 此时
 *        看门狗不再起保护作用，**绝不能用于实车控制**，验收前必须改回来。
 */
constexpr bool kDebugAlwaysFeedWatchdog = false;
#if (defined(ROBOT_GROUND_GIMBAL_LOCK) && ROBOT_GROUND_GIMBAL_LOCK) || \
    (defined(ROBOT_COMPETITION_FINISH) && ROBOT_COMPETITION_FINISH)
static_assert(!kDebugAlwaysFeedWatchdog, "Keep watchdog supervision active");
#endif

/** 遥控数据超过这个时间没更新，就认为「主人失联」，不再喂狗（ms） */
constexpr uint32_t kDbusTimeoutMs = 100u;

/** DBUS 接收缓冲区 */
uint8_t s_dbus_buf[2u * hello_world::remote_control::kRcRxDataLen];

/** 最近一帧「合法」DBUS 的时间戳（只由 UART 中断写） */
volatile uint32_t s_last_valid_dbus_tick = 0;
volatile bool s_dbus_received = false;
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
    // 36 字节缓冲的半满恰好是 18 字节，但它不能代替一帧结束的空闲边界。
    __HAL_DMA_DISABLE_IT(huart3.hdmarx, DMA_IT_HT);
    s_dbus_restart_pending = false;
  } else {
    // 启动失败时保留 pending，由后续 CommTask 重试，避免一次错误永久关闭遥控。
    s_dbus_rx_start_failures = s_dbus_rx_start_failures + 1u;
  }
}


/** CAN 发送失败计数 */
uint32_t s_can_tx_fail_count = 0;

/** CAN 发送缓冲 */
uint8_t s_yaw_frame[8];
uint8_t s_pitch_frame[8];
uint8_t s_comm_frame[8];

/**
 * @brief 在短临界区内读取电机输入并编码，避免与接收侧更新发生交错。
 */
bool EncodeMotor(hello_world::motor::Motor* motor, uint8_t* frame)
{
  robot::CriticalSection lock;
  return motor != nullptr && motor->encode(frame);
}

/** 发一帧达妙电机控制报文（MIT 模式发送 ID = 电机 ID） */
bool SendDamiaoFrame(CAN_HandleTypeDef* bus, hello_world::motor::Motor* motor, uint8_t* frame,
                     bool allowed, bool enable_requested)
{
  if (motor == nullptr) {
    return false;
  }
  using hello_world::motor::DaMiao;
  // 未获许可时反复送失能；获得许可但驱动尚未确认使能时送使能命令，
  // 只有握手完成后才编码正常力矩帧。
  if (!allowed || enable_requested) {
    // 失能必须是驱动命令，不能用「MIT 力矩填 0」代替。
    for (uint8_t i = 0; i < 7; ++i) frame[i] = 0xFFu;
    frame[7] = allowed ? DaMiao::kEnable : DaMiao::kDisable;
  } else {
    for (uint8_t i = 0; i < 8; ++i) frame[i] = 0;
    if (!EncodeMotor(motor, frame)) return false;
  }
  return CAN_Send_Msg(bus, frame,
                      hello_world::motor::DM_J4310::GetTxId(motor->id()), 8);
}

/**
 * @brief 遥控数据是否足够新（决定这一周期能不能喂狗）
 * @note  本板遥控器在线，或者底盘板说它的遥控器在线。
 */
bool RemoteAliveForWatchdog()
{
  robot::CriticalSection lock;
  const bool local =
      s_dbus_received &&
      ((hello_world::tick::GetTickMs() - s_last_valid_dbus_tick) <= kDbusTimeoutMs);

  robot::GimbalChassisComm* comm = GetGimbalChassisComm();
  const bool peer = (comm != nullptr) && !comm->isOffline() &&
                    comm->chassis_data().dbus_online;

  return local || peer;
}

bool s_previous_output=false;
bool s_can1_cancel_pending=false;
bool s_can2_cancel_pending=false;
/** 两路各自撤销旧队列；不能因一条总线失败而把另一条的新失能帧反复撤销。 */
bool CancelOnStop(robot::Robot* controller)
{
  const bool allowed=controller!=nullptr && controller->outputPermitted();
  if(s_previous_output && !allowed) s_can1_cancel_pending=s_can2_cancel_pending=true;
  s_previous_output=allowed;
  if(s_can1_cancel_pending) s_can1_cancel_pending=!CAN_AbortPendingTx(&hcan1);
  if(s_can2_cancel_pending) s_can2_cancel_pending=!CAN_AbortPendingTx(&hcan2);
  return !s_can1_cancel_pending && !s_can2_cancel_pending;
}

}  // namespace

/* Exported function definitions ---------------------------------------------*/

/**
 * @brief 启动两路 CAN 接收与 USART3 空闲 DMA 接收，实际喂狗留在周期通信末尾。
 */
void CommTaskInit(void)
{
  /* ---------- 1. CAN 滤波器 + 启动 ---------- */
  CanFilter_Init(&hcan1);
  CanFilter_Init(&hcan2);

  HAL_CAN_Start(&hcan1);
  HAL_CAN_Start(&hcan2);

  HAL_CAN_ActivateNotification(&hcan1, CAN_IT_RX_FIFO0_MSG_PENDING);
  HAL_CAN_ActivateNotification(&hcan2, CAN_IT_RX_FIFO1_MSG_PENDING);

  /* ---------- 2. DBUS 接收 ---------- */
  StartDbusReceive();
}

/**
 * @brief 监督上一批发送结果，提交两轴帧、转发遥控分片并发布本板状态。
 * @note  急停/失联等关键状态优先提交，普通状态可重试，末尾再汇总结果决定喂狗。
 */
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
  const bool motor_due = (now % 2u) == 1u || stop_edge;
  if(!CancelOnStop(controller)) cycle_ok=false;
  /* ---------- 故障注入：占满 CAN2 发送邮箱（达妙在 CAN2） ---------- */
  if (robot::FaultActive(robot::FaultId::kCanTxStarve)) {
    static uint8_t dummy[8] = {0};
    for (int i = 0; i < 3; ++i) {
      (void)CAN_Send_Msg(&hcan2, dummy, 0x7F1u, 8);
    }
  }
  // 电机每两拍发送一次；未发送的那一拍沿用上次提交结果，不凭空覆盖一次失败。
  static bool last_motor_submit_ok = true;
  if (motor_due) {
    bool motor_cycle_ok = true;
    const bool motor_allowed = cycle_ok && controller != nullptr && controller->outputPermitted();
    /* ---------- 1. 两个达妙电机：各自一帧，ID 不同，不能合并 ---------- */
    if (!SendDamiaoFrame(robot::wiring::Can(robot::wiring::kGimbalYaw), GetMotorYaw(), s_yaw_frame,
                         motor_allowed, motor_allowed && controller->motorEnableRequested(0))) {
      ++s_can_tx_fail_count;
      motor_cycle_ok = false;
    }
    // 同拍 yaw 提交失败后，pitch 改送失能，避免继续扩展本拍的正常出力。
    const bool pitch_allowed = motor_allowed && motor_cycle_ok;
    if (!SendDamiaoFrame(robot::wiring::Can(robot::wiring::kGimbalPitch), GetMotorPitch(), s_pitch_frame,
                         pitch_allowed, pitch_allowed && controller->motorEnableRequested(1))) {
      ++s_can_tx_fail_count;
      motor_cycle_ok = false;
    }

    last_motor_submit_ok = motor_cycle_ok;
  }
  cycle_ok = cycle_ok && last_motor_submit_ok;

  /* ---------- 2. 双板通信帧（走 CAN1） ---------- */
  robot::GimbalChassisComm* comm = GetGimbalChassisComm();
  if (comm == nullptr) cycle_ok = false;
  static bool retry_status = false;
  // 以下 status 仅作本地变化检测，其位布局与协议 encode 的 flags 字节不同。
  static uint8_t last_queued_status = 0;
  const auto* state = comm != nullptr ? &comm->gimbal_data() : nullptr;
  const uint8_t status = state == nullptr ? 0u : uint8_t(state->working | (state->estop<<1) |
      (state->dbus_online<<2) | (state->ground_disarmed<<3));
  // 急停、失联等状态变化须即时提交；例行状态错过一次仍由收端监督新鲜度。
  const bool required = state != nullptr && (status != last_queued_status ||
      !std::isfinite(state->yaw_joint) || !std::isfinite(state->pitch_joint));
  const auto send_status = [&] {
    comm->encode(s_comm_frame);
    using robot::CanTxSubmitResult;
    const auto result = CAN_TrySend(robot::wiring::Can(robot::wiring::kGimbalPeer),
                                    s_comm_frame, comm->tx_id(), sizeof(s_comm_frame));
    retry_status = result != CanTxSubmitResult::kQueued;
    if (retry_status) {
      ++s_can_tx_fail_count;
      if (required || result != CanTxSubmitResult::kBusy) cycle_ok = false;
    } else {
      last_queued_status = status;
    }
  };
  if (required) send_status();

  // 每周期最多转发一片，避免一次占满共享CAN的三个邮箱。
  uint8_t relay_frame[8]; uint32_t relay_id = 0; bool relay_pending = false;
  if (comm != nullptr) {
    robot::CriticalSection lock;
    relay_pending = comm->remote_relay().peekTx(hello_world::tick::GetTickMs(), relay_id, relay_frame);
  }
  // 只有当前片真正入邮箱后才推进片号；UART 新帧只更新待发布缓存。
  if (relay_pending) {
    if (CAN_Send_Msg(robot::wiring::Can(robot::wiring::kGimbalPeer), relay_frame, relay_id, 8)) {
      robot::CriticalSection lock;
      comm->remote_relay().txAccepted();
    } else {
      ++s_can_tx_fail_count; cycle_ok = false;
    }
  }
  // 按期状态是可重试的；与真实遥控分片同时到期时，让分片先占用剩余邮箱。
  if (comm != nullptr && !required && ((now % 5u) == 2u || retry_status)) send_status();

  /* ---------- 3. 看门狗：整个周期都跑完了才喂 ----------
   * 全板**只有这一个**喂狗调用点。 */
  // 本拍硬件轮询失败已在编码前报告；其余失败在末尾汇总，避免同一拍重复累计。
  if(controller && !failure_reported) controller->notifyCanTxCycle(cycle_ok);
  // 汇总失败可能刚撤销了授权，因此再检查停机边沿，及时撤销尚在邮箱中的旧帧。
  if(!CancelOnStop(controller) && controller && !failure_reported && cycle_ok)
    controller->notifyCanTxCycle(false);
  // 连续 10 ms 失败停止喂狗；恢复前仅尝试零输出。HAL_OK 不等于送达。
  const bool should_feed = kDebugAlwaysFeedWatchdog ||
      (controller != nullptr && controller->canFeedWatchdog() && RemoteAliveForWatchdog());
  if (should_feed) {
    HAL_IWDG_Refresh(&hiwdg);
  }
}

/* Private function definitions ----------------------------------------------*/

/**
 * @brief 处理 USART3 DMA 接收事件，仅将长度为 18 字节的合法数据提交给遥控器。
 * @note  extern "C" 与 HAL 的 C 链接回调名保持一致，该函数由 HAL 的 IRQ 流程调用。
 *        这里**不喂狗**，只记录「收到了一帧合法数据」。
 */
extern "C" void HAL_UARTEx_RxEventCallback(UART_HandleTypeDef* huart,
                                           uint16_t Size)
{
  // 其他串口和半满事件各自处理；半满时继续接收，不能在这里抢先重启 DMA。
  if (huart != &huart3 || HAL_UARTEx_GetRxEventType(huart) == HAL_UART_RXEVENT_HT) return;
  if (huart == &huart3 && Size == hello_world::remote_control::kRcRxDataLen) {
    /* ---------- 故障注入：把拨轮两个字节改成 0xFC00 ----------
     * 期望：校验不通过 → 不解码、不刷新在线时间（详见 docs/08）。 */
    if (robot::FaultActive(robot::FaultId::kInvalidDbusWheel)) {
      s_dbus_buf[16] = 0x00;
      s_dbus_buf[17] = 0xFC;
    }

    if (robot::IsDbusFrameValid(s_dbus_buf)) {
      hello_world::remote_control::DT7* rc = GetRemoteControl();
      if (rc != nullptr) {
        rc->decode(s_dbus_buf);
      }
      robot::Robot* robot_ptr = GetRobot();
      if (robot_ptr != nullptr) {
        robot_ptr->notifyRemoteRx();
      }
      // 只有真实UART合法帧才能产生新的转发序号，状态心跳不能替代摇杆新鲜度。
      auto* comm = GetGimbalChassisComm();
      if (comm != nullptr) comm->remote_relay().publish(s_dbus_buf, hello_world::tick::GetTickMs());
      s_last_valid_dbus_tick = hello_world::tick::GetTickMs();
      s_dbus_received = true;
      s_dbus_valid_frames = s_dbus_valid_frames + 1u;
    } else {
      s_dbus_invalid_frames = s_dbus_invalid_frames + 1u;
    }
  }

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
