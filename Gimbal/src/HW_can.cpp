/**
 *******************************************************************************
 * @file      : HW_can.cpp
 * @brief     : CAN 收发基础封装实现 + 接收分发（云台板）
 * @history   :
 *  Version     Date            Author          Note
 *  V1.0.0      2026-09-18      Hello World     1. 考核骨架：CanFilter_Init + CAN_Send_Msg
 *  V1.1.0      2026-10-19      <author>        1. 补全按 ID 分发的路由
 *******************************************************************************
 * @attention :
 *  与底盘板的区别：本板只有两个达妙电机（反馈 ID 0x11 / 0x12）和一帧双板通信。
 *  ⚠️ 达妙的反馈 ID 是「ID + 0x10」，不是 0x200 系列，别照抄底盘板的判断。
 *******************************************************************************
 *  Copyright (c) 2024 Hello World Team, Zhejiang University.
 *  All Rights Reserved.
 *******************************************************************************
 */
/* Includes ------------------------------------------------------------------*/
#include "HW_can.hpp"

#include "can.h"
#include "gimbal_chassis_comm.hpp"
#include "ins_all.hpp"
#include "motor.hpp"
#include "robot.hpp"
#include "tick.hpp"
#include "can_tx_monitor.hpp"
#include "critical_section.hpp"

/* Private constants ---------------------------------------------------------*/
/* Private variables ---------------------------------------------------------*/
static CAN_RxHeaderTypeDef rx_header1;
static CAN_RxHeaderTypeDef rx_header2;
static uint8_t can1_rx_data[8];
static uint8_t can2_rx_data[8];
static robot::CanTxMonitor s_tx_monitors[2];
static robot::CanTxMonitor* TxMonitor(CAN_HandleTypeDef* hcan) {
  if(hcan==&hcan1) return &s_tx_monitors[0];
  if(hcan==&hcan2) return &s_tx_monitors[1];
  return nullptr;
}

/* Private function prototypes -----------------------------------------------*/
static void DispatchCanFrame(uint32_t std_id, const uint8_t data[8]);

/* Exported function definitions ---------------------------------------------*/

void CanFilter_Init(CAN_HandleTypeDef *hcan)
{
  CAN_FilterTypeDef can_filter = {};
  // F407两路CAN共享滤波器，0~13归CAN1、14~27归CAN2；禁止未初始化分界。
  can_filter.SlaveStartFilterBank = 14;

  /* 全收：掩码全 0，任何 ID 均通过 */
  can_filter.FilterMode = CAN_FILTERMODE_IDMASK;
  can_filter.FilterScale = CAN_FILTERSCALE_32BIT;
  can_filter.FilterIdHigh = 0x0000;
  can_filter.FilterIdLow = 0x0000;
  can_filter.FilterMaskIdHigh = 0x0000;
  can_filter.FilterMaskIdLow = 0x0000;
  can_filter.FilterActivation = ENABLE;

  if (hcan == &hcan1) {
    can_filter.FilterFIFOAssignment = CAN_FILTER_FIFO0;
    can_filter.FilterBank = 0;
  } else if (hcan == &hcan2) {
    can_filter.FilterFIFOAssignment = CAN_FILTER_FIFO1;
    can_filter.FilterBank = 14;
  } else {
    return;
  }

  if (HAL_CAN_ConfigFilter(hcan, &can_filter) != HAL_OK) {
    Error_Handler();
  }
}

bool CAN_Is_BusOff(const CAN_HandleTypeDef *hcan)
{
  return hcan == nullptr || (hcan->Instance->ESR & CAN_ESR_BOFF) != 0u;
}

bool CAN_AbortPendingTx(CAN_HandleTypeDef* hcan)
{
  auto* monitor=TxMonitor(hcan);
  if(!monitor) return false;
  const bool ok=HAL_CAN_AbortTxRequest(hcan,CAN_TX_MAILBOX0|CAN_TX_MAILBOX1|CAN_TX_MAILBOX2)==HAL_OK;
  // HAL_OK只表示写入撤销请求；硬件全部邮箱空闲后才可忘记旧帧。
  const uint32_t empty=CAN_TSR_TME0|(CAN_TSR_TME0<<1)|(CAN_TSR_TME0<<2);
  const bool completed=ok && (hcan->Instance->TSR&empty)==empty;
  if(completed) monitor->reset();
  return completed;
}

bool CAN_TxHealthy(CAN_HandleTypeDef* hcan)
{
  auto* monitor=TxMonitor(hcan);
  if(!monitor) return false;
  const uint32_t tsr=hcan->Instance->TSR;
  uint8_t pending=0,tx_ok=0;
  for(int i=0;i<3;++i) {
    if(!(tsr&(CAN_TSR_TME0<<i))) pending|=1u<<i;
    if(tsr&(CAN_TSR_TXOK0<<(8*i))) tx_ok|=1u<<i;
  }
  const uint8_t completed=monitor->tracked()&uint8_t(~pending);
  const bool ok=monitor->poll(pending,tx_ok,hello_world::tick::GetTickMs()) && !CAN_Is_BusOff(hcan);
  // RQCP写1清除完成/错误标志，防止旧TXOK污染下一次相同邮箱的结果。
  uint32_t clear=0;
  for(int i=0;i<3;++i) if(completed&(1u<<i)) clear|=1u<<(8*i);
  if(clear) hcan->Instance->TSR=clear; // RQCP为W1C；TME为只读，不改待发请求
  if(!ok) (void)CAN_AbortPendingTx(hcan);
  return ok;
}

bool CAN_Send_Msg(CAN_HandleTypeDef *hcan, uint8_t *msg, uint32_t id,
                  uint8_t len)
{
  if (!msg || len > 8 || id > 0x7FFu) return false;
  robot::CriticalSection lock;
  if (!CAN_TxHealthy(hcan)) return false;
  if (hcan->State != HAL_CAN_STATE_READY && hcan->State != HAL_CAN_STATE_LISTENING)
    return false;
  auto* monitor = TxMonitor(hcan);
  const uint32_t empty = hcan->Instance->TSR;
  for (int slot = 0; slot < 3; ++slot) {
    const uint8_t mask = uint8_t(1u << slot);
    if (!(empty & (CAN_TSR_TME0 << slot)) || (monitor->tracked() & mask)) continue;
    // 只选“硬件已空且软件已收回完成结果”的槽。HAL按实时CODE选槽时，
    // 旧请求可能恰好完成并被HAL清掉TXOK，形成偶发误保护或遗漏失败。
    // 以下标准帧寄存器顺序与STM32 HAL AddTxMessage一致，但槽位由监督器保留。
    auto& mailbox = hcan->Instance->sTxMailBox[slot];
    hcan->Instance->TSR = 1u << (8 * slot); // 仅清本槽旧RQCP（W1C）
    mailbox.TIR = id << CAN_TI0R_STID_Pos;
    mailbox.TDTR = len;
    mailbox.TDLR = uint32_t(msg[0]) | (uint32_t(msg[1]) << 8) |
        (uint32_t(msg[2]) << 16) | (uint32_t(msg[3]) << 24);
    mailbox.TDHR = uint32_t(msg[4]) | (uint32_t(msg[5]) << 8) |
        (uint32_t(msg[6]) << 16) | (uint32_t(msg[7]) << 24);
    monitor->submitted(mask, hello_world::tick::GetTickMs()); // 先登记，再允许硬件完成
    mailbox.TIR = uint32_t(mailbox.TIR) | CAN_TI0R_TXRQ;
    return true;
  }
  return false;
}

/**
 * @brief   CAN1 FIFO0 接收中断回调
 * @note    双板通信帧走 CAN1
 */
void HAL_CAN_RxFifo0MsgPendingCallback(CAN_HandleTypeDef *hcan)
{
  if (hcan == &hcan1) {
    if (HAL_CAN_GetRxMessage(hcan, CAN_RX_FIFO0, &rx_header1,
                             can1_rx_data) == HAL_OK) {
      if (rx_header1.IDE == CAN_ID_STD && rx_header1.RTR == CAN_RTR_DATA &&
          rx_header1.DLC == 8u) {
        DispatchCanFrame(rx_header1.StdId, can1_rx_data);
      }
    }
  }
  HAL_CAN_ActivateNotification(hcan, CAN_IT_RX_FIFO0_MSG_PENDING);
}

/**
 * @brief   CAN2 FIFO1 接收中断回调
 * @note    pitch反馈走CAN2；yaw反馈走共享CAN1
 */
void HAL_CAN_RxFifo1MsgPendingCallback(CAN_HandleTypeDef *hcan)
{
  if (hcan == &hcan2) {
    if (HAL_CAN_GetRxMessage(hcan, CAN_RX_FIFO1, &rx_header2,
                             can2_rx_data) == HAL_OK) {
      if (rx_header2.IDE == CAN_ID_STD && rx_header2.RTR == CAN_RTR_DATA &&
          rx_header2.DLC == 8u) {
        DispatchCanFrame(rx_header2.StdId, can2_rx_data);
      }
    }
  }
  HAL_CAN_ActivateNotification(hcan, CAN_IT_RX_FIFO1_MSG_PENDING);
}

/* Private function definitions ----------------------------------------------*/

/**
 * @brief 按标准帧 ID 路由
 * @note  电机槽位：0 = yaw，1 = pitch（与 Robot::notifyMotorRx 约定一致）
 */
static void DispatchCanFrame(uint32_t std_id, const uint8_t data[8])
{
  robot::Robot *robot = GetRobot();

  /* ---- yaw 电机：反馈 ID = id + 0x10 ---- */
  hello_world::motor::Motor *yaw = GetMotorYaw();
  if ((yaw != nullptr) &&
      (hello_world::motor::DM_J4310::GetRxId(yaw->id()) == std_id)) {
    if (yaw->decode(data) && robot != nullptr) {
      robot->notifyMotorRx(0);
    }
    return;
  }

  /* ---- pitch 电机 ---- */
  hello_world::motor::Motor *pitch = GetMotorPitch();
  if ((pitch != nullptr) &&
      (hello_world::motor::DM_J4310::GetRxId(pitch->id()) == std_id)) {
    if (pitch->decode(data) && robot != nullptr) {
      robot->notifyMotorRx(1);
    }
    return;
  }

  /* ---- 双板通信帧（底盘 → 云台，0x210） ----
   * 注意：这里**不喂狗**。喂狗统一在 CommTask 的最后一行（控制周期跑完之后），
   * 否则控制循环卡死时本中断仍会来喂，看门狗就失效了。
   * 本处只负责把数据解出来，CommTask 会读 chassis_data().dbus_online 决定是否喂。 */
  robot::GimbalChassisComm *comm = GetGimbalChassisComm();
  if ((comm != nullptr) && (comm->rx_id() == std_id)) {
    (void)comm->decode(data);
    return;
  }

  /* 其余 ID 丢弃 */
}
