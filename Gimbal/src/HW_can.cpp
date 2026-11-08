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

/* Private constants ---------------------------------------------------------*/
/* Private variables ---------------------------------------------------------*/
static CAN_RxHeaderTypeDef rx_header1;
static CAN_RxHeaderTypeDef rx_header2;
static uint8_t can1_rx_data[8];
static uint8_t can2_rx_data[8];
static uint32_t p_tx_mailbox;

/* Private function prototypes -----------------------------------------------*/
static void DispatchCanFrame(uint32_t std_id, const uint8_t data[8]);

/* Exported function definitions ---------------------------------------------*/

void CanFilter_Init(CAN_HandleTypeDef *hcan)
{
  CAN_FilterTypeDef can_filter;

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

bool CAN_Send_Msg(CAN_HandleTypeDef *hcan, uint8_t *msg, uint32_t id,
                  uint8_t len)
{
  CAN_TxHeaderTypeDef tx_header = {0};
  tx_header.StdId = id;
  tx_header.IDE = CAN_ID_STD;
  tx_header.RTR = CAN_RTR_DATA;
  tx_header.DLC = len;
  /* 邮箱满 / 总线错误时这一帧**没发出去**。达妙是"不收到新指令就保持上一帧"，
   * 所以发送失败必须让调用方知道（急停时的零力矩帧尤为关键）。 */
  return HAL_CAN_AddTxMessage(hcan, &tx_header, msg, &p_tx_mailbox) == HAL_OK;
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
 * @note    达妙电机反馈走 CAN2
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
