/**
 *******************************************************************************
 * @file      : HW_can.cpp
 * @brief     : CAN 收发基础封装实现 + 接收分发（底盘板）
 * @history   :
 *  Version     Date            Author          Note
 *  V1.0.0      2026-09-18      Hello World     1. 考核骨架：CanFilter_Init + CAN_Send_Msg
 *  V1.1.0      2026-10-19      <author>        1. 补全按 ID 分发的路由
 *******************************************************************************
 * @attention :
 *  【CAN 接收回调里为什么不能干重活】
 *  中断回调是「插队执行」的：它运行时控制循环停着。所以这里只做最短路径的
 *  事情——比对 ID、拷贝 8 字节进电机对象、记一下在线时间，然后立刻返回。
 *  任何解算、打印、等待都不能放进来。
 *
 *  【为什么两路 CAN 的帧走同一个分发函数】
 *  实车接线可能调整（哪个电机接哪一路），但帧 ID 是全局唯一的。按 ID 分发
 *  而不是按「哪路 CAN」分发，接线换了代码也不用改，也更不容易漏帧。
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

bool CAN_Send_Msg(CAN_HandleTypeDef *hcan, uint8_t *msg, uint32_t id,
                  uint8_t len)
{
  CAN_TxHeaderTypeDef tx_header = {0};
  tx_header.StdId = id;
  tx_header.IDE = CAN_ID_STD;
  tx_header.RTR = CAN_RTR_DATA;
  tx_header.DLC = len;
  /* 把 HAL 的结果如实返回：邮箱满 / 总线错误时这一帧**没发出去**，
   * 调用方必须知道（尤其是急停时的零指令帧）。 */
  return HAL_CAN_AddTxMessage(hcan, &tx_header, msg, &p_tx_mailbox) == HAL_OK;
}

/**
 * @brief   CAN1 FIFO0 接收中断回调
 * @param   hcan: CAN 句柄
 * @retval  None
 */
void HAL_CAN_RxFifo0MsgPendingCallback(CAN_HandleTypeDef *hcan)
{
  if (hcan == &hcan1) {
    if (HAL_CAN_GetRxMessage(hcan, CAN_RX_FIFO0, &rx_header1,
                             can1_rx_data) == HAL_OK) {
      /* 只接受「标准帧 + 数据帧 + 正好 8 字节」，其它一律丢弃 */
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
 * @param   hcan: CAN 句柄
 * @retval  None
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
 * @brief 按标准帧 ID 把 8 字节路由到对应的电机 / 双板协议
 * @note  查表顺序：先电机（数量多、频率高），再双板帧；都没命中就丢弃。
 *        电机槽位编号与 Robot::notifyMotorRx 的约定一致：
 *        0~3 = 驱动轮，4~7 = 舵电机。
 */
static void DispatchCanFrame(uint32_t std_id, const uint8_t data[8])
{
  robot::Robot *robot = GetRobot();

  /* ---- 驱动电机 M3508：反馈 ID = 0x200 + id ---- */
  for (int i = 0; i < 4; ++i) {
    hello_world::motor::Motor *motor = GetMotorWheel(i);
    if (motor == nullptr) {
      continue;
    }
    if (hello_world::motor::M3508::GetRxId(motor->id()) == std_id) {
      /* 只有解码成功（长度/字段合法）才算「这帧有效」，才刷新在线时间 */
      if (motor->decode(data) && robot != nullptr) {
        robot->notifyMotorRx(i);
      }
      return;
    }
  }

  /* ---- 转向电机 GM6020：反馈 ID = 0x204 + id ---- */
  for (int i = 0; i < 4; ++i) {
    hello_world::motor::Motor *motor = GetMotorSteer(i);
    if (motor == nullptr) {
      continue;
    }
    if (hello_world::motor::GM6020::GetRxId(motor->id()) == std_id) {
      if (motor->decode(data) && robot != nullptr) {
        robot->notifyMotorRx(4 + i);
      }
      return;
    }
  }

  /* ---- 双板通信帧（云台 → 底盘，0x211） ----
   * decode 内部会校验 CRC 并自己记录成功/失败计数；返回值在这里不额外处理。 */
  robot::GimbalChassisComm *comm = GetGimbalChassisComm();
  if ((comm != nullptr) && (comm->rx_id() == std_id)) {
    (void)comm->decode(data);
    return;
  }

  /* 其余 ID 一律丢弃（总线上还有别的设备，不认识的帧不要碰） */
}
