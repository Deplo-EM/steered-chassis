/**
 *******************************************************************************
 * @file      :comm_task.cpp
 * @brief     : 云台板通信任务（DBUS 接收 + CAN 发送 + 看门狗）
 * @history   :
 *  Version     Date            Author          Note
 *  V1.0.0      2026-10-19      <author>        1. 完成编写
 *******************************************************************************
 * @attention :
 *  【本板喂狗的两个来源】
 *    1. 本板自己的遥控器数据（如果接收机接在云台板上）；
 *    2. 底盘板 C2G 帧里的「遥控器在线」标志（见 src/HW_can.cpp 的解包处）。
 *  这正是考核资料里「上板在解包下板信息处调用看门狗刷新函数」的要求：
 *  底盘板一死，云台板跟着复位停机，不会出现「一半活着」的僵尸车。
 *
 *  ⚠️ 副作用：遥控器没开机时，两板都会每 200ms 复位一次（预期行为）。
 *******************************************************************************
 *  Copyright (c) 2024 Hello World Team, Zhejiang University.
 *  All Rights Reserved.
 *******************************************************************************
 */
/* Includes ------------------------------------------------------------------*/
#include "comm_task.hpp"

#include "DT7.hpp"
#include "HW_can.hpp"
#include "can.h"
#include "gimbal_chassis_comm.hpp"
#include "ins_all.hpp"
#include "iwdg.h"
#include "motor.hpp"
#include "robot.hpp"
#include "usart.h"
/* Private macro -------------------------------------------------------------*/
/* Private constants ---------------------------------------------------------*/

namespace {

/** 是否启用看门狗喂狗（调车阶段可临时关掉；验收时必须为 true） */
constexpr bool kWatchdogFeedEnable = true;

/** DBUS 接收缓冲区 */
uint8_t s_dbus_buf[hello_world::remote_control::kRcRxDataLen];

/** CAN 发送缓冲 */
uint8_t s_yaw_frame[8];
uint8_t s_pitch_frame[8];
uint8_t s_comm_frame[8];

/** 发一帧达妙电机控制报文（MIT 模式发送 ID = 电机 ID） */
void SendDamiaoFrame(hello_world::motor::Motor* motor, uint8_t* frame)
{
  if (motor == nullptr) {
    return;
  }
  for (uint8_t i = 0; i < 8; ++i) {
    frame[i] = 0;
  }
  if (!motor->encode(frame)) {
    return;
  }
  /* 使能 / 使能失败时 encode 会产出命令帧，发送 ID 依然是电机 ID */
  CAN_Send_Msg(&hcan2, frame,
               hello_world::motor::DM_J4310::GetTxId(motor->id()), 8);
}

}  // namespace

/* Exported function definitions ---------------------------------------------*/

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
  HAL_UARTEx_ReceiveToIdle_DMA(&huart3, s_dbus_buf,
                               hello_world::remote_control::kRcRxDataLen);
}

void CommTask(void)
{
  /* ---------- 1. 两个达妙电机：各自一帧，ID 不同，不能合并 ---------- */
  SendDamiaoFrame(GetMotorYaw(), s_yaw_frame);
  SendDamiaoFrame(GetMotorPitch(), s_pitch_frame);

  /* ---------- 2. 双板通信帧（走 CAN1，与底盘板的 CAN1 相连） ---------- */
  robot::GimbalChassisComm* comm = GetGimbalChassisComm();
  if (comm != nullptr) {
    comm->encode(s_comm_frame);
    CAN_Send_Msg(&hcan1, s_comm_frame, comm->tx_id(), 8);
  }
}

/* Private function definitions ----------------------------------------------*/

/**
 * @brief UART 空闲中断回调：DBUS 一帧收完时进入
 * @note  必须 extern "C"，否则中断向量表找不到这个符号。
 */
extern "C" void HAL_UARTEx_RxEventCallback(UART_HandleTypeDef* huart,
                                           uint16_t Size)
{
  if (huart != &huart3) {
    return;
  }

  if (Size == hello_world::remote_control::kRcRxDataLen) {
    hello_world::remote_control::DT7* rc = GetRemoteControl();
    if (rc != nullptr) {
      rc->decode(s_dbus_buf);
    }

    robot::Robot* robot = GetRobot();
    if (robot != nullptr) {
      robot->notifyRemoteRx();
    }

    if (kWatchdogFeedEnable) {
      HAL_IWDG_Refresh(&hiwdg);
    }
  }

  HAL_UARTEx_ReceiveToIdle_DMA(&huart3, s_dbus_buf,
                               hello_world::remote_control::kRcRxDataLen);
}
