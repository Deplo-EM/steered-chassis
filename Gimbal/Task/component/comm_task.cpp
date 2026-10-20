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

#include "DT7.hpp"
#include "HW_can.hpp"
#include "can.h"
#include "dbus_check.hpp"
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

/** 是否启用看门狗喂狗（调车阶段可临时关掉；验收时必须为 true） */
constexpr bool kWatchdogFeedEnable = true;

/** 遥控数据超过这个时间没更新，就认为「主人失联」，不再喂狗（ms） */
constexpr uint32_t kDbusTimeoutMs = 100u;

/** DBUS 接收缓冲区 */
uint8_t s_dbus_buf[hello_world::remote_control::kRcRxDataLen];

/** 最近一帧「合法」DBUS 的时间戳（只由 UART 中断写） */
volatile uint32_t s_last_valid_dbus_tick = 0;
volatile bool s_dbus_received = false;
volatile uint32_t s_dbus_valid_frames = 0;
volatile uint32_t s_dbus_invalid_frames = 0;

/** CAN 发送失败计数 */
uint32_t s_can_tx_fail_count = 0;

/** CAN 发送缓冲 */
uint8_t s_yaw_frame[8];
uint8_t s_pitch_frame[8];
uint8_t s_comm_frame[8];

/** 发一帧达妙电机控制报文（MIT 模式发送 ID = 电机 ID） */
bool SendDamiaoFrame(hello_world::motor::Motor* motor, uint8_t* frame)
{
  if (motor == nullptr) {
    return false;
  }
  for (uint8_t i = 0; i < 8; ++i) {
    frame[i] = 0;
  }
  if (!motor->encode(frame)) {
    return false;
  }
  return CAN_Send_Msg(&hcan2, frame,
                      hello_world::motor::DM_J4310::GetTxId(motor->id()), 8);
}

/**
 * @brief 遥控数据是否足够新（决定这一周期能不能喂狗）
 * @note  本板遥控器在线，或者底盘板说它的遥控器在线。
 */
bool RemoteAliveForWatchdog()
{
  const bool local =
      s_dbus_received &&
      ((hello_world::tick::GetTickMs() - s_last_valid_dbus_tick) <= kDbusTimeoutMs);

  robot::GimbalChassisComm* comm = GetGimbalChassisComm();
  const bool peer = (comm != nullptr) && !comm->isOffline() &&
                    comm->chassis_data().dbus_online;

  return local || peer;
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
  if (!SendDamiaoFrame(GetMotorYaw(), s_yaw_frame)) {
    ++s_can_tx_fail_count;
  }
  if (!SendDamiaoFrame(GetMotorPitch(), s_pitch_frame)) {
    ++s_can_tx_fail_count;
  }

  /* ---------- 2. 双板通信帧（走 CAN1） ---------- */
  robot::GimbalChassisComm* comm = GetGimbalChassisComm();
  if (comm != nullptr) {
    comm->encode(s_comm_frame);
    if (!CAN_Send_Msg(&hcan1, s_comm_frame, comm->tx_id(), 8)) {
      ++s_can_tx_fail_count;
    }
  }

  /* ---------- 3. 看门狗：整个周期都跑完了才喂 ---------- */
  if (kWatchdogFeedEnable && RemoteAliveForWatchdog()) {
    HAL_IWDG_Refresh(&hiwdg);
  }
}

/* Private function definitions ----------------------------------------------*/

/**
 * @brief UART 空闲中断回调：DBUS 一帧收完时进入
 * @note  必须 extern "C"，否则中断向量表找不到这个符号。
 *        这里**不喂狗**，只记录「收到了一帧合法数据」。
 */
extern "C" void HAL_UARTEx_RxEventCallback(UART_HandleTypeDef* huart,
                                           uint16_t Size)
{
  if (huart == &huart3 && Size == hello_world::remote_control::kRcRxDataLen) {
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
      ++s_dbus_valid_frames;
    } else {
      ++s_dbus_invalid_frames;
    }
  }

  HAL_UARTEx_ReceiveToIdle_DMA(&huart3, s_dbus_buf,
                               hello_world::remote_control::kRcRxDataLen);
}
