/**
 *******************************************************************************
 * @file      :comm_task.cpp
 * @brief     : 底盘板通信任务（DBUS 接收 + CAN 发送 + 看门狗）
 * @history   :
 *  Version     Date            Author          Note
 *  V1.0.0      2026-10-19      <author>        1. 完成编写
 *******************************************************************************
 * @attention :
 *  【为什么「收」在中断里，「发」在定时任务里】
 *  收：数据什么时候来不确定（DBUS 100Hz、CAN 1kHz），只能被动等中断。
 *  发：控制周期固定 1ms，集中在下发时刻统一发，帧率稳定、也方便和状态机对齐。
 *
 *  【看门狗喂狗条件（这是硬性验收项）】
 *  只在「收到有效遥控器数据」时喂狗。遥控器一关，200ms 后板子自动复位停机。
 *  这看起来「很暴力」，但正是考核要求的：宁可整机复位，也绝不允许一台
 *  「主人已经失联的车」继续转。
 *
 *  ⚠️ 副作用：上电时如果遥控器没开机，板子会每 200ms 复位一次（表现为
 *  LED 闪烁 / 连不上调试器）。这是预期行为，不是板子坏了。
 *  调车时若确实需要不开遥控器调试，把 kWatchdogFeedEnable 临时改成 false。
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

/** DBUS 接收缓冲区：固定 18 字节 */
uint8_t s_dbus_buf[hello_world::remote_control::kRcRxDataLen];

/** CAN 发送缓冲（每个控制周期复用，避免频繁压栈） */
uint8_t s_wheel_frame[8];
uint8_t s_steer_frame[8];
uint8_t s_comm_frame[8];

}  // namespace

/* Exported function definitions ---------------------------------------------*/

void CommTaskInit(void)
{
  /* ---------- 1. CAN 滤波器 + 启动 CAN ----------
   * 滤波器「全收」，具体要哪一帧由接收回调里按 ID 分发决定。
   * 用全收的好处：调试时能看到总线上所有帧，不怕漏配。 */
  CanFilter_Init(&hcan1);
  CanFilter_Init(&hcan2);

  HAL_CAN_Start(&hcan1);
  HAL_CAN_Start(&hcan2);

  /* 打开「收到新帧」中断：CAN1 → FIFO0，CAN2 → FIFO1（与滤波器配置对应） */
  HAL_CAN_ActivateNotification(&hcan1, CAN_IT_RX_FIFO0_MSG_PENDING);
  HAL_CAN_ActivateNotification(&hcan2, CAN_IT_RX_FIFO1_MSG_PENDING);

  /* ---------- 2. 启动 DBUS 接收 ----------
   * 空闲 + DMA：数据停止到来（一帧 18 字节收完）时触发回调。
   * 遥控器 100Hz，所以每 10ms 会进一次 HAL_UARTEx_RxEventCallback。 */
  HAL_UARTEx_ReceiveToIdle_DMA(&huart3, s_dbus_buf,
                               hello_world::remote_control::kRcRxDataLen);

  /* ---------- 3. 看门狗 ----------
   * IWDG 已经在 MX_IWDG_Init() 里启动（约 200ms 超时），这里不需要再初始化，
   * 只需要之后在收到遥控器数据时刷新（见 HAL_UARTEx_RxEventCallback）。 */
}

void CommTask(void)
{
  /* ---------- 1. 下发驱动电机（4 台共用 0x200） ----------
   * 注意：encode() 只会往缓冲区里「自己那 2 个字节」写数据
   * （索引 = (id-1)%4），所以 4 台电机依次编码到同一块缓冲区，
   * 最后发一帧即可。如果每台电机各发一帧，后发的会把先发的覆盖掉。 */
  for (uint8_t i = 0; i < 8; ++i) {
    s_wheel_frame[i] = 0;
  }
  hello_world::motor::Motor* wheel0 = GetMotorWheel(0);
  if (wheel0 != nullptr) {
    for (int i = 0; i < 4; ++i) {
      hello_world::motor::Motor* motor = GetMotorWheel(i);
      if (motor != nullptr) {
        motor->encode(s_wheel_frame);
      }
    }
    CAN_Send_Msg(&hcan1, s_wheel_frame,
                 hello_world::motor::M3508::GetTxId(wheel0->id()), 8);
  }

  /* ---------- 2. 下发转向电机（4 台共用 0x1FF 电压帧） ---------- */
  for (uint8_t i = 0; i < 8; ++i) {
    s_steer_frame[i] = 0;
  }
  hello_world::motor::Motor* steer0 = GetMotorSteer(0);
  if (steer0 != nullptr) {
    for (int i = 0; i < 4; ++i) {
      hello_world::motor::Motor* motor = GetMotorSteer(i);
      if (motor != nullptr) {
        motor->encode(s_steer_frame);
      }
    }
    CAN_Send_Msg(&hcan2, s_steer_frame,
                 hello_world::motor::GM6020::GetTxId(
                     steer0->id(), steer0->get_input_type()),
                 8);
  }

  /* ---------- 3. 双板通信帧 ----------
   * 本板走 CAN1（与云台板的 CAN1 相连）。帧 ID 由通信对象自己给出。 */
  robot::GimbalChassisComm* comm = GetGimbalChassisComm();
  if (comm != nullptr) {
    comm->encode(s_comm_frame);
    CAN_Send_Msg(&hcan1, s_comm_frame, comm->tx_id(), 8);
  }

  /* ---------- 4. 看门狗 ----------
   * 主要在 DBUS 中断里喂（见下）。这里补一条「云台板说遥控器在线」的路径：
   * 万一接收机接在云台板上，底盘板也能正常活下去；遥控器一关，两条路径
   * 同时断掉，板子照样会复位停机。 */
  if (kWatchdogFeedEnable && (comm != nullptr) && !comm->isOffline() &&
      comm->gimbal_data().dbus_online) {
    HAL_IWDG_Refresh(&hiwdg);
  }
}

/* Private function definitions ----------------------------------------------*/

/**
 * @brief UART 空闲中断回调：DBUS 一帧收完时进入
 * @note  必须用 extern "C"：C 链接的中断向量表才能找到这个函数。
 *        如果不加，链接器会找不到符号（表现为程序卡在启动文件里）。
 */
extern "C" void HAL_UARTEx_RxEventCallback(UART_HandleTypeDef* huart,
                                           uint16_t Size)
{
  if (huart != &huart3) {
    return;
  }

  if (Size == hello_world::remote_control::kRcRxDataLen) {
    /* 解包：18 字节 DBUS → 摇杆 / 拨杆 / 按键 */
    hello_world::remote_control::DT7* rc = GetRemoteControl();
    if (rc != nullptr) {
      rc->decode(s_dbus_buf);
    }

    /* 告诉 Robot「遥控器活着」，顺便刷新它内部的掉线计时 */
    robot::Robot* robot = GetRobot();
    if (robot != nullptr) {
      robot->notifyRemoteRx();
    }

    /* ★ 看门狗喂狗点：只有拿到有效遥控器数据才喂 ★ */
    if (kWatchdogFeedEnable) {
      HAL_IWDG_Refresh(&hiwdg);
    }
  }

  /* 重新启动下一轮接收。不重启的话只能收到第一帧。 */
  HAL_UARTEx_ReceiveToIdle_DMA(&huart3, s_dbus_buf,
                               hello_world::remote_control::kRcRxDataLen);
}
