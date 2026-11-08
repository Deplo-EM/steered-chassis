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

#include "DT7.hpp"
#include "HW_can.hpp"
#include "can.h"
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
 *        注意：改成 true 的效果是「板子不再复位」；改成 false（默认）才是
 *        「条件喂狗、关遥控器会复位」。审计 F2 指出的正是原文档把这两个方向说反了。
 */
constexpr bool kDebugAlwaysFeedWatchdog = false;

/** 遥控数据超过这个时间没更新，就认为「主人失联」，不再喂狗（ms） */
constexpr uint32_t kDbusTimeoutMs = 100u;

/** DBUS 接收缓冲区：固定 18 字节 */
uint8_t s_dbus_buf[hello_world::remote_control::kRcRxDataLen];

/** 最近一帧「合法」DBUS 的时间戳（只由 UART 中断写） */
volatile uint32_t s_last_valid_dbus_tick = 0;
/** 是否收到过至少一帧合法数据（防止上电瞬间被误判为在线） */
volatile bool s_dbus_received = false;
/** 统计：合法 / 非法帧数（调试用） */
volatile uint32_t s_dbus_valid_frames = 0;
volatile uint32_t s_dbus_invalid_frames = 0;

/** CAN 发送失败计数（邮箱满 / 总线错误时会增长） */
uint32_t s_can_tx_fail_count = 0;

/** CAN 发送缓冲（每个控制周期复用，避免频繁压栈） */
uint8_t s_wheel_frame[8];
uint8_t s_steer_frame[8];
uint8_t s_comm_frame[8];

/**
 * @brief 遥控数据是否足够新（决定这一周期能不能喂狗）
 * @note  两条合法来源：
 *        1. 本板自己收到过合法 DBUS 帧，且不超过 100ms；
 *        2. 云台板的帧说「它那边的遥控器在线」（接收机可能接在云台板）。
 *        注意第 2 条只用于**保活**，不用于授权控制：底盘板没有自己的遥控数据
 *        时仍然进不了 Working、输出仍然是 0。
 */
bool RemoteAliveForWatchdog()
{
  const bool local =
      s_dbus_received &&
      ((hello_world::tick::GetTickMs() - s_last_valid_dbus_tick) <= kDbusTimeoutMs);

  robot::GimbalChassisComm* comm = GetGimbalChassisComm();
  const bool peer = (comm != nullptr) && !comm->isOffline() &&
                    comm->gimbal_data().dbus_online;

  return local || peer;
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
  HAL_UARTEx_ReceiveToIdle_DMA(&huart3, s_dbus_buf,
                               hello_world::remote_control::kRcRxDataLen);

  /* ---------- 3. 看门狗 ----------
   * IWDG 已经在 MX_IWDG_Init() 里启动（约 200ms 超时）。这里不喂，
   * 第一帧合法 DBUS 到达后、由 CommTask 每 1ms 喂一次。 */
}

void CommTask(void)
{
  // 不依赖错误中断是否开启，直接读取控制器的 bus-off 状态。
  bool cycle_ok = !CAN_Is_BusOff(&hcan1) && !CAN_Is_BusOff(&hcan2);
  /* ---------- 故障注入：占满 CAN 发送邮箱 ----------
   * 连发 3 帧把 3 个邮箱占满，本周期真正的控制帧就会发送失败，
   * s_can_tx_fail_count 会增长。用来验证审计 H4 的"发送失败可检测"。 */
  if (robot::FaultActive(robot::FaultId::kCanTxStarve)) {
    static uint8_t dummy[8] = {0};
    for (int i = 0; i < 3; ++i) {
      (void)CAN_Send_Msg(&hcan1, dummy, 0x7F0u, 8);
    }
  }

  /* ---------- 1. 下发驱动电机（4 台共用 0x200） ----------
   * encode() 只会往缓冲区里「自己那 2 个字节」写数据（索引 = (id-1)%4），
   * 所以 4 台电机依次编码到同一块缓冲区，最后发一帧即可。 */
  for (uint8_t i = 0; i < 8; ++i) {
    s_wheel_frame[i] = 0;
  }
  hello_world::motor::Motor* wheel0 = GetMotorWheel(0);
  if (wheel0 == nullptr) cycle_ok = false;
  if (wheel0 != nullptr) {
    for (int i = 0; i < 4; ++i) {
      hello_world::motor::Motor* motor = GetMotorWheel(i);
      if (motor != nullptr) {
        if (!motor->encode(s_wheel_frame)) cycle_ok = false;
      } else { cycle_ok = false; }
    }
    if (!CAN_Send_Msg(&hcan1, s_wheel_frame,
                      hello_world::motor::M3508::GetTxId(wheel0->id()), 8)) {
      ++s_can_tx_fail_count;
      cycle_ok = false;
    }
  }

  /* ---------- 2. 下发转向电机（4 台共用 0x1FF 电压帧） ---------- */
  for (uint8_t i = 0; i < 8; ++i) {
    s_steer_frame[i] = 0;
  }
  hello_world::motor::Motor* steer0 = GetMotorSteer(0);
  if (steer0 == nullptr) cycle_ok = false;
  if (steer0 != nullptr) {
    for (int i = 0; i < 4; ++i) {
      hello_world::motor::Motor* motor = GetMotorSteer(i);
      if (motor != nullptr) {
        if (!motor->encode(s_steer_frame)) cycle_ok = false;
      } else { cycle_ok = false; }
    }
    if (!CAN_Send_Msg(&hcan2, s_steer_frame,
                      hello_world::motor::GM6020::GetTxId(
                          steer0->id(), steer0->get_input_type()),
                      8)) {
      ++s_can_tx_fail_count;
      cycle_ok = false;
    }
  }

  /* ---------- 3. 双板通信帧 ---------- */
  robot::GimbalChassisComm* comm = GetGimbalChassisComm();
  if (comm == nullptr) cycle_ok = false;
  if (comm != nullptr) {
    comm->encode(s_comm_frame);
    if (!CAN_Send_Msg(&hcan1, s_comm_frame, comm->tx_id(), 8)) {
      ++s_can_tx_fail_count;
      cycle_ok = false;
    }
  }

  /* ---------- 4. 看门狗：整个周期都跑完了才喂 ----------
   * 这一行在 MainTask()（控制、逆解、闭环、下发）之后执行。
   * 控制循环卡住 → 走不到这里 → 不喂 → 200ms 后复位。
   * 全板**只有这一个**喂狗调用点。 */
  robot::Robot* controller = GetRobot();
  if (controller != nullptr) controller->notifyCanTxCycle(cycle_ok);
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
  HAL_UARTEx_ReceiveToIdle_DMA(&huart3, s_dbus_buf,
                               hello_world::remote_control::kRcRxDataLen);
}
