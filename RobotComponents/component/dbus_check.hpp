/**
 *******************************************************************************
 * @file      : dbus_check.hpp
 * @brief     : DBUS 帧合法性校验（纯逻辑，两板共用，可在电脑上单测）
 * @history   :
 *  Version     Date            Author          Note
 *  V1.0.0      2026-10-20      <author>        1. 审计修复：非法帧不得授权输出
 *******************************************************************************
 * @attention :
 *  【为什么需要校验】
 *  DT7 的解码类只负责"把 18 字节切开"，它不判断这些字节**是不是一帧真正的
 *  遥控器数据**。串口错位、噪声、接收机掉电时的全零数据，都可能凑出 18 字节，
 *  解码后得到一堆"看起来正常"的杆量。
 *
 *  如果这种帧被当成有效指令：
 *    - 会刷新"遥控器在线"时间戳 → 状态机继续 Working → 电机继续出力；
 *    - 会刷新看门狗 → 板子永远不会复位。
 *  两者都直接违反"只有收到有效 DBUS 才允许电机输出"这条红线。
 *
 *  【合法帧的判据（来自 DBUS 协议）】
 *    - 5 个模拟通道（右横、右纵、左横、左纵、拨轮）：11 位，有效范围 364~1684，
 *      中位 1024。超出范围说明字节错位或数据无效。
 *    - 两个拨杆各 2 位，只有 1(上) / 2(下) / 3(中) 三个合法值，0 是非法值。
 *
 *  本函数只做"能不能用"的判断，不做解码；解码仍然交给 DT7 类。
 *******************************************************************************
 *  Copyright (c) 2024 Hello World Team, Zhejiang University.
 *  All Rights Reserved.
 *******************************************************************************
 */
/* Define to prevent recursive inclusion -------------------------------------*/
#ifndef DBUS_CHECK_HPP_
#define DBUS_CHECK_HPP_

/* Includes ------------------------------------------------------------------*/
#include <cstdint>

namespace robot {

/* Exported constants --------------------------------------------------------*/

/** DBUS 模拟通道的合法范围（含端点），中位 1024 */
constexpr uint16_t kDbusChannelMin = 364u;
constexpr uint16_t kDbusChannelMax = 1684u;

/** DBUS 帧长度 */
constexpr uint8_t kDbusFrameLen = 18u;

/* Exported function prototypes ----------------------------------------------*/

/**
 * @brief 取出第 index 个模拟通道的原始值（11 位）
 * @param data  18 字节 DBUS 数据
 * @param index 0=右横 1=右纵 2=左横 3=左纵 4=拨轮
 * @retval 0~2047 的原始值
 * @note   DBUS 是"按位紧密排列"的：通道 n 占第 n*11 位起的 11 位，
 *         所以要从任意 bit 偏移处跨字节取。拨轮单独在第 16~17 字节。
 */
inline uint16_t DbusChannel(const uint8_t data[18], int index)
{
  if (index < 4) {
    const int bit = index * 11;
    const int byte = bit / 8;
    const int shift = bit % 8;
    /* 最多需要 3 个字节（7 位偏移 + 11 位 = 18 位） */
    const uint32_t word = static_cast<uint32_t>(data[byte]) |
                          (static_cast<uint32_t>(data[byte + 1]) << 8) |
                          (static_cast<uint32_t>(data[byte + 2]) << 16);
    return static_cast<uint16_t>((word >> shift) & 0x07FFu);
  }
  return static_cast<uint16_t>(
      (static_cast<uint16_t>(data[16]) |
       (static_cast<uint16_t>(data[17]) << 8)) &
      0x07FFu);
}

/**
 * @brief 判断一帧 18 字节数据是否可能是有效的 DBUS 数据
 * @param data 18 字节
 * @retval true 合法（可以解码并授权输出）；false 非法（必须当作"没收到"）
 */
inline bool IsDbusFrameValid(const uint8_t data[18])
{
  /* 1. 五个模拟通道都在合法范围内 */
  for (int i = 0; i < 5; ++i) {
    const uint16_t value = DbusChannel(data, i);
    if (value < kDbusChannelMin || value > kDbusChannelMax) {
      return false;
    }
  }

  /* 2. 两个拨杆只能是 1 / 2 / 3
   *    第 5 字节：bit4~5 = 右拨杆，bit6~7 = 左拨杆 */
  const uint8_t right_switch = static_cast<uint8_t>((data[5] >> 4) & 0x03u);
  const uint8_t left_switch = static_cast<uint8_t>((data[5] >> 6) & 0x03u);
  if (right_switch == 0u || right_switch > 3u) {
    return false;
  }
  if (left_switch == 0u || left_switch > 3u) {
    return false;
  }

  return true;
}

}  // namespace robot

#endif /* DBUS_CHECK_HPP_ */
