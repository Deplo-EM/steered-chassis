/**
 *******************************************************************************
 * @file      :gimbal_chassis_comm.cpp
 * @brief     : 云台板 ↔ 底盘板 CAN 通信协议实现
 * @history   :
 *  Version     Date            Author          Note
 *  V1.0.0      2026-10-19      <author>        1. 完成 encode / decode / isOffline
 *******************************************************************************
 * @attention :
 *  编解码必须严格对称：一端把第 1~2 字节当 yaw 角速度，另一端也必须当成
 *  yaw 角速度。任何一端写错偏移，对面读到的就是「看起来像正常数值的垃圾」，
 *  这种 bug 极难排查——所以 tests/ 目录里专门有一个「编码后再解码应还原原值」
 *  的自测程序，改完协议先跑它。
 *******************************************************************************
 *  Copyright (c) 2024 Hello World Team, Zhejiang University.
 *  All Rights Reserved.
 *******************************************************************************
 */
/* Includes ------------------------------------------------------------------*/
#include "gimbal_chassis_comm.hpp"

#include <cmath>

#include "crc8.hpp"
/* Private macro -------------------------------------------------------------*/
/* Private constants ---------------------------------------------------------*/

namespace {
/** 角度缩放：1 LSB = 0.0001 rad（约 0.0057°），int16 可表示 ±3.2767 rad */
constexpr float kAngleScale = 10000.0f;
/** yaw 角速度缩放：1 LSB = 0.001 rad/s，int16 可表示 ±32.767 rad/s */
constexpr float kYawRateScale = 1000.0f;

/** 浮点 → int16，带范围保护。
 *  注意下界取 -32767 而不是 -32768：避免 +32767 与 -32768 不对称，
 *  也避免 lroundf 溢出到实现未定义的行为。 */
inline int16_t FloatToInt16(float value, float scale)
{
  if (!std::isfinite(value)) return 0;
  float scaled = value * scale;
  if (scaled > 32767.0f) {
    scaled = 32767.0f;
  } else if (scaled < -32767.0f) {
    scaled = -32767.0f;
  }
  return static_cast<int16_t>(std::lroundf(scaled));
}

/** int16 → 浮点（与 FloatToInt16 互为逆运算） */
inline float Int16ToFloat(int16_t value, float scale)
{
  return static_cast<float>(value) / scale;
}

/** 大端写入 16 位整数（CAN 帧字节序由我们自己约定，这里统一用大端） */
inline void WriteInt16(uint8_t* data, int offset, int16_t value)
{
  const uint16_t raw = static_cast<uint16_t>(value);
  data[offset] = static_cast<uint8_t>(raw >> 8);
  data[offset + 1] = static_cast<uint8_t>(raw & 0xFFu);
}

/** 大端读取 16 位整数 */
inline int16_t ReadInt16(const uint8_t* data, int offset)
{
  const uint16_t raw =
      static_cast<uint16_t>((static_cast<uint16_t>(data[offset]) << 8) |
                            static_cast<uint16_t>(data[offset + 1]));
  return static_cast<int16_t>(raw);
}

/** 计算前 7 字节的 CRC8（多项式 0x07，初值 0），放在第 8 字节 */
inline uint8_t CalcCrc(const uint8_t* data)
{
  hello_world::tools::crc8_check crc;
  return crc.getCrc8CheckNum(data, 7u, 0u);
}

}  // namespace

/* Private variables ---------------------------------------------------------*/
/* External variables --------------------------------------------------------*/
/* Private function prototypes -----------------------------------------------*/
/* Exported function definitions ---------------------------------------------*/
robot::GimbalChassisComm::GimbalChassisComm(robot::GimbalChassisComm::CodePart code_part,
                                            uint32_t tx_id, uint32_t rx_id) {
  code_part_ = code_part;
  tx_id_ = tx_id;
  rx_id_ = rx_id;
  last_rx_tick_ = hello_world::tick::GetTickMs();
}

/* ============================================================================
 * 组一帧 8 字节发出去（方向不同，字段含义不同）
 * ---------------------------------------------------------------------------
 * 字节布局（两个方向共用）：
 *   byte0    flags：alive/working/estop/dbus_online/mode/imu_ready/...
 *            （位定义按方向不同，见函数内注释）
 *   byte1-2  int16：底盘→云台是 yaw_rate；云台→底盘是 yaw_joint
 *   byte3-4  int16：底盘→云台是 pitch_target；云台→底盘是 pitch_joint
 *   byte5    保留（固定 0）
 *   byte6    发送序号（每帧 +1，接收端用来识别丢帧/重复）
 *   byte7    CRC8（对 byte0~6 计算，多项式 0x07，初值 0）
 *
 * 关键：先把整帧清零再填，保留字节固定 0，绝不把栈上随机值发出去；
 * 若数据里出现非有限值，整帧降级为"急停"状态，防止把 working/enable 带给对板。
 * ========================================================================== */
void robot::GimbalChassisComm::encode(uint8_t tx_data[8]) {
  /* 先整体清零：保留字节固定填 0，避免把栈上的随机值发出去 */
  for (uint8_t i = 0; i < kCommFrameLen; ++i) {
    tx_data[i] = 0;
  }

  if (code_part_ == CodePart::Chassis) {
    /* ---------------------------- 底盘 → 云台 ---------------------------- */
    ChassisData d = chassis_data_;
    if (!std::isfinite(d.yaw_rate) || !std::isfinite(d.pitch_target)) {
      d = ChassisData{};
      d.estop = true;  // 异常数不得带着 working/enable 被发给对板。
    }
    d.alive = true;  /* 能执行到 encode，说明本板活着 */

    uint8_t flags = 0;
    if (d.alive) flags |= (1u << 0);
    if (d.working) flags |= (1u << 1);
    if (d.estop) flags |= (1u << 2);
    if (d.dbus_online) flags |= (1u << 3);
    flags |= (static_cast<uint8_t>(d.mode) & 0x03u) << 4;
    if (d.imu_ready) flags |= (1u << 6);
    if (d.gimbal_enable) flags |= (1u << 7);
    tx_data[0] = flags;

    WriteInt16(tx_data, 1, FloatToInt16(d.yaw_rate, kYawRateScale));
    WriteInt16(tx_data, 3, FloatToInt16(d.pitch_target, kAngleScale));
  } else {
    /* ---------------------------- 云台 → 底盘 ---------------------------- */
    GimbalData d = gimbal_data_;
    if (!std::isfinite(d.yaw_joint) || !std::isfinite(d.pitch_joint)) {
      d = GimbalData{};
      d.estop = true;
    }
    d.alive = true;

    uint8_t flags = 0;
    if (d.alive) flags |= (1u << 0);
    if (d.working) flags |= (1u << 1);
    if (d.estop) flags |= (1u << 2);
    if (d.dbus_online) flags |= (1u << 3);
    if (d.imu_ready) flags |= (1u << 4);
    if (d.pitch_at_limit) flags |= (1u << 5);
    if (d.working && d.ground_disarmed) flags |= (1u << 6);
    tx_data[0] = flags;

    WriteInt16(tx_data, 1, FloatToInt16(d.yaw_joint, kAngleScale));
    WriteInt16(tx_data, 3, FloatToInt16(d.pitch_joint, kAngleScale));
  }

  tx_data[5] = 0;  /* 保留字节 */
  tx_data[6] = tx_seq_++;
  tx_data[7] = CalcCrc(tx_data);
}

/* ============================================================================
 * 收一帧并校验
 * ---------------------------------------------------------------------------
 * 校验顺序：CRC8 → 字段范围/有限性 → 序号连续性。
 * 任何一项不过：整帧丢弃，且**不刷新 last_rx_tick_**（等于没收到），
 * 让上层按"离线超时"处理，而不是拿半帧坏数据继续跑。
 * 通过后才更新 gimbal_data_/chassis_data_、序号和在线时间。
 * ========================================================================== */
bool robot::GimbalChassisComm::decode(const uint8_t rx_data[8]) {
  /* ---- 第一步：校验 ----
   * CRC 不对就整帧丢弃，并且不刷新 last_rx_tick_（等于当成没收到）。
   * 返回 false 让调用方知道「这一帧不能用」——不要拿旧缓存去喂狗。 */
  if (rx_data[7] != CalcCrc(rx_data)) {
    ++rx_error_count_;
    return false;
  }

  const uint8_t flags = rx_data[0];
#if defined(ROBOT_GROUND_TRIAL) && ROBOT_GROUND_TRIAL
  constexpr uint8_t kG2cReservedMask = 0x80u;
#else
  // 普通底盘保持原协议约束，避免把首跑失能云台当成考核固件使用。
  constexpr uint8_t kG2cReservedMask = 0xC0u;
#endif
  const bool fields_ok = rx_data[5] == 0u &&
      (code_part_ == CodePart::Chassis ? (flags & kG2cReservedMask) == 0u
                                    : ((flags >> 4) & 0x03u) <= 2u);
  if (!fields_ok) {
    ++rx_error_count_;
    return false;
  }

  const uint32_t now = hello_world::tick::GetTickMs();
  const uint8_t seq = rx_data[6];
  const uint8_t advance = static_cast<uint8_t>(seq - rx_seq_);
  if (received_ && (advance == 0u || advance >= 128u)) {
    // 同序号永不保活。离线后的逆序可能是对方重启：先暂存，再等递增帧确认。
    // 8 位序号只检查短窗口次序，不是认证，也无法排除完整循环的历史重放。
    const bool resynced = advance != 0u && isOffline() && resync_pending_ &&
        static_cast<uint8_t>(seq - resync_seq_) == 1u &&
        (now - resync_tick_) <= kCommOfflineThresholdMs;
    if (!resynced) {
      if (advance != 0u && isOffline()) {
        resync_pending_ = true;
        resync_seq_ = seq;
        resync_tick_ = now;
      }
      ++rx_error_count_;
      return false;
    }
  }
  resync_pending_ = false;
  rx_seq_ = seq;

  if (code_part_ == CodePart::Chassis) {
    /* 底盘板收到的是云台的 G2C 帧 */
    GimbalData& d = gimbal_data_;
    d.alive = (flags & (1u << 0)) != 0;
    d.working = (flags & (1u << 1)) != 0;
    d.estop = (flags & (1u << 2)) != 0;
    d.dbus_online = (flags & (1u << 3)) != 0;
    d.imu_ready = (flags & (1u << 4)) != 0;
    d.pitch_at_limit = (flags & (1u << 5)) != 0;
    d.ground_disarmed = (flags & (1u << 6)) != 0;

    d.yaw_joint = Int16ToFloat(ReadInt16(rx_data, 1), kAngleScale);
    d.pitch_joint = Int16ToFloat(ReadInt16(rx_data, 3), kAngleScale);
  } else {
    /* 云台板收到的是底盘的 C2G 帧 */
    ChassisData& d = chassis_data_;
    d.alive = (flags & (1u << 0)) != 0;
    d.working = (flags & (1u << 1)) != 0;
    d.estop = (flags & (1u << 2)) != 0;
    d.dbus_online = (flags & (1u << 3)) != 0;
    d.mode = static_cast<ChassisMode>((flags >> 4) & 0x03u);
    d.imu_ready = (flags & (1u << 6)) != 0;
    d.gimbal_enable = (flags & (1u << 7)) != 0;

    d.yaw_rate = Int16ToFloat(ReadInt16(rx_data, 1), kYawRateScale);
    d.pitch_target = Int16ToFloat(ReadInt16(rx_data, 3), kAngleScale);
  }

  /* 只有校验通过、字段解析完成，才认为「对方还活着」 */
  last_rx_tick_ = hello_world::tick::GetTickMs();
  received_ = true;
  ++rx_ok_count_;
  return true;
}

bool robot::GimbalChassisComm::isOffline() const {
  if (!received_) {
    return true;  /* 上电后一帧都没收到，当然算离线 */
  }
  /* 无符号减法：即使 GetTickMs 回绕（约 49.7 天一次）结果也正确 */
  const uint32_t elapsed = hello_world::tick::GetTickMs() - last_rx_tick_;
  return elapsed > kCommOfflineThresholdMs;
}

/* Private function definitions ----------------------------------------------*/
