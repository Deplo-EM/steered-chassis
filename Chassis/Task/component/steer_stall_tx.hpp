#pragma once
#include <cstdint>

namespace robot::diag {
/** 停机当轮复制，急停确认后仍保留。TXOK 不代表目标电机已经执行。 */
struct SteerStallTx {
  uint32_t sequence = 0;
  uint32_t tick_ms = 0;
  uint32_t attempted_nonzero = 0;
  uint32_t queued_nonzero = 0;     ///< 仅统计带回执的非零帧。
  uint32_t confirmed_nonzero = 0;  ///< 同上，均为本次上电累计。
  uint32_t unconfirmed_nonzero = 0;
  bool has_recent_txok = false;    ///< 最近 10 ms 内确认发送的非零帧。
  uint16_t frame_id = 0;
  int16_t command_raw[4] = {};     ///< 按帧中四个设备 ID 槽位排列。
};
extern volatile SteerStallTx g_steer_stall_tx;
}  // namespace robot::diag
