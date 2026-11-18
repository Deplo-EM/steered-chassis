#pragma once
#include <cstdint>

namespace robot {
/** 提交结果：仅暂时没有空邮箱时为 kBusy；已入邮箱还要监督实际发送结果。 */
enum class CanTxSubmitResult : uint8_t { kQueued, kBusy, kFault };
enum class CanTxStatus : uint8_t { kIdle, kPending, kSent, kUnconfirmed };
struct CanTxReceipt {
  CanTxStatus status = CanTxStatus::kIdle;
};
}  // namespace robot
