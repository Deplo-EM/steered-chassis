#pragma once
#include <cstdint>

namespace robot {
/** 提交结果：仅暂时没有空邮箱时为 kBusy；已入邮箱还要监督实际发送结果。 */
enum class CanTxSubmitResult : uint8_t { kQueued, kBusy, kFault };
/** 单帧回执生命周期；kSent 表示控制器确认发送，执行器动作仍由反馈核验。 */
enum class CanTxStatus : uint8_t { kIdle, kPending, kSent, kUnconfirmed };
/** 发送层写状态、调用方消费状态；静态实例可跨越多个控制周期等待完成。 */
struct CanTxReceipt {
  CanTxStatus status = CanTxStatus::kIdle;
};
}  // namespace robot
