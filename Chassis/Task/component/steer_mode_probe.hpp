/** @file steer_mode_probe.hpp
 *  @brief 独立台架诊断：比较 GM6020 ID 2 对电压帧和电流帧的响应。
 *  @note 仅诊断构建使用；正常控制环不使用此状态机。拨杆值与 DT7 一致。
 */
#pragma once
#include <cmath>
#include <cstdint>

namespace robot::probe {
struct Input {
  bool healthy = false, estop = true, other_neutral = true;
  uint8_t left = 0, right = 0;
  float forward = 0, angle = 0, current = 0;
};

struct Command {
  uint16_t id = 0x1FF;
  uint16_t raw = 0;
  void encode(uint8_t (&frame)[8]) const {
    for (auto& byte : frame) byte = 0;
    frame[2] = static_cast<uint8_t>(raw >> 8); // ID 2 占第 2、3 字节。
    frame[3] = static_cast<uint8_t>(raw);
  }
};

struct Result {
  uint32_t completed = 0;
  float peak_angle_rad = 0, peak_delta_a = 0;
  uint32_t attempted = 0, queued = 0, sent = 0, unconfirmed = 0;
};

class SteerModeProbe {
 public:
  enum class Phase : uint8_t { kWaitStop, kHoldStop, kWaitNeutral, kReady, kPulse, kSpent };
  Result voltage_result, current_result;
  Phase phase = Phase::kWaitStop;

  Result& resultFor(uint16_t id) { return id == 0x1FF ? voltage_result : current_result; }

  Command step(const Input& in, uint32_t now) {
    if (!in.healthy || !std::isfinite(in.angle) || !std::isfinite(in.current) ||
        !std::isfinite(in.forward) || !in.other_neutral || std::fabs(in.current) >= 0.8f) {
      phase = Phase::kWaitStop;
      return {};
    }
    const uint16_t id = in.left == 1 ? 0x1FF : in.left == 3 ? 0x1FE : 0;
    if (in.estop || in.right == 2) {
      if (in.estop && in.right == 2 && id && in.forward == 0.0f) {
        if (phase != Phase::kHoldStop || id != selected_) {
          phase = Phase::kHoldStop;
          since_ = now;
          selected_ = id;
        }
      } else {
        phase = Phase::kWaitStop;
      }
      return {};
    }
    if (in.right != 3 || !id || id != selected_) {
      phase = Phase::kWaitStop;
      return {};
    }
    if (phase == Phase::kHoldStop) {
      if (now - since_ < 150u || in.forward != 0.0f) {
        phase = Phase::kWaitStop;
        return {};
      }
      since_ = now;
      phase = Phase::kWaitNeutral;
    }
    if (phase == Phase::kWaitNeutral) {
      if (in.forward != 0.0f) {
        phase = Phase::kSpent;
      } else if (now - since_ >= 80u) {
        phase = Phase::kReady;
      }
      return {};
    }
    auto& result = resultFor(id);
    if (phase == Phase::kReady && in.forward >= 0.16f) {
      angle_start_ = in.angle;
      current_start_ = in.current;
      result.peak_angle_rad = result.peak_delta_a = 0.0f;
      result.attempted = result.queued = result.sent = result.unconfirmed = 0;
      since_ = now;
      phase = Phase::kPulse;
    }
    if (phase != Phase::kPulse) return {};

    const float travel = std::fabs(std::remainder(in.angle - angle_start_, 6.2831853f));
    const float delta_a = std::fabs(in.current - current_start_);
    if (travel > result.peak_angle_rad) result.peak_angle_rad = travel;
    if (delta_a > result.peak_delta_a) result.peak_delta_a = delta_a;
    const uint32_t elapsed = now - since_;
    if (elapsed >= 160u || travel >= 0.12f || in.forward < 0.12f) {
      if (elapsed >= 160u || travel >= 0.12f) ++result.completed;
      phase = Phase::kSpent;
      return {};
    }
    const uint32_t limit = id == 0x1FF ? 3000u : 2457u; // 电压满量程的 12%，或约 0.45 A。
    const uint32_t rise = elapsed < 40u ? elapsed + 1u : 40u;
    return {id, static_cast<uint16_t>(limit * rise / 40u)};
  }

 private:
  uint32_t since_ = 0;
  uint16_t selected_ = 0;
  float angle_start_ = 0, current_start_ = 0;
};
}  // namespace robot::probe
