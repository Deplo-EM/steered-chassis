/**
 * @file ground_trial.hpp
 * @brief 四轮首次落地候选的限值和独立起步授权。
 */
#ifndef CHASSIS_GROUND_TRIAL_HPP_
#define CHASSIS_GROUND_TRIAL_HPP_

#include <cmath>
#include <cstdint>

#include "remote_input.hpp"

namespace robot::ground {

inline constexpr float kMaxSpeedMps = 0.20f;         ///< 车身前后速度的指令上限。
inline constexpr float kAccelMps2 = 0.35f;           ///< 正常起停的指令变化率。
inline constexpr float kWheelCurrentA = 2.0f;        ///< 每台驱动电机的请求电流上限。
inline constexpr float kAlignEnterRad = 0.10f;
inline constexpr float kAlignExitRad = 0.15f;
inline constexpr float kAlignCutoffMs = 600.0f;      ///< 连续对不齐即锁存、舵轮同时归零。
inline constexpr uint32_t kNeutralHoldMs = 250u;
// 越过此值才起步；首跑限速下对应的速度还须高于逆解低速保持阈值。
inline constexpr float kStartDemand = 0.15f;
#if defined(ROBOT_GROUND_TRIAL_MOTION) && ROBOT_GROUND_TRIAL_MOTION
inline constexpr bool kMotionEnabled = true;
#else
inline constexpr bool kMotionEnabled = false;
#endif

enum class Phase : uint8_t { kLocked, kAwaitEstop, kAwaitNeutral, kReady, kDriving, kStopping };

class StartGate {
 public:
  void invalidate()
  {
    phase_ = kMotionEnabled ? Phase::kAwaitEstop : Phase::kLocked;
    timing_neutral_ = false;
    direction_ = 0;
  }

  void update(const RemoteInput& rc, bool ready, bool estop, uint32_t now)
  {
    using namespace hello_world::remote_control;
    if (!kMotionEnabled || !ready || !rc.online || rc.left != kSwitchStateUp) {
      invalidate();
      return;
    }
    if (rc.right == kSwitchStateDown) {
      awaitNeutral();
      return;
    }
    if (rc.right != kSwitchStateMid || rc.lh != 0.0f || rc.rv != 0.0f ||
        rc.rh != 0.0f || rc.wheel != 0.0f) {
      invalidate();
      return;
    }
    if (estop) {
      // 遥控转发和板间状态分别到达：退出下档时允许等待急停位消退。
      if (phase_ == Phase::kAwaitNeutral) awaitNeutral();
      else invalidate();
      return;
    }

    switch (phase_) {
      case Phase::kAwaitNeutral:
        if (rc.lv != 0.0f) {
          timing_neutral_ = false;
        } else if (!timing_neutral_) {
          neutral_since_ms_ = now;
          timing_neutral_ = true;
        } else if (now - neutral_since_ms_ >= kNeutralHoldMs) {
          phase_ = Phase::kReady;
        }
        break;
      case Phase::kReady:
        if (std::fabs(rc.lv) >= kStartDemand) {
          direction_ = rc.lv > 0.0f ? 1 : -1;
          phase_ = Phase::kDriving;
        }
        break;
      case Phase::kDriving:
        // 松杆和直接反打都先停；减速完成之前忽略新的前后指令。
        if (rc.lv * direction_ < kStartDemand) phase_ = Phase::kStopping;
        break;
      default:
        break;
    }
  }

  bool permitsOutput() const { return phase_ == Phase::kDriving || phase_ == Phase::kStopping; }
  bool stopping() const { return phase_ == Phase::kStopping; }
  float velocityTarget(const RemoteInput& rc) const
  {
    return phase_ == Phase::kDriving ? rc.lv * kMaxSpeedMps : 0.0f;
  }

  /** 仅在减速指令归零后调用。新指令仍须经过一次连续回中。 */
  void finishStop() { awaitNeutral(); }

 private:
  void awaitNeutral()
  {
    phase_ = Phase::kAwaitNeutral;
    timing_neutral_ = false;
    direction_ = 0;
  }

  Phase phase_ = kMotionEnabled ? Phase::kAwaitEstop : Phase::kLocked;
  bool timing_neutral_ = false;
  int8_t direction_ = 0;
  uint32_t neutral_since_ms_ = 0;
};

}  // namespace robot::ground

#endif  // CHASSIS_GROUND_TRIAL_HPP_
