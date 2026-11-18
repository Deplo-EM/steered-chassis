/**
 *******************************************************************************
 * @file      :chassis_ik_solver.cpp
 * @brief     : 舵轮底盘运动学逆解实现
 * @history   :
 *  Version     Date            Author          Note
 *  V1.0.0      2026-10-19      <author>        1. 完成编写
 *******************************************************************************
 * @attention :
 *  推导过程（务必对着纸画一遍）：
 *
 *  1) 把速度指令从云台系旋转到车体系
 *     设 δ = 云台朝向 − 车头朝向（逆时针为正），则云台系到车体系要「旋转 +δ」：
 *         vx' =  vx·cosδ − vy·sinδ
 *         vy' =  vx·sinδ + vy·cosδ
 *     检验：枪口在车头左边 90°（δ=+90°）时推「前进」，应往车体左侧（+y）走。
 *     （考核说明 3.2 印的公式是 R(−δ)，与其自身 δ 定义不一致，详见 .cpp 里的说明。）
 *
 *  2) 刚体上任一点的速度 = 平动 + 转动
 *     轮组 i 位置 r_i = (x_i, y_i)，车体角速度 ω = w·ẑ，则
 *         轮组速度 = (vx', vy') + ω × r_i
 *     ω × r_i = (−w·y_i, w·x_i)，所以
 *         v_ix = vx' − w·y_i
 *         v_iy = vy' + w·x_i
 *
 *  3) 该轮组需要「朝向 atan2(v_iy, v_ix)、线速度 |v_i|」。
 *
 *  自洽性检查（手工验证用）：
 *     - 纯前进 w=0, vx=1, vy=0, δ=0 → 四个轮组都是 θ=0、v=1。✓
 *     - 纯自旋 w=1, v=0      → 左前轮位置 (+, +)，速度 (−y, +x) 指向左后方，
 *       即各轮都沿着「绕中心逆时针的切线」方向。✓
 *******************************************************************************
 *  Copyright (c) 2024 Hello World Team, Zhejiang University.
 *  All Rights Reserved.
 *******************************************************************************
 */
/* Includes ------------------------------------------------------------------*/
#include "chassis_ik_solver.hpp"

/* Private macro -------------------------------------------------------------*/
namespace {
/** 低于这个速度就不更新舵角目标（速度太小方向没有意义，硬算会让轮子乱摆） */
constexpr float kSpeedDeadband = 0.02f;  // m/s
/** 两支等效解与实测角距离相同的分界（90°）。 */
constexpr float kHalfPi = 0.5f * robot::kPi;

/** 在实测角为 0 的局部坐标里，检查整个有向路径（含起点和终点）。
 *  center 是已归一化的禁入区中心；检查相邻两圈，可处理跨 ±π 的区间。 */
bool SteerArcClear(float center, float half_width, float travel)
{
  const float start = std::fmin(0.0f, travel);
  const float end = std::fmax(0.0f, travel);
  for (int turn = -1; turn <= 1; ++turn) {
    const float shifted = center + static_cast<float>(turn) * robot::kTwoPi;
    if (start <= shifted + half_width && end >= shifted - half_width) return false;
  }
  return true;
}
}  // namespace
/* Private constants ---------------------------------------------------------*/
/* Private types -------------------------------------------------------------*/
/* Private variables ---------------------------------------------------------*/
/* External variables --------------------------------------------------------*/
/* Private function prototypes -----------------------------------------------*/
/* Exported function definitions ---------------------------------------------*/

void hello_world::chassis_ik_solver::ChassisIkSolver::init(
    const Geometry& geo, float max_steer_rate, float max_wheel_speed,
    float steer_flip_margin)
{
  geo_ = geo;
  max_steer_rate_ = max_steer_rate;
  max_wheel_speed_ = max_wheel_speed;
  steer_flip_margin_ = steer_flip_margin;
  reset(nullptr);
}

void hello_world::chassis_ik_solver::ChassisIkSolver::reset(
    const float steer_now[kWheelNum])
{
  for (int i = 0; i < kWheelNum; ++i) {
    command_[i] = WheelCommand{};
    branch_[i] = SteerBranch::kUnselected;
    last_angle_[i] = (steer_now != nullptr && std::isfinite(steer_now[i])) ? steer_now[i] : 0.0f;
  }
  has_last_ = (steer_now != nullptr);
  chassis_motion_.reset();
  last_solve_valid_ = false;
  keepout_blocked_mask_ = 0u;
}

float hello_world::chassis_ik_solver::ChassisIkSolver::wheelPosX(int index) const
{
  /* 左前 / 右前在 +x，左后 / 右后在 −x */
  switch (index) {
    case kLeftFront:
    case kRightFront:
      return geo_.half_length;
    case kLeftBack:
    case kRightBack:
      return -geo_.half_length;
    default:
      return 0.0f;
  }
}

float hello_world::chassis_ik_solver::ChassisIkSolver::wheelPosY(int index) const
{
  /* 左前 / 左后在 +y，右后 / 右前在 −y */
  switch (index) {
    case kLeftFront:
    case kLeftBack:
      return geo_.half_width;
    case kRightBack:
    case kRightFront:
      return -geo_.half_width;
    default:
      return 0.0f;
  }
}

void hello_world::chassis_ik_solver::ChassisIkSolver::solve(
    const robot::ChassisMotion& motion, float delta,
    const float steer_now[kWheelNum], float dt, WheelCommand out[kWheelNum], bool keep_rotation)
{
  auto reject = [&]() {
    reset();
    if (out != nullptr) for (int i = 0; i < kWheelNum; ++i) out[i] = WheelCommand{};
  };
  if (out == nullptr || !std::isfinite(dt) || dt <= 0.0f ||
      !std::isfinite(delta) || !std::isfinite(motion.vx) ||
      !std::isfinite(motion.vy) || !std::isfinite(motion.w) ||
      !std::isfinite(geo_.wheel_radius) || geo_.wheel_radius <= 0.0f ||
      !std::isfinite(geo_.half_length) || geo_.half_length <= 0.0f ||
      !std::isfinite(geo_.half_width) || geo_.half_width <= 0.0f ||
      !std::isfinite(max_steer_rate_) || max_steer_rate_ <= 0.0f ||
      !std::isfinite(max_wheel_speed_) || max_wheel_speed_ <= 0.0f ||
      !std::isfinite(steer_flip_margin_) || steer_flip_margin_ < 0.0f ||
      steer_flip_margin_ >= kHalfPi ||
      !std::isfinite(max_steer_rate_ * dt)) {
    reject(); return;
  }
  for (int i = 0; i < kWheelNum; ++i) {
    if (steer_now != nullptr && !std::isfinite(steer_now[i])) { reject(); return; }
  }

  /* ---------- 第 1 步：云台系 → 车体系 ----------
   * 设云台系相对车体系绕 z 轴转了 +δ（δ 从车头 +x 逆时针量到枪口 +x），则
   *    x_云台 在车体系里 = ( cosδ, sinδ)
   *    y_云台 在车体系里 = (−sinδ, cosδ)
   * 一个向量在云台系的坐标是 (vx, vy)，说明
   *    v = vx·x_云台 + vy·y_云台
   * 代进车体系坐标就得到「旋转 +δ」：
   *    vx_c = vx·cosδ − vy·sinδ
   *    vy_c = vx·sinδ + vy·cosδ
   *
   * 直觉检验：δ=+90°（枪口在车头左边），操作手推「前进」。
   *   云台系 (1,0) → 车体系 (0,+1) = 车体左侧 → 车往左平移。
   * 这正是我们要的：车沿枪口方向走。
   *
   * ⚠️ 考核说明 3.2 印的公式是 R(−δ)（两个 sin 的符号相反），与它自己
   *   「δ = 云台朝向 − 车头朝向」的定义不一致；按物理行为应为上面这个 R(+δ)。
   *   本工程以物理正确性为准，并用单测锁定（见 tests/test_main.cpp 的
   *   「云台在车头左侧 90°」与「小陀螺世界方向不变」两个用例）。
   *
   * 实车方向异常时先核对 yaw 正方向、零点和电机方向；坐标定义变更需同步推导与测试。 */
  const float cos_d = std::cos(delta);
  const float sin_d = std::sin(delta);
  float vx_c = motion.vx * cos_d - motion.vy * sin_d;
  float vy_c = motion.vx * sin_d + motion.vy * cos_d;
  float w = motion.w;
  if (keep_rotation) {
    const float module_radius = std::hypot(geo_.half_length, geo_.half_width);
    const float translation = std::hypot(vx_c, vy_c);
    if (!std::isfinite(module_radius) || module_radius <= 0.0f ||
        !std::isfinite(translation)) { reject(); return; }
    const float w_limit = max_wheel_speed_ / module_radius;
    w = robot::Clamp(w, -w_limit, w_limit);
    // 模长之和只用来快速排除超限；可能超限时按该轮的实际速度矢量求余量。
    if (translation > 0.0f && translation + std::fabs(w) * module_radius > max_wheel_speed_) {
      const float ux = vx_c / translation, uy = vy_c / translation;
      const float limit_squared = max_wheel_speed_ * max_wheel_speed_;
      if (!std::isfinite(limit_squared)) { reject(); return; }
      float allowed = translation;
      for (int i = 0; i < kWheelNum; ++i) {
        const float rx = -w * wheelPosY(i), ry = w * wheelPosX(i);
        const float along = ux * rx + uy * ry;
        const float across = ux * ry - uy * rx;
        // 平移沿原方向：满足 (平移 + along)^2 + across^2 <= 轮速上限^2。
        const float lateral_room = std::fmax(0.0f, limit_squared - across * across);
        allowed = std::fmin(allowed, std::fmax(0.0f, std::sqrt(lateral_room) - along));
      }
      vx_c *= allowed / translation;
      vy_c *= allowed / translation;
    }
  }
  chassis_motion_.vx = vx_c;
  chassis_motion_.vy = vy_c;
  chassis_motion_.w = w;

  float max_speed = 0.0f;
  uint8_t blocked_mask = 0u;
  /* ---------- 第 2~4 步：逐个轮组 ---------- */
  for (int i = 0; i < kWheelNum; ++i) {
    const float x_i = wheelPosX(i);
    const float y_i = wheelPosY(i);

    const float v_ix = vx_c - w * y_i;
    const float v_iy = vy_c + w * x_i;

    /* magnitude 一定为正：它是「轮子该转多快」；
     * sign 单独保存方向，等效解翻转时只翻 sign，不影响「是否低速」的判断。 */
    float magnitude = std::sqrt(v_ix * v_ix + v_iy * v_iy);
    if (!std::isfinite(magnitude)) { reject(); return; }
    if (magnitude > max_speed) max_speed = magnitude;
    float sign = 1.0f;
    float angle = std::atan2(v_iy, v_ix);

    /* ---- 限速起点：延续上次伺服目标，没有历史时从实测角起步 ---- */
    float reference;
    if (has_last_) {
      reference = last_angle_[i];
    } else if (steer_now != nullptr) {
      reference = steer_now[i];
    } else {
      reference = angle;
    }

    const float measured = steer_now != nullptr ? steer_now[i] : reference;
    const SteerKeepout& zone = geo_.steer_keepout[i];
    const bool avoid = zone.half_width_rad != 0.0f;
    const auto wheel_bit = static_cast<uint8_t>(1u << i);
    float keepout_center = 0.0f;
    bool reanchored = false;
    if (avoid) {
      // 位置未知、配置无效或启动时已在禁入区内，不尝试电动脱困。
      if (steer_now == nullptr || !std::isfinite(zone.center_rad) ||
          std::fabs(zone.center_rad) > robot::kPi || !std::isfinite(zone.half_width_rad) ||
          zone.half_width_rad <= 0.0f || zone.half_width_rad >= robot::kPi) {
        blocked_mask |= wheel_bit;
        continue;
      }
      keepout_center = robot::WrapPi(zone.center_rad - measured);
      if (!SteerArcClear(keepout_center, zone.half_width_rad, 0.0f)) {
        blocked_mask |= wheel_bit;
        continue;
      }
      // 反馈已偏离上次轨迹一侧时，从实测角重新出发，不能继续追不可达的历史目标。
      if (!SteerArcClear(keepout_center, zone.half_width_rad, robot::WrapPi(reference - measured))) {
        reference = measured;
        reanchored = true;
      }
    }

    float speed = 0.0f;
    float desired_angle = reference;
    if (magnitude > kSpeedDeadband) {
      /* 选支以实测为准：限速目标可能已走到，但舵机还没跟上。
       * 接近 90° 时保持上次选支；短暂经过零速也不丢这段历史。 */
      const float direct_error = robot::WrapPi(angle - measured);
      const float reversed_error = robot::WrapPi(direct_error + robot::kPi);
      const bool direct_ok = !avoid || SteerArcClear(keepout_center, zone.half_width_rad, direct_error);
#if defined(ROBOT_STEER_NO_FLIP) && ROBOT_STEER_NO_FLIP
      /* 现场让步（构建开关）：关闭等效解翻转，始终直取原始方向。
       * 快转时舵轮会有滞后，但不会再出现 ±180° 的突然换支。 */
      if (!direct_ok) {
        blocked_mask |= wheel_bit;
        continue;
      }
      const bool reverse = false;
#else
      const float margin = branch_[i] == SteerBranch::kUnselected ? 0.0f : steer_flip_margin_;
      const float threshold = kHalfPi + (branch_[i] == SteerBranch::kReversed ? -margin : margin);
      const bool reversed_ok = !avoid ||
          SteerArcClear(keepout_center, zone.half_width_rad, reversed_error);
      if (!direct_ok && !reversed_ok) {
        blocked_mask |= wheel_bit;
        continue;
      }
      // 两支都畅通时保留原回差。否则选可达的一支，并按其滚动方向给有符号轮速。
      const bool reverse = reversed_ok && (!direct_ok || std::fabs(direct_error) > threshold);
#endif
      branch_[i] = reverse ? SteerBranch::kReversed : SteerBranch::kDirect;
      const float desired_error = reverse ? reversed_error : direct_error;
      sign = reverse ? -1.0f : 1.0f;

      /* 在实测角两侧插值。若再对两误差之差做 WrapPi，+92° 到 -92°
       * 会走舵机背后，误差跨过 ±π 时输出反号。把结果接回原来的连续目标。 */
      float reference_error = robot::WrapPi(reference - measured);
      // 快速反向时，新目标可能已经贴合实测舵角。继续追离它很远的旧轨迹
      // 会让舵机先朝错方向用力，且空等轮速放行。就地起步仍限制本拍的实际角差。
      if (steer_now != nullptr && std::fabs(desired_error) <= kDriveAlignedRad &&
          std::fabs(desired_error - reference_error) > kDriveAlignedRad) {
        reference -= reference_error;  // 取与实测等价、且连续圈数最接近旧起点的表示。
        reference_error = 0.0f;
        reanchored = true;
      }
      const float move = desired_error - reference_error;
      desired_angle = reference + move;
      const float max_step = max_steer_rate_ * dt;
      angle = reference + robot::Clamp(move, -max_step, max_step);
      speed = sign * magnitude;
    } else {
      // 低速时方向没有意义，保持舵角并给零轮速。
      angle = reference;
    }

    if (!std::isfinite(angle) || !std::isfinite(desired_angle)) { reject(); return; }

    /* ---- 记录 ---- */
    last_angle_[i] = angle;
    command_[i].speed = speed;
    command_[i].angle = angle;
    command_[i].desired_angle = desired_angle;
    command_[i].reanchored = reanchored;
    out[i] = command_[i];
  }

  if (blocked_mask != 0u) {
    reject();  // 任一轮不可达就不释放剩余三轮；保留本拍轮位原因。
    keepout_blocked_mask_ = blocked_mask;
    return;
  }
  keepout_blocked_mask_ = 0u;

  // 四轮统一缩放，保持刚体速度的比例关系和瞬时旋转中心。
  const float scale = max_speed > max_wheel_speed_ ? max_wheel_speed_ / max_speed : 1.0f;
  chassis_motion_.vx *= scale;
  chassis_motion_.vy *= scale;
  chassis_motion_.w *= scale;
  for (int i = 0; i < kWheelNum; ++i) {
    command_[i].speed *= scale;
    out[i] = command_[i];
  }
  has_last_ = true;
  last_solve_valid_ = true;
}

/* Private function definitions ----------------------------------------------*/
