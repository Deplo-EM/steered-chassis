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
/** 半圈 */
constexpr float kHalfPi = 0.5f * 3.14159265358979f;
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
    last_angle_[i] = (steer_now != nullptr && std::isfinite(steer_now[i])) ? steer_now[i] : 0.0f;
  }
  has_last_ = (steer_now != nullptr);
  chassis_motion_.reset();
  last_solve_valid_ = false;
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
    const float steer_now[kWheelNum], float dt, WheelCommand out[kWheelNum])
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
  const float vx_c = motion.vx * cos_d - motion.vy * sin_d;
  const float vy_c = motion.vx * sin_d + motion.vy * cos_d;
  chassis_motion_.vx = vx_c;
  chassis_motion_.vy = vy_c;
  chassis_motion_.w = motion.w;

  float max_speed = 0.0f;
  /* ---------- 第 2~4 步：逐个轮组 ---------- */
  for (int i = 0; i < kWheelNum; ++i) {
    const float x_i = wheelPosX(i);
    const float y_i = wheelPosY(i);

    const float v_ix = vx_c - motion.w * y_i;
    const float v_iy = vy_c + motion.w * x_i;

    /* magnitude 一定为正：它是「轮子该转多快」；
     * sign 单独保存方向，等效解翻转时只翻 sign，不影响「是否低速」的判断。 */
    float magnitude = std::sqrt(v_ix * v_ix + v_iy * v_iy);
    if (!std::isfinite(magnitude)) { reject(); return; }
    if (magnitude > max_speed) max_speed = magnitude;
    float sign = 1.0f;
    float angle = std::atan2(v_iy, v_ix);

    /* ---- 参考角：优先用上一次的目标（连续、无跳变），没有历史就用实测 ---- */
    float reference;
    if (has_last_) {
      reference = last_angle_[i];
    } else if (steer_now != nullptr) {
      reference = steer_now[i];
    } else {
      reference = angle;
    }

    /* ---- 等效解选择 ----
     * d = 从参考角转到「原始解」需要转过的角度（已归一化到 [−π, π)）。
     * 若 |d| 超过 90°(+回差)，说明「转 180°、轮速取反」的另一支更近，
     * 此时把目标角加上 π、轮速取反。 */
    float diff = robot::WrapPi(angle - reference);
    if (magnitude > kSpeedDeadband &&
        std::fabs(diff) > (kHalfPi + steer_flip_margin_)) {
      angle += 3.14159265358979f;
      sign = -sign;
      diff = robot::WrapPi(angle - reference);
    }

    // 最终等效角与限速后的伺服目标必须分开，驱动门不能拿小步进角当已对齐。
    float desired_angle = reference + diff;

    /* ---- 速率限幅：目标角每周期最多变化 max_steer_rate*dt ----
     * 注意是在「连续角」上做限幅，这样目标不会绕远路。 */
    const float max_step = max_steer_rate_ * dt;
    if (diff > max_step) {
      diff = max_step;
    } else if (diff < -max_step) {
      diff = -max_step;
    }

    /* ---- 速度太小：保持上一次的舵角，轮速给 0 ----
     * 否则停车时会因为 atan2(0,0) 把四个轮子胡乱摆正，既难看又吵。 */
    float speed = 0.0f;
    if (magnitude <= kSpeedDeadband) {
      speed = 0.0f;
      angle = has_last_ ? last_angle_[i] : reference;
      desired_angle = angle;
    } else {
      angle = reference + diff;
      speed = sign * magnitude;
    }

    if (!std::isfinite(angle) || !std::isfinite(desired_angle)) { reject(); return; }

    /* ---- 记录 ---- */
    last_angle_[i] = angle;
    command_[i].speed = speed;
    command_[i].angle = angle;
    command_[i].desired_angle = desired_angle;
    out[i] = command_[i];
  }

  // 四轮统一缩放，保持刚体速度的比例关系和瞬时旋转中心。
  const float scale = max_speed > max_wheel_speed_ ? max_wheel_speed_ / max_speed : 1.0f;
  for (int i = 0; i < kWheelNum; ++i) {
    command_[i].speed *= scale;
    out[i] = command_[i];
  }
  has_last_ = true;
  last_solve_valid_ = true;
}

/* Private function definitions ----------------------------------------------*/
