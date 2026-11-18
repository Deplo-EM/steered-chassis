/**
 *******************************************************************************
 * @file      :chassis_ik_solver.hpp
 * @brief     : 舵轮底盘运动学逆解（纯数学，不依赖任何硬件）
 * @history   :
 *  Version     Date            Author          Note
 *  V1.0.0      2026-10-19      <author>        1. 完成编写
 *******************************************************************************
 * @attention :
 *  【什么是「逆解」】
 *  正解：知道 8 个电机怎么转 → 算出车往哪走。
 *  逆解：知道车想往哪走（vx, vy, w）→ 算出 8 个电机该怎么转。
 *  我们只需要逆解。
 *
 *  【舵轮和麦轮的区别】
 *  麦轮：轮子方向固定，靠辊子产生横向力，逆解是一组固定线性方程。
 *  舵轮：每个轮组自己有转向电机，轮子朝向是「自由变量」。所以每个轮组要解出
 *  两个量：转向角 θ 和轮子转速 v。判断依据只有一条：
 *      轮组中心的目标线速度矢量 → 轮子必须朝这个矢量方向、以这个大小转。
 *
 *  【轮组编号与安装位置】（车体坐标系：x 前、y 左）
 *      y
 *      ↑
 *      │  LB(-hl,+hw)        LF(+hl,+hw)
 *      │
 *      └──────── x         RB(-hl,-hw)        RF(+hl,-hw)
 *   索引固定为 0=左前 LF、1=左后 LB、2=右后 RB、3=右前 RF，
 *   与 Instance 层的 GetMotorWheel / GetMotorSteer 顺序一致。
 *
 *  【为什么要有「等效解翻转」】
 *  舵角是周期的：θ 和 θ+π 只差「轮子反过来转」，对整车运动完全等价。
 *  例如实测舵角为 0°、速度方向为 160°，转到 -20° 并反转轮速即可。
 *  选哪支解以实测角为准；再用上一次目标限制目标角的变化率。
 *******************************************************************************
 *  Copyright (c) 2024 Hello World Team, Zhejiang University.
 *  All Rights Reserved.
 *******************************************************************************
 */
/* Define to prevent recursive inclusion -------------------------------------*/
#ifndef CHASSIS_IK_SOLVER_HPP_
#define CHASSIS_IK_SOLVER_HPP_

/* Includes ------------------------------------------------------------------*/
#include <cmath>

#include "module_state.hpp"

namespace hello_world {
namespace chassis_ik_solver {

/* Exported constants --------------------------------------------------------*/
/** 实测舵角在此误差内时，正常驱动不因对齐误差降速。 */
inline constexpr float kDriveAlignedRad = 0.08f;

/* Exported types ------------------------------------------------------------*/

/** 单轮已测的舵角禁入区；半宽包括动态跟踪裕量，0 表示未配置。
 *
 * 坐标：车体系舵角（x 轴为 0，逆时针为正），与 steer_feedback / 命令角同一套。
 * 用途：某些轮位在固定角度会被机械刮擦卡住，逆解遇到该区间时改走另一支，
 *       避免让舵机一直顶住卡点。
 * 测法：架空、分离档慢速扫舵角，target 在走而 feedback 停住的那一段就是
 *       禁入区；中心取停住时的实测角，半宽取范围的一半。
 * 单位：rad。 */
struct SteerKeepout {
  float center_rad = 0.0f;     ///< 禁入区中心（实测舵角）
  float half_width_rad = 0.0f; ///< 半宽；0 表示该轮未配置
};

/** 整车几何参数（单位：m）。
 *  @note 数值来自考核说明书（已补全版）6.2「底盘几何参数」，实车如有改动请同步。 */
struct Geometry {
  /** 轮子半径 = 直径 132mm / 2 */
  float wheel_radius = 0.066f;

  /** 轮组中心到车体中心的距离（前后方向，x 轴）
   *  说明书 6.2：LF(0.167, 0.167) LB(-0.167, 0.167)
   *              RB(-0.167,-0.167) RF(0.167,-0.167) */
  float half_length = 0.167f;

  /** 轮组中心到车体中心的距离（左右方向，y 轴）。
   *  与 x 方向相同 → 四个轮组构成正方形。 */
  float half_width = 0.167f;

  /** 左前、左后、右后、右前。角度用该物理轮位标定后的车体系舵角。 */
  SteerKeepout steer_keepout[4] = {};
};

/** 单个轮组的解算结果 */
struct WheelCommand {
  /** 轮子线速度，单位 m/s。
   *  正负号含义：正 = 沿当前舵角指向「向前」滚；负 = 反向滚。
   *  等效解翻转（舵角转 180°）时这里会取反，两者合起来才是同一个滚动方向。 */
  float speed = 0.0f;

  /** 舵角目标，单位 rad（车体系，x 轴为 0，逆时针为正）。
   *  @note 这是「连续角」，可能超出 ±π（例如累计转到 4.2 rad）。
   *        连续的好处是：目标不会在 ±π 处跳变，PID 和速率限幅都好处理；
   *        代价是显示上会看到几十 rad 的累积值，实际跟的是折算后的方向。 */
  float angle = 0.0f;
  /** 未限速的最终等效舵角(rad)，用于判断驱动轮实际方向是否对齐：
   *  它和 angle 的差别只在于「是否已经用 max_steer_rate 限速」。 */
  float desired_angle = 0.0f;
  /** 本拍舍弃了已不适用的旧限速起点，需清除该轮的舵角 PID 历史。
   *  典型场景：快速换向后，新目标已经贴近实测角，而旧轨迹离得很远。 */
  bool reanchored = false;
};

/**
 * @brief 舵轮底盘运动学逆解器
 *
 * 使用顺序：
 *   init(几何参数) → 每 1ms 调一次 solve(...) → 读 command(i) 拿结果。
 *
 * 一次 solve 的五个步骤（与 .cpp 里的注释对应）：
 *   1) 云台系 → 车体系：把 (vx, vy) 按云台-车头夹角 δ 旋转；
 *   2) 单轮速度：v_i = 平移 + ω × r_i（r_i 为轮组相对车体中心的坐标）；
 *   3) 速度 → 舵角 atan2(vy, vx) 与轮速模长；
 *   4) 等效支选择（直取 / 反向 180°）+ 舵角限速 + 禁入区绕行；
 *   5) 小陀螺档保旋转：先满足 w，再用剩余轮速按比例缩放平移。
 */
class ChassisIkSolver {
 public:
  /** 轮组数量 */
  static constexpr int kWheelNum = 4;
  /** 轮组索引（与 Instance 层顺序一致） */
  enum WheelIndex : int {
    kLeftFront = 0,
    kLeftBack = 1,
    kRightBack = 2,
    kRightFront = 3,
  };

  ChassisIkSolver() = default;

  /**
   * @brief 初始化
   * @param geo             几何参数
   * @param max_steer_rate  舵角目标变化率上限，单位 rad/s（防止舵机猛打）
   * @param max_wheel_speed 轮速上限，单位 m/s（防止超速）
   * @param steer_flip_margin 等效解翻转的「回差」，单位 rad。
   *        例如取 0.05：超过 90°+2.9° 切到反向，低于 90°−2.9° 切回正向。
   */
  void init(const Geometry& geo, float max_steer_rate = 10.0f,
            float max_wheel_speed = 3.0f, float steer_flip_margin = 0.05f);

  /** 清空历史目标（状态切换、急停恢复时调用，避免残留的大角度目标） */
  void reset(const float steer_now[kWheelNum] = nullptr);

  /**
   * @brief 逆解一次
   * @param motion    云台坐标系下的速度指令 (vx, vy, w)，单位 m/s、m/s、rad/s
   * @param delta     云台相对车头的偏航角 δ（rad），即 G2C 帧里的 yaw_joint
   * @param steer_now 4 个舵角的当前实测值（rad，车体系），用于等效解选择；
   *                  传 nullptr 表示没有反馈（此时以历史目标为准）
   * @param dt        控制周期，单位 s
   * @param out       输出 4 个轮组的解
   * @param keep_rotation 小陀螺模式先分配旋转所需轮速，剩余轮速给同方向平移。
   * @retval None
   *
   * 失败处理：任一输入非有限/非正，或某轮找不到安全路径时，会重置历史并把
   * 四个轮组的输出清零（内部 reject()），由上层决定是否停车。
   * 调用后用 lastSolveValid() 和 keepoutBlockedMask() 判断本拍结果。
   */
  void solve(const robot::ChassisMotion& motion, float delta,
             const float steer_now[kWheelNum], float dt,
             WheelCommand out[kWheelNum], bool keep_rotation = false);

  /* 结果查询（solve 之后有效） */
  const WheelCommand& command(int index) const
  {
    return command_[(index < 0 || index >= kWheelNum) ? 0 : index];
  }

  bool lastSolveValid() const { return last_solve_valid_; }
  /** 本拍无法找到安全舵角路径的轮位。索引与轮速、舵角数组相同。 */
  uint8_t keepoutBlockedMask() const { return keepout_blocked_mask_; }

  const Geometry& geometry() const { return geo_; }
  const robot::ChassisMotion& chassis_motion() const { return chassis_motion_; }

  /** 轮组安装位置（车体系），index 见 WheelIndex */
  float wheelPosX(int index) const;
  float wheelPosY(int index) const;

 private:
  enum class SteerBranch : uint8_t { kUnselected, kDirect, kReversed };

  Geometry geo_;
  float max_steer_rate_ = 10.0f;
  float max_wheel_speed_ = 3.0f;
  float steer_flip_margin_ = 0.05f;

  WheelCommand command_[kWheelNum];
  /** 上一次的舵角目标（连续角），用于限制每周期的转角变化。 */
  float last_angle_[kWheelNum] = {0.0f, 0.0f, 0.0f, 0.0f};
  SteerBranch branch_[kWheelNum] = {};  ///< 低速也保留选支；仅 reset() 清除。
  bool has_last_ = false;
  bool last_solve_valid_ = false;
  uint8_t keepout_blocked_mask_ = 0u;

  /** 限幅后分配给四轮的车体系速度（对齐与实车跟踪另行处理）。 */
  robot::ChassisMotion chassis_motion_;
};

}  // namespace chassis_ik_solver
}  // namespace hello_world

#endif /* CHASSIS_IK_SOLVER_HPP_ */
