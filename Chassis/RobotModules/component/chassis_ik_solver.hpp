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
 *  如果目标舵角突然从 +170° 变到 -170°，直接跟过去要转 340°，舵机原地大甩；
 *  而换成「-170°+180°=+10°、轮速取反」只需要转 20°。所以当目标与当前相差
 *  超过 90° 时，必须选另一支解。
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

/* Exported types ------------------------------------------------------------*/

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
};

/** 单个轮组的解算结果 */
struct WheelCommand {
  /** 轮子线速度，单位 m/s。
   *  正负号含义：正 = 沿当前舵角指向「向前」滚；负 = 反向滚。
   *  等效解翻转时这里会取反。 */
  float speed = 0.0f;

  /** 舵角目标，单位 rad（车体系，x 轴为 0，逆时针为正）。
   *  @note 这是「连续角」，可能超出 ±π（例如累计转到 4.2 rad）。
   *        连续的好处是：目标不会在 ±π 处跳变，PID 和速率限幅都好处理。 */
  float angle = 0.0f;
};

/**
 * @brief 舵轮底盘运动学逆解器
 *
 * 使用顺序：
 *   init(几何参数) → 每 1ms 调一次 solve(...) → 读 command(i) 拿结果。
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
   *        例如取 0.05：只有超过 90°+2.9° 才翻转，避免在 90° 附近来回跳。
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
   * @retval None
   */
  void solve(const robot::ChassisMotion& motion, float delta,
             const float steer_now[kWheelNum], float dt,
             WheelCommand out[kWheelNum]);

  /* 结果查询（solve 之后有效） */
  const WheelCommand& command(int index) const
  {
    return command_[(index < 0 || index >= kWheelNum) ? 0 : index];
  }

  const Geometry& geometry() const { return geo_; }
  const robot::ChassisMotion& chassis_motion() const { return chassis_motion_; }

  /** 轮组安装位置（车体系），index 见 WheelIndex */
  float wheelPosX(int index) const;
  float wheelPosY(int index) const;

 private:
  Geometry geo_;
  float max_steer_rate_ = 10.0f;
  float max_wheel_speed_ = 3.0f;
  float steer_flip_margin_ = 0.05f;

  WheelCommand command_[kWheelNum];
  /** 上一次的舵角目标（连续角），用于速率限幅与等效解选择 */
  float last_angle_[kWheelNum] = {0.0f, 0.0f, 0.0f, 0.0f};
  bool has_last_ = false;

  /** 最近一次使用的车体系速度（调试用） */
  robot::ChassisMotion chassis_motion_;
};

}  // namespace chassis_ik_solver
}  // namespace hello_world

#endif /* CHASSIS_IK_SOLVER_HPP_ */
