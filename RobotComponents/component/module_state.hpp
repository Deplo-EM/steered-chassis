/**
 *******************************************************************************
 * @file      :module_state.hpp
 * @brief     : 底盘板与云台板共享的状态类型与枚举
 * @history   :
 *  Version     Date            Author          Note
 *  V1.0.0      2026-10-19      <author>        1. 完成共享类型定义
 *******************************************************************************
 * @attention :
 *  本文件被底盘板、云台板同时编译，只允许出现「纯类型 / 枚举 / 内联函数」，
 *  不允许依赖电机、CAN、HAL 等任何具体硬件，否则另一块板编不过。
 *
 *  坐标系约定（全工程统一，两板一致）：
 *    - 车体坐标系（底盘系）：x 轴朝车头正前方，y 轴朝车的左侧，z 轴竖直向上，
 *      满足右手法则（从上方看，逆时针旋转为正）。
 *    - 云台坐标系：x 轴 = 枪管指向，y 轴 = 枪管左侧，同样右手系。
 *    - 一切「前进正方向」都以云台为准：vx > 0 永远表示「操作手正前方」，
 *      与车头朝向无关（见考核说明 3.2）。
 *
 *  角度一律用弧度（rad），线速度用 m/s，角速度用 rad/s，时间用 s。
 *  混用单位是这类工程最常见、也最隐蔽的 bug 来源，所以在这里一次定死。
 *******************************************************************************
 *  Copyright (c) 2024 Hello World Team, Zhejiang University.
 *  All Rights Reserved.
 *******************************************************************************
 */
/* Define to prevent recursive inclusion -------------------------------------*/
#ifndef MODULE_STATE_HPP_
#define MODULE_STATE_HPP_

/* Includes ------------------------------------------------------------------*/
#include <cmath>
#include <cstdint>

/* Exported macro ------------------------------------------------------------*/
/* Exported constants --------------------------------------------------------*/
namespace robot {

/** 圆周率与 2π（不用 M_PI，避免不同编译器下 M_PI 不存在） */
constexpr float kPi = 3.14159265358979f;
constexpr float kTwoPi = 2.0f * kPi;

/** 把任意角度归一化到 [-π, π)
 *
 *  为什么需要它：角度是「周期量」，359° 和 -1° 在物理上是同一个方向，
 *  但数值上差了 360°。云台 yaw 可以连续转 360°，如果直接拿 359° 和 1° 相减，
 *  控制器会以为要「倒转 358°」，电机就会猛地回摆（考核说明 3.5 明确要求处理）。
 *
 *  实现：先 fmod 到 (-2π, 2π)，再平移半个周期，最后减回去。
 *  等价于 fmod(angle + π, 2π) 落在 [0, 2π) 后再减 π。 */
inline float WrapPi(float angle)
{
  float wrapped = std::fmod(angle + kPi, kTwoPi);
  if (wrapped < 0.0f) {
    wrapped += kTwoPi;
  }
  return wrapped - kPi;
}

/** 把 value 限制在 [lo, hi] 之间，且不要求 lo < hi（内部自行比较） */
template <typename T>
inline T Clamp(T value, T bound1, T bound2)
{
  const T hi = (bound1 > bound2) ? bound1 : bound2;
  const T lo = (bound1 > bound2) ? bound2 : bound1;
  return (value > hi) ? hi : ((value < lo) ? lo : value);
}

/* ============================================================================
 * 1. 生命状态（状态机用）
 * ==========================================================================*/
/** 机器人生命状态：上电未就绪 → 恢复自检 → 正常工作
 *
 *  Dead          上电后还没准备好（遥控器没连上、电机没反馈、IMU 零漂未收敛）。
 *                此时所有电机必须输出 0，绝对不能动。
 *  Resurrection  自检/复位中：等待零漂收敛、把舵角目标拉到当前反馈、
 *                清空 PID 积分，避免进入 Working 的瞬间电机「跳一下」。
 *  Working       正常控制：才允许根据遥控器指令输出。
 *
 *  注意：这三个状态和「急停」不是一回事。急停是独立于状态机最外层的一道闸，
 *  任何状态下只要急停生效，输出立刻归零（见 RobotModules/robot.cpp）。 */
enum class LifeState : uint8_t {
  kDead = 0,
  kResurrection = 1,
  kWorking = 2,
};

/* ============================================================================
 * 2. 控制指令来源（本次只用遥控器，预留扩展）
 * ==========================================================================*/
enum class ControlSource : uint8_t {
  kNone = 0,    ///< 无指令源（输出应归零）
  kRemote = 1,  ///< 遥控器（DT7）
  kAuto = 2,    ///< 自动控制（视觉/上位机，本次不使用，预留）
};

/* ============================================================================
 * 3. 工作模式
 * ==========================================================================*/
/** 底盘工作模式（考核说明 3.1） */
enum class ChassisMode : uint8_t {
  kSeparate = 0,  ///< 分离：底盘与云台解耦，(vx, vy, w) 全部来自遥控器直接映射
  kFollow = 1,    ///< 跟随：车头自动对准云台朝向，w 由「云台-车头」夹角闭环给出
  kSpin = 2,      ///< 小陀螺：底盘绕中心持续自旋，平移方向仍以云台为基准
};

/** 云台工作模式（考核说明 3.1 / 云台控制那一节）
 *
 *  本次实现采用「混用」：yaw 走 IMU 世界系稳定（否则底盘一转，枪口就跟着转，
 *  跟随模式永远收敛不了），pitch 走关节空间限位控制（不需要抗车体转动）。
 *  枚举保留两种以便讲清设计取舍。 */
enum class GimbalMode : uint8_t {
  kJointSpace = 0,      ///< 纯关节空间：直接控制关节角
  kImuStabilized = 1,   ///< IMU 世界系：控制云台在世界坐标下的朝向
};

/* ============================================================================
 * 4. 运动状态结构体
 * ==========================================================================*/
/** 底盘速度指令 / 状态
 *
 *  vx, vy 定义在「云台坐标系」（单位 m/s），w 是车体绕 z 轴的角速度（rad/s）。
 *  为什么不直接定在车体系：考核要求「前进正方向始终是云台朝向」，
 *  操作手推杆时心里想的是「枪口方向往前走」，而不是「车头方向往前走」。
 *  所以指令先以云台为基准给出，再由 Chassis 逆解前旋转到车体系。 */
struct ChassisMotion {
  float vx = 0.0f;
  float vy = 0.0f;
  float w = 0.0f;

  ChassisMotion& operator+=(const ChassisMotion& other)
  {
    vx += other.vx;
    vy += other.vy;
    w += other.w;
    return *this;
  }

  ChassisMotion& operator-=(const ChassisMotion& other)
  {
    vx -= other.vx;
    vy -= other.vy;
    w -= other.w;
    return *this;
  }

  ChassisMotion& operator*=(float k)
  {
    vx *= k;
    vy *= k;
    w *= k;
    return *this;
  }

  void reset()
  {
    vx = 0.0f;
    vy = 0.0f;
    w = 0.0f;
  }
};

inline ChassisMotion operator+(ChassisMotion lhs, const ChassisMotion& rhs)
{
  lhs += rhs;
  return lhs;
}

inline ChassisMotion operator-(ChassisMotion lhs, const ChassisMotion& rhs)
{
  lhs -= rhs;
  return lhs;
}

inline ChassisMotion operator*(ChassisMotion motion, float k)
{
  motion *= k;
  return motion;
}

inline ChassisMotion operator*(float k, ChassisMotion motion)
{
  motion *= k;
  return motion;
}

/** 云台角度指令 / 状态（单位 rad） */
struct GimbalMotion {
  float yaw = 0.0f;    ///< yaw 角（世界系或关节系，由 GimbalMode 决定）
  float pitch = 0.0f;  ///< pitch 角（关节系）

  GimbalMotion& operator+=(const GimbalMotion& other)
  {
    yaw += other.yaw;
    pitch += other.pitch;
    return *this;
  }

  GimbalMotion& operator-=(const GimbalMotion& other)
  {
    yaw -= other.yaw;
    pitch -= other.pitch;
    return *this;
  }

  void reset()
  {
    yaw = 0.0f;
    pitch = 0.0f;
  }
};

/* Exported variables --------------------------------------------------------*/
/* Exported function prototypes ----------------------------------------------*/

/** 模式名字（调试/打印用） */
inline const char* ChassisModeName(ChassisMode mode)
{
  switch (mode) {
    case ChassisMode::kSeparate:
      return "Separate";
    case ChassisMode::kFollow:
      return "Follow";
    case ChassisMode::kSpin:
      return "Spin";
    default:
      return "Unknown";
  }
}

}  // namespace robot

#endif /* MODULE_STATE_HPP_ */
