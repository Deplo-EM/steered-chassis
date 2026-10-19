/**
 *******************************************************************************
 * @file      :ins_motor.cpp
 * @brief     : 底盘板电机实例化（4×M3508 驱动 + 4×GM6020 转向）
 * @history   :
 *  Version     Date            Author          Note
 *  V1.0.0      2026-10-19      <author>        1. 完成电机配置
 *******************************************************************************
 * @attention :
 *  ★★★ 这一页是「整车参数总表」，实车接线 / 标定改了，只改这里 ★★★
 *
 *  轮组编号（与运动学逆解顺序一致）：
 *      0 = 左前 LF    1 = 左后 LB    2 = 右后 RB    3 = 右前 RF
 *
 *  ── 标定三步（第一次装车必须做，否则方向可能全反）──
 *  第 1 步 转向电机正方向：
 *      把车架起来，给某个舵电机一个正电压，看轮子是不是「向左转（俯视逆时针）」。
 *      若是向右转 → 把该电机的 kSteerDir 改成 kDirRev。
 *  第 2 步 驱动电机正方向：
 *      给某个轮电机一个正电流，看轮子是不是朝「车头方向」滚。
 *      若是倒着滚 → 把该电机的 kWheelDir 改成 kDirRev。
 *  第 3 步 舵角零点：
 *      把轮子摆正（轮面与车头方向平行），读该舵电机的 `motor->angle()`，
 *      把这个读数填进 kSteerAngleOffset（这样 angle() 就会变成 0）。
 *
 *  ── 减速比 ──
 *  M3508 出厂齿轮箱是 19.2:1，但本车按考核资料配置为 14:1 外置减速箱，
 *  因此用 remove_build_in_reducer=true + ex_redu_rat=14 覆盖默认值。
 *  如果实车其实是出厂 19.2:1，把 remove_build_in_reducer 改成 false、
 *  ex_redu_rat 改成 1（速度反馈会变约 37%，速度环要重新整定）。
 *******************************************************************************
 *  Copyright (c) 2024 Hello World Team, Zhejiang University.
 *  All Rights Reserved.
 *******************************************************************************
 */
/* Includes ------------------------------------------------------------------*/
#include "ins_motor.hpp"
/* Private macro -------------------------------------------------------------*/
/* Private constants ---------------------------------------------------------*/

namespace {

using hello_world::motor::AngleRange;
using hello_world::motor::Dir;
using hello_world::motor::InputType;
using hello_world::motor::OptionalParams;
/* Dir 是不限定作用域的枚举，枚举值在命名空间里，需要单独引入 */
using hello_world::motor::kDirFwd;

/* ---------------- 驱动电机 M3508（4 个，挂 CAN1） ---------------- */

/** 电机 ID（1~4 共用 0x200 控制帧） */
constexpr uint8_t kWheelId[4] = {1, 2, 3, 4};

/** 转向关系：1 = 正电流使轮子朝车头滚；-1 = 反。按标定第 2 步填。 */
constexpr Dir kWheelDir[4] = {kDirFwd, kDirFwd, kDirFwd, kDirFwd};

/** 轮电机减速比（外置减速箱） */
constexpr float kWheelReduRatio = 14.0f;

/** 驱动电流软件限幅，单位 A（M3508 满量程 20A，这里先给一半） */
constexpr float kWheelCurrentLimit = 8.0f;

/* ---------------- 转向电机 GM6020（4 个，挂 CAN2） ---------------- */

/** 电机 ID（1~4 共用 0x1FF 电压控制帧） */
constexpr uint8_t kSteerId[4] = {1, 2, 3, 4};

/** 转向关系：1 = 正电压使轮子俯视逆时针转 */
constexpr Dir kSteerDir[4] = {kDirFwd, kDirFwd, kDirFwd, kDirFwd};

/** 舵角零点偏移，单位 rad。按标定第 3 步填。 */
constexpr float kSteerAngleOffset[4] = {0.0f, 0.0f, 0.0f, 0.0f};

/** 转向电压软件限幅（GM6020 满量程 ±25000，给 80%） */
constexpr float kSteerVoltageLimit = 20000.0f;

/** 电机对象：函数内静态，第一次调用时构造（此时硬件时钟已初始化） */
hello_world::motor::M3508 s_wheels[4];
hello_world::motor::GM6020 s_steers[4];
bool s_inited = false;

void InitMotors()
{
  if (s_inited) {
    return;
  }

  for (int i = 0; i < 4; ++i) {
    OptionalParams opt;
    opt.input_type = InputType::kCurr;              ///< 电流控制：力可控、好限幅
    opt.angle_range = AngleRange::kNegInfToPosInf;  ///< 轮子连续转，不归一化
    opt.dir = kWheelDir[i];
    opt.remove_build_in_reducer = true;  ///< 拆除出厂减速箱
    opt.ex_redu_rat = kWheelReduRatio;   ///< 换成实车减速比
    opt.max_curr_input_lim = kWheelCurrentLimit;
    s_wheels[i].init(kWheelId[i], opt);
  }

  for (int i = 0; i < 4; ++i) {
    OptionalParams opt;
    opt.input_type = InputType::kRaw;  ///< 电压控制（不需要上位机开电流环）
    opt.angle_range = AngleRange::kNegPiToPosPi;  ///< 舵角用 [-π, π)
    opt.dir = kSteerDir[i];
    opt.angle_offset = kSteerAngleOffset[i];
    opt.max_raw_input_lim = kSteerVoltageLimit;
    s_steers[i].init(kSteerId[i], opt);
  }

  s_inited = true;
}

}  // namespace

/* Exported function definitions ---------------------------------------------*/

hello_world::motor::Motor *GetMotorWheel(int idx)
{
  InitMotors();
  if (idx < 0 || idx >= 4) {
    return nullptr;
  }
  return &s_wheels[idx];
}

hello_world::motor::Motor *GetMotorSteer(int idx)
{
  InitMotors();
  if (idx < 0 || idx >= 4) {
    return nullptr;
  }
  return &s_steers[idx];
}

/* Private function definitions ----------------------------------------------*/
