/**
 *******************************************************************************
 * @file      :ins_motor.cpp
 * @brief     : 底盘板电机实例化（4×M3508 驱动 + 4×GM6020 转向）
 * @history   :
 *  Version     Date            Author          Note
 *  V1.0.0      2026-10-19      <author>        1. 完成电机配置
 *******************************************************************************
 * @attention :
 *  本文件集中电机实例参数；几何、PID、限位、通信分别见对应模块与 docs/14。
 *
 *  轮组编号（与运动学逆解顺序一致）：
 *      0 = 左前 LF    1 = 左后 LB    2 = 右后 RB    3 = 右前 RF
 *
 *  ── 方向和零点从哪里来 ──
 *  下面的 kSteerDir / kSteerAngleOffset / kWheelDir 已经按考核说明书
 *  （已补全版）6.2 的「方向 / 偏移」列**全部预置**，正常情况下不需要再试。
 *  上车时只需要做一次「验证」（不是标定）：
 *    第 1 步 给某个舵电机一个正电压，看轮子是否俯视逆时针转；
 *    第 2 步 给某个轮电机一个正电流，看轮子是否朝车头方向滚；
 *    第 3 步 把四个轮子摆正，读 `motor->angle()`，应都接近 0。
 *  若某台不符 → 说明该电机的接线/安装与说明书不一致，
 *  按实际现象改对应的 kSteerDir / kWheelDir / kSteerAngleOffset。
 *
 *  ── 减速比 ──
 *  考核说明书（已补全版）2.3 明确 M3508 减速比为 3591/187 ≈ 19.2（出厂齿轮箱），
 *  因此这里直接用组件库默认值：remove_build_in_reducer=false、ex_redu_rat=1。
 *  如果实车换过外置齿轮箱（例如 14:1），把 remove_build_in_reducer 改成 true，
 *  并把 kWheelExternalReduRatio 改成对应比值（速度反馈会变，速度环需重调）。
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
using hello_world::motor::kDirRev;

/* ---------------- 驱动电机 M3508（4 个，挂 CAN1） ---------------- */

/** 电机 ID（1~4 共用 0x200 控制帧） */
constexpr uint8_t kWheelId[4] = {1, 2, 3, 4};

/** 转向关系：1 = 正电流使轮子朝车头滚；-1 = 反。按标定第 2 步填。 */
constexpr Dir kWheelDir[4] = {kDirFwd, kDirFwd, kDirFwd, kDirFwd};

/** 轮电机外置减速箱的额外减速比。1.0 = 直接用出厂 19.2:1（说明书 2.3） */
constexpr float kWheelExternalReduRatio = 1.0f;

/** 驱动电流软件限幅，单位 A（M3508 满量程 20A，这里先给一半） */
constexpr float kWheelCurrentLimit = 8.0f;

/* ---------------- 转向电机 GM6020（4 个，挂 CAN2） ---------------- */

/** 电机 ID（1~4 共用 0x1FF 电压控制帧） */
constexpr uint8_t kSteerId[4] = {1, 2, 3, 4};

/** 转向关系：说明书 6.2 给出四台转向电机均为「反向」 */
constexpr Dir kSteerDir[4] = {kDirRev, kDirRev, kDirRev, kDirRev};

/** 舵角零点偏移，单位 rad（说明书 6.2「舵向零点角度偏移」，固定值） */
constexpr float kSteerAngleOffset[4] = {2.61346f, -2.06844f, 0.00729f, 1.58518f};

/** 转向电压软件限幅（GM6020 满量程 ±25000，给 80%）。仅电压模式下使用。 */
constexpr float kSteerVoltageLimit = 20000.0f;

/** 转向电流软件限幅（GM6020 电流模式 ±16384 ↔ ±3A，给 3A）。 */
constexpr float kSteerCurrentLimit = 3.0f;

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
    opt.remove_build_in_reducer = false;        ///< 保留出厂 19.2:1 齿轮箱
    opt.ex_redu_rat = kWheelExternalReduRatio;  ///< 外加减速比（1.0 = 无）
    opt.max_curr_input_lim = kWheelCurrentLimit;
    s_wheels[i].init(kWheelId[i], opt);
  }

  for (int i = 0; i < 4; ++i) {
    OptionalParams opt;
    opt.input_type = InputType::kRaw;  ///< 电压控制（说明书 2.3：默认 kRaw，电流需上位机开环）
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
