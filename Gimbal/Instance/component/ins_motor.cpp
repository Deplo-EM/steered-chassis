/**
 *******************************************************************************
 * @file      :ins_motor.cpp
 * @brief     : 云台板电机实例化（yaw / pitch 双达妙 DM-J4310）
 * @history   :
 *  Version     Date            Author          Note
 *  V1.0.0      2026-10-19      <author>        1. 完成电机配置
 *******************************************************************************
 * @attention :
 *  ★ 达妙电机用之前必须先用上位机配置（否则不发指令 / 不动）★
 *      CAN ID   = 电机 ID     （本车：yaw=1，pitch=2）
 *      Master ID= 电机 ID + 0x10
 *      PMAX = 3.141593，VMAX = 21，TMAX = 7.5
 *      控制模式 = MIT
 *
 *  上电默认是「失能」状态，失能时既不动作也不反馈。本类配置了 auto_enable，
 *  会一直发使能指令直到电机反馈「已使能」，之后才开始正常控制。
 *
 *  ── 方向标定 ──
 *  给 yaw 一个正力矩，云台应该「俯视逆时针转（向左）」；若不是，把 kYawDir 改 kDirRev。
 *  给 pitch 一个正力矩，枪口应该「向下」或「向上」——本车约定「角度增大 = 枪口抬高」，
 *  若不是，把 kPitchDir 改 kDirRev。
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
using hello_world::motor::DM_J4310;
using hello_world::motor::InputType;
using hello_world::motor::OptionalParams;
using hello_world::motor::kDirFwd;

/** yaw 电机 ID（发送 ID = 1，反馈 ID = 0x11） */
constexpr uint8_t kYawId = 1;
/** pitch 电机 ID（发送 ID = 2，反馈 ID = 0x12） */
constexpr uint8_t kPitchId = 2;

constexpr Dir kYawDir = kDirFwd;
constexpr Dir kPitchDir = kDirFwd;

/** 力矩软件限幅，单位 N·m（DM-J4310 满量程 7.5，这里先给一半） */
constexpr float kYawTorqueLimit = 3.5f;
constexpr float kPitchTorqueLimit = 3.5f;

DM_J4310 s_yaw_motor;
DM_J4310 s_pitch_motor;
bool s_inited = false;

void InitMotors()
{
  if (s_inited) {
    return;
  }

  {
    OptionalParams opt;
    opt.input_type = InputType::kTorq;  ///< 达妙只支持力矩输入（MIT 模式）
    opt.angle_range = AngleRange::kNegPiToPosPi;
    opt.dir = kYawDir;
    opt.max_torq_input_lim = kYawTorqueLimit;
    /* auto_enable=true：上电自动使能；enable_mode_switch=false：固定 MIT 模式 */
    s_yaw_motor.init(kYawId, opt, true, false);
  }

  {
    OptionalParams opt;
    opt.input_type = InputType::kTorq;
    opt.angle_range = AngleRange::kNegPiToPosPi;
    opt.dir = kPitchDir;
    opt.max_torq_input_lim = kPitchTorqueLimit;
    s_pitch_motor.init(kPitchId, opt, true, false);
  }

  s_inited = true;
}

}  // namespace

/* Exported function definitions ---------------------------------------------*/

hello_world::motor::Motor *GetMotorYaw()
{
  InitMotors();
  return &s_yaw_motor;
}

hello_world::motor::Motor *GetMotorPitch()
{
  InitMotors();
  return &s_pitch_motor;
}

/* Private function definitions ----------------------------------------------*/
