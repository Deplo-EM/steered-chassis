/**
 *******************************************************************************
 * @file      :ins_motor.cpp
 * @brief     : 云台板 yaw / pitch 达妙电机实例化
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
 *  ── 方向与零点 ──
 *  已按考核说明书（已补全版）6.2 的「方向 / 偏移」列预置：
 *      yaw  ：正向，角度偏移 1.60879 rad
 *      pitch：反向，角度偏移 2.91772 rad
 *  上车时只需验证：给 yaw 正力矩应俯视逆时针转；给 pitch 正力矩应使
 *  「角度增大 = 枪口抬高」。若不符，说明接线与说明书不一致，再改下面的常量。
 *******************************************************************************
 *  Copyright (c) 2024 Hello World Team, Zhejiang University.
 *  All Rights Reserved.
 *******************************************************************************
 */
/* Includes ------------------------------------------------------------------*/
#include "ins_motor.hpp"
#include "gimbal_drive_feedback.hpp"
/* Private macro -------------------------------------------------------------*/
/* Private constants ---------------------------------------------------------*/

namespace {

using hello_world::motor::AngleRange;
using hello_world::motor::Dir;
using hello_world::motor::DM_J4310;
// 最新拓扑标注 DMJ620：它确认接线和 ID，但没有给出 MIT 的 P/V/T 量程。
// 现有 DM_J4310 类型仅代表既有协议标度，待铭牌与电机只读参数核对后再替换。
using hello_world::motor::InputType;
using hello_world::motor::OptionalParams;
using hello_world::motor::kDirFwd;
using hello_world::motor::kDirRev;

/** yaw 电机 ID（发送 ID = 1，反馈 ID = 0x11） */
constexpr uint8_t kYawId = 1;
/** pitch 电机 ID（发送 ID = 2，反馈 ID = 0x12） */
constexpr uint8_t kPitchId = 2;

/** 方向：说明书 6.2 —— yaw 正向、pitch 反向 */
constexpr Dir kYawDir = kDirFwd;
constexpr Dir kPitchDir = kDirRev;

/** 关节零点偏移（说明书 6.2「方向 / 偏移」列），单位 rad。
 *  作用：让 angle() 在「枪口与车头对齐（yaw）/ 枪口水平（pitch）」时等于 0。 */
constexpr float kYawAngleOffset = 1.60879f;
constexpr float kPitchAngleOffset = 2.91772f;

/** 力矩软件限幅，单位 N·m；Gimbal 合成反馈和前馈时也读取此上限。 */
constexpr float kYawTorqueLimit = 3.5f;
constexpr float kPitchTorqueLimit = 3.5f;

DM_J4310 s_yaw_motor;
DM_J4310 s_pitch_motor;
bool s_inited = false;

/**
 * @brief 首次使用时按各轴 ID、方向、零偏和力矩上限装配电机对象。
 * @note  实例初始化不发使能，CommTask 等有效授权后才执行驱动握手。
 */
void InitMotors()
{
  if (s_inited) {
    return;
  }

  {
    OptionalParams opt;
    opt.input_type = InputType::kTorq;  ///< 本项目通过 MIT 力矩通道执行自写位置闭环。
    opt.angle_range = AngleRange::kNegPiToPosPi;
    opt.dir = kYawDir;
    opt.angle_offset = kYawAngleOffset;
    opt.max_torq_input_lim = kYawTorqueLimit;
    /* 仅通信任务在有效授权后送使能帧；驱动编码器不自动使能。 */
    s_yaw_motor.init(kYawId, opt, false, false);
  }

  {
    OptionalParams opt;
    opt.input_type = InputType::kTorq;
    opt.angle_range = AngleRange::kNegPiToPosPi;
    opt.dir = kPitchDir;
    opt.angle_offset = kPitchAngleOffset;
    opt.max_torq_input_lim = kPitchTorqueLimit;
    s_pitch_motor.init(kPitchId, opt, false, false);
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

/**
 * @brief 区分正常失能、正常使能与故障，给自检和恢复握手提供双轴状态。
 * @note  正常失能仍可参与就绪自检，disabled_mask 的 bit0/bit1 分别对应 yaw/pitch。
 */
robot::GimbalDriveFeedback GetGimbalDriveFeedback()
{
  const auto normally_disabled = [](const DM_J4310& motor) {
    return motor.status_code() == DM_J4310::kMotorDisabled && !motor.faultLatched();
  };
  const bool yaw_disabled = normally_disabled(s_yaw_motor);
  const bool pitch_disabled = normally_disabled(s_pitch_motor);
  return {(yaw_disabled || s_yaw_motor.isHealthy()) &&
              (pitch_disabled || s_pitch_motor.isHealthy()),
          static_cast<uint8_t>(unsigned(yaw_disabled) | (unsigned(pitch_disabled) << 1u))};
}

/* Private function definitions ----------------------------------------------*/
