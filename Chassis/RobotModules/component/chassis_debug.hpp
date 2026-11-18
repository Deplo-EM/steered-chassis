/**
 * @file chassis_debug.hpp
 * @brief 底盘调试快照：转向/轮速常用量收进一个符号，Ozone 展开即可，免翻符号树。
 *
 * 只读观察，不参与控制。索引 0~3 = 左前、左后、右后、右前。
 * 单位：角度 rad，轮速 rad/s，轮电流 A，舵输出 raw（电压或电流由构建决定）。
 */
#ifndef ROBOT_CHASSIS_DEBUG_HPP_
#define ROBOT_CHASSIS_DEBUG_HPP_

#include <cstdint>

namespace robot::diag {

/** 越界反馈的原值和轮位，供进入 Dead 时保存。 */
struct SteerAngleFault {
  uint8_t mask = 0;
  float measured_rad[4] = {};  ///< 只解释 mask 对应的元素。
};

struct ChassisDebug {
  volatile uint8_t state;             ///< 0=Dead, 1=Resurrection, 2=Working
  volatile uint8_t estop;             ///< 1=急停中
  volatile uint8_t authorized;        ///< 1=状态机允许下发；是否有轮驱动另看 drive_ready。
  volatile uint8_t align_blocked_mask;///< bit0–3：该轮仍未对齐

  volatile float steer_feedback[4];   ///< 驱动给出的单圈舵角 rad；异常保留原值。
  volatile float steer_target[4];     ///< 限速后的伺服目标角 rad（cmd.angle）
  volatile float steer_desired[4];    ///< 最终等效目标角 rad（desired_angle）
  volatile float steer_output[4];     ///< 舵角 PID 请求 raw（电压/电流按构建）
  volatile float wheel_velocity[4];   ///< 轮速实测 rad/s
  volatile float wheel_target[4];     ///< 轮速目标 rad/s（未放行/零速时为 0）
  volatile float wheel_output[4];     ///< 轮电流请求 A
  volatile float steer_error[4];      ///< PID 使用的周期误差 rad；停机后为 0。
  volatile uint8_t steer_angle_fault_mask; ///< 实时越界位；历史停机读 last_dead_.steer_angle。
  volatile uint8_t drive_ready;       ///< 1=有驱动需求且整组满足对齐条件。
  volatile uint8_t mode;              ///< 0=分离 1=跟随 2=小陀螺 3=其它
  volatile float delta_rad;           ///< 跟随用：云台相对车头角 rad
  volatile float follow_w_cmd;        ///< 跟随/小陀螺的 raw.w（限幅前）
  volatile float follow_gyro_z;       ///< 跟随阻尼用的底盘陀螺 z rad/s
  volatile float ramped_w;            ///< 斜坡后的实际旋转指令 rad/s
  volatile float drive_w_request;     ///< 轮速分配和整组对齐缩放后，理想条件下请求的偏航角速度 rad/s。
  volatile float gyro_z_radps;        ///< 底盘陀螺 Z 原始符号的角速度；和车体系正方向需按实车标定对照。
  volatile float alignment_scale;    ///< 四轮对齐共同限速比例；未对齐时为 0。
  volatile uint8_t steer_keepout_configured_mask;  ///< 配了干涉区的轮位。
  volatile uint8_t steer_keepout_blocked_mask;     ///< 当拍至少有一舵无可用路径时整组归零。
  volatile uint8_t last_steer_keepout_blocked_mask;///< 上次阻塞轮位，松杆后仍保留。
};

extern volatile ChassisDebug g_chassis_dbg;

/**
 * @brief 舵轮卡点测量专用视图：一个符号看四个轮子的跟随情况和已配置的干涉区。
 *
 * 用途：架空 -> 分离档慢速画圆，扫过舵角全行程时观察
 *   - target 在走、feedback 停住/滞后 -> 该角度就是卡点；
 *   - output 同时顶高 -> 卡点处出力饱和；
 *   - center_rad/half_width_rad 是当前配置值，方便核对填得对不对。
 * 索引 0~3 与 g_chassis_dbg 相同（解算器轮序），物理轮位仍需逐轮核对。
 */
struct KeepoutDebug {
  volatile float target[4];       ///< 伺服目标角 rad（cmd.angle）
  volatile float feedback[4];     ///< 实测舵角 rad
  volatile float velocity[4];     ///< 实测舵速 rad/s
  volatile float output[4];       ///< 舵角 PID 请求 raw
  volatile float error[4];        ///< 周期误差 rad
  volatile float center_rad[4];   ///< 已配置的卡点中心 rad（0 且半宽 0 表示未配置）
  volatile float half_width_rad[4];///< 已配置的卡点半宽 rad
  volatile uint8_t configured_mask;///< 配了干涉区的轮位
  volatile uint8_t blocked_mask;   ///< 当拍无可用路径的轮位
  volatile uint8_t in_zone_mask;   ///< 当前实测角落在干涉区内的轮位
};

extern volatile KeepoutDebug g_keepout_dbg;

}  // namespace robot::diag

#endif  // ROBOT_CHASSIS_DEBUG_HPP_
