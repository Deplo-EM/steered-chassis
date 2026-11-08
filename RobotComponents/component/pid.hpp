/**
 *******************************************************************************
 * @file      :pid.hpp
 * @brief     : 通用 PID 控制器（位置式）
 * @history   :
 *  Version     Date            Author          Note
 *  V1.0.0      2026-10-19      <author>        1. 完成编写
 *******************************************************************************
 * @attention :
 *  1. 本文件只依赖 <cmath>，不依赖任何硬件，底盘板 / 云台板共用，
 *     也可以在电脑上单独跑单元测试（tests/ 目录）。
 *  2. 微分项作用在「误差」上，并做了一阶低通滤波（否则 1ms 周期下反馈的
 *     量化噪声会被放大成剧烈抖动）。
 *  3. 积分项带限幅 + 抗饱和：输出已经顶到上限、误差还在往同方向推时，
 *     不再继续累加积分，避免「积分灌死」后松杆还要等很久才退出来。
 *******************************************************************************
 *  Copyright (c) 2024 Hello World Team, Zhejiang University.
 *  All Rights Reserved.
 *******************************************************************************
 */
/* Define to prevent recursive inclusion -------------------------------------*/
#ifndef PID_HPP_
#define PID_HPP_

/* Includes ------------------------------------------------------------------*/
#include <cmath>

namespace hello_world {
namespace pid {

/* Exported types ------------------------------------------------------------*/

/**
 * @brief 位置式 PID
 *
 * 输出 = Kp * e + Ki * ∫e dt + Kd * de/dt
 *        （其中 de/dt 经过一阶低通）
 *
 * 使用流程：
 *   1. 构造 / init() 设定 Kp、Ki、Kd、输出限幅、积分限幅；
 *   2. 每个控制周期调用一次 update(目标值, 反馈值, 周期)；
 *   3. 状态切换 / 长时间不控制时调用 reset()，把积分和微分历史清干净。
 *
 * 为什么用位置式而不是增量式：位置式输出就是「最终指令」本身，断电/复位后
 * 行为可预测；对舵角、云台角度这种需要「绝对保持」的对象更直观。
 */
class Pid {
 public:
  Pid() = default;

  /**
   * @param kp 比例系数
   * @param ki 积分系数
   * @param kd 微分系数
   * @param out_lim 输出限幅（对称，±out_lim）
   * @param i_lim 积分项单独限幅（对称，防止积分无限累积）
   */
  Pid(float kp, float ki, float kd, float out_lim, float i_lim)
      : kp_(kp), ki_(ki), kd_(kd), out_lim_(out_lim), i_lim_(i_lim)
  {
  }

  /** 初始化 / 重新设定参数（同时清空内部状态） */
  void init(float kp, float ki, float kd, float out_lim, float i_lim)
  {
    kp_ = kp;
    ki_ = ki;
    kd_ = kd;
    out_lim_ = std::fabs(out_lim);
    i_lim_ = std::fabs(i_lim);
    reset();
  }

  /** 清空积分与微分历史，首帧跳过差分以免产生微分尖峰。
   *  @param measure 仅为调用兼容保留，不参与计算；微分使用误差历史。 */
  void reset(float measure = 0.0f)
  {
    integral_ = 0.0f;
    last_error_ = 0.0f;
    d_filtered_ = 0.0f;
    (void)measure;
    output_ = 0.0f;
    first_run_ = true;
  }

  /**
   * @brief 计算一次 PID 输出
   * @param target    目标值
   * @param measure   反馈值
   * @param dt        控制周期，单位 s（本项目固定 0.001）
   * @retval 限幅后的输出
   */
  float update(float target, float measure, float dt)
  {
    return updateError(target - measure, dt);
  }

  /**
   * @brief 直接给定误差的版本（角度环用：误差需要先做 ±π 归一化再送进来）
   * @param error 已经算好、且必要时归一化过的误差
   * @param dt    控制周期，单位 s
   */
  float updateError(float error, float dt)
  {
    if (!std::isfinite(error) || !std::isfinite(dt) || dt <= 0.0f ||
        !std::isfinite(kp_) || !std::isfinite(ki_) || !std::isfinite(kd_) ||
        !std::isfinite(out_lim_) || !std::isfinite(i_lim_)) {
      reset();
      return 0.0f;
    }

    /* ---- 微分项：对误差差分，再做一阶低通 ----
     * 低通系数 d_lpf_ 越小越平滑、但相位滞后越大。
     * 1ms 周期、舵机角度环这种噪声水平，取 0.1~0.2 比较合适。 */
    float d_raw = 0.0f;
    if (!first_run_) {
      d_raw = (error - last_error_) / dt;
    }
    d_filtered_ += d_lpf_ * (d_raw - d_filtered_);

    /* ---- 积分项：先累加，再限幅 ---- */
    const float previous_integral = integral_;
    integral_ += error * dt;
    if (integral_ > i_lim_) {
      integral_ = i_lim_;
    } else if (integral_ < -i_lim_) {
      integral_ = -i_lim_;
    }

    float out = kp_ * error + ki_ * integral_ + kd_ * d_filtered_;

    /* ---- 抗饱和（条件积分）：输出已饱和且误差仍在把输出往饱和方向推时，
     *      撤销本次积分，防止「越顶越积」。 ---- */
    const float integral_push = ki_ * (integral_ - previous_integral);
    if ((out > out_lim_ && integral_push > 0.0f) ||
        (out < -out_lim_ && integral_push < 0.0f)) {
      // 限幅后的积分不能减回 error*dt；直接恢复保存值，避免越过积分下限。
      integral_ = previous_integral;
      out = kp_ * error + ki_ * integral_ + kd_ * d_filtered_;
    }

    if (!std::isfinite(out) || !std::isfinite(integral_) || !std::isfinite(d_filtered_)) {
      reset();
      return 0.0f;
    }

    /* ---- 输出限幅 ---- */
    if (out > out_lim_) {
      out = out_lim_;
    } else if (out < -out_lim_) {
      out = -out_lim_;
    }

    last_error_ = error;
    output_ = out;
    first_run_ = false;
    return out;
  }

  /* 参数修改（在线整定时可以临时改，不必 reset） */
  void setGains(float kp, float ki, float kd)
  {
    kp_ = kp;
    ki_ = ki;
    kd_ = kd;
  }
  void setOutputLimit(float out_lim) { out_lim_ = std::fabs(out_lim); }
  void setIntegralLimit(float i_lim)
  {
    i_lim_ = std::fabs(i_lim);
    // 在线缩小限幅时立即收紧状态，否则下一次抗饱和可能回滚到旧的越界积分。
    if (!std::isfinite(i_lim_)) integral_ = 0.0f;
    else if (integral_ > i_lim_) integral_ = i_lim_;
    else if (integral_ < -i_lim_) integral_ = -i_lim_;
  }
  /** 微分低通系数，范围 (0, 1]，1 表示不滤波 */
  void setDerivativeLpf(float alpha)
  {
    if (alpha > 0.0f && alpha <= 1.0f) {
      d_lpf_ = alpha;
    }
  }

  /* 状态查询 */
  float output() const { return output_; }
  float integral() const { return integral_; }
  float kp() const { return kp_; }
  float ki() const { return ki_; }
  float kd() const { return kd_; }

 private:
  float kp_ = 0.0f;
  float ki_ = 0.0f;
  float kd_ = 0.0f;
  float out_lim_ = 0.0f;
  float i_lim_ = 0.0f;

  float integral_ = 0.0f;    ///< 误差×s；积分输出为 ki_ * integral_
  float last_error_ = 0.0f;  ///< 上一周期误差（用于差分）
  float d_filtered_ = 0.0f;  ///< 低通后的微分值
  float output_ = 0.0f;

  float d_lpf_ = 0.15f;  ///< 微分一阶低通系数
  bool first_run_ = true;  ///< 首帧不做差分（没有「上一帧」）
};

}  // namespace pid
}  // namespace hello_world

#endif /* PID_HPP_ */
