/**
 *******************************************************************************
 * @file      : diagnostics.cpp
 * @brief     : 运行期诊断数据实现
 * @history   :
 *  Version     Date            Author          Note
 *  V1.0.0      2026-10-20      <author>        1. 初版
 *******************************************************************************
 *  Copyright (c) 2024 Hello World Team, Zhejiang University.
 *  All Rights Reserved.
 *******************************************************************************
 */
/* Includes ------------------------------------------------------------------*/
#include "diagnostics.hpp"

namespace robot {
namespace diag {

volatile CpuStats g_cpu;

void RecordCycle(uint32_t ticks, uint32_t budget_ticks)
{
  g_cpu.cycle_count = g_cpu.cycle_count + 1u;
  g_cpu.last_cycle_ticks = ticks;

  if (ticks > g_cpu.cycle_ticks_max) {
    g_cpu.cycle_ticks_max = ticks;
  }

  /* 一阶低通平均：avg += (ticks - avg) / 16
   * 用移位避免浮点，也不会因为偶尔一次抖动把平均值拉得很离谱。 */
  g_cpu.cycle_ticks_avg =
      g_cpu.cycle_ticks_avg - (g_cpu.cycle_ticks_avg >> 4) + (ticks >> 4);

  if (ticks > budget_ticks) {
    g_cpu.overrun_count = g_cpu.overrun_count + 1u;
  }
}

}  // namespace diag
}  // namespace robot
