/**
 *******************************************************************************
 * @file      : diagnostics.hpp
 * @brief     : 运行期诊断数据（周期耗时 / 超时次数），调试器里直接观察
 * @history   :
 *  Version     Date            Author          Note
 *  V1.0.0      2026-10-20      <author>        1. 初版：配合 docs/08 验收
 *******************************************************************************
 * @attention :
 *  【为什么需要】
 *  考核要求「1ms 控制周期」——但"设计上是 1ms"和"实际跑得完 1ms"是两回事。
 *  SPI 读 IMU、8 个电机的 PID、CAN 发送都挤在这 1ms 里，必须有一个客观数字
 *  来证明没有超时。这里用 TIM2（84MHz 自由计数）测量每个控制周期的实际耗时。
 *
 *  【怎么读】
 *  调试器里看 robot::diag::g_cpu：
 *    - cycle_ticks_max  历史最大耗时（单位 tick，84 tick = 1us）
 *    - cycle_ticks_avg  一阶平均耗时
 *    - overrun_count    超过预算（1ms）的周期数，正常应为 0
 *    - cycle_count      已经跑了多少个周期（也在侧面证明控制循环没停）
 *
 *  ⚠️ 测量口径：这是"墙钟时间"，包含被更高优先级中断（UART/CAN）打断的时间。
 *     对"这一周期能不能在 1ms 内干完"这个判断来说，这样偏保守，是合理的。
 *******************************************************************************
 *  Copyright (c) 2024 Hello World Team, Zhejiang University.
 *  All Rights Reserved.
 *******************************************************************************
 */
/* Define to prevent recursive inclusion -------------------------------------*/
#ifndef DIAGNOSTICS_HPP_
#define DIAGNOSTICS_HPP_

/* Includes ------------------------------------------------------------------*/
#include <cstdint>

namespace robot {
namespace diag {

/* Exported types ------------------------------------------------------------*/

/** 控制周期诊断数据 */
struct CpuStats {
  uint32_t cycle_count = 0;       ///< 累计控制周期数
  uint32_t last_cycle_ticks = 0;  ///< 最近一次周期耗时
  uint32_t cycle_ticks_max = 0;   ///< 历史最大耗时
  uint32_t cycle_ticks_avg = 0;   ///< 一阶平均耗时
  uint32_t overrun_count = 0;     ///< 超过预算的周期数
};

/* Exported variables --------------------------------------------------------*/

/** 全局诊断数据（中断里写，调试器里读；volatile 防止被优化掉） */
extern volatile CpuStats g_cpu;

/* Exported function prototypes ----------------------------------------------*/

/**
 * @brief 记录一个控制周期的耗时
 * @param ticks         本周期耗时（TIM2 tick，84 tick = 1us）
 * @param budget_ticks  预算（1ms = 84000 tick）
 */
void RecordCycle(uint32_t ticks, uint32_t budget_ticks);

/** tick → 微秒（TIM2 时钟 84MHz） */
inline uint32_t TicksToUs(uint32_t ticks) { return ticks / 84u; }

}  // namespace diag
}  // namespace robot

#endif /* DIAGNOSTICS_HPP_ */
