/**
 *******************************************************************************
 * @file      :stub_tick.cpp
 * @brief     : 电脑端测试用的 tick 实现（替代 src/tick.cpp）
 * @history   :
 *  Version     Date            Author          Note
 *  V1.0.0      2026-10-19      <author>        1. 初版
 *******************************************************************************
 * @attention :
 *  单片机上的 tick.cpp 依赖 SysTick（USE_HAL_TICK 分支）或 chrono
 *  （USE_UNIX_TICK 分支）。后者与 tick.hpp 的声明不一致（uint32_t vs
 *  uint64_t），而且我们不想在电脑上把 STM32 HAL 拖进来，所以测试工程
 *  自己提供一份等价实现，行为与真机一致（毫秒/微秒递增）。
 *******************************************************************************
 */
#include <chrono>

#include "tick.hpp"

namespace hello_world {
namespace tick {

namespace {
const auto kStart = std::chrono::steady_clock::now();

uint64_t ElapsedUs()
{
  return static_cast<uint64_t>(
      std::chrono::duration_cast<std::chrono::microseconds>(
          std::chrono::steady_clock::now() - kStart)
          .count());
}
}  // namespace

uint64_t GetTickUs(void) { return ElapsedUs(); }

uint32_t GetTickMs(void) { return static_cast<uint32_t>(ElapsedUs() / 1000u); }

uint32_t GetTickS(void) { return static_cast<uint32_t>(ElapsedUs() / 1000000u); }

void DelayUs(uint32_t us)
{
  const uint64_t until = ElapsedUs() + us;
  while (ElapsedUs() < until) {
  }
}

}  // namespace tick
}  // namespace hello_world
