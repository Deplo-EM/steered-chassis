/** @file critical_section.hpp
 *  @brief 仅用于复制输入/反馈的短临界区；恢复进入前PRIMASK，允许嵌套。
 *  @note 禁止在此范围内执行SPI、PID、CAN发送、等待或打印。
 */
#ifndef ROBOT_CRITICAL_SECTION_HPP_
#define ROBOT_CRITICAL_SECTION_HPP_
#include <cstdint>
#include "stm32_hal.hpp"
namespace robot {
class CriticalSection {
 public:
  CriticalSection() : saved_(__get_PRIMASK()) { __disable_irq(); __DMB(); }
  ~CriticalSection() { __DMB(); __set_PRIMASK(saved_); }
  CriticalSection(const CriticalSection&) = delete;
  CriticalSection& operator=(const CriticalSection&) = delete;
 private:
  uint32_t saved_;
};
}
#endif
