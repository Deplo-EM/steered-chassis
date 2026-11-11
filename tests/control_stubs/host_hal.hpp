#pragma once
#include <cstdint>
// motor_base/base 的硬件临界区替身；本测试为单线程。
inline uint32_t host_irq_mask = 0;
inline uint32_t __get_PRIMASK() { return host_irq_mask; }
inline void __disable_irq() { host_irq_mask=1; }
inline void __enable_irq() { host_irq_mask=0; }
inline void __set_PRIMASK(uint32_t value) { host_irq_mask=value; }
inline void __DMB() {}
