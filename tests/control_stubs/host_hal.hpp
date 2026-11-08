#pragma once
// motor_base/base 的硬件临界区替身；本测试为单线程。
inline void __disable_irq() {}
inline void __enable_irq() {}
