/**
 *******************************************************************************
 * @file      : fault_injection.cpp
 * @brief     : 台架故障注入开关的实现（变量定义）
 * @history   :
 *  Version     Date            Author          Note
 *  V1.0.0      2026-10-20      <author>        1. 初版
 *  V1.1.0      2026-10-20      <author>        2. 复核修复：两个变量强制保留在 ELF 里，
 *                                                否则关闭注入时被优化掉，"安全固件"无法核实
 *******************************************************************************
 *  Copyright (c) 2024 Hello World Team, Zhejiang University.
 *  All Rights Reserved.
 *******************************************************************************
 */
/* Includes ------------------------------------------------------------------*/
#include "fault_injection.hpp"

/* Private macro -------------------------------------------------------------*/
#if defined(__GNUC__) && (__GNUC__ >= 11)
/** retain：在 ELF 段上打 SHF_GNU_RETAIN 标记，让**链接器**的 --gc-sections 也保留它。
 *  （只用 used 只能让编译器保留；本工程的链接参数带 -Wl,--gc-sections，
 *    没有 retain 时符号仍会被丢掉——复核意见 1 的根因。） */
#define HW_KEEP_SYMBOL __attribute__((used, retain))
#elif defined(__GNUC__)
#define HW_KEEP_SYMBOL __attribute__((used))
#else
#define HW_KEEP_SYMBOL
#endif

namespace robot {

/* 定义成非 static 全局变量：调试器里能按名字找到并直接改写。
 *
 * 为什么这两个变量必须能被保住：
 * 总开关为 false 时，FaultActive() 全是 `false && ...`，编译器会把对 g_fault_id
 * 的引用全部删掉；再叠加链接器的 --gc-sections，符号就整个消失，
 * "文档让你去调试器里看，但变量根本不存在"——复核意见 1 说的正是这个。
 *
 * 实测：单靠 __attribute__((used)) 不够（只管编译器），
 *       __attribute__((retain)) 在本工程的链接脚本下也没留住。
 * 因此最终方案 = volatile 变量 + 在 MainTaskInit() 里做一次 volatile 写。
 * volatile 访问是编译器不能消除的副作用，符号必然保留；同时启动时把
 * g_fault_id 归零，也顺便保证"上电默认不注入"。 */
HW_KEEP_SYMBOL volatile uint32_t g_fault_id =
    static_cast<uint32_t>(FaultId::kNone);

/** 本固件是否带故障注入能力（启动时由 kEnableFaultInjection 写入）。
 *  volatile 是为了让它不被优化/回收；运行期不会有人改它。 */
HW_KEEP_SYMBOL volatile bool g_fault_injection_enabled =
    kEnableFaultInjection;

}  // namespace robot
