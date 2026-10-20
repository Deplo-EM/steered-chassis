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
#if defined(__GNUC__)
/** used：让编译器即使发现"没人引用"也把符号放进目标文件。
 *  @note 只靠它**不够**——真正保住符号的是「volatile 变量 + 启动时写一次」。
 *        （曾尝试 __attribute__((retain)) 想让链接器 --gc-sections 也保留，
 *         但本工具链下该属性被忽略，编译会报 'retain' attribute ignored，
 *         所以已移除，见复核意见。） */
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
 * 最终方案 = volatile 变量 + 在 MainTaskInit() 里做一次 volatile 写。
 * volatile 访问是编译器不能消除的副作用，符号必然保留；同时启动时把
 * g_fault_id 归零，也顺便保证"上电默认不注入"。 */
HW_KEEP_SYMBOL volatile uint32_t g_fault_id =
    static_cast<uint32_t>(FaultId::kNone);

/** 本固件是否带故障注入能力（启动时由 kEnableFaultInjection 写入）。
 *  volatile 是为了让它不被优化/回收；运行期不会有人改它。 */
HW_KEEP_SYMBOL volatile bool g_fault_injection_enabled =
    kEnableFaultInjection;

/* ---------------------------------------------------------------------------
 * 构建标签：**离线**区分两种固件的证据
 *
 * 为什么需要它：`nm` 只能证明符号存在——带注入的固件也有同样的两个符号，
 * 所以 nm **不能**用来判断开关是 true 还是 false（复核意见）。
 * 而 g_fault_injection_enabled 的值是启动时写进 .bss 的，ELF 文件里也读不到。
 *
 * 于是把两种构建编成两段不同的字符串放进 .rodata：
 *     SAFE-BUILD                ← 安全固件
 *     FAULT-INJECTION-BUILD     ← 带注入的固件
 * 刷机前后都可以用 strings 直接在 .elf 上核对，不用连板子。
 * ------------------------------------------------------------------------- */
HW_KEEP_SYMBOL const char* const g_build_tag =
    kEnableFaultInjection ? "FAULT-INJECTION-BUILD" : "SAFE-BUILD";

/** 构建标签的地址（启动时写入）。
 *  作用有两个：① 让上面的字符串真的被引用，不会被链接器回收；
 *             ② 调试器里能直接看到它指向哪段文字。 */
HW_KEEP_SYMBOL const char* volatile g_build_tag_addr = nullptr;

}  // namespace robot
