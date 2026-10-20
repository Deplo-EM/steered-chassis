/**
 *******************************************************************************
 * @file      : fault_injection.cpp
 * @brief     : 台架故障注入开关的实现（变量定义）
 * @history   :
 *  Version     Date            Author          Note
 *  V1.0.0      2026-10-20      <author>        1. 初版
 *******************************************************************************
 *  Copyright (c) 2024 Hello World Team, Zhejiang University.
 *  All Rights Reserved.
 *******************************************************************************
 */
/* Includes ------------------------------------------------------------------*/
#include "fault_injection.hpp"

namespace robot {

/* 定义成非 static 全局变量：调试器里能按名字找到并直接改写。 */
volatile uint32_t g_fault_id = static_cast<uint32_t>(FaultId::kNone);

const bool g_fault_injection_enabled = kEnableFaultInjection;

}  // namespace robot
