/**
 *******************************************************************************
 * @file      : fault_injection.hpp
 * @brief     : 台架故障注入开关（默认关闭；只用于验收演示，绝不上车）
 * @history   :
 *  Version     Date            Author          Note
 *  V1.0.0      2026-10-20      <author>        1. 初版：配合 docs/08 故障注入清单
 *******************************************************************************
 * @attention :
 *  【为什么需要它】
 *  审计反复强调一句实话：静态审查 + 编译 + 主机测试，**不能**替代台架上的
 *  故障注入。看门狗到底会不会在 200ms 内复位、非法数据会不会让车动起来，
 *  只有真的"把故障做出来"才算验证过。
 *
 *  【怎么用（详见 docs/08）】
 *    1. 把 kEnableFaultInjection 改成 true，重新编译烧录；
 *    2. 上电后默认 g_fault_id = 0（不注入任何故障）；
 *    3. 在调试器的表达式/变量窗口里把 g_fault_id 改成对应编号，不用重烧；
 *    4. 观察文档里列出的变量，记录现象；
 *    5. 测完把 kEnableFaultInjection 改回 false 重新烧录。
 *
 *  【安全红线】
 *  - 只在轮子架空的台架上使用；
 *  - 带故障注入的固件**绝对不能**用于实车 / 验收演示；
 *  - 提交代码前 grep 一下 kEnableFaultInjection 必须是 false。
 *******************************************************************************
 *  Copyright (c) 2024 Hello World Team, Zhejiang University.
 *  All Rights Reserved.
 *******************************************************************************
 */
/* Define to prevent recursive inclusion -------------------------------------*/
#ifndef FAULT_INJECTION_HPP_
#define FAULT_INJECTION_HPP_

/* Includes ------------------------------------------------------------------*/
#include <cstdint>

namespace robot {

/* Exported constants --------------------------------------------------------*/

/**
 * @brief 故障注入总开关
 * @note  **默认必须是 false**。改成 true 之后，下面的 g_fault_id 才会生效。
 *        为 false 时，所有注入分支在优化构建里会被编译器完全删掉。
 */
constexpr bool kEnableFaultInjection = false;

/* Exported types ------------------------------------------------------------*/

/** 可注入的故障编号（调试器里把 g_fault_id 改成这里的值） */
enum class FaultId : uint32_t {
  kNone = 0,            ///< 不注入（默认）
  kHangControlLoop = 1, ///< 控制任务死循环：验证看门狗能否在 ~200ms 复位
  kInvalidDbusWheel = 2,///< 把 DBUS 拨轮改成 0xFC00：验证非法帧不会授权输出
  kImuFrozen = 3,       ///< 冻结 IMU 采样：验证"读数冻结"这一残留风险
  kCanTxStarve = 4,     ///< 占满 CAN 发送邮箱：验证发送失败可被计数
};

/* Exported variables --------------------------------------------------------*/

/** 当前注入的故障编号（volatile：调试器直接改写即可生效，无需重烧）
 *  @note 靠"volatile + 启动时写一次"保留在 ELF 里（详见 .cpp 的说明）。
 *        仅凭符号存在**不能**判断开关状态，判断方法见下。 */
extern volatile uint32_t g_fault_id;

/** 本固件是否带故障注入能力：`false` 表示安全固件。
 *  @note 运行期值，只能在调试器里读。
 *        想**离线**核对"这个 .elf 是安全版吗"，请用下面的 g_build_tag。 */
extern volatile bool g_fault_injection_enabled;

/**
 * @brief 构建标签：`"SAFE-BUILD"` 或 `"FAULT-INJECTION-BUILD"`
 * @note  它是 .rodata 里的字符串，可以直接在固件文件上查：
 *          arm-none-eabi-strings 固件.elf | findstr BUILD
 *        安全固件只应出现 `SAFE-BUILD`。
 *        这是"已烧回安全固件"的**离线证据**（不依赖调试器，也不依赖符号表）。
 */
extern const char* const g_build_tag;

/** 构建标签地址（启动时写入；用于保证标签字符串不被链接器回收，也方便调试器查看） */
extern const char* volatile g_build_tag_addr;

/* Exported function prototypes ----------------------------------------------*/

/**
 * @brief 判断某个故障是否处于激活状态
 * @param id 故障编号
 * @retval true 需要执行注入代码
 * @note  总开关关闭时恒为 false（编译器会把调用处的分支删掉）。
 */
inline bool FaultActive(FaultId id)
{
  return kEnableFaultInjection &&
         (g_fault_id == static_cast<uint32_t>(id));
}

}  // namespace robot

#endif /* FAULT_INJECTION_HPP_ */
