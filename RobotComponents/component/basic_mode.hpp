/** @file basic_mode.hpp
 * @brief 拒绝旧版绕过保护的构建选项；基础三模式与测试使用同一保护契约。
 */
#ifndef ROBOT_BASIC_MODE_HPP_
#define ROBOT_BASIC_MODE_HPP_
#if defined(ROBOT_BASIC_MODE) && ROBOT_BASIC_MODE != 0
#error "ROBOT_BASIC_MODE safety bypass is retired; remove the obsolete build definition"
#endif
#endif
