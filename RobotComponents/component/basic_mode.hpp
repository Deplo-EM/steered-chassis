/**
 * @file basic_mode.hpp
 * @brief 拒绝历史保护旁路；固件与主机测试共用同一套准入条件。
 */
#ifndef ROBOT_BASIC_MODE_HPP_
#define ROBOT_BASIC_MODE_HPP_

#if defined(ROBOT_BASIC_MODE) && ROBOT_BASIC_MODE
#error "ROBOT_BASIC_MODE may not bypass safety guards"
#endif

#endif  // ROBOT_BASIC_MODE_HPP_
