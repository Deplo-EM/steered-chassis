/**
 * @file basic_mode.hpp
 * @brief 基本功能模式开关（考核保底）。
 *
 * 固件构建定义 ROBOT_BASIC_MODE=1：底盘不再等云台 Working、IMU 只要完成零漂标定、
 * CAN 发送失败不再锁存/停喂。目的：先让车能完成分离/跟随/小陀螺的基本动作。
 * 主机测试不定义该宏（默认 0），继续验证严格安全契约。
 *
 * 代价（答辩时如实说明）：CAN 发送失败不再停机、IMU 运行期异常不再一票否决。
 * 保留：急停最外层、遥控离线归零、未进 Working 归零、看门狗"控制卡死复位"。
 */
#ifndef ROBOT_BASIC_MODE_HPP_
#define ROBOT_BASIC_MODE_HPP_

#ifndef ROBOT_BASIC_MODE
#define ROBOT_BASIC_MODE 0
#endif

#endif  // ROBOT_BASIC_MODE_HPP_
