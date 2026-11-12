/**
 * @file basic_mode.hpp
 * @brief 基本功能模式开关（考核前保底用）。
 *
 * 固件构建在 CMakeLists 里定义 ROBOT_BASIC_MODE=1：去掉若干"一票否决"门槛
 * （运行期 IMU 健康、对板先 Working、CAN 失败锁存/停喂），保证车能先动起来。
 * 主机测试不定义该宏（默认 0），继续验证严格的原始安全契约。
 *
 * 代价：CAN 发送失败不再停机、IMU 运行期异常不再一票否决。
 * 急停、遥控离线归零、未进 Working 归零、看门狗"控制卡死复位"仍然保留。
 */
#ifndef ROBOT_BASIC_MODE_HPP_
#define ROBOT_BASIC_MODE_HPP_

#ifndef ROBOT_BASIC_MODE
#define ROBOT_BASIC_MODE 0
#endif

#endif  // ROBOT_BASIC_MODE_HPP_
