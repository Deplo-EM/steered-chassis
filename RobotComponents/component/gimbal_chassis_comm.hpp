/**
 *******************************************************************************
 * @file      :gimbal_chassis_comm.hpp
 * @brief     : 云台板 ↔ 底盘板 CAN 通信协议
 * @history   :
 *  Version     Date            Author          Note
 *  V1.0.0      2026-10-19      <author>        1. 完成字段设计与编解码
 *******************************************************************************
 * @attention :
 *  【为什么需要这一层】
 *  两块板是两台独立的单片机，各自跑自己的固件，内存不共享、函数不能互调。
 *  它们之间唯一的联系就是 CAN 总线上的一帧帧数据。这一层要做三件事：
 *    1. 把「我要告诉对方什么」打包成 8 个字节（encode）；
 *    2. 把「对方告诉我的 8 个字节」还原成语义（decode）；
 *    3. 判断对方是不是还活着（isOffline）。
 *
 *  【帧格式（8 字节定长，收发严格对称）】
 *  ── 底盘 → 云台（C2G，CAN 标准帧 0x210）─────────────────────────────
 *   byte 0 : 标志位
 *            bit0 alive        本板已初始化并开始发帧
 *            bit1 working      本板状态机处于 Working
 *            bit2 estop        急停
 *            bit3 dbus_online  底盘选定遥控源在线（本地或完整转发）
 *            bit4~5 mode       底盘工作模式（0 分离 / 1 跟随 / 2 小陀螺）
 *            bit6 imu_ready    本板 IMU 零漂标定完成
 *            bit7 gimbal_enable 允许云台输出
 *   byte 1~2: yaw_rate_cmd  int16，单位 0.001 rad/s（云台 yaw 角速度设定）
 *   byte 3~4: pitch_target  int16，单位 0.0001 rad（云台 pitch 关节角目标）
 *   byte 5   : 保留（填 0，将来加字段用）
 *   byte 6   : 序号 seq，每帧 +1；拒绝重复/短窗口逆序，离线后可同步重启序号
 *   byte 7   : CRC8（对 byte0~6 计算，多项式 0x07，初值 0）
 *
 *  ── 云台 → 底盘（G2C，CAN 标准帧 0x211）─────────────────────────────
 *   byte 0 : 标志位
 *            bit0 alive        本板已初始化并开始发帧
 *            bit1 working      本板状态机处于 Working
 *            bit2 estop        仅云台本地遥控急停，不回显 C2G 急停
 *            bit3 dbus_online  云台板遥控器在线
 *            bit4 imu_ready    云台板 IMU 零漂标定完成
 *            bit5 pitch_at_limit pitch 已到软件限位
 *            bit6~7 保留
 *   byte 1~2: yaw_joint     int16，单位 0.0001 rad（yaw 关节实际角，即 δ）
 *   byte 3~4: pitch_joint   int16，单位 0.0001 rad（pitch 关节实际角）
 *   byte 5   : 保留（填 0）
 *   byte 6   : 序号 seq
 *   byte 7   : CRC8
 *
 *  【为什么 yaw 下发的是「角速度」而不是「角度目标」？】
 *  考核说明 3.1 提到 C2G 可以带「期望的云台 yaw/pitch 目标」。这里只对 pitch
 *  下发角度目标，yaw 下发角速度设定，原因是一个物理事实：
 *    - 云台 yaw 电机装在底盘和云台之间，它只能控制「云台相对底盘的夹角」。
 *    - 如果云台只做关节角闭环，那么底盘一转，枪口就跟着转（因为电机的定子
 *      跟着底盘走了），「跟随模式」里底盘无论怎么转，云台-底盘夹角都不变，
 *      永远收敛不了，还会一路自旋。
 *    - 所以 yaw 必须做「世界系稳定」：用云台自己的 IMU 把枪口钉在世界方向
 *      上。而世界角只能由持有 IMU 的一方自己积分，底盘板算不了云台的绝对
 *      朝向，因此底盘板下发「转多快」，云台板自己积分成目标角并闭环。
 *  pitch 不存在这个问题（俯仰不随车体偏航转动），所以直接下发关节角目标，
 *  精度也更高（0.0001 rad ≈ 0.006°）。
 *
 *  【CAN ID 选择】
 *  0x210 / 0x211 属于空闲区间，避开了：
 *    M3508 控制 0x200 / 0x1FF、反馈 0x201~0x208；
 *    GM6020 控制 0x1FF / 0x2FF（电压）、0x1FE / 0x2FE（电流）、反馈 0x205~0x20B；
 *    达妙 MIT 发送 0x01~0x0F、反馈 0x11~0x1F。
 *******************************************************************************
 *  Copyright (c) 2024 Hello World Team, Zhejiang University.
 *  All Rights Reserved.
 *******************************************************************************
 */
/* Define to prevent recursive inclusion -------------------------------------*/
#ifndef GIMBAL_CHASSIS_COMM_HPP_
#define GIMBAL_CHASSIS_COMM_HPP_

/* Includes ------------------------------------------------------------------*/
#include <cstdint>

#include "module_state.hpp"
#include "dbus_relay.hpp"
#include "tick.hpp"

/* Exported macro ------------------------------------------------------------*/
namespace robot {
/* Exported constants --------------------------------------------------------*/

/** 双板通信帧 ID（两板必须约定一致） */
constexpr uint32_t kCommIdChassisToGimbal = 0x210u;  ///< 底盘 → 云台
constexpr uint32_t kCommIdGimbalToChassis = 0x211u;  ///< 云台 → 底盘

/** 对方离线判定阈值（ms）：超过这个时间没收到新帧就算掉线。
 *  状态帧率是200Hz，100ms约为连续丢20帧，足够排除偶发丢帧。 */
constexpr uint32_t kCommOfflineThresholdMs = 100u;

/** 通信帧长度 */
constexpr uint8_t kCommFrameLen = 8u;

/* Exported types ------------------------------------------------------------*/

/* 底盘↔云台双板 CAN 通讯协议类。底盘板与云台板各实例化一个，用 code_part 区分方向。 */
class GimbalChassisComm {
 public:
  /* 标记本实例运行在哪块板，决定 encode / decode 的方向。 */
  enum class CodePart : uint8_t { Chassis = 0, Gimbal = 1 };

  /* 底盘→云台（C2G）帧数据：字段由新人按考核要求设计。 */
  struct ChassisData {
    /* 标志位 */
    bool alive = false;        ///< 本板已初始化并开始发帧
    bool working = false;      ///< 状态机处于 Working
    bool estop = false;        ///< 急停
    bool dbus_online = false;  ///< 底盘选定遥控源在线
    bool imu_ready = false;    ///< 本板 IMU 零漂标定完成
    bool gimbal_enable = false;///< 允许云台输出（急停 / 未就绪时为 false）

    ChassisMode mode = ChassisMode::kSeparate;  ///< 底盘工作模式

    float yaw_rate = 0.0f;      ///< 云台 yaw 角速度设定，单位 rad/s
    float pitch_target = 0.0f;  ///< 云台 pitch 关节角目标，单位 rad
  };

  /* 云台→底盘（G2C）帧数据：字段由新人按考核要求设计。 */
  struct GimbalData {
    bool alive = false;
    bool working = false;
    bool estop = false;
    bool dbus_online = false;
    bool imu_ready = false;
    bool pitch_at_limit = false;

    float yaw_joint = 0.0f;    ///< yaw 关节实际角（云台相对底盘的偏航角 δ），rad
    float pitch_joint = 0.0f;  ///< pitch 关节实际角，rad
  };

  GimbalChassisComm(CodePart code_part, uint32_t tx_id, uint32_t rx_id);

  /* 编码己方帧：把本地 ChassisData / GimbalData 打包进 8 字节。 */
  void encode(uint8_t tx_data[8]);
  /* 解码对方帧：把 8 字节还原成对方语义并更新本地缓存。
   * @retval true 校验通过、数据已更新；false CRC / 字段 / 序号不通过，整帧丢弃、不刷新在线时间。
   * @note  返回值很重要：调用方必须只在 true 时认为"对方活着"（喂狗、刷新时间戳），
   *        否则一帧干扰数据也能把看门狗喂住。 */
  bool decode(const uint8_t rx_data[8]);

  /* 判断对方是否离线（超过阈值时长未收到新帧）。 */
  bool isOffline() const;

  /* 通信质量计数（调试用；CRC 错误持续增长说明总线或协议有问题） */
  uint32_t rx_ok_count() const { return rx_ok_count_; }
  uint32_t rx_error_count() const { return rx_error_count_; }

  /* 本板发送帧的 CAN ID（Task 层发帧时用） */
  uint32_t tx_id() const { return tx_id_; }
  /* 对方发送帧的 CAN ID（CAN 接收分发时用来命中） */
  uint32_t rx_id() const { return rx_id_; }

  /* 数据访问（供 RobotModules / Task 读写）。 */
  ChassisData &chassis_data() { return chassis_data_; }
  GimbalData &gimbal_data() { return gimbal_data_; }
  const ChassisData &chassis_data() const { return chassis_data_; }
  const GimbalData &gimbal_data() const { return gimbal_data_; }
  DbusRelay& remote_relay() { return remote_relay_; }
  const DbusRelay& remote_relay() const { return remote_relay_; }

 private:
  CodePart code_part_;
  uint32_t tx_id_;
  uint32_t rx_id_;
  uint32_t last_rx_tick_;  ///< 上次收到对方帧的 GetTickMs() 时间戳
  bool received_ = false;  ///< 是否收到过至少一帧（没收到过时一律算离线）
  uint8_t rx_seq_ = 0;     ///< 最近成功接受的序号
  bool resync_pending_ = false;
  uint8_t resync_seq_ = 0;
  uint32_t resync_tick_ = 0;
  uint8_t tx_seq_ = 0;     ///< 发送序号
  uint32_t rx_ok_count_ = 0;     ///< 成功解码的帧数
  uint32_t rx_error_count_ = 0;  ///< CRC、字段或序号校验失败的帧数
  ChassisData chassis_data_;
  GimbalData gimbal_data_;
  DbusRelay remote_relay_;  ///< 云台本地DBUS向底盘转发，独立于状态心跳的新鲜度
};
/* Exported variables --------------------------------------------------------*/
/* Exported function prototypes ----------------------------------------------*/
}  // namespace robot

#endif /* GIMBAL_CHASSIS_COMM_HPP_ */
