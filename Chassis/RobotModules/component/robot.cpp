/**
 *******************************************************************************
 * @file      :robot.cpp
 * @brief     : 底盘板顶层状态机实现（模式逻辑 + 安全保护）
 * @history   :
 *  Version     Date            Author          Note
 *  V1.0.0      2026-10-19      <author>        1. 完成编写
 *******************************************************************************
 * @attention :
 *  拨杆定义（第一次上车前请先确认手感，可以在 tests 或调试器里看 mode_）：
 *    左拨杆  上 = 分离模式   中 = 跟随模式   下 = 小陀螺模式
 *    右拨杆  下 = 急停（所有电机立即归零）   上/中 = 允许输出
 *
 *  摇杆定义：
 *    左摇杆  上下 = 前进/后退（云台坐标系）   左右 = 左移/右移
 *    右摇杆  上下 = 云台 pitch                左右 = 云台 yaw 转速
 *    拨轮    分离模式下 = 底盘自转角速度      小陀螺模式下 = 自旋快慢
 *******************************************************************************
 *  Copyright (c) 2024 Hello World Team, Zhejiang University.
 *  All Rights Reserved.
 *******************************************************************************
 */
/* Includes ------------------------------------------------------------------*/
#include "robot.hpp"

#include <cmath>
#include <cstring>

#include "DT7.hpp"
#include "chassis.hpp"
#include "fault_injection.hpp"
#include "gimbal_chassis_comm.hpp"
#include "imu.hpp"
#include "tick.hpp"
#include "critical_section.hpp"
#include "basic_mode.hpp"
#include "chassis_debug.hpp"
#include "gimbal_limits.hpp"
/* Private macro -------------------------------------------------------------*/
/* Private constants ---------------------------------------------------------*/

namespace {

/** 控制周期，单位 s（TIM6 配置为 1kHz） */
constexpr float kDt = 0.001f;

/* ---------------- 速度上限（安全相关，先给保守值） ---------------- */
constexpr float kMaxLinearSpeed = 2.0f;  ///< 平移速度上限，m/s
constexpr float kMaxChassisW = 6.0f;     ///< 底盘自转角速度上限，rad/s
constexpr float kMaxSpinW = 6.0f;        ///< 小陀螺最大自旋，rad/s（≈1 转/秒）
constexpr float kMinSpinW = 1.5f;        ///< 小陀螺最小自旋，rad/s（保证「非零旋转分量」）

/** 所有模式里允许出现的最大 |w|（模式间取大者），用于最后一道兜底限幅 */
constexpr float kMaxAbsW =
    (kMaxChassisW > kMaxSpinW) ? kMaxChassisW : kMaxSpinW;

/**
 * @brief 把摇杆值钳到 [-1, 1]
 * @note  摇杆的正常范围就是 [-1,1]。这里再钳一次是防御性的：
 *        万一解码/校验环节出问题（例如校验器与解码器位宽不一致，
 *        见审计 F1），也不会让越界值直接乘上速度上限变成失控指令。
 */
inline float UnitClamp(float value)
{
  if (!std::isfinite(value)) return 0.0f;
  return (value > 1.0f) ? 1.0f : ((value < -1.0f) ? -1.0f : value);
}

/* ---------------- 加速度限幅（让起步/停车平顺，保护机械） ---------------- */
constexpr float kLinearAccel = 8.0f;    ///< m/s²
constexpr float kAngularAccel = 80.0f;  ///< rad/s²

/* ---------------- 跟随模式增益 ---------------- */
/** 角度误差到目标角速度的比例，单位 1/s；实际命令还会经过模式限幅和角加速度斜坡。 */
constexpr float kFollowKp = 8.0f;
/** 阻尼项系数；本车 gyroZ 正方向与旋转命令相反，在 generateCommand 中用加号。 */
constexpr float kFollowKd = 1.00f;

/* ---------------- 云台指令 ---------------- */
constexpr float kMaxGimbalYawRate = 6.0f;  ///< 云台 yaw 最大转速，rad/s
constexpr float kGimbalPitchRate = 2.0f;   ///< 云台 pitch 最大转速，rad/s
/** pitch 软件限位（保守值，实车按机械行程改）。到限位后不再往同方向推。 */
constexpr float kPitchLowerLimit = robot::gimbal_limits::kPitchLowerRad;
constexpr float kPitchUpperLimit = robot::gimbal_limits::kPitchUpperRad;
/** 右摇杆推上去时枪口应该抬高；若实车相反，把这里改成 −1 */
constexpr float kPitchStickSign = 1.0f;

/* ---------------- 状态机 / 在线检测 ---------------- */
constexpr uint32_t kRemoteOfflineMs = 100;   ///< 遥控器掉线阈值
constexpr uint32_t kMotorOfflineMs = 100;    ///< 电机掉线阈值
constexpr uint32_t kResurrectionMs = 500;    ///< 恢复自检持续时长

/** 把 value 以每周期最多 max_delta 的速度拉向 target */
inline float Slew(float value, float target, float max_delta)
{
  const float diff = target - value;
  if (diff > max_delta) {
    return value + max_delta;
  }
  if (diff < -max_delta) {
    return value - max_delta;
  }
  return target;
}

}  // namespace

/* Private variables ---------------------------------------------------------*/
/* Private function prototypes -----------------------------------------------*/
/* Exported function definitions ---------------------------------------------*/

/**
 * @brief 初始化在线检测、生命状态和遥控授权，准备首次自检。
 * @note  对象由 Instance 层长期持有，此函数在 MainTaskInit 中调用一次。
 */
void robot::Robot::init()
{
  rc_checker_.init(kRemoteOfflineMs);
  for (int i = 0; i < kMotorSlotNum; ++i) {
    motor_checkers_[i].init(kMotorOfflineMs);
  }

  state_ = LifeState::kDead;
  state_enter_tick_ = hello_world::tick::GetTickMs();
  last_dead_ = {};
  last_imu_dead_ = {};
  imu_control_state_ = ImuControlState::kUnavailable;
  mode_ = ChassisMode::kSeparate;
  estop_ = false;
  can_tx_guard_ = CanTxGuard{};
  remote_armed_ = false;
#if defined(ROBOT_GROUND_TRIAL) && ROBOT_GROUND_TRIAL
  ground_gate_.invalidate();
#endif
  resetMotion();
  gimbal_yaw_rate_ = 0.0f;
  gimbal_pitch_target_ = 0.0f;
  initialized_ = (rc_ != nullptr) && (imu_ != nullptr) && (chassis_ != nullptr) &&
                 (comm_ != nullptr);
}

void robot::Robot::update()
{
  if (!initialized_) {
    return;
  }

  /* ---- 1. 更新传感器 ----
   * IMU 必须每个周期都读：零漂标定需要连续样本，标定期间也不能停。
   * 上一有效样本超过 20 ms 后 isSampleValid 会撤销输出资格（由状态机使用）。
   * kImuFrozen 只是故障注入开关，正常构建不会触发。 */
  if (!robot::FaultActive(robot::FaultId::kImuFrozen)) {
    imu_->update();
  }

  /* ---- 2. 读遥控器状态 ----
   * 解码 DBUS、合并板间帧里的遥控分片、判断在线/急停，最终写进 remote_input_。 */
  updateInput();

  /* ---- 3. 状态迁移 ----
   * 根据通信/IMU/电机/对板/CAN 监督是否正常，在 Dead/Resurrection/Working 之间切换。 */
  updateStateMachine();

  /* ---- 4. 生成指令并下发 ----
   * 急停闸门放在 applyOutput 的最外层：只要急停有效，不管什么状态都一步不动。 */
  applyOutput();

  /* ---- 4.5 调试快照：状态/授权（运动量由 Chassis::update 填） ---- */
  {
    auto& d = diag::g_chassis_dbg;
    d.state = static_cast<uint8_t>(state_);
    d.estop = estop_ ? 1u : 0u;
    d.authorized = outputPermitted() ? 1u : 0u;
    d.gyro_z_radps = imu_->gyroZ();
  }

  /* ---- 5. 把本板状态写进通信帧，CommTask 会把它发出去 ---- */
  updateCommData();
}

/**
 * @brief 记录一帧合法本地 DBUS 的到达时刻，供后续在线检查。
 */
void robot::Robot::notifyRemoteRx()
{
  rc_checker_.update();
}

/**
 * @brief 接收本周期通信结果，发送故障发生时同步撤销底盘与对板授权。
 * @note  通信任务可能在编码前报告故障，因此这里立即清理仍可被编码的运动目标。
 */
void robot::Robot::notifyCanTxCycle(bool success)
{
  if (!initialized_) return;
  const bool reset_ack = remote_input_.online &&
      remote_input_.right == hello_world::remote_control::kSwitchStateDown;
  const uint32_t now = hello_world::tick::GetTickMs();
  can_tx_guard_.update(success, reset_ack, now);
  if (!can_tx_guard_.healthy()) {
    enterState(LifeState::kDead, now, kDeadCanTx);
    chassis_->stop();
    diag::g_chassis_dbg.authorized = 0u;
    remote_armed_ = false;
    resetMotion();
    gimbal_yaw_rate_ = 0.0f;
    updateCommData();
  }
}

/**
 * @brief 刷新对应电机的反馈时刻，槽位 0～3 为驱动、4～7 为舵向。
 */
void robot::Robot::notifyMotorRx(int slot)
{
  if (slot >= 0 && slot < kMotorSlotNum) {
    motor_checkers_[slot].update();
  }
}

bool robot::Robot::isRemoteOnline()
{
  CriticalSection lock;
  return remote_input_.online;
}

bool robot::Robot::areAllMotorsOnline()
{
  CriticalSection lock;
  for (int i = 0; i < kMotorSlotNum; ++i) {
    if (motor_checkers_[i].isOffline()) {
      return false;
    }
  }
  return true;
}

/**
 * @brief 给通信层判断本拍是否可编码非零输出，停机边沿也以此值检测。
 */
bool robot::Robot::outputPermitted() const
{
  if (state_ != LifeState::kWorking || estop_ || chassis_ == nullptr ||
      chassis_->steerKeepoutBlockedMask() != 0u) return false;
#if defined(ROBOT_GROUND_TRIAL) && ROBOT_GROUND_TRIAL
  return ground::kMotionEnabled && ground_gate_.permitsOutput();
#else
  return true;
#endif
}

/**
 * @brief 取得云台相对车头的关节偏角，归一化到一个半周范围。
 * @note  坐标旋转使用此角，不能用云台的世界 yaw 代替。
 */
float robot::Robot::delta() const
{
  if (comm_ == nullptr) {
    return 0.0f;
  }
  return robot::WrapPi(peer_input_.yaw_joint);
}

/* Private function definitions ----------------------------------------------*/

/**
 * @brief 生成本拍一致的遥控、对板及电机反馈快照，再选择控制来源。
 * @note  中断保护区内只复制数据，DT7 解码和死区浮点运算放在保护区外。
 */
void robot::Robot::updateInput()
{
  RemoteInput local, forwarded;
  uint8_t forwarded_frame[18] = {};
  bool forwarded_online = false;
  {
    CriticalSection lock;
    local.capture(rc_, !rc_checker_.isOffline());
    peer_input_ = comm_->gimbal_data();
    peer_online_ = !comm_->isOffline();
    const auto& relay = comm_->remote_relay();
    // 状态帧和完整遥控分片都要新鲜，避免仅有在线心跳却继续使用旧摇杆。
    forwarded_online = peer_online_ && peer_input_.dbus_online &&
        relay.online(hello_world::tick::GetTickMs());
    if (forwarded_online) std::memcpy(forwarded_frame, relay.frame(), 18);
    motors_online_ = areAllMotorsOnline();
    chassis_->captureFeedback();
  }
  // 继续使用同一个DT7解码器，避免跨板链路另造一套通道位宽/比例。
  hello_world::remote_control::DT7 decoded;
  if (forwarded_online) decoded.decode(forwarded_frame);
  forwarded.capture(&decoded, forwarded_online);
  local.normalize(); forwarded.normalize();
  local_estop_ = local.online && local.right == hello_world::remote_control::kSwitchStateDown;

  RemoteSource selected = remote_source_;
#if defined(ROBOT_GROUND_TRIAL) && ROBOT_GROUND_TRIAL
  // 修正拓扑中的 DR16 位于云台 UART3；首跑只从该来源取得运动许可。
  if (!forwarded.online) selected = RemoteSource::kNone;
  else if (forwarded.right == hello_world::remote_control::kSwitchStateDown)
    selected = RemoteSource::kGimbal;
#else
  if ((selected == RemoteSource::kLocal && !local.online) ||
      (selected == RemoteSource::kGimbal && !forwarded.online)) selected = RemoteSource::kNone;
  if (selected == RemoteSource::kNone) {
    // 新来源必须先给出有效急停档。另一接收机突然上线不能抢走运动指令。
    if (local_estop_) selected = RemoteSource::kLocal;
    else if (forwarded.online && forwarded.right == hello_world::remote_control::kSwitchStateDown)
      selected = RemoteSource::kGimbal;
  }
#endif
  // 输入来源变化后，旧来源的急停确认和运动斜坡全部作废，再等待新来源确认。
  if (selected != remote_source_) {
    remote_armed_ = false; resetMotion(); gimbal_yaw_rate_ = 0.0f;
#if defined(ROBOT_GROUND_TRIAL) && ROBOT_GROUND_TRIAL
    ground_gate_.invalidate();
#endif
  }
  remote_source_ = selected;
  remote_input_ = selected == RemoteSource::kLocal ? local :
      (selected == RemoteSource::kGimbal ? forwarded : RemoteInput{});
  mode_ = readModeSwitch();
  estop_ = readEstopSwitch();
  if (!remote_input_.online) remote_armed_ = false;
  else if (remote_input_.right == hello_world::remote_control::kSwitchStateDown)
    remote_armed_ = true;
}

/**
 * @brief 把左拨杆编码转换成三种底盘模式，非法或缺省情况选择分离。
 */
robot::ChassisMode robot::Robot::readModeSwitch() const
{
  if (rc_ == nullptr) {
    return ChassisMode::kSeparate;
  }
  switch (remote_input_.left) {
    case hello_world::remote_control::kSwitchStateUp:
      return ChassisMode::kSeparate;
    case hello_world::remote_control::kSwitchStateMid:
      return ChassisMode::kFollow;
    case hello_world::remote_control::kSwitchStateDown:
      return ChassisMode::kSpin;
    default:
      return ChassisMode::kSeparate;
  }
}

bool robot::Robot::readEstopSwitch() const
{
  if (rc_ == nullptr) {
    return true;  /* 没有遥控器 = 必须停机（失效即归零） */
  }
  /* 右拨杆打到最下面 = 急停 */
  const bool local_estop =
      (remote_input_.right == hello_world::remote_control::kSwitchStateDown);
  /* 在线对板的急停状态也参与本板判断，任一有效急停源都可撤销出力。 */
  const bool peer_estop =
      peer_online_ ? peer_input_.estop : false;
  return local_estop_ || local_estop || peer_estop;
}

/**
 * @brief 汇总本拍运行条件，推进 Dead、自检、Working，恢复过程要求条件连续成立。
 * @note  生命状态表示就绪程度，操作者急停由 applyOutput 外层另行关断。
 */
void robot::Robot::updateStateMachine()
{
  const bool remote_ok = remote_input_.online;
  const uint32_t now = hello_world::tick::GetTickMs();
#if defined(ROBOT_GROUND_TRIAL) && ROBOT_GROUND_TRIAL
  const bool allow_imu_hold = false;
#else
  const bool allow_imu_hold = remote_armed_ && outputPermitted();
#endif
  imu_control_state_ = EvaluateImuForControl(imu_, now, allow_imu_hold);
  const bool imu_ready = imu_control_state_ != ImuControlState::kUnavailable;
#if (defined(ROBOT_GROUND_TRIAL) && ROBOT_GROUND_TRIAL) || \
    (defined(ROBOT_STEER_MODE_PROBE) && ROBOT_STEER_MODE_PROBE)
  const bool imu_ok = imu_ready;
#else
  // 首次进入 Working 和故障恢复仍须通过 IMU 自检。运行中只有跟随的阻尼项
  // 消费本板陀螺；分离和小陀螺用的是新鲜的云台 Yaw 关节角。
  const bool imu_ok = imu_ready || (state_ == LifeState::kWorking && mode_ != ChassisMode::kFollow);
#endif
  const bool motors_ok = motors_online_ && chassis_->feedbackHealthy();
  bool peer_ok = peer_online_ && peer_input_.working;
#if defined(ROBOT_GROUND_TRIAL) && ROBOT_GROUND_TRIAL
  peer_ok = peer_ok && peer_input_.alive && peer_input_.imu_ready && peer_input_.ground_disarmed;
#endif
  if (remote_ok && remote_input_.right == hello_world::remote_control::kSwitchStateDown &&
      imu_ok && motors_ok && peer_ok && can_tx_guard_.healthy()) {
    chassis_->acknowledgeSteerStall();
  }
  const uint8_t causes = deadCauses(imu_ok, motors_ok, peer_ok);
  const bool all_ok = causes == kDeadNone;

  switch (state_) {
    case LifeState::kDead:
      if (all_ok) {
        enterState(LifeState::kResurrection, now);
      }
      break;

    case LifeState::kResurrection:
      if (!all_ok) {
        enterState(LifeState::kDead, now, causes);
      } else if ((now - state_enter_tick_) >= kResurrectionMs) {
        enterState(LifeState::kWorking, now);
      }
      break;

    case LifeState::kWorking:
      if (!all_ok) {
        /* 任何一项安全条件丢失：立刻退回 Dead，输出归零 */
        enterState(LifeState::kDead, now, causes);
      }
      break;

    default:
      enterState(LifeState::kDead, now, kDeadInvalidState);
      break;
  }
}

/**
 * @brief 将同时失效的条件编码为原因位图，供状态迁移和历史诊断共用。
 */
uint8_t robot::Robot::deadCauses(bool imu_ok, bool motors_ok, bool peer_ok) const
{
  uint8_t causes = kDeadNone;
  if (!remote_input_.online) causes |= kDeadRemote;
  if (!remote_armed_) causes |= kDeadNotArmed;
  if (!imu_ok) causes |= kDeadImu;
  if (!motors_ok) causes |= kDeadMotor;
  if (!peer_ok) causes |= kDeadPeer;
  if (!can_tx_guard_.healthy()) causes |= kDeadCanTx;
  if (chassis_->steerStallMask() != 0) causes |= kDeadSteerStall;
  return causes;
}

/**
 * @brief 使用已过连续死区的平移通道判断回中，供分离模式保存减速参考方向。
 */
bool robot::Robot::leftCentered() const
{
  return remote_input_.online && remote_input_.lv == 0.0f && remote_input_.lh == 0.0f;
}

/**
 * @brief 清空速度斜坡及其坐标参考，防止旧速度在再次授权后释放。
 */
void robot::Robot::resetMotion()
{
  ramped_.reset();
  steering_ref_yaw_ = 0.0f;
  steering_ref_held_ = false;
}

/**
 * @brief 统一执行状态迁移动作，先保存故障现场，再清目标和出力授权。
 * @note  同态调用直接返回，保留首次进入该状态的时间戳。
 */
void robot::Robot::enterState(LifeState next, uint32_t now, uint8_t dead_causes)
{
  if (next == state_) {
    return;
  }
  state_ = next;
  state_enter_tick_ = now;
  // 先记录，再撤销授权和目标；事后看实时字段可能已经恢复或被清零。
  if (next == LifeState::kDead) {
    last_dead_ = {state_enter_tick_, chassis_->lastSteerStall().sequence,
                  can_tx_guard_.failedCycles(), ramped_.vx, ramped_.vy, ramped_.w,
                  dead_causes, mode_, leftCentered(), remote_source_, remote_input_, {}, {}};
    if (dead_causes & kDeadMotor) last_dead_.steer_angle = chassis_->steerAngleFault();
    if ((dead_causes & kDeadImu) && imu_ != nullptr) {
      last_dead_.imu = imu_->faultDetail(now);
      last_imu_dead_ = last_dead_;
    }
    remote_armed_ = false;
  }

  if (next == LifeState::kWorking) {
    initWorkingTargets();
  } else {
    /* 离开 Working：把指令清干净，避免下次进入时「攒着的目标」突然释放 */
    resetMotion();
    gimbal_yaw_rate_ = 0.0f;
    chassis_->stop();
#if defined(ROBOT_GROUND_TRIAL) && ROBOT_GROUND_TRIAL
    ground_gate_.invalidate();
#endif
  }
}

/**
 * @brief 恢复时以当前俯仰关节角接续目标，再安排自动抬平。
 */
void robot::Robot::initWorkingTargets()
{
  resetMotion();

  /* 用实测角接续以避免目标突跳；回到水平的速度由 updateGimbalCommand 控制。 */
  gimbal_pitch_target_ = (comm_ != nullptr) ? peer_input_.pitch_joint
                                            : 0.0f;
  pitch_level_pending_ = true;
  gimbal_pitch_target_ =
      robot::Clamp(gimbal_pitch_target_, kPitchLowerLimit, kPitchUpperLimit);
  gimbal_yaw_rate_ = 0.0f;

  /* 清掉上一状态残留的 PID 积分与逆解历史 */
  chassis_->stop();
}

void robot::Robot::generateCommand()
{
  const bool remote_ok = remote_input_.online;

  /* 连上遥控的瞬间也安排一次抬平（不直接跳 0，避免阶跃）；
   * 急停解除边沿是另一条触发路径。 */
  if (remote_ok && !remote_was_online_) {
    gimbal_pitch_target_ = (comm_ != nullptr) ? peer_input_.pitch_joint : 0.0f;
    pitch_level_pending_ = true;
  }
  remote_was_online_ = remote_ok;

  /* ---------------- 平移：摇杆 → 云台坐标系速度 ----------------
   * UnitClamp：摇杆理论上在 [-1,1]，但上游一旦给出越界值（协议改动、
   * 解码器与校验器位宽不一致等），直接乘上限会得到离谱的目标速度。
   * 限幅要放在「用之前」，不能只依赖后面的加速度斜坡（斜坡只限变化率）。 */
  ChassisMotion raw;
  if (remote_ok) {
    /* 实车方向：代码 +x 与实车车头相反，整体旋转 180°（vx、vy 同时取反）。 */
    raw.vx = -UnitClamp(remote_input_.lv) * kMaxLinearSpeed;
    raw.vy = UnitClamp(remote_input_.lh) * kMaxLinearSpeed;
  }

  /* ---------------- 旋转：按模式给 w ---------------- */
  auto& dbg = diag::g_chassis_dbg;
  dbg.mode = static_cast<uint8_t>(mode_);
  switch (mode_) {
    case ChassisMode::kSeparate:
      /* 分离：底盘不自旋，w 直接由拨轮给（操作手想转多少就转多少） */
      raw.w = remote_ok ? (UnitClamp(remote_input_.wheel) * kMaxChassisW) : 0.0f;
      break;

    case ChassisMode::kFollow: {
      /* 跟随：让车头对准云台。
       * δ = 云台朝向 − 车头朝向，δ>0 说明云台在车头左边，车应该向左转（w>0）。
       * 再加一项陀螺仪阻尼，避免转过头。 */
      const float delta_rad = delta();
      /* 2026-11-16 实车：-Kd*gyro 是正反馈（Kd=2 时单向跑飞），
       * 说明该陀螺 z 轴符号与 w 约定相反；改为 +Kd*gyro 才是阻尼。 */
      float w = kFollowKp * delta_rad + kFollowKd * imu_->gyroZ();
      // 靠近对正且底盘已近静止时，不用编码器量化和陀螺零偏反复拽动四个舵。
      if (std::fabs(delta_rad) < 0.02f && std::fabs(imu_->gyroZ()) < 0.05f) w = 0.0f;
      dbg.delta_rad = delta_rad;
      dbg.follow_w_cmd = w;
      dbg.follow_gyro_z = imu_->gyroZ();
      /* 切换瞬间 delta 可能很大，直接给满会让车猛冲过头；限制跟随段的最大角速度。 */
      constexpr float kFollowMaxW = 8.0f;
      raw.w = robot::Clamp(w, -kFollowMaxW, kFollowMaxW);
      break;
    }

    case ChassisMode::kSpin: {
      /* 小陀螺：必须有非零旋转分量。
       * 拨轮大小决定转速，正负决定转向；拨轮回中时用最小转速继续转。 */
      const float wheel = remote_ok ? UnitClamp(remote_input_.wheel) : 0.0f;
      const float magnitude =
          kMinSpinW + std::fabs(wheel) * (kMaxSpinW - kMinSpinW);
      const float direction = (wheel < -0.05f) ? -1.0f : 1.0f;
      raw.w = direction * magnitude;
      break;
    }

    default:
      raw.reset();
      break;
  }

  /* ---------------- 兜底限幅 ----------------
   * 不管走哪个分支，目标都不能超过声明上限。这是一道独立的防线：
   * 即使上面的某个分支将来被改错，也不会把整车指令放大到失控。 */
  raw.vx = robot::Clamp(raw.vx, -kMaxLinearSpeed, kMaxLinearSpeed);
  raw.vy = robot::Clamp(raw.vy, -kMaxLinearSpeed, kMaxLinearSpeed);
  raw.w = robot::Clamp(raw.w, -kMaxAbsW, kMaxAbsW);
  const float speed = std::hypot(raw.vx, raw.vy);
  if (speed > kMaxLinearSpeed) {
    raw.vx *= kMaxLinearSpeed / speed;
    raw.vy *= kMaxLinearSpeed / speed;
  }
  // 数值异常不进入斜坡；清除整组目标，避免只停一轴后仍有意外运动。
  if (!std::isfinite(raw.vx) || !std::isfinite(raw.vy) || !std::isfinite(raw.w)) {
    resetMotion();
    gimbal_yaw_rate_ = 0.0f;
    return;
  }

  /* ---------------- 加速度限幅 ---------------- */
  // 分离档松开平移杆，只沿上一拍的车体系方向减速；这段残余速度不再跟着云台转。
  // 有平移输入时仍用实时云台角。w 单独处理，拨轮或其他模式的旋转不受影响。
  const float yaw = delta();
  const bool has_translation = ramped_.vx != 0.0f || ramped_.vy != 0.0f;
  const bool hold_ref = mode_ == ChassisMode::kSeparate && leftCentered() && has_translation;
  if (!hold_ref) {
    if (steering_ref_held_ && has_translation) {
      // 松杆尚未停稳又推杆：把剩余速度换到新参考系，再继续限加速度。
      // 只换坐标，避免把原有车体系速度突然转向。
      const float rebase = WrapPi(steering_ref_yaw_ - yaw);
      const float c = std::cos(rebase), s = std::sin(rebase);
      const float vx = ramped_.vx, vy = ramped_.vy;
      ramped_.vx = c * vx - s * vy;
      ramped_.vy = s * vx + c * vy;
    }
    steering_ref_yaw_ = yaw;
  }
  steering_ref_held_ = hold_ref;
  // 三档共用平移斜坡。小陀螺有轮速余量时不另降平移。
  const float dx = raw.vx - ramped_.vx, dy = raw.vy - ramped_.vy;
  const float distance = std::hypot(dx, dy);
  if (distance > kLinearAccel * kDt) {
    const float scale = kLinearAccel * kDt / distance;
    ramped_.vx += dx * scale;
    ramped_.vy += dy * scale;
  } else {
    ramped_.vx = raw.vx;
    ramped_.vy = raw.vy;
  }
  ramped_.w = Slew(ramped_.w, raw.w, kAngularAccel * kDt);
  dbg.follow_w_cmd = raw.w;
  dbg.ramped_w = ramped_.w;

  /* ---------------- 云台指令 ---------------- */
  updateGimbalCommand();
}

/**
 * @brief 把右杆转换成 yaw 角速度和 pitch 关节角目标。
 * @note  yaw 的积分由持有世界姿态的云台板完成，pitch 在本板按周期积分并限行程。
 */
void robot::Robot::updateGimbalCommand()
{
  if (!remote_input_.online) {
    gimbal_yaw_rate_ = 0.0f;
    return;
  }

  /* 右摇杆向右（+）→ 云台顺时针转 → yaw 角减小（逆时针为正） */
  gimbal_yaw_rate_ = -UnitClamp(remote_input_.rh) * kMaxGimbalYawRate;
  gimbal_yaw_rate_ =
      robot::Clamp(gimbal_yaw_rate_, -kMaxGimbalYawRate, kMaxGimbalYawRate);

  constexpr float kPitchStickDeadband = 0.06f;
  const float rv = UnitClamp(remote_input_.rv);
  const float rv_eff = (std::fabs(rv) < kPitchStickDeadband) ? 0.0f : rv;
  // 人的明确指令可接管自动抬平，仍沿同一角度、速率和力矩限制运行。
  if (rv_eff != 0.0f) pitch_level_pending_ = false;
  if (pitch_level_pending_) {
    constexpr float kLevelRate = 0.5f;  // rad/s（放慢，减少抬平时的力矩冲击）
    const float step = kLevelRate * kDt;
    if (std::fabs(gimbal_pitch_target_) <= step) {
      gimbal_pitch_target_ = 0.0f;
      pitch_level_pending_ = false;
    } else {
      gimbal_pitch_target_ -= (gimbal_pitch_target_ > 0.0f ? step : -step);
    }
  }

  /* 右摇杆向上（+）抬高 Pitch，回中保持。 */
  if (!pitch_level_pending_) {
    gimbal_pitch_target_ += rv_eff * kPitchStickSign * kGimbalPitchRate * kDt;
  }
  gimbal_pitch_target_ =
      robot::Clamp(gimbal_pitch_target_, kPitchLowerLimit, kPitchUpperLimit);
}

void robot::Robot::applyOutput()
{
#if defined(ROBOT_GROUND_TRIAL) && ROBOT_GROUND_TRIAL
  applyGroundOutput();
#else
#if defined(ROBOT_STEER_MODE_PROBE) && ROBOT_STEER_MODE_PROBE
  // 诊断版不运行常规三模式输出。只有通信层的独立单电机试探可下发。
  chassis_->stop();
  resetMotion();
  return;
#endif
  /* 每次急停解除（1→0）都重新开始抬平。
   * 必须放在急停 return 之前：急停期间 generateCommand 不会被调用，
   * 边沿检测放里面会永远看不到 estop_ 变 true。 */
  if (!estop_ && estop_prev_) {
    gimbal_pitch_target_ = peer_input_.pitch_joint;
    gimbal_pitch_target_ =
        robot::Clamp(gimbal_pitch_target_, kPitchLowerLimit, kPitchUpperLimit);
    pitch_level_pending_ = true;
  }
  estop_prev_ = estop_;

  /* ★ 急停闸门：放在最外层 ★
   * 放在状态判断之前，是因为「恢复自检阶段舵角回中」也是一次真实动作——
   * 如果只在 Working 分支里判急停，按着急停上电时舵轮仍会自己转回中位。
   * 现在的顺序保证：只要急停生效，不管什么状态都一步不动。 */
  if (estop_) {
    chassis_->stop();
    resetMotion();
    gimbal_yaw_rate_ = 0.0f;
    // 急停期间重新同步 pitch，解除时不释放急停前残留目标。
    gimbal_pitch_target_ = peer_input_.pitch_joint;
    return;
  }

  if (state_ != LifeState::kWorking) {
    /* 自检期间保持零输入，等进入 Working 后才处理操作者的运动目标。 */
    chassis_->stop();
    return;
  }

  generateCommand();

  /* 逆解使用当前参考角；仅分离档松杆后剩余的平移速度使用冻结参考角。 */
  chassis_->setMotion(ramped_, steering_ref_yaw_, mode_ == ChassisMode::kSpin);
  chassis_->update(kDt);
  if (chassis_->steerStallMask() != 0) {
    enterState(LifeState::kDead, hello_world::tick::GetTickMs(), kDeadSteerStall);
  }
#endif
}

#if defined(ROBOT_GROUND_TRIAL) && ROBOT_GROUND_TRIAL
void robot::Robot::applyGroundOutput()
{
  const uint32_t now = hello_world::tick::GetTickMs();
  ground_gate_.update(remote_input_, state_ == LifeState::kWorking, estop_, now);
  // 仍须按新输入重新解算，上一拍无路不应永久阻止本拍选择可达方向。
  if (!ground::kMotionEnabled || state_ != LifeState::kWorking || estop_ ||
      !ground_gate_.permitsOutput()) {
    resetMotion();
    chassis_->stop();
    return;
  }

  const float target = ground_gate_.velocityTarget(remote_input_);
  ramped_.vx = Slew(ramped_.vx, target, ground::kAccelMps2 * kDt);
  if (ground_gate_.stopping() && ramped_.vx == 0.0f) {
    ground_gate_.finishStop();
    chassis_->stop();
    return;
  }

  // 首跑以车头为前；失能云台的偏航不参与方向转换。
  chassis_->setMotion(ramped_, 0.0f);
  chassis_->update(kDt);
  if (chassis_->steerStallMask() != 0) enterState(LifeState::kDead, now, kDeadSteerStall);
}
#endif

/**
 * @brief 发布状态机结果及云台目标，通信任务稍后统一编码发送。
 * @note  停止授权时立即把 yaw 角速度置零，pitch 仍保留用于恢复对齐的目标。
 */
void robot::Robot::updateCommData()
{
  if (comm_ == nullptr) {
    return;
  }
  GimbalChassisComm::ChassisData& d = comm_->chassis_data();
  d.working = (state_ == LifeState::kWorking);
  d.estop = estop_;
  d.dbus_online = remote_input_.online;
  d.imu_ready = imu_control_state_ != ImuControlState::kUnavailable;
  d.gimbal_enable = d.working && !estop_;
#if defined(ROBOT_GROUND_TRIAL) && ROBOT_GROUND_TRIAL
  d.gimbal_enable = false;
#endif
#if defined(ROBOT_STEER_MODE_PROBE) && ROBOT_STEER_MODE_PROBE
  // 即使底盘解除本地急停开始短测，也持续通知云台保持停机。
  d.estop = true;
  d.gimbal_enable = false;
#endif
  d.mode = mode_;
  d.yaw_rate = d.gimbal_enable ? gimbal_yaw_rate_ : 0.0f;
  d.pitch_target = gimbal_pitch_target_;
#if defined(ROBOT_GROUND_TRIAL) && ROBOT_GROUND_TRIAL
  d.pitch_target = 0.0f;
#endif
}


