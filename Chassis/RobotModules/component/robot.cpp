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
constexpr float kAngularAccel = 25.0f;  ///< rad/s²

/* ---------------- 跟随模式增益 ---------------- */
/** 角度误差 → 目标角速度的比例（1/s）。Kp=4 表示误差 1rad 时目标转速 4rad/s，
 *  仅在理想角速度跟踪下约对应0.25s时间常数；实际阻尼和稳定性须实车验证。 */
constexpr float kFollowKp = 4.0f;
/** 用底盘陀螺仪角速度做阻尼：车已经在转了就少给一点，抑制过冲 */
constexpr float kFollowKd = 0.15f;

/* ---------------- 云台指令 ---------------- */
constexpr float kMaxGimbalYawRate = 6.0f;  ///< 云台 yaw 最大转速，rad/s
constexpr float kGimbalPitchRate = 2.0f;   ///< 云台 pitch 最大转速，rad/s
/** pitch 软件限位（保守值，实车按机械行程改）。到限位后不再往同方向推。 */
constexpr float kPitchLowerLimit = -0.9f;  ///< rad（约 −51°）
constexpr float kPitchUpperLimit = 0.7f;   ///< rad（约 +40°）
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

void robot::Robot::init()
{
  rc_checker_.init(kRemoteOfflineMs);
  for (int i = 0; i < kMotorSlotNum; ++i) {
    motor_checkers_[i].init(kMotorOfflineMs);
  }

  state_ = LifeState::kDead;
  state_enter_tick_ = hello_world::tick::GetTickMs();
  mode_ = ChassisMode::kSeparate;
  estop_ = false;
  can_tx_guard_ = CanTxGuard{};
  remote_armed_ = false;
  ramped_.reset();
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

  /* ---- 1. 更新传感器（IMU 必须每个周期都读，标定期间也要读） ----
   * kImuFrozen跳过采样；上一有效样本超过20ms后isSampleValid撤销输出资格。
   * 设备时钟/DRDY冻结与SPI错误由真实采样路径另行监督。 */
  if (!robot::FaultActive(robot::FaultId::kImuFrozen)) {
    imu_->update();
  }

  /* ---- 2. 读遥控器状态 ---- */
  updateInput();

  /* ---- 3. 状态迁移 ---- */
  updateStateMachine();

  /* ---- 4. 生成指令并下发（急停在这条链路的最外层生效） ---- */
  applyOutput();

  /* ---- 5. 把本板状态写进通信帧，CommTask 会把它发出去 ---- */
  updateCommData();
}

void robot::Robot::notifyRemoteRx()
{
  rc_checker_.update();
}

void robot::Robot::notifyCanTxCycle(bool success)
{
  if (!initialized_) return;
  const bool reset_ack = remote_input_.online &&
      remote_input_.right == hello_world::remote_control::kSwitchStateDown;
  can_tx_guard_.update(success, reset_ack, hello_world::tick::GetTickMs());
  if (!can_tx_guard_.healthy()) {
    enterState(LifeState::kDead);
    chassis_->stop();
    remote_armed_ = false;
    ramped_.reset();
    gimbal_yaw_rate_ = 0.0f;
    updateCommData();
  }
}

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

float robot::Robot::delta() const
{
  if (comm_ == nullptr) {
    return 0.0f;
  }
  return robot::WrapPi(peer_input_.yaw_joint);
}

/* Private function definitions ----------------------------------------------*/

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
  if ((selected == RemoteSource::kLocal && !local.online) ||
      (selected == RemoteSource::kGimbal && !forwarded.online)) selected = RemoteSource::kNone;
  if (selected == RemoteSource::kNone) {
    // 新来源必须先给出有效急停档。另一接收机突然上线不能抢走运动指令。
    if (local_estop_) selected = RemoteSource::kLocal;
    else if (forwarded.online && forwarded.right == hello_world::remote_control::kSwitchStateDown)
      selected = RemoteSource::kGimbal;
  }
  if (selected != remote_source_) {
    remote_armed_ = false; ramped_.reset(); gimbal_yaw_rate_ = 0.0f;
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
  /* 云台板那边如果判定急停（它自己也有遥控器），也一并对本板生效 */
  const bool peer_estop =
      peer_online_ ? peer_input_.estop : false;
  return local_estop_ || local_estop || peer_estop;
}

void robot::Robot::updateStateMachine()
{
  const bool remote_ok = remote_input_.online;
#if ROBOT_BASIC_MODE
  /* ★ 基本功能模式 ★ 运行期只要求零漂标定完成；并且不等云台先 Working。 */
  const bool imu_ok = (imu_ != nullptr) && imu_->isCalibrated();
  const bool motors_ok = motors_online_ && chassis_->feedbackHealthy();
  const bool peer_ok = true;
#else
  const bool imu_ok =
      (imu_ != nullptr) && imu_->isCalibrated() && imu_->isHealthy() && imu_->isSampleValid();
  const bool motors_ok = motors_online_ && chassis_->feedbackHealthy();
  const bool peer_ok = peer_online_ && peer_input_.working;
#endif
  const bool all_ok = remote_ok && remote_armed_ && imu_ok && motors_ok && peer_ok && can_tx_guard_.healthy();

  const uint32_t now = hello_world::tick::GetTickMs();

  switch (state_) {
    case LifeState::kDead:
      if (all_ok) {
        enterState(LifeState::kResurrection);
      }
      break;

    case LifeState::kResurrection:
      if (!all_ok) {
        enterState(LifeState::kDead);
      } else if ((now - state_enter_tick_) >= kResurrectionMs) {
        enterState(LifeState::kWorking);
      }
      break;

    case LifeState::kWorking:
      if (!all_ok) {
        /* 任何一项安全条件丢失：立刻退回 Dead，输出归零 */
        enterState(LifeState::kDead);
      }
      break;

    default:
      enterState(LifeState::kDead);
      break;
  }
}

void robot::Robot::enterState(LifeState next)
{
  if (next == state_) {
    return;
  }
  state_ = next;
  state_enter_tick_ = hello_world::tick::GetTickMs();
  // 对板单独复位也会使本板退回 Dead；撤销旧授权，防止对板恢复后自动带杆起动。
  if (next == LifeState::kDead) remote_armed_ = false;

  if (next == LifeState::kWorking) {
    initWorkingTargets();
  } else {
    /* 离开 Working：把指令清干净，避免下次进入时「攒着的目标」突然释放 */
    ramped_.reset();
    gimbal_yaw_rate_ = 0.0f;
    chassis_->stop();
  }
}

void robot::Robot::initWorkingTargets()
{
  ramped_.reset();

  /* pitch 目标从「当前实际角度」接着走：如果直接给 0，进入 Working 的瞬间
   * 云台会猛地甩回中位。这叫「无扰切换」。 */
  gimbal_pitch_target_ = (comm_ != nullptr) ? peer_input_.pitch_joint
                                            : 0.0f;
  gimbal_pitch_target_ =
      robot::Clamp(gimbal_pitch_target_, kPitchLowerLimit, kPitchUpperLimit);
  gimbal_yaw_rate_ = 0.0f;

  /* 清掉上一状态残留的 PID 积分与逆解历史 */
  chassis_->stop();
}

void robot::Robot::generateCommand()
{
  const bool remote_ok = remote_input_.online;

  /* ---------------- 平移：摇杆 → 云台坐标系速度 ----------------
   * UnitClamp：摇杆理论上在 [-1,1]，但上游一旦给出越界值（协议改动、
   * 解码器与校验器位宽不一致等），直接乘上限会得到离谱的目标速度。
   * 限幅要放在「用之前」，不能只依赖后面的加速度斜坡（斜坡只限变化率）。 */
  ChassisMotion raw;
  if (remote_ok) {
    /* 左摇杆向上（+）→ 沿云台 x 轴前进；
     * 左摇杆向右（+）→ 云台系 y 轴是「左」，所以取负。 */
    raw.vx = UnitClamp(remote_input_.lv) * kMaxLinearSpeed;
    raw.vy = -UnitClamp(remote_input_.lh) * kMaxLinearSpeed;
  }

  /* ---------------- 旋转：按模式给 w ---------------- */
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
      float w = kFollowKp * delta_rad - kFollowKd * imu_->gyroZ();
      raw.w = robot::Clamp(w, -kMaxChassisW, kMaxChassisW);
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
    ramped_.reset();
    gimbal_yaw_rate_ = 0.0f;
    return;
  }

  /* ---------------- 加速度限幅 ---------------- */
  // 平移按向量限加速度：保持斜推方向，合加速度也不超过 8 m/s²。
  const float dx = raw.vx - ramped_.vx, dy = raw.vy - ramped_.vy;
  const float distance = std::hypot(dx, dy);
  const float scale = distance > kLinearAccel * kDt ? kLinearAccel * kDt / distance : 1.0f;
  ramped_.vx += dx * scale;
  ramped_.vy += dy * scale;
  ramped_.w = Slew(ramped_.w, raw.w, kAngularAccel * kDt);

  /* ---------------- 云台指令 ---------------- */
  updateGimbalCommand();
}

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

  /* 右摇杆向上（+）→ 枪口抬高。这里给的是「关节角目标」，由云台板做位置环。 */
  gimbal_pitch_target_ +=
      UnitClamp(remote_input_.rv) * kPitchStickSign * kGimbalPitchRate * kDt;
  gimbal_pitch_target_ =
      robot::Clamp(gimbal_pitch_target_, kPitchLowerLimit, kPitchUpperLimit);
}

void robot::Robot::applyOutput()
{
  /* ★ 急停闸门：放在最外层 ★
   * 放在状态判断之前，是因为「恢复自检阶段舵角回中」也是一次真实动作——
   * 如果只在 Working 分支里判急停，按着急停上电时舵轮仍会自己转回中位。
   * 现在的顺序保证：只要急停生效，不管什么状态都一步不动。 */
  if (estop_) {
    chassis_->stop();
    ramped_.reset();
    gimbal_yaw_rate_ = 0.0f;
    // 急停期间重新同步 pitch，解除时不释放急停前残留目标。
    gimbal_pitch_target_ = peer_input_.pitch_joint;
    return;
  }

  if (state_ != LifeState::kWorking) {
    /* 审计 H3 修复：考核红线是「生命状态未进入 Working 时，所有电机必须
     * 立刻输出归零」。所以这里一律 stop()，不再做「恢复自检阶段舵轮回中」。
     * 回中虽然好看，但它是「无人下达的运动指令」，按红线判会不通过。
     * 需要这个功能的话，必须在考核方书面确认后，再做成「受遥控器确认、
     * 限速、且急停可打断」的流程。 */
    chassis_->stop();
    return;
  }

  generateCommand();

  /* 把云台系速度指令 + 当前 δ 交给底盘模块：它内部先旋转到车体系再逆解 */
  chassis_->setMotion(ramped_, delta());
  chassis_->update(kDt);
}

void robot::Robot::updateCommData()
{
  if (comm_ == nullptr) {
    return;
  }
  GimbalChassisComm::ChassisData& d = comm_->chassis_data();
  d.working = (state_ == LifeState::kWorking);
  d.estop = estop_;
  d.dbus_online = remote_input_.online;
  d.imu_ready = (imu_ != nullptr) && imu_->isCalibrated() && imu_->isHealthy() && imu_->isSampleValid();
  d.gimbal_enable = d.working && !estop_;
  d.mode = mode_;
  d.yaw_rate = d.gimbal_enable ? gimbal_yaw_rate_ : 0.0f;
  d.pitch_target = gimbal_pitch_target_;
}
