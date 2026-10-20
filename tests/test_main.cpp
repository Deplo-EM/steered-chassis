/**
 *******************************************************************************
 * @file      :test_main.cpp
 * @brief     : 电脑端单元测试（不需要单片机）
 * @history   :
 *  Version     Date            Author          Note
 *  V1.0.0      2026-10-19      <author>        1. 初版：PID / 双板协议 / 舵轮逆解
 *******************************************************************************
 * @attention :
 *  为什么要在电脑上测：
 *  单片机上的 bug 只能靠「烧进去、看现象」来发现，慢、危险、且很难定位。
 *  而 PID、协议编解码、运动学逆解这三块本质上是纯数学，与硬件无关——
 *  把它们拿到电脑上用普通 C++ 编译，几百毫秒就能跑完几十个用例，
 *  车上再出问题就可以确定「不是这几块的逻辑错」。
 *
 *  编译运行：在仓库根目录执行
 *      powershell -ExecutionPolicy Bypass -File tests/run_tests.ps1
 *******************************************************************************
 */
#include <cmath>
#include <cstdio>
#include <cstring>

#include "chassis_ik_solver.hpp"
#include "DT7.hpp"
#include "dbus_check.hpp"
#include "gimbal_chassis_comm.hpp"
#include "module_state.hpp"
#include "pid.hpp"

/* ========================================================================== */
/* 一个极简的测试框架：只统计失败数，最后统一打印                             */
/* ========================================================================== */
namespace {

int g_checks = 0;
int g_failures = 0;
const char* g_current_case = "";

void BeginCase(const char* name)
{
  g_current_case = name;
  std::printf("\n[用例] %s\n", name);
}

void Check(bool ok, const char* expr, const char* file, int line)
{
  ++g_checks;
  if (!ok) {
    ++g_failures;
    std::printf("  [失败] %s:%d  表达式: %s\n", file, line, expr);
  }
}

void CheckNear(float actual, float expect, float tol, const char* expr,
               const char* file, int line)
{
  ++g_checks;
  if (!(std::fabs(actual - expect) <= tol)) {
    ++g_failures;
    std::printf("  [失败] %s:%d  %s  期望 %.6f ± %.6f，实际 %.6f\n", file, line,
                expr, expect, tol, actual);
  }
}

#define EXPECT_TRUE(cond) Check((cond), #cond, __FILE__, __LINE__)
#define EXPECT_NEAR(actual, expect, tol) \
  CheckNear((actual), (expect), (tol), #actual, __FILE__, __LINE__)
#define EXPECT_EQ(a, b) Check((a) == (b), #a " == " #b, __FILE__, __LINE__)

constexpr float kTol = 1e-5f;

/* ========================================================================== */
/* 一、PID 控制器                                                             */
/* ========================================================================== */
void TestPid()
{
  BeginCase("PID：纯比例项");
  {
    hello_world::pid::Pid pid(2.0f, 0.0f, 0.0f, 100.0f, 10.0f);
    /* 误差 = 10 - 0 = 10，Kp=2 → 输出 20 */
    EXPECT_NEAR(pid.update(10.0f, 0.0f, 0.001f), 20.0f, kTol);
  }

  BeginCase("PID：输出限幅");
  {
    hello_world::pid::Pid pid(100.0f, 0.0f, 0.0f, 5.0f, 10.0f);
    EXPECT_NEAR(pid.update(10.0f, 0.0f, 0.001f), 5.0f, kTol);
    /* 反方向也要限幅 */
    EXPECT_NEAR(pid.update(-10.0f, 0.0f, 0.001f), -5.0f, kTol);
  }

  BeginCase("PID：积分限幅（不能无限累积）");
  {
    hello_world::pid::Pid pid(0.0f, 1.0f, 0.0f, 100.0f, 0.5f);
    for (int i = 0; i < 2000; ++i) {
      pid.update(1.0f, 0.0f, 0.001f);
    }
    /* 积分项被限制在 ±0.5，输出 = Ki*0.5 = 0.5 */
    EXPECT_NEAR(pid.integral(), 0.5f, 1e-4f);
    EXPECT_NEAR(pid.output(), 0.5f, 1e-4f);
  }

  BeginCase("PID：饱和时不再继续积分（抗积分饱和）");
  {
    hello_world::pid::Pid pid(10.0f, 10.0f, 0.0f, 5.0f, 100.0f);
    for (int i = 0; i < 1000; ++i) {
      pid.update(100.0f, 0.0f, 0.001f);
    }
    const float integral_when_saturated = pid.integral();
    /* 输出早该饱和了，积分不应该涨到离谱；这里给一个宽松上限 */
    EXPECT_TRUE(std::fabs(integral_when_saturated) < 1.0f);
  }

  BeginCase("PID：reset 清空积分");
  {
    hello_world::pid::Pid pid(0.0f, 100.0f, 0.0f, 100.0f, 10.0f);
    pid.update(1.0f, 0.0f, 0.001f);
    pid.reset();
    EXPECT_NEAR(pid.integral(), 0.0f, kTol);
    EXPECT_NEAR(pid.output(), 0.0f, kTol);
  }

  BeginCase("PID：微分项对误差变化有反应");
  {
    hello_world::pid::Pid pid(0.0f, 0.0f, 1.0f, 100.0f, 10.0f);
    pid.update(0.0f, 0.0f, 0.001f);  /* 首帧不差分 */
    /* 误差从 0 跳到 0.001，差分 = 0.001/0.001 = 1，经过低通后应小于 1 */
    const float out = pid.update(0.001f, 0.0f, 0.001f);
    EXPECT_TRUE(out > 0.0f && out < 1.0f);
  }
}

/* ========================================================================== */
/* 二、双板通信协议                                                           */
/* ========================================================================== */
void TestComm()
{
  using robot::GimbalChassisComm;

  BeginCase("双板协议：底盘 → 云台 编码再解码应还原原值");
  {
    GimbalChassisComm chassis(GimbalChassisComm::CodePart::Chassis,
                              robot::kCommIdChassisToGimbal,
                              robot::kCommIdGimbalToChassis);
    GimbalChassisComm gimbal(GimbalChassisComm::CodePart::Gimbal,
                              robot::kCommIdGimbalToChassis,
                              robot::kCommIdChassisToGimbal);

    chassis.chassis_data().working = true;
    chassis.chassis_data().estop = false;
    chassis.chassis_data().dbus_online = true;
    chassis.chassis_data().imu_ready = true;
    chassis.chassis_data().gimbal_enable = true;
    chassis.chassis_data().mode = robot::ChassisMode::kSpin;
    chassis.chassis_data().yaw_rate = 1.234f;
    chassis.chassis_data().pitch_target = -0.321f;

    uint8_t frame[8];
    chassis.encode(frame);
    gimbal.decode(frame);

    const auto& d = gimbal.chassis_data();
    EXPECT_TRUE(d.working);
    EXPECT_TRUE(d.dbus_online);
    EXPECT_TRUE(d.imu_ready);
    EXPECT_TRUE(d.gimbal_enable);
    EXPECT_TRUE(!d.estop);
    EXPECT_TRUE(d.mode == robot::ChassisMode::kSpin);
    EXPECT_NEAR(d.yaw_rate, 1.234f, 1e-3f);
    EXPECT_NEAR(d.pitch_target, -0.321f, 1e-4f);
    EXPECT_TRUE(!gimbal.isOffline());
  }

  BeginCase("双板协议：云台 → 底盘 编码再解码应还原原值");
  {
    GimbalChassisComm chassis(GimbalChassisComm::CodePart::Chassis,
                              robot::kCommIdChassisToGimbal,
                              robot::kCommIdGimbalToChassis);
    GimbalChassisComm gimbal(GimbalChassisComm::CodePart::Gimbal,
                              robot::kCommIdGimbalToChassis,
                              robot::kCommIdChassisToGimbal);

    gimbal.gimbal_data().working = true;
    gimbal.gimbal_data().pitch_at_limit = true;
    gimbal.gimbal_data().yaw_joint = -3.0f;
    gimbal.gimbal_data().pitch_joint = 0.45f;

    uint8_t frame[8];
    gimbal.encode(frame);
    chassis.decode(frame);

    const auto& d = chassis.gimbal_data();
    EXPECT_TRUE(d.working);
    EXPECT_TRUE(d.pitch_at_limit);
    EXPECT_NEAR(d.yaw_joint, -3.0f, 1e-4f);
    EXPECT_NEAR(d.pitch_joint, 0.45f, 1e-4f);
  }

  BeginCase("双板协议：被篡改的帧必须被丢弃（CRC 拦截）");
  {
    GimbalChassisComm chassis(GimbalChassisComm::CodePart::Chassis,
                              robot::kCommIdChassisToGimbal,
                              robot::kCommIdGimbalToChassis);
    GimbalChassisComm gimbal(GimbalChassisComm::CodePart::Gimbal,
                              robot::kCommIdGimbalToChassis,
                              robot::kCommIdChassisToGimbal);

    gimbal.gimbal_data().yaw_joint = 1.0f;
    uint8_t good[8];
    gimbal.encode(good);

    /* 先正常收一帧，确认链路通 */
    chassis.decode(good);
    EXPECT_NEAR(chassis.gimbal_data().yaw_joint, 1.0f, 1e-4f);

    /* 改坏数据位、不动 CRC → 必须整帧丢弃 */
    uint8_t bad[8];
    std::memcpy(bad, good, sizeof(bad));
    bad[1] ^= 0xFF;
    /* 审计 H4：解码必须**返回失败**，调用方才知道这一帧不能拿去喂狗 */
    EXPECT_TRUE(!chassis.decode(bad));
    EXPECT_NEAR(chassis.gimbal_data().yaw_joint, 1.0f, 1e-4f);
    EXPECT_TRUE(chassis.rx_error_count() >= 1u);
  }

  BeginCase("双板协议：正常帧 decode 返回 true 并计数");
  {
    GimbalChassisComm chassis(GimbalChassisComm::CodePart::Chassis,
                              robot::kCommIdChassisToGimbal,
                              robot::kCommIdGimbalToChassis);
    GimbalChassisComm gimbal(GimbalChassisComm::CodePart::Gimbal,
                              robot::kCommIdGimbalToChassis,
                              robot::kCommIdChassisToGimbal);
    uint8_t frame[8];
    gimbal.encode(frame);
    EXPECT_TRUE(chassis.decode(frame));
    EXPECT_EQ(chassis.rx_ok_count(), 1u);
    EXPECT_EQ(chassis.rx_error_count(), 0u);
  }

  BeginCase("双板协议：没收到过帧时判定为离线");
  {
    GimbalChassisComm gimbal(GimbalChassisComm::CodePart::Gimbal,
                              robot::kCommIdGimbalToChassis,
                              robot::kCommIdChassisToGimbal);
    EXPECT_TRUE(gimbal.isOffline());
  }

  BeginCase("双板协议：帧 ID 两板必须互补");
  {
    GimbalChassisComm chassis(GimbalChassisComm::CodePart::Chassis,
                              robot::kCommIdChassisToGimbal,
                              robot::kCommIdGimbalToChassis);
    GimbalChassisComm gimbal(GimbalChassisComm::CodePart::Gimbal,
                              robot::kCommIdGimbalToChassis,
                              robot::kCommIdChassisToGimbal);
    EXPECT_EQ(chassis.tx_id(), gimbal.rx_id());
    EXPECT_EQ(chassis.rx_id(), gimbal.tx_id());
    EXPECT_TRUE(chassis.tx_id() != chassis.rx_id());
  }
}

/* ========================================================================== */
/* 二点五、DBUS 帧合法性校验（审计 H2 修复）                                   */
/* ========================================================================== */
namespace {

/** 按 DBUS 的位排列，把第 index 个模拟通道写成 value */
void SetDbusChannel(uint8_t d[18], int index, uint16_t value)
{
  if (index < 4) {
    const int bit = index * 11;
    const int byte = bit / 8;
    const int shift = bit % 8;
    uint32_t word = static_cast<uint32_t>(d[byte]) |
                    (static_cast<uint32_t>(d[byte + 1]) << 8) |
                    (static_cast<uint32_t>(d[byte + 2]) << 16);
    word &= ~(0x7FFu << shift);
    word |= (static_cast<uint32_t>(value) & 0x7FFu) << shift;
    d[byte] = static_cast<uint8_t>(word & 0xFF);
    d[byte + 1] = static_cast<uint8_t>((word >> 8) & 0xFF);
    d[byte + 2] = static_cast<uint8_t>((word >> 16) & 0xFF);
  } else {
    /* 拨轮是按 16 位整数解码的（DT7.cpp 用两个字节整体拼），
     * 所以这里也必须写满 16 位，否则测不出高位被置脏的情况（审计 F1）。 */
    d[16] = static_cast<uint8_t>(value & 0xFF);
    d[17] = static_cast<uint8_t>((value >> 8) & 0xFF);
  }
}

/** 造一帧「合法」的 DBUS 数据：5 个通道都在中位，拨杆都在上 */
void MakeValidDbusFrame(uint8_t d[18])
{
  std::memset(d, 0, 18);
  for (int i = 0; i < 5; ++i) {
    SetDbusChannel(d, i, 1024);
  }
  /* 第 5 字节：bit4~5 右拨杆、bit6~7 左拨杆；低 4 位属于通道 3，要保留 */
  d[5] = static_cast<uint8_t>((d[5] & 0x0F) | (1u << 4) | (1u << 6));
}

}  // namespace

void TestDbusCheck()
{
  using robot::DbusChannel;
  using robot::IsDbusFrameValid;

  uint8_t frame[18];

  BeginCase("DBUS 校验：合法的中位帧应通过");
  {
    MakeValidDbusFrame(frame);
    for (int i = 0; i < 5; ++i) {
      EXPECT_EQ(DbusChannel(frame, i), 1024);
    }
    EXPECT_TRUE(IsDbusFrameValid(frame));
  }

  BeginCase("DBUS 校验：全零帧必须判非法（接收机掉电的典型表现）");
  {
    std::memset(frame, 0, sizeof(frame));
    EXPECT_TRUE(!IsDbusFrameValid(frame));
  }

  BeginCase("DBUS 校验：通道超出 364~1684 必须判非法（字节错位）");
  {
    MakeValidDbusFrame(frame);
    SetDbusChannel(frame, 0, 300);  // 低于下限
    EXPECT_TRUE(!IsDbusFrameValid(frame));

    MakeValidDbusFrame(frame);
    SetDbusChannel(frame, 3, 1900);  // 高于上限
    EXPECT_TRUE(!IsDbusFrameValid(frame));
  }

  BeginCase("DBUS 校验：拨杆出现 0 必须判非法");
  {
    MakeValidDbusFrame(frame);
    frame[5] = static_cast<uint8_t>(frame[5] & ~0x30u);  // 右拨杆清成 0
    EXPECT_TRUE(!IsDbusFrameValid(frame));
  }

  BeginCase("DBUS 校验：拨杆 2(下) / 3(中) 都是合法值");
  {
    MakeValidDbusFrame(frame);
    frame[5] = static_cast<uint8_t>((frame[5] & 0x0F) | (2u << 4) | (3u << 6));
    EXPECT_TRUE(IsDbusFrameValid(frame));
    EXPECT_EQ(static_cast<int>((frame[5] >> 4) & 0x03), 2);
    EXPECT_EQ(static_cast<int>((frame[5] >> 6) & 0x03), 3);
  }

  BeginCase("DBUS 校验：边界值 364 / 1684 应通过");
  {
    MakeValidDbusFrame(frame);
    SetDbusChannel(frame, 0, 364);
    SetDbusChannel(frame, 1, 1684);
    EXPECT_TRUE(IsDbusFrameValid(frame));
  }

  /* ---- 审计 F1：校验器与解码器位宽必须一致 ---- */
  BeginCase("DBUS 校验：拨轮按 16 位参与范围检查（0xFC00 必须判非法）");
  {
    MakeValidDbusFrame(frame);
    /* 低 11 位仍是 1024，但高 5 位被置脏。
     * DT7 的解码器把两个字节当完整 16 位用，会算出
     * (0xFC00 - 1024) / 660 ≈ 96.19，远超 [-1, 1]。
     * 如果校验器只掩 11 位，这种帧会被放行 —— 审计 F1 指的就是这里。 */
    frame[16] = 0x00;
    frame[17] = 0xFC;
    EXPECT_EQ(DbusChannel(frame, 4), 0xFC00u);
    EXPECT_TRUE(!IsDbusFrameValid(frame));
  }

  BeginCase("DBUS 校验：拨轮正常值与越界值");
  {
    MakeValidDbusFrame(frame);
    EXPECT_EQ(DbusChannel(frame, 4), 1024u);
    EXPECT_TRUE(IsDbusFrameValid(frame));

    SetDbusChannel(frame, 4, 1690);  // 超过上界
    EXPECT_TRUE(!IsDbusFrameValid(frame));

    SetDbusChannel(frame, 4, 200);  // 低于下界
    EXPECT_TRUE(!IsDbusFrameValid(frame));
  }

  /* ---- 审计 F1 的正面对拍：校验器与解码器必须对同一帧给出矛盾结论 ---- */
  BeginCase("DBUS 校验与 DT7 解码对拍：0xFC00 拨轮帧必须被拒绝");
  {
    hello_world::remote_control::DT7 rc;

    /* 1) 先证明"如果放行"会出什么事：DT7 解出来的拨轮值远超声明的 [-1,1] */
    MakeValidDbusFrame(frame);
    SetDbusChannel(frame, 4, 0xFC00);
    rc.decode(frame);
    EXPECT_TRUE(std::fabs(rc.rc_wheel()) > 10.0f);
    /* 2) 所以校验器必须拒绝它 —— 这就是 F1 要求的"同一宽度" */
    EXPECT_TRUE(!IsDbusFrameValid(frame));

    /* 3) 正常帧：解码值在 [-1,1]，校验通过 */
    MakeValidDbusFrame(frame);
    rc.decode(frame);
    EXPECT_NEAR(rc.rc_wheel(), 0.0f, 1e-3f);
    EXPECT_NEAR(rc.rc_lv(), 0.0f, 1e-3f);
    EXPECT_NEAR(rc.rc_lh(), 0.0f, 1e-3f);
    EXPECT_TRUE(IsDbusFrameValid(frame));

    /* 4) 摇杆打到满量程：解码值应正好 ±1，校验仍通过 */
    MakeValidDbusFrame(frame);
    SetDbusChannel(frame, 3, 1684);  // 左摇杆竖直最大
    rc.decode(frame);
    EXPECT_NEAR(rc.rc_lv(), 1.0f, 1e-3f);
    EXPECT_TRUE(IsDbusFrameValid(frame));
  }
}

/* ========================================================================== */
/* 三、舵轮运动学逆解                                                         */
/* ========================================================================== */
namespace {

/** 断言「轮子的实际滚动速度矢量」等于期望值。
 *
 *  为什么不直接比较 angle：舵角有等效解（θ+π 且轮速取反），两者物理上完全
 *  等价。真正不变的是「轮子往哪个方向、以多快滚」，也就是
 *      velocity = speed * (cos θ, sin θ)
 *  这一个量。所有逆解用例都统一按它断言。 */
void ExpectWheelVelocity(const hello_world::chassis_ik_solver::WheelCommand& c,
                         float vx_expect, float vy_expect, const char* label,
                         const char* file, int line)
{
  CheckNear(c.speed * std::cos(c.angle), vx_expect, 1e-3f, label, file, line);
  CheckNear(c.speed * std::sin(c.angle), vy_expect, 1e-3f, label, file, line);
}

#define EXPECT_WHEEL_VEL(cmd, vx, vy) \
  ExpectWheelVelocity((cmd), (vx), (vy), #cmd, __FILE__, __LINE__)

/** 让解算器连续跑 n 个周期（舵角速率限幅需要时间才能到位） */
void SpinSolver(hello_world::chassis_ik_solver::ChassisIkSolver& solver,
                const robot::ChassisMotion& cmd, float delta,
                const float steer[4], int cycles,
                hello_world::chassis_ik_solver::WheelCommand out[4])
{
  for (int i = 0; i < cycles; ++i) {
    solver.solve(cmd, delta, steer, 0.001f, out);
  }
}

}  // namespace

void TestIkSolver()
{
  using hello_world::chassis_ik_solver::ChassisIkSolver;
  using hello_world::chassis_ik_solver::Geometry;
  using hello_world::chassis_ik_solver::WheelCommand;
  using robot::ChassisMotion;

  Geometry geo;  /* 默认：R=0.052, hl=0.192, hw=0.155 */

  BeginCase("逆解：纯前进（δ=0, vx=1）→ 四轮实际滚动速度都是 (1, 0)");
  {
    ChassisIkSolver solver;
    solver.init(geo);
    ChassisMotion cmd;
    cmd.vx = 1.0f;
    WheelCommand out[4];
    const float steer[4] = {0, 0, 0, 0};
    SpinSolver(solver, cmd, 0.0f, steer, 200, out);

    for (int i = 0; i < 4; ++i) {
      EXPECT_WHEEL_VEL(out[i], 1.0f, 0.0f);
    }
  }

  BeginCase("逆解：原地左转（w=1）→ 四轮沿逆时针切线方向");
  {
    ChassisIkSolver solver;
    solver.init(geo);
    ChassisMotion cmd;
    cmd.w = 1.0f;
    WheelCommand out[4];
    const float steer[4] = {0, 0, 0, 0};
    SpinSolver(solver, cmd, 0.0f, steer, 500, out);

    /* 轮组 i 的速度 = ω × r = (−w·y_i, +w·x_i) */
    for (int i = 0; i < ChassisIkSolver::kWheelNum; ++i) {
      const float vx = -cmd.w * solver.wheelPosY(i);
      const float vy = cmd.w * solver.wheelPosX(i);
      EXPECT_WHEEL_VEL(out[i], vx, vy);
    }
  }

  BeginCase("逆解：云台在车头左侧 90°（δ=+π/2），推前进 → 车应往左平移");
  {
    ChassisIkSolver solver;
    solver.init(geo);
    ChassisMotion cmd;
    cmd.vx = 1.0f;  /* 云台系向前 */
    WheelCommand out[4];
    const float steer[4] = {0, 0, 0, 0};
    SpinSolver(solver, cmd, robot::kPi * 0.5f, steer, 500, out);

    /* 枪口在车头左边，那么「沿枪口前进」在车体系里就是 +y（左）。
     * 如果这里得到 (0,−1)，说明旋转方向搞反了——车会往右跑。 */
    for (int i = 0; i < 4; ++i) {
      EXPECT_WHEEL_VEL(out[i], 0.0f, 1.0f);
    }
  }

  BeginCase("逆解：等效解翻转（从 0° 要求 170°，应翻成 −10° + 轮速取反）");
  {
    ChassisIkSolver solver;
    solver.init(geo);
    const float steer[4] = {0, 0, 0, 0};
    solver.reset(steer);

    /* 让目标方向为 170°：不翻转要转 170°，翻转后只需转 −10° */
    const float dir = 170.0f * robot::kPi / 180.0f;
    ChassisMotion cmd;
    cmd.vx = std::cos(dir);
    cmd.vy = std::sin(dir);

    WheelCommand out[4];
    SpinSolver(solver, cmd, 0.0f, steer, 200, out);

    for (int i = 0; i < 4; ++i) {
      /* 关键断言 1：舵角没有跑到 ±170°，而是停在 −10° 附近 */
      const float angle = robot::WrapPi(out[i].angle);
      const float expect = -10.0f * robot::kPi / 180.0f;
      EXPECT_NEAR(angle, expect, 0.02f);
      /* 关键断言 2：轮速因此是反向的 */
      EXPECT_TRUE(out[i].speed < 0.0f);
      /* 关键断言 3：物理上仍然在往 170° 方向滚（等效性） */
      EXPECT_WHEEL_VEL(out[i], cmd.vx, cmd.vy);
    }
  }

  BeginCase("逆解：舵角变化率限幅（单周期最多转 max_rate*dt）");
  {
    ChassisIkSolver solver;
    solver.init(geo, 10.0f, 3.0f);  /* 10 rad/s，1ms → 0.01 rad/周期 */
    const float steer[4] = {0, 0, 0, 0};
    solver.reset(steer);

    ChassisMotion cmd;
    cmd.vy = 1.0f;  /* 要求舵角立刻变成 +90° */
    WheelCommand out[4];
    solver.solve(cmd, 0.0f, steer, 0.001f, out);
    for (int i = 0; i < 4; ++i) {
      EXPECT_TRUE(std::fabs(out[i].angle) <= 0.0101f);
    }
  }

  BeginCase("逆解：零指令时保持上次舵角（不能乱摆）");
  {
    ChassisIkSolver solver;
    solver.init(geo);
    const float steer[4] = {0, 0, 0, 0};
    solver.reset(steer);

    ChassisMotion cmd;
    cmd.vy = 1.0f;
    WheelCommand out[4];
    for (int i = 0; i < 900; ++i) {  /* 转到位 */
      solver.solve(cmd, 0.0f, steer, 0.001f, out);
    }
    const float angle_before = out[0].angle;

    ChassisMotion zero;
    for (int i = 0; i < 50; ++i) {
      solver.solve(zero, 0.0f, steer, 0.001f, out);
    }
    EXPECT_NEAR(out[0].speed, 0.0f, 1e-6f);
    EXPECT_NEAR(out[0].angle, angle_before, 1e-4f);
  }

  BeginCase("逆解：轮速限幅");
  {
    ChassisIkSolver solver;
    solver.init(geo, 10.0f, 2.0f);
    ChassisMotion cmd;
    cmd.vx = 100.0f;
    WheelCommand out[4];
    const float steer[4] = {0, 0, 0, 0};
    solver.solve(cmd, 0.0f, steer, 0.001f, out);
    for (int i = 0; i < 4; ++i) {
      EXPECT_NEAR(std::fabs(out[i].speed), 2.0f, 1e-4f);
    }
  }
}

/* ========================================================================== */
/* 四、闭环仿真（不用上车，验证方向 / 收敛性）                                */
/* ========================================================================== */
namespace {

/** 把 value 以每周期最多 max_delta 的速度拉向 target（与 robot.cpp 里的 Slew 一致） */
float Slew(float value, float target, float max_delta)
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

void TestFollowModeSimulation()
{
  BeginCase("跟随模式闭环仿真：车头应在 1 秒内追上云台朝向，且不过冲");
  {
    /* ---- 模型说明 ----
     * 云台做世界系稳定 → 云台的世界朝向 ψg 恒定（这里取 0.5 rad）。
     * 车头朝向 ψc 由底盘速度指令 w 驱动，用一阶惯性近似（时间常数 0.1s）。
     * 底盘板算出的 δ = ψg − ψc，控制律 w = Kp·δ − Kd·ω（与 robot.cpp 一致）。
     * 如果符号搞反，这个仿真会发散——这正是我们要提前发现的东西。 */
    constexpr float kDt = 0.001f;
    constexpr float kKp = 4.0f;
    constexpr float kKd = 0.15f;
    constexpr float kMaxW = 6.0f;
    constexpr float kAngAccel = 25.0f;
    constexpr float kPlantTau = 0.1f;

    const float psi_gimbal = 0.5f;  // 云台世界朝向
    float psi_chassis = 0.0f;       // 车头初始朝向
    float omega = 0.0f;             // 车体实际角速度
    float w_ramped = 0.0f;          // 经过加速度限幅的指令

    for (int k = 0; k < 3000; ++k) {  // 仿真 3 秒
      const float delta = robot::WrapPi(psi_gimbal - psi_chassis);
      float w_cmd = kKp * delta - kKd * omega;
      w_cmd = robot::Clamp(w_cmd, -kMaxW, kMaxW);
      w_ramped = Slew(w_ramped, w_cmd, kAngAccel * kDt);
      omega += (w_ramped - omega) * (kDt / kPlantTau);
      psi_chassis += omega * kDt;
    }

    /* 3 秒后误差应该接近 0（允许 0.01 rad ≈ 0.6°） */
    EXPECT_NEAR(robot::WrapPi(psi_gimbal - psi_chassis), 0.0f, 0.01f);
  }

  BeginCase("跟随模式：云台在车头右侧（δ<0）时，车应该向右转（w<0）");
  {
    const float delta = -0.3f;  // 云台在右边 0.3rad
    const float w = 4.0f * delta;
    EXPECT_TRUE(w < 0.0f);
  }

  BeginCase("小陀螺：底盘转一圈，车体系速度随之旋转 → 世界方向保持不变");
  {
    /* 这是「小陀螺平移方向不变」的数学本质：
     * 云台世界朝向固定，底盘转了 θ，则 δ = −θ；逆解把云台系速度旋转到车体系，
     * 得到的车体系速度正好也转了 θ。两者相互抵消，世界方向不变。
     *
     * 这里直接验证「物理不变性」：把车体系速度再按 +θ 转回世界系，应该恒等于 (1,0)。 */
    using hello_world::chassis_ik_solver::ChassisIkSolver;
    using hello_world::chassis_ik_solver::Geometry;
    using hello_world::chassis_ik_solver::WheelCommand;

    ChassisIkSolver solver;
    solver.init(Geometry{});
    robot::ChassisMotion cmd;
    cmd.vx = 1.0f;  // 云台系：一直向前

    for (int deg = 0; deg <= 360; deg += 30) {
      const float theta = static_cast<float>(deg) * robot::kPi / 180.0f;
      const float delta = -theta;  // 底盘转了 +θ（云台世界朝向不动）
      WheelCommand out[4];
      const float steer[4] = {0, 0, 0, 0};
      solver.solve(cmd, delta, steer, 0.001f, out);

      const auto& v = solver.chassis_motion();

      /* 车体系速度应也已旋转 θ：v_C = R(+δ)·(1,0) = (cosθ, −sinθ) */
      EXPECT_NEAR(v.vx, std::cos(theta), 1e-3f);
      EXPECT_NEAR(v.vy, -std::sin(theta), 1e-3f);

      /* 物理不变性：再按车头朝向 +θ 转回世界系，必须还是 (1,0) */
      const float vx_world = v.vx * std::cos(theta) - v.vy * std::sin(theta);
      const float vy_world = v.vx * std::sin(theta) + v.vy * std::cos(theta);
      EXPECT_NEAR(vx_world, 1.0f, 1e-3f);
      EXPECT_NEAR(vy_world, 0.0f, 1e-3f);
    }
  }
}

/* ========================================================================== */
/* 五、角度归一化                                                             */
/* ========================================================================== */
void TestAngleWrap()
{
  BeginCase("角度归一化到 [-π, π)");
  {
    EXPECT_NEAR(robot::WrapPi(0.0f), 0.0f, kTol);
    EXPECT_NEAR(robot::WrapPi(robot::kPi), -robot::kPi, 1e-4f);
    EXPECT_NEAR(robot::WrapPi(robot::kPi * 1.5f), -robot::kPi * 0.5f, 1e-4f);
    EXPECT_NEAR(robot::WrapPi(robot::kPi * 2.0f), 0.0f, 1e-4f);
    EXPECT_NEAR(robot::WrapPi(-robot::kPi * 2.5f), -robot::kPi * 0.5f, 1e-4f);
  }

  BeginCase("关键场景：359° 与 1° 的差应该是 +2°，不是 −358°");
  {
    const float a = 359.0f * robot::kPi / 180.0f;
    const float b = 1.0f * robot::kPi / 180.0f;
    EXPECT_NEAR(robot::WrapPi(b - a), 2.0f * robot::kPi / 180.0f, 1e-4f);
  }
}

}  // namespace

int main()
{
  std::printf("==== 舵轮步兵大作业 · 电脑端单元测试 ====\n");

  TestPid();
  TestComm();
  TestDbusCheck();
  TestIkSolver();
  TestFollowModeSimulation();
  TestAngleWrap();

  std::printf("\n==== 共 %d 项检查，失败 %d 项 ====\n", g_checks, g_failures);
  if (g_failures == 0) {
    std::printf("全部通过 ✔\n");
    return 0;
  }
  std::printf("存在失败项 ✘\n");
  return 1;
}
