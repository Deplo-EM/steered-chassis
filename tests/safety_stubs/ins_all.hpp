#pragma once
#include "motor_base.hpp"
#include "DT7.hpp"
#include "gimbal_chassis_comm.hpp"
#include "robot.hpp"
hello_world::motor::Motor* GetMotorWheel(int);
hello_world::motor::Motor* GetMotorSteer(int);
hello_world::motor::Motor* GetMotorYaw();
hello_world::motor::Motor* GetMotorPitch();
hello_world::remote_control::DT7* GetRemoteControl();
robot::Robot* GetRobot();
#ifdef TEST_CHASSIS
robot::Chassis* GetChassis();
#endif
robot::GimbalChassisComm* GetGimbalChassisComm();
