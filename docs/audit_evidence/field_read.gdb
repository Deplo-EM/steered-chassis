# Read-only diagnostics; isolate actuator power before halting the CPU.
set pagination off
set language c++
set $r = &'GetRobot()::s_robot'
set $imu = &'GetImu()::s_imu'
set $peer = &'GetGimbalChassisComm()::s_comm'
print $r->state_
print $r->estop_
print $r->can_tx_guard_
print $r->rc_checker_
print $r->motor_checkers_
print $imu->error_state_
print $imu->inited_
print $imu->calibrated_
print $imu->samples_
print $imu->sample_valid_
print $imu->healthy_
print $peer->received_
print $peer->last_rx_tick_
print uwTick
print $peer->chassis_data_
print $peer->gimbal_data_

print $r->remote_input_
print $r->peer_input_
print $r->motors_online_
print $imu->last_sample_tick_
print $imu->last_clock_tick_
print $imu->last_accel_tick_
print $imu->last_gyro_tick_
print $imu->bad_samples_total_
print $imu->clock_seen_
print $imu->accel_seen_
print $imu->gyro_seen_
