param([switch]$GroundOnly, [string[]]$Only)
$ErrorActionPreference = 'Stop'
$repo = Split-Path -Parent $PSScriptRoot
$build = Join-Path $PSScriptRoot 'build'
New-Item -ItemType Directory -Force -Path $build | Out-Null
$gxx = (Get-Command g++ -ErrorAction Stop).Source
$failed = 0
$groundProfiles = @('GroundLocked', 'GroundMotion', 'GroundGimbal')
$profiles = if ($Only) { $Only } elseif ($GroundOnly) { $groundProfiles } else { @('Chassis', 'ChassisCurrent', 'Gimbal', 'GimbalGravity', 'ChassisProbe') + $groundProfiles }
foreach ($profile in $profiles) {
    $board = if ($profile -in @('Gimbal', 'GimbalGravity', 'GroundGimbal')) { 'Gimbal' } else { 'Chassis' }
    $sources = @('tests/test_robot_safety.cpp', "$board/RobotModules/component/robot.cpp", "$board/Task/component/comm_task.cpp", "$board/src/DT7.cpp", "$board/src/motor_base.cpp", "$board/src/base.cpp", 'RobotComponents/component/gimbal_chassis_comm.cpp')
    if ($board -eq 'Chassis') { $sources += @('Chassis/RobotModules/component/chassis.cpp', 'Chassis/RobotModules/component/chassis_ik_solver.cpp') }
    else { $sources += @('Gimbal/RobotModules/component/gimbal.cpp', 'Gimbal/Instance/component/ins_pid.cpp') }
    $sources = $sources | ForEach-Object { Join-Path $repo $_ }
    $inc = @('tests/safety_stubs', 'RobotComponents/component', "$board/inc", "$board/RobotModules/component", "$board/Task/component", "$board/Instance/component") | ForEach-Object { '-I' + (Join-Path $repo $_) }
    $defines = @('-DUSE_UNIX_TICK', '-DSTM32_HAL_FILENAME=<host_hal.hpp>')
    if ($board -eq 'Chassis') { $defines += '-DTEST_CHASSIS' }
    if ($profile -eq 'ChassisCurrent') { $defines += '-DROBOT_STEER_CURRENT_FIELD=1' }
    if ($profile -eq 'ChassisProbe') { $defines += '-DROBOT_STEER_MODE_PROBE=1' }
    if ($profile -in @('GroundLocked', 'GroundMotion')) {
        $defines += @('-DROBOT_GROUND_TRIAL=1', '-DROBOT_STEER_CURRENT_FIELD=1')
    }
    if ($profile -eq 'GroundMotion') { $defines += '-DROBOT_GROUND_TRIAL_MOTION=1' }
    if ($profile -eq 'GroundGimbal') { $defines += '-DROBOT_GROUND_GIMBAL_LOCK=1' }
    if ($profile -eq 'GimbalGravity') { $defines += '-DROBOT_PITCH_GRAVITY_FF=1' }
    $exe = Join-Path $build "safety_$profile.exe"
    & $gxx -std=c++20 -O1 -Wall @defines @inc @sources -o $exe
    if ($LASTEXITCODE -ne 0) { exit $LASTEXITCODE }
    Write-Host "Safety profile: $profile"
    & $exe
    if ($LASTEXITCODE -ne 0) { $failed = 1 }
}
if ($failed -ne 0) { exit $failed }
if ($GroundOnly -or $Only) { exit 0 }
$exe = Join-Path $build 'test_steer_mode_probe.exe'
& $gxx -std=c++20 -O1 -Wall ('-I' + (Join-Path $repo 'Chassis/Task/component')) (Join-Path $PSScriptRoot 'test_steer_mode_probe.cpp') -o $exe
if ($LASTEXITCODE -ne 0) { exit $LASTEXITCODE }
& $exe
if ($LASTEXITCODE -ne 0) { exit $LASTEXITCODE }
exit $failed
