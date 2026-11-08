$ErrorActionPreference = 'Stop'
$repo = Split-Path -Parent $PSScriptRoot
$build = Join-Path $PSScriptRoot 'build'
New-Item -ItemType Directory -Force -Path $build | Out-Null
$gxx = (Get-Command g++ -ErrorAction Stop).Source
$failed = 0
foreach ($board in @('Chassis', 'Gimbal')) {
    $sources = @('tests/test_robot_safety.cpp', "$board/RobotModules/component/robot.cpp", "$board/Task/component/comm_task.cpp", "$board/src/DT7.cpp", "$board/src/motor_base.cpp", "$board/src/base.cpp", 'RobotComponents/component/gimbal_chassis_comm.cpp')
    if ($board -eq 'Chassis') { $sources += @('Chassis/RobotModules/component/chassis.cpp', 'Chassis/RobotModules/component/chassis_ik_solver.cpp') }
    else { $sources += 'Gimbal/RobotModules/component/gimbal.cpp' }
    $sources = $sources | ForEach-Object { Join-Path $repo $_ }
    $inc = @('tests/safety_stubs', 'RobotComponents/component', "$board/inc", "$board/RobotModules/component", "$board/Task/component") | ForEach-Object { '-I' + (Join-Path $repo $_) }
    $defines = @('-DUSE_UNIX_TICK', '-DSTM32_HAL_FILENAME=<host_hal.hpp>')
    if ($board -eq 'Chassis') { $defines += '-DTEST_CHASSIS' }
    $exe = Join-Path $build "safety_$board.exe"
    & $gxx -std=c++20 -O1 -Wall @defines @inc @sources -o $exe
    if ($LASTEXITCODE -ne 0) { exit $LASTEXITCODE }
    & $exe
    if ($LASTEXITCODE -ne 0) { $failed = 1 }
}
exit $failed
