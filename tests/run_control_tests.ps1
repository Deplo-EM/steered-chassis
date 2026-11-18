$ErrorActionPreference = 'Stop'
$repo = Split-Path -Parent $PSScriptRoot
$build = Join-Path $PSScriptRoot 'build'
New-Item -ItemType Directory -Force -Path $build | Out-Null
$gxx = (Get-Command g++ -ErrorAction Stop).Source
$sources = @('tests/test_control.cpp', 'Chassis/RobotModules/component/chassis.cpp', 'Chassis/RobotModules/component/chassis_ik_solver.cpp', 'Chassis/Instance/component/ins_chassis_iksolver.cpp', 'Chassis/Instance/component/ins_pid.cpp', 'Chassis/Instance/component/ins_motor.cpp', 'Chassis/src/motor_GM6020.cpp', 'Chassis/src/motor_M3508.cpp', 'Chassis/src/motor_base.cpp', 'Chassis/src/base.cpp') | ForEach-Object { Join-Path $repo $_ }
$inc = @('tests/control_stubs', 'RobotComponents/component', 'Chassis/inc', 'Chassis/RobotModules/component', 'Chassis/Instance/component') | ForEach-Object { '-I' + (Join-Path $repo $_) }
$failed = 0
foreach ($profile in @('voltage', 'current', 'finish_current')) {
    $defs = @('-DSTM32_HAL_FILENAME=<host_hal.hpp>')
    if ($profile -ne 'voltage') { $defs += '-DROBOT_STEER_CURRENT_FIELD=1' }
    if ($profile -eq 'finish_current') { $defs += '-DROBOT_COMPETITION_FINISH=1' }
    $exe = Join-Path $build "control_tests_$profile.exe"
    & $gxx -std=c++20 -O1 -Wall -Wno-unknown-pragmas @defs @inc @sources -o $exe
    if ($LASTEXITCODE -ne 0) { exit $LASTEXITCODE }
    Write-Host "Control profile: $profile"
    & $exe
    if ($LASTEXITCODE -ne 0) { $failed = 1 }
}
exit $failed
