$ErrorActionPreference = 'Stop'
$repo = Split-Path -Parent $PSScriptRoot
$build = Join-Path $PSScriptRoot 'build'
New-Item -ItemType Directory -Force -Path $build | Out-Null
$gxx = (Get-Command g++ -ErrorAction Stop).Source
$sources = @('tests/test_control.cpp', 'Chassis/RobotModules/component/chassis.cpp', 'Chassis/RobotModules/component/chassis_ik_solver.cpp', 'Chassis/Instance/component/ins_pid.cpp', 'Chassis/src/motor_base.cpp', 'Chassis/src/base.cpp') | ForEach-Object { Join-Path $repo $_ }
$inc = @('tests/control_stubs', 'RobotComponents/component', 'Chassis/inc', 'Chassis/RobotModules/component', 'Chassis/Instance/component') | ForEach-Object { '-I' + (Join-Path $repo $_) }
$exe = Join-Path $build 'control_tests.exe'
& $gxx -std=c++20 -O1 -Wall '-DSTM32_HAL_FILENAME=<host_hal.hpp>' @inc @sources -o $exe
if ($LASTEXITCODE -ne 0) { exit $LASTEXITCODE }
& $exe
exit $LASTEXITCODE
