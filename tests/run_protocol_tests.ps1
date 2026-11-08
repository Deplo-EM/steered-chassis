$ErrorActionPreference = 'Stop'
$repo = Split-Path -Parent $PSScriptRoot
$build = Join-Path $PSScriptRoot 'build'
New-Item -ItemType Directory -Force -Path $build | Out-Null
$exe = Join-Path $build 'protocol_safety.exe'
& g++ -std=c++20 -O1 -Wall -DUSE_UNIX_TICK ('-I' + $repo + '/RobotComponents/component') ('-I' + $repo + '/Chassis/inc') ($PSScriptRoot + '/test_protocol_safety.cpp') ($repo + '/RobotComponents/component/gimbal_chassis_comm.cpp') -o $exe
if ($LASTEXITCODE -ne 0) { exit $LASTEXITCODE }
& $exe
exit $LASTEXITCODE
