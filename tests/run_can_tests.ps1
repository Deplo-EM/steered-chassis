$ErrorActionPreference='Stop'
$repo=Split-Path -Parent $PSScriptRoot
$build=Join-Path $PSScriptRoot 'build'
New-Item -ItemType Directory -Force -Path $build | Out-Null
$fail=0
foreach ($board in @('Chassis','Gimbal')) {
  $exe=Join-Path $build "can_$board.exe"
  $sources=@('tests/test_can_transport.cpp',"$board/src/HW_can.cpp","$board/src/base.cpp",'RobotComponents/component/gimbal_chassis_comm.cpp') | ForEach-Object { Join-Path $repo $_ }
  $inc=@('tests/can_stubs','RobotComponents/component',"$board/inc","$board/RobotModules/component",'tests/safety_stubs') | ForEach-Object { '-I'+(Join-Path $repo $_) }
  $defines=@('-DSTM32_HAL_FILENAME=<host_hal.hpp>')
  if($board -eq 'Chassis'){$defines+='-DTEST_CHASSIS'}
  & g++ -std=c++20 -O1 -Wall -Wno-unknown-pragmas @defines @inc @sources -o $exe
  if($LASTEXITCODE -ne 0){exit $LASTEXITCODE}
  & $exe
  if($LASTEXITCODE -ne 0){$fail=1}
}
exit $fail
