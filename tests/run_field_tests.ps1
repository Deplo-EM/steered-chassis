$ErrorActionPreference='Stop'
$repo=Split-Path -Parent $PSScriptRoot
$build=Join-Path $PSScriptRoot 'build'
New-Item -ItemType Directory -Force -Path $build | Out-Null
foreach ($board in @('Chassis','Gimbal')) {
  $exe=Join-Path $build "can_config_$board.exe"
  & g++ -std=c++20 -Wall ('-I'+(Join-Path $PSScriptRoot 'can_init_stubs')) (Join-Path $PSScriptRoot 'test_can_configuration.cpp') (Join-Path $repo "$board/Core/Src/can.c") -o $exe
  if($LASTEXITCODE -ne 0){exit $LASTEXITCODE}
  & $exe
  if($LASTEXITCODE -ne 0){exit $LASTEXITCODE}
}
$exe=Join-Path $build 'remote_relay.exe'
$sources=@('tests/test_remote_relay.cpp','tests/stub_tick.cpp','RobotComponents/component/gimbal_chassis_comm.cpp','Chassis/src/DT7.cpp') | ForEach-Object {Join-Path $repo $_}
$inc=@('RobotComponents/component','Chassis/inc') | ForEach-Object {'-I'+(Join-Path $repo $_)}
& g++ -std=c++20 -O1 -Wall -DUSE_UNIX_TICK @inc @sources -o $exe
if($LASTEXITCODE -ne 0){exit $LASTEXITCODE}
& $exe
exit $LASTEXITCODE
