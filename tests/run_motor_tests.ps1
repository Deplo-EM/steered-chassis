$ErrorActionPreference = 'Stop'
$repo = Split-Path -Parent $PSScriptRoot
$build = Join-Path $PSScriptRoot 'build'
New-Item -ItemType Directory -Force -Path $build | Out-Null
$fail = 0
foreach ($board in @('Chassis','Gimbal')) {
    $exe = Join-Path $build "motor_$board.exe"
    $sources = @('tests/test_motor_driver.cpp',"$board/src/motor_DaMiao.cpp","$board/src/motor_base.cpp","$board/src/base.cpp") | ForEach-Object { Join-Path $repo $_ }
    & g++ -std=c++20 -O1 -Wall -Wno-unknown-pragmas '-DSTM32_HAL_FILENAME=<host_hal.hpp>' ('-I'+$repo+'/tests/control_stubs') ('-I'+$repo+'/'+$board+'/inc') @sources -o $exe
    if ($LASTEXITCODE -ne 0) { exit $LASTEXITCODE }
    & $exe
    if ($LASTEXITCODE -ne 0) { $fail = 1 }
}
exit $fail
