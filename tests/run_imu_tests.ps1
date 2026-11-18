$ErrorActionPreference = 'Stop'
$repo = Split-Path -Parent $PSScriptRoot
$build = Join-Path $PSScriptRoot 'build'
New-Item -ItemType Directory -Force -Path $build | Out-Null
$gxx = (Get-Command g++ -ErrorAction Stop).Source
$failed = 0
foreach ($board in @('Chassis', 'Gimbal')) {
    $exe = Join-Path $build "imu_$board.exe"
    $stub = Join-Path $PSScriptRoot 'imu_stubs/dependencies.hpp'
    $inc = '-I' + (Join-Path $repo "$board/inc")
    $test = Join-Path $PSScriptRoot 'test_imu.cpp'
    $source = Join-Path $repo "$board/src/imu.cpp"
    & $gxx -std=c++20 -O1 -Wall -Wextra -include $stub $inc ('-I'+$repo+'/RobotComponents/component') $test $source -o $exe
    if ($LASTEXITCODE -ne 0) { exit $LASTEXITCODE }
    & $exe
    if ($LASTEXITCODE -ne 0) { $failed = 1 }
}
exit $failed
