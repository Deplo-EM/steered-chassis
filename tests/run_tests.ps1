# 编译并运行电脑端单元测试（不需要单片机）
#
# 用法（在仓库根目录 steered_skeleton/ 下执行）：
#   powershell -ExecutionPolicy Bypass -File tests/run_tests.ps1
#
# 做三件事：
#   1. 用主机上的 g++ 编译 tests/test_main.cpp 以及被它引用的纯算法源文件；
#   2. 生成 tests/build/tests.exe；
#   3. 运行并返回退出码（0 = 全部通过）。

$ErrorActionPreference = "Stop"

$repoRoot = Split-Path -Parent $PSScriptRoot
$buildDir = Join-Path $PSScriptRoot "build"
if (-not (Test-Path $buildDir)) {
    New-Item -ItemType Directory -Path $buildDir | Out-Null
}

$gxx = (Get-Command g++ -ErrorAction SilentlyContinue).Source
if (-not $gxx) {
    Write-Error "找不到 g++，请把 MinGW 的 bin 目录加入 PATH"
}

# 需要参与编译的源文件（都是纯算法，不含 HAL）
# DT7.cpp 也编进来：让「校验器」和「解码器」在同一组用例里对拍（审计 F1）
$sources = @(
    (Join-Path $PSScriptRoot "test_main.cpp"),
    (Join-Path $PSScriptRoot "stub_tick.cpp"),
    (Join-Path $repoRoot "RobotComponents\component\gimbal_chassis_comm.cpp"),
    (Join-Path $repoRoot "Chassis\RobotModules\component\chassis_ik_solver.cpp"),
    (Join-Path $repoRoot "Chassis\src\DT7.cpp")
)

# 头文件搜索路径
$includes = @(
    (Join-Path $repoRoot "RobotComponents\component"),
    (Join-Path $repoRoot "Chassis\inc"),
    (Join-Path $repoRoot "Chassis\RobotModules\component")
)

$includeArgs = $includes | ForEach-Object { "-I$_" }

$exe = Join-Path $buildDir "tests.exe"

Write-Host "编译中..." -ForegroundColor Cyan
& $gxx -std=c++20 -O1 -Wall -DUSE_UNIX_TICK @includeArgs @sources -o $exe
if ($LASTEXITCODE -ne 0) {
    Write-Error "编译失败（退出码 $LASTEXITCODE）"
}

Write-Host "运行测试..." -ForegroundColor Cyan
& $exe
exit $LASTEXITCODE
