#Requires -Version 7
<#
Daedalus 启动脚本（Windows）

用法:
  .\run.ps1                     ROS2 模式（release，默认）
  .\run.ps1 -Mode talos         Talos 模式（默认特性，无需 ROS 环境）
  .\run.ps1 -Dev                用 dev profile（不加 --release）
  .\run.ps1 -Check              只设置并检查环境，不启动模拟器
  .\run.ps1 --bin daedalus ...  额外参数原样传给 cargo run

脚本会自动完成（原理见 BUILD.md 坑位 1/7）：
  - 用 vswhere 定位 MSVC 工具集，rustup 与 MSVC link.exe 顶到 PATH 最前，
    防止 pixi 环境的 rust 1.93 / coreutils link.exe 遮蔽
  - ros2 模式下叠加 ROS2 前缀、rm_interfaces overlay、conda 运行库 DLL 目录，
    并设置 ROS_DISTRO=lyrical 与 IDL_PACKAGE_FILTER
#>
param(
    [ValidateSet("ros2", "talos")]
    [string]$Mode = "ros2",
    [string]$RosEnv = "D:\ros2\lyrical",
    [switch]$Dev,
    [switch]$Check,
    [Parameter(ValueFromRemainingArguments = $true)]
    [string[]]$CargoArgs
)

$ErrorActionPreference = "Stop"
$Repo = $PSScriptRoot
$Overlay = Join-Path $Repo "install\rm_interfaces"
# OOM abort（memory allocation failed / 0xc0000409）时打印失败分配点的回溯
if (-not $env:RUST_BACKTRACE) { $env:RUST_BACKTRACE = "1" }

# --- 定位 MSVC x64 工具集（不依赖 VS 开发者提示符） ---
$vswhere = Join-Path ${env:ProgramFiles(x86)} "Microsoft Visual Studio\Installer\vswhere.exe"
if (-not (Test-Path $vswhere)) { throw "找不到 vswhere.exe：请确认 Visual Studio / BuildTools 已安装" }
$vsRoot = & $vswhere -latest -products * -requires Microsoft.VisualStudio.Component.VC.Tools.x86.x64 -property installationPath
if (-not $vsRoot) { throw "未找到带 C++ 工具集（x86/x64）的 VS 安装" }
$toolset = Get-ChildItem (Join-Path $vsRoot "VC\Tools\MSVC") -Directory |
    Sort-Object Name -Descending | Select-Object -First 1
$MsvcBin = Join-Path $toolset.FullName "bin\Hostx64\x64"
if (-not (Test-Path (Join-Path $MsvcBin "link.exe"))) { throw "MSVC 链接器不存在: $MsvcBin" }

# --- PATH 第一段：rustup 与 MSVC 必须压过 pixi 的 rust 1.93 / coreutils link.exe ---
$env:PATH = "$env:USERPROFILE\.cargo\bin;$MsvcBin;$env:PATH"

if ($Mode -eq "ros2") {
    if (-not (Test-Path $RosEnv)) { throw "ROS2 环境不存在: $RosEnv（用 -RosEnv 另行指定）" }
    if (-not (Test-Path (Join-Path $Overlay "share"))) {
        throw "overlay 缺失: $Overlay`n请先按 BUILD.md 用 colcon 构建 rm_interfaces 并重跑本脚本"
    }
    $pixiBin = Join-Path $RosEnv ".pixi\envs\default\Library\bin"
    $env:ROS_DISTRO = "lyrical"   # 必须是 lyrical：r2r 0.9.6 的 ManualByNode 门控为 not(lyrical)
    $env:AMENT_PREFIX_PATH = "$RosEnv;$Overlay"
    $env:CMAKE_PREFIX_PATH = "$RosEnv;$Overlay"
    $env:IDL_PACKAGE_FILTER = "rm_interfaces;builtin_interfaces;geometry_msgs;sensor_msgs;std_msgs;visualization_msgs;tf2_msgs"
    $env:LIBCLANG_PATH = Join-Path $env:ProgramFiles "LLVM\bin"
    if (-not (Test-Path $env:LIBCLANG_PATH)) { Write-Warning "未找到 LLVM（$env:LIBCLANG_PATH）：增量编译 r2r 时会失败，重装请 winget install LLVM.LLVM" }
    # 注意顺序：rustup 与 MSVC 必须仍在最前（pixi 的 Library\bin 里有 rust 1.93 和其他遮蔽物），
    # 其后才是 ros2/conda/overlay 的 DLL 目录
    $env:PATH = "$env:USERPROFILE\.cargo\bin;$MsvcBin;$RosEnv\bin;$pixiBin;$Overlay\bin;$env:PATH"
}

# --- 校验解析结果 ---
$cargoSrc = (Get-Command cargo -ErrorAction Stop).Source
$rustcVer = ((& rustc --version) -split ' ')[1]
if ($rustcVer -lt [version]"1.95.0") {
    throw "rustc $rustcVer < 1.95.0（bevy 0.19 要求）。cargo 解析到 $cargoSrc — 若在 pixi 环境里启动，检查 PATH 遮蔽（BUILD.md 坑位 7），或先 rustup update"
}
$linkSrc = (Get-Command link.exe -ErrorAction SilentlyContinue).Source

Write-Host "mode     : $Mode"
Write-Host "cargo    : $cargoSrc"
Write-Host "rustc    : $rustcVer"
Write-Host "link.exe : $linkSrc"
if ($Mode -eq "ros2") {
    Write-Host "ros env  : $RosEnv (ROS_DISTRO=$env:ROS_DISTRO)"
    Write-Host "overlay  : $Overlay"
}

if ($Check) { Write-Host "`n环境检查通过（-Check，未启动）。"; exit 0 }

# --- 启动 ---
Set-Location $Repo
# 注意：if 表达式赋值会拆包单元素数组（标量字符串 splat 时会逐字符传参），
# 类型标注必须写在赋值目标上（右侧的 [string[]] 转型拦不住输出层拆包）
[string[]]$profileArgs = if ($Dev) { @() } else { @("--release") }
[string[]]$featureArgs = if ($Mode -eq "ros2") { @("--no-default-features", "--features", "ros2") } else { @() }
& cargo run @featureArgs @profileArgs @CargoArgs
exit $LASTEXITCODE
