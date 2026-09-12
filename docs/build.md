# Daedalus 编译指南

本文档整理两条构建路径：**默认构建（Talos IPC）** 与 **ROS2 集成构建**，以及 Windows 本机（pixi 版 ROS2 环境 `D:\ros2\lyrical`）的完整流程和已知坑位。

## 平台与特性概览

| 构建方式 | 命令 | 平台 | 前置条件 |
|---|---|---|---|
| 默认（talos） | `cargo run --release` | Windows / Linux / macOS | 仅需 Rust 工具链 |
| ROS2 集成 | `cargo run --no-default-features --features ros2 --release` | Windows 用 pixi 环境（见二）；Linux 原生（见三） | ROS2 环境 + libclang + 已构建的 `rm_interfaces` |
| 云台 mock 服务器 | `cargo run --features talos,ffmpeg --bin talos_gimbal_mock_server` | 同上 | FFmpeg 开发库（见四） |

> ROS2 采集激活时 Talos 插件自动跳过；`DAEDALUS_FORCE_TALOS_CAPTURE=1` 可强制两者并存。

---

## 一、默认构建（Talos，无需 ROS）

```sh
cargo run --release
```

依赖全部跨平台（bevy / avian3d / memmap2），Windows 直接可用。macOS 额外编译 MetalFX（自动），WSL 下有 wgpu 适配（自动），均无需手动干预。

---

## 二、ROS2 构建（Windows + `D:\ros2\lyrical` pixi 环境）

### 环境清单（一次性安装）

| 组件 | 用途 | 状态 / 安装方式 |
|---|---|---|
| Rust ≥ 1.95（bevy 0.19 的硬性 rustc 要求，edition 2024 只需 1.85） | 编译本体 | 已装 1.97.1（stable，`rustup update` 维护） |
| VS BuildTools 2026（C++ 工具集，x64） | 编译 `rm_interfaces` 的 C++ | 已装 14.51；用 **x64 Native Tools** 提示符 |
| pixi ≥ 0.74 | 激活 ROS2 环境的工具链 | 已装 |
| LLVM | 提供 `libclang.dll`，bindgen 编译期解析 ROS2 C 头文件必需 | `winget install LLVM.LLVM` |
| ROS2 环境 `D:\ros2\lyrical` | OSRF pixi 源码构建的完整 ROS2（含 colcon/cmake） | 已就绪 |

### 关键坑位（先读再动手）

1. **r2r 版本与 `ROS_DISTRO=lyrical`（已修复，勿回退）**。crates.io 上的 r2r 0.9.5 编不过 lyrical/rolling 的头文件（`rcl_timer_init` 已改名 `rcl_timer_init2`、`RMW_QOS_POLICY_LIVELINESS_MANUAL_BY_NODE` 已删除），而修复版 0.9.6 **从未发布到 crates.io**。本仓库的 `Cargo.toml` 已用 `[patch.crates-io]` 把 `r2r`/`r2r_rcl`/`r2r_common` 指向 GitHub tag `0.9.6`；`third_party/r2r_msg_gen` 是 0.9.6 源码 + 本地 patch（本环境 bindgen 生成的是**不透明**的 `rosidl_message_type_support_t`，无 `data` 字段，必须经手工声明的 `MessageTypeSupportView` 取数据），经 `[patch."https://github.com/sequenceplanner/r2r"]` 注入。**`ROS_DISTRO` 必须设为 `lyrical`**：0.9.6 对 `ManualByNode` 的门控是 `not(lyrical)`，设 `rolling` 反而会引用本环境已删除的枚举。
2. **需要用高版本 CMake**，pixi 环境的 cmake 是 3.28.3，不认识 VS2026 生成器（"VS 18" 支持自 CMake 4.1 起），可以单独装一个 CMake 或者用 VC 自带的 CMake。
3. **中文系统上 colcon 会因编码崩溃**：colcon 以 UTF-8 解码子进程输出，而中文 Windows 上 MSVC 工具输出 GBK，一旦报错信息含中文就会抛 `UnicodeDecodeError` 并吞掉真实错误。构建前 `$env:VSLANG = "1033"` 强制 MSVC 输出英文即可避免；真实报错原文始终在 `log\build_<时间戳>\rm_interfaces\stderr.log`（GBK 编码）里。
4. **必须以 Release 构建 rm_interfaces**（`-DCMAKE_BUILD_TYPE=Release`）：Debug 模式下 MSVC 定义 `_DEBUG`，Python 的 `pyconfig.h` 会 `#pragma` 链接 `python312_d.lib`，而 conda 的 Python 只发布 release 库——Debug 配置在此环境必然 `LNK1104`。另注意：**会话环境变量 `CMAKE_BUILD_TYPE=Debug` 会被 CMake 无感种进缓存**（来源隐蔽，注册表/激活脚本/colcon 均可能无责），构建前 `$env:CMAKE_BUILD_TYPE = $null` 清掉；命令行 `-D` 优先级高于缓存和环境变量，显式传参永远安全。
5. **libclang 只在编译期需要**，不参与生成代码、运行时不需要；默认 talos 构建也不需要。
6. 换 ROS 环境 / 修改消息定义后，需 `cargo clean -p r2r_msg_gen` 强制重新生成绑定。
7. **`pixi shell` 的 PATH 遮蔽问题（三处）**：其一，环境内钉死的 rust 1.93.0（`pixi.toml` 中 `rust = "==1.93.0"`，装在 `Library\bin`）遮住 rustup，而 bevy 0.19 要求 rustc ≥ 1.95，报 `rustc 1.93.0 is not supported`；其二，`Library\usr\bin\link.exe` 是 **GNU coreutils 的建硬链接工具**，会遮住 MSVC 链接器，报 `linking with link.exe failed` 伴随 `/usr/bin/link: extra operand`。`pixi shell` **之后**执行一行 `$env:PATH = "$env:USERPROFILE\.cargo\bin;${env:VCToolsInstallDir}bin\Hostx64\x64;$env:PATH"` 同时解决（`$env:VCToolsInstallDir` 是 VS 提示符自带变量，免写死版本号），用 `rustc --version` 和 `Get-Command link.exe` 验证。其三反着来：`Library\bin` **不能从 PATH 里去掉**——ROS2 DLL 依赖的 conda 运行库（`yaml.dll`/`spdlog.dll`/`fmt.dll` 等）在里面，缺了会在启动 r2r 构建脚本或运行 exe 时报 `STATUS_DLL_NOT_FOUND`；同理 `install\rm_interfaces\bin`（overlay 消息 DLL）必须在 PATH 上，手动设 `$env:PATH` 时注意包含此目录。
8. **ros2 采集代码的两处源码适配已直接修在 `src/ros2/plugin.rs`**：bevy 0.19 的 `TextureFormat::bevy_default()` 需显式导入 `BevyDefault` trait；装甲中心查询从按名搜索（`suffix("CENTER")`）改为 `ArmorParts::marker()` 直接取实体——旧查询在 vehicle/hero 模型上能命中 `*_ARMOR_CENTER` 节点，但 OUTPOST.glb 只有 `*_ARMOR_MARKER` 节点，outpost 场景下 `.unwrap()` 必 panic。注意 marker 节点自带 ~180° 局部旋转，`armor_N` tf 朝向相对旧实现有翻转（有意改用装甲板平面节点）。

### 完整流程

以下命令全部在 **"x64 Native Tools Command Prompt for VS 2026"（PowerShell 版）** 中执行（ROS2 同时提供了 `.ps1` 版本，pwsh 用 `. .\setup.ps1` 点执行）。

**1. 激活环境（每次开新终端都要做）**

```powershell
$env:LIBCLANG_PATH = "C:\Program Files\LLVM\bin"

Set-Location D:\ros2\lyrical
pixi shell                    # 激活工具链（colcon/python）
# 以下在 pixi 子 Shell 中执行

# 顶回被 pixi 遮蔽的 rustup 与 MSVC link.exe（坑位 7）
$env:PATH = "$env:USERPROFILE\.cargo\bin;${env:VCToolsInstallDir}bin\Hostx64\x64;$env:PATH"

. .\setup.ps1                 # 激活 ROS2 前缀（PATH / AMENT_PREFIX_PATH）
$env:ROS_DISTRO = "lyrical"   # 必须是 lyrical（坑位 1），不要用 rolling
```

**2. 构建 rm_interfaces（自定义消息，一次性；消息定义变更后重做）**

```powershell
Set-Location D:\Git_clone\bevy_robomaster_simulator
$env:VSLANG = "1033"
$env:CMAKE_BUILD_TYPE = $null
colcon build --packages-select rm_interfaces --cmake-args -DCMAKE_BUILD_TYPE=Release -DBUILD_TESTING=OFF
. .\install\setup.ps1         # 叠加 overlay，让 r2r 能发现 Armor/GimbalCmd 等自定义消息
```

产物在 `install\`（已被 .gitignore 忽略）。

**3. 编译并运行模拟器**

```powershell
# 源码环境消息包极多，过滤后绑定生成快一个量级（filter 不做依赖解析，嵌套包须列全；tf2_msgs 别漏）
$env:IDL_PACKAGE_FILTER = "rm_interfaces;builtin_interfaces;geometry_msgs;sensor_msgs;std_msgs;visualization_msgs;tf2_msgs"

cargo run --no-default-features --features ros2 --release
```

**运行时也必须在完成上述激活的同一终端里执行**（运行期需要 ROS2 的 DLL 在 PATH 上）。始终用 `cargo run` 启动：它会自动把 `target\release\deps` 和工具链的动态 std 目录加入 PATH（bevy 的 `dynamic_linking` 需要），并且其注入的 `CARGO_MANIFEST_DIR` 会让 bevy 找到仓库根的 `assets\`。若要直接执行 exe，需自行设置 `$env:BEVY_ASSET_ROOT = "D:\Git_clone\bevy_robomaster_simulator"` 并把上述目录加入 PATH。

### 日常启动：`run.ps1`

上述"激活环境 → 构建/运行"的完整流程已封装为仓库根目录的 `run.ps1`（PowerShell 7，无需 VS 开发者提示符、无需 `pixi shell`）：

```powershell
.\run.ps1              # ROS2 模式（release，默认）
.\run.ps1 -Mode talos  # 默认 talos 特性，不需要任何 ROS 环境
.\run.ps1 -Check       # 只设置并校验环境，不启动
.\run.ps1 -Dev         # dev profile（不加 --release）
```

脚本用 vswhere 动态定位 MSVC 工具集，自动完成 PATH 遮蔽修正（坑位 7）与全部环境变量（`ROS_DISTRO=lyrical`、`IDL_PACKAGE_FILTER`、AMENT/CMAKE 前缀等），启动前校验 cargo/rustc/link.exe 的解析结果和 rustc ≥ 1.95，失败会给出对应本文坑位提示。`rm_interfaces` 消息变更后仍需按上节第 2 步用 colcon 重建 overlay。

> Linux 原生构建无需本节的 NMake/编码/PATH 遮蔽那一类处理，见下一章【三、ROS2 构建（Linux）】。

---

## 三、ROS2 构建（Linux，Ubuntu 参考）

Linux 是 ROS2 生态的原生平台，流程比 Windows 简单得多：系统 gcc + 默认生成器即可，没有 NMake、编码、PATH 遮蔽那一类坑位。以下以 Ubuntu 22.04 / 24.04 为参考。

### 环境清单（一次性安装）

| 组件 | 用途 |
|---|---|
| ROS2 Humble / Jazzy / Lyrical | 构建 overlay 与运行期共享库；`ROS_DISTRO` 由 `setup.bash` 自动设置，需与所 source 的发行版一致 |
| Rust ≥ 1.95 | 编译本体；用 rustup 安装维护（发行版自带 rustc 过旧，bevy 0.19 硬性要求） |
| `libclang-dev` | r2r 的 bindgen 编译期解析 ROS2 C 头文件（运行期不需要） |
| `build-essential` / `pkg-config` / `cmake` | C/C++ 工具链与构建系统 |
| `libudev-dev` | bevy 手柄支持（gilrs）链接需要 |
| `libasound2-dev` | bevy 音频（ALSA/cpal）链接需要 |
| `python3-colcon-common-extensions` | colcon 构建工具 |
| GPU 驱动 + 图形会话（运行期） | wgpu 渲染：AMD/Intel 装 `mesa-vulkan-drivers`，NVIDIA 装专有驱动；仿真器是 GUI 程序，需要桌面/显示（WSL2 见下） |

```sh
sudo apt update
sudo apt install -y build-essential pkg-config cmake git libclang-dev \
  libudev-dev libasound2-dev python3-colcon-common-extensions
```

Rust 按 [rustup](https://rustup.rs) 官方脚本安装，之后 `rustup update` 维持 ≥ 1.95。

### 完整流程

**1. 构建 rm_interfaces overlay（一次性；消息定义或 ROS 环境变更后重做）**

```sh
source /opt/ros/<distro>/setup.bash
cd /path/to/bevy_robomaster_simulator
colcon build --packages-select rm_interfaces --cmake-args -DCMAKE_BUILD_TYPE=Release -DBUILD_TESTING=OFF
source install/setup.bash        # 叠加 overlay，让 r2r 能发现 Armor/GimbalCmd 等自定义消息
```

产物在 `install/`。

**2. 编译并运行模拟器**

```sh
source /opt/ros/<distro>/setup.bash        # 运行期也需要：r2r 加载 librcl 等共享库依赖它注入 LD_LIBRARY_PATH
source install/setup.bash
export IDL_PACKAGE_FILTER=rm_interfaces;builtin_interfaces;geometry_msgs;sensor_msgs;std_msgs;visualization_msgs;tf2_msgs

cargo run --no-default-features --features ros2 --release
```

### 流程要点（Linux 专属）

- **必须用 `cargo run` 启动**：本仓库在 `Cargo.toml` 中无条件启用了 bevy 的 `dynamic_linking`，bevy 以动态库参与链接，直接执行 `target/release/daedalus` 会报找不到 `libbevy_dylib*.so`。`cargo run` 会自动设置 `LD_LIBRARY_PATH`，并通过 `CARGO_MANIFEST_DIR` 让 bevy 找到仓库根的 `assets/`。首次编译会把 bevy 作为动态库全量构建一遍，耗时较长属正常。
- `config.toml` 从**当前工作目录**读取，保持在仓库根目录下启动。
- 消息定义变更、升级或更换 ROS2 环境后：重建 overlay 并执行 `cargo clean -p r2r_msg_gen`（原理同 Windows 坑位 1/6，否则运行期可能在 GimbalCmd 订阅处 panic `RCL_RET_ERROR`）。

### WSL2 / 无显示器环境

- **WSL2（WSLg）可运行**：`src/main.rs` 已内置 WSL 下 wgpu 适配器不合规的 workaround，前提是 Windows 侧安装了支持 WSL 的 GPU 驱动。
- **无头服务器无法运行**：仿真器是 GUI 程序，Xvfb 等虚拟显示没有可用的 Vulkan 实现，基本走不通；只做话题联调请用带桌面的机器。

### 常见问题（Linux）

| 症状 | 原因 / 解决 |
|---|---|
| `linker 'cc' not found` / 缺 C++ 编译器 | `sudo apt install build-essential` |
| bindgen / clang-sys 报找不到 libclang | `sudo apt install libclang-dev`（个别发行版还需 `llvm-dev`） |
| 编译 bevy 时 alsa-sys / pkg-config 找不到 alsa | `sudo apt install libasound2-dev` |
| 编译 bevy 时报 libudev 缺失 | `sudo apt install libudev-dev` |
| 启动报 `error while loading shared libraries: librcl.so ...` | 当前 shell 没 source ROS2 环境，重新两级 `source`（见流程第 2 步） |
| 启动报找不到 `libbevy_dylib*.so` | 直接执行了二进制；改用 `cargo run`（见流程要点） |
| 启动后 wgpu 报无可用 Adapter / Vulkan 初始化失败 | 装对应 GPU 驱动的 Vulkan 用户态（AMD/Intel `mesa-vulkan-drivers`，NVIDIA 专有驱动）；WSL2 见上 |

---

## 四、FFmpeg 开发库（可选：仅 `talos_gimbal_mock_server` 需要）

日常跑模拟器（talos 或 ros2）**不需要** FFmpeg。需要 mock 服务器时，Windows 推荐 vcpkg：

```powershell
git clone https://github.com/microsoft/vcpkg D:\vcpkg
D:\vcpkg\bootstrap-vcpkg.bat
D:\vcpkg\vcpkg install ffmpeg:x64-windows-release

$env:FFMPEG_DIR = "D:\vcpkg\installed\x64-windows-release"
cargo run --features talos,ffmpeg --bin talos_gimbal_mock_server
```

运行时把 `D:\vcpkg\installed\x64-windows-release\bin` 加入 PATH（avcodec 等 DLL），或将 DLL 拷到 exe 旁。备选：gyan.dev 的 *ffmpeg-release-full-shared* 解压后直接设 `FFMPEG_DIR`，但对 MSVC 的兼容性不如 vcpkg 稳。

---

## 五、故障排查速查

> 本表以 Windows/pixi 流程为主；Linux 专属问题见第三章末尾的「常见问题（Linux）」。

| 症状 | 原因 / 解决 |
|---|---|
| r2r 编译报 `rcl_timer_init` 不存在 / `MANUAL_BY_NODE` 不存在 | 用了 crates.io 的 r2r 0.9.5；本仓库 `[patch.crates-io]` 已指 git 0.9.6，勿删（坑位 1） |
| `ROS_DISTRO not supported: lyrical` | r2r 被回退到 0.9.5（0.9.6 才支持 lyrical）；检查 Cargo.toml patch 段是否被改动 |
| colcon 报 `UnicodeDecodeError` 且看不到真实错误 | 中文 GBK 输出撞上 colcon 的 UTF-8 解码；`$env:VSLANG = "1033"`，并到 `log\build_*\rm_interfaces\stderr.log` 看原文 |
| `LNK1104: 无法打开文件"python312_d.lib"` | Debug 构建类型混入（会话环境变量 `CMAKE_BUILD_TYPE` 或缓存残留）；`$env:CMAKE_BUILD_TYPE = $null` 清变量 + `-DCMAKE_BUILD_TYPE=Release` 显式指定 |
| `rustc 1.93.0 is not supported`（bevy 要求 1.95） | `pixi shell` 后环境内 rust 1.93 遮住了 rustup；`$env:PATH = "$env:USERPROFILE\.cargo\bin;$env:PATH"` + `rustup update` |
| `linking with link.exe failed`（伴随 `/usr/bin/link: extra operand`） | pixi 的 coreutils `link.exe` 遮住了 MSVC 链接器；按坑位 7 把 `${env:VCToolsInstallDir}bin\Hostx64\x64` 顶回 PATH 最前（`Get-Command link.exe` 验证） |
| `STATUS_DLL_NOT_FOUND`（0xc0000135）启动即崩 | PATH 缺 ROS2 DLL 运行库目录：`D:\ros2\lyrical\bin`、`...\pixi\envs\default\Library\bin`、`install\rm_interfaces\bin` 三个都要在 |
| r2r_msg_gen 编译报 `no field data` | third_party patch 未生效（缺 `[patch."https://github.com/sequenceplanner/r2r"]` 段）；见坑位 1 |
| cmake 报找不到 Visual Studio 实例 / 不支持的生成器 | cmake 3.28 不支持 VS2026 生成器；用 VC 自带的CMake，设置环境变量`$env:Path = "C:\Program Files (x86)\Microsoft Visual Studio\18\BuildTools\Common7\IDE\CommonExtensions\Microsoft\CMake\CMake\bin;$env:Path"` |
| bindgen 报找不到 libclang | 没装 LLVM 或没设 `$env:LIBCLANG_PATH = "C:\Program Files\LLVM\bin"` |
| 编译 rm_interfaces 时 cl.exe 找不到 | 没在 x64 Native Tools 提示符里，或没先 `pixi shell` |
| 链接错误找不到 `rcl.lib` 等 | 没设 `$env:AMENT_PREFIX_PATH` |
| 代码里 `r2r::rm_interfaces::...` / `r2r::tf2_msgs::...` 类型不存在 | overlay 未激活、`IDL_PACKAGE_FILTER` 漏包，或消息变更后未 `cargo clean -p r2r_msg_gen` |
| 绑定生成极慢 | 没设 `IDL_PACKAGE_FILTER`，在给全部消息包生成绑定 |
| 运行 exe 报缺 DLL / 找不到 assets | 用 `cargo run` 启动（自动处理动态库与资产路径），或见坑位 7 与流程第 3 步的说明 |
| 运行一两分钟后 `memory allocation of 4665600 bytes failed`（0xc0000409 退出） | 局域网设备订阅了 `/image_raw`：1440×1080 RGB 一帧 ~4.7MB，110+FPS 全速发布约 500MB/s，慢链路消费不过来时 rmw/DDS 在仿真进程内无界堆积直到 OOM（FastDDS 与 CycloneDDS、reliable 与 best_effort 均复现）。解决：远端订阅用 `[ros2] publish_compressed = true`（改订 `/image_raw/compressed`，JPEG，~20-40MB/s），必要时调低 `[ros2] publish_hz`；本机高速消费方可用 raw（`publish_compressed = false`）。注意 `[ros2]` 段改动需重启生效（见下"日常开发备注"） |
| 远端 rqt_image_view 看不到图像 | 压缩模式下只发 `/image_raw/compressed`（`CompressedImage`），rqt_image_view 只认原始 `Image`。消费端转回 raw：`ros2 run image_transport republish compressed in:=/image_raw raw out:=/image_raw`，或用 RViz2 Image 显示并把 Transport Hint 设为 `compressed` |

---

## 六、日常开发备注

- 提交钩子（`.githooks/pre-commit`）会跑 `cargo fmt -- --check`，提交前先 `cargo fmt`。
- `config.toml` 运行期热更新（改完保存即生效，无需重启），但目前热生效的只有 physics 段的 `substep_count`/`fixed_hz`；`[ros2]` 段（`publish_hz`/`publish_compressed`）在插件构建时读入一次，改动需重启。
- 双特性切换构建后建议 `cargo clean -p r2r_msg_gen`，避免缓存的环境哈希错配。
- `cargo-wrapper.sh` / `build.sh` / `env.sh` 是作者本地（Linux/zsh）的便捷脚本，Windows 流程不使用。
