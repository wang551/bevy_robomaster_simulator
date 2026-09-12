# Daedalus ROS2 自瞄开发指南

本指南面向想在 **Daedalus 模拟器**上开发 ROS2 自瞄（auto-aim）算法的开发者，覆盖环境搭建、话题接口、坐标系约定、控制协议、弹道模型，以及可直接编译的 Python / C++ 骨架模板。

> **目标环境**：Ubuntu + 原生 ROS2（Humble / Jazzy / Lyrical 均可，`ROS_DISTRO` 需与所 source 的发行版一致）。Windows 构建的完整流程见 [build.md](build.md)，本文不展开。
>
> 代码事实以 master 分支为准（含 `5b2b777` 位姿修复）。接口有变动时，权威来源是 `src/ros2/topic.rs`（话题表）与 `src/ros2/plugin.rs`（TF 树与指令处理）。

---

## 目录

1. [总览与闭环架构](#1-总览与闭环架构)
2. [环境准备（Linux）](#2-环境准备linux)
3. [话题接口参考](#3-话题接口参考)
4. [图像与相机模型](#4-图像与相机模型)
5. [坐标系与 TF 树](#5-坐标系与-tf-树)
6. [GimbalCmd 控制协议](#6-gimbalcmd-控制协议)
7. [云台闭环与弹道模型](#7-云台闭环与弹道模型)
8. [场景与目标](#8-场景与目标)
9. [示例骨架模板](#9-示例骨架模板)
10. [验证与调试](#10-验证与调试)
11. [故障排查速查](#11-故障排查速查)
12. [附录](#12-附录)

---

## 1. 总览与闭环架构

```
        ┌────────────────────────────────────────────────────┐
        │                  Daedalus 仿真器                   │
        │                                                    │
        │  离屏捕获相机 ──┬── /image_raw（RGB8）             │
        │  1440x1080     │   或 /image_raw/compressed（JPEG）│
        │                └── /camera_info（内参，无畸变）    │
        │  每渲染帧整树真值 ─── /tf                          │
        │  装甲板/云台/枪口/相机位姿 ── /tf + 4 个 pose 话题 │
        │  深度 → /livox/lidar（可选点云）                   │
        └──────────────┬────────────────────▲────────────────┘
                       │ 传感器流           │ /rm_gimbal/cmd
                       ▼                    │ （GimbalCmd，BestEffort）
            ┌───────────────────────────────────────┐
            │              你的自瞄节点             │
            │  目标检测（视觉）→ 位姿解算 → 弹道预测│
            │  → 云台角指令 → 开火建议              │
            └───────────────────────────────────────┘
                       │
                       ▼  仿真器内闭环（无需你实现）
        指令 → 双轴 PID 跟踪 → 云台转动 → fire_advice 开火
             → 弹丸物理弹道 → 装甲板命中判定 → 命中统计
```

三个关键认知，先建立再动手：

1. **仿真器只消费一种消息：`rm_interfaces/GimbalCmd`**（话题 `/rm_gimbal/cmd`）。你的节点是发布方，仿真器是订阅方。
2. **仿真器不发布任何"检测结果"**。`rm_interfaces` 里的 `Armors`、`Target`、`RuneTarget` 等消息是为团队既有流水线（rm_vision 风格）预留的兼容接口，仿真器本身不发布它们。**目标真值一律走 `/tf`**——每块装甲板、能量机关扇叶、云台、相机都有独立 frame，可用于：
   - 在写视觉检测**之前**，先真值调试解算器、控制器、弹道补偿（推荐的开发路径）；
   - 写视觉检测**之后**，用它做检测/位姿结果的定量对拍。
3. **指令是"绝对世界方向"**：`yaw`/`pitch` 描述的是**弹道应指向的世界系方向**（odom 系，ROS 轴向），不是相对当前云台的增量；仿真器内部用 PID 闭环驱动云台跟踪，你不需要做云台速度控制。

---

## 2. 环境准备（Linux）

### 2.1 依赖

| 组件 | 说明 |
|---|---|
| ROS2（含 colcon） | 推荐桌面完整安装；colcon 缺失时 `sudo apt install python3-colcon-common-extensions` |
| Rust ≥ 1.95 | 用 [rustup](https://rustup.rs) 安装维护（bevy 0.19 的硬性要求；发行版自带 rustc 通常过旧） |
| `libclang-dev` | r2r 在编译期用 bindgen 解析 ROS2 C 头文件必需（运行期不需要） |
| 桌面环境 / GPU 驱动 | 仿真器是图形程序，需要 OpenGL/Vulkan；WSL2 下 wgpu 适配已内置，但推荐原生桌面 |

### 2.2 构建 rm_interfaces overlay（一次性）

`rm_interfaces/` 是仓库内的 ament 包，仿真器编译期要从 overlay 里读取消息定义：

```sh
cd /path/to/bevy_robomaster_simulator
colcon build --packages-select rm_interfaces --cmake-args -DCMAKE_BUILD_TYPE=Release -DBUILD_TESTING=OFF
source install/setup.bash
```

> 每次修改 `.msg`/`.srv` 定义后需要重跑 colcon，并执行 `cargo clean -p r2r_msg_gen` 强制 Rust 侧重新生成绑定。升级/更换 ROS2 环境后同样要重做（否则运行时可能报 `RCL_RET_ERROR`——编译能过，但 ABI 已不匹配）。

### 2.3 构建并运行仿真器（ros2 特性）

```sh
source /opt/ros/<distro>/setup.bash          # 同时会设置 ROS_DISTRO
cd /path/to/bevy_robomaster_simulator
export IDL_PACKAGE_FILTER=rm_interfaces;builtin_interfaces;geometry_msgs;sensor_msgs;std_msgs;visualization_msgs;tf2_msgs
cargo run --no-default-features --features ros2 --release
```

要点：

- 必须用 `cargo run` 启动（而不是直接执行二进制）：它会把 `assets/` 资源路径和动态库路径注入环境（bevy dynamic_linking）。
- `config.toml` 从**当前工作目录**读取，所以要保持在仓库根目录下启动。
- 默认特性是 `talos`（共享内存通道）。ros2 采集激活时 Talos 插件自动跳过，二者不冲突；特殊情况下可用 `DAEDALUS_FORCE_TALOS_CAPTURE=1` 强制并存。
- 首次编译较久（依赖全量构建），之后增量。

### 2.4 首次自检

仿真器窗口出现、进入场景后：

1. 按 **F5** 打开自瞄订阅（窗口内有效，开/关有提示；不开时 `/rm_gimbal/cmd` 的消息会被忽略）。
2. 另开终端（已 source 同一 ROS 环境 + `install/setup.bash`）：

```sh
ros2 topic list
# 应看到 /camera_info /image_raw /tf /gimbal_pose ... /rm_gimbal/cmd

ros2 topic hz /image_raw                      # 默认约等于 [ros2] publish_hz = 60
ros2 topic echo /gimbal_pose --field pose     # 手动转动云台（方向键）观察变化
```

自瞄节点工程（colcon 工作区）与仿真器 overlay 的关系见 [9.3 构建与运行](#93-构建与运行)。

---

## 3. 话题接口参考

话题定义集中在 `src/ros2/topic.rs`（146–167 行），全部发布者使用 r2r 默认 QoS（**Reliable / KeepLast / depth 10 / Volatile**），除特别注明外。

### 3.1 仿真器发布

| 话题 | 类型 | 频率 | 说明 |
|---|---|---|---|
| `/camera_info` | `sensor_msgs/CameraInfo` | `publish_hz`（默认 60Hz） | 与图像同拍发布，`frame_id = camera_optical_frame`，无畸变，内参见第 4 节 |
| `/image_raw` | `sensor_msgs/Image` | `publish_hz` | RGB8，1440×1080（`[capture.color]` 可改分辨率，重启生效） |
| `/image_raw/compressed` | `sensor_msgs/CompressedImage` | `publish_hz` | JPEG。与 `/image_raw` **二选一**，由 `[ros2] publish_compressed` 决定（重启生效）；命名遵循 image_transport 约定 |
| `/tf` | `tf2_msgs/TFMessage` | **每个渲染帧**（不节流，可达 100+ Hz） | 整棵 TF 树打包在一条消息里，见第 5 节 |
| `/gimbal_pose` | `geometry_msgs/PoseStamped` | 每渲染帧 | `gimbal_link` 相对父 frame `odom` 的位姿 |
| `/odom_pose` | `geometry_msgs/PoseStamped` | 每渲染帧 | `odom` 相对父 frame `map` 的位姿 |
| `/muzzle_pose` | `geometry_msgs/PoseStamped` | 每渲染帧 | `muzzle_link` 相对父 frame `muzzle` |
| `/camera_pose` | `geometry_msgs/PoseStamped` | 每渲染帧 | `camera_link` 相对父 frame `gimbal_link` |
| `/livox/lidar` | `sensor_msgs/PointCloud2` | `[livox_ros] publish_freq`（默认 10Hz） | 由深度图模拟的 Livox 风格点云；`[livox_ros] enabled = true` 时才发布 |
| `/simulator/marker` | `visualization_msgs/Marker` | 每渲染帧 | 每块装甲板一个 CUBE（ns `armors`，寿命 0.3s，frame `map`；当前 `alpha = 0.0`，RViz 默认渲染下不可见） |
| `/simulator/tech_core/state` | `std_msgs/String` | 20Hz | 科技核心状态的 JSON（`{stamp, cores:[...]}`） |

> 4 个 pose 话题是 `/tf` 对应变换的便捷别名：自 `5b2b777` 起，四元数与 `/tf` 中对应父子变换**逐位一致**，`header.frame_id` 为父 frame。用 TF2 listener 的可以只订阅 `/tf`。

### 3.2 仿真器订阅

| 话题 | 类型 | QoS | 说明 |
|---|---|---|---|
| `/rm_gimbal/cmd` | `rm_interfaces/msg/GimbalCmd` | `sensor_data()`（**BestEffort** / KeepLast 5 / Volatile） | 唯一控制入口，语义见第 6 节 |

### 3.3 带宽与 OOM 警告（重要）

1440×1080 RGB8 一帧约 4.7MB，不压缩全速（100+ FPS）可达 **~500MB/s** 的 DDS 流量。发布端通道有界（16 槽、丢新保旧），但**订阅方消费不及时时，DDS 会在仿真器进程内堆积未发送数据直到内存耗尽崩溃**。

- 本机开发：默认配置（raw, 60Hz）没问题。
- 跨机 / WiFi / 慢链路：在 `config.toml` 里设置 `publish_compressed = true`（JPEG，约 20–40MB/s），必要时配合 `publish_hz = 30` 降低帧率，然后**重启仿真器**。
- 只认 raw 的工具（如 rqt_image_view）在消费端转回：

  ```sh
  ros2 run image_transport republish compressed in:=/image_raw raw out:=/image_raw
  ```

---

## 4. 图像与相机模型

### 4.1 基本参数（默认值，`config.toml` 可调）

| 参数 | 值 | 来源 |
|---|---|---|
| 分辨率 | 1440×1080（4:3） | `[capture.color] width/height`（重启生效） |
| 像素格式 | RGB8（`step = width*3`，小端） | 固定 |
| 视场角 | 垂直 FOV = 45° | `[camera] fov`（重启生效） |
| 畸变 | 无（`distortion_model = plumb_bob`，`d = [0,0,0,0,0]`） | 固定 |
| 时间戳 | **系统墙钟**（ROS `Clock` SystemTime），非仿真时间 | 固定 |

图像与 `/tf`、各 pose 话题使用同一个时钟源，可直接用 `message_filters` 做时间同步。

### 4.2 内参

`/camera_info` 每拍图像都会发布，**直接用它**，不要自己假设。公式（`src/capture.rs` `compute_camera_intrinsics`）如下，便于离线推导：

```
fov_x = 2·atan(tan(fov_y/2) · width/height)
fx = width  / (2·tan(fov_x/2))
fy = height / (2·tan(fov_y/2))
cx = width/2,  cy = height/2
```

默认配置下：**fx = fy ≈ 1303.6 px，cx = 720，cy = 540，水平 FOV ≈ 57.9°**。改了分辨率或 `[camera] fov` 后按公式重算或直接读 `/camera_info`。

### 4.3 与视觉库对接

- `cv_bridge`：发布的是 `rgb8`，`imgmsg_to_cv2(msg, "bgr8")` 会自动转换；OpenCV 默认 BGR，别漏了通道序。
- `/camera_info` 的 `frame_id` 是 `camera_optical_frame`，与 `/tf` 中的光学系 frame 对应——PnP 得到的装甲板位姿（光学系）可以直接和 `camera_optical_frame → armor_N` 的 TF 真值对拍。
- 深度数据**没有**独立的深度图话题：深度图只在内部用于合成 `/livox/lidar` 点云（frame `camera_link`，Livox 坐标惯例，`[capture.depth]` 1280×720，near 0.1 / far 80 m）。

---

## 5. 坐标系与 TF 树

### 5.1 轴向约定

仿真器内部是 Bevy 坐标系（Y 轴向上），发布 ROS 话题时经过固定矩阵对齐为 **ROS 惯例（REP-103）：X 向前、Y 向左、Z 向上，单位米/弧度，四元数为 x y z w**。正 yaw 绕 +Z（俯视逆时针），姿态/位置全部是世界系量。

### 5.2 TF 树（`/tf`，每渲染帧整树发布）

```
map                                    # 世界/场地固定系
├── odom                               # 平移 = 云台世界位置，旋转 = 单位（机器人世界位置）
│   └── gimbal_link                    # 云台；X 轴沿枪管方向（含安装旋转与 90° 修正）
│       ├── muzzle → muzzle_link       # 枪口（弹丸出发点）
│       └── camera_link                # 相机外参（由 vehicle.glb 的 CAM_DIRECTION 节点换算）
│           └── camera_optical_frame   # 标准光学系（z 前 x 右 y 下），PnP 用
├── power_rune_small / power_rune_large            # 每面能量机关的基座
│   └── power_rune_{small|large}_{0..4}            # 当前激活的扇叶（仅激活期间存在）
└── armor_0 … armor_N                  # 所有装甲板（世界系位姿，随机器人运动实时更新）
```

### 5.3 `armor_N` 命名的重要坑位

`armor_N` 的 `N` 是**仿真器内部的生成顺序计数器**（`AtomicUsize` 递增），**不是 RM 规则的机器人编号**（1英雄/2工程/3、4、5步兵/6前哨/7基地），且所有机器人（含双方、前哨站）的装甲板共享同一套编号。哪些 N 属于哪个机器人取决于场景加载顺序，**不要硬编码**。定位目标身份的实用办法：

- 打印各 `armor_N` 在 `map` 系的位置，对照 [第 8 节](#8-场景与目标) 的场景布局；
- 让假人动起来（`I J K L`）或旋转底盘（`U`），同一辆车的 4 块装甲板绕同一轴心运动；
- 结合目标的运动特征（前哨站固定点旋转、能量机关扇叶仅激活期出现）。

另外注意：`armor_N` 取的是装甲板平面 marker 节点的位姿，该节点自带约 180° 的局部旋转（有意为之，见 build.md 坑位 8），相对旧实现朝向有翻转。涉及装甲板法线/朝向的解算，先用一帧 TF 实测确认法线方向再用，不要凭直觉硬编码。

### 5.4 动态外参，不要硬编码

相机相对云台的外参（`camera_link`）与枪口偏移（`muzzle`）由 `vehicle.glb` 模型节点定义、运行时换算，且 README「近期计划」中自定义外参在路线图上。**正确姿势是从 `/tf` 查 `camera_optical_frame → odom` 等变换参与解算**，而不是把偏移量写死在代码里。

---

## 6. GimbalCmd 控制协议

### 6.1 消息定义（`rm_interfaces/msg/GimbalCmd.msg`）

```
std_msgs/Header header     # 仿真器不校验，可为空
float64 pitch              # 度。从竖直轴起算：90 = 水平，>90 抬头，<90 低头
float64 yaw                # 度。odom 系绕 +Z，正方向俯视逆时针（向左）
float64 yaw_diff           # 信息字段，仿真器只打日志，可为 0
float64 pitch_diff         # 信息字段，同上
float64 distance           # 目标距离（米）；**-1.0 = 放弃目标**
bool   fire_advice         # true 触发开火一次（仿真器限频 10 发/秒）
```

### 6.2 语义细节（每条都会坑人，逐条看）

| 项 | 约定 |
|---|---|
| **QoS** | 仿真器订阅端是 **BestEffort**。DDS 匹配规则要求发布端提供的可靠性 ≥ 订阅端请求，**Reliable 发布者匹配不上 BestEffort 订阅者**——你的 publisher 必须也是 BestEffort（`rclcpp::SensorDataQoS()` / `ReliabilityPolicy.BEST_EFFORT`），否则发了也收不到且不报错 |
| 单位与方向 | `yaw`/`pitch` 单位是**度**不是弧度；`pitch = 90° + 仰角`（水平 90、抬头 120 = 仰角 30°、竖直向上 180） |
| 指令性质 | 绝对方向（odom 系世界方向），不是增量。仿真器内部 PID（`[vehicle.gimbal_pid]`）驱动云台跟踪，底盘平移/旋转不影响误差计算 |
| 无解 | `distance = -1.0` → 仿真器移除跟踪目标、PID 停止驱动。**注意消息其他字段默认值 0 会被当作有效指令**（yaw=0/pitch=0 是指向地面的合法方向），无解时务必显式发 `distance=-1.0`，不要发全零消息 |
| 开火 | `fire_advice = true` 触发一次发射，仿真器限频 **10Hz**（超频的 fire 被静默丢弃）。F5 开启后 Space 手动开火仍可用，但外部 fire_advice 与之独立 |
| 消息频率 | 建议 ≥ 50Hz 持续发布（PID 按消息连续 retarget）。长时间不发消息 = 云台停在最后一个目标上 |
| **F5 总开关** | 仿真器窗口内按 **F5** 才开始消费 `/rm_gimbal/cmd`；关闭时消息被静默忽略（这也是"发了没反应"的第一大原因）。F5 开启期间方向键手动云台被禁用，`WASD` 底盘仍可手动开 |
| 联调日志 | 仿真器以 2Hz 打印收到的指令：`[ROS2] GimbalCmd yaw=.. pitch=.. yaw_diff=.. pitch_diff=.. distance=.. fire=..`，收发字段逐值比对、验证符号约定都靠它 |

### 6.3 手动冒烟测试

不用写代码就能验证整条链路（先按 F5）：

```sh
source /opt/ros/<distro>/setup.bash && source <simulator>/install/setup.bash

# 云台向左转 30°，保持水平；distance=-1 会丢弃目标，这里给一个正距离让 PID 跟踪
ros2 topic pub -r 50 /rm_gimbal/cmd rm_interfaces/msg/GimbalCmd \
  "{yaw: 30.0, pitch: 90.0, distance: 3.0}" --qos-reliability best_effort
```

仿真器终端应出现 2Hz 日志，云台平滑转到指令方向；松开（Ctrl+C）云台停住。改 `pitch: 105.0` 观察抬头 15°，可确认俯仰方向。

### 6.4 求解器输出公式速查

设目标相对枪口的方向向量 `d = (dx, dy, dz)`（odom 系，米；用 `/tf` 查 `odom → armor_N` 或 PnP 结果变换到 odom 系）：

```text
yaw   = atan2(dy, dx) × 180/π
pitch = 90 + asin(dz / |d|) × 180/π        # 再加弹道补偿，见第 7.3 节
```

---

## 7. 云台闭环与弹道模型

### 7.1 云台闭环（仿真器内实现，你只需理解行为）

- 双轴独立 PID（yaw / pitch 各一组），参数在 `config.toml` 的 `[vehicle.gimbal_pid.yaw|pitch]`（默认 kp=50, ki=0, kd=0.1，含积分限幅与速率饱和），带抗积分饱和，连续消息间保持环路状态。
- pitch 机械限位 **±45°**（`[vehicle] gimbal_pitch_limit`）；指令超限会被 PID 输出自然钳住。
- 调大 `kp`/`max_rate` 可以让跟踪更"硬"，用于模拟不同强度的云台电机；`ki` 非零可消除稳态误差。

### 7.2 弹丸模型（`[projectile]`）

| 参数 | 默认值 | 说明 |
|---|---|---|
| 初速 `speed` | 25 m/s | 出膛速度，沿枪口方向 |
| 重力 | 9.81 m/s² | 恒定，方向 -Z |
| 弹径 / 质量 | 17mm / 3.2 g | 球形碰撞体 |
| 气动阻力 | **默认关闭** | `[projectile.aerodynamics] enabled = false`；开启后为二次阻力（ρ、Cd、风矢量可配） |
| 寿命 | 5 s | 之后消失 |
| 命中判定 | RM 规则入射角门控 | 下边缘 105°/上边缘 120°/左右 145°，背面来弹不计数；阈值 `[armor]` 段可热重载 |

### 7.3 弹道补偿（你的求解器要做的事）

平抛近似（v=25, g=9.81）下，水平距离 d 处的弹丸下坠量：

```text
t    = d / 25            # 飞行时间（秒）
drop = 0.5 · 9.81 · t²   # 下坠量（米）
```

对 5m 目标：t = 0.2s，drop ≈ 0.196m，对应需要**抬高**的俯仰修正 ≈ `degrees(atan(drop/d))` ≈ 2.25°。把 `pitch` 在几何瞄准角上加这一修正即可。当前配置无空气阻力，无需风项；若你打开了 `[projectile.aerodynamics]`，按二次阻力自行扩展。

---

## 8. 场景与目标

默认场景（`src/scene.rs`）加载顺序与布局（世界系，米）：

| 实体 | 位置 | 队伍 | 说明 |
|---|---|---|---|
| 玩家步兵（你控制的） | (0, 1, 0) | **红** | `Controlled`；相机/云台指令作用于它 |
| 假人步兵 | (1, 1, 1) | 蓝 | `Tab` 切换控制权到假人 |
| 假人英雄 | (2, 1, 1) | 蓝 | 大装甲板配置，会主动攻击 |
| 前哨站 ×2 | 场地固定点 | 红/蓝各一 | **绕轴持续旋转 0.8π rad/s**——打前哨要预测相位 |
| 能量机关 | 场地固定点 | 双面各一 | 小/大机关两扇面，各 5 个扇叶；`power_rune_*` frame 仅在激活期出现 |
| 标定板 CALIB.glb ×2 | (1, 2.5, 1)、(2, 0.5, 2) | — | 高对比标定图案，可用于手眼标定/内参验证 |
| 场地 GROUND.glb | 原点 | — | 含飞镖发射方向标记 |

控制键速查（完整表见 README / 附录 12.3）：

- **己方**：`WASD` 移动，方向键云台，`Space` 开火，`G` 飞镖，`Q` 小陀螺，`Shift` 加速
- **假人**：`I J K L` 移动，`U` 小陀螺，`C/B` 偏航，`F/V` 俯仰；`Tab` 切换控制对象
- **视角/功能**：`F3` 视角（自由/第一/第三），`F5` 自瞄订阅开关，`F2` 截图

对自瞄开发最有用的组合拳：**F3 切自由视角**观察战场全貌，用假人键把目标"喂"到不同方位/姿态，验证你的解算与预测。

---

## 9. 示例骨架模板

两套等价的骨架：**Python（rclpy，单文件）** 用于快速验证，**C++（ament_cmake 包）** 用于向真实工程演进。均为**骨架**——话题接线、TF、QoS、约定全部就位且可直接编译运行；算法部分（检测/解算）留 TODO，按注释填入即可。骨架默认持续发布 `distance=-1.0`（无解），对仿真器无副作用。

两个模板共同的行为模型：

```
on_image   : /image_raw 回调 —— TODO 视觉检测 → 得到目标方向/位姿
on_control : 100Hz 定时 —— 取目标 → 解算 yaw/pitch/distance/fire → 发布 /rm_gimbal/cmd
             目标丢失/无解 → distance = -1.0
```

### 9.1 Python（rclpy，单文件）

保存为 `autoaim_template.py`：

```python
#!/usr/bin/env python3
# -*- coding: utf-8 -*-
"""Daedalus ROS2 自瞄骨架模板（rclpy）。

接口约定（详见 docs/ros2-autoaim.md）：
  订阅 /image_raw、/camera_info   1440x1080 RGB8 图像 + 内参（BestEffort 即可）
  订阅 /tf                       装甲板等真值位姿（视觉检测前的调试替代）
  发布 /rm_gimbal/cmd            GimbalCmd；必须 BestEffort（仿真器订阅端是 sensor_data QoS）

运行前：
  source /opt/ros/<distro>/setup.bash
  source <本仓库>/install/setup.bash    # 提供 rm_interfaces
  python3 autoaim_template.py
  （另：仿真器窗口内按 F5 开启自瞄订阅）
"""
import math

import rclpy
from rclpy.node import Node
from rclpy.qos import QoSProfile, ReliabilityPolicy, HistoryPolicy
from sensor_msgs.msg import CameraInfo, Image
from tf2_ros import Buffer, TransformListener

from rm_interfaces.msg import GimbalCmd

ARMOR_FRAME = "armor_0"    # TODO: 目标装甲板 frame（armor_N 是生成顺序编号，见文档 5.3）
CONTROL_HZ = 100.0         # 控制频率，建议不低于图像帧率


def sensor_qos(depth: int = 5) -> QoSProfile:
    # 关键：发布端必须 BestEffort，Reliable 匹配不上仿真器的 BestEffort 订阅
    return QoSProfile(
        reliability=ReliabilityPolicy.BEST_EFFORT,
        history=HistoryPolicy.KEEP_LAST,
        depth=depth,
    )


class AutoAimTemplate(Node):
    def __init__(self) -> None:
        super().__init__("autoaim_template")

        # TF：真值调试与外参查询都走这里（配合 /camera_info 的 frame_id 使用）
        self.tf_buffer = Buffer()
        self.tf_listener = TransformListener(self.tf_buffer, self)

        self.camera_info: CameraInfo | None = None
        qos = sensor_qos()
        self.create_subscription(Image, "/image_raw", self.on_image, qos)
        self.create_subscription(CameraInfo, "/camera_info", self.on_camera_info, qos)
        self.cmd_pub = self.create_publisher(GimbalCmd, "/rm_gimbal/cmd", qos)
        self.create_timer(1.0 / CONTROL_HZ, self.on_control)
        self.get_logger().info("autoaim template started")

    def on_camera_info(self, msg: CameraInfo) -> None:
        # 内参（无畸变）：fx=msg.k[0], fy=msg.k[4], cx=msg.k[2], cy=msg.k[5]
        self.camera_info = msg

    def on_image(self, msg: Image) -> None:
        # TODO(视觉)：装甲板检测 -> PnP 解算位姿（内参用 self.camera_info）
        #   from cv_bridge import CvBridge
        #   frame = CvBridge().imgmsg_to_cv2(msg, "bgr8")   # 话题是 rgb8，按需转
        #   PnP 得到 camera_optical_frame 系位姿后，可用 TF 链换算到 odom 系再解算
        #
        # 视觉就绪前的真值调试替代（查 odom -> armor_N 的世界位姿）：
        #   from rclpy.time import Time
        #   tf = self.tf_buffer.lookup_transform("odom", ARMOR_FRAME, Time())
        pass

    def on_control(self) -> None:
        # TODO(解算)：在此填入你的解算器。参考（odom 系目标方向 d=(dx,dy,dz)）：
        #   yaw   = math.degrees(math.atan2(dy, dx))
        #   pitch = 90.0 + math.degrees(math.asin(dz / dist))
        #   # 弹道补偿：pitch += degrees(atan(0.5 * 9.81 * (dist/25)**2 / dist))
        #   # fire：角误差小于阈值时置 True（仿真器限频 10Hz）
        #
        # 骨架默认无解：distance=-1.0 让仿真器停止跟踪（不要发全零消息！）
        self.publish_cmd(yaw=0.0, pitch=90.0, distance=-1.0, fire_advice=False)

    def publish_cmd(self, yaw: float, pitch: float, distance: float,
                    fire_advice: bool = False) -> None:
        msg = GimbalCmd()
        msg.header.stamp = self.get_clock().now().to_msg()
        msg.yaw = float(yaw)              # 度，绕 +Z 俯视逆时针
        msg.pitch = float(pitch)          # 度，90=水平，>90 抬头
        msg.distance = float(distance)    # 米；-1.0 = 放弃目标
        msg.fire_advice = bool(fire_advice)
        self.cmd_pub.publish(msg)


def main() -> None:
    rclpy.init()
    node = AutoAimTemplate()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == "__main__":
    main()
```

### 9.2 C++（ament_cmake 包）

包结构：

```
rm_autoaim_template/
├── CMakeLists.txt
├── package.xml
└── src/
    └── autoaim_node.cpp
```

`package.xml`：

```xml
<?xml version="1.0"?>
<package format="3">
  <name>rm_autoaim_template</name>
  <version>0.0.1</version>
  <description>Daedalus ROS2 auto-aim skeleton template</description>
  <maintainer email="you@example.com">you</maintainer>
  <license>AGPL-3.0</license>

  <buildtool_depend>ament_cmake</buildtool_depend>
  <depend>rclcpp</depend>
  <depend>sensor_msgs</depend>
  <depend>tf2</depend>
  <depend>tf2_ros</depend>
  <depend>rm_interfaces</depend>
</package>
```

`CMakeLists.txt`：

```cmake
cmake_minimum_required(VERSION 3.16)
project(rm_autoaim_template)

set(CMAKE_CXX_STANDARD 17)
set(CMAKE_CXX_STANDARD_REQUIRED ON)

find_package(ament_cmake REQUIRED)
find_package(rclcpp REQUIRED)
find_package(sensor_msgs REQUIRED)
find_package(tf2 REQUIRED)
find_package(tf2_ros REQUIRED)
find_package(rm_interfaces REQUIRED)

add_executable(autoaim_node src/autoaim_node.cpp)
ament_target_dependencies(autoaim_node
  rclcpp sensor_msgs tf2 tf2_ros rm_interfaces)

install(TARGETS autoaim_node
  DESTINATION lib/${PROJECT_NAME})

ament_package()
```

`src/autoaim_node.cpp`：

```cpp
#include <chrono>
#include <memory>

#include <rclcpp/rclcpp.hpp>
#include <sensor_msgs/msg/camera_info.hpp>
#include <sensor_msgs/msg/image.hpp>
#include <tf2_ros/buffer.hpp>
#include <tf2_ros/transform_listener.hpp>

#include <rm_interfaces/msg/gimbal_cmd.hpp>

using namespace std::chrono_literals;

namespace rm_autoaim_template {

class AutoAimNode : public rclcpp::Node {
 public:
  AutoAimNode() : Node("autoaim_template") {
    // TF：真值调试与外参查询都走这里
    tf_buffer_ = std::make_unique<tf2_ros::Buffer>(get_clock());
    tf_listener_ = std::make_shared<tf2_ros::TransformListener>(*tf_buffer_);

    // 关键：SensorDataQoS = BestEffort，Reliable 匹配不上仿真器的 BestEffort 订阅
    const auto qos = rclcpp::SensorDataQoS();
    camera_info_sub_ = create_subscription<sensor_msgs::msg::CameraInfo>(
        "/camera_info", qos, [this](sensor_msgs::msg::CameraInfo::ConstSharedPtr msg) {
          camera_info_ = std::move(msg);  // 内参：k[0]=fx k[4]=fy k[2]=cx k[5]=cy，无畸变
        });
    image_sub_ = create_subscription<sensor_msgs::msg::Image>(
        "/image_raw", qos, [this](sensor_msgs::msg::Image::ConstSharedPtr msg) { OnImage(msg); });
    cmd_pub_ = create_publisher<rm_interfaces::msg::GimbalCmd>("/rm_gimbal/cmd", qos);
    control_timer_ = create_wall_timer(10ms, [this] { OnControl(); });  // 100 Hz
    RCLCPP_INFO(get_logger(), "autoaim template started");
  }

 private:
  void OnImage(sensor_msgs::msg::Image::ConstSharedPtr msg) {
    // TODO(视觉)：装甲板检测 -> PnP 解算位姿（内参用 camera_info_，无畸变）
    //   cv_bridge::toCvCopy(msg, "bgr8")   // 话题是 rgb8，按需转
    // 视觉就绪前的真值调试替代（查 odom -> armor_N 的世界位姿）：
    //   tf_buffer_->lookupTransform("odom", "armor_0", tf2::TimePointZero);
    (void)msg;
  }

  void OnControl() {
    // TODO(解算)：在此填入你的解算器。参考（odom 系目标方向 d=(dx,dy,dz)）：
    //   yaw   = atan2(dy, dx) * 180 / M_PI;
    //   pitch = 90 + asin(dz / dist) * 180 / M_PI;
    //   // 弹道补偿：pitch += atan(0.5 * 9.81 * pow(dist / 25, 2) / dist) * 180 / M_PI;
    //   // fire：角误差小于阈值时置 true（仿真器限频 10Hz）
    //
    // 骨架默认无解：distance=-1 让仿真器停止跟踪（不要发全零消息！）
    PublishCmd(/*yaw=*/0.0, /*pitch=*/90.0, /*distance=*/-1.0);
  }

  void PublishCmd(double yaw, double pitch, double distance, bool fire_advice = false) {
    rm_interfaces::msg::GimbalCmd cmd;
    cmd.header.stamp = now();
    cmd.yaw = yaw;                // 度，绕 +Z 俯视逆时针
    cmd.pitch = pitch;            // 度，90=水平，>90 抬头
    cmd.distance = distance;      // 米；-1.0 = 放弃目标
    cmd.fire_advice = fire_advice;
    cmd_pub_->publish(cmd);
  }

  std::unique_ptr<tf2_ros::Buffer> tf_buffer_;
  std::shared_ptr<tf2_ros::TransformListener> tf_listener_;
  rclcpp::Subscription<sensor_msgs::msg::CameraInfo>::SharedPtr camera_info_sub_;
  rclcpp::Subscription<sensor_msgs::msg::Image>::SharedPtr image_sub_;
  rclcpp::Publisher<rm_interfaces::msg::GimbalCmd>::SharedPtr cmd_pub_;
  rclcpp::TimerBase::SharedPtr control_timer_;
  sensor_msgs::msg::CameraInfo::ConstSharedPtr camera_info_;
};

}  // namespace rm_autoaim_template

int main(int argc, char** argv) {
  rclcpp::init(argc, argv);
  rclcpp::spin(std::make_shared<rm_autoaim_template::AutoAimNode>());
  rclcpp::shutdown();
  return 0;
}
```

### 9.3 构建与运行

自瞄工程是独立的 colcon 工作区，通过 source 仿真器的 overlay 获得 `rm_interfaces`：

```sh
# 1. 组建工作区
mkdir -p ~/rm_ws/src
cp -r rm_autoaim_template ~/rm_ws/src/          # C++ 模板；Python 模板任意位置均可

# 2. 构建（C++ 模板）
source /opt/ros/<distro>/setup.bash
source <simulator>/install/setup.bash           # 仿真器 overlay：提供 rm_interfaces
cd ~/rm_ws && colcon build && source install/setup.bash

# 3. 运行（保持上面的 source 状态）
ros2 run rm_autoaim_template autoaim_node       # C++
python3 autoaim_template.py                     # Python

# 4. 仿真器窗口内按 F5，开始闭环
```

联调顺序建议（自底向上，每步都有明确观测点）：

1. **链路验证**：骨架直接跑 + F5 → 仿真器终端出现 2Hz `GimbalCmd` 日志。
2. **解算验证（真值）**：在 `on_control` 里查 `/tf` 取 `armor_N` 位姿，按 6.4 公式解算 → 云台应持续咬住选定装甲板；切换假人/旋转底盘考验跟踪。
3. **弹道验证**：解算加 7.3 补偿 + `fire_advice` → 观察命中率。
4. **视觉接入**：`on_image` 里实现检测/PnP，与 `/tf` 真值对拍误差，再替换解算输入。

---

## 10. 验证与调试

### 10.1 命令行

```sh
ros2 topic list                                   # 话题是否齐全
ros2 topic hz /image_raw                          # 图像流频率（应 ≈ publish_hz）
ros2 topic echo /image_raw --field header \       # 只看时间戳，别整帧 echo（4.7MB/帧）
    --qos-reliability best_effort
ros2 topic echo /gimbal_pose --field pose         # 云台位姿（与 /tf 逐位一致）
ros2 topic echo /rm_gimbal/cmd ...                # 注意：echo 你自己发布的指令无意义，
                                                  # 用仿真器 2Hz 日志验证"收到"
```

### 10.2 RViz2

Fixed Frame 设为 **`map`**：

- **TF** 显示：整棵树，直观检查 armor_N / power_rune / 云台位姿；
- **Marker**（`/simulator/marker`）：装甲板占位块（注意当前 alpha=0，默认渲染不可见）；
- **PointCloud2**（`/livox/lidar`）：模拟雷达点云；
- **Image**（`/image_raw`）：compressed 模式下 Transport Hint 选 `compressed`。

> Foxglove 用户：master 分支暂无内置桥接；`feature/foxglove-ws-server` 分支内置了 ws://0.0.0.0:8765 的 Foxglove SDK 服务器（图像 + TF），合并进度见仓库。

### 10.3 仿真器内观测

- 2Hz 指令日志 `[ROS2] GimbalCmd yaw=.. pitch=.. ...` 是收发对拍的基准。
- 命中统计（命中率判定采用 RM 规则入射角门控，见 README「实用功能」）。
- `[debug] diagnostics = true` 时终端打印 FPS/帧时。

### 10.4 云台行为调参

云台跟不上/过冲时，调 `config.toml` 的 `[vehicle.gimbal_pid.yaw|pitch]`（kp/ki/kd/integral_limit/max_rate）与 `[vehicle] gimbal_rotation_speed`、`gimbal_pitch_limit`。这是**模拟真实云台特性**的旋钮——自瞄算法应该在"中等"参数下可用，而不是把 PID 调成理想舵机来掩盖算法缺陷。

---

## 11. 故障排查速查

| 症状 | 原因 / 解决 |
|---|---|
| 发了 `/rm_gimbal/cmd` 云台毫无反应 | ① 仿真器窗口没按 **F5**；② 发布端 QoS 是 Reliable（必须 BestEffort，见 6.2）；③ 消息里 `distance` 忘了给有效值（或全零消息语义错） |
| 云台动一下就停 / 行为像抽风 | 消息发得太稀（PID 需要连续 retarget）；或无解时没发 `distance=-1.0` 而是停发 |
| `ros2 topic list` 看不到话题 / 时有时无 | discovery 未完成或 daemon 状态异常：`ros2 daemon stop` 后重试 |
| echo `/image_raw` 卡死/巨量输出 | 一帧 4.7MB；用 `--field header` 只看头，或加 `--qos-reliability best_effort` |
| 仿真器运行约 1 分钟后内存暴涨崩溃 | 慢速订阅方导致 DDS 进程内堆积：`publish_compressed = true`、降 `publish_hz`、消费端 `image_transport republish`（详见 3.3） |
| rqt_image_view 黑屏（compressed 模式） | 该工具只认 raw：消费端 republish，或 RViz2 Image + Transport Hint `compressed` |
| 改了 `.msg` 后 Rust 编译还是旧定义 | colcon 重编 `rm_interfaces` 后执行 `cargo clean -p r2r_msg_gen` |
| 运行时启动即 panic `RCL_RET_ERROR`（GimbalCmd 订阅处） | ROS 环境升级导致 overlay 与 Rust 绑定 ABI 失配：重编 overlay + `cargo clean -p r2r_msg_gen` |
| `ros2 run` 找不到 `rm_interfaces` 类型 | 没 source 仿真器 overlay：`source <simulator>/install/setup.bash` |
| 云台方向和预期相反 | 先用 6.3 的冒烟命令标定你对符号约定的理解（yaw 正 = 俯视逆时针/向左；pitch 90 = 水平、>90 抬头），再检查解算公式 |
| WSL2 下黑屏/渲染异常 | 使用原生桌面；WSL 有内置 wgpu 适配但依赖宿主 GPU 驱动配置 |

---

## 12. 附录

### 12.1 `rm_interfaces` 消息一览

仿真器**只使用** `GimbalCmd`；其余为团队流水线兼容预留（仿真器不发布）。

| 消息/服务 | 字段摘要 | 用途（团队流水线约定） |
|---|---|---|
| `GimbalCmd` | `pitch, yaw, yaw_diff, pitch_diff, distance, fire_advice` | **仿真器唯一订阅**，见第 6 节 |
| `Armor` / `Armors` | `number, type, distance_to_image_center, pose` / 数组 | 检测器输出的装甲板列表 |
| `Target` | `tracking, id, armors_num, position, velocity, yaw, v_yaw, radius_1/2, d_za, d_zc, ...` | 跟踪器输出的整车目标模型 |
| `RuneTarget` | `pts[5], is_lost, is_big_rune` | 能量机关扇叶检测结果 |
| `ChassisCmd` | `is_spining, is_navigating, twist` | 底盘控制（仿真器当前不订阅） |
| `DebugLight(s)` / `DebugArmor(s)` / `DebugRuneAngle` | 调试字段 | 中间结果调试 |
| `SerialReceiveData` | `mode, bullet_speed, roll, yaw, pitch, judge_system_data` | 下位机串口数据模型 |
| `JudgeSystemData` / `OperatorCommand` | 血量/比赛时间/操作手指令 | 裁判系统模型 |
| `Measurement` / `PlanGimbalCmd` / `Point2d` | 规划/量测字段 | 规划器接口 |
| `srv/SetMode` | `mode`（0自瞄红/1自瞄蓝/2小符红/…）→ `success, message` | 模式切换服务约定 |
| `srv/RunHandEyeCalibration` | 空 → `success, yaml_path` | 手眼标定服务约定 |

### 12.2 `config.toml` 相关键位速查

| 键 | 默认 | 生效时机 |
|---|---|---|
| `[ros2] publish_hz` | 60.0（0=每帧） | **启动时**，重启生效 |
| `[ros2] publish_compressed` | false | **启动时**，重启生效 |
| `[capture.color] width/height` | 1440/1080 | 启动时（内参随之变化，读 `/camera_info`） |
| `[capture.depth] width/height/near/far` | 1280/720/0.1/80 | 启动时 |
| `[livox_ros] enabled/publish_freq/points_per_second/frame_id` | true/10/400000/camera_link | 启动时 |
| `[camera] fov` | 45.0（垂直 FOV，度） | 启动时 |
| `[vehicle] gimbal_pitch_limit` | 0.785 rad（≈45°） | 启动时 |
| `[vehicle.gimbal_pid.yaw|pitch]` kp/ki/kd/... | 50/0/0.1 | 每帧读取 |
| `[projectile] speed` 等弹丸参数 | 25 m/s 等 | 每帧读取 |
| `[projectile.aerodynamics] enabled` | false | 每帧读取 |
| `[armor] hit_angle_max` | 75 度（入射方向与装甲板法线夹角上限） | 热重载 |
| `[physics] fixed_hz / substep_count` | 120 / 8 | 热重载 |
| `[preview] enabled` / `[debug] egui/inspector/diagnostics` | true / false | 启动时 |

### 12.3 键位表

| 对象 | 功能 | 按键 |
|---|---|---|
| 己方步兵 | 移动 / 云台 / 开火 / 飞镖 | `W A S D` / `↑ ↓ ← →` / `Space` / `G` |
| | 小陀螺 / 加速 | `Q` / `Shift` |
| 假人 | 移动 / 小陀螺 | `I J K L` / `U` |
| | 云台偏航 / 俯仰 | `C` `B` / `F` `V` |
| 通用 | 切换控制假人 | `Tab` |
| | 视角（自由/第一/第三） | `F3` |
| | 自瞄订阅开关 | `F5` |
| | 截图 | `F2` |
| 自由视角 | 移动 / 环视 | `W A S D` + `N` `J` / 鼠标拖动 |

> 手柄完全等效支持（摇杆移动/瞄准、扳机开火等），详见屏幕帮助。

### 12.4 权威代码索引

改接口前先看这几处（行号为 master 当前状态）：

| 内容 | 位置 |
|---|---|
| 话题名称/类型/QoS 总表 | `src/ros2/topic.rs:146-167` |
| TF 树构建 + pose 话题 | `src/ros2/plugin.rs:126-294` |
| GimbalCmd 消费逻辑 | `src/ros2/plugin.rs:296-349` |
| 指令角度约定（pitch 从竖直轴） | `src/systems/gimbal_pid.rs:14-24` |
| 图像/内参发布 | `src/ros2/capture.rs` |
| 相机内参公式 | `src/capture.rs:147-176` |
| 点云合成 | `src/ros2/livox.rs` |
| 场景布局 | `src/scene.rs:80-216` |
| 弹丸发射与气动 | `src/systems/projectile.rs` |
| 装甲板命中门控 | `src/robomaster/armor/collision.rs` |
