# camera_module v0.2.4

`camera_module` 是面向机器人小脑控制域的 RGB-D 相机基础模块。v0.2.4 在 v0.2.3 已经完成 Gemini335Le 实机 RGB-D、同步、D2C、三维反投影、抓拍、重连和安全退出验证的基础上，重点进行 **V1.3 协议边界收口、数据面会话身份加固、深度范围过滤和工程回归测试**。

本仓库对照《具身本体大脑-小脑通信协议（ROS2）方案 V1.3（2026-09-10 Candidate）》开发，但它**不是整机公共协议仓库**。厂商 SDK、原始 RGB-D、单相机状态和像素三维查询都留在小脑本地；大脑只能通过整机 `robot_body_interfaces` 中的公共能力访问语义结果。

## 当前定位

```text
Brain / AGX Orin
    │
    │  V1.3 PUBLIC ABI
    │  QueryCapabilities / ExecuteTask / ...
    ▼
Whole-body / Router / Capability Aggregator
    │
    │  PRIVATE + LOCAL_ONLY
    ▼
camera_manager
    │
    ├── GetPoint3D.srv
    ├── QueryCameraCapability.srv
    ├── CaptureImage.action
    └── CameraState.msg
    │
    ▼
camera_driver
    ├── Orbbec SDK v2   -> Gemini335Le
    ├── librealsense2   -> D436（预留，计划v0.3.0实机）
    └── Mock            -> 无硬件回归
```

V1.3 映射固定为：`COMPONENT_CAMERA=7`、`DEVICE_NONE=255`、当前时间能力语义为 `HW_PTP_NONE`，最终视觉闭环语义为 `CLOSED_LOOP_CEREBELLUM`。本模块的本地清单命名为 `camera_private_manifest.yaml`，故意不使用整机权威文件名 `interface_manifest.yaml`。

## v0.2.4 相比 v0.2.3 的主要变化

- 协议映射统一为 `protocol=1.9 / ABI=17`，严格对齐当前 V1.3 候选稿；
- 将模块局部清单更名为 `release_contract/camera_private_manifest.yaml`，明确不冒充整机公共 `interface_manifest.yaml`；
- `CameraState`、`GetPoint3D`、`CaptureImage` 增加 `frame_session_uuid`，重连后新会话 UUID 必须变化，帧序号允许重新从 1 开始；
- Orbbec/RealSense/Mock 都只在 `start()` 成功后创建新的 streaming session UUID；
- Orbbec 初始化中的重复 `instance_/sequence_` 初始化已清理；
- `min_depth_m/max_depth_m` 现在真正参与真实后端深度过滤，范围外深度统一写为 `0.0F`；
- 保留 Gemini335Le 实测配置：Color `640x400@30 RGB`，Depth `640x400@30 Y16`，FrameSync + ALL_TYPE_FRAME_REQUIRE + SW D2C；
- `build_orbbec.sh` 避免把当前工作空间旧版 `camera_driver` 重新作为 underlay，从而减少 colcon override warning；
- 增加 `protocol_validate.py`、配置范围校验、Mock 会话 UUID 回归和统一 `test_v024.sh`；
- 删除根目录编译产物 `test_orbbec_profiles`，Profile 探针源码移动到 `tools/orbbec_profile_probe.cpp`；
- 所有新增/修改源码、脚本、IDL、YAML 都补充中文注释，保留 v0.2.3 中已有说明。

## 项目结构

```text
camera_module_v0.2.4/
├── VERSION
├── README.md
├── config/
│   ├── cameras.yaml
│   ├── cameras.mock.yaml
│   └── calibration_template.yaml
├── docs/
│   ├── protocol_mapping.md
│   ├── implementation_plan.md
│   ├── V0.2.4_CHANGELOG.md
│   └── MIGRATION_v0.2.3_to_v0.2.4.md
├── launch/
│   └── camera_system.launch.py
├── release_contract/
│   ├── camera_private_manifest.yaml
│   ├── release_contract.yaml
│   ├── qos_profiles.yaml
│   ├── security_policy.yaml
│   └── deployment_topology.yaml
├── scripts/
│   ├── build_mock.sh
│   ├── build_orbbec.sh
│   ├── run_head_camera.sh
│   └── test_v024.sh
├── src/
│   ├── camera_adapter/
│   ├── camera_driver/
│   ├── camera_interfaces/
│   ├── camera_manager/
│   ├── camera_msgs/
│   └── camera_tools/
└── tools/
    └── orbbec_profile_probe.cpp
```

## 已验证的 Gemini335Le 配置

当前头部相机配置来自 v0.2.3 实机验证结果：

```text
Model:   Orbbec Gemini 335Le
Serial:  CPEP163000LG
IP:      192.168.1.10
Port:    8090
Color:   640x400 @ 30 FPS / RGB
Depth:   640x400 @ 30 FPS / Y16
Align:   Depth -> Color software alignment
Sync:    enableFrameSync()
Aggregate: OB_FRAME_AGGREGATE_OUTPUT_ALL_TYPE_FRAME_REQUIRE
```

`config/cameras.yaml` 当前只启用 `head_camera`。D436 配置继续保留为注释模板，不在 v0.2.4 宣称实机完成。

## 无硬件完整回归

不需要 ROS2、不需要 Orbbec/RealSense SDK 时，可直接运行：

```bash
cd ~/hxb_code/camera_module
./scripts/test_v024.sh
```

该脚本会依次验证：真实配置、双 Mock 配置、V1.3 私有协议映射、C++ Mock 编译与运行、三维点查询、stop/start 后会话 UUID 变化，以及根目录不存在误提交的 Profile 二进制。

也可以单独执行：

```bash
python3 src/camera_tools/camera_tools/config_validate.py config/cameras.yaml
python3 src/camera_tools/camera_tools/protocol_validate.py release_contract
./scripts/build_mock.sh
```

Mock 演示成功时应包含：

```text
point_code=OK
session_changed=true
```

## ROS2 Humble + Orbbec 构建

目标环境：Ubuntu 22.04 + ROS2 Humble。厂商 SDK 安装在工作空间外：

```bash
cd ~/hxb_code/camera_module
export CAMERA_SDK_SETUP="$HOME/hxb_code/camera_sdk/activate_camera_sdks.sh"
./scripts/build_orbbec.sh
```

构建脚本按三阶段执行：

```text
camera_adapter / camera_interfaces / camera_msgs / camera_tools
        ↓
camera_driver（Orbbec ON，RealSense OFF）
        ↓
camera_manager
```

Orbbec SDK 的条件宏当前仍使用 `PUBLIC`，因为 v0.2.4 延续 v0.2.3 的公共驱动头结构；消费者必须看到与动态库完全相同的类布局，避免 ABI 错位。后续若改为 PImpl，可以再把厂商 SDK 依赖收回 PRIVATE，但不在本版做架构大改。

## 启动真实头部相机

```bash
cd ~/hxb_code/camera_module
./scripts/run_head_camera.sh
```

正常日志应进入：

```text
Pipeline 启动成功
RGB-D 数据流已启动，等待第一组同步帧
```

状态验证：

```bash
source /opt/ros/humble/setup.bash
source "$HOME/hxb_code/camera_sdk/activate_camera_sdks.sh"
source "$HOME/hxb_code/camera_module/install/setup.bash"

ros2 topic echo /camera_manager/head_camera/state --once
```

核心期望：

```text
state: 2
rgb_ready: true
depth_ready: true
depth_aligned_to_color: true
pair_synchronized: true
frame_sequence: >0
frame_session_uuid: 非全0
error_code: 0
```

## 本地私有接口

这些接口全部是 `PRIVATE/LOCAL_ONLY`，禁止直接作为 Brain 公共接口：

```text
/camera_manager/head_camera/state
/camera_manager/head_camera/get_point_3d
/camera_manager/head_camera/query_capability
/camera_manager/capture_image
```

### GetPoint3D

```bash
ros2 service call \
  /camera_manager/head_camera/get_point_3d \
  camera_interfaces/srv/GetPoint3D \
  "{camera_name: 'head_camera', sequence: 0, u: 320, v: 200, require_trusted_time: false}"
```

`sequence=0` 表示最新帧；返回值包含 XYZ 米制坐标、`frame_sequence`、`frame_session_uuid`、坐标系、可信时间和错误码。重连后 sequence 可以重新从 1 开始，因此持久化引用必须同时保存 session UUID 和 sequence。

### QueryCameraCapability

```bash
ros2 service call \
  /camera_manager/head_camera/query_capability \
  camera_interfaces/srv/QueryCameraCapability \
  "{camera_name: 'head_camera'}"
```

该服务只提供小脑内部的单相机能力摘要。整机层应把它转换成 V1.3 `QueryCapabilities.srv` 的 `ComponentCapability/SkillCapability`；Brain 不应直接调用本服务。

### CaptureImage

```bash
ros2 action send_goal \
  /camera_manager/capture_image \
  camera_interfaces/action/CaptureImage \
  "{camera_name: 'head_camera', save_uri: 'file:///tmp/head_camera_test.ppm', timeout_ms: 2000, require_trusted_time: false}" \
  --feedback
```

Action 只保存本地 RGB 证据并返回路径、session、sequence、坐标系和时间摘要，不把整帧 RGB-D 塞进任务热路径。

## 深度有效范围

v0.2.4 开始，`min_depth_m/max_depth_m` 不再只是配置检查字段，而是进入统一数据面：

```text
raw depth
    ↓
SDK单位 -> meter
    ↓
finite && min_depth_m <= depth <= max_depth_m ?
    ├── yes -> 保存实际米制值
    └── no  -> 保存0.0F，统一表示INVALID_DEPTH
```

因此所有使用 `CameraFrame.depth_m` 的本地模块都得到一致的深度有效性语义。

## 会话 UUID 规则

`frame_session_uuid` 表示一次实际 streaming session：

```text
initialize()
    不创建有效session

start()成功
    创建新UUID
    sequence清零

frame1 -> sequence=1
frame2 -> sequence=2
...

掉线/stop/reconnect
    清空旧缓存

下一次start()成功
    创建另一个新UUID
    sequence重新从1开始
```

因此 `(frame_session_uuid, frame_sequence)` 才能唯一定位一个本地帧会话中的帧，不能仅保存 `frame_sequence`。

## 时间策略

当前 Gemini335Le 配置保持：

```yaml
capture_delay_bound_verified: false
capture_delay_bound_ms: 0
```

所以真实相机返回 `time_trusted=false`、`capture_time=0` 是设计预期。没有实际测量采集到主机接收延迟上界前，禁止通过系统接收时间伪装设备采集时间。

V1.3 下当前公共语义应映射为：

```text
HW_PTP_NONE
CLOSED_LOOP_CEREBELLUM
local_camera_available=true（相机STREAMING时）
```

这意味着大脑发送语义目标/粗位姿，小脑使用本地相机完成近距确认、三维定位和最后闭环，而不是把高速相机控制闭环放到跨 SoC 大脑端。

## 标定状态

当前：

```text
calibration_revision = factory-unverified-20260916
```

它只代表使用 SDK 当前内参可以进行数据链路验证，不代表机器人抓取空间精度已经正式验收。生产前应根据 `config/calibration_template.yaml` 生成真实标定制品，记录设备序列号、分辨率、内外参 revision、验证误差、工具版本和 SHA-256。

## 发布边界

`release_contract/camera_private_manifest.yaml` 只登记 camera_module 私有端点。整机真正的 V1.3 权威内容必须由独立整机仓库维护，包括：

```text
robot_body_bootstrap_interfaces/GetProtocolManifest
正式interface_manifest.yaml
正式release_contract.yaml和成员hash
robot_body_interfaces/QueryCapabilities
ComponentCapability / SkillCapability
GetDetailedStatus
RobotState / SafetyState
WholeBody ExecuteTask
Router / SROS2 / DDS部署成员
```

整机启动顺序应为：Bootstrap/hash 兼容预检成功后，再启用 `/capabilities/query`；不兼容时保持 `SAFE_IDLE`。

## v0.2.4 完成后下一步

本版不继续扩张公共 ABI。后续建议把 v0.3.0 定位为 **D436 实机 + 双 RGB-D 相机隔离验证**，然后再由更高层 perception/skill 模块实现 ObjectRef 本地重确认、视觉伺服和抓取闭环。
