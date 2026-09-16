# camera_module v0.2.3

这是从 GitHub `v0.1.1` 整理出的相机小脑实现。它按照《具身本体大脑-小脑通信协议（ROS2）方案 V1.3》的边界开发：厂商 SDK、设备枚举、帧同步、标定和原始数据都留在小脑；大脑只通过整机公共能力获得视觉证据。

当前实现包含：

- 统一 C++17 RGB-D 数据契约和有界帧缓存；
- Orbbec SDK v2 的 Gemini335Le 后端；
- librealsense 的 D436 后端；
- 无硬件 Mock 后端和故障注入；
- 生命周期、重连、状态、能力和本地三维查询管理器；
- `GetPoint3D.srv`、`QueryCameraCapability.srv`、`CaptureImage.action` 和私有 `CameraState.msg`；
- ROS2 参数、启动文件、协议清单、QoS 和部署拓扑草案。

## 与 v0.1.1 的关键修改

v0.2.3 在 v0.2.2 真实头部相机配置基础上修复了同步初始化阻塞：ROS2 控制面会先启动，
硬件连接在后台执行。SDK 永久阻塞时，节点退出也不会无限等待；掉线超时、重新初始化、
网络设备序列号核对、抓拍超时和取消、接收时间采样均已补齐。

当前仍把相机服务标为 `PRIVATE/LOCAL_ONLY`。协议要求的公共 `robot_body_interfaces`、Bootstrap hash 比较、Router/SROS2 和对象本地重确认必须在整机仓库完成，不能把本仓库的私有服务直接给大脑。

## 目录

```text
camera_module/
├── config/cameras.yaml
├── launch/camera_system.launch.py
├── release_contract/
├── docs/
├── src/camera_adapter/       # 厂商无关的数据契约、时间和帧缓存
├── src/camera_driver/        # Mock、Orbbec、RealSense SDK 封装
├── src/camera_interfaces/    # ROS2 私有 srv/action
├── src/camera_manager/       # 生命周期、重连、状态和 ROS2 节点
├── src/camera_msgs/          # ROS2 私有状态消息
└── src/camera_tools/         # 参数校验和冒烟工具
```

## 先跑无硬件验证

当前容器没有 ROS2、colcon 或 CMake，因此可以直接用系统 C++ 编译器验证核心路径：

```bash
cd camera_module
mkdir -p ../tmp/build_mock
INC="-Isrc/camera_adapter/include -Isrc/camera_driver/include -Isrc/camera_manager/include"
g++ -std=c++17 -Wall -Wextra -Wpedantic -pthread $INC \
  src/camera_adapter/src/camera_adapter.cpp \
  src/camera_adapter/src/frame_buffer.cpp \
  src/camera_driver/src/mock_camera_driver.cpp \
  src/camera_driver/src/orbbec_camera_driver.cpp \
  src/camera_driver/src/realsense_camera_driver.cpp \
  src/camera_manager/src/camera_health.cpp \
  src/camera_manager/src/camera_registry.cpp \
  src/camera_manager/src/camera_manager_demo.cpp \
  -o ../tmp/build_mock/camera_manager_demo
../tmp/build_mock/camera_manager_demo
```

通过标准是输出 `state=ready`、`point_code=OK`，并得到米制三维坐标。`camera_tools` 还可以校验 ROS2 参数：

```bash
python3 src/camera_tools/camera_tools/config_validate.py config/cameras.yaml
```

## ROS2 构建

目标环境是 Ubuntu 22.04 和 ROS2 Humble。项目使用工作空间外的独立 SDK 安装目录，
先设置 SDK 环境脚本位置，再运行分阶段构建脚本：

```bash
cd ~/hxb_code/camera_module

# 指向已部署的SDK环境脚本，默认值就是下面这个路径。
export CAMERA_SDK_SETUP="$HOME/hxb_code/camera_sdk/activate_camera_sdks.sh"

# 只启用当前需要的Orbbec后端，并分包构建以消除无关CMake警告。
./scripts/build_orbbec.sh
```

构建完成后启动头部相机：

```bash
./scripts/run_head_camera.sh
```

无硬件的 ROS2 联调可以加载 `config/cameras.mock.yaml`：

```bash
ros2 launch camera_manager camera_system.launch.py \
  config:=$(ros2 pkg prefix camera_manager)/share/camera_manager/config/cameras.mock.yaml
```

真实 SDK 的头文件和库由系统安装提供，不能把 SDK 二进制提交进仓库。CMake 找不到 SDK 时会在启用对应选项的阶段直接报错；未启用时，后端会返回 `UNSUPPORTED_BACKEND`，不会伪装成可用设备。

## 设备绑定和参数

编辑 `config/cameras.yaml`：

1. Gemini335Le 填固定 IP 或序列号；网络设备默认端口为 8090。
2. D436 必须填写实际序列号，RealSense 后端不接受 IP 绑定。
3. 为每台设备填写不重复的 `component_instance_id`。
4. 填写真实 `optical_frame` 和标定修订号。
5. 未完成采集延迟测量时保持 `capture_delay_bound_verified: false`。
6. 确认 SDK 能提供 RGB 与对齐深度；当前统一契约输出相同的彩色像素尺寸。

配置通过后也可以手动启动：

```bash
source /opt/ros/humble/setup.bash
source "$HOME/hxb_code/camera_sdk/activate_camera_sdks.sh"
source "$HOME/hxb_code/camera_module/install/setup.bash"

ros2 launch camera_manager camera_system.launch.py \
  config:="$HOME/hxb_code/camera_module/config/cameras.yaml"
```

节点从参数中的 `camera_names` 和 `<camera_name>.<field>` 读取相机。v0.2.3 会先建立本地
ROS2 端点，再由后台线程连接相机；连接期间状态为 `CONNECTING`，失败后进入
`FAULT/RECONNECTING`，不会阻塞参数服务和状态 Topic。

## 私有接口

这些接口用于小脑本地调试和执行，不是大脑公共接口：

```text
~/head_camera/state
~/head_camera/get_point_3d
~/head_camera/query_capability
~/capture_image
```

`GetPoint3D` 请求像素、可选帧序号和是否要求可信时间。服务会拒绝过期帧、没有可信采集时间的帧、无效深度和越界像素。结果坐标单位为米，坐标系为相机 `optical_frame`。

`CaptureImage` 只返回本地 PPM 证据文件的 URI、帧序号和时间摘要，不把原始 RGB-D 大块数据放进任务服务响应。生产版本应将文件证据替换为小脑内部的数据面，并由整机 Router 转换成公共语义能力。

## 时间、同步和安全边界

- RGB 与深度必须来自同一帧组，并在 `max_pair_delta_ms` 内；不满足时整帧拒绝。
- SDK 的设备时间不自动被接收时间覆盖；没有经过测量的延迟上界就不声明时间可信。
- `local_camera_available=true` 只表示小脑有相机，不能替代对象的本地视觉重确认。
- 没有硬件 PTP 时，最终视觉闭环保留在小脑；大脑只发送语义目标或粗略位姿。
- 相机断连会进入 `FAULT/RECONNECTING`，恢复后清理旧缓存并重新开始帧会话。
- 生产环境应使用静态 peer 或 discovery server、SROS2 和 topic/service 白名单，禁止大脑发现 `camera_interfaces`。

协议映射详见 [`docs/protocol_mapping.md`](docs/protocol_mapping.md)，发布前步骤详见 [`docs/implementation_plan.md`](docs/implementation_plan.md)。

## 联调顺序

1. 先用 Mock 验证帧契约、缓存、三维反投影、错误码和重连。
2. 单独接入 Gemini335Le，确认 RGB、深度、对齐、帧同步和内参。
3. 单独接入 D436，确认按 serial 绑定、深度单位和 timestamp domain。
4. 两台真实设备同时运行，检查组件实例号、状态和缓存不会串台。
5. 在整机仓库完成 Bootstrap 版本/ABI/hash 比较；不一致必须进入 `SAFE_IDLE`。
6. 接入 Router 的公共视觉能力，验证大脑完全看不到相机私有服务。
7. 进行断连、过期、时间不可信、标定版本变化、对象重确认和安全停机验收。
