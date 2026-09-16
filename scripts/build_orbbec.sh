#!/bin/bash

# 构建 v0.2.3 的 Orbbec 单相机版本，SDK 保持在工作空间之外。

# 开启错误退出和管道错误检测。
# 不启用 nounset（set -u），因为 ROS2 / colcon / 厂商 SDK 的环境脚本
# 可能会读取尚未定义的环境变量，例如 AMENT_TRACE_SETUP_FILES、COLCON_TRACE。
set -eo pipefail

# 获取当前项目根目录。
PROJECT_ROOT="$(cd "$(dirname "${BASH_SOURCE[0]}")/.." && pwd)"

# ROS2 Humble 环境脚本路径。
ROS_SETUP="${ROS_SETUP:-/opt/ros/humble/setup.bash}"

# 相机 SDK 环境脚本路径。
# 默认使用 ~/hxb_code/camera_sdk/activate_camera_sdks.sh。
SDK_SETUP="${CAMERA_SDK_SETUP:-$HOME/hxb_code/camera_sdk/activate_camera_sdks.sh}"

# 检查 ROS2 环境脚本是否存在。
if [[ ! -f "$ROS_SETUP" ]]; then
    echo "错误：找不到 ROS2 环境脚本：$ROS_SETUP" >&2
    exit 1
fi

# 检查相机 SDK 环境脚本是否存在。
if [[ ! -f "$SDK_SETUP" ]]; then
    echo "错误：找不到相机 SDK 环境脚本：$SDK_SETUP" >&2
    exit 1
fi

# 先加载 ROS2 Humble 环境。
source "$ROS_SETUP"

# 再加载独立安装的 Orbbec / RealSense SDK 环境。
source "$SDK_SETUP"

# 进入项目根目录。
cd "$PROJECT_ROOT"

# 构建前先校验相机配置文件。
# 如果 cameras.yaml 参数不合法，立即停止构建。
python3 \
    src/camera_tools/camera_tools/config_validate.py \
    config/cameras.yaml

# 第一阶段：
# 构建与厂商 SDK 无关的公共 ROS2 包。
# 这些包不接收 CAMERA_DRIVER_ENABLE_* 参数，
# 避免出现 “Manually-specified variables were not used” 警告。
colcon build \
    --symlink-install \
    --packages-select \
    camera_adapter \
    camera_interfaces \
    camera_msgs \
    camera_tools

# 加载第一阶段生成的工作空间环境。
# 这里不能启用 set -u，否则 colcon 生成的 setup.bash
# 可能因为 COLCON_TRACE 等未定义变量而退出。
source "$PROJECT_ROOT/install/setup.bash"

# 第二阶段：
# 单独构建 camera_driver。
# 当前只启用 Orbbec 后端，禁用 RealSense 后端。
# --cmake-clean-cache 用于清除之前缓存的 CMake 选项，
# 防止 CAMERA_DRIVER_ENABLE_* 参数被旧缓存影响。
colcon build \
    --symlink-install \
    --cmake-clean-cache \
    --packages-select camera_driver \
    --cmake-args \
    -DCMAKE_BUILD_TYPE=Release \
    -DCAMERA_DRIVER_ENABLE_ORBBEC=ON \
    -DCAMERA_DRIVER_ENABLE_REALSENSE=OFF

# 加载 camera_driver 安装后的环境，
# 让后续 camera_manager 能找到 camera_driver_core。
source "$PROJECT_ROOT/install/setup.bash"

# 第三阶段：
# 构建最终的 camera_manager ROS2 管理节点。
colcon build \
    --symlink-install \
    --cmake-clean-cache \
    --packages-select camera_manager \
    --cmake-args \
    -DCMAKE_BUILD_TYPE=Release

# 构建全部完成。
echo "v0.2.3 Orbbec 版本构建完成"