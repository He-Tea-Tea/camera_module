#!/bin/bash

# 构建v0.2.3的Orbbec单相机版本，SDK保持在工作空间之外。
set -euo pipefail

PROJECT_ROOT="$(cd "$(dirname "${BASH_SOURCE[0]}")/.." && pwd)"
ROS_SETUP="${ROS_SETUP:-/opt/ros/humble/setup.bash}"
SDK_SETUP="${CAMERA_SDK_SETUP:-$HOME/hxb_code/camera_sdk/activate_camera_sdks.sh}"

if [[ ! -f "$ROS_SETUP" ]]; then
    echo "错误：找不到ROS2环境脚本：$ROS_SETUP" >&2
    exit 1
fi

if [[ ! -f "$SDK_SETUP" ]]; then
    echo "错误：找不到相机SDK环境脚本：$SDK_SETUP" >&2
    exit 1
fi

# 先加载ROS2，再加载隔离安装的厂商SDK。
source "$ROS_SETUP"
source "$SDK_SETUP"

cd "$PROJECT_ROOT"

# 构建前先拒绝非法部署参数。
python3 src/camera_tools/camera_tools/config_validate.py config/cameras.yaml

# 公共包不接收厂商CMake选项，避免产生“变量未使用”警告。
colcon build \
    --symlink-install \
    --packages-select camera_adapter camera_interfaces camera_msgs camera_tools

source "$PROJECT_ROOT/install/setup.bash"

# 仅在camera_driver包中启用Orbbec后端。
colcon build \
    --symlink-install \
    --cmake-clean-cache \
    --packages-select camera_driver \
    --cmake-args \
    -DCMAKE_BUILD_TYPE=Release \
    -DCAMERA_DRIVER_ENABLE_ORBBEC=ON \
    -DCAMERA_DRIVER_ENABLE_REALSENSE=OFF

source "$PROJECT_ROOT/install/setup.bash"

# 最后链接相机管理节点。
colcon build \
    --symlink-install \
    --cmake-clean-cache \
    --packages-select camera_manager \
    --cmake-args \
    -DCMAKE_BUILD_TYPE=Release

echo "v0.2.3 Orbbec版本构建完成"
