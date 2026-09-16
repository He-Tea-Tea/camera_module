#!/bin/bash

# 启动头部Gemini335LE相机小脑节点。
set -euo pipefail

PROJECT_ROOT="$(cd "$(dirname "${BASH_SOURCE[0]}")/.." && pwd)"
ROS_SETUP="${ROS_SETUP:-/opt/ros/humble/setup.bash}"
SDK_SETUP="${CAMERA_SDK_SETUP:-$HOME/hxb_code/camera_sdk/activate_camera_sdks.sh}"
WORKSPACE_SETUP="$PROJECT_ROOT/install/setup.bash"
CAMERA_CONFIG="${CAMERA_CONFIG:-$PROJECT_ROOT/config/cameras.yaml}"

for required_file in "$ROS_SETUP" "$SDK_SETUP" "$WORKSPACE_SETUP" "$CAMERA_CONFIG"; do
    if [[ ! -f "$required_file" ]]; then
        echo "错误：缺少启动文件：$required_file" >&2
        exit 1
    fi
done

# 环境加载顺序固定为ROS2、厂商SDK、当前工作空间。
source "$ROS_SETUP"
source "$SDK_SETUP"
source "$WORKSPACE_SETUP"

python3 "$PROJECT_ROOT/src/camera_tools/camera_tools/config_validate.py" "$CAMERA_CONFIG"

# 使用exec让ROS2 Launch直接接管信号，保证Ctrl+C可以正常结束。
exec ros2 launch camera_manager camera_system.launch.py \
    config:="$CAMERA_CONFIG"
