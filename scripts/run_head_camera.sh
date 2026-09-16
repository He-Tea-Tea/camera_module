#!/bin/bash

# 启动头部 Gemini335LE 相机小脑节点。

# 开启错误退出和管道错误检测。
# 不启用 nounset（set -u），因为 ROS2、colcon 和厂商 SDK 的环境脚本
# 可能会读取尚未定义的变量，例如 AMENT_TRACE_SETUP_FILES、COLCON_TRACE。
set -eo pipefail

# 显式关闭 nounset。
# 即使调用当前脚本的外部 Shell 开启过 nounset，也避免影响 ROS2 环境加载。
set +u

# 获取当前脚本所在项目的根目录。
PROJECT_ROOT="$(cd "$(dirname "${BASH_SOURCE[0]}")/.." && pwd)"

# ROS2 Humble 环境脚本。
ROS_SETUP="${ROS_SETUP:-/opt/ros/humble/setup.bash}"

# 独立安装的厂商 SDK 环境脚本。
# 默认同时加载 Orbbec SDK 和 RealSense SDK 的路径。
SDK_SETUP="${CAMERA_SDK_SETUP:-$HOME/hxb_code/camera_sdk/activate_camera_sdks.sh}"

# 当前 camera_module 编译生成的 ROS2 工作空间环境。
WORKSPACE_SETUP="$PROJECT_ROOT/install/setup.bash"

# 相机配置文件。
# 可以通过 CAMERA_CONFIG 环境变量临时指定其他配置文件。
CAMERA_CONFIG="${CAMERA_CONFIG:-$PROJECT_ROOT/config/cameras.yaml}"

# 启动前检查所有必要文件是否存在。
for required_file in \
    "$ROS_SETUP" \
    "$SDK_SETUP" \
    "$WORKSPACE_SETUP" \
    "$CAMERA_CONFIG"; do

    if [[ ! -f "$required_file" ]]; then
        echo "错误：缺少启动文件：$required_file" >&2
        exit 1
    fi
done

# 第一层：加载 ROS2 Humble 环境。
source "$ROS_SETUP"

# 第二层：加载独立安装的 Orbbec / RealSense SDK。
source "$SDK_SETUP"

# 第三层：加载当前 camera_module 工作空间。
source "$WORKSPACE_SETUP"

# 启动前再次校验相机参数。
# 参数非法时直接终止，防止使用错误 IP、序列号、分辨率等配置启动设备。
python3 \
    "$PROJECT_ROOT/src/camera_tools/camera_tools/config_validate.py" \
    "$CAMERA_CONFIG"

# 输出当前使用的配置文件，方便确认真实启动参数。
echo "相机配置：$CAMERA_CONFIG"

# 使用 exec 让 ros2 launch 直接接管当前进程。
# 这样 Ctrl+C 的 SIGINT 会直接传给 ROS2 Launch，
# 避免外层 Bash 进程影响节点正常退出。
exec ros2 launch \
    camera_manager \
    camera_system.launch.py \
    config:="$CAMERA_CONFIG"
