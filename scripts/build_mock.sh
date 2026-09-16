#!/usr/bin/env bash
# 在没有ROS2和厂商SDK的环境中编译核心Mock演示，用于快速验证统一数据契约和会话语义。
set -euo pipefail

# 获取项目根目录，脚本可以从任意当前工作目录执行。
ROOT_DIR="$(cd "$(dirname "${BASH_SOURCE[0]}")/.." && pwd)"
# 手工Mock构建输出放到build/mock_manual，不污染源码目录。
BUILD_DIR="${ROOT_DIR}/build/mock_manual"
# 创建构建目录。
mkdir -p "${BUILD_DIR}"

# 仅包含厂商无关的三个公共/内部头文件目录；不需要ROS2或Orbbec/RealSense头文件。
INCLUDES=(
  "-I${ROOT_DIR}/src/camera_adapter/include"
  "-I${ROOT_DIR}/src/camera_driver/include"
  "-I${ROOT_DIR}/src/camera_manager/include"
)

# Mock手工构建仍编译Orbbec/RealSense源文件，以验证未启用SDK宏时的明确失败路径可以正常编译。
SOURCES=(
  "${ROOT_DIR}/src/camera_adapter/src/camera_adapter.cpp"
  "${ROOT_DIR}/src/camera_adapter/src/frame_buffer.cpp"
  "${ROOT_DIR}/src/camera_driver/src/mock_camera_driver.cpp"
  "${ROOT_DIR}/src/camera_driver/src/orbbec_camera_driver.cpp"
  "${ROOT_DIR}/src/camera_driver/src/realsense_camera_driver.cpp"
  "${ROOT_DIR}/src/camera_manager/src/camera_health.cpp"
  "${ROOT_DIR}/src/camera_manager/src/camera_registry.cpp"
  "${ROOT_DIR}/src/camera_manager/src/camera_manager_demo.cpp"
)

# 使用C++17、常用警告和pthread编译最小演示程序。
g++ -std=c++17 -Wall -Wextra -Wpedantic -pthread "${INCLUDES[@]}" \
  "${SOURCES[@]}" -o "${BUILD_DIR}/camera_manager_demo"

# 运行演示；程序内部会验证3D查询、session UUID和重启后的会话变化。
"${BUILD_DIR}/camera_manager_demo"
