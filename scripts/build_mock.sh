#!/usr/bin/env bash
# 在没有 ROS2 和厂商 SDK 的环境中编译核心 Mock 演示。
set -euo pipefail

ROOT_DIR="$(cd "$(dirname "${BASH_SOURCE[0]}")/.." && pwd)"
BUILD_DIR="${ROOT_DIR}/build/mock_manual"
mkdir -p "${BUILD_DIR}"
INCLUDES=(
  "-I${ROOT_DIR}/src/camera_adapter/include"
  "-I${ROOT_DIR}/src/camera_driver/include"
  "-I${ROOT_DIR}/src/camera_manager/include"
)
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
g++ -std=c++17 -Wall -Wextra -Wpedantic -pthread "${INCLUDES[@]}" \
  "${SOURCES[@]}" -o "${BUILD_DIR}/camera_manager_demo"
"${BUILD_DIR}/camera_manager_demo"
