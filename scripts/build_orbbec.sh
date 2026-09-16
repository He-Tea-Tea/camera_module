#!/bin/bash

# camera_module v0.2.4
# Orbbec 单相机构建脚本。
#
# 构建目标：
# 1. 使用 ROS2 Humble。
# 2. 使用工作空间之外独立安装的 Orbbec SDK v2。
# 3. 启用 Orbbec 后端。
# 4. 当前禁用 RealSense 后端。
# 5. 构建前校验真实相机配置。
# 6. 构建前校验 V1.3 PRIVATE / LOCAL_ONLY 协议边界。
# 7. 清除当前 Shell 中旧 camera_module overlay 对本次构建的污染。
# 8. 分阶段构建，避免把厂商 SDK CMake 参数传给无关包。
#
# 注意：
# 不启用 set -u。
# ROS2、colcon 和厂商 SDK 的环境脚本可能读取尚未定义的变量，
# 例如 AMENT_TRACE_SETUP_FILES、COLCON_TRACE，使用 nounset 可能导致异常退出。

set -eo pipefail

# ---------------------------------------------------------------------------
# 基础路径
# ---------------------------------------------------------------------------

# 获取 camera_module 项目根目录。
PROJECT_ROOT="$(cd "$(dirname "${BASH_SOURCE[0]}")/.." && pwd)"

# ROS2 Humble 环境脚本。
# 可以通过外部 ROS_SETUP 环境变量覆盖。
ROS_SETUP="${ROS_SETUP:-/opt/ros/humble/setup.bash}"

# 独立安装的相机 SDK 激活脚本。
# 默认同时配置 Orbbec 和 RealSense SDK 搜索路径。
SDK_SETUP="${CAMERA_SDK_SETUP:-$HOME/hxb_code/camera_sdk/activate_camera_sdks.sh}"

# 当前真实设备配置。
CAMERA_CONFIG="${CAMERA_CONFIG:-$PROJECT_ROOT/config/cameras.yaml}"

# v0.2.4 私有协议和部署契约目录。
RELEASE_CONTRACT_DIR="$PROJECT_ROOT/release_contract"

# ---------------------------------------------------------------------------
# 清理旧 camera_module overlay
# ---------------------------------------------------------------------------

# 从冒号分隔环境变量中删除当前项目 install 下的旧路径。
#
# 如果当前 Shell 曾执行：
#
#   source ~/hxb_code/camera_module/install/setup.bash
#
# AMENT_PREFIX_PATH、CMAKE_PREFIX_PATH、LD_LIBRARY_PATH 等变量中可能残留
# 上一次 v0.2.3 / v0.2.4 构建结果。
#
# 这些旧路径会使 colcon 把当前项目自身误判成 underlay。
strip_project_prefixes() {
    local variable_name="$1"
    local old_value="${!variable_name:-}"
    local new_value=""
    local item=""

    # 空环境变量无需处理。
    if [[ -z "$old_value" ]]; then
        return
    fi

    # 按冒号拆分搜索路径。
    IFS=':' read -r -a items <<< "$old_value"

    for item in "${items[@]}"; do
        # 忽略空项。
        [[ -z "$item" ]] && continue

        # 删除 install 根目录。
        if [[ "$item" == "$PROJECT_ROOT/install" ]]; then
            continue
        fi

        # 删除所有当前项目包级 install 路径。
        if [[ "$item" == "$PROJECT_ROOT/install/"* ]]; then
            continue
        fi

        # 其余 ROS2、系统和其他工作空间路径继续保留。
        if [[ -z "$new_value" ]]; then
            new_value="$item"
        else
            new_value="$new_value:$item"
        fi
    done

    export "$variable_name=$new_value"
}

# ---------------------------------------------------------------------------
# 加载单个 ament_cmake 包
# ---------------------------------------------------------------------------

# ament_cmake 包由 colcon 安装后，其包级 local_setup.bash 位于：
#
#   install/<package>/share/<package>/local_setup.bash
#
# 例如：
#
#   install/camera_adapter/share/camera_adapter/local_setup.bash
#
# 注意：
# camera_tools 是 ament_python 工具包，本脚本不需要 source camera_tools。
source_cmake_package() {
    local package_name="$1"
    local setup_file="$PROJECT_ROOT/install/$package_name/share/$package_name/local_setup.bash"

    if [[ ! -f "$setup_file" ]]; then
        echo "错误：找不到 CMake 包环境脚本：" >&2
        echo "  $setup_file" >&2
        echo "请确认 $package_name 已成功完成 colcon build。" >&2
        exit 1
    fi

    source "$setup_file"
}

# ---------------------------------------------------------------------------
# 基础文件检查
# ---------------------------------------------------------------------------

# 检查 ROS2 Humble。
if [[ ! -f "$ROS_SETUP" ]]; then
    echo "错误：找不到 ROS2 环境脚本：" >&2
    echo "  $ROS_SETUP" >&2
    exit 1
fi

# 检查 SDK 环境。
if [[ ! -f "$SDK_SETUP" ]]; then
    echo "错误：找不到相机 SDK 环境脚本：" >&2
    echo "  $SDK_SETUP" >&2
    exit 1
fi

# 检查相机配置。
if [[ ! -f "$CAMERA_CONFIG" ]]; then
    echo "错误：找不到相机配置：" >&2
    echo "  $CAMERA_CONFIG" >&2
    exit 1
fi

# 检查 release contract。
if [[ ! -d "$RELEASE_CONTRACT_DIR" ]]; then
    echo "错误：找不到协议目录：" >&2
    echo "  $RELEASE_CONTRACT_DIR" >&2
    exit 1
fi

# 检查 Python。
if ! command -v python3 >/dev/null 2>&1; then
    echo "错误：没有找到 python3。" >&2
    exit 1
fi

# ---------------------------------------------------------------------------
# 清除当前 Shell 的旧 overlay
# ---------------------------------------------------------------------------

strip_project_prefixes AMENT_PREFIX_PATH
strip_project_prefixes CMAKE_PREFIX_PATH
strip_project_prefixes COLCON_PREFIX_PATH
strip_project_prefixes LD_LIBRARY_PATH
strip_project_prefixes PYTHONPATH
strip_project_prefixes PATH

# ---------------------------------------------------------------------------
# 加载 ROS2 和厂商 SDK
# ---------------------------------------------------------------------------

# ROS2 Humble。
source "$ROS_SETUP"

# 独立 SDK。
source "$SDK_SETUP"

# 进入项目目录。
cd "$PROJECT_ROOT"

echo
echo "============================================================"
echo " camera_module v0.2.4 Orbbec 构建"
echo "============================================================"
echo "项目目录：$PROJECT_ROOT"
echo "ROS2环境：$ROS_SETUP"
echo "SDK环境：$SDK_SETUP"
echo "相机配置：$CAMERA_CONFIG"
echo

# ---------------------------------------------------------------------------
# 第 1 步：相机参数校验
# ---------------------------------------------------------------------------

echo "[1/6] 校验相机配置..."

python3 \
    "$PROJECT_ROOT/src/camera_tools/camera_tools/config_validate.py" \
    "$CAMERA_CONFIG"

# ---------------------------------------------------------------------------
# 第 2 步：V1.3 私有协议校验
# ---------------------------------------------------------------------------

echo
echo "[2/6] 校验 V1.3 私有协议映射..."

# 检查：
#
# protocol_major = 1
# protocol_minor = 9
# abi_revision = 17
#
# camera_module 接口必须为：
#
# PRIVATE
# LOCAL_ONLY
#
# 当前仓库不能冒充 robot_body_interfaces 整机公共 ABI。
python3 \
    "$PROJECT_ROOT/src/camera_tools/camera_tools/protocol_validate.py" \
    "$RELEASE_CONTRACT_DIR"

# ---------------------------------------------------------------------------
# 第 3 步：构建基础包
# ---------------------------------------------------------------------------

echo
echo "[3/6] 构建厂商无关基础包..."

# camera_adapter：
#   C++ 厂商无关 RGB-D 数据结构、帧缓存、时间质量和反投影。
#
# camera_interfaces：
#   小脑 PRIVATE Service / Action。
#
# camera_msgs：
#   小脑 PRIVATE 状态消息。
#
# camera_tools：
#   Python 参数校验和协议测试工具。
#
# 此阶段不向这些包传 CAMERA_DRIVER_ENABLE_*，
# 避免无关 CMake 警告。
colcon build \
    --symlink-install \
    --packages-select \
    camera_adapter \
    camera_interfaces \
    camera_msgs \
    camera_tools

# ---------------------------------------------------------------------------
# 为 camera_driver 准备依赖
# ---------------------------------------------------------------------------

# camera_driver 的工程依赖是 camera_adapter。
#
# camera_adapter 是 ament_cmake 包，
# 因此加载：
#
# install/camera_adapter/share/camera_adapter/local_setup.bash
source_cmake_package camera_adapter

# ---------------------------------------------------------------------------
# 第 4 步：构建 Orbbec 驱动
# ---------------------------------------------------------------------------

echo
echo "[4/6] 构建 Orbbec camera_driver..."

# 当前只打开 Orbbec。
#
# Mock 驱动仍然始终保留。
# RealSense 源码也继续保留，但本次不链接 librealsense。
#
# --cmake-clean-cache 清除之前构建遗留的 CAMERA_DRIVER_ENABLE_*。
colcon build \
    --symlink-install \
    --cmake-clean-cache \
    --packages-select camera_driver \
    --cmake-args \
    -DCMAKE_BUILD_TYPE=Release \
    -DCAMERA_DRIVER_ENABLE_ORBBEC=ON \
    -DCAMERA_DRIVER_ENABLE_REALSENSE=OFF

# 加载刚刚构建完成的 camera_driver。
source_cmake_package camera_driver

# ---------------------------------------------------------------------------
# 为 camera_manager 准备 ROS2 接口依赖
# ---------------------------------------------------------------------------

# camera_manager_node 依赖 camera_interfaces 和 camera_msgs。
#
# v0.2.4 修改了：
#
# CameraState.msg
# GetPoint3D.srv
# CaptureImage.action
#
# 因此必须使用这一轮刚刚重新生成的接口代码。
source_cmake_package camera_interfaces
source_cmake_package camera_msgs

# camera_tools 不参与 camera_manager C++ 链接。
# 不需要 source camera_tools。

# ---------------------------------------------------------------------------
# 第 5 步：构建 camera_manager
# ---------------------------------------------------------------------------

echo
echo "[5/6] 构建 camera_manager..."

colcon build \
    --symlink-install \
    --cmake-clean-cache \
    --packages-select camera_manager \
    --cmake-args \
    -DCMAKE_BUILD_TYPE=Release

# ---------------------------------------------------------------------------
# 第 6 步：检查最终产物
# ---------------------------------------------------------------------------

echo
echo "[6/6] 检查构建结果..."

# ROS2 主节点。
CAMERA_MANAGER_NODE="$PROJECT_ROOT/install/camera_manager/lib/camera_manager/camera_manager_node"

if [[ ! -x "$CAMERA_MANAGER_NODE" ]]; then
    echo "错误：没有生成 camera_manager_node：" >&2
    echo "  $CAMERA_MANAGER_NODE" >&2
    exit 1
fi

# camera_manager 自身包环境。
CAMERA_MANAGER_SETUP="$PROJECT_ROOT/install/camera_manager/share/camera_manager/local_setup.bash"

if [[ ! -f "$CAMERA_MANAGER_SETUP" ]]; then
    echo "错误：没有生成 camera_manager 环境脚本：" >&2
    echo "  $CAMERA_MANAGER_SETUP" >&2
    exit 1
fi

# ---------------------------------------------------------------------------
# 完成
# ---------------------------------------------------------------------------

echo
echo "============================================================"
echo " camera_module v0.2.4 Orbbec 版本构建完成"
echo "============================================================"
echo
echo "已生成："
echo "  $CAMERA_MANAGER_NODE"
echo
echo "下一步启动："
echo
echo "  cd \"$PROJECT_ROOT\""
echo "  ./scripts/run_head_camera.sh"
echo