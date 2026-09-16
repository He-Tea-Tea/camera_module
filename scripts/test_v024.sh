#!/usr/bin/env bash
# v0.2.4 无硬件回归入口：验证配置、协议草案、Mock核心路径和会话UUID语义。
set -euo pipefail

# 解析项目根目录，保证脚本从任意目录执行都能找到文件。
ROOT_DIR="$(cd "$(dirname "${BASH_SOURCE[0]}")/.." && pwd)"
cd "$ROOT_DIR"

# 第一步：校验真实相机配置的字段范围和设备绑定规则。
python3 src/camera_tools/camera_tools/config_validate.py config/cameras.yaml

# 第二步：校验双Mock配置，避免未来双相机参数漂移。
python3 src/camera_tools/camera_tools/config_validate.py config/cameras.mock.yaml

# 第三步：校验V1.3映射版本、PRIVATE/LOCAL_ONLY边界和发布草案一致性。
python3 src/camera_tools/camera_tools/protocol_validate.py release_contract

# 第四步：手工编译并运行Mock核心链路；无需ROS2和厂商SDK。
./scripts/build_mock.sh

# 第五步：确认v0.2.3误提交的根目录二进制不再进入源码包。
if [[ -f "$ROOT_DIR/test_orbbec_profiles" ]]; then
    echo "错误：源码包中不应包含编译后的test_orbbec_profiles二进制" >&2
    exit 1
fi

# 所有无硬件回归项通过。
echo "camera_module v0.2.4 无硬件回归通过"
