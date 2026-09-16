"""校验camera_module v0.2.4与V1.3之间的私有接口边界和版本映射。"""

from __future__ import annotations

import argparse
import pathlib
import sys
from typing import Any

try:
    import yaml
except ImportError:  # pragma: no cover - 目标机通过python3-yaml提供该依赖。
    yaml = None

# 当前代码包严格对照用户提供的2026-09-10 V1.3候选稿。
EXPECTED_PROTOCOL_MAJOR = 1
EXPECTED_PROTOCOL_MINOR = 9
EXPECTED_ABI_REVISION = 17
EXPECTED_IMPLEMENTATION_RELEASE = "v0.2.4"


def _load_yaml(path: pathlib.Path) -> dict[str, Any]:
    """读取一个YAML映射；格式错误直接抛出供上层统一报告。"""
    if yaml is None:
        raise RuntimeError("需要安装 python3-yaml")
    with path.open("r", encoding="utf-8") as stream:
        document = yaml.safe_load(stream) or {}
    if not isinstance(document, dict):
        raise ValueError(f"{path} 根节点必须是YAML映射")
    return document


def validate(release_dir: pathlib.Path) -> list[str]:
    """返回发布草案中的全部协议边界问题。"""
    errors: list[str] = []
    private_path = release_dir / "camera_private_manifest.yaml"
    release_path = release_dir / "release_contract.yaml"
    old_public_name = release_dir / "interface_manifest.yaml"

    # v0.2.4故意不再使用interface_manifest.yaml这个权威名称，防止局部清单冒充整机公共清单。
    if old_public_name.exists():
        errors.append("release_contract/interface_manifest.yaml 不应存在；局部清单必须命名为camera_private_manifest.yaml")
    if not private_path.is_file():
        errors.append("缺少camera_private_manifest.yaml")
    if not release_path.is_file():
        errors.append("缺少release_contract.yaml")
    if errors:
        return errors

    private_manifest = _load_yaml(private_path)
    release_contract = _load_yaml(release_path)

    protocol = private_manifest.get("protocol", {})
    expected = (EXPECTED_PROTOCOL_MAJOR, EXPECTED_PROTOCOL_MINOR, EXPECTED_ABI_REVISION)
    actual_private = (
        protocol.get("protocol_major"),
        protocol.get("protocol_minor"),
        protocol.get("abi_revision"),
    )
    actual_release = (
        release_contract.get("protocol_major"),
        release_contract.get("protocol_minor"),
        release_contract.get("abi_revision"),
    )
    if actual_private != expected:
        errors.append(f"camera_private_manifest协议版本应为{expected}，实际为{actual_private}")
    if actual_release != expected:
        errors.append(f"release_contract协议版本应为{expected}，实际为{actual_release}")

    implementation = private_manifest.get("implementation", {})
    if implementation.get("implementation_release") != EXPECTED_IMPLEMENTATION_RELEASE:
        errors.append("implementation_release 必须是v0.2.4")
    if implementation.get("component_kind") != 7:
        errors.append("相机component_kind必须映射V1.3 COMPONENT_CAMERA=7")
    if implementation.get("device_id") != 255:
        errors.append("相机device_id必须映射V1.3 DEVICE_NONE=255")

    # camera_module中的rosidl全部属于小脑私有接口，禁止把任何一个标记为PUBLIC。
    interfaces = private_manifest.get("interfaces", [])
    if not isinstance(interfaces, list) or not interfaces:
        errors.append("camera_private_manifest.interfaces 不能为空")
    else:
        seen_types: set[tuple[str, str, str]] = set()
        for item in interfaces:
            if not isinstance(item, dict):
                errors.append("interfaces元素必须是YAML映射")
                continue
            identity = (str(item.get("package", "")), str(item.get("type", "")), str(item.get("name", "")))
            if identity in seen_types:
                errors.append(f"接口重复: {identity}")
            seen_types.add(identity)
            if item.get("visibility") != "PRIVATE":
                errors.append(f"{identity}: visibility必须为PRIVATE")
            if item.get("route") != "LOCAL_ONLY":
                errors.append(f"{identity}: route必须为LOCAL_ONLY")

    # 每个本地端点必须保持LOCAL_ONLY，并引用明确的授权/QoS/SROS2策略名。
    endpoints = private_manifest.get("endpoints", [])
    if not isinstance(endpoints, list) or not endpoints:
        errors.append("camera_private_manifest.endpoints 不能为空")
    else:
        seen_names: set[str] = set()
        for endpoint in endpoints:
            if not isinstance(endpoint, dict):
                errors.append("endpoints元素必须是YAML映射")
                continue
            name = str(endpoint.get("name", ""))
            if not name:
                errors.append("endpoint.name不能为空")
            if name in seen_names:
                errors.append(f"endpoint重复: {name}")
            seen_names.add(name)
            if endpoint.get("route_direction") != "LOCAL_ONLY":
                errors.append(f"{name}: route_direction必须为LOCAL_ONLY")
            for required_key in ("authorization_profile", "qos_profile_id", "sros2_policy_id", "interface"):
                if not endpoint.get(required_key):
                    errors.append(f"{name}: 缺少{required_key}")

    visibility = private_manifest.get("visibility_rules", {})
    if visibility.get("brain_must_not_call_private_endpoints") is not True:
        errors.append("brain_must_not_call_private_endpoints必须为true")
    if visibility.get("hardware_sdk_in_brain_process") is not False:
        errors.append("hardware_sdk_in_brain_process必须为false")

    startup = release_contract.get("startup_policy", {})
    if startup.get("bootstrap_before_capability_query") is not True:
        errors.append("整机公共能力查询必须在Bootstrap兼容握手之后")
    if startup.get("private_camera_interfaces_visible_to_brain") is not False:
        errors.append("private_camera_interfaces_visible_to_brain必须为false")

    return errors


def main() -> int:
    """命令行入口；默认校验项目中的release_contract目录。"""
    parser = argparse.ArgumentParser(description="校验camera_module V1.3私有协议映射")
    parser.add_argument("release_dir", type=pathlib.Path, nargs="?", default=pathlib.Path("release_contract"))
    args = parser.parse_args()
    try:
        errors = validate(args.release_dir)
    except (OSError, RuntimeError, ValueError) as error:
        print(f"协议草案读取失败: {error}", file=sys.stderr)
        return 2
    if errors:
        for error in errors:
            print(f"错误: {error}", file=sys.stderr)
        return 1
    print(
        "协议映射通过: protocol=1.9 abi=17, "
        "camera接口全部PRIVATE/LOCAL_ONLY, public ABI由整机仓库维护"
    )
    return 0


if __name__ == "__main__":
    raise SystemExit(main())
