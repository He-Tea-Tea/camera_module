"""校验相机ROS2参数中的协议边界、设备绑定和数据范围。"""

from __future__ import annotations

import argparse
import pathlib
import re
import sys
from typing import Any

try:
    import yaml
except ImportError:  # pragma: no cover - 目标机通过python3-yaml提供该依赖。
    yaml = None

# camera_name和optical_frame最终会进入ROS2名称/TF语义，限制为稳定标识符。
_IDENTIFIER = re.compile(r"^[A-Za-z][A-Za-z0-9_]*$")


def load_config(path: pathlib.Path) -> dict[str, Any]:
    """读取YAML配置；空文件按空字典处理。"""
    if yaml is None:
        raise RuntimeError("需要安装 python3-yaml")
    with path.open("r", encoding="utf-8") as stream:
        document = yaml.safe_load(stream) or {}
    if not isinstance(document, dict):
        raise ValueError("配置根节点必须是YAML映射")
    return document


def _as_int(value: Any, field: str) -> int:
    """把字段转换为整数，并在错误时保留字段名方便定位。"""
    try:
        return int(value)
    except (TypeError, ValueError) as error:
        raise ValueError(f"{field} 必须是整数") from error


def _as_float(value: Any, field: str) -> float:
    """把字段转换为浮点数，并提供字段级错误信息。"""
    try:
        return float(value)
    except (TypeError, ValueError) as error:
        raise ValueError(f"{field} 必须是数值") from error


def validate(path: pathlib.Path) -> list[str]:
    """返回全部配置错误；空列表表示校验通过。"""
    document = load_config(path)
    params = document.get("camera_manager", {}).get("ros__parameters", {})
    if not isinstance(params, dict):
        return ["camera_manager.ros__parameters 必须是YAML映射"]

    names = params.get("camera_names", [])
    errors: list[str] = []
    if not isinstance(names, list) or not names:
        return ["camera_names 不能为空且必须是列表"]

    # component_instance_id必须在同一机器人配置中唯一且跨重启稳定。
    instance_ids: set[int] = set()
    # camera_name也不能重复，否则ROS2端点和本地缓存会互相覆盖。
    seen_names: set[str] = set()

    for raw_name in names:
        name = str(raw_name)
        prefix = f"{name}."
        if not _IDENTIFIER.fullmatch(name):
            errors.append(f"{name}: camera_name 不是合法标识符")
        if name in seen_names:
            errors.append(f"{name}: camera_names 重复")
        seen_names.add(name)

        backend = str(params.get(prefix + "backend", ""))
        if backend not in {"mock", "orbbec", "realsense"}:
            errors.append(f"{name}: backend 不支持: {backend}")

        try:
            instance_id = _as_int(params.get(prefix + "component_instance_id", 0),
                                  prefix + "component_instance_id")
            if not 1 <= instance_id <= 65534 or instance_id in instance_ids:
                errors.append(f"{name}: component_instance_id 必须为1..65534且不能重复")
            instance_ids.add(instance_id)
        except ValueError as error:
            errors.append(str(error))

        optical_frame = str(params.get(prefix + "optical_frame", ""))
        if not optical_frame or not _IDENTIFIER.fullmatch(optical_frame):
            errors.append(f"{name}: optical_frame 不能为空且必须是合法标识符")

        calibration = str(params.get(prefix + "calibration_revision", ""))
        if not calibration or "FILL_" in calibration.upper():
            errors.append(f"{name}: 必须填写 calibration_revision")

        # 统一校验RGB/Depth尺寸和帧率，避免进入SDK后才失败。
        try:
            width = _as_int(params.get(prefix + "color_width", 0), prefix + "color_width")
            height = _as_int(params.get(prefix + "color_height", 0), prefix + "color_height")
            depth_width = _as_int(params.get(prefix + "depth_width", 0), prefix + "depth_width")
            depth_height = _as_int(params.get(prefix + "depth_height", 0), prefix + "depth_height")
            fps = _as_int(params.get(prefix + "fps", 0), prefix + "fps")
            if not 8 <= width <= 4096 or not 8 <= height <= 2160:
                errors.append(f"{name}: RGB尺寸非法")
            if not 8 <= depth_width <= 4096 or not 8 <= depth_height <= 2160:
                errors.append(f"{name}: Depth尺寸非法")
            if not 1 <= fps <= 120:
                errors.append(f"{name}: fps 必须为1..120")
        except ValueError as error:
            errors.append(str(error))

        # 真实设备生产配置必须绑定serial或IP，禁止依赖发现顺序选相机。
        if backend != "mock":
            serial = str(params.get(prefix + "serial", ""))
            ip_address = str(params.get(prefix + "ip_address", ""))
            allow_unbound = bool(params.get(prefix + "allow_unbound_device", False))
            if "FILL_" in serial.upper() or "FILL_" in ip_address.upper():
                errors.append(f"{name}: 设备绑定仍是占位符")
            if not serial and not ip_address and not allow_unbound:
                errors.append(f"{name}: 真实设备必须配置serial或ip_address")
            if backend == "realsense" and ip_address:
                errors.append(f"{name}: RealSense当前只允许serial绑定")
            try:
                network_port = _as_int(params.get(prefix + "network_port", 8090),
                                       prefix + "network_port")
                if backend == "orbbec" and ip_address and not 1 <= network_port <= 65535:
                    errors.append(f"{name}: Orbbec network_port 必须为1..65535")
            except ValueError as error:
                errors.append(str(error))

        # 超时和缓存参数既影响实时性，也影响掉线判断，因此在启动前统一拦截。
        try:
            wait_timeout = _as_int(params.get(prefix + "wait_timeout_ms", 200),
                                   prefix + "wait_timeout_ms")
            disconnect_timeout = _as_int(params.get(prefix + "disconnect_timeout_ms", 2000),
                                         prefix + "disconnect_timeout_ms")
            reconnect_delay = _as_int(params.get(prefix + "reconnect_delay_ms", 1000),
                                      prefix + "reconnect_delay_ms")
            cache_capacity = _as_int(params.get(prefix + "cache_capacity", 6),
                                     prefix + "cache_capacity")
            max_frame_age = _as_int(params.get(prefix + "max_frame_age_ms", 200),
                                    prefix + "max_frame_age_ms")
            max_pair_delta = _as_int(params.get(prefix + "max_pair_delta_ms", 35),
                                     prefix + "max_pair_delta_ms")
            if not 10 <= wait_timeout <= 1000:
                errors.append(f"{name}: wait_timeout_ms 必须为10..1000")
            if not wait_timeout <= disconnect_timeout <= 60000:
                errors.append(f"{name}: disconnect_timeout_ms 必须>=wait_timeout_ms且<=60000")
            if not 50 <= reconnect_delay <= 60000:
                errors.append(f"{name}: reconnect_delay_ms 必须为50..60000")
            if not 1 <= cache_capacity <= 16:
                errors.append(f"{name}: cache_capacity 必须为1..16")
            if not 1 <= max_frame_age <= 60000:
                errors.append(f"{name}: max_frame_age_ms 必须为1..60000")
            if not 0 <= max_pair_delta <= 1000:
                errors.append(f"{name}: max_pair_delta_ms 必须为0..1000")
        except ValueError as error:
            errors.append(str(error))

        # v0.2.4开始min/max深度真正参与所有后端的数据有效性过滤。
        try:
            min_depth = _as_float(params.get(prefix + "min_depth_m", 0.2),
                                  prefix + "min_depth_m")
            max_depth = _as_float(params.get(prefix + "max_depth_m", 6.0),
                                  prefix + "max_depth_m")
            if min_depth <= 0.0 or max_depth <= min_depth or max_depth > 100.0:
                errors.append(f"{name}: 深度范围必须满足0<min_depth_m<max_depth_m<=100")
            if backend == "mock":
                mock_depth = _as_float(params.get(prefix + "mock_depth_m", 1.0),
                                       prefix + "mock_depth_m")
                if not min_depth <= mock_depth <= max_depth:
                    errors.append(f"{name}: mock_depth_m 必须位于有效深度范围内")
        except ValueError as error:
            errors.append(str(error))

        # 时间可信度只能在有实测延迟上界时开启，不能通过接收时间伪装采集时间。
        verified = bool(params.get(prefix + "capture_delay_bound_verified", False))
        try:
            bound = _as_int(params.get(prefix + "capture_delay_bound_ms", 0),
                            prefix + "capture_delay_bound_ms")
            if bound < 0 or bound > 60000:
                errors.append(f"{name}: capture_delay_bound_ms 必须为0..60000")
            if verified and bound <= 0:
                errors.append(f"{name}: verified延迟上界必须大于零")
        except ValueError as error:
            errors.append(str(error))

    return errors


def main() -> int:
    """命令行入口；通过返回0，配置错误返回1，读取环境错误返回2。"""
    parser = argparse.ArgumentParser(description="校验camera_module ROS2参数")
    parser.add_argument("config", type=pathlib.Path, help="待校验的cameras.yaml路径")
    args = parser.parse_args()
    try:
        errors = validate(args.config)
    except (OSError, RuntimeError, ValueError) as error:
        print(f"配置读取失败: {error}", file=sys.stderr)
        return 2
    if errors:
        for error in errors:
            print(f"错误: {error}", file=sys.stderr)
        return 1
    print(f"配置通过: {args.config}")
    return 0


if __name__ == "__main__":
    raise SystemExit(main())
