"""校验相机参数中的协议边界和硬件绑定。"""

import argparse
import pathlib
import sys

try:
    import yaml
except ImportError:  # pragma: no cover
    yaml = None


def load_config(path: pathlib.Path):
    if yaml is None:
        raise RuntimeError("需要安装 python3-yaml")
    with path.open("r", encoding="utf-8") as stream:
        return yaml.safe_load(stream) or {}


def validate(path: pathlib.Path) -> list[str]:
    document = load_config(path)
    params = document.get("camera_manager", {}).get("ros__parameters", {})
    names = params.get("camera_names", [])
    errors = []
    if not names:
        errors.append("camera_names 不能为空")
    instance_ids = set()
    for name in names:
        prefix = f"{name}."
        backend = params.get(prefix + "backend", "")
        if backend not in {"mock", "orbbec", "realsense"}:
            errors.append(f"{name}: backend 不支持: {backend}")
        instance_id = int(params.get(prefix + "component_instance_id", 0))
        if instance_id in instance_ids or instance_id in {0, 65535}:
            errors.append(f"{name}: component_instance_id 重复或非法")
        instance_ids.add(instance_id)
        width = int(params.get(prefix + "color_width", 0))
        height = int(params.get(prefix + "color_height", 0))
        if width < 8 or height < 8:
            errors.append(f"{name}: RGB 尺寸过小")
        calibration = str(params.get(prefix + "calibration_revision", ""))
        if not calibration or "FILL_" in calibration.upper():
            errors.append(f"{name}: 必须填写 calibration_revision")
        if backend != "mock":
            serial = str(params.get(prefix + "serial", ""))
            ip_address = str(params.get(prefix + "ip_address", ""))
            allow_unbound = bool(params.get(prefix + "allow_unbound_device", False))
            if "FILL_" in serial.upper() or "FILL_" in ip_address.upper():
                errors.append(f"{name}: 设备绑定仍是占位符")
            if not serial and not ip_address and not allow_unbound:
                errors.append(f"{name}: 真实设备必须配置 serial 或 ip_address")
            if backend == "realsense" and ip_address:
                errors.append(f"{name}: RealSense 当前只允许 serial 绑定")
        verified = bool(params.get(prefix + "capture_delay_bound_verified", False))
        bound = int(params.get(prefix + "capture_delay_bound_ms", 0))
        if verified and bound <= 0:
            errors.append(f"{name}: verified 延迟上界必须大于零")
    return errors


def main() -> int:
    parser = argparse.ArgumentParser(description="校验相机 ROS2 参数")
    parser.add_argument("config", type=pathlib.Path)
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
