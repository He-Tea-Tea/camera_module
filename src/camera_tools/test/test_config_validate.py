"""config_validate的最小回归测试。"""

from pathlib import Path

from camera_tools.config_validate import validate


def test_mock_config_is_valid() -> None:
    """仓库自带双Mock配置必须始终通过静态参数校验。"""
    root = Path(__file__).resolve().parents[3]
    assert validate(root / "config" / "cameras.mock.yaml") == []
