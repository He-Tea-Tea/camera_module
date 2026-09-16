"""protocol_validate的最小回归测试。"""

from pathlib import Path

from camera_tools.protocol_validate import validate


def test_private_manifest_matches_v13_boundary() -> None:
    """v0.2.4私有清单必须保持1.9/ABI17和PRIVATE/LOCAL_ONLY边界。"""
    root = Path(__file__).resolve().parents[3]
    assert validate(root / "release_contract") == []
