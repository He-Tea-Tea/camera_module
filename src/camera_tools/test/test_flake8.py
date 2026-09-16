"""ROS2标准flake8代码风格检查入口。"""

import pytest
from ament_flake8.main import main_with_errors


@pytest.mark.flake8
def test_flake8() -> None:
    """检查Python源码中的语法、未使用变量和常见风格问题。"""
    rc, errors = main_with_errors(argv=[])
    assert rc == 0, "Found %d code style errors / warnings:\n" % len(errors) + "\n".join(errors)
