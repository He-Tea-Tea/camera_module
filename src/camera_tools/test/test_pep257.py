"""ROS2标准PEP257文档字符串检查入口。"""

import pytest
from ament_pep257.main import main


@pytest.mark.pep257
def test_pep257() -> None:
    """检查Python模块、类和函数的文档字符串。"""
    rc = main(argv=["."])
    assert rc == 0, "Found code style errors / warnings"
