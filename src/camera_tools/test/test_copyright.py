"""ROS2标准copyright lint入口。"""

import pytest
from ament_copyright.main import main


@pytest.mark.copyright
def test_copyright() -> None:
    """扫描源码中的版权声明问题；该检查由ament测试阶段运行。"""
    rc = main(argv=["."])
    assert rc == 0, "Found errors"
