"""camera_tools 的 setuptools 安装入口。"""

from setuptools import find_packages, setup

# ROS2 Python包名称必须和package.xml保持一致。
package_name = "camera_tools"

setup(
    # 安装包的基础元数据。
    name=package_name,
    version="0.2.4",
    packages=find_packages(exclude=["test"]),
    # ament索引和package.xml需要安装到share目录，ros2 pkg才能发现本包。
    data_files=[
        ("share/ament_index/resource_index/packages", ["resource/" + package_name]),
        ("share/" + package_name, ["package.xml"]),
    ],
    # setuptools由ROS2 Python运行环境提供；PyYAML在package.xml中声明系统依赖。
    install_requires=["setuptools"],
    zip_safe=True,
    maintainer="dyf",
    maintainer_email="HeXuBin13@outlook.com",
    description="相机参数、V1.3私有边界和Mock冒烟验证工具",
    license="Apache-2.0",
    # 命令行入口统一使用camera-前缀，便于安装后直接调用。
    entry_points={
        "console_scripts": [
            "camera-config-validate = camera_tools.config_validate:main",
            "camera-protocol-validate = camera_tools.protocol_validate:main",
            "camera-mock-smoke = camera_tools.mock_smoke:main",
        ],
    },
)
