from setuptools import find_packages, setup

package_name = "camera_tools"

setup(
    name=package_name,
    version="0.2.0",
    packages=find_packages(exclude=["test"]),
    data_files=[
        ("share/ament_index/resource_index/packages", ["resource/" + package_name]),
        ("share/" + package_name, ["package.xml"]),
    ],
    install_requires=["setuptools"],
    zip_safe=True,
    maintainer="dyf",
    maintainer_email="HeXuBin13@outlook.com",
    description="相机参数校验和无硬件冒烟测试工具",
    license="Apache-2.0",
    entry_points={
        "console_scripts": [
            "camera-config-validate = camera_tools.config_validate:main",
            "camera-mock-smoke = camera_tools.mock_smoke:main",
        ],
    },
)
