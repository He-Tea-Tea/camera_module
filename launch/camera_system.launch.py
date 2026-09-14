"""启动相机小脑节点；接口默认绑定在本机 ROS2 域内。"""
from pathlib import Path

from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node


def generate_launch_description():
    default_config = Path(get_package_share_directory("camera_manager")) / "config" / "cameras.yaml"
    config_path = DeclareLaunchArgument(
        "config", default_value=str(default_config), description="相机 ROS2 参数文件"
    )
    node = Node(
        package="camera_manager",
        executable="camera_manager_node",
        name="camera_manager",
        output="screen",
        parameters=[LaunchConfiguration("config")],
    )
    return LaunchDescription([config_path, node])
