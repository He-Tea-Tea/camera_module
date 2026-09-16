"""启动相机小脑节点；接口默认绑定在小脑本地 ROS2 控制域内。"""

# pathlib用于组合安装后的默认参数文件路径，避免手工拼接平台路径。
from pathlib import Path

# ament索引用于找到camera_manager安装后的share目录。
from ament_index_python.packages import get_package_share_directory
# LaunchDescription描述本次ROS2启动图。
from launch import LaunchDescription
# DeclareLaunchArgument允许命令行覆盖配置文件。
from launch.actions import DeclareLaunchArgument
# LaunchConfiguration读取运行时launch参数。
from launch.substitutions import LaunchConfiguration
# Node负责启动camera_manager_node进程。
from launch_ros.actions import Node


def generate_launch_description():
    """生成camera_manager的最小启动图，不在launch层暴露任何厂商SDK细节。"""
    # 默认使用安装到camera_manager/share目录中的cameras.yaml。
    default_config = Path(get_package_share_directory("camera_manager")) / "config" / "cameras.yaml"
    # 声明config参数，实机、Mock和测试环境都可以传入独立YAML。
    config_path = DeclareLaunchArgument(
        "config", default_value=str(default_config), description="相机 ROS2 参数文件"
    )
    # 相机管理器只读取统一参数并创建PRIVATE/LOCAL_ONLY本地端点。
    node = Node(
        package="camera_manager",               # ROS2包名
        executable="camera_manager_node",      # 安装后的节点可执行文件
        name="camera_manager",                 # 节点名必须与YAML顶层名称一致
        output="screen",                       # 日志直接输出到当前终端，方便硬件联调
        parameters=[LaunchConfiguration("config")],  # 加载用户指定的相机参数文件
    )
    # 返回完整LaunchDescription。
    return LaunchDescription([config_path, node])
