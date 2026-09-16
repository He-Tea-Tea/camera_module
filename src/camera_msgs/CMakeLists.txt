# camera_msgs：生成相机小脑私有状态消息；不属于V1.3公共DeviceState。
cmake_minimum_required(VERSION 3.8)
project(camera_msgs)

# ament和rosidl接口生成器。
find_package(ament_cmake REQUIRED)
find_package(rosidl_default_generators REQUIRED)
# CameraState携带builtin_interfaces/Time时间字段。
find_package(builtin_interfaces REQUIRED)

# 当前私有状态包只包含CameraState一个消息。
rosidl_generate_interfaces(${PROJECT_NAME}
  "msg/CameraState.msg"
  DEPENDENCIES builtin_interfaces
)

# 向运行时消费者导出rosidl和时间依赖。
ament_export_dependencies(rosidl_default_runtime builtin_interfaces)
ament_package()
