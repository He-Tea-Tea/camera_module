# camera_interfaces：只生成camera_module小脑本地PRIVATE/LOCAL_ONLY服务和Action。
cmake_minimum_required(VERSION 3.8)
project(camera_interfaces)

# ament负责包构建；rosidl负责生成C++/Python接口代码。
find_package(ament_cmake REQUIRED)
find_package(rosidl_default_generators REQUIRED)
# GetPoint3D和CaptureImage使用builtin_interfaces/Time。
find_package(builtin_interfaces REQUIRED)

# 显式列出全部私有rosidl接口，避免新增文件却漏出构建清单。
rosidl_generate_interfaces(${PROJECT_NAME}
  "action/CaptureImage.action"
  "srv/GetPoint3D.srv"
  "srv/QueryCameraCapability.srv"
  DEPENDENCIES builtin_interfaces
)

# 向消费者导出rosidl运行时和时间类型依赖。
ament_export_dependencies(rosidl_default_runtime builtin_interfaces)
# 完成ament包声明。
ament_package()
