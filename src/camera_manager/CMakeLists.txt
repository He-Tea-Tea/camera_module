# camera_manager：编排相机生命周期、缓存、PRIVATE ROS2端点和自动恢复，不直接暴露厂商SDK。
cmake_minimum_required(VERSION 3.8)
project(camera_manager)

# GCC/Clang开启常用警告。
if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

# 核心管理层始终依赖厂商无关契约和驱动抽象。
find_package(ament_cmake REQUIRED)
find_package(camera_adapter REQUIRED)
find_package(camera_driver REQUIRED)

# 不依赖ROS2消息的核心Registry库，允许在无ROS环境下手工编译Mock演示。
add_library(camera_manager_core SHARED
  src/camera_registry.cpp
  src/camera_health.cpp
)
target_compile_features(camera_manager_core PUBLIC cxx_std_17)
target_include_directories(camera_manager_core PUBLIC
  $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>
  $<INSTALL_INTERFACE:include>
)
ament_target_dependencies(camera_manager_core camera_adapter camera_driver)

# 无ROS2核心演示程序，用于快速验证缓存、3D查询和session UUID。
add_executable(camera_manager_demo src/camera_manager_demo.cpp)
target_compile_features(camera_manager_demo PUBLIC cxx_std_17)
target_link_libraries(camera_manager_demo camera_manager_core)
ament_target_dependencies(camera_manager_demo camera_adapter camera_driver)

# ROS2依赖采用QUIET探测，使核心Mock手工路径不强制依赖ROS2消息生成结果。
find_package(rclcpp QUIET)
find_package(rclcpp_action QUIET)
find_package(camera_msgs QUIET)
find_package(camera_interfaces QUIET)
if(rclcpp_FOUND AND rclcpp_action_FOUND AND camera_msgs_FOUND AND camera_interfaces_FOUND)
  # 真正的ROS2相机管理节点，创建PRIVATE/LOCAL_ONLY端点。
  add_executable(camera_manager_node src/camera_manager_node.cpp)
  target_compile_features(camera_manager_node PUBLIC cxx_std_17)
  target_link_libraries(camera_manager_node camera_manager_core)
  ament_target_dependencies(camera_manager_node
    rclcpp rclcpp_action camera_adapter camera_driver camera_msgs camera_interfaces)
  install(TARGETS camera_manager_node DESTINATION lib/${PROJECT_NAME})
endif()

# 安装核心库和Mock演示。
install(TARGETS camera_manager_core camera_manager_demo
  EXPORT camera_manager_targets
  ARCHIVE DESTINATION lib
  LIBRARY DESTINATION lib
  RUNTIME DESTINATION bin
)
# 安装公共管理器头文件。
install(DIRECTORY include/ DESTINATION include)
# 把根级config和launch一起安装到share，保证标准ros2 launch可找到默认参数。
install(DIRECTORY ../../config DESTINATION share/${PROJECT_NAME})
install(DIRECTORY ../../launch DESTINATION share/${PROJECT_NAME})
# 导出核心target和依赖。
ament_export_targets(camera_manager_targets HAS_LIBRARY_TARGET)
ament_export_dependencies(camera_adapter camera_driver)
ament_package()
