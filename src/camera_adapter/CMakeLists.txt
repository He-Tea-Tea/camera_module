# camera_adapter：只包含厂商无关的C++17数据契约、时间逻辑和帧缓存。
cmake_minimum_required(VERSION 3.8)
project(camera_adapter)

# GCC/Clang开启常用告警，尽早暴露类型、生命周期和未使用变量问题。
if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

# ament_cmake负责ROS2包构建和导出。
find_package(ament_cmake REQUIRED)

# 核心动态库由统一相机契约和有界帧缓存两部分组成。
add_library(camera_adapter_core SHARED
  src/camera_adapter.cpp
  src/frame_buffer.cpp
)
# 统一要求C++17，后续camera_driver/camera_manager继承相同语言基线。
target_compile_features(camera_adapter_core PUBLIC cxx_std_17)
# 构建期和安装期分别暴露公共头文件目录。
target_include_directories(camera_adapter_core PUBLIC
  $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>
  $<INSTALL_INTERFACE:include>
)

# 安装动态库并导出CMake target，供camera_driver和camera_manager链接。
install(TARGETS camera_adapter_core
  EXPORT camera_adapter_targets
  ARCHIVE DESTINATION lib
  LIBRARY DESTINATION lib
  RUNTIME DESTINATION bin
)
# 安装所有厂商无关公共头文件。
install(DIRECTORY include/ DESTINATION include)
# 导出library target和include目录。
ament_export_targets(camera_adapter_targets HAS_LIBRARY_TARGET)
ament_export_include_directories(include)
# 完成ament包声明。
ament_package()
