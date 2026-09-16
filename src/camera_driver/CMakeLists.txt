cmake_minimum_required(VERSION 3.8)
project(camera_driver)

# GCC 或 Clang 编译时开启常用警告，
# 便于尽早发现潜在的 C++ 类型和接口问题。
if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(
    -Wall
    -Wextra
    -Wpedantic
  )
endif()

# ROS2 ament 构建系统。
find_package(ament_cmake REQUIRED)

# 相机厂商无关的数据契约。
find_package(camera_adapter REQUIRED)

# 默认不启用任何真实厂商 SDK，
# 由具体构建脚本决定启用哪个后端。
option(
  CAMERA_DRIVER_ENABLE_ORBBEC
  "启用 Orbbec SDK v2"
  OFF
)

option(
  CAMERA_DRIVER_ENABLE_REALSENSE
  "启用 librealsense"
  OFF
)

# 相机驱动核心动态库。
add_library(
  camera_driver_core
  SHARED
  src/mock_camera_driver.cpp
  src/orbbec_camera_driver.cpp
  src/realsense_camera_driver.cpp
)

# 项目统一使用 C++17。
target_compile_features(
  camera_driver_core
  PUBLIC cxx_std_17
)

# camera_driver 自己的公共头文件。
target_include_directories(
  camera_driver_core
  PUBLIC
  $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>
  $<INSTALL_INTERFACE:include>
)

# 依赖厂商无关的 camera_adapter。
ament_target_dependencies(
  camera_driver_core
  camera_adapter
)

# -------------------- Orbbec 后端 --------------------

if(CAMERA_DRIVER_ENABLE_ORBBEC)

  # 查找 Orbbec SDK v2 头文件。
  find_path(
    ORBBEC_INCLUDE_DIR
    libobsensor/ObSensor.hpp
    REQUIRED
  )

  # 查找 Orbbec SDK 动态库。
  find_library(
    ORBBEC_LIBRARY
    NAMES
    OrbbecSDK
    OrbbecSDK_v2
    REQUIRED
  )

  # 该宏会改变 OrbbecCameraDriver 头文件中的类内存布局。
  # 因此必须使用 PUBLIC，让 camera_manager 等消费者看到完全一致的类定义。
  target_compile_definitions(
    camera_driver_core
    PUBLIC CAMERA_DRIVER_HAS_ORBBEC=1
  )

  # 消费者在包含 orbbec_camera_driver.hpp 时，
  # 也会包含 libobsensor/ObSensor.hpp，所以该目录必须公开。
  target_include_directories(
    camera_driver_core
    PUBLIC ${ORBBEC_INCLUDE_DIR}
  )

  # 与 ament_target_dependencies 使用相同的 plain signature，
  # 避免 ROS2 Humble 下 target_link_libraries 写法冲突。
  target_link_libraries(
    camera_driver_core
    ${ORBBEC_LIBRARY}
  )

endif()

# -------------------- RealSense 后端 --------------------

if(CAMERA_DRIVER_ENABLE_REALSENSE)

  # 查找 librealsense2 头文件。
  find_path(
    REALSENSE_INCLUDE_DIR
    librealsense2/rs.hpp
    REQUIRED
  )

  # 查找 librealsense2 动态库。
  find_library(
    REALSENSE_LIBRARY
    NAMES realsense2
    REQUIRED
  )

  # 与 Orbbec 相同，该宏会改变 RealSense 驱动类的内存布局，
  # 因此必须公开给所有消费者。
  target_compile_definitions(
    camera_driver_core
    PUBLIC CAMERA_DRIVER_HAS_REALSENSE=1
  )

  # 消费者需要能够找到 librealsense2 的公共头文件。
  target_include_directories(
    camera_driver_core
    PUBLIC ${REALSENSE_INCLUDE_DIR}
  )

  # 链接 librealsense2。
  target_link_libraries(
    camera_driver_core
    ${REALSENSE_LIBRARY}
  )

endif()

# -------------------- 安装与导出 --------------------

# 安装 camera_driver_core 动态库。
install(
  TARGETS camera_driver_core
  EXPORT camera_driver_targets
  ARCHIVE DESTINATION lib
  LIBRARY DESTINATION lib
  RUNTIME DESTINATION bin
)

# 安装公共头文件。
install(
  DIRECTORY include/
  DESTINATION include
)

# 导出动态库目标，
# 使 camera_manager 等 ROS2 包能够正常链接。
ament_export_targets(
  camera_driver_targets
  HAS_LIBRARY_TARGET
)

# 导出 camera_driver 公共头文件目录。
ament_export_include_directories(
  include
)

# 导出 camera_adapter 依赖。
ament_export_dependencies(
  camera_adapter
)

ament_package()