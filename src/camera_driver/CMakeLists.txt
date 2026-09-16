# camera_driver：统一封装Mock、Orbbec SDK v2和librealsense后端。
cmake_minimum_required(VERSION 3.8)
project(camera_driver)

# GCC或Clang编译时开启常用警告，便于尽早发现潜在C++类型和接口问题。
if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(
    -Wall
    -Wextra
    -Wpedantic
  )
endif()

# ROS2 ament构建系统。
find_package(ament_cmake REQUIRED)
# 厂商无关的相机数据契约。
find_package(camera_adapter REQUIRED)

# 默认不启用任何真实厂商SDK，由具体构建脚本决定启用哪个后端。
option(CAMERA_DRIVER_ENABLE_ORBBEC "启用 Orbbec SDK v2" OFF)
option(CAMERA_DRIVER_ENABLE_REALSENSE "启用 librealsense" OFF)

# 相机驱动核心动态库；未启用SDK的真实后端会编译为明确的UNSUPPORTED_BACKEND失败路径。
add_library(camera_driver_core SHARED
  src/mock_camera_driver.cpp
  src/orbbec_camera_driver.cpp
  src/realsense_camera_driver.cpp
)
# 项目统一使用C++17。
target_compile_features(camera_driver_core PUBLIC cxx_std_17)
# camera_driver自己的公共头文件。
target_include_directories(camera_driver_core PUBLIC
  $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>
  $<INSTALL_INTERFACE:include>
)
# 依赖厂商无关的camera_adapter。
ament_target_dependencies(camera_driver_core camera_adapter)

# -------------------- Orbbec 后端 --------------------
if(CAMERA_DRIVER_ENABLE_ORBBEC)
  # 查找Orbbec SDK v2头文件。
  find_path(ORBBEC_INCLUDE_DIR libobsensor/ObSensor.hpp REQUIRED)
  # 查找Orbbec SDK动态库。
  find_library(ORBBEC_LIBRARY NAMES OrbbecSDK OrbbecSDK_v2 REQUIRED)

  # 该宏会改变OrbbecCameraDriver头文件中的类内存布局。
  # v0.2.3已经实机证明必须使用PUBLIC，让camera_manager等消费者看到完全一致的类定义。
  target_compile_definitions(camera_driver_core PUBLIC CAMERA_DRIVER_HAS_ORBBEC=1)
  # 消费者包含orbbec_camera_driver.hpp时也会包含ObSensor.hpp，因此SDK include目录当前必须公开。
  target_include_directories(camera_driver_core PUBLIC ${ORBBEC_INCLUDE_DIR})
  # 与ament_target_dependencies使用相同的plain signature，避免ROS2 Humble下签名混用错误。
  target_link_libraries(camera_driver_core ${ORBBEC_LIBRARY})
endif()

# -------------------- RealSense 后端 --------------------
if(CAMERA_DRIVER_ENABLE_REALSENSE)
  # 查找librealsense2头文件。
  find_path(REALSENSE_INCLUDE_DIR librealsense2/rs.hpp REQUIRED)
  # 查找librealsense2动态库。
  find_library(REALSENSE_LIBRARY NAMES realsense2 REQUIRED)

  # 与Orbbec相同，该宏会改变RealSense驱动类布局，因此当前必须PUBLIC传播给消费者。
  target_compile_definitions(camera_driver_core PUBLIC CAMERA_DRIVER_HAS_REALSENSE=1)
  # 消费者包含realsense_camera_driver.hpp时需要能够找到SDK头文件。
  target_include_directories(camera_driver_core PUBLIC ${REALSENSE_INCLUDE_DIR})
  # 链接librealsense2。
  target_link_libraries(camera_driver_core ${REALSENSE_LIBRARY})
endif()

# -------------------- 安装与导出 --------------------
# 安装camera_driver_core动态库。
install(TARGETS camera_driver_core
  EXPORT camera_driver_targets
  ARCHIVE DESTINATION lib
  LIBRARY DESTINATION lib
  RUNTIME DESTINATION bin
)
# 安装公共头文件。
install(DIRECTORY include/ DESTINATION include)
# 导出动态库目标，使camera_manager等ROS2包能够正常链接。
ament_export_targets(camera_driver_targets HAS_LIBRARY_TARGET)
# 导出camera_driver公共头文件目录。
ament_export_include_directories(include)
# 导出camera_adapter依赖。
ament_export_dependencies(camera_adapter)
# 完成ament包声明。
ament_package()
