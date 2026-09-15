cmake_minimum_required(VERSION 3.8)
project(camera_driver)

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

find_package(ament_cmake REQUIRED)
find_package(camera_adapter REQUIRED)

option(CAMERA_DRIVER_ENABLE_ORBBEC "启用 Orbbec SDK v2" OFF)
option(CAMERA_DRIVER_ENABLE_REALSENSE "启用 librealsense" OFF)

add_library(camera_driver_core SHARED
  src/mock_camera_driver.cpp
  src/orbbec_camera_driver.cpp
  src/realsense_camera_driver.cpp
)

target_compile_features(camera_driver_core PUBLIC cxx_std_17)

target_include_directories(camera_driver_core PUBLIC
  $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>
  $<INSTALL_INTERFACE:include>
)

# ament_target_dependencies 内部使用 plain 签名
ament_target_dependencies(camera_driver_core camera_adapter)

if(CAMERA_DRIVER_ENABLE_ORBBEC)
  find_path(ORBBEC_INCLUDE_DIR libobsensor/ObSensor.hpp REQUIRED)
  find_library(ORBBEC_LIBRARY NAMES OrbbecSDK OrbbecSDK_v2 REQUIRED)

  target_compile_definitions(camera_driver_core PRIVATE CAMERA_DRIVER_HAS_ORBBEC=1)
  target_include_directories(camera_driver_core PRIVATE ${ORBBEC_INCLUDE_DIR})

  # 关键：去掉 PRIVATE，改成 plain 写法
  target_link_libraries(camera_driver_core ${ORBBEC_LIBRARY})
endif()

if(CAMERA_DRIVER_ENABLE_REALSENSE)
  find_path(REALSENSE_INCLUDE_DIR librealsense2/rs.hpp REQUIRED)
  find_library(REALSENSE_LIBRARY NAMES realsense2 REQUIRED)

  target_compile_definitions(camera_driver_core PRIVATE CAMERA_DRIVER_HAS_REALSENSE=1)
  target_include_directories(camera_driver_core PRIVATE ${REALSENSE_INCLUDE_DIR})

  # 关键：去掉 PRIVATE，改成 plain 写法
  target_link_libraries(camera_driver_core ${REALSENSE_LIBRARY})
endif()

install(TARGETS camera_driver_core
  EXPORT camera_driver_targets
  ARCHIVE DESTINATION lib
  LIBRARY DESTINATION lib
  RUNTIME DESTINATION bin
)

install(DIRECTORY include/ DESTINATION include)

ament_export_targets(camera_driver_targets HAS_LIBRARY_TARGET)
ament_export_include_directories(include)
ament_export_dependencies(camera_adapter)

ament_package()