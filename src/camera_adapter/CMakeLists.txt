cmake_minimum_required(VERSION 3.8)
project(camera_adapter)

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

find_package(ament_cmake REQUIRED)

add_library(camera_adapter_core SHARED
  src/camera_adapter.cpp
  src/frame_buffer.cpp
)
target_compile_features(camera_adapter_core PUBLIC cxx_std_17)
target_include_directories(camera_adapter_core PUBLIC
  $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>
  $<INSTALL_INTERFACE:include>
)

install(TARGETS camera_adapter_core
  EXPORT camera_adapter_targets
  ARCHIVE DESTINATION lib
  LIBRARY DESTINATION lib
  RUNTIME DESTINATION bin
)
install(DIRECTORY include/ DESTINATION include)
ament_export_targets(camera_adapter_targets HAS_LIBRARY_TARGET)
ament_export_include_directories(include)
ament_package()
