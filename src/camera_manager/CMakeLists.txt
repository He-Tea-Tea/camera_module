cmake_minimum_required(VERSION 3.8)
project(camera_manager)

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

find_package(ament_cmake REQUIRED)
find_package(camera_adapter REQUIRED)
find_package(camera_driver REQUIRED)

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

add_executable(camera_manager_demo src/camera_manager_demo.cpp)
target_compile_features(camera_manager_demo PUBLIC cxx_std_17)
target_link_libraries(camera_manager_demo camera_manager_core)
ament_target_dependencies(camera_manager_demo camera_adapter camera_driver)

find_package(rclcpp QUIET)
find_package(rclcpp_action QUIET)
find_package(camera_msgs QUIET)
find_package(camera_interfaces QUIET)
if(rclcpp_FOUND AND rclcpp_action_FOUND AND camera_msgs_FOUND AND camera_interfaces_FOUND)
  add_executable(camera_manager_node src/camera_manager_node.cpp)
  target_compile_features(camera_manager_node PUBLIC cxx_std_17)
  target_link_libraries(camera_manager_node camera_manager_core)
  ament_target_dependencies(camera_manager_node
    rclcpp rclcpp_action camera_adapter camera_driver camera_msgs camera_interfaces)
  install(TARGETS camera_manager_node DESTINATION lib/${PROJECT_NAME})
endif()

install(TARGETS camera_manager_core camera_manager_demo
  EXPORT camera_manager_targets
  ARCHIVE DESTINATION lib
  LIBRARY DESTINATION lib
  RUNTIME DESTINATION bin
)
install(DIRECTORY include/ DESTINATION include)
install(DIRECTORY ../../config DESTINATION share/${PROJECT_NAME})
install(DIRECTORY ../../launch DESTINATION share/${PROJECT_NAME})
ament_export_targets(camera_manager_targets HAS_LIBRARY_TARGET)
ament_export_dependencies(camera_adapter camera_driver)
ament_package()
