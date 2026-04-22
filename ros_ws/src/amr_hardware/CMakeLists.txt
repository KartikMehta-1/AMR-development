cmake_minimum_required(VERSION 3.5)
project(amr_hardware)

# Default to C++14
if(NOT CMAKE_CXX_STANDARD)
  set(CMAKE_CXX_STANDARD 14)
endif()

find_package(ament_cmake REQUIRED)
find_package(hardware_interface REQUIRED)
find_package(pluginlib REQUIRED)
find_package(rclcpp REQUIRED)
find_package(sensor_msgs REQUIRED)
find_package(std_msgs REQUIRED)

add_library(${PROJECT_NAME} SHARED
  src/amr_hardware.cpp
)

add_executable(amr_link_watchdog
  src/amr_link_watchdog.cpp
)

target_include_directories(${PROJECT_NAME}
  PUBLIC
    $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>
    $<INSTALL_INTERFACE:include>
)

ament_target_dependencies(${PROJECT_NAME}
  hardware_interface
  pluginlib
  rclcpp
  sensor_msgs
  std_msgs
)

ament_target_dependencies(amr_link_watchdog
  rclcpp
  sensor_msgs
  std_msgs
)

pluginlib_export_plugin_description_file(hardware_interface amr_hardware_plugins.xml)

install(
  DIRECTORY include/
  DESTINATION include
)

install(
  TARGETS ${PROJECT_NAME}
  amr_link_watchdog
  ARCHIVE DESTINATION lib
  LIBRARY DESTINATION lib
  RUNTIME DESTINATION bin
)

install(
  FILES amr_hardware_plugins.xml
  DESTINATION share/${PROJECT_NAME}
)

ament_package()
