cmake_minimum_required(VERSION 3.8)
project(odrive_ros2_control)

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

# find dependencies
find_package(ament_cmake REQUIRED)
find_package(rclcpp REQUIRED)
find_package(ament_cmake_auto REQUIRED)
find_package(std_msgs REQUIRED)
find_package(osc_utils REQUIRED)
find_package(osc_interfaces REQUIRED)
ament_auto_find_build_dependencies()

ament_export_dependencies(rosidl_default_runtime std_msgs osc_utils)


include_directories(include)
include_directories(../odrive_base/include)
include_directories(
  ${osc_utils_INCLUDE_DIRS}
)


ament_auto_add_library(
  odrive_ros2_control_plugin SHARED
  ../odrive_base/src/epoll_event_loop.cpp
  ../odrive_base/src/socket_can.cpp
  src/odrive_hardware_interface.cpp
)

ament_target_dependencies(odrive_ros2_control_plugin
  rclcpp
  std_msgs
  osc_utils
)

target_link_libraries(odrive_ros2_control_plugin ${osc_utils_LIBRARIES})

target_include_directories(
  odrive_ros2_control_plugin
  PRIVATE
  ${osc_utils_INCLUDE_DIRS}
)
pluginlib_export_plugin_description_file(hardware_interface odrive_hardware_interface.xml)

target_compile_features(odrive_ros2_control_plugin PRIVATE cxx_std_20)

install(TARGETS odrive_ros2_control_plugin
  ARCHIVE DESTINATION lib
  LIBRARY DESTINATION lib
  RUNTIME DESTINATION bin
)

ament_export_include_directories(
  include
)
ament_export_libraries(
  odrive_ros2_control_plugin
)

ament_package()
