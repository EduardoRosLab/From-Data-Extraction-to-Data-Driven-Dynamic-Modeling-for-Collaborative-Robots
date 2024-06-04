cmake_minimum_required(VERSION 3.5)
project(external_controller)

# Default to C++14
if(NOT CMAKE_CXX_STANDARD)
  set(CMAKE_CXX_STANDARD 14)
endif()

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

# find dependencies
find_package(ament_cmake REQUIRED)
find_package(controller_interface REQUIRED)
find_package(hardware_interface REQUIRED)
find_package(pluginlib REQUIRED)
find_package(rclcpp REQUIRED)
find_package(rclcpp_lifecycle REQUIRED)
find_package(realtime_tools REQUIRED)
find_package(custom_msg REQUIRED)

add_library(external_controller
  SHARED
  src/external_controller.cpp
  )

target_include_directories(external_controller PRIVATE include)

ament_target_dependencies(external_controller
  builtin_interfaces
  controller_interface
  hardware_interface
  pluginlib
  rclcpp_lifecycle
  rcutils
  realtime_tools
  custom_msg
  )
# Causes the visibility macros to use dllexport rather than dllimport,
# which is appropriate when building the dll but not consuming it.
target_compile_definitions(external_controller PRIVATE "EXTERNAL_CONTROLLER_BUILDING_DLL")
# prevent pluginlib from using boost
target_compile_definitions(external_controller PUBLIC "PLUGINLIB__DISABLE_BOOST_FUNCTIONS")
pluginlib_export_plugin_description_file(controller_interface external_plugin.xml)

install(
  DIRECTORY include/
  DESTINATION include
  )

install(TARGETS
  external_controller
  RUNTIME DESTINATION bin
  ARCHIVE DESTINATION lib
  LIBRARY DESTINATION lib
  )

ament_export_dependencies(
  controller_interface
  hardware_interface
  rclcpp
  rclcpp_lifecycle
  realtime_tools
  )
ament_export_include_directories(
  include
  )
ament_export_libraries(
  external_controller
  )
ament_package()
