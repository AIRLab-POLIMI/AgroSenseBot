cmake_minimum_required(VERSION 3.16)
project(asb_ros2_control LANGUAGES CXX)

if(CMAKE_CXX_COMPILER_ID MATCHES "(GNU|Clang)")
  add_compile_options(-Wall -Wextra)
endif()

# find dependencies
set(THIS_PACKAGE_INCLUDE_DEPENDS
  hardware_interface
  pluginlib
  rclcpp
  rclcpp_lifecycle
  lely_core_libraries
)

# find dependencies
find_package(ament_cmake REQUIRED)
foreach(Dependency IN ITEMS ${THIS_PACKAGE_INCLUDE_DEPENDS})
  find_package(${Dependency} REQUIRED)
endforeach()


## COMPILE
add_library(
  asb_ros2_control
  SHARED
  hardware/asb_system.cpp
  hardware/canopen_slave_node.cpp
)
target_compile_features(asb_ros2_control PUBLIC cxx_std_17)
target_include_directories(asb_ros2_control PUBLIC
$<BUILD_INTERFACE:${PROJECT_SOURCE_DIR}/hardware/include>
$<INSTALL_INTERFACE:include/asb_ros2_control>
)
ament_target_dependencies(
  asb_ros2_control PUBLIC
  ${THIS_PACKAGE_INCLUDE_DEPENDS}
)

# Causes the visibility macros to use dllexport rather than dllimport,
# which is appropriate when building the dll but not consuming it.
target_compile_definitions(${PROJECT_NAME} PRIVATE "ASB_ROS2_CONTROL_BUILDING_DLL")

# Export hardware plugins
pluginlib_export_plugin_description_file(hardware_interface asb_ros2_control.xml)

# INSTALL
install(
  DIRECTORY hardware/include/
  DESTINATION include/asb_ros2_control
)
install(
  DIRECTORY description/launch description/ros2_control description/urdf
  DESTINATION share/asb_ros2_control
)
install(
  DIRECTORY bringup/launch bringup/config
  DESTINATION share/asb_ros2_control
)
install(TARGETS asb_ros2_control
  EXPORT export_asb_ros2_control
  ARCHIVE DESTINATION lib
  LIBRARY DESTINATION lib
  RUNTIME DESTINATION bin
)

if(BUILD_TESTING)
  find_package(ament_cmake_gtest REQUIRED)
endif()

## EXPORTS
ament_export_targets(export_asb_ros2_control HAS_LIBRARY_TARGET)
ament_export_dependencies(${THIS_PACKAGE_INCLUDE_DEPENDS})
ament_package()
