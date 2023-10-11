cmake_minimum_required(VERSION 3.16)
project(asb_control_system_status_controller LANGUAGES CXX)

if(CMAKE_CXX_COMPILER_ID MATCHES "(GNU|Clang)")
  add_compile_options(-Wall -Wextra)
endif()

set(THIS_PACKAGE_INCLUDE_DEPENDS
  controller_interface
  generate_parameter_library
  asb_msgs
  hardware_interface
  pluginlib
  rclcpp
  rclcpp_lifecycle
  rcpputils
  realtime_tools
)

find_package(ament_cmake REQUIRED)
find_package(backward_ros REQUIRED)
foreach(Dependency IN ITEMS ${THIS_PACKAGE_INCLUDE_DEPENDS})
  find_package(${Dependency} REQUIRED)
endforeach()

generate_parameter_library(asb_control_system_status_controller_parameters
  src/asb_control_system_status_controller_parameter.yaml
)

add_library(asb_control_system_status_controller SHARED
  src/asb_control_system_status_controller.cpp
)
target_compile_features(asb_control_system_status_controller PUBLIC cxx_std_17)
target_include_directories(asb_control_system_status_controller PUBLIC
  $<BUILD_INTERFACE:${PROJECT_SOURCE_DIR}/include>
  $<INSTALL_INTERFACE:include/asb_control_system_status_controller>
)
target_link_libraries(asb_control_system_status_controller PUBLIC asb_control_system_status_controller_parameters)
ament_target_dependencies(asb_control_system_status_controller PUBLIC ${THIS_PACKAGE_INCLUDE_DEPENDS})
# Causes the visibility macros to use dllexport rather than dllimport,
# which is appropriate when building the dll but not consuming it.
target_compile_definitions(asb_control_system_status_controller PRIVATE "ASB_CONTROL_SYSTEM_STATUS_CONTROLLER_BUILDING_DLL")
pluginlib_export_plugin_description_file(controller_interface asb_control_system_status_plugin.xml)

#if(BUILD_TESTING)
#  find_package(ament_cmake_gmock REQUIRED)
#  find_package(controller_manager REQUIRED)
#  find_package(ros2_control_test_assets REQUIRED)
#
#  ament_add_gmock(test_asb_control_system_status_controller
#    test/test_asb_control_system_status_controller.cpp
#    ENV config_file=${CMAKE_CURRENT_SOURCE_DIR}/test/config/test_asb_control_system_status_controller.yaml)
#  target_link_libraries(test_asb_control_system_status_controller
#    asb_control_system_status_controller
#  )
#  ament_target_dependencies(test_asb_control_system_status_controller
#    geometry_msgs
#    hardware_interface
#    nav_msgs
#    rclcpp
#    rclcpp_lifecycle
#    realtime_tools
#    tf2
#    tf2_msgs
#  )
#
#  ament_add_gmock(test_load_asb_control_system_status_controller
#    test/test_load_asb_control_system_status_controller.cpp
#  )
#  ament_target_dependencies(test_load_asb_control_system_status_controller
#    controller_manager
#    ros2_control_test_assets
#  )
#endif()

install(
  DIRECTORY include/
  DESTINATION include/asb_control_system_status_controller
)
install(TARGETS asb_control_system_status_controller asb_control_system_status_controller_parameters
  EXPORT export_asb_control_system_status_controller
  RUNTIME DESTINATION bin
  ARCHIVE DESTINATION lib
  LIBRARY DESTINATION lib
)

ament_export_targets(export_asb_control_system_status_controller HAS_LIBRARY_TARGET)
ament_export_dependencies(${THIS_PACKAGE_INCLUDE_DEPENDS})
ament_package()