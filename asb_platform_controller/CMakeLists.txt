cmake_minimum_required(VERSION 3.16)
project(asb_platform_controller LANGUAGES CXX)

if(CMAKE_CXX_COMPILER_ID MATCHES "(GNU|Clang)")
  add_compile_options(-Wall -Wextra)
endif()

set(THIS_PACKAGE_INCLUDE_DEPENDS
  controller_interface
  asb_msgs
  hardware_interface
  pluginlib
  rclcpp
  rclcpp_lifecycle
  rcpputils
)

find_package(ament_cmake REQUIRED)
foreach(Dependency IN ITEMS ${THIS_PACKAGE_INCLUDE_DEPENDS})
  find_package(${Dependency} REQUIRED)
endforeach()

add_library(asb_platform_controller SHARED
  src/asb_platform_controller.cpp
)
target_compile_features(asb_platform_controller PUBLIC cxx_std_17)
target_include_directories(asb_platform_controller PUBLIC
  $<BUILD_INTERFACE:${PROJECT_SOURCE_DIR}/include>
  $<INSTALL_INTERFACE:include/asb_platform_controller>
)
ament_target_dependencies(asb_platform_controller PUBLIC ${THIS_PACKAGE_INCLUDE_DEPENDS})
# Causes the visibility macros to use dllexport rather than dllimport,
# which is appropriate when building the dll but not consuming it.
target_compile_definitions(asb_platform_controller PRIVATE "ASB_PLATFORM_CONTROLLER_BUILDING_DLL")
pluginlib_export_plugin_description_file(controller_interface asb_platform_plugin.xml)

install(
  DIRECTORY include/
  DESTINATION include/asb_platform_controller
)
install(TARGETS asb_platform_controller
  EXPORT export_asb_platform_controller
  RUNTIME DESTINATION bin
  ARCHIVE DESTINATION lib
  LIBRARY DESTINATION lib
)

ament_export_targets(export_asb_platform_controller HAS_LIBRARY_TARGET)
ament_export_dependencies(${THIS_PACKAGE_INCLUDE_DEPENDS})
ament_package()
