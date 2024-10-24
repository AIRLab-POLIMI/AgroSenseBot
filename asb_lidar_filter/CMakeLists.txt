cmake_minimum_required(VERSION 3.8)
project(asb_lidar_filter)

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

# find dependencies
find_package(ament_cmake REQUIRED)
find_package(rclcpp REQUIRED)
find_package(pcl_conversions REQUIRED)
find_package(pcl_ros REQUIRED)
find_package(tf2 REQUIRED)
find_package(tf2_ros REQUIRED)
find_package(tf2_geometry_msgs REQUIRED)
find_package(geometry_msgs REQUIRED)

add_executable(asb_lidar_filter_node
    src/asb_lidar_filter_node.cpp
    src/asb_lidar_filter.cpp
)

target_include_directories(asb_lidar_filter_node PUBLIC
    $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>
    $<INSTALL_INTERFACE:include>
)
target_compile_features(asb_lidar_filter_node PUBLIC c_std_99 cxx_std_17)  # Require C99 and C++17

# Link the executable with the necessary libraries
ament_target_dependencies(asb_lidar_filter_node
    rclcpp
    pcl_conversions
    pcl_ros
    tf2
    tf2_ros
    geometry_msgs
)

install(
    TARGETS asb_lidar_filter_node
    DESTINATION lib/${PROJECT_NAME})

install(
    DIRECTORY config
    DESTINATION share/${PROJECT_NAME}
)

if(BUILD_TESTING)
  find_package(ament_lint_auto REQUIRED)
  # the following line skips the linter which checks for copyrights
  # comment the line when a copyright and license is added to all source files
  set(ament_cmake_copyright_FOUND TRUE)
  # the following line skips cpplint (only works in a git repo)
  # comment the line when this package is in a git repo and when
  # a copyright and license is added to all source files
  set(ament_cmake_cpplint_FOUND TRUE)
  ament_lint_auto_find_test_dependencies()
endif()

ament_package()
