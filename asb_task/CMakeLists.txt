cmake_minimum_required(VERSION 3.5)
project(asb_task)

# Default to C99
if(NOT CMAKE_C_STANDARD)
  set(CMAKE_C_STANDARD 99)
endif()

# Default to C++14
if(NOT CMAKE_CXX_STANDARD)
  set(CMAKE_CXX_STANDARD 14)
endif()

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

# find dependencies
find_package(ament_cmake REQUIRED)
find_package(rclcpp REQUIRED)
find_package(tf2 REQUIRED)
find_package(tf2_ros REQUIRED)
find_package(tf2_geometry_msgs REQUIRED)
find_package(asb_msgs REQUIRED)
find_package(geometry_msgs REQUIRED)
find_package(octomap_msgs REQUIRED)
find_package(visualization_msgs REQUIRED)
find_package(octomap REQUIRED)

add_executable(canopy_volume_estimation_node
    src/canopy_volume_estimation_node.cpp
    src/canopy_volume_estimation.cpp
)

target_include_directories(canopy_volume_estimation_node PUBLIC
    $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>
    $<INSTALL_INTERFACE:include>
)
target_compile_features(canopy_volume_estimation_node PUBLIC c_std_99 cxx_std_17 cxx_std_20)  # Require C99 and C++17

# Link the executable with the necessary libraries
ament_target_dependencies(canopy_volume_estimation_node
    rclcpp
    tf2
    tf2_ros
    tf2_geometry_msgs
    asb_msgs
    geometry_msgs
    octomap_msgs
    visualization_msgs
    octomap
)

install(
    TARGETS canopy_volume_estimation_node
    DESTINATION lib/${PROJECT_NAME})

install(
    DIRECTORY scripts/
    DESTINATION lib/${PROJECT_NAME}
)

install(
    DIRECTORY launch/
    DESTINATION share/${PROJECT_NAME}/launch/
)
install(
    DIRECTORY config/
    DESTINATION share/${PROJECT_NAME}/config/
)

if(BUILD_TESTING)
  find_package(ament_lint_auto REQUIRED)
  # the following line skips the linter which checks for copyrights
  # uncomment the line when a copyright and license is not present in all source files
  #set(ament_cmake_copyright_FOUND TRUE)
  # the following line skips cpplint (only works in a git repo)
  # uncomment the line when this package is not in a git repo
  #set(ament_cmake_cpplint_FOUND TRUE)
  ament_lint_auto_find_test_dependencies()
endif()

ament_package()
