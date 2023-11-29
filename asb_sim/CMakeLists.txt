cmake_minimum_required(VERSION 3.5)
project(asb_sim)

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

include(FindPkgConfig)
pkg_search_module(liblely-coapp REQUIRED liblely-coapp=2.3.2)  # 2.3.2 is the latest version from the lel PPA at the moment

# find dependencies
find_package(ament_cmake REQUIRED)
find_package(rclcpp REQUIRED)
find_package(rclcpp_lifecycle REQUIRED)
find_package(lifecycle_msgs REQUIRED)
find_package(asb_msgs REQUIRED)

add_executable(
        test_node
        src/main.cpp
        src/asb_system_test_node.cpp
        src/VCU_canopen_slave_node.cpp
        src/motor_drive_canopen_slave_node.cpp
)
target_compile_features(
        test_node
        PUBLIC c_std_99 cxx_std_17  # Require C99 and C++17
)
target_include_directories(
        test_node
        PUBLIC
        $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>
        $<INSTALL_INTERFACE:include>
)
target_link_libraries(test_node ${liblely-coapp_LIBRARIES})
ament_target_dependencies(
        test_node
        rclcpp
        liblely-coapp
        rclcpp_lifecycle
        lifecycle_msgs
        asb_msgs
)

install(
        TARGETS test_node
        DESTINATION lib/${PROJECT_NAME}
)
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
