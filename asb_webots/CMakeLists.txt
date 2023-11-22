cmake_minimum_required(VERSION 3.5)
project(asb_webots)

if(NOT CMAKE_CXX_STANDARD)
  set(CMAKE_CXX_STANDARD 14)
endif()

find_package(ament_cmake REQUIRED)
find_package(rclcpp REQUIRED)
find_package(std_msgs REQUIRED)
find_package(geometry_msgs REQUIRED)
find_package(nav_msgs REQUIRED)
find_package(asb_msgs REQUIRED)
find_package(pluginlib REQUIRED)
find_package(webots_ros2_driver REQUIRED)

pluginlib_export_plugin_description_file(webots_ros2_driver asb_webots_driver.xml)

add_library(
        ${PROJECT_NAME}
        SHARED
        src/ASBWebotsDriver.cpp
)
target_include_directories(
        ${PROJECT_NAME}
        PRIVATE
        include
)
ament_target_dependencies(
        ${PROJECT_NAME}
        pluginlib
        rclcpp
        nav_msgs
        asb_msgs
        webots_ros2_driver
)
install(TARGETS
        ${PROJECT_NAME}
        ARCHIVE DESTINATION lib
        LIBRARY DESTINATION lib
        RUNTIME DESTINATION bin
)

install(DIRECTORY
        launch
        resource
        worlds
        DESTINATION share/${PROJECT_NAME}/
)
ament_export_include_directories(
        include
)
ament_export_libraries(
        ${PROJECT_NAME}
)
ament_package()