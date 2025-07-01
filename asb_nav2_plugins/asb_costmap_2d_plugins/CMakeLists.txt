cmake_minimum_required(VERSION 3.5)
project(asb_costmap_2d_plugins)

find_package(ament_cmake REQUIRED)
find_package(rclcpp REQUIRED)
find_package(nav2_costmap_2d REQUIRED)
find_package(asb_msgs REQUIRED)
find_package(pluginlib REQUIRED)

set(dependencies
    rclcpp
    nav2_costmap_2d
    asb_msgs
    pluginlib
)

include_directories(include)

add_library(asb_voxel_layer SHARED
    plugins/voxel_layer.cpp
    include/asb_costmap_2d_plugins/voxel_layer.hpp
    plugins/voxel_grid.cpp
    include/asb_voxel_grid/voxel_grid.hpp
)
ament_target_dependencies(asb_voxel_layer ${dependencies})
install(TARGETS asb_voxel_layer DESTINATION lib)

pluginlib_export_plugin_description_file(nav2_costmap_2d nav2_costmap_2d_plugins.xml)
ament_package()
