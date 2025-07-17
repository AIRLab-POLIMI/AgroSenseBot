cmake_minimum_required(VERSION 3.5)
project(asb_regulated_pure_pursuit_controller)

find_package(ament_cmake REQUIRED)
find_package(nav2_common REQUIRED)
find_package(nav2_core REQUIRED)
find_package(nav2_costmap_2d REQUIRED)
find_package(nav2_util REQUIRED)
find_package(rclcpp REQUIRED)
find_package(geometry_msgs REQUIRED)
find_package(nav_msgs REQUIRED)
find_package(pluginlib REQUIRED)
find_package(tf2 REQUIRED)
find_package(tf2_geometry_msgs REQUIRED)

nav2_package()
set(CMAKE_CXX_STANDARD 17)

include_directories(
    include
)

set(dependencies
    rclcpp
    geometry_msgs
    nav2_costmap_2d
    pluginlib
    nav_msgs
    nav2_util
    nav2_core
    tf2
    tf2_geometry_msgs
)

set(library_name asb_regulated_pure_pursuit_controller)

add_library(${library_name} SHARED
    src/regulated_pure_pursuit_controller.cpp
    include/asb_regulated_pure_pursuit_controller/regulated_pure_pursuit_controller.hpp

    src/collision_checker.cpp
    include/asb_regulated_pure_pursuit_controller/collision_checker.hpp

    src/parameter_handler.cpp
    include/asb_regulated_pure_pursuit_controller/parameter_handler.hpp

    src/path_handler.cpp
    include/asb_regulated_pure_pursuit_controller/path_handler.hpp

    include/asb_regulated_pure_pursuit_controller/regulation_functions.hpp
)

ament_target_dependencies(${library_name}
    ${dependencies}
)

install(TARGETS ${library_name}
    ARCHIVE DESTINATION lib
    LIBRARY DESTINATION lib
    RUNTIME DESTINATION bin
)

install(DIRECTORY include/
    DESTINATION include/
)

ament_export_include_directories(include)
ament_export_libraries(${library_name})
ament_export_dependencies(${dependencies})

pluginlib_export_plugin_description_file(nav2_core asb_regulated_pure_pursuit_controller.xml)

ament_package()

