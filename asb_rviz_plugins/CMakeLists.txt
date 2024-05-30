cmake_minimum_required(VERSION 3.5)
project(asb_rviz_plugins)

# Default to C++14
if(NOT CMAKE_CXX_STANDARD)
  set(CMAKE_CXX_STANDARD 20)
endif()

if(CMAKE_CXX_COMPILER_ID MATCHES "GNU" OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic -Werror)
endif()

set(Qwt_DIR cmake)

# Qt5 boilerplate options from http://doc.qt.io/qt-5/cmake-manual.html
set(CMAKE_INCLUDE_CURRENT_DIR ON)
set(CMAKE_AUTOMOC ON)

find_package(ament_cmake REQUIRED)
find_package(asb_msgs REQUIRED)
find_package(microstrain_inertial_msgs REQUIRED)
find_package(pluginlib REQUIRED)
find_package(Qt5 REQUIRED COMPONENTS Core Gui Widgets Test Concurrent)
find_package(Qwt REQUIRED)
find_package(rclcpp REQUIRED)
find_package(rclcpp_lifecycle REQUIRED)
find_package(rviz2 REQUIRED)
find_package(rviz_common REQUIRED)
find_package(rviz_default_plugins REQUIRED)
find_package(rviz_ogre_vendor REQUIRED)
find_package(rviz_rendering REQUIRED)
find_package(yaml_cpp_vendor REQUIRED)

set (THIS_PACKAGE_INCLUDE_DEPENDS
    asb_msgs
    microstrain_inertial_msgs
    rclcpp
    class_loader
    pluginlib
    Qt5
    Qwt
    rviz2
    rviz_common
    rviz_default_plugins
    rviz_rendering
    rviz_ogre_vendor
)

include_directories(
        include
        ${QWT_INCLUDE_DIRS}
)

# Define source file
set(${PROJECT_NAME}_SRCS
        src/asb_panel.cpp
        src/asb_widget.cpp
        src/asb_linear_dial.cpp
)

# Define header file
set(${PROJECT_NAME}_HDRS
        include/${PROJECT_NAME}/asb_panel.hpp
        include/${PROJECT_NAME}/asb_widget.h
        include/${PROJECT_NAME}/asb_linear_dial.h
)

# Define ui file
set(${PROJECT_NAME}_UIS
        resources/asb.ui
)

message(STATUS "Generate header for ui with rviz2_QT_VERSION: ${rviz2_QT_VERSION}")
qt5_wrap_ui(${PROJECT_NAME}_UIS_H ${${PROJECT_NAME}_UIS})
foreach(header "${${PROJECT_NAME}_HDRS}")
    qt5_wrap_cpp(${PROJECT_NAME}_MOCS ${header})
endforeach()

## Add library is needed in order to generate the header file from ui file.
add_library(${PROJECT_NAME} SHARED
        ${${PROJECT_NAME}_SRCS}
        ${${PROJECT_NAME}_HDRS}
        ${${PROJECT_NAME}_UIS_H}
        ${${PROJECT_NAME}_MOCS}
)
ament_target_dependencies(${PROJECT_NAME} ${THIS_PACKAGE_INCLUDE_DEPENDS})
target_include_directories(${PROJECT_NAME} PUBLIC
        ${Qt5Widgets_INCLUDE_DIRS}
        ${OGRE_INCLUDE_DIRS}
)
target_link_libraries(${PROJECT_NAME} rviz_common::rviz_common Qwt::Qwt)
target_compile_definitions(${PROJECT_NAME} PUBLIC "PLUGINLIB__DISABLE_BOOST_FUNCTIONS")
target_compile_definitions(${PROJECT_NAME} PRIVATE "RVIZ_DEFAULT_PLUGINS_BUILDING_LIBRARY")


pluginlib_export_plugin_description_file(rviz_common plugins_description.xml)

install(
        TARGETS ${PROJECT_NAME}
        EXPORT ${PROJECT_NAME}
        LIBRARY DESTINATION lib
        ARCHIVE DESTINATION lib
        RUNTIME DESTINATION bin
        INCLUDES DESTINATION include
)

install(
        DIRECTORY include/
        DESTINATION include
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

ament_export_include_directories(include)
ament_export_libraries(${PROJECT_NAME})
ament_export_targets(${PROJECT_NAME} HAS_LIBRARY_TARGET)
ament_export_dependencies(${THIS_PACKAGE_INCLUDE_DEPENDS})

ament_package()