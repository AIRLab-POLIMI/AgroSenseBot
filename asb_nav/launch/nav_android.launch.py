# Copyright (c) 2018 Intel Corporation
#
# Licensed under the Apache License, Version 2.0 (the "License");
# you may not use this file except in compliance with the License.
# You may obtain a copy of the License at
#
#     http://www.apache.org/licenses/LICENSE-2.0
#
# Unless required by applicable law or agreed to in writing, software
# distributed under the License is distributed on an "AS IS" BASIS,
# WITHOUT WARRANTIES OR CONDITIONS OF ANY KIND, either express or implied.
# See the License for the specific language governing permissions and
# limitations under the License.

import os

from ament_index_python.packages import get_package_share_directory

from launch import LaunchDescription
from launch.substitutions import LaunchConfiguration, PathJoinSubstitution, FindExecutable, Command
from launch.actions import IncludeLaunchDescription, DeclareLaunchArgument
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.conditions import IfCondition
from launch_ros.actions import Node


def generate_launch_description():
    pkg = get_package_share_directory

    use_sim_time_launch_configuration = LaunchConfiguration('use_sim_time')
    use_sim_time_launch_argument = DeclareLaunchArgument(
        'use_sim_time',
        default_value='false',
        description='Whether to set the use_sim_time parameter to true. Should only be set to true if using Gazebo (not Webots).',
    )

    rviz_launch_configuration = LaunchConfiguration('rviz')
    rviz_launch_argument = DeclareLaunchArgument(
        'rviz',
        default_value='true',
        description='Whether to start RViz',
    )

    # Get URDF via xacro
    robot_description_content = Command(
        [
            PathJoinSubstitution([FindExecutable(name="xacro")]),
            " ",
            PathJoinSubstitution([pkg("asb_ros2_control"), "urdf", "asb.urdf.xacro"]),
            " ",
            "test:=",
            "true",
        ]
    )

    robot_description = {"robot_description": robot_description_content}

    robot_state_publisher_node = Node(
        package="robot_state_publisher",
        executable="robot_state_publisher",
        name="robot_state_publisher",
        output="both",
        parameters=[robot_description],
    )

    fake_localization_transform_publisher = Node(
        package="tf2_ros",
        executable="static_transform_publisher",
        arguments=["0", "0", "0", "0", "0", "0", "odom", "base_footprint"],
    )

    ekf_filter_node = Node(
        package="robot_localization",
        executable="ekf_node",
        name="ekf_filter_map_odom",
        output="screen",
        parameters=[
            os.path.join(pkg("asb_nav"), "config", "nav_gnss", "robot_localization_params", "robot_localization_ekf_gnss_imu.yaml"),
            {"use_sim_time": use_sim_time_launch_configuration},
        ],
        remappings=[
            ("odometry/filtered", "odometry/global"),
        ],
    )

    navsat_transform_node = Node(
        package="robot_localization",
        executable="navsat_transform_node",
        name="navsat_transform",
        output="screen",
        parameters=[
            os.path.join(pkg("asb_nav"), "config", "nav_gnss", "robot_localization_params", "robot_localization_ekf_gnss_imu.yaml"),
            {"use_sim_time": use_sim_time_launch_configuration},
        ],
        remappings=[
            ("gps/fix", "gnss/fix"),
            ("odometry/filtered", "odometry/global"),
        ],
    )

    android_sensors_node = Node(
        package="asb_sim",
        executable="android_sensors.py",
        name="android_sensors",
        output="screen",
    )

    rviz_include = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(os.path.join(pkg("asb_nav"), "launch", "rviz.launch.py")),
        condition=IfCondition(rviz_launch_configuration)
    )

    # Create the launch description and populate
    ld = LaunchDescription()

    # launch arguments
    ld.add_action(use_sim_time_launch_argument)
    ld.add_action(rviz_launch_argument)

    # localization
    ld.add_action(robot_state_publisher_node)
    ld.add_action(fake_localization_transform_publisher)
    ld.add_action(ekf_filter_node)
    ld.add_action(navsat_transform_node)

    # sensors
    ld.add_action(android_sensors_node)

    # viz
    ld.add_action(rviz_include)

    return ld
