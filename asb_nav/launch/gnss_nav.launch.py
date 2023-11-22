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
from launch.substitutions import LaunchConfiguration
from launch.actions import IncludeLaunchDescription, DeclareLaunchArgument
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.conditions import IfCondition
from launch_ros.actions import Node
from nav2_common.launch import RewrittenYaml


def generate_launch_description():
    pkg = get_package_share_directory

    use_sim_time_launch_configuration = LaunchConfiguration('use_sim_time')
    use_sim_time_launch_argument = DeclareLaunchArgument(
        'use_sim_time',
        default_value='false',
        description='Whether to set the use_sim_time parameter to true. Should be the same as the use_gazebo parameter.',
    )

    rviz_launch_configuration = LaunchConfiguration('rviz')
    rviz_launch_argument = DeclareLaunchArgument(
        'rviz',
        default_value='true',
        description='Whether to start RViz',
    )

    ekf_filter_node = Node(
        package="robot_localization",
        executable="ekf_node",
        name="ekf_filter_node_map",
        output="screen",
        parameters=[
            os.path.join(pkg("asb_nav"), "config", "gnss_odom_ekf.yaml"),
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
            os.path.join(pkg("asb_nav"), "config", "gnss_odom_ekf.yaml"),
            {"use_sim_time": use_sim_time_launch_configuration},
        ],
        remappings=[
            ("odometry/filtered", "odometry/global"),
        ],
    )

    nav2_bringup_include = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(os.path.join(pkg("nav2_bringup"), "launch", "navigation_launch.py")),
        launch_arguments={
            "use_sim_time": use_sim_time_launch_configuration,
            "params_file": os.path.join(pkg("asb_nav"), "config", "nav2_params_gnss_nav.yaml"),
            "autostart": "true",
        }.items(),
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
    # ld.add_action(robot_localization_cmd)
    ld.add_action(ekf_filter_node)
    ld.add_action(navsat_transform_node)

    # navigation
    ld.add_action(nav2_bringup_include)

    # viz
    ld.add_action(rviz_include)

    return ld
