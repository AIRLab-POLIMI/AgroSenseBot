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

    fake_localization_transform_publisher = Node(
        package="tf2_ros",
        executable="static_transform_publisher",
        arguments=["0", "0", "0", "0", "0", "0", "map", "odom"],
    )

    fake_scan_node = Node(
        package="asb_sim",
        executable="fake_scan_pub.py",
        name="fake_scan_publisher",
        output="screen",
    )

    nav2_bringup_include = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(os.path.join(pkg("asb_nav"), "launch", "nav2_navigation_launch.py")),
        launch_arguments={
            "use_sim_time": use_sim_time_launch_configuration,
            "params_file": os.path.join(pkg("asb_nav"), "config", "nav_blind", "nav2_params", "nav2_params.yaml"),
            "controller_params_file": os.path.join(pkg("asb_nav"), "config", "nav_blind", "nav2_params", "nav2_controller_params_mppi.yaml"),
            # "controller_params_file": os.path.join(pkg("asb_nav"), "config", "nav_blind", "nav2_params", "nav2_controller_params_rpp.yaml"),
            # "controller_params_file": os.path.join(pkg("asb_nav"), "config", "nav_blind", "nav2_params", "nav2_controller_params_dwb.yaml"),
            "planner_params_file": os.path.join(pkg("asb_nav"), "config", "nav_blind", "nav2_params", "nav2_planner_params_smac_hybrid.yaml"),
            # "planner_params_file": os.path.join(pkg("asb_nav"), "config", "nav_blind", "nav2_params", "nav2_planner_params_navfn.yaml"),
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
    ld.add_action(fake_localization_transform_publisher)

    # navigation
    ld.add_action(nav2_bringup_include)

    # sensors
    ld.add_action(fake_scan_node)

    # viz
    ld.add_action(rviz_include)

    return ld
