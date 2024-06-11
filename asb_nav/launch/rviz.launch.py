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
from launch.actions import DeclareLaunchArgument
from launch_ros.actions import Node
from launch.substitutions import LaunchConfiguration


def generate_launch_description():
    pkg = get_package_share_directory

    use_sim_time_launch_configuration = LaunchConfiguration('use_sim_time')
    use_sim_time_launch_argument = DeclareLaunchArgument(
        'use_sim_time',
        default_value='false',
        description='Whether to set the use_sim_time parameter to true. Should only be set to true if using Gazebo (not Webots).',
    )

    # Launch rviz
    start_rviz_cmd = Node(
        package='rviz2',
        executable='rviz2',
        arguments=['-d', os.path.join(pkg('asb_nav'), 'config', 'nav.rviz')],
        output='screen',
        parameters=[
            {"use_sim_time": use_sim_time_launch_configuration},
        ],
    )

    # Create the launch description and populate
    ld = LaunchDescription()
    ld.add_action(use_sim_time_launch_argument)
    ld.add_action(start_rviz_cmd)

    return ld
