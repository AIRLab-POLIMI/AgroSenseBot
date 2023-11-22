# Copyright 2018 Open Source Robotics Foundation, Inc.
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

from launch import LaunchDescription
from ament_index_python.packages import get_package_share_directory
import launch_ros.actions
import os
import launch.actions
from launch.actions import DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration


def generate_launch_description():
    # pkg = get_package_share_directory
    package_share_dir = get_package_share_directory("asb_nav")
    rl_params_file = os.path.join(package_share_dir, "config", "gnss_odom_ekf.yaml")

    use_sim_time_launch_configuration = LaunchConfiguration('use_sim_time')
    use_sim_time_launch_argument = DeclareLaunchArgument(
        'use_sim_time',
        default_value='false',
        description='Whether to set the use_sim_time parameter to true. Should be the same as the use_gazebo parameter.'
    )

    ekf_filter_node = launch_ros.actions.Node(
        package="robot_localization",
        executable="ekf_node",
        name="ekf_filter_node_map",
        output="screen",
        parameters=[
            rl_params_file,
            {"use_sim_time": use_sim_time_launch_configuration}
        ],
        remappings=[("odometry/filtered", "odometry/global")],
    )

    navsat_transform_node = launch_ros.actions.Node(
        package="robot_localization",
        executable="navsat_transform_node",
        name="navsat_transform",
        output="screen",
        parameters=[
            rl_params_file,
            {"use_sim_time": use_sim_time_launch_configuration},
        ],
        remappings=[
            ("odometry/filtered", "odometry/global"),
        ],
    )

    return LaunchDescription(
        [
            use_sim_time_launch_argument,
            ekf_filter_node,
            navsat_transform_node,
            # launch_ros.actions.Node(
            #     package="robot_localization",
            #     executable="ekf_node",
            #     name="ekf_filter_node_map",
            #     output="screen",
            #     parameters=[
            #         rl_params_file,
            #         {"use_sim_time": use_sim_time_launch_configuration}
            #     ],
            #     remappings=[("odometry/filtered", "odometry/global")],
            # ),
            # launch_ros.actions.Node(
            #     package="robot_localization",
            #     executable="navsat_transform_node",
            #     name="navsat_transform",
            #     output="screen",
            #     parameters=[
            #         rl_params_file,
            #         {"use_sim_time": use_sim_time_launch_configuration}
            #     ],
            #     remappings=[
            #         ("odometry/filtered", "odometry/global"),
            #     ],
            # ),
        ]
    )
