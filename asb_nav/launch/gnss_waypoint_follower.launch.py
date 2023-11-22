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
from nav2_common.launch import RewrittenYaml


def generate_launch_description():
    pkg = get_package_share_directory

    nav2_params = os.path.join(pkg("asb_nav"), "config", "nav2_no_map_params.yaml")
    configured_params = RewrittenYaml(source_file=nav2_params, root_key="", param_rewrites={}, convert_types=True)

    use_sim_time_launch_configuration = LaunchConfiguration('use_sim_time')
    use_sim_time_launch_argument = DeclareLaunchArgument(
        'use_sim_time',
        default_value='false',
        description='Whether to set the use_sim_time parameter to true. Should be the same as the use_gazebo parameter.'
    )

    use_gazebo_launch_configuration = LaunchConfiguration('use_gazebo')
    use_gazebo_launch_argument = DeclareLaunchArgument(
        'use_gazebo',
        default_value='false',
        description='Whether to use Gazebo as simulator'
    )

    use_webots_launch_configuration = LaunchConfiguration('use_webots')
    use_webots_launch_argument = DeclareLaunchArgument(
        'use_webots',
        default_value='true',
        description='Whether to use Webots as simulator'
    )

    rviz_launch_configuration = LaunchConfiguration('rviz')
    rviz_launch_argument = DeclareLaunchArgument(
        'rviz',
        default_value='true',
        description='Whether to start RViz'
    )

    gazebo_launch = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(os.path.join(pkg("asb_gazebo"), "launch", "sim.launch.py")),
        condition=IfCondition(use_gazebo_launch_configuration)
    )

    webots_launch = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(os.path.join(pkg("asb_webots"), "launch", "sim.launch.py")),
        condition=IfCondition(use_webots_launch_configuration)
    )

    robot_localization_cmd = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(os.path.join(pkg("asb_nav"), "launch", "dual_ekf_navsat.launch.py"))
    )

    navigation2_cmd = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(os.path.join(pkg("nav2_bringup"), "launch", "navigation_launch.py")),
        launch_arguments={
            "use_sim_time": use_sim_time_launch_configuration,
            "params_file": configured_params,
            "autostart": "true",
        }.items(),
    )

    rviz_cmd = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(os.path.join(pkg("asb_nav"), "launch", "rviz.launch.py")),
        condition=IfCondition(rviz_launch_configuration)
    )

    # Create the launch description and populate
    ld = LaunchDescription()

    # launch arguments
    ld.add_action(use_sim_time_launch_argument)
    ld.add_action(use_gazebo_launch_argument)
    ld.add_action(use_webots_launch_argument)
    ld.add_action(rviz_launch_argument)

    # viz
    ld.add_action(rviz_cmd)

    # sim
    ld.add_action(gazebo_launch)
    ld.add_action(webots_launch)

    # localization
    ld.add_action(robot_localization_cmd)

    # navigation
    ld.add_action(navigation2_cmd)

    return ld
