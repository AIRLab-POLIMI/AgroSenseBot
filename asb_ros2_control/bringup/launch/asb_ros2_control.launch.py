# Copyright 2020 ros2_control Development Team
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

from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, RegisterEventHandler, IncludeLaunchDescription
from launch.conditions import IfCondition
from launch.event_handlers import OnProcessExit
from launch.substitutions import Command, FindExecutable, PathJoinSubstitution, LaunchConfiguration
from launch.launch_description_sources import PythonLaunchDescriptionSource

from launch_ros.actions import Node
from ament_index_python import get_package_share_directory as pkg


def generate_launch_description():

    use_vcan0_launch_configuration = LaunchConfiguration("use_vcan0")
    use_vcan0_launch_argument = DeclareLaunchArgument(
        "use_vcan0",
        default_value="false",
        description="Use the virtual CAN network vcan0 instead of the physical CAN network (can0).",
    )

    fake_heartbeat_launch_configuration = LaunchConfiguration("fake_heartbeat")
    fake_heartbeat_launch_argument = DeclareLaunchArgument(
        "fake_heartbeat",
        default_value="false",
        description="Whether to fake the heartbeat sent to the platform controller, ONLY SET TRUE IF TESTING!",
    )

    # Get URDF via xacro
    robot_description_content = Command(
        [
            PathJoinSubstitution([FindExecutable(name="xacro")]),
            " ",
            PathJoinSubstitution([pkg("asb_ros2_control"), "urdf", "asb.urdf.xacro"]),
            " ",
            "test:=",
            use_vcan0_launch_configuration,
        ]
    )

    robot_description = {"robot_description": robot_description_content}

    robot_controllers = PathJoinSubstitution([pkg("asb_ros2_control"), "config", "asb_controllers.yaml"])

    control_node = Node(
        package="controller_manager",
        executable="ros2_control_node",
        parameters=[robot_controllers],
        output="both",
        remappings=[
            ("/controller_manager/robot_description", "/robot_description"),
            ("/asb_base_controller/cmd_vel_unstamped", "/cmd_vel"),
            ("/asb_base_controller/odom", "/odom"),
            ("/asb_base_controller/imu", "/imu"),
        ],
    )

    robot_state_publisher_node = Node(
        package="robot_state_publisher",
        executable="robot_state_publisher",
        name="robot_state_publisher",
        output="both",
        parameters=[robot_description],
    )

    joint_state_broadcaster_spawner = Node(
        package="controller_manager",
        executable="spawner",
        arguments=["joint_state_broadcaster", "--controller-manager", "/controller_manager"],
    )

    robot_controller_spawner = Node(
        package="controller_manager",
        executable="spawner",
        arguments=["asb_base_controller", "--controller-manager", "/controller_manager"],
    )

    platform_controller_spawner = Node(
        package="controller_manager",
        executable="spawner",
        arguments=["asb_platform_controller", "--controller-manager", "/controller_manager"],
    )

    # Delay start of platform_controller_spawner after `joint_state_broadcaster`
    delay_platform_controller_spawner_after_joint_state_broadcaster_spawner = RegisterEventHandler(
        event_handler=OnProcessExit(
            target_action=joint_state_broadcaster_spawner,
            on_exit=[platform_controller_spawner],
        )
    )

    # Delay start of robot_controller after `platform_controller_spawner`
    delay_robot_controller_spawner_after_joint_state_broadcaster_spawner = RegisterEventHandler(
        event_handler=OnProcessExit(
            target_action=platform_controller_spawner,
            on_exit=[robot_controller_spawner],
        )
    )

    include_logging_launch = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(os.path.join(pkg("asb_logging"), "launch", "record_bag.launch.py"))
    )

    fake_heartbeat_publisher_node = Node(
        package="asb_sim",
        executable="test_heartbeat.py",
        name="test_heartbeat_publisher",
        output="screen",
        condition=IfCondition(fake_heartbeat_launch_configuration),
    )

    # Create the launch description and populate
    ld = LaunchDescription()

    ld.add_action(use_vcan0_launch_argument)
    ld.add_action(fake_heartbeat_launch_argument)

    ld.add_action(control_node)

    ld.add_action(robot_state_publisher_node)
    ld.add_action(joint_state_broadcaster_spawner)
    ld.add_action(delay_platform_controller_spawner_after_joint_state_broadcaster_spawner)
    ld.add_action(delay_robot_controller_spawner_after_joint_state_broadcaster_spawner)
    ld.add_action(include_logging_launch)
    ld.add_action(fake_heartbeat_publisher_node)

    return ld
