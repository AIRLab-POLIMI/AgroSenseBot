
import os

import launch
import launch.actions
import launch.events
from launch.conditions import IfCondition
from launch.substitutions import LaunchConfiguration
from launch.actions import DeclareLaunchArgument, IncludeLaunchDescription
from launch.launch_description_sources import PythonLaunchDescriptionSource

from ament_index_python import get_package_share_directory as pkg

import launch_ros
import launch_ros.events
import launch_ros.events.lifecycle

import lifecycle_msgs.msg
from launch_ros.actions import Node


def generate_launch_description():

    print_debug_launch_configuration = LaunchConfiguration("print_debug")
    print_debug_launch_argument = DeclareLaunchArgument(
        'print_debug',
        default_value='false',
        description="Whether to print (lots of) additional data.",
    )

    include_control_launch = IncludeLaunchDescription(
        launch_description_source=PythonLaunchDescriptionSource(os.path.join(pkg("asb_ros2_control"), "launch", "asb_ros2_control.launch.py")),
    )

    include_nav_launch = IncludeLaunchDescription(
        launch_description_source=PythonLaunchDescriptionSource(os.path.join(pkg("asb_nav"), "launch", "gnss_nav.launch.py")),
    )

    test_heartbeat_publisher_node = Node(
        package="asb_sim",
        executable="test_heartbeat.py",
        name="test_heartbeat_publisher",
        output="screen",
    )

    ld = launch.LaunchDescription()

    ld.add_action(print_debug_launch_argument)
    ld.add_action(include_control_launch)
    ld.add_action(include_nav_launch)
    ld.add_action(test_heartbeat_publisher_node)

    return ld
