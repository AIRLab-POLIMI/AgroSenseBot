
import os

import launch
import launch.actions
import launch.events
from launch.actions import IncludeLaunchDescription
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch_ros.actions import Node

from ament_index_python import get_package_share_directory as pkg


def generate_launch_description():

    include_control_launch = IncludeLaunchDescription(
        launch_description_source=PythonLaunchDescriptionSource(os.path.join(pkg("asb_ros2_control"), "launch", "asb_ros2_control.launch.py")),
    )

    include_nav_launch = IncludeLaunchDescription(
        launch_description_source=PythonLaunchDescriptionSource(os.path.join(pkg("asb_nav"), "launch", "nav_gnss.launch.py")),
    )

    test_heartbeat_publisher_node = Node(
        package="asb_sim",
        executable="test_heartbeat.py",
        name="test_heartbeat_publisher",
        output="screen",
    )

    ld = launch.LaunchDescription()

    ld.add_action(include_control_launch)
    ld.add_action(include_nav_launch)
    ld.add_action(test_heartbeat_publisher_node)

    return ld
