
import os

import launch
import launch.actions
import launch.events
from launch.substitutions import LaunchConfiguration
from launch.actions import DeclareLaunchArgument, IncludeLaunchDescription
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch_ros.actions import Node

from ament_index_python import get_package_share_directory as pkg


def generate_launch_description():

    system_status_logger_node = Node(
        package="asb_logging",
        executable="system_status_logger.py",
        name="asb_system_status_logger",
        output="screen",
    )

    ld = launch.LaunchDescription()

    ld.add_action(system_status_logger_node)

    return ld
