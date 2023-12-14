
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

    # record_node = Node(  TODO ros2 bag record
    #     package="",
    #     executable="",
    #     name="",
    #     output="screen",
    # )

    ld = launch.LaunchDescription()

    # ld.add_action(record_node)  TODO ros2 bag record

    return ld
