
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

    scan_launch = IncludeLaunchDescription(
        launch_description_source=PythonLaunchDescriptionSource(os.path.join(pkg("asb_real"), "launch", "scan.launch.py")),
    )

    # Launch separately to avoid resetting RTK fix when restarting nodes
    # microstrain_3dm_gq7_launch = IncludeLaunchDescription(
    #     launch_description_source=PythonLaunchDescriptionSource(os.path.join(pkg("asb_real"), "launch", "gq7_launch.py")),
    # )

    ld = launch.LaunchDescription()

    ld.add_action(include_control_launch)
    ld.add_action(scan_launch)  # Launch separately to avoid resetting RTK fix when restarting nodes
    # ld.add_action(microstrain_3dm_gq7_launch)  # Launch separately to avoid resetting RTK fix when restarting nodes

    return ld
