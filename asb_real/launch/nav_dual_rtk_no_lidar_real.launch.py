
import os

import launch
import launch.actions
import launch.events
from launch.actions import IncludeLaunchDescription
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch_ros.actions import Node

from ament_index_python import get_package_share_directory as pkg


def generate_launch_description():

    include_bringup_real_launch = IncludeLaunchDescription(
        launch_description_source=PythonLaunchDescriptionSource(os.path.join(pkg("asb_real"), "launch", "bringup_real.launch.py")),
    )

    include_nav_launch = IncludeLaunchDescription(
        launch_description_source=PythonLaunchDescriptionSource(os.path.join(pkg("asb_nav"), "launch", "nav_dual_rtk_no_lidar.launch.py")),
    )

    ld = launch.LaunchDescription()

    ld.add_action(include_bringup_real_launch)
    ld.add_action(include_nav_launch)

    return ld
