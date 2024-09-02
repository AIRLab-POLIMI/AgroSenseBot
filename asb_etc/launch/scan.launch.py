
import os
import launch
import launch.actions
import launch.events
from launch.actions import IncludeLaunchDescription
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch_ros.actions import Node

from ament_index_python import get_package_share_directory as pkg


def generate_launch_description():

    include_rear_os0_launch = IncludeLaunchDescription(
        launch_description_source=PythonLaunchDescriptionSource(os.path.join(pkg("ouster_ros"), "launch", "driver.launch.py")),
        launch_arguments={
            "params_file": os.path.join(pkg("asb_real"), "config", "os0.yaml"),
            "ouster_ns": "scan_rear_multilayer",
            "viz": "False",
        }.items(),
    )

    ld = launch.LaunchDescription()

    ld.add_action(include_rear_os0_launch)

    return ld
