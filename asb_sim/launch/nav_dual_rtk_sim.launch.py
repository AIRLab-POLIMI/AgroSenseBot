
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

    include_sim_launch = IncludeLaunchDescription(
        launch_description_source=PythonLaunchDescriptionSource(os.path.join(pkg("asb_sim"), "launch", "bringup_sim.launch.py")),
    )

    include_nav_launch = IncludeLaunchDescription(
        launch_description_source=PythonLaunchDescriptionSource(os.path.join(pkg("asb_nav"), "launch", "nav_dual_rtk.launch.py")),
    )

    ld = launch.LaunchDescription()

    ld.add_action(include_sim_launch)
    ld.add_action(include_nav_launch)

    return ld
