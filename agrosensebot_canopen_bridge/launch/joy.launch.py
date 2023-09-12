
import os
import sys
from sympy import true
sys.path.insert(0, os.path.join(os.path.dirname(__file__), '..'))  # noqa
sys.path.insert(0, os.path.join(os.path.dirname(__file__), '..', '..', 'launch'))  # noqa

import launch
import launch.actions
import launch.events
from launch.substitutions import LaunchConfiguration, TextSubstitution
from launch.actions import DeclareLaunchArgument

import launch_ros
import launch_ros.events  
import launch_ros.events.lifecycle

import lifecycle_msgs.msg


def generate_launch_description():
    path_to_test = os.path.dirname(__file__)

    joy_node = launch_ros.actions.Node(
        namespace="",
        name="joy",
        package="joy",
        executable="joy_node",
        output="screen",
    )
    teleop_node = launch_ros.actions.Node(
        namespace="",
        name="teleop_twist_joy",
        package="teleop_twist_joy",
        executable="teleop_node",
        output="screen",
        parameters=[
                {
                    "enable_button": 7,
                    "axis_linear.x": 1,
                    "scale_linear.x": 0.1,
                    "scale_angular.yaw": 0.2,
                },
            ],
        remappings=[
                ('/cmd_vel', '/asb_base_controller/cmd_vel_unstamped'),
            ],
    )

    ld = launch.LaunchDescription()
    ld.add_action(joy_node)
    ld.add_action(teleop_node)
    return ld
