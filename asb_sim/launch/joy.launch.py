
import os
import sys
sys.path.insert(0, os.path.join(os.path.dirname(__file__), '..'))  # noqa
sys.path.insert(0, os.path.join(os.path.dirname(__file__), '..', '..', 'launch'))  # noqa

import launch
import launch.actions
import launch.events

import launch_ros
import launch_ros.events  
import launch_ros.events.lifecycle


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
                {  # these keybindings are for a G-LAB K-PAD-THORIUM joy-pad (SHANWAN Android Gamepad)
                    "enable_button": 7,
                    "axis_linear.x": 1,
                    "scale_linear.x": 0.5,
                    "scale_angular.yaw": 0.6,
                },
            ],
        # remappings=[
        #         ('/cmd_vel', '/asb_base_controller/cmd_vel_unstamped'),
        #     ],
    )

    ld = launch.LaunchDescription()
    ld.add_action(joy_node)
    ld.add_action(teleop_node)
    return ld
