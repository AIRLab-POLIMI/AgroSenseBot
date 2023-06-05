
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

    canopen_node_config_arg = DeclareLaunchArgument(
        'canopen_node_config',
        default_value=TextSubstitution(text=os.path.join(path_to_test, "..", "config", "test_slave.eds")),
        description="Path to EDS file to be used for the CANOpen slave node.",
    )
    can_interface_arg = DeclareLaunchArgument(
        'can_interface_name',
        default_value=TextSubstitution(text="vcan0"),
        description="CAN interface name.",
    )
    ros_node_name_arg = DeclareLaunchArgument(
        'ros_node_name',
        default_value=TextSubstitution(text="bridge_node"),
        description="Name of the ros node.",
    )

    ros2_canopen_bridge_node = launch_ros.actions.LifecycleNode(
        name=LaunchConfiguration("ros_node_name"),
        namespace="",
        package="agrosensebot_canopen_bridge",
        output="screen",
        executable="bridge_node",
        parameters=[
                {
                    "canopen_node_config": LaunchConfiguration("canopen_node_config"),
                    "can_interface_name": LaunchConfiguration("can_interface_name"),
                },
            ],
    )
    lifecycle_inactive_state_handler = launch.actions.RegisterEventHandler(
        launch_ros.event_handlers.OnStateTransition(
            target_lifecycle_node=ros2_canopen_bridge_node,
            goal_state='inactive',
            handle_once=true,
            entities=[
                launch.actions.LogInfo(
                    msg="node reached the 'inactive' state, activating."),
                launch.actions.EmitEvent(event=launch_ros.events.lifecycle.ChangeState(
                    lifecycle_node_matcher=launch.events.matches_action(ros2_canopen_bridge_node),
                    transition_id=lifecycle_msgs.msg.Transition.TRANSITION_ACTIVATE,
                )),
            ],
        ),
    )
    lifecycle_configure = launch.actions.EmitEvent(
        event=launch_ros.events.lifecycle.ChangeState(
            lifecycle_node_matcher=launch.events.matches_action(ros2_canopen_bridge_node),
            transition_id=lifecycle_msgs.msg.Transition.TRANSITION_CONFIGURE,
        )
    )

    ld = launch.LaunchDescription()
    ld.add_action(canopen_node_config_arg)
    ld.add_action(can_interface_arg)
    ld.add_action(ros_node_name_arg)
    ld.add_action(ros2_canopen_bridge_node)
    ld.add_action(lifecycle_inactive_state_handler)
    ld.add_action(lifecycle_configure)
    return ld
