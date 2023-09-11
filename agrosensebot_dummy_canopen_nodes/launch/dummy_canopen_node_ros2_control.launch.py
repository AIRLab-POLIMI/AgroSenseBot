
import os
import sys

from sympy import true
sys.path.insert(0, os.path.join(os.path.dirname(__file__), '..'))  # noqa
sys.path.insert(0, os.path.join(os.path.dirname(__file__), '..', '..', 'launch'))  # noqa

import launch
import launch.actions
import launch.events
from launch.substitutions import LaunchConfiguration, TextSubstitution
from launch.actions import DeclareLaunchArgument, IncludeLaunchDescription
from launch.launch_description_sources import PythonLaunchDescriptionSource

from ament_index_python import get_package_share_directory

import launch_ros
import launch_ros.events
import launch_ros.events.lifecycle
from launch_ros.actions import PushRosNamespace

import lifecycle_msgs.msg


def generate_launch_description():
    path_to_test = os.path.dirname(__file__)

    dummy_canopen_node_config_arg = DeclareLaunchArgument(
        'dummy_canopen_node_config',
        default_value=TextSubstitution(text=os.path.join(path_to_test, "..", "config", "dummy_node.dcf")),
        description="Path to DCF file to be used for the dummy CANOpen node.",
    )
    can_interface_arg = DeclareLaunchArgument(
        'can_interface_name',
        default_value=TextSubstitution(text="vcan0"),
        description="CAN interface name.",
    )
    dummy_ros_node_name_arg = DeclareLaunchArgument(
        'dummy_ros_node_name',
        default_value=TextSubstitution(text="dummy_node"),
        description="Name of the ros node.",
    )

    dummy_ros2_canopen_bridge_node = launch_ros.actions.LifecycleNode(
        name=LaunchConfiguration("dummy_ros_node_name"),
        namespace="dummy_test",
        package="agrosensebot_dummy_canopen_nodes",
        output="screen",
        executable="dummy_node",
        parameters=[
                {
                    "dummy_canopen_node_config": LaunchConfiguration("dummy_canopen_node_config"),
                    "can_interface_name": LaunchConfiguration("can_interface_name"),
                },
            ],
    )
    lifecycle_inactive_state_handler = launch.actions.RegisterEventHandler(
        launch_ros.event_handlers.OnStateTransition(
            target_lifecycle_node=dummy_ros2_canopen_bridge_node,
            goal_state='inactive',
            handle_once=true,
            entities=[
                launch.actions.LogInfo(
                    msg="node reached the 'inactive' state, activating."),
                launch.actions.EmitEvent(event=launch_ros.events.lifecycle.ChangeState(
                    lifecycle_node_matcher=launch.events.matches_action(dummy_ros2_canopen_bridge_node),
                    transition_id=lifecycle_msgs.msg.Transition.TRANSITION_ACTIVATE,
                )),
            ],
        ),
    )
    lifecycle_configure = launch.actions.EmitEvent(
        event=launch_ros.events.lifecycle.ChangeState(
            lifecycle_node_matcher=launch.events.matches_action(dummy_ros2_canopen_bridge_node),
            transition_id=lifecycle_msgs.msg.Transition.TRANSITION_CONFIGURE,
        )
    )

    include_control_launch = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            os.path.join(
                get_package_share_directory('asb_ros2_control'), 'launch/asb_ros2_control_test.launch.py'))
    )

    ld = launch.LaunchDescription()
    ld.add_action(dummy_canopen_node_config_arg)
    ld.add_action(can_interface_arg)
    ld.add_action(dummy_ros_node_name_arg)
    ld.add_action(dummy_ros2_canopen_bridge_node)
    ld.add_action(lifecycle_inactive_state_handler)
    ld.add_action(lifecycle_configure)
    ld.add_action(include_control_launch)
    return ld

