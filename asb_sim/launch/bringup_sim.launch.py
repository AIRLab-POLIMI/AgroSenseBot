
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

    print_debug_launch_configuration = LaunchConfiguration("print_debug")
    print_debug_launch_argument = DeclareLaunchArgument(
        'print_debug',
        default_value='false',
        description="Whether to print (lots of) additional data.",
    )

    start_in_control_mode_gcu_launch_configuration = LaunchConfiguration("start_in_control_mode_GCU")
    start_in_control_mode_gcu_launch_argument = DeclareLaunchArgument(
        'start_in_control_mode_GCU',
        default_value='false',
        description="Whether to set the initial control mode value to GCU.",
    )

    use_simulator_launch_configuration = LaunchConfiguration("use_simulator")
    use_simulator_launch_argument = DeclareLaunchArgument(
        'use_simulator',
        default_value='true',
        description="Whether to send and receive motor velocities from simulator.",
    )

    webots_launch = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(os.path.join(pkg("asb_webots"), "launch", "sim_webots.launch.py")),
        condition=IfCondition(use_simulator_launch_configuration)
    )

    front_os0_filter_node = Node(
        package="asb_lidar_filter",
        executable="asb_lidar_filter_node",
        name="lidar_filter_front",
        parameters=[
            os.path.join(pkg("asb_lidar_filter"), "config", "lidar_filter.yaml"),
        ],
        remappings={
            "points_in": "/scan_front_multilayer/points",
            "points_out": "/scan_front_multilayer/points_filtered",
            "scan_out": "/scan_front",
        }.items(),
        output="screen",
    )

    rear_os0_filter_node = Node(
        package="asb_lidar_filter",
        executable="asb_lidar_filter_node",
        name="lidar_filter_rear",
        parameters=[
            os.path.join(pkg("asb_lidar_filter"), "config", "lidar_filter.yaml"),
        ],
        remappings={
            "points_in": "/scan_rear_multilayer/points",
            "points_out": "/scan_rear_multilayer/points_filtered",
            "scan_out": "/scan_rear",
        }.items(),
        output="screen",
    )

    system_test_node = launch_ros.actions.LifecycleNode(
        name="test_node",
        namespace="system_test",
        package="asb_sim",
        output="screen",
        executable="test_node",
        parameters=[{
            "print_debug": print_debug_launch_configuration,
            "start_in_control_mode_GCU": start_in_control_mode_gcu_launch_configuration,
            "use_simulator": use_simulator_launch_configuration,
            "dummy_VCU_canopen_node_config": os.path.join(pkg("asb_sim"), "config", "dummy_VCU.dcf"),
            "dummy_MDL_canopen_node_config": os.path.join(pkg("asb_sim"), "config", "dummy_MDL.dcf"),
            "dummy_MDR_canopen_node_config": os.path.join(pkg("asb_sim"), "config", "dummy_MDR.dcf"),
            "dummy_FAN_canopen_node_config": os.path.join(pkg("asb_sim"), "config", "dummy_FAN.dcf"),
            "can_interface_name": "vcan0",
        }],
    )
    lifecycle_inactive_state_handler = launch.actions.RegisterEventHandler(
        launch_ros.event_handlers.OnStateTransition(
            target_lifecycle_node=system_test_node,
            goal_state='inactive',
            handle_once=True,
            entities=[
                launch.actions.LogInfo(msg="node reached the 'inactive' state, activating."),
                launch.actions.EmitEvent(event=launch_ros.events.lifecycle.ChangeState(
                    lifecycle_node_matcher=launch.events.matches_action(system_test_node),
                    transition_id=lifecycle_msgs.msg.Transition.TRANSITION_ACTIVATE,
                )),
            ],
        ),
    )
    lifecycle_configure = launch.actions.EmitEvent(
        event=launch_ros.events.lifecycle.ChangeState(
            lifecycle_node_matcher=launch.events.matches_action(system_test_node),
            transition_id=lifecycle_msgs.msg.Transition.TRANSITION_CONFIGURE,
        )
    )

    include_control_launch = IncludeLaunchDescription(
        launch_description_source=PythonLaunchDescriptionSource(os.path.join(pkg("asb_ros2_control"), "launch", "asb_ros2_control.launch.py")),
        launch_arguments={'use_vcan0': 'true'}.items(),
    )

    ld = launch.LaunchDescription()

    ld.add_action(print_debug_launch_argument)
    ld.add_action(start_in_control_mode_gcu_launch_argument)
    ld.add_action(use_simulator_launch_argument)

    ld.add_action(webots_launch)
    ld.add_action(front_os0_filter_node)
    ld.add_action(rear_os0_filter_node)
    ld.add_action(system_test_node)
    ld.add_action(lifecycle_inactive_state_handler)
    ld.add_action(lifecycle_configure)
    ld.add_action(include_control_launch)

    return ld
