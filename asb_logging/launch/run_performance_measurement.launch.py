
import os

import launch
import launch.actions
import launch.events
from launch.actions import DeclareLaunchArgument, IncludeLaunchDescription, RegisterEventHandler, LogInfo, EmitEvent
from launch.event_handlers import OnProcessExit
from launch.events import Shutdown
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import LaunchConfiguration, PathJoinSubstitution
from ament_index_python import get_package_share_directory as pkg
from launch_ros.actions import Node


def generate_launch_description():

    bag_name_launch_configuration = LaunchConfiguration("bag_name")
    bag_name_launch_argument = DeclareLaunchArgument(
        "bag_name",
        default_value=os.path.expanduser("~/asb_logs/test_data/scan_multilayer_test_data_1"),
        description="Name of the bag in ~/asb_logs/ to play.",
    )

    play_node = launch.actions.ExecuteProcess(
        cmd="xterm -e ros2 bag play".split() + [PathJoinSubstitution([bag_name_launch_configuration])],
        cwd=os.path.expanduser("~/asb_logs/test_data/"),
        output='screen',
    )

    ros_bag_msg_timestamp_republisher_node = Node(
        package="asb_logging",
        executable="ros_bag_msg_timestamp_republisher.py",
        name="ros_bag_msg_timestamp_republisher",
        output="both",
    )

    performance_logger_node = Node(
        package="asb_logging",
        executable="node_performance_logger.py",
        name="node_performance_logger",
        output="both",
    )

    finished_event = RegisterEventHandler(
        OnProcessExit(
            target_action=performance_logger_node,
            on_exit=[
                LogInfo(msg='performance_logger_node terminated'),
                EmitEvent(event=Shutdown(reason='performance_logger_node terminated')),
            ]
        )
    )

    ld = launch.LaunchDescription()

    ld.add_action(bag_name_launch_argument)
    ld.add_action(play_node)
    ld.add_action(ros_bag_msg_timestamp_republisher_node)
    ld.add_action(performance_logger_node)
    ld.add_action(finished_event)

    return ld
