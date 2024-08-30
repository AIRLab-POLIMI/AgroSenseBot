
import os
from datetime import datetime

import launch
import launch.actions
import launch.events
from launch.actions import DeclareLaunchArgument
from launch.conditions import IfCondition
from launch.substitutions import LaunchConfiguration


def generate_launch_description():

    sensors_regex = "/scan_front_multilayer/.*"

    stamp = datetime.now().strftime("%Y-%m-%d__%H-%M-%S")

    record_launch_configuration = LaunchConfiguration("record")
    record_launch_argument = DeclareLaunchArgument(
        "record",
        default_value="true",
        description="Whether to record the system data.",
    )

    record_sensors_launch_configuration = LaunchConfiguration("record_sensors")
    record_sensors_launch_argument = DeclareLaunchArgument(
        "record_sensors",
        default_value="true",
        description="Whether to record the system data.",
    )

    record_node = launch.actions.ExecuteProcess(
        cmd=f"ros2 bag record --node-name record_all --output rosbag2_{stamp}_all -a --exclude {sensors_regex} --compression-mode file --compression-format zstd --max-bag-duration 60".split(),
        cwd=os.path.expanduser("~/asb_logs/"),
        output='screen',
        condition=IfCondition(record_launch_configuration),
    )

    record_sensors_node = launch.actions.ExecuteProcess(
        cmd=f"ros2 bag record --node-name record_sensors --output rosbag2_{stamp}_sensors -e {sensors_regex} --compression-mode file --compression-format zstd --max-bag-duration 60".split(),
        cwd=os.path.expanduser("~/asb_logs/"),
        output='screen',
        condition=IfCondition(record_sensors_launch_configuration),
    )

    ld = launch.LaunchDescription()

    ld.add_action(record_launch_argument)
    ld.add_action(record_sensors_launch_argument)
    ld.add_action(record_node)
    ld.add_action(record_sensors_node)

    return ld
