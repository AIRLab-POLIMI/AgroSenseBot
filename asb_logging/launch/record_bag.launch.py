
import os
from datetime import datetime

import launch
import launch.actions
import launch.events
from launch.actions import DeclareLaunchArgument
from launch.conditions import IfCondition
from launch.substitutions import LaunchConfiguration, AndSubstitution


def generate_launch_description():

    date_stamp = datetime.now().strftime("%Y-%m-%d")
    stamp = datetime.now().strftime("%Y-%m-%d__%H-%M-%S")
    logs_dir = os.path.join(os.path.expanduser("~/asb_logs/"), date_stamp)

    sensors_regex = "/scan_(front|rear)_multilayer/.*"

    if not os.path.exists(logs_dir):
        os.makedirs(logs_dir)

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

    record_all_except_sensors_node = launch.actions.ExecuteProcess(
        cmd=f"ros2 bag record --node-name record_all_except_sensors --output rosbag2_{stamp}_all_except_sensors -a --exclude {sensors_regex} --compression-mode file --compression-format zstd --max-bag-duration 60".split(),
        cwd=logs_dir,
        output='screen',
        condition=IfCondition(record_launch_configuration),
    )

    record_all_node = launch.actions.ExecuteProcess(
        cmd=f"ros2 bag record --node-name record_all --output rosbag2_{stamp}_all -a --compression-mode file --compression-format zstd --max-bag-duration 60".split(),
        cwd=logs_dir,
        output='screen',
        condition=IfCondition(AndSubstitution(record_launch_configuration, record_sensors_launch_configuration)),
    )

    ld = launch.LaunchDescription()

    ld.add_action(record_launch_argument)
    ld.add_action(record_sensors_launch_argument)
    ld.add_action(record_all_except_sensors_node)
    ld.add_action(record_all_node)

    return ld
