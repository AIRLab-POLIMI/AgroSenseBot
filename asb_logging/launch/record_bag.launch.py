
import os
import re
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

    if not os.path.exists(logs_dir):
        os.makedirs(logs_dir)

    existing_bags = list(filter(re.compile(r"\d+_rosbag2_*").match, os.listdir(logs_dir)))
    existing_indices = list(map(lambda s: int(re.findall(r"\d+", s)[0]), existing_bags))
    next_index = max(existing_indices) + 1 if len(existing_indices) else 0

    sensors_regex = "/scan_(front|rear)_multilayer/.*"

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
        cmd=f"ros2 bag record --node-name record_all_except_sensors --output {next_index:04}_rosbag2_{stamp}_no_sensors -a --exclude {sensors_regex} --compression-mode file --compression-format zstd --max-bag-duration 60".split(),
        cwd=logs_dir,
        output='screen',
        condition=IfCondition(record_launch_configuration),
    )

    record_all_node = launch.actions.ExecuteProcess(
        cmd=f"ros2 bag record --node-name record_all --output {next_index:04}_rosbag2_{stamp}_all -a --compression-mode file --compression-format zstd --max-bag-duration 60".split(),
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
