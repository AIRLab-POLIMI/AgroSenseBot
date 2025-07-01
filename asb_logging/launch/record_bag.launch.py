
import os
import re
from datetime import datetime

import launch
import launch.actions
import launch.events
from launch.actions import DeclareLaunchArgument
from launch.conditions import IfCondition
from launch.substitutions import LaunchConfiguration, AndSubstitution
from launch_ros.actions import Node


def generate_launch_description():
    now = datetime.now()
    date_stamp = now.strftime("%Y-%m-%d")
    date_time_stamp = now.strftime("%Y-%m-%d__%H-%M-%S")
    date_stamped_logs_dir = os.path.join(os.path.expanduser("~/asb_logs/"), date_stamp)

    if not os.path.exists(date_stamped_logs_dir):
        os.makedirs(date_stamped_logs_dir)

    existing_bags = list(filter(re.compile(r"\d+").match, os.listdir(date_stamped_logs_dir)))
    existing_indices = list(map(lambda s: int(re.findall(r"\d+", s)[0]), existing_bags))

    print(f"making log dir...")
    if len(existing_indices) == 0:
        # if there are no indexed log dirs, create the first one
        next_index = 0
        print(f"there are no indexed dirs, using index {next_index}")
    else:
        latest_index = max(existing_indices)
        latest_indexed_dir = os.path.join(date_stamped_logs_dir, f"{latest_index}")
        if len(os.listdir(latest_indexed_dir)) == 0:
            # if there are indexed log dirs, and the latest one is empty, use that one (generate_launch_description is run twice, so this happens every time)
            next_index = latest_index
            print(f"latest log dir is empty, using index {next_index}")
        else:
            # if there are indexed log dirs, and the latest one is not empty, we need to create another one (because generate_launch_description is run twice, this new dir will be seen as emtpy the next time this code is executed)
            next_index = latest_index + 1
            print(f"latest log dir is not empty, using index {next_index}")

    indexed_log_dir = os.path.join(date_stamped_logs_dir, f"{next_index}")

    if not os.path.exists(indexed_log_dir):
        os.makedirs(indexed_log_dir)

    always_exclude_regex = "/expansions"  # multiple topics can be expressed like: "/topic_1|/topic_2"
    sensors_regex = f"{always_exclude_regex}|/scan_(front|rear)_multilayer/.*"

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
        cmd=f"ros2 bag record --node-name record_all_except_sensors --output rosbag2_{date_time_stamp}_no_sensors -a --exclude {sensors_regex} --compression-mode file --compression-format zstd --max-bag-duration 60".split(),
        cwd=indexed_log_dir,
        output='screen',
        condition=IfCondition(record_launch_configuration),
    )

    record_all_node = launch.actions.ExecuteProcess(
        cmd=f"ros2 bag record --node-name record_all --output rosbag2_{date_time_stamp}_all -a --exclude {always_exclude_regex} --compression-mode file --compression-format zstd --max-bag-duration 60".split(),
        cwd=indexed_log_dir,
        output='screen',
        condition=IfCondition(AndSubstitution(record_launch_configuration, record_sensors_launch_configuration)),
    )

    log_workspace_packages_share_node = Node(
        package="asb_logging",
        executable="workspace_packages_share_logger.py",
        name="workspace_packages_share_logger",
        output="screen",
        parameters=[{
            'workspace_install_dir_path': "~/w/agrosensebot_ws/install",
            'log_dir_path': os.path.join(indexed_log_dir, "workspace_packages_share"),
        }],
    )

    ld = launch.LaunchDescription()

    ld.add_action(record_launch_argument)
    ld.add_action(record_sensors_launch_argument)
    ld.add_action(record_all_except_sensors_node)
    ld.add_action(record_all_node)
    ld.add_action(log_workspace_packages_share_node)

    return ld
