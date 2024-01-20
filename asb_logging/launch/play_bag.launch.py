
import os

import launch
import launch.actions
import launch.events
from launch.actions import DeclareLaunchArgument, IncludeLaunchDescription
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import LaunchConfiguration, PathJoinSubstitution
from ament_index_python import get_package_share_directory as pkg


def generate_launch_description():

    bag_name_launch_configuration = LaunchConfiguration("bag_name")
    bag_name_launch_argument = DeclareLaunchArgument(
        "bag_name",
        description="Name of the bag in ~/asb_logs/ to play.",
    )

    play_node = launch.actions.ExecuteProcess(
        cmd="xterm -e ros2 bag play".split() + [PathJoinSubstitution([bag_name_launch_configuration])],
        cwd=os.path.expanduser("~/asb_logs/"),
        output='screen',
    )

    include_rviz_launch = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(os.path.join(pkg("asb_nav"), "launch", "rviz.launch.py")),
    )

    ld = launch.LaunchDescription()

    ld.add_action(bag_name_launch_argument)
    ld.add_action(play_node)
    ld.add_action(include_rviz_launch)

    return ld
