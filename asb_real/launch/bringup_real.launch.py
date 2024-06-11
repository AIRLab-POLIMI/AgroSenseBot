
import os
import launch
import launch.actions
import launch.events
from launch.actions import IncludeLaunchDescription
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch_ros.actions import Node

from ament_index_python import get_package_share_directory as pkg


def generate_launch_description():

    include_control_launch = IncludeLaunchDescription(
        launch_description_source=PythonLaunchDescriptionSource(os.path.join(pkg("asb_ros2_control"), "launch", "asb_ros2_control.launch.py")),
    )

    microstrain_3dm_gq7_launch = IncludeLaunchDescription(
        launch_description_source=PythonLaunchDescriptionSource(os.path.join(pkg("asb_real"), "launch", "gq7_launch.py")),
    )

    include_front_os0_launch = IncludeLaunchDescription(
        launch_description_source=PythonLaunchDescriptionSource(os.path.join(pkg("ouster_ros"), "launch", "driver.launch.py")),
        launch_arguments={
            "params_file": os.path.join(pkg("asb_real"), "config", "os0.yaml"),
            "ouster_ns": "scan_front_right_multilayer",
            "viz": "False",
        }.items(),
    )

    test_heartbeat_publisher_node = Node(
        package="asb_sim",
        executable="test_heartbeat.py",
        name="test_heartbeat_publisher",
        output="screen",
    )

    ld = launch.LaunchDescription()

    ld.add_action(include_control_launch)
    # ld.add_action(microstrain_3dm_gq7_launch)  # Launch separately to avoid resetting RTK fix when restarting nodes
    ld.add_action(include_front_os0_launch)
    ld.add_action(test_heartbeat_publisher_node)

    return ld
