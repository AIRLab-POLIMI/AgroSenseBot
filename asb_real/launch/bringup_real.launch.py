
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

    # Launch separately to avoid resetting RTK fix when restarting nodes
    # microstrain_3dm_gq7_launch = IncludeLaunchDescription(
    #     launch_description_source=PythonLaunchDescriptionSource(os.path.join(pkg("asb_real"), "launch", "gq7_launch.py")),
    # )

    include_front_os0_launch = IncludeLaunchDescription(
        launch_description_source=PythonLaunchDescriptionSource(os.path.join(pkg("ouster_ros"), "launch", "driver.launch.py")),
        launch_arguments={
            "params_file": os.path.join(pkg("asb_real"), "config", "os0.yaml"),
            "ouster_ns": "scan_front_multilayer",
            "viz": "False",
        }.items(),
    )

    front_os0_filter_node = Node(
        package="asb_lidar_filter",
        executable="asb_lidar_filter_node",
        name="lidar_filter_front",
        parameters=[
            os.path.join(pkg("asb_lidar_filter"), "config", "lidar_filter.yaml"),
            {
                "point_type": "asb_ouster_ros::Point",
            },
        ],
        remappings={
            "points_in": "/scan_front_multilayer/points",
            "points_out": "/scan_front_multilayer/points_filtered",
        }.items(),
        output="screen",
    )

    ld = launch.LaunchDescription()

    ld.add_action(include_control_launch)
    # ld.add_action(microstrain_3dm_gq7_launch)  # Launch separately to avoid resetting RTK fix when restarting nodes
    ld.add_action(include_front_os0_launch)
    ld.add_action(front_os0_filter_node)

    return ld
