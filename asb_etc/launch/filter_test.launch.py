
import os
import launch
import launch.actions
import launch.events
from launch.actions import IncludeLaunchDescription
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch_ros.actions import Node

from ament_index_python import get_package_share_directory as pkg


def generate_launch_description():

    front_os0_filter_node = Node(
        package="asb_lidar_filter",
        executable="asb_lidar_filter_node",
        name="lidar_filter_front",
        parameters=[
            os.path.join(pkg("asb_lidar_filter"), "config", "lidar_filter.yaml"),
            {
                "use_sim_time": True,
            },
        ],
        remappings={
            "points_in": "/scan_front_multilayer/points",
            "points_out": "/scan_front_multilayer/points_filtered_2",
            "scan_out": "/scan_front_2",
            "heartbeat_out": "/scan_heartbeat_front_2",
        }.items(),
        output="screen",
    )

    rear_os0_filter_node = Node(
        package="asb_lidar_filter",
        executable="asb_lidar_filter_node",
        name="lidar_filter_rear",
        parameters=[
            os.path.join(pkg("asb_lidar_filter"), "config", "lidar_filter.yaml"),
            {
                "use_sim_time": True,
            },
        ],
        remappings={
            "points_in": "/scan_rear_multilayer/points",
            "points_out": "/scan_rear_multilayer/points_filtered_2",
            "scan_out": "/scan_rear_2",
            "heartbeat_out": "/scan_heartbeat_rear_2",
        }.items(),
        output="screen",
    )

    ld = launch.LaunchDescription()

    ld.add_action(front_os0_filter_node)
    ld.add_action(rear_os0_filter_node)

    return ld
