
import os

from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription

from launch_ros.actions import Node


def generate_launch_description():
    pkg = get_package_share_directory

    location_env_var = os.environ['ASB_LOCATION']

    geofence_map_server_node = Node(
        package="asb_nav",
        executable="geofence_map_server.py",
        name="geofence_map_server",
        output="screen",
        parameters=[
            {"geofence_file_path": os.path.join(pkg("asb_nav"), "config", "local_data", location_env_var, "geofence.yaml")},
            os.path.join(pkg("asb_nav"), "config", "geofence_map_publisher_params", "geofence_map_publisher_params.yaml"),
        ],
        remappings=[
            ("polygons_in", "/geofence_polygons"),
            ("map", "/geofence_map"),
        ],
    )

    # Create the launch description and populate
    ld = LaunchDescription()
    ld.add_action(geofence_map_server_node)

    return ld
