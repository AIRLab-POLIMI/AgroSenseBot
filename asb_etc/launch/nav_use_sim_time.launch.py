
import os

from ament_index_python.packages import get_package_share_directory

from launch import LaunchDescription
from launch.substitutions import LaunchConfiguration
from launch.actions import IncludeLaunchDescription, DeclareLaunchArgument
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.conditions import IfCondition
from launch_ros.actions import Node


def generate_launch_description():
    pkg = get_package_share_directory

    use_sim_time_launch_configuration = LaunchConfiguration('use_sim_time')
    use_sim_time_launch_argument = DeclareLaunchArgument(
        'use_sim_time',
        default_value='true',
        description='Whether to set the use_sim_time parameter to true. Should only be set to true if using Gazebo (not Webots).',
    )

    rviz_launch_configuration = LaunchConfiguration('rviz')
    rviz_launch_argument = DeclareLaunchArgument(
        'rviz',
        default_value='true',
        description='Whether to start RViz',
    )

    nav2_bringup_include = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(os.path.join(pkg("asb_nav"), "launch", "../../asb_nav/launch/nav2_navigation_launch.py")),
        launch_arguments={
            "use_sim_time": use_sim_time_launch_configuration,
            "params_file": os.path.join(pkg("asb_nav"), "config", "nav2_params", "nav2_params.yaml"),
            "controller_params_file": os.path.join(pkg("asb_nav"), "config", "nav2_params", "nav2_controller_params_asb_rpp.yaml"),
            "planner_params_file": os.path.join(pkg("asb_nav"), "config", "nav2_params", "nav2_planner_params_smac_hybrid.yaml"),
            "map_server_params_file": os.path.join(pkg("asb_nav"), "config", "map_server_params", "map_server_params.yaml"),
            "autostart": "true",
        }.items(),
    )

    rviz_include = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(os.path.join(pkg("asb_nav"), "launch", "../../asb_nav/launch/rviz.launch.py")),
        condition=IfCondition(rviz_launch_configuration)
    )

    # Create the launch description and populate
    ld = LaunchDescription()

    # launch arguments
    ld.add_action(use_sim_time_launch_argument)
    ld.add_action(rviz_launch_argument)

    # navigation
    ld.add_action(nav2_bringup_include)

    # viz
    ld.add_action(rviz_include)

    return ld
