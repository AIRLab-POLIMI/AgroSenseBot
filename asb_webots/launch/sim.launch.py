import os
import launch
from launch import LaunchDescription
from launch_ros.actions import *
from ament_index_python.packages import get_package_share_directory
from webots_ros2_driver.webots_launcher import WebotsLauncher
from webots_ros2_driver.webots_controller import WebotsController


def generate_launch_description():
    package_share_dir = get_package_share_directory('asb_webots')
    robot_description_path = os.path.join(package_share_dir, 'resource', 'asb_webots_robot.urdf')

    with open(robot_description_path, 'r') as f:
        robot_description = f.read()

    webots_launcher = WebotsLauncher(
        world=os.path.join(package_share_dir, 'worlds', 'asb_gnss_world.wbt')
    )

    webots_controller = WebotsController(
        robot_name='asb_webots_robot',
        parameters=[
            {'robot_description': robot_description_path},
        ]
    )

    robot_state_publisher_cmd = Node(
        package='robot_state_publisher',
        executable='robot_state_publisher',
        name='robot_state_publisher',
        output='both',
        parameters=[{'robot_description': robot_description}]
    )

    return LaunchDescription([
        webots_launcher,
        webots_controller,
        robot_state_publisher_cmd,
        launch.actions.RegisterEventHandler(
            event_handler=launch.event_handlers.OnProcessExit(
                target_action=webots_launcher,
                on_exit=[launch.actions.EmitEvent(event=launch.events.Shutdown())],
            )
        )
    ])
