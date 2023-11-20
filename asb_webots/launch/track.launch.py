import os
import launch
from launch import LaunchDescription
from ament_index_python.packages import get_package_share_directory
from webots_ros2_driver.webots_launcher import WebotsLauncher
from webots_ros2_driver.webots_controller import WebotsController


def generate_launch_description():
    package_share_dir = get_package_share_directory('asb_webots')
    robot_description_path = os.path.join(package_share_dir, 'resource', 'asb_webots_robot.urdf')

    webots_launcher = WebotsLauncher(
        world=os.path.join(package_share_dir, 'worlds', 'track.wbt')
    )

    webots_controller = WebotsController(
        robot_name='asb_webots_robot',
        parameters=[
            {'robot_description': robot_description_path},
        ]
    )

    return LaunchDescription([
        webots_launcher,
        webots_controller,
        launch.actions.RegisterEventHandler(
            event_handler=launch.event_handlers.OnProcessExit(
                target_action=webots_launcher,
                on_exit=[launch.actions.EmitEvent(event=launch.events.Shutdown())],
            )
        )
    ])
