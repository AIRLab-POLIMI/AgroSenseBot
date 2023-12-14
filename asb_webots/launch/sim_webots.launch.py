import os
import launch
from launch import LaunchDescription
from ament_index_python.packages import get_package_share_directory as pkg
from webots_ros2_driver.webots_launcher import WebotsLauncher
from webots_ros2_driver.webots_controller import WebotsController


def generate_launch_description():
    webots_launcher = WebotsLauncher(
        world=os.path.join(pkg('asb_webots'), 'worlds', 'asb_gnss_world.wbt')
    )

    webots_controller = WebotsController(
        robot_name='asb_webots_robot',
        parameters=[
            os.path.join(pkg('asb_webots'), 'config', 'asb_webots_driver.yaml'),
            {'robot_description': os.path.join(pkg('asb_webots'), 'config', 'asb_webots_robot.urdf')},
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
