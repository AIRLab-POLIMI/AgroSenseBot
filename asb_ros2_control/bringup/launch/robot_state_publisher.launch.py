
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.substitutions import Command, FindExecutable, PathJoinSubstitution, LaunchConfiguration

from launch_ros.actions import Node
from ament_index_python import get_package_share_directory as pkg


def generate_launch_description():

    use_vcan0_launch_configuration = LaunchConfiguration("use_vcan0")
    use_vcan0_launch_argument = DeclareLaunchArgument(
        "use_vcan0",
        default_value="false",
        description="Use the virtual CAN network vcan0 instead of the physical CAN network (can2).",
    )

    use_sim_time_launch_configuration = LaunchConfiguration("use_vcan0")
    use_sim_time_launch_argument = DeclareLaunchArgument(
        "use_sim_time",
        default_value="false",
    )

    # Get URDF via xacro
    robot_description_content = Command(
        [
            PathJoinSubstitution([FindExecutable(name="xacro")]),
            " ",
            PathJoinSubstitution([pkg("asb_ros2_control"), "urdf", "asb.urdf.xacro"]),
            " ",
            "test:=",
            use_vcan0_launch_configuration,
        ]
    )

    robot_description = {"robot_description": robot_description_content}

    robot_state_publisher_node = Node(
        package="robot_state_publisher",
        executable="robot_state_publisher",
        name="robot_state_publisher",
        output="both",
        parameters=[
            robot_description,
            {"use_sim_time": use_sim_time_launch_configuration},
        ],
    )

    # Create the launch description and populate
    ld = LaunchDescription()

    ld.add_action(use_vcan0_launch_argument)
    ld.add_action(use_sim_time_launch_argument)

    ld.add_action(robot_state_publisher_node)

    return ld
