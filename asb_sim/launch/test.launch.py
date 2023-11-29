
import os
import sys

from launch.conditions import IfCondition
from sympy import true
sys.path.insert(0, os.path.join(os.path.dirname(__file__), '..'))  # noqa
sys.path.insert(0, os.path.join(os.path.dirname(__file__), '..', '..', 'launch'))  # noqa

import launch
import launch.actions
import launch.events
from launch.substitutions import LaunchConfiguration, TextSubstitution, Command
from launch.actions import DeclareLaunchArgument, IncludeLaunchDescription, ExecuteProcess
from launch.launch_description_sources import PythonLaunchDescriptionSource

from ament_index_python import get_package_share_directory as pkg

import launch_ros
import launch_ros.events
import launch_ros.events.lifecycle

import lifecycle_msgs.msg


def generate_launch_description():
    # pkg = get_package_share_directory
    path_to_test = os.path.dirname(__file__)

    print_debug_launch_configuration = LaunchConfiguration("print_debug")
    print_debug_launch_argument = DeclareLaunchArgument(
        'print_debug',
        default_value='false',
        description="Whether to print (lots of) additional data.",
    )

    use_simulator_launch_configuration = LaunchConfiguration("use_simulator")
    use_simulator_launch_argument = DeclareLaunchArgument(
        'use_simulator',
        default_value='false',
        description="Whether to send and receive motor velocities from simulator, or compute their values internally.",
    )

    use_gazebo_launch_configuration = LaunchConfiguration('use_gazebo')
    use_gazebo_launch_argument = DeclareLaunchArgument(
        'use_gazebo',
        default_value='false',
        description='Whether to use Gazebo as simulator',
    )

    use_webots_launch_configuration = LaunchConfiguration('use_webots')
    use_webots_launch_argument = DeclareLaunchArgument(
        'use_webots',
        default_value='true',
        description='Whether to use Webots as simulator',
    )

    dummy_vcu_canopen_node_config_arg = DeclareLaunchArgument(
        'dummy_vcu_canopen_node_config',
        default_value=TextSubstitution(text=os.path.join(path_to_test, "..", "config", "dummy_VCU.dcf")),  # TODO set as params only
        description="Path to DCF file to be used for the dummy_VCU CANOpen node.",
    )

    dummy_mdl_canopen_node_config_arg = DeclareLaunchArgument(
        'dummy_mdl_canopen_node_config',
        default_value=TextSubstitution(text=os.path.join(path_to_test, "..", "config", "dummy_MDL.dcf")),
        description="Path to DCF file to be used for the dummy_MDL CANOpen node.",
    )

    dummy_mdr_canopen_node_config_arg = DeclareLaunchArgument(
        'dummy_mdr_canopen_node_config',
        default_value=TextSubstitution(text=os.path.join(path_to_test, "..", "config", "dummy_MDR.dcf")),
        description="Path to DCF file to be used for the dummy_MDR CANOpen node.",
    )

    dummy_fan_canopen_node_config_arg = DeclareLaunchArgument(
        'dummy_fan_canopen_node_config',
        default_value=TextSubstitution(text=os.path.join(path_to_test, "..", "config", "dummy_FAN.dcf")),
        description="Path to DCF file to be used for the dummy_FAN CANOpen node.",
    )

    can_interface_arg = DeclareLaunchArgument(
        'can_interface_name',
        default_value=TextSubstitution(text="vcan0"),
        description="CAN interface name.",
    )

    test_node_name_arg = DeclareLaunchArgument(
        'test_node_name',
        default_value=TextSubstitution(text="test_node"),
        description="Name of the ros node.",
    )

    gazebo_launch = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(os.path.join(pkg("asb_gazebo"), "launch", "sim.launch.py")),
        condition=IfCondition(use_gazebo_launch_configuration)
    )

    webots_launch = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(os.path.join(pkg("asb_webots"), "launch", "sim.launch.py")),
        condition=IfCondition(use_webots_launch_configuration)
    )

    system_test_node = launch_ros.actions.LifecycleNode(
        name=LaunchConfiguration("test_node_name"),
        namespace="system_test",
        package="asb_sim",
        output="screen",
        executable="test_node",
        parameters=[{
            "print_debug": print_debug_launch_configuration,
            "use_simulator": use_simulator_launch_configuration,
            "dummy_VCU_canopen_node_config": LaunchConfiguration("dummy_vcu_canopen_node_config"),
            "dummy_MDL_canopen_node_config": LaunchConfiguration("dummy_mdl_canopen_node_config"),
            "dummy_MDR_canopen_node_config": LaunchConfiguration("dummy_mdr_canopen_node_config"),
            "dummy_FAN_canopen_node_config": LaunchConfiguration("dummy_fan_canopen_node_config"),
            "can_interface_name": LaunchConfiguration("can_interface_name")
        }],
    )
    lifecycle_inactive_state_handler = launch.actions.RegisterEventHandler(
        launch_ros.event_handlers.OnStateTransition(
            target_lifecycle_node=system_test_node,
            goal_state='inactive',
            handle_once=true,
            entities=[
                launch.actions.LogInfo(msg="node reached the 'inactive' state, activating."),
                launch.actions.EmitEvent(event=launch_ros.events.lifecycle.ChangeState(
                    lifecycle_node_matcher=launch.events.matches_action(system_test_node),
                    transition_id=lifecycle_msgs.msg.Transition.TRANSITION_ACTIVATE,
                )),
            ],
        ),
    )
    lifecycle_configure = launch.actions.EmitEvent(
        event=launch_ros.events.lifecycle.ChangeState(
            lifecycle_node_matcher=launch.events.matches_action(system_test_node),
            transition_id=lifecycle_msgs.msg.Transition.TRANSITION_CONFIGURE,
        )
    )

    include_control_launch = IncludeLaunchDescription(
        launch_description_source=PythonLaunchDescriptionSource(os.path.join(pkg('asb_ros2_control'), 'launch/asb_ros2_control.launch.py')),
        launch_arguments={'test': 'true'}.items(),
    )

    include_nav_launch = IncludeLaunchDescription(
        launch_description_source=PythonLaunchDescriptionSource(os.path.join(pkg('asb_nav'), 'launch/gnss_nav.launch.py')),
    )

    ld = launch.LaunchDescription()

    ld.add_action(print_debug_launch_argument)
    ld.add_action(use_simulator_launch_argument)
    ld.add_action(use_gazebo_launch_argument)
    ld.add_action(use_webots_launch_argument)
    ld.add_action(dummy_vcu_canopen_node_config_arg)
    ld.add_action(dummy_mdl_canopen_node_config_arg)
    ld.add_action(dummy_mdr_canopen_node_config_arg)
    ld.add_action(dummy_fan_canopen_node_config_arg)
    ld.add_action(can_interface_arg)

    ld.add_action(gazebo_launch)
    ld.add_action(webots_launch)
    ld.add_action(test_node_name_arg)
    ld.add_action(system_test_node)
    ld.add_action(lifecycle_inactive_state_handler)
    ld.add_action(lifecycle_configure)
    ld.add_action(include_control_launch)
    ld.add_action(include_nav_launch)

    return ld

