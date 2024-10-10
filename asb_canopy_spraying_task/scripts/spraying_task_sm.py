#!/usr/bin/python3

import os.path
from datetime import datetime

import threading

import smach
from smach import State, StateMachine, cb_interface

import rclpy
from rclpy import Future
from rclpy.qos import QoSProfile, ReliabilityPolicy, DurabilityPolicy, HistoryPolicy
from rclpy.client import Client
from rclpy.timer import Timer
from rclpy.time import Time
from rclpy.node import Node
from rclpy.duration import Duration
from asb_msgs.msg import Heartbeat, PlatformState, SprayRegulatorStatus
from asb_msgs.srv import StartRowSpraying, StartRowSpraying_Request, StopRowSpraying, StopRowSpraying_Request
from std_msgs.msg import String

from enum import Enum
from typing_extensions import Self

from control_mode_manager import ControlModeManager
from plan_manager import PlanManager
from navigation_manager import NavigationManager, NavigationActionStatus
from spraying_task_plan import SprayingTaskPlan, TaskPlanItem, TaskPlanItemType, TaskPlanRow


class SprayState(Enum):
    NOT_SPRAYING = 0
    REQUESTED = 1
    STARTING = 2
    STARTED = 3
    FAILED = 4


class ControlMode(Enum):
    STOP = 0
    MANUAL = 1
    AUTO = 2
    OVERRIDE = 3
    UNKNOWN = 4

    @classmethod
    def from_msg(cls, control_mode_code: int) -> Self:
        if control_mode_code == 0:
            return ControlMode.STOP
        elif control_mode_code == 1:
            return ControlMode.MANUAL
        elif control_mode_code == 2:
            return ControlMode.AUTO
        elif control_mode_code == 3:
            return ControlMode.OVERRIDE
        else:
            return ControlMode.UNKNOWN


class Chronometer:
    node: Node | None = None

    def __init__(self):
        self._t0 = self.node.get_clock().now()
        self._t = self._t0

    def delta(self):
        delta = self.node.get_clock().now() - self._t
        self._t = self.node.get_clock().now()
        return delta.nanoseconds / 1e9

    def total(self):
        total = self.node.get_clock().now() - self._t0
        return total.nanoseconds / 1e9


class SprayingTaskPlanExecutor(Node):
    def __init__(self):
        super().__init__('spraying_task_plan_executor')

        Chronometer.node = self

        default_task_plan_file_path = os.path.expanduser("~/asb_canopy_spraying_task_plan.yaml")
        self.declare_parameter('task_plan_file_path', default_task_plan_file_path)
        self.task_plan_file_path = os.path.expanduser(self.get_parameter('task_plan_file_path').get_parameter_value().string_value)
        self.get_logger().info(f"task_plan_file_path set to {self.task_plan_file_path}")

        date_stamp = datetime.now().strftime("%Y-%m-%d")
        default_log_dir_path = os.path.join(os.path.expanduser("~/asb_logs/"), date_stamp)
        self.declare_parameter('log_dir_path', default_log_dir_path)
        self.task_log_dir_path = os.path.expanduser(self.get_parameter('log_dir_path').get_parameter_value().string_value)
        self.get_logger().info(f"log_dir_path set to {self.task_log_dir_path}")

        self.task_result_filename = datetime.now().strftime("%Y-%m-%d__%H-%M-%S__spraying_task_plan_result.yaml")

        default_auto_set_control_mode = False
        self.declare_parameter('auto_set_control_mode', default_auto_set_control_mode)
        self.auto_set_control_mode = self.get_parameter('auto_set_control_mode').get_parameter_value().bool_value
        if self.auto_set_control_mode:
            self.get_logger().info(
                f"\n"
                f"*********************\n"
                f"* AUTO CONTROL MODE *\n"
                f"*********************\n"
            )

        default_dry_run = False
        self.declare_parameter('dry_run', default_dry_run)
        self.dry_run = self.get_parameter('dry_run').get_parameter_value().bool_value
        if self.dry_run:
            self.get_logger().info(
                f"\n"
                f"***********\n"
                f"* DRY RUN *\n"
                f"***********\n"
            )

        if self.dry_run:
            self.loop = False
        else:
            default_loop = False
            self.declare_parameter('loop', default_loop)
            self.loop = self.get_parameter('loop').get_parameter_value().bool_value
            if self.loop:
                self.get_logger().info(
                    f"\n"
                    f"***********\n"
                    f"*  LOOP   *\n"
                    f"***********\n"
                )

        default_base_frame = "base_link"
        self.declare_parameter('base_frame', default_base_frame)
        self.base_frame = self.get_parameter('base_frame').get_parameter_value().string_value

        default_start_up_timeout = 30.0  # s
        self.declare_parameter('start_up_timeout', float(default_start_up_timeout))
        self.start_up_timeout = self.get_parameter('start_up_timeout').get_parameter_value().double_value

        default_robot_pose_time_tolerance = 0.2  # s
        self.declare_parameter('robot_pose_time_tolerance', float(default_robot_pose_time_tolerance))
        self.robot_pose_time_tolerance = Duration(seconds=self.get_parameter('robot_pose_time_tolerance').get_parameter_value().double_value)

        default_platform_status_timeout = 0.2  # s
        self.declare_parameter('platform_status_timeout', float(default_platform_status_timeout))
        self.platform_status_timeout = Duration(seconds=self.get_parameter('platform_status_timeout').get_parameter_value().double_value)

        default_start_spray_regulator_timeout = 10.0  # s
        self.declare_parameter('start_spray_regulator_timeout', float(default_start_spray_regulator_timeout))
        self.start_spray_regulator_timeout = self.get_parameter('start_spray_regulator_timeout').get_parameter_value().double_value

        default_start_navigation_action_timeout = 10.0  # s
        self.declare_parameter('start_navigation_action_timeout', float(default_start_navigation_action_timeout))
        self.start_navigation_action_timeout = self.get_parameter('start_navigation_action_timeout').get_parameter_value().double_value

        default_min_loop_rate = 25  # Hz
        self.declare_parameter('min_loop_rate', float(default_min_loop_rate))
        self.min_loop_rate = self.get_parameter('min_loop_rate').get_parameter_value().double_value
        self.max_loop_duration = 1 / self.min_loop_rate

        default_target_loop_rate = 50  # Hz
        self.declare_parameter('target_loop_rate', float(default_target_loop_rate))
        self.target_loop_rate = self.get_parameter('target_loop_rate').get_parameter_value().double_value
        self.target_loop_duration = 1 / self.target_loop_rate

        default_max_loop_rate = 100  # Hz
        self.declare_parameter('max_loop_rate', float(default_max_loop_rate))
        self.max_loop_rate = self.get_parameter('max_loop_rate').get_parameter_value().double_value
        self.min_loop_duration = 1 / self.max_loop_rate

        # load task plan
        self.task_plan: SprayingTaskPlan = SprayingTaskPlan.load(self.task_plan_file_path)
        if len(self.task_plan.items):
            self.get_logger().info(f"loaded task plan with item: {self.task_plan.get_item_ids()}")
        else:
            self.task_plan.generate_items(only_inter_row_items=True)
            self.get_logger().info(f"loaded task plan with no items, auto generated items: {self.task_plan.get_item_ids()}")

        if not len(self.task_plan.get_item_ids()):
            self.get_logger().error(f"empty task plan")
            return

        # variables for task execution
        self.item_index: int = 0
        self.current_item: TaskPlanItem | None = None
        self.loop_operations_chrono: Chronometer | None = None
        self.run_chrono: Chronometer | None = None
        self.nav_chrono: Chronometer | None = None
        self.start_navigation_action_chrono: Chronometer | None = None
        self.start_spray_regulator_chrono: Chronometer | None = None
        self.left_spraying_state: SprayState = SprayState.NOT_SPRAYING
        self.left_row: TaskPlanRow | None = None
        self.right_spraying_state: SprayState = SprayState.NOT_SPRAYING
        self.right_row: TaskPlanRow | None = None
        self.heartbeat_alive_bit: bool = False
        self.last_platform_status_msg: PlatformState | None = None
        self.stop_platform: bool = True

        # managers
        self.control_mode_manager = ControlModeManager(node=self)
        self.plan_manager = PlanManager(node=self, task_plan=self.task_plan)
        self.navigation_manager = NavigationManager(node=self)

        # publishers, subscribers, timers and loop rate
        qos_reliable_transient_local_depth_10 = QoSProfile(
            reliability=ReliabilityPolicy.RELIABLE,
            durability=DurabilityPolicy.TRANSIENT_LOCAL,
            history=HistoryPolicy.KEEP_LAST,
            depth=10
        )
        self.heartbeat_pub = self.create_publisher(Heartbeat, '/asb_platform_controller/heartbeat', rclpy.qos.qos_profile_sensor_data)
        self.current_item_pub = self.create_publisher(String, '~/current_item', 10)
        self.platform_status_sub = self.create_subscription(PlatformState, '/asb_platform_controller/platform_state', self.platform_status_callback, 10)
        self.start_row_spraying_service = self.create_client(StartRowSpraying, 'start_row_spraying')
        self.stop_row_spraying_service = self.create_client(StopRowSpraying, 'stop_row_spraying')
        self.spray_regulator_status_sub = self.create_subscription(SprayRegulatorStatus, 'spray_regulator_status', self.spray_regulator_status_callback, qos_reliable_transient_local_depth_10)
        self.start_left_row_spraying_timeout_timer: Timer | None = None
        self.start_right_row_spraying_timeout_timer: Timer | None = None
        self.loop_rate = self.create_rate(self.target_loop_rate)

        positioning_approach_sm = StateMachine(outcomes=['success', 'failure'])
        with positioning_approach_sm:
            StateMachine.add(
                label='start_positioning_approach',
                state=CallbackState(self.start_positioning_approach_sm_cb, class_instance=self), transitions={
                    'success': 'wait_navigation_started',
                    'failure': 'failure',
                }
            )
            StateMachine.add(
                label='wait_navigation_started',
                state=CallbackState(self.wait_navigation_started_sm_cb, class_instance=self), transitions={
                    'success': 'wait_navigation_complete',
                    'waiting': 'wait_navigation_started',
                    'failure': 'failure',
                }
            )
            StateMachine.add(
                label='wait_navigation_complete',
                state=CallbackState(self.wait_navigation_complete_sm_cb, class_instance=self), transitions={
                    'success': 'success',
                    'waiting': 'wait_navigation_complete',
                    'stop': 'stop_navigation',
                    'failure': 'failure',
                }
            )
            StateMachine.add(
                label='stop_navigation',
                state=CallbackState(self.stop_navigation_sm_cb, class_instance=self), transitions={
                    'success': 'failure',
                }
            )

        straightening_approach_sm = StateMachine(outcomes=['success', 'failure'])
        with straightening_approach_sm:
            StateMachine.add(
                label='start_straightening_approach',
                state=CallbackState(self.start_straightening_approach_sm_cb, class_instance=self), transitions={
                    'success': 'wait_navigation_started',
                    'failure': 'failure',
                }
            )
            StateMachine.add(
                label='wait_navigation_started',
                state=CallbackState(self.wait_navigation_started_sm_cb, class_instance=self), transitions={
                    'success': 'wait_navigation_complete',
                    'waiting': 'wait_navigation_started',
                    'failure': 'failure',
                }
            )
            StateMachine.add(
                label='wait_navigation_complete',
                state=CallbackState(self.wait_navigation_complete_sm_cb, class_instance=self), transitions={
                    'success': 'success',
                    'waiting': 'wait_navigation_complete',
                    'stop': 'stop_navigation',
                    'failure': 'failure',
                }
            )
            StateMachine.add(
                label='stop_navigation',
                state=CallbackState(self.stop_navigation_sm_cb, class_instance=self), transitions={
                    'success': 'failure',
                }
            )

        init_spraying_sm = StateMachine(outcomes=['success', 'failure'])
        with init_spraying_sm:
            StateMachine.add(
                label='start_spray_regulator',
                state=CallbackState(self.start_spray_regulator_sm_cb, class_instance=self), transitions={
                    'success': 'wait_spray_regulator_start',
                    'failure': 'stop_spray_regulator',
                }
            )
            StateMachine.add(
                label='wait_spray_regulator_start',
                state=CallbackState(self.wait_spray_regulator_start_sm_cb, class_instance=self), transitions={
                    'success': 'success',
                    'waiting': 'wait_spray_regulator_start',
                    'failure': 'stop_spray_regulator',
                }
            )
            StateMachine.add(
                label='stop_spray_regulator',
                state=CallbackState(self.stop_spray_regulator_sm_cb, class_instance=self), transitions={
                    'success': 'failure',
                }
            )

        inter_row_navigation_sm = StateMachine(outcomes=['success', 'failure'])
        with inter_row_navigation_sm:
            StateMachine.add(
                label='start_inter_row_navigation',
                state=CallbackState(self.start_inter_row_navigation_sm_cb, class_instance=self), transitions={
                    'success': 'wait_navigation_started',
                    'failure': 'failure',
                }
            )
            StateMachine.add(
                label='wait_navigation_started',
                state=CallbackState(self.wait_navigation_started_sm_cb, class_instance=self), transitions={
                    'success': 'wait_navigation_complete',
                    'waiting': 'wait_navigation_started',
                    'failure': 'failure',
                }
            )
            StateMachine.add(
                label='wait_navigation_complete',
                state=CallbackState(self.wait_navigation_complete_sm_cb, class_instance=self), transitions={
                    'success': 'success',
                    'waiting': 'wait_navigation_complete',
                    'stop': 'stop_navigation',
                    'failure': 'failure',
                }
            )
            StateMachine.add(
                label='stop_navigation',
                state=CallbackState(self.stop_navigation_sm_cb, class_instance=self), transitions={
                    'success': 'failure',
                }
            )

        # main state machine
        self.main_sm = StateMachine(outcomes=['end'])
        with self.main_sm:
            StateMachine.add(
                label='setup',
                state=CallbackState(self.setup_sm_cb, class_instance=self), transitions={
                    'success': 'select_first_item',
                    'failure': 'end',
                }
            )
            StateMachine.add(
                label='select_first_item',
                state=CallbackState(self.select_first_item_sm_cb, class_instance=self), transitions={
                    'success': 'positioning_approach',
                }
            )
            StateMachine.add(
                label='positioning_approach',
                state=positioning_approach_sm, transitions={
                    'success': 'straightening_approach',
                    'failure': 'stop_platform_and_wait_operator_1',
                }
            )
            StateMachine.add(
                label='straightening_approach',
                state=straightening_approach_sm, transitions={
                    'success': 'init_spraying',
                    'failure': 'stop_platform_and_wait_operator_1',
                }
            )
            StateMachine.add(
                label='stop_platform_and_wait_operator_1',
                state=CallbackState(self.stop_platform_and_wait_operator_sm_cb, class_instance=self), transitions={
                    'success': 'positioning_approach',
                }
            )
            StateMachine.add(
                label='init_spraying',
                state=init_spraying_sm, transitions={
                    'success': 'inter_row_navigation',
                    'failure': 'stop_platform_and_wait_operator_2',
                }
            )
            StateMachine.add(
                label='stop_platform_and_wait_operator_2',
                state=CallbackState(self.stop_platform_and_wait_operator_sm_cb, class_instance=self), transitions={
                    'success': 'init_spraying',
                }
            )
            StateMachine.add(
                label='inter_row_navigation',
                state=inter_row_navigation_sm, transitions={
                    'success': 'stop_spray_regulator_1',
                    'failure': 'stop_spray_regulator_2',
                }
            )
            StateMachine.add(
                label='stop_spray_regulator_1',
                state=CallbackState(self.stop_spray_regulator_sm_cb, class_instance=self), transitions={
                    'success': 'select_next_item',
                }
            )
            StateMachine.add(
                label='stop_spray_regulator_2',
                state=CallbackState(self.stop_spray_regulator_sm_cb, class_instance=self), transitions={
                    'success': 'stop_platform_and_wait_operator_3',
                }
            )
            StateMachine.add(
                label='stop_platform_and_wait_operator_3',
                state=CallbackState(self.stop_platform_and_wait_operator_sm_cb, class_instance=self), transitions={
                    'success': 'straightening_approach',
                }
            )
            StateMachine.add(
                label='select_next_item',
                state=CallbackState(self.select_next_item_sm_cb, class_instance=self), transitions={
                    'success': 'positioning_approach',
                    'task_complete': 'end',
                }
            )

    @cb_interface(outcomes=['success', 'failure'])
    def setup_sm_cb(self) -> str:
        if not self.dry_run:
            # wait for localization
            self.get_logger().info(f"waiting for robot pose...")
            robot_pose_chrono = Chronometer()
            robot_pose = self.navigation_manager.get_robot_pose(timeout=self.start_up_timeout)
            if robot_pose is not None:
                self.get_logger().info(f"robot pose received (took {robot_pose_chrono.total():.4f} s)")
            else:
                self.get_logger().fatal(f"robot pose not received (took {robot_pose_chrono.total():.4f} s), aborting task")
                return 'failure'

            # wait for navigation stack
            self.get_logger().info(f"waiting for navigation stack...")
            nav_stack_chrono = Chronometer()
            nav_stack_ready = self.navigation_manager.wait_navigation_stack_is_ready(timeout=self.start_up_timeout)
            if nav_stack_ready:
                self.get_logger().info(f"navigation stack is ready (took {nav_stack_chrono.total():.4f} s)")
            else:
                self.get_logger().fatal(f"navigation stack timeout (took {nav_stack_chrono.total():.4f} s), aborting task")
                return 'failure'

            # wait for spray regulator
            self.get_logger().info(f"waiting for spray regulator...")
            spray_regulator_chrono = Chronometer()
            spray_regulator_ready = self.wait_spray_regulator_is_ready(timeout=self.start_up_timeout)
            if spray_regulator_ready:
                self.get_logger().info(f"spray regulator is ready (took {spray_regulator_chrono.total():.4f} s), ready to start the task\n\n")
            else:
                self.get_logger().fatal(f"spray regulator timeout (took {spray_regulator_chrono.total():.4f} s), aborting task")
                return 'failure'

        self.prepare_task_log()

        self.plan_manager.setup()

        self.stop_platform_and_wait_control_mode_manual_to_auto()

        # clear costmaps
        self.navigation_manager.clear_local_costmap()
        self.navigation_manager.clear_global_costmap()

        return 'success'

    @cb_interface(outcomes=['success'])
    def select_first_item_sm_cb(self) -> str:
        self.run_chrono = Chronometer()
        self.do_loop_operations_and_sleep()

        self.item_index = 0
        self.current_item = self.task_plan.items[self.item_index]
        self.get_logger().info(f"\n*************\nSTARTING ITEM {self.item_index} --- {self.current_item.get_item_id()}")
        return 'success'

    @cb_interface(outcomes=['success', 'task_complete'])
    def select_next_item_sm_cb(self) -> str:
        self.item_index += 1
        if self.item_index < len(self.task_plan.items):
            self.current_item = self.task_plan.items[self.item_index]
            self.get_logger().info(f"\n*************\nSTARTING ITEM {self.item_index} --- {self.current_item.get_item_id()}")
            return 'success'
        else:
            self.get_logger().info(f"finished task plan in {self.run_chrono.delta():.1f} s")
            if self.loop:
                self.get_logger().info(
                    f"\n*************************************************************************\n"
                    f"                        looping back to start\n"
                    f"*************************************************************************\n"
                )
                self.item_index = 0
                self.current_item = self.task_plan.items[self.item_index]
                return 'success'
            else:
                return 'task_complete'

    @cb_interface(outcomes=['success', 'waiting', 'failure'])
    def wait_navigation_started_sm_cb(self) -> str:
        self.do_loop_operations_and_sleep(current_item=self.current_item)

        if self.start_navigation_action_chrono.total() > self.start_navigation_action_timeout:
            self.get_logger().error(f"navigation could not be started before timeout [{self.start_navigation_action_timeout} s] for item {self.current_item.get_item_id()}")
            return 'failure'

        if self.navigation_manager.navigation_action_status == NavigationActionStatus.FAILED_TO_START:
            self.get_logger().error(f"navigation could not be started for item {self.current_item.get_item_id()}")
            return 'failure'

        if self.navigation_manager.navigation_action_status in [NavigationActionStatus.NOT_STARTED, NavigationActionStatus.REQUESTED]:
            return 'waiting'

        return 'success'

    @cb_interface(outcomes=['success', 'failure'])
    def start_positioning_approach_sm_cb(self) -> str:
        self.start_navigation_action_chrono = Chronometer()
        self.navigation_manager.start_positioning_approach(self.current_item)
        return 'success'

    @cb_interface(outcomes=['success', 'failure'])
    def start_straightening_approach_sm_cb(self) -> str:
        self.start_navigation_action_chrono = Chronometer()
        self.navigation_manager.start_straightening_approach(self.current_item)
        return 'success'

    @cb_interface(outcomes=['success', 'waiting', 'stop', 'failure'])
    def wait_navigation_complete_sm_cb(self) -> str:
        self.nav_chrono = Chronometer()
        self.do_loop_operations_and_sleep(current_item=self.current_item)

        if self.dry_run:
            self.get_logger().info(f"navigation completed in {self.nav_chrono.total():.3f} s for item {self.current_item.get_item_id()} (DRY RUN)")
            return 'success'

        if self.navigation_manager.navigation_action_status == NavigationActionStatus.SUCCEEDED:
            self.get_logger().info(f"navigation succeeded in {self.nav_chrono.total():.3f} s for item {self.current_item.get_item_id()}")
            return 'success'

        if self.navigation_manager.navigation_action_status == NavigationActionStatus.FAILED:
            self.get_logger().error(f"navigation failed after {self.nav_chrono.total():.3f} s for item {self.current_item.get_item_id()}")
            return 'failure'

        if not self.get_control_mode() == ControlMode.AUTO:
            return 'stop'

        return 'waiting'

    @cb_interface(outcomes=['success'])
    def stop_navigation_sm_cb(self) -> str:
        self.navigation_manager.cancel_navigation_action()
        return 'success'

    @cb_interface(outcomes=['success'])
    def stop_platform_and_wait_operator_sm_cb(self) -> str:
        self.stop_platform_and_wait_control_mode_manual_to_auto()
        return 'success'

    @cb_interface(outcomes=['success', 'failure'])
    def start_spray_regulator_sm_cb(self) -> str:
        self.start_spray_regulator_chrono = Chronometer()
        self.start_spray_regulator(self.current_item)
        return 'success'

    @cb_interface(outcomes=['success', 'waiting', 'failure'])
    def wait_spray_regulator_start_sm_cb(self) -> str:
        self.do_loop_operations_and_sleep(current_item=self.current_item)

        if self.is_spray_regulator_started():
            self.get_logger().info(f"spray regulator started")
            return 'success'

        if self.start_spray_regulator_chrono.total() > self.start_spray_regulator_timeout:
            self.get_logger().error(f"spray regulator failed to start before timeout [{self.start_spray_regulator_timeout} s] for item {self.current_item.get_item_id()}")
            return 'failure'

        if self.is_spray_regulator_failed():
            self.get_logger().error(f"spray regulator failed")
            return 'failure'

        return 'waiting'

    @cb_interface(outcomes=['success'])
    def stop_spray_regulator_sm_cb(self) -> str:
        self.stop_spray_regulator()
        return 'success'

    @cb_interface(outcomes=['success', 'failure'])
    def start_inter_row_navigation_sm_cb(self) -> str:
        self.start_navigation_action_chrono = Chronometer()
        self.navigation_manager.start_inter_row_navigation(self.current_item)
        return 'success'

    def run(self):
        self.main_sm.execute()

    """
     Called at the end of the task. Not called in case of KeyboardInterrupt or other exceptions.
    """
    def end(self):
        self.get_logger().info(f"terminating node")
        rclpy.shutdown()

    """
     Always called at the very end of the task before self is destructed.
     Also called in case of KeyboardInterrupt, but not other exceptions.
    """
    def terminate(self):
        self.get_logger().info(f"doing end work")
        self.get_logger().info(f"requesting to cancel navigation action")
        self.navigation_manager.cancel_navigation_action()
        self.get_logger().info(f"writing task results")
        self.log_task_results()

    def prepare_task_log(self) -> None:
        if not os.path.isdir(self.task_log_dir_path):
            os.makedirs(self.task_log_dir_path)

    def log_task_results(self) -> None:
        chrono = Chronometer()
        self.task_plan.write(os.path.expanduser(os.path.join(self.task_log_dir_path, self.task_result_filename)))
        self.get_logger().info(f"wrote task results, it took {chrono.total():.3f} s")

    def do_loop_operations_and_sleep(self, current_item: TaskPlanItem = None) -> None:
        if current_item is not None:
            self.current_item_pub.publish(String(data=current_item.get_item_id()))
        else:
            self.current_item_pub.publish(String())

        if not self.stop_platform:
            # publish heartbeat message
            self.heartbeat_alive_bit = not self.heartbeat_alive_bit
            self.heartbeat_pub.publish(Heartbeat(stamp=self.get_clock().now().to_msg(), alive_bit=self.heartbeat_alive_bit))

        # [!] always sleep at the end to limit the loop rate, don't skip this by returning
        self.loop_rate.sleep()

        # check we are running this function at an acceptable rate
        if self.loop_operations_chrono is None:
            self.loop_operations_chrono = Chronometer()
        else:
            loop_operations_delta = self.loop_operations_chrono.delta()
            if loop_operations_delta > self.max_loop_duration:
                self.get_logger().warn(f"LOW LOOP RATE loop_operations_delta [{loop_operations_delta:.3f} s] > max_loop_duration [{self.max_loop_duration:.3f} s]")
            if loop_operations_delta < self.min_loop_duration:
                self.get_logger().warn(f"HIGH LOOP RATE loop_operations_delta [{loop_operations_delta:.3f} s] < min_loop_duration [{self.min_loop_duration:.3f} s], {1/loop_operations_delta:.3f} Hz")

    def platform_status_callback(self, platform_status: PlatformState):
        self.last_platform_status_msg = platform_status

    def get_control_mode(self):
        platform_status_age = self.get_clock().now() - Time.from_msg(self.last_platform_status_msg.stamp)
        if platform_status_age > self.platform_status_timeout:
            self.get_logger().error(f"platform_status_age: {platform_status_age.nanoseconds/1e9}")
            return ControlMode.UNKNOWN
        else:
            return ControlMode.from_msg(self.last_platform_status_msg.control_mode)

    def stop_platform_and_wait_control_mode_manual_to_auto(self) -> None:
        if self.dry_run:
            return

        self.stop_platform = True

        if self.auto_set_control_mode:
            self.control_mode_manager.set_control_mode_manual()  # only has effect in simulator

        while rclpy.ok() and self.get_control_mode() != ControlMode.MANUAL:
            self.do_loop_operations_and_sleep()
            self.get_logger().info(f"WAITING control mode switch to MANUAL", throttle_duration_sec=10.0)

        self.stop_platform = False

        while rclpy.ok() and self.get_control_mode() != ControlMode.AUTO:
            self.do_loop_operations_and_sleep()
            self.get_logger().info(f"WAITING control mode switch to AUTO", throttle_duration_sec=10.0)

            if self.auto_set_control_mode:
                self.control_mode_manager.set_control_mode_auto()  # only has effect in simulator

    def start_spray_regulator(self, item: TaskPlanItem) -> None:
        if item.get_type() != TaskPlanItemType.ROW:
            self.get_logger().error(f"trying to start row spraying for item of type different than ROW")
            return

        if self.left_spraying_state != SprayState.NOT_SPRAYING or self.right_spraying_state != SprayState.NOT_SPRAYING:
            self.get_logger().error(f"trying to start row spraying while already executing")
            return

        if item.get_left_row_id() is None and item.get_right_row_id() is None:
            self.get_logger().error(f"neither left nor right row spraying in row item [{item.get_item_id()}]")

        if item.get_left_row_id() is not None:
            self.left_row = self.task_plan.get_row(item.get_left_row_id())
            self.get_logger().info(f"starting spray regulator for left row [{self.left_row.get_row_id()}]")
            if self.dry_run:
                self.left_spraying_state = SprayState.STARTED
            else:
                left_response_future: Future = self.start_row_spraying_service.call_async(StartRowSpraying_Request(
                    row_id=self.left_row.get_row_id(),
                    side=StartRowSpraying_Request.SIDE_LEFT,
                    start=self.left_row.get_start_point(),
                    end=self.left_row.get_end_point()
                ))
                left_response_future.add_done_callback(self.start_left_row_spraying_response_callback)
                self.start_left_row_spraying_timeout_timer = self.create_timer(1.0, self.start_left_row_spraying_timeout_callback)
                self.left_spraying_state = SprayState.REQUESTED

        if item.get_right_row_id() is not None:
            self.right_row = self.task_plan.get_row(item.get_right_row_id())
            self.get_logger().info(f"starting spray regulator for right row [{self.right_row.get_row_id()}]")
            if self.dry_run:
                self.right_spraying_state = SprayState.STARTED
            else:
                right_response_future: Future = self.start_row_spraying_service.call_async(StartRowSpraying_Request(
                    row_id=self.right_row.get_row_id(),
                    side=StartRowSpraying_Request.SIDE_RIGHT,
                    start=self.right_row.get_start_point(),
                    end=self.right_row.get_end_point()
                ))
                right_response_future.add_done_callback(self.start_right_row_spraying_response_callback)
                self.start_right_row_spraying_timeout_timer = self.create_timer(1.0, self.start_right_row_spraying_timeout_callback)
                self.right_spraying_state = SprayState.REQUESTED

    def start_left_row_spraying_timeout_callback(self) -> None:
        self.get_logger().warn(f"left spraying service request timed out, retrying")
        self.start_left_row_spraying_timeout_timer.cancel()
        left_response_future: Future = self.start_row_spraying_service.call_async(StartRowSpraying_Request(
            row_id=self.left_row.get_row_id(),
            side=StartRowSpraying_Request.SIDE_LEFT,
            start=self.left_row.get_start_point(),
            end=self.left_row.get_end_point()
        ))
        left_response_future.add_done_callback(self.start_left_row_spraying_response_callback)
        self.start_left_row_spraying_timeout_timer = self.create_timer(1.0, self.start_left_row_spraying_timeout_callback)

    def start_left_row_spraying_response_callback(self, response_future) -> None:
        self.start_left_row_spraying_timeout_timer.cancel()
        if response_future.result().result:
            self.left_spraying_state = SprayState.STARTING
            self.get_logger().info(f"left_spraying_state: STARTING")
        else:
            self.left_spraying_state = SprayState.FAILED
            self.get_logger().error(f"left_spraying_state: FAILED")

    def start_right_row_spraying_timeout_callback(self) -> None:
        self.get_logger().warn(f"right spraying service request timed out, retrying")
        self.start_right_row_spraying_timeout_timer.cancel()
        right_response_future: Future = self.start_row_spraying_service.call_async(StartRowSpraying_Request(
            row_id=self.right_row.get_row_id(),
            side=StartRowSpraying_Request.SIDE_RIGHT,
            start=self.right_row.get_start_point(),
            end=self.right_row.get_end_point()
        ))
        right_response_future.add_done_callback(self.start_right_row_spraying_response_callback)
        self.start_right_row_spraying_timeout_timer = self.create_timer(1.0, self.start_right_row_spraying_timeout_callback)

    def start_right_row_spraying_response_callback(self, response_future) -> None:
        self.start_right_row_spraying_timeout_timer.cancel()
        if response_future.result().result:
            self.right_spraying_state = SprayState.STARTING
            self.get_logger().info(f"right_spraying_state: STARTING")
        else:
            self.right_spraying_state = SprayState.FAILED
            self.get_logger().error(f"right_spraying_state: FAILED")

    def spray_regulator_status_callback(self, status_msg: SprayRegulatorStatus) -> None:
        if status_msg.status == SprayRegulatorStatus.STATUS_FAILED:
            if self.left_spraying_state != SprayState.FAILED and self.left_row is not None and status_msg.row_id == self.left_row.get_row_id():
                self.left_spraying_state = SprayState.FAILED
                self.get_logger().error(f"left_spraying_state: FAILED")
            if self.right_spraying_state != SprayState.FAILED and self.right_row is not None and status_msg.row_id == self.right_row.get_row_id():
                self.right_spraying_state = SprayState.FAILED
                self.get_logger().error(f"right_spraying_state: FAILED")

        elif status_msg.status == SprayRegulatorStatus.STATUS_OK:
            if self.left_spraying_state == SprayState.STARTING and self.left_row is not None and status_msg.row_id == self.left_row.get_row_id():
                self.left_spraying_state = SprayState.STARTED
                self.get_logger().info(f"left_spraying_state: STARTED")
            if self.right_spraying_state == SprayState.STARTING and self.right_row is not None and status_msg.row_id == self.right_row.get_row_id():
                self.right_spraying_state = SprayState.STARTED
                self.get_logger().info(f"right_spraying_state: STARTED")

        elif status_msg.status == SprayRegulatorStatus.STATUS_STOPPED:
            if self.left_spraying_state != SprayState.NOT_SPRAYING and self.left_row is not None and status_msg.row_id == self.left_row.get_row_id():
                self.left_spraying_state = SprayState.NOT_SPRAYING
                self.left_row = None
                self.get_logger().info(f"left_spraying_state: NOT_SPRAYING")
            if self.right_spraying_state != SprayState.NOT_SPRAYING and self.right_row is not None and status_msg.row_id == self.right_row.get_row_id():
                self.right_spraying_state = SprayState.NOT_SPRAYING
                self.right_row = None
                self.get_logger().info(f"right_spraying_state: NOT_SPRAYING")

    def stop_spray_regulator(self) -> None:
        if self.left_spraying_state == SprayState.NOT_SPRAYING and self.right_spraying_state == SprayState.NOT_SPRAYING:
            self.get_logger().error(f"trying to stop row spraying while already stopped")
            return

        if self.left_row is not None:
            self.get_logger().info(f"stopping spray regulator for left row [{self.left_row.get_row_id()}]")
            if self.dry_run:
                self.left_spraying_state = SprayState.NOT_SPRAYING
                self.left_row = None
            else:
                if not self.start_left_row_spraying_timeout_timer.is_canceled():
                    self.start_left_row_spraying_timeout_timer.cancel()
                left_response_future: Future = self.stop_row_spraying_service.call_async(StopRowSpraying_Request(
                    row_id=self.left_row.get_row_id()
                ))
                left_response_future.add_done_callback(self.stop_left_row_spraying_response_callback)

        if self.right_row is not None:
            self.get_logger().info(f"stopping spray regulator for right row [{self.right_row.get_row_id()}]")
            if self.dry_run:
                self.right_spraying_state = SprayState.NOT_SPRAYING
                self.right_row = None
            else:
                if not self.start_right_row_spraying_timeout_timer.is_canceled():
                    self.start_right_row_spraying_timeout_timer.cancel()
                right_response_future: Future = self.stop_row_spraying_service.call_async(StopRowSpraying_Request(
                    row_id=self.right_row.get_row_id()
                ))
                right_response_future.add_done_callback(self.stop_right_row_spraying_response_callback)

    def stop_left_row_spraying_response_callback(self, response_future) -> None:
        if response_future.result().result:
            self.left_spraying_state = SprayState.NOT_SPRAYING
            self.get_logger().info(f"left_spraying_state: NOT_SPRAYING")
        else:
            self.left_spraying_state = SprayState.FAILED
            self.get_logger().error(f"left_spraying_state: FAILED")

    def stop_right_row_spraying_response_callback(self, response_future) -> None:
        if response_future.result().result:
            self.right_spraying_state = SprayState.NOT_SPRAYING
            self.get_logger().info(f"right_spraying_state: NOT_SPRAYING")
        else:
            self.right_spraying_state = SprayState.FAILED
            self.get_logger().error(f"right_spraying_state: FAILED")

    def is_spray_regulator_started(self) -> bool:
        if self.left_spraying_state != SprayState.NOT_SPRAYING and self.right_spraying_state != SprayState.NOT_SPRAYING:
            return self.left_spraying_state == SprayState.STARTED and self.right_spraying_state == SprayState.STARTED
        elif self.left_spraying_state != SprayState.NOT_SPRAYING:
            return self.left_spraying_state == SprayState.STARTED
        elif self.right_spraying_state != SprayState.NOT_SPRAYING:
            return self.right_spraying_state == SprayState.STARTED
        else:
            self.get_logger().error(f"waiting for spray regulator, but no start request was made")
            return False

    def is_spray_regulator_failed(self) -> bool:
        return self.left_spraying_state == SprayState.FAILED or self.right_spraying_state == SprayState.FAILED

    def wait_spray_regulator_is_ready(self, timeout: float) -> bool:
        return self.wait_for_service(self.start_row_spraying_service, timeout=timeout)

    def wait_for_service(self, service: Client, timeout: float) -> bool:
        self.get_logger().info(f"waiting for {service.srv_name} service")
        timeout_chrono = Chronometer()
        while rclpy.ok() and not service.wait_for_service(timeout_sec=0.05):
            self.get_logger().info(f"{service.srv_name} service not available, waiting", throttle_duration_sec=1.0)
            if timeout_chrono.total() > timeout:
                return False
        return True


class CallbackState(State):
    def __init__(self, cb, class_instance: SprayingTaskPlanExecutor):
        State.__init__(self, outcomes=list(), input_keys=list(), output_keys=list(), io_keys=list())
        self._cb = cb
        self._cb_class_instance = class_instance

        if smach.util.has_smach_interface(cb):
            self._cb_outcomes = cb.get_registered_outcomes()
            self.register_outcomes(self._cb_outcomes)

    def execute(self, _):
        return self._cb(self._cb_class_instance)


def thread_main(node):
    try:
        rclpy.spin(node)
    except (KeyboardInterrupt, rclpy.executors.ExternalShutdownException):
        pass


def main():

    rclpy.init()
    node = SprayingTaskPlanExecutor()
    thread = threading.Thread(target=thread_main, args=(node,), daemon=True)
    thread.start()

    try:
        node.run()
        node.end()
    except KeyboardInterrupt:
        pass
    finally:
        node.terminate()

    thread.join()


if __name__ == '__main__':
    main()
