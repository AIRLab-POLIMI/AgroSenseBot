#!/usr/bin/python3

import os.path

import numpy as np
import threading

import rclpy
from rclpy import Future
from rclpy.qos import QoSProfile, ReliabilityPolicy, DurabilityPolicy, HistoryPolicy
from rclpy.client import Client
from rclpy.action import ActionClient
from rclpy.time import Time
from rclpy.node import Node
from rclpy.duration import Duration
from lifecycle_msgs.srv import GetState, GetState_Request
from asb_msgs.msg import Heartbeat, ControlSystemState, SprayRegulatorStatus
from asb_msgs.srv import StartRowSpraying, StartRowSpraying_Request, StopRowSpraying, StopRowSpraying_Request
from geometry_msgs.msg import Point, Pose, PoseStamped
from nav_msgs.msg import Path
from std_msgs.msg import Header, String
from action_msgs.msg import GoalStatus, GoalInfo
from nav2_msgs.srv import ClearEntireCostmap, ClearEntireCostmap_Request
from nav2_msgs.action import FollowPath, NavigateToPose

from tf2_ros import TransformException
from tf2_ros.buffer import Buffer
from tf2_ros.transform_listener import TransformListener

# noinspection PyUnresolvedReferences
import tf2_geometry_msgs  # imports PoseStamped into tf2

from enum import Enum
from typing_extensions import Self

from spraying_task_plan import SprayingTaskPlan, TaskPlanItem, TaskPlanItemResult, TaskPlanItemType, TaskPlanRow


class NavigationActionStatus(Enum):
    NOT_STARTED = 0
    REQUESTED = 1
    FAILED_TO_START = 2
    IN_PROGRESS = 3
    SUCCEEDED = 4
    FAILED = 5


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

        default_log_dir_path = os.path.expanduser("~/asb_logs/asb_canopy_spraying_task_plan.yaml")
        self.declare_parameter('log_dir_path', default_log_dir_path)
        self.task_log_dir_path = os.path.expanduser(self.get_parameter('log_dir_path').get_parameter_value().string_value)
        self.get_logger().info(f"log_dir_path set to {self.task_log_dir_path}")

        self.task_plan: SprayingTaskPlan = SprayingTaskPlan.load(self.task_plan_file_path)
        self.get_logger().info(f"loaded task plan with path ids: {self.task_plan.get_item_ids()}")

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

        default_base_frame = "base_link"
        self.declare_parameter('base_frame', default_base_frame)
        self.base_frame = self.get_parameter('base_frame').get_parameter_value().string_value

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

        default_max_loop_rate = 55  # Hz
        self.declare_parameter('max_loop_rate', float(default_max_loop_rate))
        self.max_loop_rate = self.get_parameter('max_loop_rate').get_parameter_value().double_value
        self.min_loop_duration = 1 / self.max_loop_rate

        if not len(self.task_plan.get_item_ids()):
            self.get_logger().error(f"empty task plan")
            return

        self.tf_buffer = Buffer()
        self.tf_listener = TransformListener(self.tf_buffer, self)

        # variables for task execution
        self.loop_operations_chrono: Chronometer | None = None
        self.navigation_action_status: NavigationActionStatus = NavigationActionStatus.NOT_STARTED
        self.left_spraying_state: SprayState = SprayState.NOT_SPRAYING
        self.left_row: TaskPlanRow | None = None
        self.right_spraying_state: SprayState = SprayState.NOT_SPRAYING
        self.right_row: TaskPlanRow | None = None
        self.heartbeat_alive_bit: bool = False
        self.last_platform_status_msg: ControlSystemState | None = None
        self.stop_platform: bool = True

        # publishers, subscribers, timers and loop rate
        qos_reliable_transient_local_depth_10 = QoSProfile(
            reliability=ReliabilityPolicy.RELIABLE,
            durability=DurabilityPolicy.TRANSIENT_LOCAL,
            history=HistoryPolicy.KEEP_LAST,
            depth=10
        )
        self.path_pub = self.create_publisher(Path, '/plan', 10)
        self.heartbeat_pub = self.create_publisher(Heartbeat, '/asb_control_system_status_controller/heartbeat', rclpy.qos.qos_profile_sensor_data)
        self.current_item_pub = self.create_publisher(String, '~/current_item', 10)
        self.platform_status_sub = self.create_subscription(ControlSystemState, '/asb_control_system_status_controller/control_system_state', self.platform_status_callback, 10)
        self.start_row_spraying_service = self.create_client(StartRowSpraying, 'start_row_spraying')
        self.stop_row_spraying_service = self.create_client(StopRowSpraying, 'stop_row_spraying')
        self.spray_regulator_status_sub = self.create_subscription(SprayRegulatorStatus, 'spray_regulator_status', self.spray_regulator_status_callback, qos_reliable_transient_local_depth_10)
        self.loop_rate = self.create_rate(self.target_loop_rate)

        # navigation action variables
        self.clear_local_costmap_service = self.create_client(ClearEntireCostmap, '/local_costmap/clear_entirely_local_costmap')
        self.clear_global_costmap_service = self.create_client(ClearEntireCostmap, '/global_costmap/clear_entirely_global_costmap')
        self.follow_path_client = ActionClient(self, FollowPath, 'follow_path')
        self.navigate_to_pose_client = ActionClient(self, NavigateToPose, 'navigate_to_pose')
        self.navigation_goal_handle: rclpy.action.client.ClientGoalHandle | None = None
        self.navigate_to_pose_feedback: NavigateToPose.Feedback | None = None
        self.follow_path_feedback: FollowPath.Feedback | None = None

    def run(self):
        run_chrono = Chronometer()

        if not self.dry_run:
            # wait for localization
            self.get_logger().info(f"waiting for robot pose...")
            robot_pose_chrono = Chronometer()
            robot_pose = self.get_robot_pose(timeout=10.0)
            if robot_pose is not None:
                self.get_logger().info(f"robot pose received (took {robot_pose_chrono.total():.4f} s)")
            else:
                self.get_logger().fatal(f"robot pose not received (took {robot_pose_chrono.total():.4f} s), aborting task")
                return

            # wait for navigation stack
            self.get_logger().info(f"waiting for navigation stack...")
            nav_stack_chrono = Chronometer()
            nav_stack_ready = self.wait_navigation_stack_is_ready(timeout=10.0)
            if nav_stack_ready:
                self.get_logger().info(f"navigation stack is ready (took {nav_stack_chrono.total():.4f} s)")
            else:
                self.get_logger().fatal(f"navigation stack timeout (took {nav_stack_chrono.total():.4f} s), aborting task")
                return

            # wait for spray regulator
            self.get_logger().info(f"waiting for spray regulator...")
            spray_regulator_chrono = Chronometer()
            spray_regulator_ready = self.wait_spray_regulator_is_ready(timeout=10.0)
            if spray_regulator_ready:
                self.get_logger().info(f"spray regulator is ready (took {spray_regulator_chrono.total():.4f} s), ready to start the task\n\n")
            else:
                self.get_logger().fatal(f"spray regulator timeout (took {spray_regulator_chrono.total():.4f} s), aborting task")
                return

        self.prepare_task_log()

        self.stop_platform_and_wait_control_mode_manual_to_auto()

        # clear costmaps
        self.prepare_navigation()

        # [!]
        # Beyond this point, no functions should block or take too long to execute.
        # If self.do_loop_operations_and_sleep is not called with the correct frequency, for example due to an exception
        # interrupting this function, the platform will stop receiving the heartbeat and go to STOP or MANUAL mode.
        # self.stop_platform_and_wait_control_mode_manual_to_auto will stop sending the heartbeat on purpose to send the
        # platform to control mode STOP or MANUAL

        item_index = 0

        while rclpy.ok():
            self.do_loop_operations_and_sleep()

            item = self.task_plan.items[item_index]
            item.set_result(TaskPlanItemResult())
            item.get_result().item_started = True

            self.do_loop_operations_and_sleep(current_item=item)

            if item.get_type() == TaskPlanItemType.ROW:
                self.start_spray_regulator(item)

                # wait for spray regulator to start or to fail to start
                start_spray_regulator_chrono = Chronometer()
                while rclpy.ok() and not self.is_spray_regulator_started() and not self.is_spray_regulator_failed() and start_spray_regulator_chrono.total() < self.start_spray_regulator_timeout:
                    self.do_loop_operations_and_sleep(current_item=item)

                if start_spray_regulator_chrono.total() > self.start_spray_regulator_timeout:
                    self.get_logger().error(f"spray regulator failed to start before timeout [{self.start_spray_regulator_timeout} s] for item {item.get_item_id()}")
                    self.stop_spray_regulator()
                    self.stop_platform_and_wait_control_mode_manual_to_auto()
                    continue  # (back to start of main loop)
                if self.is_spray_regulator_failed():
                    self.get_logger().error(f"spray regulator failed")
                    self.stop_spray_regulator()
                    self.stop_platform_and_wait_control_mode_manual_to_auto()
                    continue  # (back to start of main loop)
                self.get_logger().info(f"spray regulator started")

            start_navigation_action_chrono = Chronometer()
            navigation_action_requested = self.start_navigation(item)

            if not navigation_action_requested:
                self.get_logger().error(f"navigation could not be started for item {item.get_item_id()}")
                item.get_result().navigation_started = False
                if item.get_type() == TaskPlanItemType.ROW:
                    self.stop_spray_regulator()
                self.stop_platform_and_wait_control_mode_manual_to_auto()
                continue  # (back to start of main loop)

            # wait for navigation action to start or to fail to start, assume it could also immediately become succeeded or failed
            while rclpy.ok() and self.navigation_action_status in [NavigationActionStatus.NOT_STARTED, NavigationActionStatus.REQUESTED] and start_navigation_action_chrono.total() < self.start_navigation_action_timeout:
                self.do_loop_operations_and_sleep(current_item=item)

            if start_navigation_action_chrono.total() > self.start_navigation_action_timeout:
                self.get_logger().error(f"navigation could not be started before timeout [{self.start_navigation_action_timeout} s] for item {item.get_item_id()}")
                item.get_result().navigation_started = False
                if item.get_type() == TaskPlanItemType.ROW:
                    self.stop_spray_regulator()
                self.stop_platform_and_wait_control_mode_manual_to_auto()
                continue  # (back to start of main loop)

            if self.navigation_action_status == NavigationActionStatus.FAILED_TO_START:
                self.get_logger().error(f"navigation could not be started for item {item.get_item_id()}")
                item.get_result().navigation_started = False
                if item.get_type() == TaskPlanItemType.ROW:
                    self.stop_spray_regulator()
                self.stop_platform_and_wait_control_mode_manual_to_auto()
                continue  # (back to start of main loop)

            item.get_result().navigation_started = True
            self.get_logger().info(f"waiting for navigation to complete")
            nav_chrono = Chronometer()
            while rclpy.ok():
                self.do_loop_operations_and_sleep(current_item=item)

                if self.dry_run:
                    item.get_result().navigation_result = self.navigation_action_status.name
                    self.get_logger().info(f"navigation completed in {nav_chrono.total():.3f} s for item {item.get_item_id()} (DRY RUN)")
                    self.do_loop_operations_and_sleep(current_item=item)
                    if item.get_type() == TaskPlanItemType.ROW:
                        self.stop_spray_regulator()
                    item_index += 1
                    if item_index < len(self.task_plan.items):
                        break  # (back to start of main loop)
                    else:
                        self.get_logger().info(f"finished task plan in {run_chrono.delta():.1f} s")
                        return

                if self.navigation_action_status == NavigationActionStatus.SUCCEEDED:
                    item.get_result().navigation_result = self.navigation_action_status.name
                    self.do_loop_operations_and_sleep(current_item=item)
                    self.get_logger().info(f"navigation succeeded in {nav_chrono.total():.3f} s for item {item.get_item_id()}")
                    if item.get_type() == TaskPlanItemType.ROW:
                        self.stop_spray_regulator()
                    item_index += 1
                    if item_index < len(self.task_plan.items):
                        break  # (back to start of main loop)
                    else:
                        self.get_logger().info(f"finished task plan in {run_chrono.delta():.1f} s")
                        return

                if self.navigation_action_status == NavigationActionStatus.FAILED:
                    item.get_result().navigation_result = self.navigation_action_status.name
                    self.get_logger().error(f"navigation failed after {nav_chrono.total():.3f} s for item {item.get_item_id()}")
                    self.do_loop_operations_and_sleep(current_item=item)
                    if item.get_type() == TaskPlanItemType.ROW:
                        self.stop_spray_regulator()
                    self.stop_platform_and_wait_control_mode_manual_to_auto()
                    break  # (back to start of main loop)

                if not self.get_control_mode() == ControlMode.AUTO:
                    self.cancel_navigation_action()
                    if item.get_type() == TaskPlanItemType.ROW:
                        self.stop_spray_regulator()
                    self.stop_platform_and_wait_control_mode_manual_to_auto()
                    break  # (back to start of main loop)

                if self.is_spray_regulator_failed():
                    self.cancel_navigation_action()
                    self.stop_spray_regulator()
                    self.stop_platform_and_wait_control_mode_manual_to_auto()
                    break  # (back to start of main loop)

                # TODO check item timeout
                # TODO check navigation feedback

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
        self.cancel_navigation_action()
        self.get_logger().info(f"writing task results")
        self.log_task_results()

    def prepare_task_log(self) -> None:
        if not os.path.isdir(self.task_log_dir_path):
            os.makedirs(self.task_log_dir_path)

    def log_task_results(self) -> None:
        chrono = Chronometer()
        self.task_plan.write(os.path.expanduser(os.path.join(self.task_log_dir_path, "spraying_task_plan_result.yaml")))
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

    def platform_status_callback(self, platform_status: ControlSystemState):
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

        while rclpy.ok() and self.get_control_mode() != ControlMode.MANUAL:
            self.do_loop_operations_and_sleep()
            self.get_logger().info(f"WAITING control mode switch to MANUAL", throttle_duration_sec=1.0)

        self.stop_platform = False

        while rclpy.ok() and self.get_control_mode() != ControlMode.AUTO:
            self.do_loop_operations_and_sleep()
            self.get_logger().info(f"WAITING control mode switch to AUTO", throttle_duration_sec=1.0)

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
                self.right_spraying_state = SprayState.REQUESTED

    def start_left_row_spraying_response_callback(self, response_future) -> None:
        if response_future.result().result:
            self.left_spraying_state = SprayState.STARTING
            self.get_logger().info(f"left_spraying_state: STARTING")
        else:
            self.left_spraying_state = SprayState.FAILED
            self.get_logger().error(f"left_spraying_state: FAILED")

    def start_right_row_spraying_response_callback(self, response_future) -> None:
        if response_future.result().result:
            self.right_spraying_state = SprayState.STARTING
            self.get_logger().info(f"right_spraying_state: STARTING")
        else:
            self.right_spraying_state = SprayState.FAILED
            self.get_logger().error(f"right_spraying_state: FAILED")

    def spray_regulator_status_callback(self, status_msg: SprayRegulatorStatus) -> None:
        if status_msg.status == SprayRegulatorStatus.STATUS_FAILED:
            if self.left_spraying_state != SprayState.FAILED and status_msg.row_id == self.left_row.get_row_id():
                self.left_spraying_state = SprayState.FAILED
                self.get_logger().error(f"left_spraying_state: FAILED")
            if self.right_spraying_state != SprayState.FAILED and status_msg.row_id == self.right_row.get_row_id():
                self.right_spraying_state = SprayState.FAILED
                self.get_logger().error(f"right_spraying_state: FAILED")

        elif status_msg.status == SprayRegulatorStatus.STATUS_OK:
            if self.left_spraying_state == SprayState.STARTING and status_msg.row_id == self.left_row.get_row_id():
                self.left_spraying_state = SprayState.STARTED
                self.get_logger().info(f"left_spraying_state: STARTED")
            if self.right_spraying_state == SprayState.STARTING and status_msg.row_id == self.right_row.get_row_id():
                self.right_spraying_state = SprayState.STARTED
                self.get_logger().info(f"right_spraying_state: STARTED")

        elif status_msg.status == SprayRegulatorStatus.STATUS_STOPPED:
            if self.left_spraying_state != SprayState.NOT_SPRAYING and status_msg.row_id == self.left_row.get_row_id():
                self.left_spraying_state = SprayState.NOT_SPRAYING
                self.left_row = None
                self.get_logger().info(f"left_spraying_state: NOT_SPRAYING")
            if self.right_spraying_state != SprayState.NOT_SPRAYING and status_msg.row_id == self.right_row.get_row_id():
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

    def prepare_navigation(self):
        self.clear_local_costmap_service.call_async(ClearEntireCostmap_Request())
        self.clear_global_costmap_service.call_async(ClearEntireCostmap_Request())

    def start_navigation(self, item: TaskPlanItem) -> bool:

        # NOTE: the navigation action functions set self.navigation_action_status to REQUESTED, and eventually to
        # - NavigationActionStatus.FAILED_TO_START
        # - NavigationActionStatus.IN_PROGRESS
        # - NavigationActionStatus.SUCCEEDED
        # - NavigationActionStatus.FAILED
        self.navigation_action_status = NavigationActionStatus.NOT_STARTED

        if item.get_type() == TaskPlanItemType.APPROACH:
            self.get_logger().info(f"STARTING approach navigation: {item.get_item_id()}")

            if item.get_approach_pose().header.frame_id != self.task_plan.map_frame:
                self.get_logger().error(f"the approach pose's frame_id does not match the task plan map_frame")
                return False

            if self.dry_run:
                self.navigation_action_status = NavigationActionStatus.SUCCEEDED
            else:
                self.execute_navigate_to_pose_action(pose=item.get_approach_pose())

            return True

        elif item.get_type() == TaskPlanItemType.ROW:
            self.get_logger().info(f"STARTING row navigation: {item.get_item_id()}")

            row_path = self.get_row_path(item)
            if row_path is None:
                return False

            self.path_pub.publish(row_path)

            if self.dry_run:
                self.navigation_action_status = NavigationActionStatus.SUCCEEDED
            else:
                self.execute_follow_path_action(path=row_path, controller_id=self.task_plan.row_path_controller_id, goal_checker_id=self.task_plan.row_path_goal_checker_id, progress_checker_id=self.task_plan.row_path_progress_checker_id)

            return True

        else:
            self.get_logger().error(f"unknown task plan item type [{item.get_type().name}] for item {item.get_item_id()}")
            return False

    def get_robot_pose(self, timeout) -> PoseStamped | None:
        robot_pose = PoseStamped()
        robot_pose.header.frame_id = self.base_frame
        robot_pose.pose.orientation.w = 1

        transform_chrono = Chronometer()
        last_ex: TransformException | None = None
        while rclpy.ok() and transform_chrono.total() < timeout:
            try:
                robot_pose_map_frame = self.tf_buffer.transform(robot_pose, self.task_plan.map_frame, timeout=Duration(seconds=0.01))
                transform_age = self.get_clock().now() - Time.from_msg(robot_pose_map_frame.header.stamp)
                if transform_age < self.robot_pose_time_tolerance:
                    return robot_pose_map_frame
                else:
                    self.get_logger().error(f"robot pose age too high: {transform_age.nanoseconds/1e9:.6f}")
            except TransformException as ex:
                last_ex = ex

            self.loop_rate.sleep()

        if last_ex is not None:
            self.get_logger().error(f"could not transform {self.base_frame} to {self.task_plan.map_frame}: {last_ex}")
        return None

    def get_row_path(self, item: TaskPlanItem) -> Path | None:

        row_waypoints = item.get_row_waypoints()
        if len(row_waypoints) < 2:
            self.get_logger().error(f"less than 2 row poses for row item {item.get_item_id()}")
            return None

        if self.dry_run:
            prev_item = self.task_plan.get_preceding_item(item)
            if prev_item.get_type() == TaskPlanItemType.APPROACH:
                start_pose = prev_item.get_approach_pose()
            elif prev_item.get_type() == TaskPlanItemType.ROW:
                start_pose = prev_item.get_row_waypoints()[-1]
            else:
                self.get_logger().error(f"previous item [{prev_item.get_item_id()}] has unknown type: {prev_item.get_type().name}")
                return None
        else:
            start_pose = self.get_robot_pose(timeout=0.1)
            if start_pose is None:
                self.get_logger().error(f"could not get robot pose for row item {item.get_item_id()}")
                return None

        # closest_row_waypoint_index = ...  # TODO
        # if closest_row_waypoint_index != 0:
        #     self.get_logger().error(f"robot is not at start of row for item {item.get_item_id()}")
        #     return None

        if start_pose is not None:
            row_poses = [start_pose] + item.get_row_waypoints()
        else:
            row_poses = item.get_row_waypoints()

        path_header = Header(stamp=self.get_clock().now().to_msg(), frame_id=self.task_plan.map_frame)
        path = Path(header=path_header)

        for row_pose in row_poses:
            if row_pose.header.frame_id != path_header.frame_id:
                self.get_logger().error(f"row pose frame id [{row_pose.header.frame_id}] does not match the task plan's map frame id [{path_header.frame_id}]")
                return None
            row_pose.header = path_header

        for i, (p1, p2) in enumerate(zip(row_poses[0: -1], row_poses[1:])):
            p1_array = np.array([p1.pose.position.x, p1.pose.position.y, p1.pose.position.z])
            p2_array = np.array([p2.pose.position.x, p2.pose.position.y, p2.pose.position.z])
            int_points = list(np.linspace(
                p1_array,
                p2_array,
                num=int(np.ceil(np.linalg.norm(p2_array - p1_array) / self.task_plan.row_path_pose_distance)),
                endpoint=i == len(row_poses) - 2
            ))
            interpolated_poses = list(map(
                lambda v: PoseStamped(
                    header=path_header,
                    pose=Pose(
                        position=Point(x=v[0], y=v[1], z=v[2]),
                        orientation=p2.pose.orientation
                    )
                ),
                int_points
            ))
            path.poses += interpolated_poses

        return path

    def wait_spray_regulator_is_ready(self, timeout: float) -> bool:
        return self.wait_for_service(self.start_row_spraying_service, timeout=timeout)

    def wait_navigation_stack_is_ready(self, timeout: float) -> bool:
        if not self.wait_node_is_active("bt_navigator", timeout=timeout):
            return False
        if not self.wait_node_is_active("controller_server", timeout=timeout):
            return False
        if not self.wait_for_service(self.clear_local_costmap_service, timeout=timeout):
            return False
        if not self.wait_for_service(self.clear_global_costmap_service, timeout=timeout):
            return False
        if not self.wait_action_server(self.navigate_to_pose_client, "navigate_to_pose", timeout=timeout):
            return False
        if not self.wait_action_server(self.follow_path_client, "follow_path", timeout=timeout):
            return False
        return True

    def wait_for_service(self, service: Client, timeout: float) -> bool:
        self.get_logger().info(f"waiting for {service.srv_name} service")
        timeout_chrono = Chronometer()
        while rclpy.ok() and not service.wait_for_service(timeout_sec=0.05):
            self.get_logger().info(f"{service.srv_name} service not available, waiting", throttle_duration_sec=1.0)
            if timeout_chrono.total() > timeout:
                return False
        return True

    def wait_node_is_active(self, node_name, timeout: float) -> bool:
        self.get_logger().info(f"waiting for {node_name}")
        state_client = self.create_client(GetState, f"{node_name}/get_state")
        timeout_chrono = Chronometer()
        while rclpy.ok() and not state_client.wait_for_service(timeout_sec=0.05):
            self.get_logger().info(f'still waiting {node_name} lifecycle service', throttle_duration_sec=1.0)
            if timeout_chrono.total() > timeout:
                self.get_logger().error(f'{node_name} lifecycle state service was not available before timeout [{timeout} s]')
                return False

        prev_state = None
        while rclpy.ok():
            response_future = state_client.call_async(GetState_Request())
            while rclpy.ok() and response_future.result() is None:
                if timeout_chrono.total() > timeout:
                    self.get_logger().error(f"{node_name} lifecycle state service did not respond or did not transition to active before timeout [{timeout} s]")
                    return False
                else:
                    self.loop_rate.sleep()

            state = response_future.result().current_state.label
            if state != prev_state:
                prev_state = state
                self.get_logger().info(f'{node_name} lifecycle state: {state}')

            if state == "active":
                return True
            else:
                self.loop_rate.sleep()
        return False

    def wait_action_server(self, action_client: ActionClient, action_client_name: str, timeout: float) -> bool:
        self.get_logger().info(f"waiting for {action_client_name} action server")
        timeout_chrono = Chronometer()
        while rclpy.ok() and not action_client.wait_for_server(timeout_sec=0.05):
            self.get_logger().info(f"{action_client_name} action server not available, waiting", throttle_duration_sec=1.0)
            if timeout_chrono.total() > timeout:
                return False
        return True

    """
     Send the NavigateToPose action request.
    """
    def execute_navigate_to_pose_action(self, pose: PoseStamped) -> None:
        if self.navigation_action_status not in [NavigationActionStatus.NOT_STARTED, NavigationActionStatus.SUCCEEDED]:
            self.get_logger().error(f"trying to start a navigation action while another action is executing")
            return

        goal_msg = NavigateToPose.Goal()
        goal_msg.pose = pose

        self.get_logger().debug(f"execute_navigate_to_pose_action: sending goal")
        response_future: Future = self.navigate_to_pose_client.send_goal_async(goal_msg, self.navigate_to_pose_feedback_callback)
        response_future.add_done_callback(self.navigate_to_pose_response_callback)
        self.navigation_action_status = NavigationActionStatus.REQUESTED

    """
     Receive the NavigateToPose action response.
    """
    def navigate_to_pose_response_callback(self, future: Future) -> None:
        if self.navigation_action_status != NavigationActionStatus.REQUESTED:
            self.get_logger().error(
                f"received a navigation action response but navigation_action_status is "
                f"{self.navigation_action_status.name}, it should be REQUESTED")
            return

        self.navigation_goal_handle = future.result()
        if self.navigation_goal_handle.accepted:
            self.get_logger().debug(f"navigate_to_pose action response: accepted")
            result_future: Future = self.navigation_goal_handle.get_result_async()
            result_future.add_done_callback(self.navigate_to_pose_result_callback)
            self.navigation_action_status = NavigationActionStatus.IN_PROGRESS

        else:
            self.get_logger().warn(f"navigate_to_pose action response: rejected")
            self.navigation_action_status = NavigationActionStatus.FAILED_TO_START

    """
     Receive the NavigateToPose action feedback.
    """
    def navigate_to_pose_feedback_callback(self, msg) -> None:
        self.navigate_to_pose_feedback = msg.feedback
        self.get_logger().info(
            f"navigate_to_pose_feedback:\n"
            f"    distance_remaining:       {msg.feedback.distance_remaining:.1f} m\n"
            f"    estimated_time_remaining: {Duration.from_msg(msg.feedback.estimated_time_remaining).nanoseconds/1e9:.1f} s\n"
            f"    navigation_time:          {Duration.from_msg(msg.feedback.navigation_time).nanoseconds/1e9:.1f} s\n",
            throttle_duration_sec=5.0
        )

    """
     Receive the NavigateToPose action result.
    """
    def navigate_to_pose_result_callback(self, goal_result_future) -> None:
        navigation_result_status = goal_result_future.result().status
        if navigation_result_status == GoalStatus.STATUS_SUCCEEDED:
            self.get_logger().debug('navigate_to_pose action succeeded')
            self.navigation_action_status = NavigationActionStatus.SUCCEEDED
        else:
            self.get_logger().debug(f'navigate_to_pose action failed with status code: {navigation_result_status}')
            self.navigation_action_status = NavigationActionStatus.FAILED

    """
     Send the FollowPath action request.
    """
    def execute_follow_path_action(self, path: Path, controller_id, goal_checker_id='', progress_checker_id='') -> None:
        if self.navigation_action_status not in [NavigationActionStatus.NOT_STARTED, NavigationActionStatus.SUCCEEDED]:
            self.get_logger().error(f"trying to start a navigation action while another action is executing")
            return

        goal_msg = FollowPath.Goal()
        goal_msg.path = path
        goal_msg.controller_id = controller_id
        goal_msg.goal_checker_id = goal_checker_id
        goal_msg.progress_checker_id = progress_checker_id

        self.get_logger().debug(f"execute_follow_path_action: sending goal")
        response_future: Future = self.follow_path_client.send_goal_async(goal_msg, self.follow_path_feedback_callback)
        response_future.add_done_callback(self.follow_path_response_callback)
        self.navigation_action_status = NavigationActionStatus.REQUESTED

    """
     Receive the FollowPath action response.
    """
    def follow_path_response_callback(self, future: Future) -> None:
        if self.navigation_action_status != NavigationActionStatus.REQUESTED:
            self.get_logger().error(
                f"received a navigation action response but navigation_action_status is "
                f"{self.navigation_action_status.name}, it should be REQUESTED")
            return

        self.navigation_goal_handle = future.result()
        if self.navigation_goal_handle.accepted:
            self.get_logger().debug(f"follow_path action response: accepted")
            result_future: Future = self.navigation_goal_handle.get_result_async()
            result_future.add_done_callback(self.follow_path_result_callback)
            self.navigation_action_status = NavigationActionStatus.IN_PROGRESS

        else:
            self.get_logger().warn(f"follow_path action response: rejected")
            self.navigation_action_status = NavigationActionStatus.FAILED_TO_START

    """
     Receive the FollowPath action feedback.
    """
    def follow_path_feedback_callback(self, msg) -> None:
        self.follow_path_feedback = msg.feedback
        self.get_logger().info(
            f"follow_path_feedback:\n"
            f"    distance_to_goal:         {msg.feedback.distance_to_goal:.1f} m\n"
            f"    speed:                    {msg.feedback.speed:.3f} m/s\n",
            throttle_duration_sec=5.0
        )

    """
     Receive the FollowPath action result.
    """
    def follow_path_result_callback(self, future: Future) -> None:
        navigation_result_status = future.result().status
        if navigation_result_status == GoalStatus.STATUS_SUCCEEDED:
            self.get_logger().debug('follow_path action succeeded')
            self.navigation_action_status = NavigationActionStatus.SUCCEEDED
        else:
            self.get_logger().debug(f'follow_path action failed with status code: {navigation_result_status}')
            self.navigation_action_status = NavigationActionStatus.FAILED

    """
     Request to cancel the current navigation action, if there is one in progress.
    """
    def cancel_navigation_action(self):
        if self.navigation_goal_handle is not None:
            self.get_logger().info('canceling current navigation action')
            cancel_navigation_action_future: Future = self.navigation_goal_handle.cancel_goal_async()
            cancel_navigation_action_future.add_done_callback(self.cancel_navigation_action_response_callback)
        else:
            self.get_logger().info('no navigation actions in progress')

    """
     Receive the action cancel result.
    """
    def cancel_navigation_action_response_callback(self, _: GoalInfo) -> None:
        self.get_logger().info(f"current navigation action was cancelled")
        self.navigation_goal_handle = None
        self.navigation_action_status = NavigationActionStatus.NOT_STARTED


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
