#!/usr/bin/python3

import os.path
import numpy as np
import threading

import rclpy
from rclpy import Future
from rclpy.action import ActionClient
from rclpy.time import Time
from rclpy.node import Node
from rclpy.duration import Duration
from geometry_msgs.msg import Point, Pose, PoseStamped
from nav_msgs.msg import Path
from std_msgs.msg import Header

from tf2_ros import TransformException
from tf2_ros.buffer import Buffer
from tf2_ros.transform_listener import TransformListener

# noinspection PyUnresolvedReferences
import tf2_geometry_msgs  # imports PoseStamped into tf2

from action_msgs.msg import GoalStatus
from nav2_msgs.action import FollowPath, NavigateToPose

from enum import Enum
from spraying_task_plan import SprayingTaskPlan, TaskPlanItem, TaskPlanItemResult


class NavigationActionResult(Enum):
    UNKNOWN = 0
    SUCCEEDED = 1
    CANCELED = 2
    FAILED = 3


def result2str(r):
    if r == NavigationActionResult.SUCCEEDED:
        return 'succeeded'
    elif r == NavigationActionResult.CANCELED:
        return 'canceled'
    elif r == NavigationActionResult.FAILED:
        return 'failed'
    else:
        return 'unknown'


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


class AsbTaskPlanNavigator(Node):
    def __init__(self):
        super().__init__('spraying_task_plan_maker')

        Chronometer.node = self

        default_task_plan_file_path = os.path.expanduser("~/asb_canopy_spraying_task_plan.yaml")
        self.declare_parameter('task_plan_file_path', default_task_plan_file_path)
        self.task_plan_file_path = os.path.expanduser(self.get_parameter('task_plan_file_path').get_parameter_value().string_value)
        self.get_logger().info(f"task_plan_file_path set to {self.task_plan_file_path}")

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

        default_robot_pose_time_tolerance = 0.2
        self.declare_parameter('robot_pose_time_tolerance', float(default_robot_pose_time_tolerance))
        self.robot_pose_time_tolerance = Duration(seconds=self.get_parameter('robot_pose_time_tolerance').get_parameter_value().double_value)

        default_target_loop_rate = 100  # Hz
        self.declare_parameter('target_loop_rate', float(default_target_loop_rate))
        self.target_loop_rate = self.get_parameter('target_loop_rate').get_parameter_value().double_value
        self.target_loop_duration = 1 / self.target_loop_rate

        default_min_loop_rate = 5  # Hz # TODO should be 50 Hz to have high frequency heartbeat, check if true
        self.declare_parameter('min_loop_rate', float(default_min_loop_rate))
        self.min_loop_rate = self.get_parameter('min_loop_rate').get_parameter_value().double_value
        self.max_loop_duration = 1 / self.min_loop_rate

        default_max_loop_rate = 150  # Hz
        self.declare_parameter('max_loop_rate', float(default_max_loop_rate))
        self.max_loop_rate = self.get_parameter('max_loop_rate').get_parameter_value().double_value
        self.min_loop_duration = 1 / self.max_loop_rate

        if not len(self.task_plan.get_item_ids()):
            self.get_logger().error(f"Empty task plan")
            return

        self.tf_buffer = Buffer()
        self.tf_listener = TransformListener(self.tf_buffer, self)

        self.robot_pose_timer_callback_chrono: Chronometer | None = None
        self.loop_things_chrono: Chronometer | None = None

        self.path_pub = self.create_publisher(Path, '/plan', 10)
        self.loop_rate = self.create_rate(self.target_loop_rate)

        # navigation action variables
        self.follow_path_client = ActionClient(self, FollowPath, 'follow_path')
        self.navigate_to_pose_client = ActionClient(self, NavigateToPose, 'navigate_to_pose')
        self.goal_handle: rclpy.action.client.ClientGoalHandle | None = None
        self.send_goal_future: Future | None = None
        self.result_future: Future | None = None
        self.cancel_navigation_action_future: Future | None = None
        self.navigate_to_pose_feedback: NavigateToPose.Feedback | None = None
        self.follow_path_feedback: FollowPath.Feedback | None = None
        self.navigation_result_status: NavigationActionResult | None = None

    def run(self):
        run_chrono = Chronometer()

        self.get_logger().info(f"waiting for robot pose...")
        robot_pose_chrono = Chronometer()
        self.get_robot_pose(timeout=10.0)
        self.get_logger().info(f"robot pose received (took {robot_pose_chrono.total():.4f} s), ready to start the task")

        item_index = 0

        while rclpy.ok():
            self.loop_rate.sleep()
            start_nav_chrono = Chronometer()

            item = self.task_plan.task_plan_items[item_index]
            item.result = TaskPlanItemResult()
            item.result.item_started = True

            self.do_loop_things()
            self.loop_rate.sleep()

            # TODO send spray regulation command, wait for start completion if necessary

            navigation_started = self.start_navigation(item)
            if navigation_started:
                item.result.navigation_started = True
                self.get_logger().info(f"start_nav_duration: {start_nav_chrono.total():.3f} s")

                self.get_logger().info(f"waiting for navigation to complete...")
                nav_chrono = Chronometer()
                while rclpy.ok() and not self.is_navigation_action_complete():
                    self.loop_rate.sleep()
                    self.do_loop_things()

                    # TODO update heartbeat
                    # TODO check loop rate
                    # TODO check navigation timeout
                    # TODO check navigation mode
                    # TODO check navigation feedback
                    # TODO check spray regulation feedback

                self.get_logger().info(f"navigation completed in {nav_chrono.total():.3f} s")
                self.loop_rate.sleep()
                self.do_loop_things()

                # TODO send stop spray regulation command

                item.result.navigation_result = self.get_navigation_action_result()
                if item.result.navigation_result == NavigationActionResult.SUCCEEDED:
                    self.get_logger().info(f"navigation result for item {item.item_id}: {result2str(item.result.navigation_result)}")
                else:
                    if self.dry_run:
                        self.get_logger().info(f"(DRY RUN) navigation result for item {item.item_id}: {result2str(item.result.navigation_result)}")
                    else:
                        self.get_logger().error(f"navigation result for item {item.item_id}: {result2str(item.result.navigation_result)}")
                        # TODO wait for operator mode switch to MANUAL, or activate SW E-STOP
            else:
                self.get_logger().error(f"navigation could not be started for item {item.item_id}")
                item.result.navigation_started = False
                start_nav_duration = start_nav_chrono.delta()
                self.get_logger().info(f"start_nav_duration: {start_nav_duration:.3f} s")
                # TODO send stop spray regulation command
                # TODO wait for operator mode switch to MANUAL, or activate SW E-STOP

            item_index += 1
            if item_index >= len(self.task_plan.task_plan_items):
                self.get_logger().info(f"Finished task plan in {run_chrono.delta():.1f} s")
                return

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
        self.cancel_navigation_action()

    def do_loop_things(self):
        # check we are running this function at an acceptable rate
        if self.loop_things_chrono is None:
            self.loop_things_chrono = Chronometer()
        else:
            loop_things_delta = self.loop_things_chrono.delta()
            if loop_things_delta > self.max_loop_duration:
                self.get_logger().error(f"LOW LOOP RATE loop_things_delta > self.max_loop_duration: {loop_things_delta:.3f} s, {1/loop_things_delta:.3f} Hz")
            if loop_things_delta < self.min_loop_duration:
                self.get_logger().error(f"HIGH LOOP RATE loop_things_delta < self.target_loop_duration: {loop_things_delta:.3f} s, {1/loop_things_delta:.3f} Hz")

    def start_navigation(self, item: TaskPlanItem) -> bool:

        if item.type == "approach":
            self.get_logger().info(f"STARTING approach navigation: {item.item_id}")

            if item.get_approach_pose().header.frame_id != self.task_plan.map_frame:
                self.get_logger().error(f"the approach pose's frame_id does not match the task plan map_frame")
                return False

            return self.execute_navigate_to_pose_action(pose=item.get_approach_pose(), timeout=0.1) if not self.dry_run else True

        elif item.type == "row":
            self.get_logger().info(f"STARTING row navigation: {item.item_id}")

            row_path = self.get_row_path(item)
            if row_path is None:
                return False

            self.path_pub.publish(row_path)
            return self.execute_follow_path_action(path=row_path, controller_id=self.task_plan.row_path_controller_id, timeout=0.1) if not self.dry_run else True

        else:
            self.get_logger().error(f"unknown task plan item type [{item.type}] for item {item.item_id}")
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

        robot_pose = self.get_robot_pose(timeout=0.1)
        if robot_pose is None:
            self.get_logger().error(f"Could not get robot pose for row item {item.item_id}")
            return None

        row_waypoints = item.get_row_waypoints()
        if len(row_waypoints) < 2:
            self.get_logger().error(f"Less than 2 row poses for row item {item.item_id}")
            return None

        # if not self.dry_run:
        #     closest_row_waypoint_index = ...  # TODO
        #     if closest_row_waypoint_index != 0:
        #         self.get_logger().error(f"robot is not at start of row for item {item.item_id}")
        #         return None

        row_poses = [robot_pose] + item.get_row_waypoints()

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

    """
     Send the NavigateToPose action request.
    """
    def execute_navigate_to_pose_action(self, pose: PoseStamped, timeout: float) -> bool:
        self.get_logger().info("Waiting for NavigateToPose action server")
        timeout_chrono = Chronometer()
        while rclpy.ok() and not self.navigate_to_pose_client.wait_for_server(timeout_sec=0.05):
            self.get_logger().info("NavigateToPose action server not available, waiting...")
            if timeout_chrono.total() > timeout:
                return False

        goal_msg = NavigateToPose.Goal()
        goal_msg.pose = pose

        self.get_logger().info(f"execute_navigate_to_pose_action: sending goal")
        response_future: Future = self.navigate_to_pose_client.send_goal_async(goal_msg, self.navigate_to_pose_feedback_callback)
        response_future.add_done_callback(self.navigate_to_pose_response_callback)
        return True

    """
     Receive the NavigateToPose action response.
    """
    def navigate_to_pose_response_callback(self, future: Future):
        self.goal_handle = future.result()
        if self.goal_handle.accepted:
            self.get_logger().info(f"navigate_to_pose_response_callback: goal accepted")
        else:
            self.get_logger().error(f"navigate_to_pose_response_callback: goal rejected")

        self.result_future: Future = self.goal_handle.get_result_async()
        self.result_future.add_done_callback(self.navigate_to_pose_result_callback)

    """
     Receive the NavigateToPose action feedback.
    """
    def navigate_to_pose_feedback_callback(self, msg):
        self.navigate_to_pose_feedback = msg.feedback
        self.get_logger().info(
            f"navigate_to_pose_feedback:\n"
            f"    distance_remaining:       {msg.feedback.distance_remaining:.1f} m\n"
            f"    estimated_time_remaining: {Duration.from_msg(msg.feedback.estimated_time_remaining).nanoseconds/1e9:.1f} s\n"
            f"    navigation_time:          {Duration.from_msg(msg.feedback.navigation_time).nanoseconds/1e9:.1f} s\n"
            f"    number_of_recoveries:     {msg.feedback.number_of_recoveries}\n",
            throttle_duration_sec=5.0
        )

    """
     Receive the NavigateToPose action result.
    """
    def navigate_to_pose_result_callback(self, _: Future):
        self.get_logger().info(f"navigate_to_pose_result_callback, action completed")

    """
     Send the FollowPath action request.
    """
    def execute_follow_path_action(self, path: Path, timeout: float, controller_id, goal_checker_id='') -> bool:
        self.get_logger().info("Waiting for FollowPath action server")
        timeout_chrono = Chronometer()
        while rclpy.ok() and not self.follow_path_client.wait_for_server(timeout_sec=0.01):
            self.get_logger().info("FollowPath action server not available, waiting...")
            if timeout_chrono.total() > timeout:
                return False

        goal_msg = FollowPath.Goal()
        goal_msg.path = path
        goal_msg.controller_id = controller_id
        goal_msg.goal_checker_id = goal_checker_id

        self.get_logger().info(f"execute_follow_path_action: sending goal")
        response_future: Future = self.follow_path_client.send_goal_async(goal_msg, self.follow_path_feedback_callback)
        response_future.add_done_callback(self.follow_path_response_callback)
        return True

    """
     Receive the FollowPath action response.
    """
    def follow_path_response_callback(self, future: Future):
        self.goal_handle: rclpy.action.client.ClientGoalHandle = future.result()
        if self.goal_handle.accepted:
            self.get_logger().info(f"follow_path_response_callback: goal accepted")
        else:
            self.get_logger().error(f"follow_path_response_callback: goal rejected")

        self.result_future: Future = self.goal_handle.get_result_async()
        self.result_future.add_done_callback(self.follow_path_result_callback)

    """
     Receive the FollowPath action feedback.
    """
    def follow_path_feedback_callback(self, msg):
        self.follow_path_feedback = msg.feedback
        self.get_logger().info(
            f"navigate_to_pose_feedback:\n"
            f"    distance_to_goal:         {msg.feedback.distance_to_goal:.1f} m\n"
            f"    speed:                    {msg.feedback.speed:.3f} m/s\n",
            throttle_duration_sec=5.0
        )

    """
     Receive the FollowPath action result.
    """
    def follow_path_result_callback(self, _: Future):
        self.get_logger().info(f"follow_path_result_callback, action completed")

    def cancel_navigation_action(self):
        if self.goal_handle is not None:
            self.get_logger().info('canceling current navigation action')
            self.cancel_navigation_action_future: Future = self.goal_handle.cancel_goal_async()
            self.cancel_navigation_action_future.add_done_callback(self.cancel_navigation_action_response_callback)
        else:
            self.get_logger().info('cancel_navigation_action: no action to cancel')

    def cancel_navigation_action_response_callback(self, _):
        self.get_logger().info('current navigation action cancelled')

    def is_navigation_action_complete(self):
        if self.result_future is None:
            self.get_logger().error(f"checking action result future before sending a goal or receiving a goal response")
            return False

        if self.result_future.result() is not None:
            self.navigation_result_status = self.result_future.result().status
            if self.navigation_result_status != GoalStatus.STATUS_SUCCEEDED:
                self.get_logger().info(f'Navigation action failed with status code: {self.navigation_result_status}')
                return True
            else:
                self.get_logger().info('Task succeeded!')
                return True
        else:
            # Timed out, still processing, not complete yet
            return False  # TODO timed out???

    def get_navigation_action_result(self):
        if self.navigation_result_status == GoalStatus.STATUS_SUCCEEDED:
            return NavigationActionResult.SUCCEEDED
        elif self.navigation_result_status == GoalStatus.STATUS_ABORTED:
            return NavigationActionResult.FAILED
        elif self.navigation_result_status == GoalStatus.STATUS_CANCELED:
            return NavigationActionResult.CANCELED
        else:
            return NavigationActionResult.UNKNOWN


def thread_main(node):

    try:
        rclpy.spin(node)
    except (KeyboardInterrupt, rclpy.executors.ExternalShutdownException):
        pass


def main():

    rclpy.init()
    task_plan_navigator_node = AsbTaskPlanNavigator()
    thread = threading.Thread(target=thread_main, args=(task_plan_navigator_node,), daemon=True)
    thread.start()

    try:
        task_plan_navigator_node.run()
        task_plan_navigator_node.end()
    except KeyboardInterrupt:
        pass
    finally:
        task_plan_navigator_node.terminate()

    thread.join()

    # executor_thread.join()


if __name__ == '__main__':
    main()
