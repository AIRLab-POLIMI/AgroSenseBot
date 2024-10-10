#!/usr/bin/python3

from __future__ import annotations

import numpy as np

import rclpy
from rclpy import Future
from rclpy.qos import QoSProfile, ReliabilityPolicy, DurabilityPolicy, HistoryPolicy
from rclpy.client import Client
from rclpy.action import ActionClient
from rclpy.time import Time
from rclpy.node import Node
from rclpy.duration import Duration
from lifecycle_msgs.srv import GetState, GetState_Request
from geometry_msgs.msg import Point, Pose, PoseStamped, PoseArray, Polygon, PolygonStamped, Point32, Quaternion
from nav_msgs.msg import Path
from std_msgs.msg import Header
from action_msgs.msg import GoalStatus, GoalInfo
from nav2_msgs.srv import ClearEntireCostmap, ClearEntireCostmap_Request
from nav2_msgs.action import FollowPath, NavigateToPose

from tf2_ros import TransformException
from tf2_ros.buffer import Buffer
from tf2_ros.transform_listener import TransformListener

# noinspection PyUnresolvedReferences
import tf2_geometry_msgs  # imports PoseStamped into tf2

from enum import Enum

from spraying_task_plan import TaskPlanItem, TaskPlanItemType, TaskPlanRow

from typing import TYPE_CHECKING
if TYPE_CHECKING:
    from spraying_task_sm import SprayingTaskPlanExecutor


class NavigationActionStatus(Enum):
    NOT_STARTED = 0
    REQUESTED = 1
    FAILED_TO_START = 2
    IN_PROGRESS = 3
    SUCCEEDED = 4
    FAILED = 5


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


class NavigationManager:
    def __init__(self, node: SprayingTaskPlanExecutor):
        self._node = node

        Chronometer.node = node

        self._tf_buffer = Buffer()
        self._tf_listener = TransformListener(self._tf_buffer, self._node)

        # variables for task execution
        self.navigation_action_status: NavigationActionStatus = NavigationActionStatus.NOT_STARTED
        self._approach_poses_viz = PoseArray(header=Header(frame_id=self._node.task_plan.map_frame))
        self._last_robot_pose_stamped: PoseStamped | None = None

        # publishers, subscribers, timers and loop rate
        qos_reliable_transient_local_depth_10 = QoSProfile(
            reliability=ReliabilityPolicy.RELIABLE,
            durability=DurabilityPolicy.TRANSIENT_LOCAL,
            history=HistoryPolicy.KEEP_LAST,
            depth=10
        )

        # viz publishers
        self._approach_poses_viz_pub = self._node.create_publisher(PoseArray, '/approach_poses', qos_reliable_transient_local_depth_10)
        self._path_viz_pub = self._node.create_publisher(Path, '/plan', qos_reliable_transient_local_depth_10)
        self._row_left_viz_pub = self._node.create_publisher(PolygonStamped, '/row_left_viz', qos_reliable_transient_local_depth_10)
        self._row_right_viz_pub = self._node.create_publisher(PolygonStamped, '/row_right_viz', qos_reliable_transient_local_depth_10)

        # navigation service, action, rate
        self._clear_local_costmap_service = self._node.create_client(ClearEntireCostmap, '/local_costmap/clear_entirely_local_costmap')
        self._clear_global_costmap_service = self._node.create_client(ClearEntireCostmap, '/global_costmap/clear_entirely_global_costmap')
        self._follow_path_client = ActionClient(self._node, FollowPath, 'follow_path')
        self._navigate_to_pose_client = ActionClient(self._node, NavigateToPose, 'navigate_to_pose')
        self._navigation_goal_handle: rclpy.action.client.ClientGoalHandle | None = None
        self._navigate_to_pose_feedback: NavigateToPose.Feedback | None = None
        self._follow_path_feedback: FollowPath.Feedback | None = None
        self._loop_rate = self._node.create_rate(self._node.target_loop_rate)

    def clear_local_costmap(self):
        self._clear_local_costmap_service.call_async(ClearEntireCostmap_Request())

    def clear_global_costmap(self):
        self._clear_global_costmap_service.call_async(ClearEntireCostmap_Request())

    def start_positioning_approach(self, item: TaskPlanItem):
        # NOTE: the navigation action functions set self.navigation_action_status to REQUESTED, and eventually to
        # - NavigationActionStatus.FAILED_TO_START
        # - NavigationActionStatus.IN_PROGRESS
        # - NavigationActionStatus.SUCCEEDED
        # - NavigationActionStatus.FAILED
        self.navigation_action_status = NavigationActionStatus.NOT_STARTED

        if item.get_type() != TaskPlanItemType.ROW:
            self._node.get_logger().error(f"only ROW items should be used with state machine task executor")

        self._node.get_logger().info(f"STARTING positioning approach navigation for {item.get_item_id()}")

        approach_frame_id = item.get_item_id()
        positioning_approach_pose_stamped = PoseStamped(
            header=Header(
                frame_id=approach_frame_id,
                stamp=self._node.get_clock().now().to_msg(),
            ),
            pose=Pose(
                position=Point(x=0.0, y=0.0),
                orientation=Quaternion(w=1.0),
            )
        )

        self._approach_poses_viz.poses.append(positioning_approach_pose_stamped.pose)
        self._approach_poses_viz.header.stamp = self._node.get_clock().now().to_msg()
        self._approach_poses_viz_pub.publish(self._approach_poses_viz)

        if self._node.dry_run:
            self.navigation_action_status = NavigationActionStatus.SUCCEEDED
            self._last_robot_pose_stamped = positioning_approach_pose_stamped
        else:
            self._execute_navigate_to_pose_action(pose=positioning_approach_pose_stamped)

    def start_straightening_approach(self, item: TaskPlanItem):
        # NOTE: the navigation action functions set self.navigation_action_status to REQUESTED, and eventually to
        # - NavigationActionStatus.FAILED_TO_START
        # - NavigationActionStatus.IN_PROGRESS
        # - NavigationActionStatus.SUCCEEDED
        # - NavigationActionStatus.FAILED
        self.navigation_action_status = NavigationActionStatus.NOT_STARTED

        if item.get_type() != TaskPlanItemType.ROW:
            self._node.get_logger().error(f"only ROW items should be used with state machine task executor")

        self._node.get_logger().info(f"STARTING straightening approach navigation for {item.get_item_id()}")

        approach_frame_id = item.get_item_id()
        straightening_approach_pose_stamped = PoseStamped(
            header=Header(
                frame_id=approach_frame_id,
                stamp=self._node.get_clock().now().to_msg(),
            ),
            pose=Pose(
                position=Point(x=-self._node.task_plan.row_approach_margin, y=0.0),
                orientation=Quaternion(w=1.0),
            )
        )

        self._approach_poses_viz.poses.append(straightening_approach_pose_stamped.pose)
        self._approach_poses_viz.header.stamp = self._node.get_clock().now().to_msg()
        self._approach_poses_viz_pub.publish(self._approach_poses_viz)

        if self._node.dry_run:
            if self._last_robot_pose_stamped is not None:
                start_pose = self._transform_pose_stamped(self._last_robot_pose_stamped, frame_id=approach_frame_id, timeout=0.1)
            else:
                start_pose = straightening_approach_pose_stamped
            self._last_robot_pose_stamped = straightening_approach_pose_stamped
        else:
            start_pose = self.get_robot_pose(timeout=0.1, frame_id=approach_frame_id)
            if start_pose is None:
                self._node.get_logger().error(f"could not get robot pose for straight approach item {item.get_item_id()}")
                self.navigation_action_status = NavigationActionStatus.FAILED_TO_START
                return

        straight_approach_path = self._make_path([start_pose, straightening_approach_pose_stamped])
        if straight_approach_path is None:
            self.navigation_action_status = NavigationActionStatus.FAILED_TO_START
            return

        self._path_viz_pub.publish(straight_approach_path)

        if self._node.dry_run:
            self.navigation_action_status = NavigationActionStatus.SUCCEEDED
        else:
            self._execute_follow_path_action(
                path=straight_approach_path,
                controller_id=self._node.task_plan.straight_approach_controller_id,
                goal_checker_id=self._node.task_plan.straight_approach_goal_checker_id,
                progress_checker_id=self._node.task_plan.straight_approach_progress_checker_id,
            )

    def start_inter_row_navigation(self, item: TaskPlanItem):
        # NOTE: the navigation action functions set self.navigation_action_status to REQUESTED, and eventually to
        # - NavigationActionStatus.FAILED_TO_START
        # - NavigationActionStatus.IN_PROGRESS
        # - NavigationActionStatus.SUCCEEDED
        # - NavigationActionStatus.FAILED
        self.navigation_action_status = NavigationActionStatus.NOT_STARTED

        self._node.get_logger().info(f"STARTING row navigation: {item.get_item_id()}")

        if item.get_left_row_id() is not None:
            left_row: TaskPlanRow = self._node.task_plan.get_row(item.get_left_row_id())
            p_s = left_row.get_start_point().point
            p_e = left_row.get_end_point().point

            self._row_left_viz_pub.publish(PolygonStamped(
                header=Header(
                    frame_id=left_row.get_start_point().header.frame_id,
                    stamp=self._node.get_clock().now().to_msg(),
                ),
                polygon=Polygon(points=[
                    Point32(x=p_s.x, y=p_s.y),
                    Point32(x=p_e.x, y=p_e.y),
                ])

            ))

        if item.get_right_row_id() is not None:
            right_row: TaskPlanRow = self._node.task_plan.get_row(item.get_right_row_id())
            p_s = right_row.get_start_point().point
            p_e = right_row.get_end_point().point

            self._row_right_viz_pub.publish(PolygonStamped(
                header=Header(
                    frame_id=right_row.get_start_point().header.frame_id,
                    stamp=self._node.get_clock().now().to_msg(),
                ),
                polygon=Polygon(points=[
                    Point32(x=p_s.x, y=p_s.y),
                    Point32(x=p_e.x, y=p_e.y),
                ])

            ))

        row_waypoints = item.get_row_waypoints()
        if len(row_waypoints) < 2:
            self._node.get_logger().error(f"less than 2 row poses for row item {item.get_item_id()}")
            self.navigation_action_status = NavigationActionStatus.FAILED_TO_START
            return

        if self._node.dry_run:
            if self._last_robot_pose_stamped is not None:
                start_pose = self._transform_pose_stamped(self._last_robot_pose_stamped, frame_id=row_waypoints[0].header.frame_id, timeout=0.1)
            else:
                start_pose = None
            self._last_robot_pose_stamped = row_waypoints[-1]
        else:
            start_pose = self.get_robot_pose(timeout=0.1)
            if start_pose is None:
                self._node.get_logger().error(f"could not get robot pose for row item {item.get_item_id()}")
                self.navigation_action_status = NavigationActionStatus.FAILED_TO_START
                return

        if start_pose is not None:
            row_poses = [start_pose] + row_waypoints
        else:
            row_poses = row_waypoints

        row_path = self._make_path(row_poses)
        if row_path is None:
            self.navigation_action_status = NavigationActionStatus.FAILED_TO_START
            return

        self._path_viz_pub.publish(row_path)

        if self._node.dry_run:
            self.navigation_action_status = NavigationActionStatus.SUCCEEDED
        else:
            self._execute_follow_path_action(
                path=row_path, controller_id=self._node.task_plan.row_path_controller_id,
                goal_checker_id=self._node.task_plan.row_path_goal_checker_id,
                progress_checker_id=self._node.task_plan.row_path_progress_checker_id,
            )

    def get_robot_pose(self, timeout, frame_id=None) -> PoseStamped | None:
        if frame_id is None:
            frame_id = self._node.task_plan.map_frame

        robot_pose = PoseStamped()
        robot_pose.header.frame_id = self._node.base_frame
        robot_pose.pose.orientation.w = 1

        # TODO use self.transform_pose_stamped

        transform_chrono = Chronometer()
        last_ex: TransformException | None = None
        while rclpy.ok() and transform_chrono.total() < timeout:
            try:
                robot_pose_t = self._tf_buffer.transform(robot_pose, frame_id, timeout=Duration(seconds=0.01))
                transform_age = self._node.get_clock().now() - Time.from_msg(robot_pose_t.header.stamp)
                if transform_age < self._node.robot_pose_time_tolerance:
                    return robot_pose_t
                else:
                    self._node.get_logger().error(f"robot pose age too high: {transform_age.nanoseconds/1e9:.6f}")
            except TransformException as ex:
                last_ex = ex

            self._loop_rate.sleep()

        if last_ex is not None:
            self._node.get_logger().error(f"could not transform {self._node.base_frame} to {self._node.task_plan.map_frame}: {last_ex}")
        return None

    def _transform_pose_stamped(self, pose_stamped, frame_id, timeout):
        transform_chrono = Chronometer()
        last_ex: TransformException | None = None
        while rclpy.ok() and transform_chrono.total() < timeout:
            try:
                robot_pose_t = self._tf_buffer.transform(pose_stamped, frame_id, timeout=Duration(seconds=0.01))
                transform_age = self._node.get_clock().now() - Time.from_msg(robot_pose_t.header.stamp)
                if transform_age < self._node.robot_pose_time_tolerance:
                    return robot_pose_t
                else:
                    self._node.get_logger().error(f"robot pose age too high: {transform_age.nanoseconds/1e9:.6f}")
            except TransformException as ex:
                last_ex = ex

            self._loop_rate.sleep()

        if last_ex is not None:
            self._node.get_logger().error(f"could not transform {self._node.base_frame} to {self._node.task_plan.map_frame}: {last_ex}")
        return None

    def _make_path(self, poses_list: list[PoseStamped]) -> Path | None:
        path_header = Header(stamp=self._node.get_clock().now().to_msg(), frame_id=poses_list[0].header.frame_id)
        path = Path(header=path_header)

        for row_pose in poses_list:
            if row_pose.header.frame_id != path_header.frame_id:
                self._node.get_logger().error(f"row pose frame id [{row_pose.header.frame_id}] does not match the other poses frame id [{path_header.frame_id}]")
                return None
            row_pose.header = path_header

        for i, (p1, p2) in enumerate(zip(poses_list[0: -1], poses_list[1:])):
            p1_array = np.array([p1.pose.position.x, p1.pose.position.y, p1.pose.position.z])
            p2_array = np.array([p2.pose.position.x, p2.pose.position.y, p2.pose.position.z])
            int_points = list(np.linspace(
                p1_array,
                p2_array,
                num=int(np.ceil(np.linalg.norm(p2_array - p1_array) / self._node.task_plan.row_path_pose_distance)),
                endpoint=i == len(poses_list) - 2
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

    def wait_navigation_stack_is_ready(self, timeout: float) -> bool:
        if not self._wait_node_is_active("bt_navigator", timeout=timeout):
            return False
        if not self._wait_node_is_active("controller_server", timeout=timeout):
            return False
        if not self._wait_for_service(self._clear_local_costmap_service, timeout=timeout):
            return False
        if not self._wait_for_service(self._clear_global_costmap_service, timeout=timeout):
            return False
        if not self._wait_action_server(self._navigate_to_pose_client, "navigate_to_pose", timeout=timeout):
            return False
        if not self._wait_action_server(self._follow_path_client, "follow_path", timeout=timeout):
            return False
        return True

    def _wait_for_service(self, service: Client, timeout: float) -> bool:
        self._node.get_logger().info(f"waiting for {service.srv_name} service")
        timeout_chrono = Chronometer()
        while rclpy.ok() and not service.wait_for_service(timeout_sec=0.05):
            self._node.get_logger().info(f"{service.srv_name} service not available, waiting", throttle_duration_sec=1.0)
            if timeout_chrono.total() > timeout:
                return False
        return True

    def _wait_node_is_active(self, node_name, timeout: float) -> bool:
        self._node.get_logger().info(f"waiting for {node_name}")
        state_client = self._node.create_client(GetState, f"{node_name}/get_state")
        timeout_chrono = Chronometer()
        while rclpy.ok() and not state_client.wait_for_service(timeout_sec=0.05):
            self._node.get_logger().info(f'still waiting {node_name} lifecycle service', throttle_duration_sec=1.0)
            if timeout_chrono.total() > timeout:
                self._node.get_logger().error(f'{node_name} lifecycle state service was not available before timeout [{timeout} s]')
                return False

        prev_state = None
        while rclpy.ok():
            response_future = state_client.call_async(GetState_Request())
            while rclpy.ok() and response_future.result() is None:
                if timeout_chrono.total() > timeout:
                    self._node.get_logger().error(f"{node_name} lifecycle state service did not respond or did not transition to active before timeout [{timeout} s]")
                    return False
                else:
                    self._loop_rate.sleep()

            state = response_future.result().current_state.label
            if state != prev_state:
                prev_state = state
                self._node.get_logger().info(f'{node_name} lifecycle state: {state}')

            if state == "active":
                return True
            else:
                self._loop_rate.sleep()
        return False

    def _wait_action_server(self, action_client: ActionClient, action_client_name: str, timeout: float) -> bool:
        self._node.get_logger().info(f"waiting for {action_client_name} action server")
        timeout_chrono = Chronometer()
        while rclpy.ok() and not action_client.wait_for_server(timeout_sec=0.05):
            self._node.get_logger().info(f"{action_client_name} action server not available, waiting", throttle_duration_sec=1.0)
            if timeout_chrono.total() > timeout:
                return False
        return True

    """
     Send the NavigateToPose action request.
    """
    def _execute_navigate_to_pose_action(self, pose: PoseStamped) -> None:
        if self.navigation_action_status not in [NavigationActionStatus.NOT_STARTED, NavigationActionStatus.SUCCEEDED]:
            self._node.get_logger().error(f"trying to start a navigation action while another action is executing")
            return

        goal_msg = NavigateToPose.Goal()
        goal_msg.pose = pose

        self._node.get_logger().debug(f"execute_navigate_to_pose_action: sending goal")
        response_future: Future = self._navigate_to_pose_client.send_goal_async(goal_msg, self._navigate_to_pose_feedback_callback)
        response_future.add_done_callback(self._navigate_to_pose_response_callback)
        self.navigation_action_status = NavigationActionStatus.REQUESTED
        # TODO action timeout

    """
     Receive the NavigateToPose action response.
    """
    def _navigate_to_pose_response_callback(self, future: Future) -> None:
        if self.navigation_action_status != NavigationActionStatus.REQUESTED:
            self._node.get_logger().error(
                f"received a navigation action response but navigation_action_status is "
                f"{self.navigation_action_status.name}, it should be REQUESTED")
            return

        self._navigation_goal_handle = future.result()
        if self._navigation_goal_handle.accepted:
            self._node.get_logger().debug(f"navigate_to_pose action response: accepted")
            result_future: Future = self._navigation_goal_handle.get_result_async()
            result_future.add_done_callback(self._navigate_to_pose_result_callback)
            self.navigation_action_status = NavigationActionStatus.IN_PROGRESS

        else:
            self._node.get_logger().warn(f"navigate_to_pose action response: rejected")
            self.navigation_action_status = NavigationActionStatus.FAILED_TO_START

    """
     Receive the NavigateToPose action feedback.
    """
    def _navigate_to_pose_feedback_callback(self, msg) -> None:
        self._navigate_to_pose_feedback = msg.feedback
        self._node.get_logger().info(
            f"navigate_to_pose_feedback:\n"
            f"    distance_remaining:       {msg.feedback.distance_remaining:.1f} m\n"
            f"    estimated_time_remaining: {Duration.from_msg(msg.feedback.estimated_time_remaining).nanoseconds/1e9:.1f} s\n"
            f"    navigation_time:          {Duration.from_msg(msg.feedback.navigation_time).nanoseconds/1e9:.1f} s\n",
            throttle_duration_sec=5.0
        )

    """
     Receive the NavigateToPose action result.
    """
    def _navigate_to_pose_result_callback(self, goal_result_future) -> None:
        navigation_result_status = goal_result_future.result().status
        if navigation_result_status == GoalStatus.STATUS_SUCCEEDED:
            self._node.get_logger().debug('navigate_to_pose action succeeded')
            self.navigation_action_status = NavigationActionStatus.SUCCEEDED
        else:
            self._node.get_logger().debug(f'navigate_to_pose action failed with status code: {navigation_result_status}')
            self.navigation_action_status = NavigationActionStatus.FAILED

    """
     Send the FollowPath action request.
    """
    def _execute_follow_path_action(self, path: Path, controller_id, goal_checker_id='', progress_checker_id='') -> None:
        if self.navigation_action_status not in [NavigationActionStatus.NOT_STARTED, NavigationActionStatus.SUCCEEDED]:
            self._node.get_logger().error(f"trying to start a navigation action while another action is executing")
            return

        goal_msg = FollowPath.Goal()
        goal_msg.path = path
        goal_msg.controller_id = controller_id
        goal_msg.goal_checker_id = goal_checker_id
        goal_msg.progress_checker_id = progress_checker_id

        self._node.get_logger().debug(f"execute_follow_path_action: sending goal")
        response_future: Future = self._follow_path_client.send_goal_async(goal_msg, self._follow_path_feedback_callback)
        response_future.add_done_callback(self._follow_path_response_callback)
        self.navigation_action_status = NavigationActionStatus.REQUESTED
        # TODO action timeout

    """
     Receive the FollowPath action response.
    """
    def _follow_path_response_callback(self, future: Future) -> None:
        if self.navigation_action_status != NavigationActionStatus.REQUESTED:
            self._node.get_logger().error(
                f"received a navigation action response but navigation_action_status is "
                f"{self.navigation_action_status.name}, it should be REQUESTED")
            return

        self._navigation_goal_handle = future.result()
        if self._navigation_goal_handle.accepted:
            self._node.get_logger().debug(f"follow_path action response: accepted")
            result_future: Future = self._navigation_goal_handle.get_result_async()
            result_future.add_done_callback(self._follow_path_result_callback)
            self.navigation_action_status = NavigationActionStatus.IN_PROGRESS

        else:
            self._node.get_logger().warn(f"follow_path action response: rejected")
            self.navigation_action_status = NavigationActionStatus.FAILED_TO_START

    """
     Receive the FollowPath action feedback.
    """
    def _follow_path_feedback_callback(self, msg) -> None:
        self._follow_path_feedback = msg.feedback
        self._node.get_logger().info(
            f"follow_path_feedback:\n"
            f"    distance_to_goal:         {msg.feedback.distance_to_goal:.1f} m\n"
            f"    speed:                    {msg.feedback.speed:.3f} m/s\n",
            throttle_duration_sec=5.0
        )

    """
     Receive the FollowPath action result.
    """
    def _follow_path_result_callback(self, future: Future) -> None:
        navigation_result_status = future.result().status
        if navigation_result_status == GoalStatus.STATUS_SUCCEEDED:
            self._node.get_logger().debug('follow_path action succeeded')
            self.navigation_action_status = NavigationActionStatus.SUCCEEDED
        else:
            self._node.get_logger().debug(f'follow_path action failed with status code: {navigation_result_status}')
            self.navigation_action_status = NavigationActionStatus.FAILED

    """
     Request to cancel the current navigation action, if there is one in progress.
    """
    def cancel_navigation_action(self):
        if self._navigation_goal_handle is not None:
            self._node.get_logger().info('canceling current navigation action')
            cancel_navigation_action_future: Future = self._navigation_goal_handle.cancel_goal_async()
            cancel_navigation_action_future.add_done_callback(self._cancel_navigation_action_response_callback)
        else:
            self._node.get_logger().info('no navigation actions in progress')

    """
     Receive the action cancel result.
    """
    def _cancel_navigation_action_response_callback(self, _: GoalInfo) -> None:
        self._node.get_logger().info(f"current navigation action was cancelled")
        self._navigation_goal_handle = None
        self.navigation_action_status = NavigationActionStatus.NOT_STARTED
