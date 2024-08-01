#!/usr/bin/python3

from spraying_task_plan import SprayingTaskPlan, TaskPlanItemResult

import os.path
import numpy as np

import rclpy
from rclpy.node import Node
from rclpy.duration import Duration
from geometry_msgs.msg import Point, Pose, PoseStamped
from nav_msgs.msg import Path
from std_msgs.msg import Header

from nav2_simple_commander.robot_navigator import BasicNavigator, TaskResult

from ament_index_python.packages import get_package_share_directory

from tf2_ros import TransformException
from tf2_ros.buffer import Buffer
from tf2_ros.transform_listener import TransformListener

# noinspection PyUnresolvedReferences
import tf2_geometry_msgs  # imports PoseStamped into tf2


def result2str(r):
    if r == TaskResult.SUCCEEDED:
        return 'succeeded'
    elif r == TaskResult.CANCELED:
        return 'canceled'
    elif r == TaskResult.FAILED:
        return 'failed'
    else:
        return 'unknown'


class AsbTaskPlanNavigator(Node):

    def __init__(self, navigator_node: BasicNavigator):
        super().__init__('spraying_task_plan_maker')

        self.navigator_node = navigator_node

        default_task_plan_file_path = os.path.expanduser("~/asb_task_plan.yaml")
        self.declare_parameter('task_plan_file_path', default_task_plan_file_path)
        self.task_plan_file_path = os.path.expanduser(self.get_parameter('task_plan_file_path').get_parameter_value().string_value)
        self.get_logger().info(f"task_plan_file_path set to {self.task_plan_file_path}")

        self.task_plan: SprayingTaskPlan = SprayingTaskPlan.load(self.task_plan_file_path)
        self.get_logger().info(f"loaded task plan with path ids: {self.task_plan.get_item_ids()}")

        default_base_frame = "base_footprint"
        self.declare_parameter('base_frame', default_base_frame)
        self.base_frame = self.get_parameter('base_frame').get_parameter_value().string_value

        if not len(self.task_plan.get_item_ids()):
            self.get_logger().error(f"Empty task plan")
            return

        self.tf_buffer = Buffer()
        self.tf_listener = TransformListener(self.tf_buffer, self)

        self.path_pub = self.create_publisher(Path, '/plan', 10)

        self.item_index = 0
        self.item_timeout = None
        self.timer = self.create_timer(1, self.timer_callback)

    def timer_callback(self):

        if self.item_index >= len(self.task_plan.task_plan_items):
            self.get_logger().info(f"Finished task plan")

            for i in self.task_plan.task_plan_items:
                i.result = None
            self.item_index = 0
            return

        item = self.task_plan.task_plan_items[self.item_index]

        # if we are just starting this item execution, send the navigation command
        if item.result is None:
            item.result = TaskPlanItemResult()
            item.result.started = True

            if item.type == "approach":
                self.get_logger().info(f"STARTING approach navigation: {item.item_id}")
                self.navigator_node.goToPose(pose=item.get_pose_stamped_list()[-1])
            elif item.type == "row":
                self.get_logger().info(f"STARTING row navigation: {item.item_id}")
                path = item.get_path()
                if not len(path.poses):
                    self.get_logger().error(f"Empty path in item {item.item_id}")
                    self.item_index += 1
                    return

                # robot pose, untransformed (in base frame)
                current_robot_pose = PoseStamped()
                current_robot_pose.header.frame_id = self.base_frame
                current_robot_pose.pose.orientation.w = 1

                path_header = Header(stamp=self.get_clock().now().to_msg(), frame_id=path.poses[0].header.frame_id)
                path.header = path_header
                try:
                    current_robot_pose = self.tf_buffer.transform(current_robot_pose, path_header.frame_id, timeout=Duration(seconds=0.1))
                except TransformException as ex:
                    self.get_logger().error(f"Could not transform {self.base_frame} to {path_header.frame_id}: {ex}")
                    return

                path.poses.insert(0, current_robot_pose)
                for p in path.poses:
                    p.header = path_header

                path_int = Path(header=path.header)
                for i, (p1, p2) in enumerate(zip(path.poses[0: -1], path.poses[1:])):
                    p1_v = np.array([p1.pose.position.x, p1.pose.position.y, p1.pose.position.z])
                    p2_v = np.array([p2.pose.position.x, p2.pose.position.y, p2.pose.position.z])
                    int_points = list(np.linspace(
                        p1_v,
                        p2_v,
                        num=int(np.ceil(np.linalg.norm(p2_v - p1_v) / 0.1)),
                        endpoint=i == len(path.poses) - 2
                    ))
                    int_poses = list(map(
                        lambda v: PoseStamped(
                            header=path_header,
                            pose=Pose(
                                position=Point(x=v[0], y=v[1], z=v[2]),
                                orientation=p1.pose.orientation
                            )
                        ),
                        int_points
                    ))
                    path_int.poses += int_poses

                self.path_pub.publish(path_int)

                self.navigator_node.followPath(path=path_int, controller_id='FollowPath')
            else:
                self.get_logger().error(f"unknown task plan item type [{item.type}] in item {item.item_id}")
                self.item_index += 1
                return

            # if item.nav_timeout is not None:
            #     self.item_timeout = Duration(seconds=item.nav_timeout)
            # else:
            #     self.item_timeout = None

        # check the execution timeout
        if not self.navigator_node.isTaskComplete():
            pass
            # feedback = self.navigator_node.getFeedback()
            # # self.get_logger().info(f"ETA:     {Duration.from_msg(feedback.estimated_time_remaining).nanoseconds / 1e9:.1f} s")
            # # self.get_logger().info(f"EDA:     {feedback.distance_remaining:.1f} s")
            # if self.item_timeout is not None:
            #     if Duration.from_msg(feedback.navigation_time) > self.item_timeout:
            #         self.navigator_node.cancelTask()
            #         self.get_logger().info("Timeout exceeded, cancelling navigation task")
            #     else:
            #         self.get_logger().info(f"Timeout: {self.item_timeout.nanoseconds / 1e9 - (Duration.from_msg(feedback.navigation_time)).nanoseconds / 1e9:.1f} s")
        else:
            item.result.nav_result = self.navigator_node.getResult()
            if item.result.nav_result == TaskResult.SUCCEEDED:
                self.get_logger().info(f"Nav result: {result2str(item.result.nav_result)}")
            else:
                self.get_logger().error(f"Nav result: {result2str(item.result.nav_result)}")
            self.item_index += 1


def main():
    rclpy.init()
    navigator_node = BasicNavigator()
    navigator_node.waitUntilNav2Active(localizer='robot_localization')
    task_plan_navigator_node = AsbTaskPlanNavigator(navigator_node=navigator_node)
    # noinspection PyBroadException
    try:
        rclpy.spin(task_plan_navigator_node)
    except KeyboardInterrupt:
        navigator_node.cancelTask()


if __name__ == '__main__':
    main()
