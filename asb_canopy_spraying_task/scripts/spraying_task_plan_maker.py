#!/usr/bin/python3

from spraying_task_plan import SprayingTaskPlan, TaskPlanItem

import os
import yaml

import rclpy
from rclpy.node import Node
from rclpy.duration import Duration

from tf2_ros import TransformException
from tf2_ros.buffer import Buffer
from tf2_ros.transform_listener import TransformListener
from tf_transformations import euler_from_quaternion

from nav2_simple_commander.robot_navigator import BasicNavigator

# noinspection PyUnresolvedReferences
import tf2_geometry_msgs  # used, provides to tf2 the conversion function for PoseStamped
from geometry_msgs.msg import PoseStamped, PoseWithCovarianceStamped


def distance_greater_than_threshold(p1: PoseStamped, p2: PoseStamped, th: float):
    return (p1.pose.position.x - p2.pose.position.x) ** 2 + (p1.pose.position.y - p2.pose.position.y) ** 2 > th ** 2


class AsbTaskPlanMaker(Node):

    def __init__(self, navigator_node: BasicNavigator):
        super().__init__('spraying_task_plan_maker')

        self.navigator_node = navigator_node

        default_source_task_plan_file_path = os.path.expanduser("~/asb_canopy_spraying_task_plan.yaml")
        self.declare_parameter('source_task_plan_file_path', default_source_task_plan_file_path)
        self.source_task_plan_file_path = os.path.expanduser(self.get_parameter('source_task_plan_file_path').get_parameter_value().string_value)
        self.get_logger().info(f"source_task_plan_file_path set to {self.source_task_plan_file_path}")

        default_destination_task_plan_file_path = os.path.expanduser("~/asb_canopy_spraying_task_plan.yaml")
        self.declare_parameter('destination_task_plan_file_path', default_destination_task_plan_file_path)
        self.destination_task_plan_file_path = os.path.expanduser(self.get_parameter('destination_task_plan_file_path').get_parameter_value().string_value)
        self.get_logger().info(f"destination_task_plan_file_path set to {self.destination_task_plan_file_path}")

        default_frame_id = "map"
        self.declare_parameter('frame_id', default_frame_id)
        self.frame_id = self.get_parameter('frame_id').get_parameter_value().string_value
        self.get_logger().info(f"frame_id set to {self.frame_id}")

        default_planner_id = "GridBased"
        self.declare_parameter('planner_id', default_planner_id)
        self.planner_id = self.get_parameter('planner_id').get_parameter_value().string_value
        self.get_logger().info(f"planner_id set to {self.planner_id}")

        default_task_plan_item_id = "unnamed"
        self.declare_parameter('task_plan_item_id', default_task_plan_item_id)
        self.task_plan_item_id = self.get_parameter('task_plan_item_id').get_parameter_value().string_value
        self.get_logger().info(f"task_plan_item_id set to {self.task_plan_item_id}")

        default_behavior_tree_name = "asb_no_replanning_navigate_through_poses.xml"
        self.declare_parameter('behavior_tree_name', default_behavior_tree_name)
        self.behavior_tree_name = self.get_parameter('behavior_tree_name').get_parameter_value().string_value
        self.get_logger().info(f"behavior_tree_name set to {self.behavior_tree_name}")

        default_dist_th = 2.0
        self.declare_parameter('dist_th', default_dist_th)
        self.dist_th = self.get_parameter('dist_th').get_parameter_value().double_value
        self.get_logger().info(f"Distance threshold set to {self.dist_th}")

        self.last_saved_pose = None

        self.task_plan: SprayingTaskPlan = SprayingTaskPlan.load(file_path=self.source_task_plan_file_path)
        self.get_logger().info(f"loaded task plan with item ids: {self.task_plan.get_item_ids()}")
        self.task_plan_item: TaskPlanItem = self.task_plan.get_item(self.task_plan_item_id)
        if self.task_plan_item is None:
            self.get_logger().info(f"creating new task plan item")
            self.task_plan_item = TaskPlanItem(item_id=self.task_plan_item_id)
            self.task_plan.items.append(self.task_plan_item)

        if self.behavior_tree_name == "":
            self.get_logger().info(f"not setting plan item's behavior_tree_name")
        else:
            if self.task_plan_item.behavior_tree_name == "":
                self.get_logger().info(f"setting (new) task plan item's behavior_tree_name to: {self.behavior_tree_name}")
            elif self.task_plan_item.behavior_tree_name != self.behavior_tree_name:
                self.get_logger().info(f"changing task plan item's behavior_tree_name to: {self.behavior_tree_name} (before it was {self.task_plan_item.behavior_tree_name})")
            else:
                self.get_logger().info(f"task plan item's behavior_tree_name is unchanged: {self.behavior_tree_name}")
            self.task_plan_item.behavior_tree_name = self.behavior_tree_name

        if len(self.task_plan_item.get_pose_stamped_list()):
            path_result = self.navigator_node.getPathThroughPoses(start=PoseStamped(), goals=self.task_plan_item.get_pose_stamped_list(), planner_id=self.planner_id, use_start=False)
            if path_result is not None:
                self.get_logger().info(f"received path")

        self.tf_buffer = Buffer()
        self.tf_listener = TransformListener(self.tf_buffer, self)

        self.pose_sub = self.create_subscription(PoseWithCovarianceStamped, '/initialpose', self.pose_callback, 10)

    def pose_callback(self, msg: PoseWithCovarianceStamped):

        pose_stamped = None
        if isinstance(msg, PoseWithCovarianceStamped):
            pose_stamped = PoseStamped(header=msg.header, pose=msg.pose.pose)
        elif isinstance(msg, PoseStamped):
            pose_stamped = msg
        else:
            self.get_logger().error(f"Could not log pose type {type(msg)}")

        q = pose_stamped.pose.orientation
        _, _, yaw = euler_from_quaternion([q.x, q.y, q.z, q.w])

        # transform the pose into the desired frame
        try:
            pose_stamped = self.tf_buffer.transform(pose_stamped, self.frame_id, timeout=Duration(seconds=0.1))
        except TransformException as ex:
            self.get_logger().info(f'Could not transform pose to {self.frame_id}: {ex}')
            return

        if self.last_saved_pose is None or distance_greater_than_threshold(pose_stamped, self.last_saved_pose, self.dist_th):
            self.task_plan_item.add_pose(pose_stamped)
            self.get_logger().info(f"Adding pose: x={pose_stamped.pose.position.x:.2f}, y={pose_stamped.pose.position.y:.2f}, yaw={yaw:.2f}, frame_id={pose_stamped.header.frame_id}")
        else:
            # if the last saved robot pose is farther from the last logged pose than the threshold, log it
            self.task_plan_item.replace_last_pose(pose_stamped)
            self.get_logger().info(f"Replacing pose: x={pose_stamped.pose.position.x}, y={pose_stamped.pose.position.y}, yaw={yaw}, frame_id={pose_stamped.header.frame_id}")

        self.last_saved_pose = pose_stamped

        self.write_task_plan()

        self.navigator_node.getPathThroughPoses(start=PoseStamped(), goals=self.task_plan_item.get_pose_stamped_list(), planner_id=self.planner_id, use_start=False)

    def write_task_plan(self):
        try:
            with open(self.destination_task_plan_file_path, 'w') as task_plan_file:
                yaml.dump(self.task_plan, task_plan_file, default_flow_style=False)
        except Exception as ex:
            self.get_logger().error(f"Error writing task plan to file: {str(ex)}")
            return


def main():
    rclpy.init()
    navigator_node = BasicNavigator()
    navigator_node.waitUntilNav2Active(localizer='robot_localization')
    task_plan_maker_node = AsbTaskPlanMaker(navigator_node=navigator_node)
    try:
        rclpy.spin(task_plan_maker_node)
    except KeyboardInterrupt:
        pass


if __name__ == '__main__':
    main()
