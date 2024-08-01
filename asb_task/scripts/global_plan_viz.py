#!/usr/bin/python3

from spraying_task_plan import SprayingTaskPlan, TaskPlanItem

import os

import rclpy
from rclpy.node import Node

from tf2_ros.buffer import Buffer
from tf2_ros.transform_listener import TransformListener

from nav2_simple_commander.robot_navigator import BasicNavigator

# noinspection PyUnresolvedReferences
import tf2_geometry_msgs  # used, provides to tf2 the conversion function for PoseStamped
from geometry_msgs.msg import PoseStamped


def distance_greater_than_threshold(p1: PoseStamped, p2: PoseStamped, th: float):
    return (p1.pose.position.x - p2.pose.position.x) ** 2 + (p1.pose.position.y - p2.pose.position.y) ** 2 > th ** 2


class AsbGlobalPlanViz(Node):

    def __init__(self, navigator_node: BasicNavigator):
        super().__init__('global_plan_viz')

        self.navigator_node = navigator_node

        default_source_task_plan_file_path = os.path.expanduser("~/asb_task_plan.yaml")
        self.declare_parameter('source_task_plan_file_path', default_source_task_plan_file_path)
        self.source_task_plan_file_path = os.path.expanduser(self.get_parameter('source_task_plan_file_path').get_parameter_value().string_value)
        self.get_logger().info(f"source_task_plan_file_path set to {self.source_task_plan_file_path}")

        default_planner_id = "GridBased"
        self.declare_parameter('planner_id', default_planner_id)
        self.planner_id = self.get_parameter('planner_id').get_parameter_value().string_value
        self.get_logger().info(f"planner_id set to {self.planner_id}")

        default_task_plan_item_id = "unnamed"
        self.declare_parameter('task_plan_item_id', default_task_plan_item_id)
        self.task_plan_item_id = self.get_parameter('task_plan_item_id').get_parameter_value().string_value
        self.get_logger().info(f"task_plan_item_id set to {self.task_plan_item_id}")

        self.task_plan: SprayingTaskPlan = SprayingTaskPlan.load(file_path=self.source_task_plan_file_path)
        self.get_logger().info(f"loaded task plan with item ids: {self.task_plan.get_item_ids()}")
        self.task_plan_item: TaskPlanItem = self.task_plan.get_item(self.task_plan_item_id)
        if self.task_plan_item is None:
            self.get_logger().info(f"creating new task plan item")
            self.task_plan_item = TaskPlanItem(item_id=self.task_plan_item_id)
            self.task_plan.task_plan_items.append(self.task_plan_item)

        if len(self.task_plan_item.get_pose_stamped_list()):
            path_result = self.navigator_node.getPathThroughPoses(start=PoseStamped(), goals=self.task_plan_item.get_pose_stamped_list(), planner_id=self.planner_id, use_start=False)
            if path_result is not None:
                self.get_logger().info(f"received path")

        self.tf_buffer = Buffer()
        self.tf_listener = TransformListener(self.tf_buffer, self)

        self.timer = self.create_timer(1, self.timer_callback)

    def timer_callback(self):

        self.navigator_node.getPathThroughPoses(start=PoseStamped(), goals=self.task_plan_item.get_pose_stamped_list(), planner_id=self.planner_id, use_start=False)


def main():
    rclpy.init()
    navigator_node = BasicNavigator()
    navigator_node.waitUntilNav2Active(localizer='robot_localization')
    task_plan_maker_node = AsbGlobalPlanViz(navigator_node=navigator_node)
    try:
        rclpy.spin(task_plan_maker_node)
    except KeyboardInterrupt:
        pass


if __name__ == '__main__':
    main()
