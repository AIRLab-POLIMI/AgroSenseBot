#!/usr/bin/python3

from spraying_task_plan import SprayingTaskPlan, TaskPlanItemResult

import os.path

import rclpy
from rclpy.node import Node
from rclpy.duration import Duration

from nav2_simple_commander.robot_navigator import BasicNavigator, TaskResult

from ament_index_python.packages import get_package_share_directory


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

        if not len(self.task_plan.get_item_ids()):
            self.get_logger().error(f"Empty task plan")
            return

        self.item_index = 0
        self.item_timeout = None
        self.timer = self.create_timer(1, self.timer_callback)

    def timer_callback(self):

        if self.item_index >= len(self.task_plan.task_plan_items):
            self.get_logger().info(f"Finished task plan")
            return

        plan_item = self.task_plan.task_plan_items[self.item_index]
        self.get_logger().info(f"current task plan item: {plan_item.item_id}")

        # if we are just starting this item execution, send the navigation command
        if plan_item.result is None:
            plan_item.result = TaskPlanItemResult()
            plan_item.result.started = True

            behavior_tree_file_path = os.path.join(get_package_share_directory("asb_task"), "config", "behavior_trees", plan_item.behavior_tree_name)
            if not os.path.isfile(behavior_tree_file_path):
                self.get_logger().error(f"behavior_tree file does not exists: {behavior_tree_file_path}, task plan item: {plan_item.item_id}")
                self.item_index += 1
                return

            self.navigator_node.goThroughPoses(poses=plan_item.get_pose_stamped_list(), behavior_tree=behavior_tree_file_path)

            if plan_item.nav_timeout is not None:
                self.item_timeout = Duration(seconds=plan_item.nav_timeout)
            else:
                self.item_timeout = None

        # check the execution timeout
        if not self.navigator_node.isTaskComplete():
            feedback = self.navigator_node.getFeedback()
            self.get_logger().info(f"ETA:     {Duration.from_msg(feedback.estimated_time_remaining).nanoseconds / 1e9:.1f} s")
            if self.item_timeout is not None:
                if Duration.from_msg(feedback.navigation_time) > self.item_timeout:
                    self.navigator_node.cancelTask()
                    self.get_logger().info("Timeout exceeded, cancelling navigation task")
                else:
                    self.get_logger().info(f"Timeout: {self.item_timeout.nanoseconds / 1e9 - (Duration.from_msg(feedback.navigation_time)).nanoseconds / 1e9:.1f} s")
        else:
            plan_item.result.nav_result = self.navigator_node.getResult()
            self.get_logger().info(f"Nav result: {result2str(plan_item.result.nav_result)}")
            self.item_index += 1


def main():
    rclpy.init()
    navigator_node = BasicNavigator()
    navigator_node.waitUntilNav2Active(localizer='robot_localization')
    task_plan_navigator_node = AsbTaskPlanNavigator(navigator_node=navigator_node)
    try:
        rclpy.spin(task_plan_navigator_node)
    except KeyboardInterrupt:
        navigator_node.cancelTask()


if __name__ == '__main__':
    main()
