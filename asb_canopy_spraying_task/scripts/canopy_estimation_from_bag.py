#!/usr/bin/python3

import os.path
from collections import defaultdict

import rclpy
from rclpy.time import Time
from rclpy.node import Node
from rclpy.duration import Duration


from plan_manager import PlanManager
from spraying_manager import SprayingManager, SprayingStatus
from spraying_task_plan import SprayingTaskPlan, TaskPlanItem

BLUE = "\033[94m"


class CanopyEstimationFromBag(Node):
    def __init__(self):
        super().__init__('est_from_bag')

        self.declare_parameter('task_plan_file_path', rclpy.Parameter.Type.STRING)
        self.task_plan_file_path = os.path.expanduser(self.get_parameter('task_plan_file_path').get_parameter_value().string_value)

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

        # run time variables
        self.item_index: int = 0
        self.current_item: TaskPlanItem = self.task_plan.items[self.item_index]

        self.started_estimation: defaultdict[str, bool] = defaultdict(lambda: False)
        self.finished_estimation: defaultdict[str, bool] = defaultdict(lambda: False)

        # ~/asb_logs/2025-06-23/d_4_rosbag2_2025-06-23__11-33-25_all/
        self.start_estimation_time: dict[str, float] = {
            'inter_row_10_c': 1750671233.92,
            'inter_row_10_b': 1750671533.11,
        }
        self.stop_estimation_time: dict[str, float] = {
            'inter_row_10_c': 1750671343.63,
            'inter_row_10_b': 1750671643.38,
        }

        # ~/asb_logs/2025-06-23/d_0_rosbag2_2025-06-23__10-24-32_all/
        # self.start_estimation_time: dict[str, float] = {
        #     'inter_row_10_c': 1750667126.68,
        # }
        # self.stop_estimation_time: dict[str, float] = {
        #     'inter_row_10_c': 1750667238.08,
        # }

        # managers
        self.dry_run = False
        self.plan_manager = PlanManager(node=self)
        self.spraying_manager = SprayingManager(node=self)

        # setup
        self.spraying_manager.setup()
        self.get_logger().info(f"self.spraying_manager.setup")

        self.create_timer(0.1, self.timer_callback)

    def timer_callback(self):
        now_s = self.get_clock().now().nanoseconds/1E9

        if not self.started_estimation[self.current_item.get_item_id()] and now_s > self.start_estimation_time[self.current_item.get_item_id()]:
            self.started_estimation[self.current_item.get_item_id()] = True

            self.spraying_manager.start_spray_regulator(self.current_item)

            if self.spraying_manager.spraying_status in [SprayingStatus.NOT_SPRAYING, SprayingStatus.STARTING]:
                return

            if self.spraying_manager.spraying_status == SprayingStatus.FAILURE:
                self.get_logger().error(f"spraying failed")
                return

        if not self.finished_estimation[self.current_item.get_item_id()] and self.started_estimation[self.current_item.get_item_id()] and now_s > self.stop_estimation_time[self.current_item.get_item_id()]:
            self.finished_estimation[self.current_item.get_item_id()] = True
            self.spraying_manager.stop_spray_regulator()

            self.item_index += 1
            if self.item_index >= len(self.task_plan.items):
                return

            self.current_item = self.task_plan.items[self.item_index]


def main(args=None):
    rclpy.init(args=args)
    node = CanopyEstimationFromBag()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass


if __name__ == '__main__':
    main()
