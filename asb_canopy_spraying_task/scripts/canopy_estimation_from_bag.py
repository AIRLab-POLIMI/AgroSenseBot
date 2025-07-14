#!/usr/bin/python3

import os.path

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
        self.current_item: TaskPlanItem | None = None

        self.started_estimation: bool = False
        self.finished_estimation: bool = False
        self.start_estimation_time: float = 1750667126.68
        self.stop_estimation_time: float = 1750667238.08

        # managers
        self.dry_run = False
        self.plan_manager = PlanManager(node=self)
        self.spraying_manager = SprayingManager(node=self)

        # setup
        # self.plan_manager.setup()
        # self.get_logger().info(f"self.plan_manager.setup")

        self.spraying_manager.setup()
        self.get_logger().info(f"self.spraying_manager.setup")

        self.current_item = self.task_plan.items[self.item_index]

        self.create_timer(0.1, self.timer_callback)

    def timer_callback(self):
        now_s = self.get_clock().now().nanoseconds/1E9

        self.get_logger().info(f"ros time: {now_s:.3f}")

        if not self.started_estimation and now_s > self.start_estimation_time:
            self.started_estimation = True
            self.spraying_manager.start_spray_regulator(self.current_item)

            if self.spraying_manager.spraying_status in [SprayingStatus.NOT_SPRAYING, SprayingStatus.STARTING]:
                return

            if self.spraying_manager.spraying_status == SprayingStatus.FAILURE:
                self.get_logger().error(f"spraying failed")
                return

        if not self.finished_estimation and self.started_estimation and now_s > self.stop_estimation_time:
            self.finished_estimation = True
            self.spraying_manager.stop_spray_regulator()


def main(args=None):
    rclpy.init(args=args)
    node = CanopyEstimationFromBag()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass


if __name__ == '__main__':
    main()
