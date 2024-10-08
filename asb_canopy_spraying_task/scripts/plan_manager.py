#! /usr/bin/python3

from rclpy.node import Node
from geometry_msgs.msg import TransformStamped, PoseStamped
from tf2_ros import StaticTransformBroadcaster

from spraying_task_plan import SprayingTaskPlan, TaskPlanItem, TaskPlanItemType


class PlanManager:

    def __init__(self, node: Node, task_plan: SprayingTaskPlan):
        self._node = node
        self._tf_static_broadcaster = StaticTransformBroadcaster(self._node)
        self._loop_rate = self._node.create_rate(10)

        self.task_plan = task_plan

    def setup(self):
        # broadcast inter-row frames
        inter_row_item: TaskPlanItem
        for inter_row_item in self.task_plan.items:
            if inter_row_item.get_type() != TaskPlanItemType.ROW:
                self._node.get_logger().error(f"only ROW items should be used with state machine task executor")
                continue
            self._loop_rate.sleep()
            p_1: PoseStamped = inter_row_item.get_row_waypoints()[0]
            self._broadcast_static_transform(child_frame_id=inter_row_item.get_item_id(), p=p_1)

    def _broadcast_static_transform(self, child_frame_id: str, p: PoseStamped) -> None:
        t = TransformStamped()
        t.header.stamp = self._node.get_clock().now().to_msg()
        t.header.frame_id = p.header.frame_id
        t.child_frame_id = child_frame_id
        t.transform.translation.x = p.pose.position.x
        t.transform.translation.y = p.pose.position.y
        t.transform.translation.z = p.pose.position.z
        t.transform.rotation = p.pose.orientation
        self._tf_static_broadcaster.sendTransform(t)
