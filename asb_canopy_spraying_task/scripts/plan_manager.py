#! /usr/bin/python3

from __future__ import annotations

from std_msgs.msg import Header

from rclpy.qos import QoSProfile, ReliabilityPolicy, DurabilityPolicy

from asb_msgs.msg import PolygonStampedArray
from geometry_msgs.msg import TransformStamped, PoseStamped, PolygonStamped, Polygon, Point32
from tf2_ros import StaticTransformBroadcaster

from spraying_task_plan import TaskPlanItem, TaskPlanItemType

from typing import TYPE_CHECKING
if TYPE_CHECKING:
    from spraying_task_sm import SprayingTaskPlanExecutor


class PlanManager:

    def __init__(self, node: SprayingTaskPlanExecutor):
        self._node = node
        self._tf_static_broadcaster = StaticTransformBroadcaster(self._node)

        qos_profile = QoSProfile(
            reliability=ReliabilityPolicy.RELIABLE,
            durability=DurabilityPolicy.TRANSIENT_LOCAL,
            depth=10
        )

        self._rows_polygons_pub = self._node.create_publisher(PolygonStampedArray, 'row_polygons', qos_profile)

        self._loop_rate = self._node.create_rate(10)

    def setup(self):
        # broadcast inter-row frames
        inter_row_item: TaskPlanItem
        for inter_row_item in self._node.task_plan.items:
            if inter_row_item.get_type() != TaskPlanItemType.ROW:
                self._node.get_logger().error(f"only ROW items should be used with state machine task executor")
                continue
            self._loop_rate.sleep()
            p_1: PoseStamped = inter_row_item.get_row_waypoints()[0]
            self._broadcast_static_transform(child_frame_id=inter_row_item.get_item_id(), p=p_1)

        # publish row polygons for the gridmap publisher
        row_polygons_msg = PolygonStampedArray()
        for row in self._node.task_plan.rows:
            row_polygon = PolygonStamped(
                header=Header(
                    frame_id=row.get_start_point().header.frame_id,
                    stamp=self._node.get_clock().now().to_msg(),
                ),
                polygon=Polygon(
                    points=[
                        Point32(x=row.get_start_point().point.x, y=row.get_start_point().point.y),
                        Point32(x=row.get_end_point().point.x, y=row.get_end_point().point.y),
                    ]
                )
            )
            row_polygons_msg.polygons.append(row_polygon)
        self._rows_polygons_pub.publish(row_polygons_msg)

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
