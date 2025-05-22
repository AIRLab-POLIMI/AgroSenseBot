#!/usr/bin/python3
import rclpy
from rclpy.node import Node
from rclpy.qos import QoSProfile, ReliabilityPolicy, DurabilityPolicy

from geometry_msgs.msg import Point32, PolygonStamped
from std_msgs.msg import Header
from asb_msgs.msg import PolygonStampedArray


class TestPolygonPublisher(Node):
    def __init__(self):
        super().__init__('test_polygon_publisher')

        qos_profile = QoSProfile(
            reliability=ReliabilityPolicy.RELIABLE,
            durability=DurabilityPolicy.TRANSIENT_LOCAL,
            depth=10
        )

        self.publisher = self.create_publisher(PolygonStampedArray, '/geofence_polygons', qos_profile)
        self.timer = self.create_timer(2.0, self.publish_polygons)

    def publish_polygons(self):
        msg = PolygonStampedArray()

        header = Header()
        header.stamp = self.get_clock().now().to_msg()
        header.frame_id = 'vineyard'

        # Create two rectangles as example polygons
        polygon1 = PolygonStamped()
        polygon1.header = header
        polygon1.polygon.points = [
            Point32(x=7.0, y=-5.0),
            Point32(x=14.0, y=5.0),
        ]

        polygon2 = PolygonStamped()
        polygon2.header = header
        polygon2.polygon.points = [
            Point32(x=7.0, y=-2.5),
            Point32(x=14.0, y=-2.5),
        ]

        msg.polygons = [polygon1, polygon2]

        self.publisher.publish(msg)
        self.get_logger().info("Published test polygons to /polygons")


def main(args=None):
    rclpy.init(args=args)
    node = TestPolygonPublisher()
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()
