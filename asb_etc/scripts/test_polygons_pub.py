#!/usr/bin/python3
import rclpy
from rclpy.node import Node
from rclpy.qos import QoSProfile, ReliabilityPolicy, DurabilityPolicy

from geometry_msgs.msg import Polygon, Point32
from std_msgs.msg import Header
from asb_msgs.msg import PolygonArrayStamped


class TestPolygonPublisher(Node):
    def __init__(self):
        super().__init__('test_polygon_publisher')

        qos_profile = QoSProfile(
            reliability=ReliabilityPolicy.RELIABLE,
            durability=DurabilityPolicy.TRANSIENT_LOCAL,
            depth=10
        )

        self.publisher = self.create_publisher(PolygonArrayStamped, '/polygons', qos_profile)
        self.timer = self.create_timer(2.0, self.publish_polygons)

    def publish_polygons(self):
        msg = PolygonArrayStamped()
        msg.header = Header()
        msg.header.stamp = self.get_clock().now().to_msg()
        msg.header.frame_id = 'vineyard'

        # Create two rectangles as example polygons
        polygon1 = Polygon()
        polygon1.points = [
            Point32(x=0.0, y=0.0),
            Point32(x=1.0, y=0.0),
        ]

        polygon2 = Polygon()
        polygon2.points = [
            Point32(x=2.0, y=2.0),
            Point32(x=3.0, y=2.0),
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
