#!/usr/bin/python3

import rclpy
from rclpy.node import Node

from visualization_msgs.msg import Marker


class TestSubscriber(Node):

    def __init__(self):
        super().__init__('test_subscriber')

        self.sub_ = self.create_subscription(Marker, '/expansions', self.callback, 10)

    def callback(self, msg):
        self.get_logger().info(f"len(msg.points): {len(msg.points)}")
        for p1, p2 in zip(msg.points[::2], msg.points[1::2]):
            self.get_logger().info(f"p1={p1.x:.2f}, {p1.y:.2f}  p2={p2.x:.2f}, {p2.y:.2f}")


def main(args=None):
    rclpy.init(args=args)
    node = TestSubscriber()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass


if __name__ == '__main__':
    main()
