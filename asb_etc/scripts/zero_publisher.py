#!/usr/bin/python3

import rclpy
from rclpy.node import Node

from std_msgs.msg import Float64


class CurvaturePublisher(Node):

    def __init__(self):
        super().__init__('zero_publisher')

        self.zero_pub_ = self.create_publisher(Float64, '/zero', 10)

        timer_period = 0.1  # seconds
        self.timer = self.create_timer(timer_period, self.timer_callback)

    def timer_callback(self):
        msg = Float64()
        self.zero_pub_.publish(msg)


def main(args=None):
    rclpy.init(args=args)
    node = CurvaturePublisher()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass


if __name__ == '__main__':
    main()
