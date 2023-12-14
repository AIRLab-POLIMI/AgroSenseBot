#! /usr/bin/python3

import rclpy
from rclpy.node import Node
from sensor_msgs.msg import LaserScan

from math import pi
import numpy as np


class ScanTestPublisher(Node):

    def __init__(self):
        super().__init__('scan_test_publisher')
        self.scan_front_pub_ = self.create_publisher(LaserScan, '/scan_front', 10)
        self.scan_rear_pub_ = self.create_publisher(LaserScan, '/scan_rear', 10)
        timer_period = 0.1  # seconds
        self.timer = self.create_timer(timer_period, self.timer_callback)

    def timer_callback(self):

        msg = LaserScan()
        msg.header.stamp = self.get_clock().now().to_msg()
        msg.header.frame_id = "base_scan_front"
        msg.angle_min = 0.
        msg.angle_max = pi
        msg.angle_increment = 2*pi/300
        msg.range_max = 100.
        msg.ranges = [np.inf]*300

        self.scan_front_pub_.publish(msg)
        self.scan_rear_pub_.publish(msg)
        self.get_logger().info('Publishing: "%s"' % msg)


def main(args=None):
    rclpy.init(args=args)

    scan_test_publisher = ScanTestPublisher()

    rclpy.spin(scan_test_publisher)

    # Destroy the node explicitly
    # (optional - otherwise it will be done automatically
    # when the garbage collector destroys the node object)
    scan_test_publisher.destroy_node()
    rclpy.shutdown()


if __name__ == '__main__':
    main()
