#!/usr/bin/python3
import sys

import numpy as np
import rclpy
from rclpy.node import Node

from numpy import pi
from geometry_msgs.msg import Twist, Vector3


class CurvaturePublisher(Node):

    def __init__(self):
        super().__init__('curvature_publisher')

        self.cmd_vel_pub_ = self.create_publisher(Twist, '/cmd_vel', 10)
        self.create_timer(0.01, self.timer_callback)

    def timer_callback(self):
        rpm = int(sys.argv[1])
        v = rpm/60*2*pi*0.151/40.61
        self.get_logger().info(f"RPM: {rpm} \t v: {v:.4f}")
        self.cmd_vel_pub_.publish(Twist(linear=Vector3(x=v)))


def main(args=None):
    rclpy.init(args=args)
    node = CurvaturePublisher()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass


if __name__ == '__main__':
    main()
