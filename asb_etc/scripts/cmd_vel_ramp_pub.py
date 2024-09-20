#!/usr/bin/python3
import sys
import yaml

import numpy as np
import rclpy
from rclpy.node import Node

from numpy import pi
from geometry_msgs.msg import Twist, Vector3


class LUTCmdVelPublisher(Node):

    def __init__(self):
        super().__init__('lut_cmd_vel_publisher')

        lin_vel_lut = {
            5.0: 0.0,
            10.0: 0.8,
            30.0: 0.8,
        }

        self.lin_vel_lut_keys = list(lin_vel_lut.keys())
        self.lin_vel_lut_values = list(lin_vel_lut.values())

        ang_vel_lut = {
            5.0: 0.0,
            10.0: 0.8/3.0,
            15.0: 0.8/2.0,
            20.0: 0.8/2.0,
            25.0: 0.8/1.5,
            30.0: 0.8/1.5,
        }

        self.ang_vel_lut_keys = list(ang_vel_lut.keys())
        self.ang_vel_lut_values = list(ang_vel_lut.values())

        self.t0 = self.get_clock().now()

        self.cmd_vel_pub_ = self.create_publisher(Twist, '/cmd_vel', 10)
        self.create_timer(0.01, self.timer_callback)

    def timer_callback(self):
        t = (self.get_clock().now() - self.t0).nanoseconds / 1E9
        lin_vel = np.interp(t, self.lin_vel_lut_keys, self.lin_vel_lut_values, left=0.0, right=0.0)
        ang_vel = np.interp(t, self.ang_vel_lut_keys, self.ang_vel_lut_values, left=0.0, right=0.0)
        self.get_logger().info(f"lin: {lin_vel:.4f} m/s \t ang: {ang_vel:.4f} rad/s")
        self.cmd_vel_pub_.publish(Twist(
            linear=Vector3(x=lin_vel),
            angular=Vector3(z=ang_vel),
        ))


def main(args=None):
    rclpy.init(args=args)
    node = LUTCmdVelPublisher()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass


if __name__ == '__main__':
    main()
