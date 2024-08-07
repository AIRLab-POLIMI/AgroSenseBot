#!/usr/bin/python3
import numpy as np
import rclpy
from rclpy.node import Node

from nav_msgs.msg import Odometry
from geometry_msgs.msg import Twist
from std_msgs.msg import Float64


class CurvaturePublisher(Node):

    def __init__(self):
        super().__init__('curvature_publisher')

        self.cmd_vel_sub_ = self.create_subscription(Twist, '/cmd_vel', self.cmd_vel_callback, 10)
        self.odom_sub_ = self.create_subscription(Odometry, '/odom', self.odom_callback, 10)

        self.cmd_vel_curvature_pub_ = self.create_publisher(Float64, '/cmd_vel_curvature', 10)
        self.odom_curvature_pub_ = self.create_publisher(Float64, '/odom_curvature', 10)

    def cmd_vel_callback(self, cmd_vel_msg: Twist):
        msg = Float64()
        msg.data = cmd_vel_msg.angular.z / cmd_vel_msg.linear.x if cmd_vel_msg.linear.x != 0 else 0.0
        self.cmd_vel_curvature_pub_.publish(msg)

    def odom_callback(self, odom_msg: Odometry):
        thr = 0.001
        if np.abs(odom_msg.twist.twist.linear.x) > thr:
            self.odom_curvature_pub_.publish(Float64(data=odom_msg.twist.twist.angular.z / odom_msg.twist.twist.linear.x))
        else:
            if np.abs(odom_msg.twist.twist.angular.z) < thr:
                self.odom_curvature_pub_.publish(Float64(data=0.0))


def main(args=None):
    rclpy.init(args=args)
    node = CurvaturePublisher()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass


if __name__ == '__main__':
    main()
