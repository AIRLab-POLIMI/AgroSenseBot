#!/usr/bin/python3

import rclpy
from rclpy.node import Node
from geometry_msgs.msg import PointStamped, Point32, PolygonStamped
import numpy as np


class ClickedPointCirclePublisher(Node):

    def __init__(self):
        super().__init__('curvature_publisher')

        self.clicked_point_sub_ = self.create_subscription(PointStamped, '/clicked_point', self.clicked_point_callback, 10)
        self.clicked_point_curvature_pub_ = self.create_publisher(PolygonStamped, '/clicked_point_circle', 10)

        self.r = 2.0  # m

    def clicked_point_callback(self, clicked_point_msg: PointStamped):
        c_x, c_y, c_z = clicked_point_msg.point.x, clicked_point_msg.point.y, clicked_point_msg.point.z
        msg = PolygonStamped()
        msg.header = clicked_point_msg.header
        msg.polygon.points = [
            Point32(x=c_x, y=c_y, z=c_z)
        ] + [
            Point32(x=c_x + self.r * np.cos(2 * np.pi * t), y=c_y + self.r * np.sin(2 * np.pi * t), z=c_z)
            for t in np.linspace(0, 1, 100, endpoint=True)
        ]
        self.clicked_point_curvature_pub_.publish(msg)


def main(args=None):
    rclpy.init(args=args)
    node = ClickedPointCirclePublisher()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass


if __name__ == '__main__':
    main()
