#!/usr/bin/python3

import rclpy
from rclpy.node import Node
from geometry_msgs.msg import PointStamped, Point32, PolygonStamped
import numpy as np


class ComputeCircle(Node):

    def __init__(self):
        super().__init__('compute_circle_from_clicked_points')

        self.clicked_point_sub_ = self.create_subscription(PointStamped, '/clicked_point', self.clicked_point_callback, 10)
        self.clicked_point_circle_pub_ = self.create_publisher(PolygonStamped, '/circle_viz', 10)

        self.points: list[PointStamped] = list()

    def clicked_point_callback(self, clicked_point_msg: PointStamped):
        if len(self.points) < 3:
            self.points.append(clicked_point_msg)

        if len(self.points) == 3:
            p_1, p_2, p_3 = self.points[0].point, self.points[1].point, self.points[2].point
            r = p_1.x + 1j*p_1.y
            s = p_2.x + 1j*p_2.y
            t = p_3.x + 1j*p_3.y

            try:
                w = t-r
                w /= s-r
                c = (r-s)*(w-abs(w)**2)/2j/w.imag-r
                c_x, c_y, c_z = -c.real, -c.imag, p_3.z
                radius = abs(c+r)
            except ZeroDivisionError:
                c_x, c_y, c_z = p_3.x, p_3.y, p_3.z
                radius = 0.0

            self.publish_circle(c_x, c_y, c_z, radius, clicked_point_msg.header)
            self.get_logger().info(f"radius: {radius}, center: {c_x, c_y, c_z}, frame_id={clicked_point_msg.header.frame_id}")
            self.points = list()

    def publish_circle(self, c_x, c_y, c_z, r, h):
        msg = PolygonStamped(header=h)
        msg.polygon.points = [
            Point32(x=c_x, y=c_y, z=c_z)
        ] + [
            Point32(x=c_x + r * np.cos(2 * np.pi * t), y=c_y + r * np.sin(2 * np.pi * t), z=c_z)
            for t in np.linspace(0, 1, 100, endpoint=True)
        ]
        self.clicked_point_circle_pub_.publish(msg)


def main(args=None):
    rclpy.init(args=args)
    node = ComputeCircle()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass


if __name__ == '__main__':
    main()
