#!/usr/bin/python3

import numpy as np
import rclpy
from rclpy.duration import Duration
from rclpy.node import Node

from nav_msgs.msg import Path
from geometry_msgs.msg import Point, PointStamped, PoseStamped, Pose, Quaternion
from std_msgs.msg import Float64
from tf2_py import TransformException
from tf2_ros import Buffer, TransformListener
import tf2_geometry_msgs  # imports PoseStamped into tf2


def closest_point_on_segment(a: Point, b: Point, p: Point) -> Point:
    v = Point(x=b.x - a.x, y=b.y - a.y)
    u = Point(x=a.x - p.x, y=a.y - p.y)
    t = np.clip(-(v.x * u.x + v.y * u.y) / (v.x ** 2 + v.y ** 2), 0.0, 1.0)
    return Point(x=(1 - t) * a.x + t * b.x, y=(1 - t) * a.y + t * b.y)


class GlobalPathDistancePublisher(Node):

    def __init__(self):
        super().__init__('global_path_distance_publisher')

        self.base_frame = 'base_footprint'
        self.global_path_msg = None

        self.global_path_sub_ = self.create_subscription(Path, '/plan', self.global_path_callback, 10)

        self.distance_pub_ = self.create_publisher(Float64, '/benchmarking/robot_global_path_distance', 10)
        self.closest_point_pub_ = self.create_publisher(PointStamped, '/benchmarking/closest_point_on_global_path', 10)
        self.distance_viz_pub_ = self.create_publisher(Path, '/benchmarking/robot_global_path_distance_viz', 10)

        self.tf_buffer = Buffer()
        self.tf_listener = TransformListener(self.tf_buffer, self)

        self.timer = self.create_timer(0.1, self.timer_callback)

    def global_path_callback(self, global_path_msg: Path):
        self.global_path_msg = global_path_msg

    def timer_callback(self):
        if self.global_path_msg is None or not len(self.global_path_msg.poses):
            return

        # transform the robot pose into the path frame
        r = PointStamped()
        r.header.frame_id = self.base_frame
        try:
            r = self.tf_buffer.transform(r, self.global_path_msg.header.frame_id, timeout=Duration(seconds=0.1))
        except TransformException as ex:
            self.get_logger().info(f'Could not transform {self.base_frame} to {self.global_path_msg.header.frame_id}: {ex}')
            return

        # find the closest point of the path to the robot position
        closest_point = PointStamped(header=self.global_path_msg.header)
        min_distance_sq = np.inf
        p1: PoseStamped
        p2: PoseStamped
        for p1, p2 in zip(self.global_path_msg.poses[0:-1], self.global_path_msg.poses[1:]):
            c = closest_point_on_segment(p1.pose.position, p2.pose.position, r.point)
            d_sq = (c.x - r.point.x) ** 2 + (c.y - r.point.y) ** 2
            if d_sq < min_distance_sq:
                min_distance_sq = d_sq
                closest_point.point = c

        self.distance_pub_.publish(Float64(data=np.sqrt(min_distance_sq)))
        self.closest_point_pub_.publish(closest_point)
        distance_viz_msg = Path(header=self.global_path_msg.header, poses=[
            PoseStamped(
                header=self.global_path_msg.header,
                pose=Pose(position=closest_point.point, orientation=Quaternion(w=1.0)),
            ),
            PoseStamped(
                header=self.global_path_msg.header,
                pose=Pose(position=r.point, orientation=Quaternion(w=1.0)),
            ),
        ])
        self.distance_viz_pub_.publish(distance_viz_msg)


def main(args=None):
    rclpy.init(args=args)
    node = GlobalPathDistancePublisher()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass


if __name__ == '__main__':
    main()
