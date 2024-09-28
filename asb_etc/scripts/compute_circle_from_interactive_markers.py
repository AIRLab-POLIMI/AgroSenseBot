#!/usr/bin/env python3

import rclpy
from rclpy.node import Node

from geometry_msgs.msg import Point32, PolygonStamped, Point, Quaternion, Pose
import numpy as np

from geometry_msgs.msg import Vector3
from interactive_markers import InteractiveMarkerServer
from std_msgs.msg import ColorRGBA, Header
from tf_transformations import quaternion_from_euler
from visualization_msgs.msg import InteractiveMarker
from visualization_msgs.msg import InteractiveMarkerControl
from visualization_msgs.msg import Marker


class ComputeCircle(Node):

    def __init__(self):
        super().__init__('compute_circle_from_interactive_markers')

        self.frame_id = "map"
        self.cross_hair_resolution = 100  # number of points
        self.cross_hair_z_offset = 0.0  # m
        self.cross_hair_radius = 0.05
        self.cross_hair_line_width = 0.001  # m
        self.cross_hair_line_color = ColorRGBA(r=0.7, g=0.7, b=0.7, a=0.9)
        self.cross_hair_fill_color = ColorRGBA(r=1.0, g=1.0, b=1.0, a=0.1)

        self.text_height = 0.1
        self.text_color = ColorRGBA(r=0.7, g=0.7, b=0.7, a=1.0)

        self.circle_resolution = 300  # number of points
        self.circle_z_offset = 0.01  # m

        self.p_1 = Point()
        self.p_2 = Point()
        self.p_3 = Point()

        self.circle_pub_ = self.create_publisher(PolygonStamped, '/circle_viz', 10)
        self.text_pub_ = self.create_publisher(Marker, '/circle_text_viz', 10)

        self.server = InteractiveMarkerServer(self, 'interactive_markers_circle')

        cross_hair_points = [
            Point(x=self.cross_hair_radius, z=self.cross_hair_z_offset),
            Point(x=-self.cross_hair_radius, z=self.cross_hair_z_offset),
            Point(y=self.cross_hair_radius, z=self.cross_hair_z_offset),
            Point(y=-self.cross_hair_radius, z=self.cross_hair_z_offset),
        ]
        cross_hair_points += [
            Point(x=self.cross_hair_radius * np.cos(2 * np.pi * t), y=self.cross_hair_radius * np.sin(2 * np.pi * t), z=self.cross_hair_z_offset)
            for t in np.linspace(0, 1, self.cross_hair_resolution + self.cross_hair_resolution % 2, endpoint=True)
        ]

        cross_hair_marker = Marker(
            type=Marker.LINE_LIST,
            scale=Vector3(x=self.cross_hair_line_width),
            color=self.cross_hair_line_color,
            points=cross_hair_points,
        )

        q = quaternion_from_euler(0, np.pi/2, 0)

        def int_marker(n):
            return InteractiveMarker(
                header=Header(frame_id=self.frame_id),
                name=n,
                controls=[
                    InteractiveMarkerControl(
                        name="move",
                        orientation=Quaternion(x=q[0], y=q[1], z=q[2], w=q[3]),
                        orientation_mode=InteractiveMarkerControl.FIXED,
                        interaction_mode=InteractiveMarkerControl.MOVE_PLANE,
                        always_visible=True,
                        markers=[
                            cross_hair_marker,
                            Marker(
                                type=Marker.CYLINDER,
                                scale=Vector3(x=2*self.cross_hair_radius, y=2*self.cross_hair_radius, z=self.cross_hair_line_width),
                                color=self.cross_hair_fill_color,
                            )
                        ]
                    )
                ]
            )

        self.server.insert(
            feedback_callback=self.point_1_feedback,
            marker=int_marker("point_1"),
        )
        self.server.insert(
            feedback_callback=self.point_2_feedback,
            marker=int_marker("point_2"),
        )
        self.server.insert(
            feedback_callback=self.point_3_feedback,
            marker=int_marker("point_3"),
        )

        self.server.applyChanges()

    def point_1_feedback(self, feedback):
        self.p_1 = feedback.pose.position
        self.compute_circle()

    def point_2_feedback(self, feedback):
        self.p_2 = feedback.pose.position
        self.compute_circle()

    def point_3_feedback(self, feedback):
        self.p_3 = feedback.pose.position
        self.compute_circle()

    def compute_circle(self):

        if self.p_1 is not None and self.p_2 is not None and self.p_3 is not None:
            r = self.p_1.x + 1j*self.p_1.y
            s = self.p_2.x + 1j*self.p_2.y
            t = self.p_3.x + 1j*self.p_3.y

            try:
                w = t-r
                w /= s-r
                c = (r-s)*(w-abs(w)**2)/2j/w.imag-r
                c_x, c_y, c_z = -c.real, -c.imag, self.p_3.z
                radius = abs(c+r)
            except ZeroDivisionError:
                c_x, c_y, c_z = self.p_3.x, self.p_3.y, self.p_3.z
                radius = 0.0

            self.publish_circle(c_x, c_y, c_z+self.circle_z_offset, radius, Header(frame_id=self.frame_id, stamp=self.get_clock().now().to_msg()))

    def publish_circle(self, c_x, c_y, c_z, r, h):
        msg = PolygonStamped(header=h)
        msg.polygon.points = [
                                 Point32(x=c_x, y=c_y, z=c_z)
                             ] + [
                                 Point32(x=c_x + r * np.cos(2 * np.pi * t), y=c_y + r * np.sin(2 * np.pi * t), z=c_z)
                                 for t in np.linspace(0, 1, self.circle_resolution, endpoint=True)
                             ]
        self.circle_pub_.publish(msg)

        max_precision = 5
        min_precision = 1
        precision = max(min_precision, -int(max(-max_precision, np.floor(np.log(r)/np.log(10))-2)))
        radius_text = f"r={r:.{precision}f}"

        self.text_pub_.publish(Marker(
            header=Header(frame_id=self.frame_id, stamp=self.get_clock().now().to_msg()),
            ns="circle",
            pose=Pose(
                position=Point(x=c_x, y=c_y, z=c_z),
                orientation=Quaternion(w=1.0),
            ),
            type=Marker.TEXT_VIEW_FACING,
            scale=Vector3(z=self.text_height),
            color=self.text_color,
            text=radius_text,
        ))

        self.get_logger().info(f"radius: {radius_text} m, center: x={c_x:.1} y={c_y:.1} z={c_z:.1} [m], frame_id: {self.frame_id}")


def main(args=None):
    rclpy.init(args=args)
    node = ComputeCircle()
    try:
        rclpy.spin(node)
        node.server.shutdown()
    except KeyboardInterrupt:
        pass


if __name__ == '__main__':
    main()
