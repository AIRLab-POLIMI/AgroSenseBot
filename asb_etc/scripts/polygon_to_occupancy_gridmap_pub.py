#!/usr/bin/python3

import rclpy
from rclpy.node import Node
from rclpy.qos import QoSProfile, ReliabilityPolicy, DurabilityPolicy

from nav_msgs.msg import OccupancyGrid
from geometry_msgs.msg import Pose, Quaternion, Point32, PolygonStamped, PointStamped
from std_msgs.msg import Header

from asb_msgs.msg import PolygonArrayStamped

import numpy as np
import cv2
import tf2_ros
import tf2_geometry_msgs
from tf2_ros import TransformException


class PolygonMapPublisher(Node):
    def __init__(self):
        super().__init__('polygon_map_publisher')

        # Parameter: fixed_frame
        self.declare_parameter("fixed_frame", "map")
        self.fixed_frame = self.get_parameter("fixed_frame").get_parameter_value().string_value

        # QoS: Reliable + Transient Local
        qos_profile = QoSProfile(
            reliability=ReliabilityPolicy.RELIABLE,
            durability=DurabilityPolicy.TRANSIENT_LOCAL,
            depth=10
        )

        # Map configuration
        self.map_resolution = 0.05  # meters/pixel
        self.map_width = 2000
        self.map_height = 2000
        self.map_origin = (-15.0, -15.0)  # origin in meters

        # TF2
        self.tf_buffer = tf2_ros.Buffer()
        self.tf_listener = tf2_ros.TransformListener(self.tf_buffer, self)

        # Topics
        self.publisher = self.create_publisher(OccupancyGrid, '/rows_map', qos_profile)
        self.create_subscription(
            PolygonArrayStamped,
            '/polygons',
            self.polygons_callback,
            qos_profile
        )

        self.transformed_polygons = []  # list of list of (x, y)
        self.timer = self.create_timer(1.0, self.publish_map)

    def polygons_callback(self, msg: PolygonArrayStamped):
        try:
            transform = self.tf_buffer.lookup_transform(
                self.fixed_frame,
                msg.header.frame_id,
                rclpy.time.Time()
            )

            transformed_polygons = []
            for poly in msg.polygons:
                points = []
                for pt in poly.points:
                    p_stamped = PointStamped()
                    p_stamped.header = msg.header
                    p_stamped.point.x = pt.x
                    p_stamped.point.y = pt.y
                    p_stamped.point.z = pt.z

                    tf_point = tf2_geometry_msgs.do_transform_point(p_stamped, transform)
                    points.append((tf_point.point.x, tf_point.point.y))
                transformed_polygons.append(points)

            self.transformed_polygons = transformed_polygons
            self.get_logger().info(f"Received and transformed {len(transformed_polygons)} polygons")

        except TransformException as ex:
            self.get_logger().warn(f'Transform failed: {ex}')

    def publish_map(self):
        if not self.transformed_polygons:
            self.get_logger().warn("No polygons transformed yet.")
            return

        img = np.zeros((self.map_height, self.map_width), dtype=np.uint8)

        self.get_logger().info(f"polygons:")
        for polygon in self.transformed_polygons:
            self.get_logger().info(str(polygon))

            poly_pixels = [self.world_to_pixel(x, y) for x, y in polygon]
            poly_np = np.array([poly_pixels], dtype=np.int32)
            cv2.fillPoly(img, poly_np, 255)

        map_data = (img / 255.0 * 100).astype(np.int8).flatten().tolist()

        msg = OccupancyGrid()
        msg.header = Header()
        msg.header.stamp = self.get_clock().now().to_msg()
        msg.header.frame_id = self.fixed_frame

        msg.info.resolution = self.map_resolution
        msg.info.width = self.map_width
        msg.info.height = self.map_height
        msg.info.origin.position.x = self.map_origin[0]
        msg.info.origin.position.y = self.map_origin[1]
        msg.info.origin.orientation = Quaternion(w=1.0)
        msg.data = map_data

        self.publisher.publish(msg)
        self.get_logger().info("Published /rows_map")

    def world_to_pixel(self, x, y):
        px = int((x - self.map_origin[0]) / self.map_resolution)
        py = int((y - self.map_origin[1]) / self.map_resolution)
        return px, self.map_height - py


def main(args=None):
    rclpy.init(args=args)
    node = PolygonMapPublisher()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass


if __name__ == '__main__':
    main()
