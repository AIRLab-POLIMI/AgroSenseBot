#!/usr/bin/python3

import rclpy
from rclpy.node import Node
from rclpy.qos import QoSProfile, ReliabilityPolicy, DurabilityPolicy
from rclpy.time import Time

from nav_msgs.msg import OccupancyGrid
from geometry_msgs.msg import Quaternion, PolygonStamped, Point32
from std_msgs.msg import Header

from asb_msgs.msg import PolygonStampedArray

import numpy as np
import cv2
import tf2_ros
import tf2_geometry_msgs
from tf2_ros import TransformException


class PolygonMapPublisher(Node):
    def __init__(self):
        super().__init__('polygon_map_publisher')

        self.declare_parameter("fixed_frame", "map")
        self.fixed_frame = self.get_parameter("fixed_frame").get_parameter_value().string_value

        self.declare_parameter("publish_period", 1.0)
        self.publish_period = self.get_parameter("publish_period").get_parameter_value().double_value

        self.declare_parameter("publish_once", True)
        self.publish_once = self.get_parameter("publish_once").get_parameter_value().bool_value

        # QoS: Reliable + Transient Local
        qos_profile = QoSProfile(
            reliability=ReliabilityPolicy.RELIABLE,
            durability=DurabilityPolicy.TRANSIENT_LOCAL,
            depth=10,
        )

        # Map configuration
        self.map_resolution = 0.05  # meters/pixel
        self.map_width = int(200 / self.map_resolution)  # initialize the map to a square with length 200 m
        self.map_height = int(200 / self.map_resolution)

        # Runtime variables
        self.transformed_polygons: PolygonStampedArray | None = None
        self.published_once: bool = False

        # TF2
        self.tf_buffer = tf2_ros.Buffer()
        self.tf_listener = tf2_ros.TransformListener(self.tf_buffer, self)

        # Topics
        self.map_pub = self.create_publisher(OccupancyGrid, 'map', qos_profile)
        self.create_subscription(PolygonStampedArray, 'polygons_in', self.polygons_callback, qos_profile)

        self.timer = self.create_timer(self.publish_period, self.publish_map_timer_callback)

    def polygons_callback(self, msg: PolygonStampedArray):
        self.published_once = False  # if we receive new polygons, we publish once more (if self.publish_once is True, otherwise we publish anyway)

        x_abs_max: float = 0.0
        y_abs_max: float = 0.0

        try:
            transformed_polygons = PolygonStampedArray()
            for polygon_stamped in msg.polygons:
                transform = self.tf_buffer.lookup_transform(self.fixed_frame, polygon_stamped.header.frame_id, Time())
                polygon_stamped_t = tf2_geometry_msgs.do_transform_polygon_stamped(polygon_stamped, transform)
                transformed_polygons.polygons.append(polygon_stamped_t)

                for p in polygon_stamped_t.polygon.points:
                    x_abs_max = max(np.abs(p.x), x_abs_max)
                    y_abs_max = max(np.abs(p.y), y_abs_max)
                    self.get_logger().info(f"x_abs_max: {x_abs_max}, y_abs_max: {y_abs_max}")

            self.map_width = int(2 * (x_abs_max + 1.0) / self.map_resolution)  # once we receive the polygons, the map is resized to include them with a margin of 1 m
            self.map_height = int(2 * (y_abs_max + 1.0) / self.map_resolution)

            self.get_logger().info(f"Received and transformed {len(msg.polygons)} polygons")
            self.transformed_polygons = transformed_polygons

        except TransformException as ex:
            self.get_logger().warn(f'Transform failed: {ex}')


    def publish_map_timer_callback(self):
        if self.publish_once and self.published_once:
            return

        img = np.zeros((self.map_height, self.map_width), dtype=np.uint8)

        if self.transformed_polygons is None:
            self.get_logger().warn("Polygons not received yet.", throttle_duration_sec=5.0)
            img[:, :] = 100  # if there are no polygons, publish a completely occupied map for safety
        else:
            polygon_stamped_t: PolygonStamped
            for polygon_stamped_t in self.transformed_polygons.polygons:
                polygon_im = [self.world_to_image_coordinates(p) for p in polygon_stamped_t.polygon.points]
                polygon_im_np = np.array([polygon_im], dtype=np.int32)
                cv2.fillPoly(img, polygon_im_np, 100)

            self.published_once = True  # once we publish the map from the polygons, we don't publish anymore if self.publish_once is True

        msg = OccupancyGrid()
        msg.header = Header()
        msg.header.stamp = self.get_clock().now().to_msg()
        msg.header.frame_id = self.fixed_frame
        msg.info.resolution = self.map_resolution
        msg.info.width = self.map_width
        msg.info.height = self.map_height
        msg.info.origin.position.x = - self.map_width / 2 * self.map_resolution
        msg.info.origin.position.y = - self.map_height / 2 * self.map_resolution
        msg.info.origin.orientation = Quaternion(w=1.0)
        msg.data = img.astype(np.int8).flatten().tolist()

        self.map_pub.publish(msg)

    def world_to_image_coordinates(self, p: Point32):
        x_im = int(p.x / self.map_resolution + self.map_width / 2)
        y_im = int(p.y / self.map_resolution + self.map_height / 2)
        return x_im, y_im


def main(args=None):
    rclpy.init(args=args)
    node = PolygonMapPublisher()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass


if __name__ == '__main__':
    main()
