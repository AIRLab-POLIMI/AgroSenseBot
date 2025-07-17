#!/usr/bin/python3

import rclpy
from rclpy.node import Node
from rclpy.qos import QoSProfile, ReliabilityPolicy, DurabilityPolicy
from rclpy.time import Time

from nav_msgs.msg import OccupancyGrid
from geometry_msgs.msg import Quaternion, PolygonStamped, Point32
from std_msgs.msg import Header

from asb_msgs.msg import PolygonStampedArray

import yaml
import numpy as np
import cv2
import tf2_ros
import tf2_geometry_msgs
from tf2_ros import TransformException

OCCUPIED = 100
UNKNOWN = 255


class GeofenceMapServer(Node):
    def __init__(self):
        super().__init__('geofence_map_server')

        self.declare_parameter("map_resolution", rclpy.Parameter.Type.DOUBLE)  # [m/pixel]
        self.map_resolution = self.get_parameter("map_resolution").get_parameter_value().double_value

        self.declare_parameter("map_margin", rclpy.Parameter.Type.DOUBLE)  # [m]
        self.map_margin = self.get_parameter("map_margin").get_parameter_value().double_value

        self.declare_parameter("geofence_file_path", rclpy.Parameter.Type.STRING)
        self.geofence_file_path = self.get_parameter("geofence_file_path").get_parameter_value().string_value

        # load the geofence polygon
        with open(self.geofence_file_path, 'r') as geofence_file:
            geofence_dict = yaml.safe_load(geofence_file)
        self.fixed_frame: str = geofence_dict["fixed_frame"]
        geofence_polygon_points: list[Point32] = list()
        x_abs_max: float = 0.0
        y_abs_max: float = 0.0
        for p_dict in geofence_dict["polygon_points"]:
            p = Point32(x=p_dict["x"], y=p_dict["y"])
            geofence_polygon_points.append(p)
            x_abs_max = max(np.abs(p.x), x_abs_max)
            y_abs_max = max(np.abs(p.y), y_abs_max)

        # initialize the map as completely occupied
        self.map_width = int(2 * (x_abs_max + self.map_margin) / self.map_resolution)
        self.map_height = int(2 * (y_abs_max + self.map_margin) / self.map_resolution)
        self.img = np.full((self.map_height, self.map_width), OCCUPIED, dtype=np.uint8)

        # clear the occupancy inside the geofence
        geofence_polygon_points_im = list(map(self.world_to_image_coordinates, geofence_polygon_points))
        geofence_polygon_points_im_np = np.array([geofence_polygon_points_im], dtype=np.int32)
        cv2.fillPoly(self.img, geofence_polygon_points_im_np, UNKNOWN)

        # TF2
        self.tf_buffer = tf2_ros.Buffer()
        self.tf_listener = tf2_ros.TransformListener(self.tf_buffer, self)

        # Topics
        qos_profile = QoSProfile(
            reliability=ReliabilityPolicy.RELIABLE,
            durability=DurabilityPolicy.TRANSIENT_LOCAL,
            depth=10,
        )
        self.map_pub = self.create_publisher(OccupancyGrid, 'map', qos_profile)
        self.create_subscription(PolygonStampedArray, 'polygons_in', self.polygons_callback, qos_profile)

        self.publish_map_timer = self.create_timer(1.0, self.publish_map_timer_callback)

    def publish_map_timer_callback(self):
        self.publish_map()
        self.publish_map_timer.cancel()

    def polygons_callback(self, msg: PolygonStampedArray):
        attempt = 1
        while rclpy.ok():
            try:
                for polygon_stamped in msg.polygons:
                    # transform each polygon into the fixed frame
                    transform = self.tf_buffer.lookup_transform(self.fixed_frame, polygon_stamped.header.frame_id, Time())
                    polygon_stamped_t: PolygonStamped = tf2_geometry_msgs.do_transform_polygon_stamped(polygon_stamped, transform)

                    # mark the occupancy in the map for each polygon
                    polygon_im = list(map(self.world_to_image_coordinates, polygon_stamped_t.polygon.points))
                    polygon_im_np = np.array([polygon_im], dtype=np.int32)
                    cv2.fillPoly(self.img, polygon_im_np, OCCUPIED)

                self.get_logger().info(f"Received and added {len(msg.polygons)} polygons to the geofence map")
                self.publish_map()
                break

            except TransformException as ex:
                attempt += 1
                if attempt < 15:
                    self.get_logger().warn(f'Transform failed (attempt {attempt}), retrying: {ex}')
                else:
                    self.get_logger().error(f'Transform failed (attempt {attempt}): {ex}')
                    break

    def publish_map(self):
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
        msg.data = self.img.astype(np.int8).flatten().tolist()
        self.map_pub.publish(msg)

    def world_to_image_coordinates(self, p: Point32):
        x_im = int(p.x / self.map_resolution + self.map_width / 2)
        y_im = int(p.y / self.map_resolution + self.map_height / 2)
        return x_im, y_im


def main(args=None):
    rclpy.init(args=args)
    node = GeofenceMapServer()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass


if __name__ == '__main__':
    main()
