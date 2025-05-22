#!/usr/bin/python3

import rclpy
import yaml
from rclpy.node import Node
from rclpy.qos import QoSProfile, ReliabilityPolicy, DurabilityPolicy
from rclpy.time import Time

from std_msgs.msg import Header
from geometry_msgs.msg import PointStamped, Point32, PolygonStamped

import tf2_ros
import tf2_geometry_msgs
from tf2_ros import TransformException


class GeofenceMaker(Node):
    def __init__(self):
        super().__init__('geofence_maker')

        # parameters
        self.declare_parameter("fixed_frame", "map")
        self.fixed_frame = self.get_parameter("fixed_frame").get_parameter_value().string_value

        self.declare_parameter("geofence_file_path", rclpy.Parameter.Type.STRING)
        self.geofence_file_path = self.get_parameter("geofence_file_path").get_parameter_value().string_value

        # runtime variables
        self.geofence_polygon: PolygonStamped = PolygonStamped(
            header=Header(
                frame_id=self.fixed_frame,
                stamp=self.get_clock().now().to_msg()
            )
        )

        # TF2
        self.tf_buffer = tf2_ros.Buffer()
        self.tf_listener = tf2_ros.TransformListener(self.tf_buffer, self)

        # publishers and subscribers
        qos_reliable_transient_local = QoSProfile(
            reliability=ReliabilityPolicy.RELIABLE,
            durability=DurabilityPolicy.TRANSIENT_LOCAL,
            depth=10,
        )
        qos_reliable_volatile = QoSProfile(
            reliability=ReliabilityPolicy.RELIABLE,
            durability=DurabilityPolicy.VOLATILE,
            depth=1,
        )
        self.clicked_point_sub_ = self.create_subscription(PointStamped, '/clicked_point', self.clicked_point_callback, qos_reliable_volatile)
        self.geofence_polygon_pub_ = self.create_publisher(PolygonStamped, '/geofence_polygon', qos_reliable_transient_local)

    def clicked_point_callback(self, clicked_point_msg: PointStamped):
        try:
            transform = self.tf_buffer.lookup_transform(self.fixed_frame, clicked_point_msg.header.frame_id, Time())
            point_t = tf2_geometry_msgs.do_transform_point(clicked_point_msg, transform)
            self.geofence_polygon.polygon.points.append(Point32(x=point_t.point.x, y=point_t.point.y))

            self.geofence_polygon_pub_.publish(self.geofence_polygon)

            with open(self.geofence_file_path, 'w') as geofence_file:
                yaml.dump({
                    "fixed_frame": self.fixed_frame,
                    "polygon_points": list(map(lambda p: {"x": float(p.x), "y": float(p.y)}, self.geofence_polygon.polygon.points)),
                }, geofence_file)

        except TransformException as ex:
            self.get_logger().warn(f'Transform failed: {ex}')


def main(args=None):
    rclpy.init(args=args)
    node = GeofenceMaker()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass


if __name__ == '__main__':
    main()
