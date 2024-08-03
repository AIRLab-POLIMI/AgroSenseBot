#!/usr/bin/python3

import os
import yaml

import rclpy
from rclpy.node import Node
from geometry_msgs.msg import TransformStamped

from tf2_ros.static_transform_broadcaster import StaticTransformBroadcaster
from tf_transformations import quaternion_from_euler


class ASBStaticTransformBroadcaster(Node):
    def __init__(self):
        super().__init__('static_transform_broadcaster')

        self.tf_static_broadcaster = StaticTransformBroadcaster(self)

        self.declare_parameter('transform_list_file_path')
        transform_list_file_path = os.path.expanduser(self.get_parameter('transform_list_file_path').get_parameter_value().string_value)
        self.get_logger().info(f"transform_list_file_path set to {transform_list_file_path}")

        try:
            with open(transform_list_file_path, 'r') as f:
                transform_list = yaml.unsafe_load(f)
        except FileNotFoundError:
            self.get_logger().fatal(f"File does not exist: {transform_list_file_path}")
            return

        if transform_list is None:
            self.get_logger().info(f"transform list is empty")
            return

        if not isinstance(transform_list, list):
            raise ValueError(f"the transform list is not a list in [{transform_list_file_path}]")

        for transform_dict in transform_list:
            if not isinstance(transform_dict, dict):
                raise ValueError(f"an element of the transform list is not a dict in [{transform_list_file_path}]")

            # rotation can be specified with a quaternion, euler angles, or not specified.
            # quaterion takes the precedence if both are specified.
            if 'qx' in transform_dict or 'qy' in transform_dict or 'qz' in transform_dict or 'qw' in transform_dict:
                q = list()
                q.append(transform_dict['qx'] if 'qx' in transform_dict else 0.0)
                q.append(transform_dict['qy'] if 'qy' in transform_dict else 0.0)
                q.append(transform_dict['qz'] if 'qz' in transform_dict else 0.0)
                q.append(transform_dict['qw'] if 'qw' in transform_dict else 0.0)
            elif 'roll' in transform_dict or 'pitch' in transform_dict or 'yaw' in transform_dict:
                q = quaternion_from_euler(transform_dict['roll'] if 'roll' in transform_dict else 0.0, transform_dict['pitch'] if 'pitch' in transform_dict else 0.0, transform_dict['yaw'] if 'yaw' in transform_dict else 0.0)
            else:
                q = [0.0, 0.0, 0.0, 1.0]  # no rotation

            t = TransformStamped()
            t.header.stamp = self.get_clock().now().to_msg()
            t.header.frame_id = transform_dict['frame_id']
            t.child_frame_id = transform_dict['child_frame_id']
            t.transform.translation.x = float(transform_dict['x'])
            t.transform.translation.y = float(transform_dict['y'])
            t.transform.translation.z = float(transform_dict['z']) if 'z' in transform_dict else 0.0
            t.transform.rotation.x, t.transform.rotation.y, t.transform.rotation.z, t.transform.rotation.w = q
            self.tf_static_broadcaster.sendTransform(t)


def main():
    rclpy.init()
    node = ASBStaticTransformBroadcaster()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass


if __name__ == "__main__":
    main()
