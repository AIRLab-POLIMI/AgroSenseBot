#! /usr/bin/python3

import rclpy
from geometry_msgs.msg import AccelStamped
from rclpy.node import Node
from rclpy.qos import QoSProfile, ReliabilityPolicy, DurabilityPolicy, HistoryPolicy
from sensor_msgs.msg import Imu

from tf2_ros import TransformListener, Buffer
import tf2_ros
import tf2_geometry_msgs
from geometry_msgs.msg import Vector3Stamped


class ImuAccelerationPublisher(Node):

    def __init__(self):
        super().__init__('imu_acceleration_publisher')

        self.sum_x = list()
        self.sum_y = list()
        self.sum_z = list()

        self.tf_buffer = Buffer()
        self.tf_listener = TransformListener(self.tf_buffer, self)

        self.pub = self.create_publisher(AccelStamped, "acceleration", 10)

        qos_reliable_volatile_depth_1 = QoSProfile(
            reliability=ReliabilityPolicy.RELIABLE,
            durability=DurabilityPolicy.VOLATILE,
            history=HistoryPolicy.KEEP_LAST,
            depth=1,
        )
        self.imu_sub = self.create_subscription(Imu, "imu", self.imu_callback, qos_profile=qos_reliable_volatile_depth_1)

    def imu_callback(self, imu_msg: Imu):

        accel_msg = AccelStamped()
        accel_msg.header = imu_msg.header
        accel_msg.accel.linear = imu_msg.linear_acceleration
        self.pub.publish(accel_msg)

        try:
            transform = self.tf_buffer.lookup_transform(
                'base_footprint',
                imu_msg.header.frame_id,
                imu_msg.header.stamp
            )

            lin_acc_stamped = Vector3Stamped()
            lin_acc_stamped.header = imu_msg.header
            lin_acc_stamped.vector = imu_msg.linear_acceleration
            transformed_linear_acceleration = tf2_geometry_msgs.do_transform_vector3(lin_acc_stamped, transform)

            # Update running sum and count
            self.sum_x.append(transformed_linear_acceleration.vector.x)
            self.sum_y.append(transformed_linear_acceleration.vector.y)
            self.sum_z.append(transformed_linear_acceleration.vector.z)

            n = 500
            # Compute running average
            avg_x = sum(self.sum_x[-n:-1]) / len(self.sum_x[-n:-1])
            avg_y = sum(self.sum_y[-n:-1]) / len(self.sum_y[-n:-1])
            avg_z = sum(self.sum_z[-n:-1]) / len(self.sum_z[-n:-1])

            self.get_logger().info(f"x: {avg_x:+.4f} y: {avg_y:+.4f} z: {avg_z:+.4f} n: {len(self.sum_x[-n:-1])}")

        except Exception as e:
            pass


def main(args=None):
    rclpy.init(args=args)
    node = ImuAccelerationPublisher()

    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass


if __name__ == '__main__':
    main()
