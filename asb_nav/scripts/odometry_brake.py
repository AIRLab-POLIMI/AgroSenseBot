#! /usr/bin/python3

import rclpy
from rclpy.node import Node
from rclpy.qos import QoSProfile
from nav_msgs.msg import Odometry
from numpy import fabs


class OdometryBrakePublisher(Node):
    """
    Whenever the robot is not moving (linear and angular velocity from asb_track_controller are 0), publish an odometry
     message to the EKF filter with the last pose (position and orientation) to ensure the yaw estimation covariance of
     the EKF does not keep increasing. For some reason, asb_track_controller publishing a 0 angular velocity is not
     enough, even with 0 covariance.
    """

    def __init__(self):
        super().__init__('scan_test_publisher')
        self.print_debug_ = False
        self.shutting_down_ = False
        self.covariance_imu_ = 0.01
        self.covariance_gnss_ = 0.0001

        qos = rclpy.qos.qos_profile_sensor_data
        self.odom_sub = self.create_subscription(
            Odometry,
            '/odom',
            self.odom_callback, qos_profile=qos)

        self.odom_ekf_sub = self.create_subscription(
            Odometry,
            '/odometry/global',
            self.odom_ekf_callback, qos_profile=qos)

        self.odom_brake_pub_ = self.create_publisher(Odometry, "/odometry/brake", 10)

        self.last_ekf_odom_ = None

        self.get_logger().info("OdometryBrakePublisher initialized")

    def odom_ekf_callback(self, msg: Odometry):
        self.last_ekf_odom_ = msg.pose

    def odom_callback(self, msg: Odometry):
        if self.last_ekf_odom_ is not None and fabs(msg.twist.twist.linear.x) < 0.001 and fabs(msg.twist.twist.angular.z) < 0.001:  # TODO tune based on IMU
            odom_brake_msg = Odometry()
            odom_brake_msg.header.stamp = self.get_clock().now().to_msg()
            odom_brake_msg.header.frame_id = "map"
            odom_brake_msg.child_frame_id = "base_footprint"
            odom_brake_msg.pose = self.last_ekf_odom_
            self.odom_brake_pub_.publish(odom_brake_msg)


def main(args=None):
    rclpy.init(args=args)

    odometry_brake_publisher = OdometryBrakePublisher()

    try:
        rclpy.spin(odometry_brake_publisher)
    except KeyboardInterrupt:
        pass


if __name__ == '__main__':
    main()
