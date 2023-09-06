#! /usr/bin/python3
import math

import rclpy
from rclpy.duration import Duration
from rclpy.node import Node
from rclpy.qos import QoSProfile, QoSReliabilityPolicy, QoSDurabilityPolicy
from agrosensebot_canopen_bridge_msgs.msg import VCUState, MotorDrive, SpeedRef
from geometry_msgs.msg import Twist


class CommandInterface(Node):

    def __init__(self):
        super().__init__('command_interface')

        qos = rclpy.qos.qos_profile_sensor_data

        # command subscriber
        self.cmd_vel_sub_ = self.create_subscription(Twist, '/cmd_vel', self.cmd_vel_callback, qos_profile=qos)

        # CANOpen bridge publisher
        self.speed_ref_pub_ = self.create_publisher(SpeedRef, '/speed_ref', qos_profile=qos)

        # CANOpen bridge subscribers  TODO
        # self.vcu_state_sub = self.create_subscription(VCUState, '/dummy_test/vcu_state', self.vcu_state_callback, qos_profile=qos)
        # self.motor_drive_right_sub = self.create_subscription(MotorDrive, '/dummy_test/motor_drive_right', self.motor_drive_right_callback, qos_profile=qos)
        # self.motor_drive_left_sub = self.create_subscription(MotorDrive, '/dummy_test/motor_drive_left', self.motor_drive_left_callback, qos_profile=qos)

        # state variables
        self.last_command_ = None
        self.last_command_stamp_ = None

        # vehicle parameters
        self.wheel_separation = 2.0  # TODO
        self.wheel_radius = 0.1  # TODO

        # other parameters
        self.timeout = 0.5  # timeout after not receiving a cmd_vel message (using teleop_twist_keyboard, the messages can have as much as 0.48s delay between messages)

        # main loop
        timer_period = 0.1  # seconds
        self.timer = self.create_timer(timer_period, self.timer_callback)

    def timer_callback(self):
        if self.last_command_ is None:
            self.get_logger().info(f"cmd_vel not received yet")
            return

        now = self.get_clock().now()
        if now - self.last_command_stamp_ > Duration(seconds=self.timeout):
            self.get_logger().info(f"cmd_vel timeout ({(now - self.last_command_stamp_).nanoseconds / 1e9} s)")
            return

        d_x = self.last_command_.linear.x  # linear velocity
        d_a = self.last_command_.angular.z  # angular velocity

        left_track_ang_vel = (d_x - d_a * self.wheel_separation / 2.0) / self.wheel_radius
        right_track_ang_vel = (d_x + d_a * self.wheel_separation / 2.0) / self.wheel_radius

        # convert angular velocity to RPM
        left_track_rpm = int(left_track_ang_vel * 60 / (2*math.pi))
        right_track_rpm = int(right_track_ang_vel * 60 / (2*math.pi))

        speed_ref_msg = SpeedRef()
        speed_ref_msg.stamp = self.get_clock().now().to_msg()
        speed_ref_msg.left_speed_ref = left_track_rpm  # TODO !!! speed ref is not rpm, it's percentage !!!
        speed_ref_msg.right_speed_ref = right_track_rpm
        self.speed_ref_pub_.publish(speed_ref_msg)
        self.get_logger().info(f"Publishing speed_ref  left_speed_ref: {left_track_rpm}  right_speed_ref: {right_track_rpm}")

    def cmd_vel_callback(self, twist_msg):
        self.last_command_ = twist_msg
        self.last_command_stamp_ = self.get_clock().now()


def main(args=None):
    rclpy.init(args=args)

    node = CommandInterface()

    rclpy.spin(node)

    # Destroy the node explicitly
    # (optional - otherwise it will be done automatically
    # when the garbage collector destroys the node object)
    node.destroy_node()
    rclpy.shutdown()


if __name__ == '__main__':
    main()
