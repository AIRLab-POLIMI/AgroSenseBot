#! /usr/bin/python3
import math

import rclpy
from rclpy.duration import Duration
from rclpy.node import Node
from rclpy.qos import QoSProfile
from agrosensebot_canopen_bridge_msgs.msg import VCUState, MotorDrive, SpeedRef
from std_msgs.msg import UInt8
from geometry_msgs.msg import Twist


class CommandInterface(Node):

    def __init__(self):
        super().__init__('command_interface')

        qos = rclpy.qos.qos_profile_sensor_data

        self.dummy_test = False
        if self.dummy_test:
            namespace = '/dummy_test'
        else:
            namespace = ''

        # publishers to GCU
        self.gcu_alive_gcu_pub = self.create_publisher(UInt8, namespace + '/gcu_alive', qos_profile=qos)
        self.speed_ref_gcu_pub = self.create_publisher(SpeedRef, namespace + '/speed_ref', qos_profile=qos)

        # publishers to dummy
        if self.dummy_test:
            self.vcu_state_dummy_pub = self.create_publisher(VCUState, namespace + '/test/vcu_state', qos_profile=qos)

        # subscribers from GCU
        self.vcu_state_gcu_sub = self.create_subscription(VCUState, namespace + '/vcu_state', self.vcu_state_gcu_callback, qos_profile=qos)
        self.motor_drive_right_gcu_sub = self.create_subscription(MotorDrive, namespace + '/motor_drive_right', self.motor_drive_right_gcu_callback, qos_profile=qos)
        self.motor_drive_left_gcu_sub = self.create_subscription(MotorDrive, namespace + '/motor_drive_left', self.motor_drive_left_gcu_callback, qos_profile=qos)
        self.motor_drive_fan_gcu_sub = self.create_subscription(MotorDrive, namespace + '/motor_drive_fan', self.motor_drive_fan_gcu_callback, qos_profile=qos)

        # subscribers from dummy
        if self.dummy_test:
            self.speed_ref_dummy_sub = self.create_subscription(SpeedRef, namespace + '/test/speed_ref', self.speed_ref_dummy_callback, qos_profile=qos)

        # subscribers from teleop
        self.cmd_vel_sub = self.create_subscription(Twist, '/cmd_vel', self.cmd_vel_callback, qos_profile=qos)

        # kinematic state variables
        self.last_command = None
        self.last_command_stamp = None

        # comm state variables
        self.alive_bit = 0

        # vehicle parameters
        self.wheel_separation = 2.0  # TODO
        self.wheel_radius = 0.1  # TODO

        # other parameters
        self.cmd_vel_timeout = 0.5  # timeout after not receiving a cmd_vel message (using teleop_twist_keyboard, the messages can have as much as 0.48s delay between messages)

        comm_period = 0.05  # seconds
        self.timer = self.create_timer(comm_period, self.comm_timer_callback)

    def compute_kinematic(self):
        if self.last_command is None:
            self.get_logger().info(f"cmd_vel not received yet")

            speed_ref_msg = SpeedRef()
            speed_ref_msg.stamp = self.get_clock().now().to_msg()
            speed_ref_msg.left_speed_ref = 0
            speed_ref_msg.right_speed_ref = 0
            self.speed_ref_gcu_pub.publish(speed_ref_msg)
            self.get_logger().info(f"Publishing speed_ref  left_speed_ref: {0}  right_speed_ref: {0}")
            return

        now = self.get_clock().now()
        if now - self.last_command_stamp > Duration(seconds=self.cmd_vel_timeout):
            self.get_logger().info(f"cmd_vel timeout ({(now - self.last_command_stamp).nanoseconds / 1e9} s)")

            speed_ref_msg = SpeedRef()
            speed_ref_msg.stamp = self.get_clock().now().to_msg()
            speed_ref_msg.left_speed_ref = 0
            speed_ref_msg.right_speed_ref = 0
            self.speed_ref_gcu_pub.publish(speed_ref_msg)
            self.get_logger().info(f"Publishing speed_ref  left_speed_ref: {0}  right_speed_ref: {0}")
            return

        d_x = self.last_command.linear.x  # linear velocity
        d_a = self.last_command.angular.z  # angular velocity

        left_track_ang_vel = (d_x - d_a * self.wheel_separation / 2.0) / self.wheel_radius
        right_track_ang_vel = (d_x + d_a * self.wheel_separation / 2.0) / self.wheel_radius

        # convert angular velocity to RPM
        left_track_rpm = int(left_track_ang_vel * 60 / (2*math.pi))
        right_track_rpm = int(right_track_ang_vel * 60 / (2*math.pi))

        speed_ref_msg = SpeedRef()
        speed_ref_msg.stamp = self.get_clock().now().to_msg()
        speed_ref_msg.left_speed_ref = left_track_rpm  # TODO !!! speed ref is not rpm, it's a percentage !!!
        speed_ref_msg.right_speed_ref = right_track_rpm
        self.speed_ref_gcu_pub.publish(speed_ref_msg)
        self.get_logger().info(f"Publishing speed_ref  left_speed_ref: {left_track_rpm}  right_speed_ref: {right_track_rpm}")

    def comm_timer_callback(self):
        self.alive_bit += 1

        print()

        # Bit 0 of TPDO1 of the GCU has to change between 0 and 1 with a period between 20ms and 100ms.
        # This callback is called every 50ms and publishes a ROS2 message with the first bit flipped from the previous
        # call.
        gcu_alive_msg = UInt8()
        gcu_alive_msg.data = self.alive_bit % 2
        self.gcu_alive_gcu_pub.publish(gcu_alive_msg)
        self.get_logger().info(f"Publishing gcu_alive {gcu_alive_msg.data:#x}")

        # The dummy node transmits TPDO1 of the dummy VCU every time it receives a VCUState message.
        # The VCU_is_alive bit is managed by the dummy node and the bit is flipped every time a VCUState message is
        # received.
        if self.dummy_test:
            vcu_state_msg = VCUState()
            vcu_state_msg.stamp = self.get_clock().now().to_msg()
            vcu_state_msg.vcu_safety_status = True
            vcu_state_msg.control_mode = 0
            self.vcu_state_dummy_pub.publish(vcu_state_msg)
            self.get_logger().info(f"Publishing vcu_state {vcu_state_msg.vcu_safety_status, vcu_state_msg.control_mode}")

        self.compute_kinematic()

    def vcu_state_gcu_callback(self, vcu_state_msg):
        self.get_logger().info(f"GCU   Received   vcu_state {vcu_state_msg.vcu_safety_status, vcu_state_msg.control_mode}")

    def speed_ref_dummy_callback(self, msg):
        self.get_logger().info(f"Dummy Received   speed_ref {msg.right_speed_ref, msg.left_speed_ref}")

    def motor_drive_right_gcu_callback(self, msg):
        self.get_logger().info(f"GCU   Received   motor_drive_right {msg.controller_temperature, msg.motor_temperature, msg.motor_rpm, msg.battery_current_display}")

    def motor_drive_left_gcu_callback(self, msg):
        self.get_logger().info(f"GCU   Received   motor_drive_left {msg.controller_temperature, msg.motor_temperature, msg.motor_rpm, msg.battery_current_display}")

    def motor_drive_fan_gcu_callback(self, msg):
        self.get_logger().info(f"GCU   Received   motor_drive_fan {msg.controller_temperature, msg.motor_temperature, msg.motor_rpm, msg.battery_current_display}")

    def cmd_vel_callback(self, twist_msg):
        self.last_command = twist_msg
        self.last_command_stamp = self.get_clock().now()


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
