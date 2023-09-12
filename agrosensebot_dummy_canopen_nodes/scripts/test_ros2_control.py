#! /usr/bin/python3

import random

import rclpy
from rclpy.node import Node
from rclpy.qos import QoSProfile

from std_msgs.msg import UInt8
from agrosensebot_canopen_bridge_msgs.msg import VCUState, MotorDrive, SpeedRef

int16_min = -32768
int16_max = 32767


class MinimalPublisher(Node):

    def __init__(self):
        super().__init__('test_node')

        self.alive_bit = 0
        self.int16_counter = 0

        qos = rclpy.qos.qos_profile_sensor_data

        # publishers to dummy
        self.vcu_state_pub = self.create_publisher(VCUState, '/dummy_test/test/vcu_state', qos_profile=qos)
        self.motor_drive_right_pub = self.create_publisher(MotorDrive, '/dummy_test/test/motor_drive_right', qos_profile=qos)
        self.motor_drive_left_pub = self.create_publisher(MotorDrive, '/dummy_test/test/motor_drive_left', qos_profile=qos)
        self.motor_drive_fan_pub = self.create_publisher(MotorDrive, '/dummy_test/test/motor_drive_fan', qos_profile=qos)

        # subscribers from dummy
        self.speed_ref_sub = self.create_subscription(SpeedRef, '/dummy_test/test/speed_ref', self.speed_ref_callback, qos_profile=qos)

        timer_period = 0.1  # seconds
        self.timer = self.create_timer(timer_period, self.timer_callback)

    def timer_callback(self):
        self.alive_bit += 1

        print()

        # The dummy node transmits TPDO1 of the dummy VCU every time it receives a VCUState message.
        # The VCU_is_alive bit is managed by the dummy node and the bit is flipped every time a VCUState message is
        # received.
        vcu_state_msg = VCUState()
        vcu_state_msg.stamp = self.get_clock().now().to_msg()
        vcu_state_msg.vcu_safety_status = True
        vcu_state_msg.control_mode = 0
        self.vcu_state_pub.publish(vcu_state_msg)
        self.get_logger().info(f"Publishing vcu_state {vcu_state_msg.vcu_safety_status, vcu_state_msg.control_mode}")

        motor_drive_right_msg = MotorDrive()
        motor_drive_right_msg.stamp = self.get_clock().now().to_msg()
        motor_drive_right_msg.controller_temperature = self.rnd_int16 * 0.1
        motor_drive_right_msg.motor_temperature = self.rnd_int16 * 0.1
        motor_drive_right_msg.motor_rpm = 100
        motor_drive_right_msg.battery_current_display = self.rnd_int16 * 0.1
        self.motor_drive_right_pub.publish(motor_drive_right_msg)
        self.get_logger().info(f"Publishing motor_drive_right {motor_drive_right_msg.controller_temperature, motor_drive_right_msg.motor_temperature, motor_drive_right_msg.motor_rpm, motor_drive_right_msg.battery_current_display}")

        motor_drive_left_msg = MotorDrive()
        motor_drive_left_msg.stamp = self.get_clock().now().to_msg()
        motor_drive_left_msg.controller_temperature = self.rnd_int16 * 0.1
        motor_drive_left_msg.motor_temperature = self.rnd_int16 * 0.1
        motor_drive_left_msg.motor_rpm = 60
        motor_drive_left_msg.battery_current_display = self.rnd_int16 * 0.1
        self.motor_drive_left_pub.publish(motor_drive_left_msg)
        self.get_logger().info(f"Publishing motor_drive_left {motor_drive_left_msg.controller_temperature, motor_drive_left_msg.motor_temperature, motor_drive_left_msg.motor_rpm, motor_drive_left_msg.battery_current_display}")

        motor_drive_fan_msg = MotorDrive()
        motor_drive_fan_msg.stamp = self.get_clock().now().to_msg()
        motor_drive_fan_msg.controller_temperature = self.rnd_int16 * 0.1
        motor_drive_fan_msg.motor_temperature = self.rnd_int16 * 0.1
        motor_drive_fan_msg.motor_rpm = self.rnd_int16
        motor_drive_fan_msg.battery_current_display = self.rnd_int16 * 0.1
        self.motor_drive_fan_pub.publish(motor_drive_fan_msg)
        self.get_logger().info(f"Publishing motor_drive_fan {motor_drive_fan_msg.controller_temperature, motor_drive_fan_msg.motor_temperature, motor_drive_fan_msg.motor_rpm, motor_drive_fan_msg.battery_current_display}")

    def vcu_state_callback(self, vcu_state_msg):
        self.get_logger().info(f"Received   vcu_state {vcu_state_msg.vcu_safety_status, vcu_state_msg.control_mode}")

    def speed_ref_callback(self, msg):
        self.get_logger().info(f"Received   speed_ref {msg.right_speed_ref, msg.left_speed_ref}")

    def motor_drive_right_callback(self, msg):
        self.get_logger().info(f"Received   motor_drive_right {msg.controller_temperature, msg.motor_temperature, msg.motor_rpm, msg.battery_current_display}")

    def motor_drive_left_callback(self, msg):
        self.get_logger().info(f"Received   motor_drive_left {msg.controller_temperature, msg.motor_temperature, msg.motor_rpm, msg.battery_current_display}")

    def motor_drive_fan_callback(self, msg):
        self.get_logger().info(f"Received   motor_drive_fan {msg.controller_temperature, msg.motor_temperature, msg.motor_rpm, msg.battery_current_display}")

    @property
    def rnd_int16(self):
        return random.randint(int16_min, int16_max)


def main(args=None):
    rclpy.init(args=args)

    minimal_publisher = MinimalPublisher()

    rclpy.spin(minimal_publisher)

    # Destroy the node explicitly
    # (optional - otherwise it will be done automatically
    # when the garbage collector destroys the node object)
    minimal_publisher.destroy_node()
    rclpy.shutdown()


if __name__ == '__main__':
    main()
