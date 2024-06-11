#! /usr/bin/python3

import rclpy
from rclpy.node import Node
from geometry_msgs.msg import Twist
from asb_msgs.msg import ControlSystemState

from math import pi


class EffCmdVelPublisher(Node):

    def __init__(self):
        super().__init__('eff_cmd_vel_publisher')
        self.control_system_state_sub_ = self.create_subscription(
            ControlSystemState, '/asb_control_system_status_controller/control_system_state', self.control_system_state_callback, 10)
        self.eff_cmd_vel_pub_ = self.create_publisher(Twist, '/eff_cmd_vel', 10)

        self.wheel_base_ = 0.2  # in the real robot the baseline is 0.88 m, but the wheelbase is not used since the control is done with the PID
        self.wheel_radius_ = 0.151
        self.reduction_ratio_ = 40.61

    def control_system_state_callback(self, control_system_state_msg: ControlSystemState):
        v_ang_left = control_system_state_msg.left_motor_velocity * 2 * pi / 60
        v_ang_right = control_system_state_msg.right_motor_velocity * 2 * pi / 60

        msg = Twist()
        msg.linear.x = (v_ang_right + v_ang_left) * (self.wheel_radius_ / self.reduction_ratio_) / 2
        msg.angular.z = (v_ang_right - v_ang_left) * (self.wheel_radius_ / self.reduction_ratio_) / self.wheel_base_
        self.eff_cmd_vel_pub_.publish(msg)


def main(args=None):
    rclpy.init(args=args)

    node = EffCmdVelPublisher()
    rclpy.spin(node)


if __name__ == '__main__':
    main()
