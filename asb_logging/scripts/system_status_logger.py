#! /usr/bin/python3

import rclpy
from rclpy.node import Node
from asb_msgs.msg import ControlSystemState

import os
from datetime import datetime
import pandas as pd


class SystemStateLogger(Node):

    def __init__(self):
        super().__init__('minimal_subscriber')
        self.subscription = self.create_subscription(
            ControlSystemState,
            '/asb_control_system_status_controller/control_system_state',
            self.listener_callback,
            10)

        self.df = None
        self.start_time = None

    def listener_callback(self, msg: ControlSystemState):
        t = msg.stamp.sec + msg.stamp.nanosec*1E-9

        if self.df is None:
            self.start_time = t
            self.df = pd.DataFrame(columns=[
                't',
                'dt',

                'bdi_percentage',
                'keyswitch_voltage',

                'left_motor_velocity_setpoint',
                'left_motor_velocity',
                'left_motor_position',
                'left_motor_battery_current',

                'right_motor_velocity_setpoint',
                'right_motor_velocity',
                'right_motor_position',
                'right_motor_battery_current',
            ])

        dt = t - self.start_time
        self.df.loc[len(self.df)] = [
            msg.stamp.sec + msg.stamp.nanosec*1E-9,
            dt,

            msg.left_motor_bdi_percentage,
            msg.left_motor_keyswitch_voltage,

            msg.left_motor_velocity_setpoint,
            msg.left_motor_velocity,
            msg.left_motor_position,
            msg.left_motor_battery_current,

            msg.right_motor_velocity_setpoint,
            msg.right_motor_velocity,
            msg.right_motor_position,
            msg.right_motor_battery_current,
        ]

    def save_data(self):
        now = datetime.now()
        datetime_str = now.strftime("%Y-%m-%d_%H-%M-%S.%f")
        csv_path = f"~/asb_logs/{datetime_str}.csv"
        print(f"saving data to {csv_path}")

        # os.makedirs(os.path.expanduser(os.path.dirname(csv_path)))
        self.df.to_csv(csv_path)


def main(args=None):
    rclpy.init(args=args)

    system_state_logger = SystemStateLogger()
    try:
        rclpy.spin(system_state_logger)
    except KeyboardInterrupt:
        system_state_logger.save_data()

    # Destroy the node explicitly
    # (optional - otherwise it will be done automatically
    # when the garbage collector destroys the node object)
    system_state_logger.destroy_node()
    # rclpy.shutdown()
    exit(0)


if __name__ == '__main__':
    main()
