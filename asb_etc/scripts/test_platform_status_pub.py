#!/usr/bin/python3

import rclpy
from asb_msgs.msg import PlatformState
from rclpy.qos import QoSProfile, ReliabilityPolicy, DurabilityPolicy, HistoryPolicy
from rclpy.node import Node
from datetime import datetime


class PlatformStatePublisher(Node):

    def __init__(self):
        super().__init__('platform_state_publisher')

        publish_rate: float = 100.0

        qos_reliable_volatile_depth_1 = QoSProfile(
            reliability=ReliabilityPolicy.RELIABLE,
            durability=DurabilityPolicy.VOLATILE,
            history=HistoryPolicy.KEEP_LAST,
            depth=1,
        )
        self.pub = self.create_publisher(
            PlatformState,
            '/asb_platform_controller/platform_state',
            qos_profile=qos_reliable_volatile_depth_1,
        )
        self.create_timer(1.0/publish_rate, self.timer_callback)
        self.t_0 = datetime.now()

    def timer_callback(self):
        t = datetime.now() - self.t_0
        i = int(t.seconds)
        error_list = list(range(0, 16)) + list(range(92, 100))

        msg = PlatformState(
            stamp=self.get_clock().now().to_msg(),

            vcu_comm_ok=True,
            vcu_comm_started=True,
            gcu_comm_started=True,
            gcu_alive_bit_rate_low=bool(i % 2),
            gcu_alive_bit_rate_critical=False,
            vcu_safety_status=True,
            control_mode=i % 4,
            # control_mode values:
            # 0 = STOP: vehicle stopped;
            # 1 = RCU: vehicle controlled by RCU;
            # 2 = GCU: vehicle controlled by GCU;
            # 3 = WAIT: waiting to receive commands to transition from 2 to 1;

            more_recent_alarm_id_to_confirm=error_list[i % len(error_list)],
            more_recent_active_alarm_id=error_list[i % len(error_list)],
            software_emergency_stop=bool(i % 2),
            pump_state=bool(i % 2),

            left_motor_controller_temperature=30.0,  # °C
            left_motor_temperature=40.0,  # °C
            left_motor_battery_current=50.0,  # A
            left_motor_torque=0.0,  # Nm
            left_motor_bdi_percentage=60,  # %, battery state of charge
            left_motor_keyswitch_voltage=50.0,  # V, battery voltage
            left_motor_zero_speed_threshold=5,  # RPM, the speed below which the EM brake is set

            left_motor_position=1000.0,  # rad
            left_motor_velocity=90.0,  # rad/s
            left_motor_velocity_setpoint=180.0,  # rad/s

            right_motor_controller_temperature=30.0,  # °C
            right_motor_temperature=40.0,  # °C
            right_motor_battery_current=50.0,  # A
            right_motor_torque=0.0,  # Nm
            right_motor_bdi_percentage=60,  # %, battery state of charge
            right_motor_keyswitch_voltage=50.0,  # V, battery voltage
            right_motor_zero_speed_threshold=5,  # RPM, the speed below which the EM brake is set

            right_motor_position=1000.0,  # rad
            right_motor_velocity=90.0,  # rad/s
            right_motor_velocity_setpoint=180.0,  # rad/s

            fan_motor_controller_temperature=30.0,  # °C
            fan_motor_temperature=40.0,  # °C
            fan_motor_battery_current=10.0,  # A
            fan_motor_torque=0.0,  # Nm
            fan_motor_bdi_percentage=60,  # %, battery state of charge
            fan_motor_keyswitch_voltage=50.0,  # V, battery voltage
            fan_motor_zero_speed_threshold=5,  # RPM, the speed below which the EM brake is set

            fan_motor_position=1000.0,  # revolutions
            fan_motor_velocity_rpm=90.0,  # RPM
            fan_motor_velocity_setpoint_rpm=180.0,  # RPM
        )
        self.pub.publish(msg)


def main(args=None):
    rclpy.init(args=args)
    node = PlatformStatePublisher()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass


if __name__ == '__main__':
    main()
