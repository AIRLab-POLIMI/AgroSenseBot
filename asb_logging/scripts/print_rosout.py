#!/usr/bin/python3
from datetime import datetime

import rclpy
from rcl_interfaces.msg import Log
from rclpy.node import Node
from rclpy.qos import QoSProfile, ReliabilityPolicy, DurabilityPolicy, HistoryPolicy


def level_bytes_to_int(l: bytes):
    return int.from_bytes(l, byteorder='big')


color_codes_name = {
    level_bytes_to_int(Log.DEBUG): "DEBUG",
    level_bytes_to_int(Log.INFO): "INFO",
    level_bytes_to_int(Log.WARN): "WARN",
    level_bytes_to_int(Log.ERROR): "ERROR",
    level_bytes_to_int(Log.FATAL): "FATAL",
}

color_codes = {
    level_bytes_to_int(Log.DEBUG): "\033[94m",   # Blue
    level_bytes_to_int(Log.INFO): "\033[0m",     # Normal
    level_bytes_to_int(Log.WARN): "\033[93m",    # Yellow
    level_bytes_to_int(Log.ERROR): "\033[91m",   # Red
    level_bytes_to_int(Log.FATAL): "\033[95m",   # Magenta
}


def print_colored(log: Log) -> None:
    if isinstance(log.level, bytes):
        level_int = level_bytes_to_int(log.level)
    else:
        level_int = log.level

    color_code = color_codes.get(level_int, "\033[0m")
    reset_code = "\033[0m"

    print(f"{color_code}[{log.name:<30} {color_codes_name[level_int]}] {log.msg}{reset_code}")


def test(level, level_name):
    # Create and print a log message for each level
    log_msg = Log()
    log_msg.level = level
    log_msg.name = f"test_node_{level_name.lower()}"
    log_msg.msg = f"TEST {level_name} message"
    log_msg.stamp.sec = int(datetime.now().timestamp())
    log_msg.stamp.nanosec = 0
    print_colored(log_msg)


test(Log.INFO, "INFO")
test(Log.WARN, "WARN")
test(Log.ERROR, "ERROR")
test(Log.FATAL, "FATAL")


class PrintRosOut(Node):

    def __init__(self):
        super().__init__('print_rosout')

        qos_reliable_volatile_depth_1 = QoSProfile(
            reliability=ReliabilityPolicy.RELIABLE,
            durability=DurabilityPolicy.VOLATILE,
            history=HistoryPolicy.KEEP_LAST,
            depth=1,
        )
        self.sub_ = self.create_subscription(Log, '/rosout', self.callback, qos_reliable_volatile_depth_1)

    def callback(self, log: Log):
        if log.name in [self.get_name(), "rviz", "rosbag2_player"]:
            return

        print_colored(log)


def main(args=None):
    rclpy.init(args=args)
    node = PrintRosOut()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass


if __name__ == '__main__':
    main()
