#! /usr/bin/python3

import os.path
from datetime import datetime
import ast

import rclpy
from rclpy.node import Node
from rclpy.time import Time, Duration
from rclpy.qos import QoSProfile, ReliabilityPolicy, DurabilityPolicy

from std_msgs.msg import Header
from asb_msgs.msg import SystemUsage, ProcessCpuUsage

BLUE = "\033[94m"


def get_node_name(process_msg: ProcessCpuUsage) -> str:
    cmdline = ast.literal_eval(process_msg.cmdline)
    cmdline_node_names = list(filter(lambda s: '__node:=' in s, cmdline))
    if len(cmdline_node_names) >= 1:
        return cmdline_node_names[0].replace('__node:=', '')
    else:
        return process_msg.name


class NodeComputerSystemUsageCounter:
    def __init__(self, node: Node, node_name: str):
        self.node: Node = node
        self.node_name: str = node_name
        self.counter_name: str = f"{node_name} counter"
        self.counter_name = f"{self.counter_name:<32}"

        self.first_heartbeat: Time | None = None
        self.first_proc_msg: ProcessCpuUsage | None = None
        self.first_proc_stamp: Time | None = None
        self.last_proc_msg: ProcessCpuUsage | None = None
        self.last_proc_stamp: Time | None = None
        self.counting: bool = False

        self.done: bool = False
        self.cpu_time_diff: float | None = None
        self.stamp_diff: float | None = None
        self.average_cpu_usage: float | None = None

    def heartbeat_callback(self, msg: Header):
        if self.first_heartbeat is None:
            self.first_heartbeat = Time.from_msg(msg.stamp)

    def system_usage_callback(self, msg: SystemUsage):

        msg_time = Time.from_msg(msg.stamp)
        heartbeat_received = self.first_heartbeat is not None

        node_proc_msg: ProcessCpuUsage | None = None

        proc_msg: ProcessCpuUsage
        for proc_msg in msg.processes:
            if get_node_name(proc_msg) == self.node_name:
                node_proc_msg = proc_msg
                self.node.get_logger().info(f"{self.counter_name} current cpu time total: {node_proc_msg.cpu_time_total}")
                break
        del proc_msg

        if not heartbeat_received:
            return

        if not self.counting and heartbeat_received and msg_time >= self.first_heartbeat + Duration(seconds=2):
            self.counting = True
            self.first_proc_msg = node_proc_msg
            self.first_proc_stamp = msg_time

        if self.counting:
            if msg_time <= self.first_heartbeat + Duration(seconds=13):
                self.node.get_logger().info(BLUE+f"{self.counter_name} counting cpu time total: {node_proc_msg.cpu_time_total}")
                self.last_proc_msg = node_proc_msg
                self.last_proc_stamp = msg_time
            else:
                self.cpu_time_diff = self.last_proc_msg.cpu_time_total - self.first_proc_msg.cpu_time_total
                self.stamp_diff = (self.last_proc_stamp - self.first_proc_stamp).nanoseconds / 1E9
                self.average_cpu_usage = self.cpu_time_diff / self.stamp_diff
                self.node.get_logger().info(f"{self.counter_name} average_cpu_time: {self.average_cpu_usage:.3f}     cpu_time_diff: {self.cpu_time_diff:.3f}   stamp_diff: {self.stamp_diff:.3f}")
                self.done = True


class ComputerSystemUsageLogger(Node):
    def __init__(self):
        super().__init__('node_computer_system_usage_logger')

        qos_reliable_transient_local_depth_10 = QoSProfile(
            reliability=ReliabilityPolicy.RELIABLE,
            durability=DurabilityPolicy.TRANSIENT_LOCAL,
            depth=10
        )
        qos_reliable_volatile_depth_1 = QoSProfile(
            reliability=ReliabilityPolicy.RELIABLE,
            durability=DurabilityPolicy.VOLATILE,
            depth=1
        )

        self.sub = self.create_subscription(SystemUsage, '/asb_computer_system_usage', self.system_usage_callback, qos_reliable_transient_local_depth_10)
        self.scan_heartbeat_sub = self.create_subscription(Header, '/scan_heartbeat_front', self.heartbeat_callback, qos_reliable_volatile_depth_1)

        self.node_names: list[str] = ["controller_server", "lidar_filter_front", "lidar_filter_rear"]
        self.counters = [NodeComputerSystemUsageCounter(self, node_name=n) for n in self.node_names]

        out_file_dir = os.path.expanduser("~/asb_logs/test_data/")
        filename_stamp = datetime.now().strftime("%Y-%m-%d__%H-%M-%S")
        self.out_file_path = os.path.join(out_file_dir, f"nodes_cpu_average_{filename_stamp}.csv")
        if not os.path.exists(out_file_dir):
            os.makedirs(out_file_dir)

    def heartbeat_callback(self, msg: Header):
        self.get_logger().info(f"received heartbeat")
        for counter in self.counters:
            counter.heartbeat_callback(msg)

    def system_usage_callback(self, msg: SystemUsage):
        all_done = True
        for counter in self.counters:
            counter.system_usage_callback(msg)
            all_done = all_done and counter.done

        if all_done:
            with open(self.out_file_path, 'w') as f:
                csv_header = ", ".join(map(lambda c: f"{c.node_name}_average_cpu_usage, {c.node_name}_cpu_time_diff, {c.node_name}_stamp_diff", self.counters)) + "\n"
                csv_data = ", ".join(map(lambda c: f"{c.average_cpu_usage}, {c.cpu_time_diff}, {c.stamp_diff}", self.counters)) + "\n"

                f.writelines(csv_header)
                f.writelines(csv_data)

            raise KeyboardInterrupt


def main(args=None):
    rclpy.init(args=args)
    node = ComputerSystemUsageLogger()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass


if __name__ == '__main__':
    main()
