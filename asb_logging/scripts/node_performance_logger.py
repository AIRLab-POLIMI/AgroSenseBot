#! /usr/bin/python3

import os.path
from collections import namedtuple
from datetime import datetime
import ast

import rclpy
from rclpy.node import Node
from rclpy.time import Time, Duration
from rclpy.qos import QoSProfile, ReliabilityPolicy, DurabilityPolicy

from std_msgs.msg import Header
from asb_msgs.msg import SystemUsage, ProcessCpuUsage, DurationStamped

BLUE = "\033[94m"

Result = namedtuple('Result', 'csv_header csv_data')


def get_node_name(process_msg: ProcessCpuUsage) -> str:
    cmdline = ast.literal_eval(process_msg.cmdline)
    cmdline_node_names = list(filter(lambda s: '__node:=' in s, cmdline))
    if len(cmdline_node_names) >= 1:
        return cmdline_node_names[0].replace('__node:=', '')
    else:
        return process_msg.name


class PerformanceCounter:

    def __init__(self, node: Node):
        self._node: Node = node

    def set_measurement_start_time(self, start_time: Time) -> None:
        pass

    def set_measurement_stop_time(self, stop_time: Time) -> None:
        pass

    def can_start(self) -> bool:
        pass

    def is_done(self) -> bool:
        pass

    def get_result(self) -> Result:
        pass


class NodeComputerSystemUsageCounter(PerformanceCounter):
    def __init__(self, node: Node, node_name: str):
        super().__init__(node=node)

        self._node_name: str = node_name
        self._counter_name: str = f"{node_name} CPU usage counter"
        self._counter_name = f"{self._counter_name:<32}"

        self._measurement_start_time: Time | None = None
        self._measurement_stop_time: Time | None = None
        self._precondition: bool = False
        self._done: bool = False
        self._cpu_time_diff: float | None = None
        self._stamp_diff: float | None = None
        self._average_cpu_usage: float | None = None

        self._first_proc_msg: ProcessCpuUsage | None = None
        self._first_proc_stamp: Time | None = None
        self._last_proc_msg: ProcessCpuUsage | None = None
        self._last_proc_stamp: Time | None = None

        qos_reliable_volatile_depth_1 = QoSProfile(
            reliability=ReliabilityPolicy.RELIABLE,
            durability=DurabilityPolicy.VOLATILE,
            depth=1
        )
        qos_reliable_transient_local_depth_10 = QoSProfile(
            reliability=ReliabilityPolicy.RELIABLE,
            durability=DurabilityPolicy.TRANSIENT_LOCAL,
            depth=10
        )
        self._scan_heartbeat_sub = self._node.create_subscription(Header, '/scan_heartbeat_front', self._heartbeat_callback, qos_reliable_volatile_depth_1)
        self._computer_system_usage_sub = self._node.create_subscription(SystemUsage, '/asb_computer_system_usage', self._system_usage_callback, qos_reliable_transient_local_depth_10)

    def set_measurement_start_time(self, start_time: Time):
        if self._measurement_start_time is None:
            self._measurement_start_time = start_time

    def set_measurement_stop_time(self, stop_time: Time):
        if self._measurement_stop_time is None:
            self._measurement_stop_time = stop_time

    def can_start(self):
        return self._precondition

    def is_done(self):
        return self._done

    def get_result(self):
        return Result(
            csv_header=f"{self._node_name}_average_cpu_usage, {self._node_name}_cpu_time_diff, {self._node_name}_stamp_diff",
            csv_data=f"{self._average_cpu_usage}, {self._cpu_time_diff}, {self._stamp_diff}",
        )

    def _heartbeat_callback(self, _):
        self._precondition = True

    def _system_usage_callback(self, msg: SystemUsage):
        if self._measurement_start_time is None:
            return

        node_proc_msg: ProcessCpuUsage | None = None
        proc_msg: ProcessCpuUsage
        for proc_msg in msg.processes:
            if get_node_name(proc_msg) == self._node_name:
                node_proc_msg = proc_msg
                break
        del proc_msg

        msg_time = Time.from_msg(msg.stamp)
        if msg_time >= self._measurement_start_time:
            if self._first_proc_msg is None:
                self._first_proc_msg = node_proc_msg
                self._first_proc_stamp = msg_time

            if self._measurement_stop_time is None or msg_time <= self._measurement_stop_time:
                self._node.get_logger().info(BLUE + f"{self._counter_name} counting cpu time total: {node_proc_msg.cpu_time_total:.3f}")
                self._last_proc_msg = node_proc_msg
                self._last_proc_stamp = msg_time
            else:
                if not self._done:

                    self._cpu_time_diff = self._last_proc_msg.cpu_time_total - self._first_proc_msg.cpu_time_total
                    self._stamp_diff = (self._last_proc_stamp - self._first_proc_stamp).nanoseconds / 1E9
                    self._average_cpu_usage = self._cpu_time_diff / self._stamp_diff

                    self._node.get_logger().info(
                        f"* {self._counter_name}\n"
                        f"    average_cpu_usage: {self._average_cpu_usage:.3f}\n"
                        f"    cpu_time_diff: {self._cpu_time_diff:.3f}\n"
                        f"    stamp_diff: {self._stamp_diff:.3f}"
                    )

                    self._done = True


class NodeExecutionTimeCounter(PerformanceCounter):
    def __init__(self, node: Node, node_name: str):
        super().__init__(node=node)

        self._node_name: str = node_name
        self._counter_name: str = f"{node_name} execution duration counter"
        self._counter_name = f"{self._counter_name:<32}"

        self._measurement_start_time: Time | None = None
        self._measurement_stop_time: Time | None = None
        self._precondition: bool = False
        self._done: bool = False

        self._measurements: list[DurationStamped] = list()

        qos_reliable_volatile_depth_1 = QoSProfile(
            reliability=ReliabilityPolicy.RELIABLE,
            durability=DurabilityPolicy.VOLATILE,
            depth=1
        )
        qos_reliable_transient_local_depth_10 = QoSProfile(
            reliability=ReliabilityPolicy.RELIABLE,
            durability=DurabilityPolicy.TRANSIENT_LOCAL,
            depth=10
        )
        self._scan_heartbeat_sub = self._node.create_subscription(Header, "/scan_heartbeat_front", self._heartbeat_callback, qos_reliable_volatile_depth_1)
        self._computer_system_usage_sub = self._node.create_subscription(DurationStamped, f"{self._node_name}/benchmarking/execution_duration", self._execution_duration_callback, qos_reliable_transient_local_depth_10)

    def set_measurement_start_time(self, start_time: Time):
        if self._measurement_start_time is None:
            self._measurement_start_time = start_time

    def set_measurement_stop_time(self, stop_time: Time):
        if self._measurement_stop_time is None:
            self._measurement_stop_time = stop_time

    def can_start(self):
        return self._precondition

    def is_done(self):
        return self._done

    def get_result(self):
        return Result(
            csv_header=f"{self._node_name}_average_execution_duration, {self._node_name}_min_execution_duration, {self._node_name}_max_execution_duration, {self._node_name}_stamp_diff",
            csv_data=f"{self._average_execution_duration}, {self._min_execution_duration}, {self._max_execution_duration}, {self._stamp_diff}",
        )

    def _heartbeat_callback(self, _):
        self._precondition = True

    def _execution_duration_callback(self, msg: DurationStamped):
        if self._measurement_start_time is None:
            return

        msg_time = Time.from_msg(msg.stamp)

        if self._measurement_start_time <= msg_time <= self._measurement_stop_time:
            self._measurements.append(msg)
            self._node.get_logger().info(BLUE + f"{self._counter_name} execution duration: {Duration.from_msg(msg.duration).nanoseconds / 1E6:.3f} ms", throttle_duration_sec=1.0)

        if msg_time >= self._measurement_stop_time:
            if not self._done:
                self._stamp_diff: float = (Time.from_msg(self._measurements[-1].stamp) - Time.from_msg(self._measurements[0].stamp)).nanoseconds / 1E9
                self._average_execution_duration: float = sum(map(lambda m: Duration.from_msg(m.duration).nanoseconds / 1E9, self._measurements))/len(self._measurements)
                self._min_execution_duration: float = min(map(lambda m: Duration.from_msg(m.duration).nanoseconds / 1E9, self._measurements))
                self._max_execution_duration: float = max(map(lambda m: Duration.from_msg(m.duration).nanoseconds / 1E9, self._measurements))

                self._node.get_logger().info(
                    f"* {self._counter_name}\n"
                    f"    average_execution_duration: {self._average_execution_duration*1E3:.3f} ms\n"
                    f"    min_execution_duration: {self._min_execution_duration*1E3:.3f} ms\n"
                    f"    max_execution_duration: {self._max_execution_duration*1E3:.3f} ms\n"
                    f"    stamp_diff: {self._stamp_diff:.3f} s"
                )

                self._done = True


class PerformanceLogger(Node):
    def __init__(self):
        super().__init__('node_computer_system_usage_logger')

        self.counters: list[PerformanceCounter] = [NodeComputerSystemUsageCounter(self, node_name=node_name) for node_name in [
            "controller_server",
            "lidar_filter_front",
            "lidar_filter_rear"
        ]]

        self.counters += [NodeExecutionTimeCounter(self, node_name=node_name) for node_name in [
            "lidar_filter_front",
            "lidar_filter_rear"
        ]]

        self.measurement_in_progress: bool = False

        out_file_dir = os.path.expanduser("~/asb_logs/test_data/")
        filename_stamp = datetime.now().strftime("%Y-%m-%d__%H-%M-%S")
        self.out_file_path = os.path.join(out_file_dir, f"nodes_cpu_average_{filename_stamp}.csv")
        if not os.path.exists(out_file_dir):
            os.makedirs(out_file_dir)

        self.timer = self.create_timer(0.01, self.timer_callback)

    def timer_callback(self):
        now = self.get_clock().now()
        if not self.measurement_in_progress:
            if all(map(lambda c: c.can_start(), self.counters)):
                self.measurement_in_progress = True
                for counter in self.counters:
                    counter.set_measurement_start_time(now + Duration(seconds=2))
                    counter.set_measurement_stop_time(now + Duration(seconds=13))
        else:
            if all(map(lambda c: c.is_done(), self.counters)):
                with open(self.out_file_path, 'w') as f:
                    f.writelines(", ".join(map(lambda c: c.get_result().csv_header, self.counters)) + "\n")
                    f.writelines(", ".join(map(lambda c: c.get_result().csv_data, self.counters)) + "\n")
                raise KeyboardInterrupt


def main(args=None):
    rclpy.init(args=args)
    node = PerformanceLogger()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass


if __name__ == '__main__':
    main()
