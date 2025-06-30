#! /usr/bin/python3

import os.path
from datetime import datetime
import ast
import pandas as pd

import rclpy
from rclpy.node import Node
from rclpy.time import Time, Duration
from rclpy.qos import QoSProfile, ReliabilityPolicy, DurabilityPolicy

from std_msgs.msg import Header
from asb_msgs.msg import SystemUsage, ProcessCpuUsage, ExecutionDurationStamped

from workspace_packages_share_logger import log_workspace_packages_share

BLUE = "\033[94m"


class PerformanceCounter:

    def __init__(self, node: Node):
        self._node: Node = node
        self._measurement_start_time = None
        self._measurement_stop_time = None

    def set_measurement_time(self, start_time: Time, stop_time: Time):
        if self._measurement_start_time is None:
            self._measurement_start_time = start_time
        else:
            raise ValueError("start_time already set")
        if self._measurement_stop_time is None:
            self._measurement_stop_time = stop_time
        else:
            raise ValueError("stop_time already set")

    def can_start(self) -> bool:
        pass

    def is_done(self) -> bool:
        pass

    def get_result(self) -> pd.DataFrame:
        pass


class NodeComputerSystemUsageCounter(PerformanceCounter):
    def __init__(self, node: Node, node_name: str):
        super().__init__(node=node)

        self._metric_type: str = "cpu_usage"
        self._node_name: str = node_name

        self._counter_display_str: str = f"{node_name} {self._metric_type}:"
        self._counter_display_str = f"{self._counter_display_str:<47}"

        self._precondition: bool = False
        self._done: bool = False

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
        self.timer = self._node.create_timer(0.01, self._timer_callback)

    def _heartbeat_callback(self, _):
        self._precondition = True

    def _system_usage_callback(self, msg: SystemUsage):
        if self._measurement_start_time is None:
            return

        node_proc_msg: ProcessCpuUsage | None = None
        proc_msg: ProcessCpuUsage
        for proc_msg in msg.processes:
            if self._node_name_from_msg(proc_msg) == self._node_name:
                node_proc_msg = proc_msg
                break
        del proc_msg

        msg_time = Time.from_msg(msg.stamp)
        if self._measurement_start_time <= msg_time <= self._measurement_stop_time:
            if self._first_proc_msg is None:
                self._first_proc_msg = node_proc_msg
                self._first_proc_stamp = msg_time

            self._last_proc_msg = node_proc_msg
            self._last_proc_stamp = msg_time

            self._node.get_logger().info(BLUE + f"{self._counter_display_str} cpu time total: {node_proc_msg.cpu_time_total:.3f}")

    def _timer_callback(self):
        now = self._node.get_clock().now()
        if self._measurement_stop_time is not None and now > self._measurement_stop_time and not self._done:
            if self._last_proc_msg is not None and self._last_proc_stamp is not None:
                stamp_diff = (self._last_proc_stamp - self._first_proc_stamp).nanoseconds / 1E9
                if stamp_diff > 0.0:
                    cpu_time_diff = self._last_proc_msg.cpu_time_total - self._first_proc_msg.cpu_time_total
                    self._average_cpu_usage = cpu_time_diff / stamp_diff

                    self._node.get_logger().info(
                        f"* {self._counter_display_str}\n"
                        f"    average_cpu_usage: {self._average_cpu_usage:.3f}\n"
                    )
                else:
                    self._node.get_logger().error(f"{self._counter_display_str}: cannot compute metric, only one measurement!")
            else:
                self._node.get_logger().error(f"{self._counter_display_str}: cannot compute metric, no measurements!")

            self._done = True

    @staticmethod
    def _node_name_from_msg(process_msg: ProcessCpuUsage) -> str:
        cmdline = ast.literal_eval(process_msg.cmdline)
        cmdline_node_names = list(filter(lambda s: '__node:=' in s, cmdline))
        if len(cmdline_node_names) >= 1:
            return cmdline_node_names[0].replace('__node:=', '')
        else:
            return process_msg.name

    def can_start(self):
        return self._precondition

    def is_done(self):
        return self._done

    def get_result(self):
        return pd.DataFrame({
            (self._metric_type, self._node_name, "average_cpu_usage"): [self._average_cpu_usage],
        })


class NodeExecutionTimeCounter(PerformanceCounter):
    def __init__(self, node: Node, alias: str, node_name: str, label: str):
        super().__init__(node=node)

        self._metric_type: str = "execution_duration"
        self._alias: str = alias
        self._node_name: str = node_name
        self._label: str = label
        self._subject_id: str = f"{self._alias}.{self._label}" if self._label else self._alias

        self._counter_display_str: str = f"{self._subject_id} {self._metric_type}:"
        self._counter_display_str = f"{self._counter_display_str:<47}"

        self._precondition: bool = False
        self._done: bool = False

        self._measurements: list[ExecutionDurationStamped] = list()

        self._average_execution_duration: float | None = None
        self._min_execution_duration: float | None = None
        self._max_execution_duration: float | None = None

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
        self._computer_system_usage_sub = self._node.create_subscription(ExecutionDurationStamped, f"{self._node_name}/benchmarking/execution_duration", self._execution_duration_callback, qos_reliable_transient_local_depth_10)
        self.timer = self._node.create_timer(0.01, self._timer_callback)

    def _heartbeat_callback(self, _):
        self._precondition = True

    def _execution_duration_callback(self, msg: ExecutionDurationStamped):
        if self._measurement_start_time is None:
            return

        if msg.label != self._label:
            return

        msg_time = Time.from_msg(msg.stamp)

        if self._measurement_start_time <= msg_time <= self._measurement_stop_time:
            self._measurements.append(msg)
            self._node.get_logger().info(BLUE + f"{self._counter_display_str} execution duration: {self._duration_to_sec(msg) * 1E3:.3f} ms", throttle_duration_sec=1.0)

    def _timer_callback(self):
        now = self._node.get_clock().now()
        if self._measurement_stop_time is not None and now > self._measurement_stop_time and not self._done:
            if len(self._measurements):
                self._average_execution_duration = sum(map(self._duration_to_sec, self._measurements)) / len(self._measurements)
                self._min_execution_duration = min(map(self._duration_to_sec, self._measurements))
                self._max_execution_duration = max(map(self._duration_to_sec, self._measurements))

                self._node.get_logger().info(
                    f"* {self._counter_display_str}\n"
                    f"    average: {self._average_execution_duration*1E3:.3f} ms\n"
                    f"    min: {self._min_execution_duration*1E3:.3f} ms\n"
                    f"    max: {self._max_execution_duration*1E3:.3f} ms\n"
                )

            else:
                self._node.get_logger().error(f"{self._counter_display_str}: cannot compute metric, no measurements!")

            self._done = True

    @staticmethod
    def _duration_to_sec(m: ExecutionDurationStamped):
        return Duration.from_msg(m.execution_duration).nanoseconds / 1E9

    def can_start(self):
        return self._precondition

    def is_done(self):
        return self._done

    def get_result(self):
        return pd.DataFrame({
            (self._metric_type, self._subject_id, "average_execution_duration"): [self._average_execution_duration],
            (self._metric_type, self._subject_id, "min_execution_duration"): [self._min_execution_duration],
            (self._metric_type, self._subject_id, "max_execution_duration"): [self._max_execution_duration],
        })


class PerformanceLogger(Node):
    def __init__(self):
        super().__init__('node_computer_system_usage_logger')

        self.start_delay: float = 2.0
        self.measurement_duration: float = 11

        self.counters: list[PerformanceCounter] = [NodeComputerSystemUsageCounter(self, node_name=node_name) for node_name in [
            "controller_server",
            "lidar_filter_front",
            "lidar_filter_rear"
        ]]

        self.counters += [NodeExecutionTimeCounter(self, alias=alias, node_name=node_name, label=label) for (alias, node_name, label) in [
            ("local_costmap", "/local_costmap/local_costmap", "updateBounds"),
            ("lidar_filter_front", "lidar_filter_front", ""),
            ("lidar_filter_rear", "lidar_filter_rear", ""),
        ]]

        self.measurement_in_progress: bool = False

        stamp = datetime.now().strftime("%Y-%m-%d__%H-%M-%S")
        out_file_dir = os.path.expanduser(f"~/asb_logs/performance_measurements/")
        self.out_file_path = os.path.join(out_file_dir, f"{stamp}_metrics.csv")
        if not os.path.exists(out_file_dir):
            os.makedirs(out_file_dir)

        workspace_install_dir = os.path.expanduser("~/w/agrosensebot_ws/install")
        workspace_share_copy_dir = os.path.expanduser(f"{out_file_dir}/{stamp}_workspace_share_copy/")
        log_workspace_packages_share(root_dir=workspace_install_dir, output_dir=workspace_share_copy_dir)

        self.timer = self.create_timer(0.01, self.timer_callback)

    def timer_callback(self):
        now = self.get_clock().now()
        if not self.measurement_in_progress:
            if all(map(lambda c: c.can_start(), self.counters)):
                self.measurement_in_progress = True
                start_time: Time = now + Duration(seconds=self.start_delay)
                stop_time: Time = start_time + Duration(seconds=self.measurement_duration)
                for counter in self.counters:
                    counter.set_measurement_time(start_time=start_time, stop_time=stop_time)
        else:
            if all(map(lambda c: c.is_done(), self.counters)):
                results = pd.concat(list(map(lambda c: c.get_result(), self.counters)), axis=1)
                results.to_csv(self.out_file_path, index=False)  # to read the data, use pd.read_csv(..., header=[0,1,2])
                self.get_logger().info("\n" + str(results))
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
